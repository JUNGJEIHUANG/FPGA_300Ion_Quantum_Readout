"""Export frozen trained decoder variants as W8/A18 HLS ROMs.

Run: python T_export_hls_parameters.py --variant dw
Requires a completed training run and frozen validation-only threshold.
Does not access the benchmark or program hardware.
"""
import argparse, hashlib, json, math, re, sys
from pathlib import Path
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array(name,ctype,values,hexadecimal=False):
    v=list(values);lines=[f"static const {ctype} {name}[{len(v)}] = {{"]
    for i in range(0,len(v),16):
        lines.append(','.join(f"0x{int(x):08x}u" if hexadecimal else str(int(x)) for x in v[i:i+16])+',')
    return '\n'.join(lines+['};'])


def quant(values,bits):
    arr=values.detach().cpu().numpy().astype(np.float64)
    if bits==8:
        mx=float(np.max(np.abs(arr)))
        step=2.**math.ceil(math.log2(mx/127.)) if mx else 2.**-8
        lo,hi=-128,127
    elif bits==18:step=2.**-8;lo,hi=-(2**17),2**17-1
    else:step=2.**-16;lo,hi=-(2**31),2**31-1
    q=np.rint(arr/step).clip(lo,hi).astype(np.int64)
    return q,step


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=['dw','dw_dilated'],required=True);args=ap.parse_args()
    run=ROOT/'runs'/args.variant
    status=json.loads((run/'status.json').read_text())
    assert status['status']=='complete'
    checkpoint_path=run/'best.pth'
    assert sha(checkpoint_path)==status['best_sha256']
    threshold_file=ROOT/'reports'/f'T_{args.variant}_validation_thresholds.json'
    threshold_receipt=json.loads(threshold_file.read_text())
    assert threshold_receipt['status']=='frozen' and threshold_receipt['checkpoint_sha256']==sha(checkpoint_path)
    t=threshold_receipt['selected_threshold']
    raw_threshold=math.floor(math.log(t/(1-t))*256)
    checkpoint=torch.load(checkpoint_path,map_location='cpu',weights_only=True)
    state=checkpoint['model_state_dict']
    assert sum(v.numel() for v in state.values())==376321
    order=[('PATCH','patch_embed')]
    for layer in range(2):
        for symbol,suffix in [('QKV','self_attn.in_proj'),('OUT','self_attn.out_proj'),('FF1','linear1'),('FF2','linear2')]:
            order.append((f'L{layer}_{symbol}',f'encoder.layers.{layer}.{suffix}'))
    order += [('DEC0','decoder.0'),('DEC_DW','decoder.2.0'),('DEC_PW','decoder.2.1'),('DEC4','decoder.4')]
    weight_words=[];bias=[];specs=[];records=[]
    for symbol,base in order:
        wn=base+'_weight' if base.endswith('in_proj') else base+'.weight'
        bn=base+'_bias' if base.endswith('in_proj') else base+'.bias'
        w,step=quant(state[wn],8);b,_=quant(state[bn],32)
        if symbol=='DEC0':w=w.transpose(2,3,1,0).reshape(4096,64)
        elif symbol=='DEC_DW':
            w=w.reshape(64,9)
            w=np.pad(w,((0,0),(0,7)),constant_values=0)
        elif symbol=='DEC_PW':w=w.reshape(32,64)
        else:w=w.reshape(w.shape[0],-1)
        assert w.shape[1]%16==0 and len(weight_words)%16==0
        exponent=int(round(math.log2(step)))
        assert -24<=exponent<=8
        specs.append(dict(symbol=symbol,rows=w.shape[0],blocks=w.shape[1]//16,
                          word_offset=len(weight_words)//16,bias_offset=len(bias),exponent=exponent))
        weight_words.extend(w.reshape(-1).tolist());bias.extend(b.reshape(-1).tolist())
        records.append(dict(symbol=symbol,weight_name=wn,bias_name=bn,shape=list(state[wn].shape),
                            padded_shape=list(w.shape),step=step,exponent=exponent))
    assert len(specs)==13
    weights=np.asarray(weight_words,np.int8).reshape(-1,16)
    banks=[]
    for bank in range(4):
        words=np.zeros(len(weights),np.uint32)
        for lane in range(4):
            words|=weights[:,bank*4+lane].astype(np.uint8).astype(np.uint32)<<(8*lane)
        banks.append(words)
    restored=np.stack([((banks[k//4]>>(8*(k%4)))&255).astype(np.uint8) for k in range(16)],axis=1).view(np.int8)
    assert np.array_equal(weights,restored)
    pos,_=quant(state['pos_embed'],18);position=pos.reshape(-1).tolist()
    assert len(position)==40128
    norm=[];offset={}
    norm_names=[]
    for layer in range(2):
        for num in (1,2):
            for suffix,symbol in [('weight','GAMMA'),('bias','BETA')]:
                norm_names.append((f'encoder.layers.{layer}.norm{num}.{suffix}',f'L{layer}_N{num}_{symbol}'))
    norm_names += [('encoder.norm.weight','FINAL_GAMMA'),('encoder.norm.bias','FINAL_BETA')]
    for name,symbol in norm_names:
        vals,_=quant(state[name],18);offset[symbol]=40128+len(norm)
        norm.extend(vals.reshape(-1).tolist())
    assert len(norm)==640
    gelu_path=ROOT/'parameters/gelu_q10_8_positive_2048.hex'
    gelu=np.asarray([int(line,16) for line in gelu_path.read_text().split()],np.int32)
    assert len(gelu)==2048
    lines=['#pragma once',f'// Generated frozen checkpoint SHA256 {sha(checkpoint_path)}.',
           f'static const int MODEL_THRESHOLD_RAW = {raw_threshold};',
           'enum MatId {'+','.join(s['symbol'] for s in specs)+'};',
           'struct MatSpec { int rows, blocks, word_offset, bias_offset, exponent; };',
           'static const MatSpec MATS[13] = {']
    for spec in specs:lines.append('{'+','.join(str(spec[k]) for k in ('rows','blocks','word_offset','bias_offset','exponent'))+'},')
    lines+=['};','#ifdef __SYNTHESIS__']
    for i,bank in enumerate(banks):lines.append(array(f'W_BANK{i}','unsigned int',bank,True))
    lines+=['#else',array('W_NATIVE','signed char',weight_words),'#endif',
            array('POSITION18','int',position),array('NORM18','int',norm),
            array('BIAS','int',bias),array('GELU_POS','unsigned short',gelu)]
    for symbol,at in offset.items():lines.append(f'static const int {symbol} = {at};')
    lines.append('static const int POS_OFFSET = 0;')
    dest=ROOT/'01_hls'/f'src_{args.variant}'/'vit_parameters.h'
    if dest.exists():raise RuntimeError(f'Refuse overwrite: {dest}')
    dest.write_text('\n'.join(lines)+'\n',encoding='ascii')
    param_dir=ROOT/'parameters'/args.variant;param_dir.mkdir(parents=True,exist_ok=True)
    for i,bank in enumerate(banks):bank.astype('<u4').tofile(param_dir/f'hls_wbank{i}.bin')
    report=dict(status='exported',variant=args.variant,checkpoint_sha256=sha(checkpoint_path),
                threshold=t,threshold_raw=raw_threshold,weight_count_unpadded=334432,
                weight_count_padded=len(weight_words),words_per_bank=len(weights),
                all_weights_inverse_verified=True,matrices=specs,matrix_records=records,
                parameter_header_sha256=sha(dest),bias_count=len(bias),position_count=len(position),norm_count=len(norm),
                numerical_format='W8 per-tensor power-of-two; positions/norm A18 Q10.8; bias Q16.16; core activation A18',
                scope='Parameter export only, not numerical or FPGA validation')
    assert report['weight_count_padded']==334880
    path=ROOT/'reports'/f'T_{args.variant}_hls_export.json'
    if path.exists():raise RuntimeError(f'Refuse overwrite: {path}')
    path.write_text(json.dumps(report,indent=2))
    print('HLS_EXPORT_COMPLETE',args.variant,len(weights),sha(dest),flush=True)

if __name__=='__main__':main()
