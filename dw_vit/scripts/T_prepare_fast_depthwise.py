"""Create an isolated channel-parallel HLS implementation of the same trained model.

No network weights, activation format, threshold or CCL behavior are changed.
The optimization replaces the generic serial 16-lane dot call for a 9-tap
depthwise kernel with 16-channel banked window accumulation.
"""
import hashlib,json,math
from pathlib import Path
import numpy as np,torch
ROOT=Path(__file__).resolve().parents[1]
VARIANT='dw_dilated';FAST='dw_dilated_fast'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def once(s,a,b):
    if s.count(a)!=1:raise RuntimeError(f'Expected one anchor; found {s.count(a)} for {a[:50]}')
    return s.replace(a,b)
def main():
    src=ROOT/'01_hls'/f'src_{VARIANT}'
    dst=ROOT/'01_hls'/f'src_{FAST}'
    if dst.exists():raise RuntimeError('Refuse overwrite fast candidate')
    dst.mkdir()
    export=json.loads((ROOT/'reports'/f'T_{VARIANT}_hls_export.json').read_text())
    checkpoint=torch.load(ROOT/'runs'/VARIANT/'best.pth',map_location='cpu',weights_only=True)
    state=checkpoint['model_state_dict']
    assert export['checkpoint_sha256']==sha(ROOT/'runs'/VARIANT/'best.pth')
    weight=state['decoder.2.0.weight'].numpy().astype(np.float64).reshape(64,9)
    exponent=next(x['exponent'] for x in export['matrices'] if x['symbol']=='DEC_DW')
    q=np.rint(weight/(2.**exponent)).clip(-128,127).astype(np.int8)
    bias=np.rint(state['decoder.2.0.bias'].numpy().astype(np.float64)*65536.).clip(-(2**31),2**31-1).astype(np.int64)
    header=(src/'vit_parameters.h').read_text()
    table=['// Same W8 depthwise values as the generic DEC_DW matrix, tap-major for channel-parallel access.',
           'static const signed char DW_TAP_W[9][64] = {']
    for tap in range(9):table.append('{'+','.join(str(int(q[c,tap])) for c in range(64))+'},')
    table+=['};','static const int DW_BIAS[64] = {'+','.join(str(int(v)) for v in bias)+'};',
            f'static const int DW_EXPONENT = {exponent};']
    header=once(header,'#ifdef __SYNTHESIS__','\n'.join(table)+'\n#ifdef __SYNTHESIS__')
    (dst/'vit_parameters.h').write_text(header,encoding='ascii')
    (dst/'vitseg_core.h').write_bytes((src/'vitseg_core.h').read_bytes())
    cpp=(src/'vitseg_core.cpp').read_text()
    insertion='''static void depthwise_pixel(const act_t shared[N*192],int y,int x,act_t output[64]) {
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable=DW_TAP_W cyclic factor=16 dim=2
    acc_t accum[64];
#pragma HLS ARRAY_PARTITION variable=accum cyclic factor=16
    for(int c=0;c<64;c++)accum[c]=0;
    for(int ky=0;ky<3;ky++)for(int kx=0;kx<3;kx++) {
        int yy=y+(ky-1)*DEC_DILATION,xx=x+(kx-1)*DEC_DILATION;
        bool valid=(yy>=0&&yy<H&&xx>=0&&xx<W);
        for(int block=0;block<4;block++) {
#pragma HLS PIPELINE II=1
            for(int lane=0;lane<16;lane++) {
#pragma HLS UNROLL
                int c=block*16+lane;
                act_t pixel=valid?shared[((yy/DEC_DILATION)%3)*W*64+xx*64+c]:(act_t)0;
                accum[c]+=(acc_t)pixel*(int)DW_TAP_W[ky*3+kx][c];
            }
        }
    }
    for(int c=0;c<64;c++)output[c]=requant(accum[c],DW_BIAS[c],DW_EXPONENT);
}
'''
    cpp=once(cpp,'static void components(bool foreground[H*W]',insertion+'static void components(bool foreground[H*W]')
    old='''                for(int c=0;c<64;c++) {
                    for(int ky=0;ky<3;ky++)for(int kx=0;kx<3;kx++) {
                        int yy=y+(ky-1)*DEC_DILATION,xx=x+(kx-1)*DEC_DILATION;
                        work[ky*3+kx]=(yy>=0&&yy<H&&xx>=0&&xx<W)?
                            shared[((yy/DEC_DILATION)%3)*W*64+xx*64+c]:(act_t)0;
                    }
                    for(int k=9;k<16;k++)work[k]=0;
                    dw[c]=dot_w8(work,DEC_DW,c);
                }'''
    cpp=once(cpp,old,'                depthwise_pixel(shared,y,x,dw);')
    (dst/'vitseg_core.cpp').write_text(cpp,encoding='utf-8')
    # The generic DEC_DW row still exists in packed ROM for now, so this
    # generation step changes scheduling, not checkpoint/quantized values.
    report=dict(status='prepared',source_variant=VARIANT,fast_variant=FAST,
                checkpoint_sha256=export['checkpoint_sha256'],threshold_raw=export['threshold_raw'],
                source_header_sha256=sha(src/'vit_parameters.h'),fast_header_sha256=sha(dst/'vit_parameters.h'),
                source_cpp_sha256=sha(src/'vitseg_core.cpp'),fast_cpp_sha256=sha(dst/'vitseg_core.cpp'),
                weight_table_same_quantized_values=True,depthwise_exponent=exponent,
                scope='HLS scheduling experiment only; C-model bitexact and synthesis/board gates pending')
    (ROOT/'reports/T_fast_depthwise_source.json').write_text(json.dumps(report,indent=2))
    # Reuse the exact export metadata except for this HLS-only header identity.
    fast_export=dict(export);fast_export['variant']=FAST;fast_export['parameter_header_sha256']=report['fast_header_sha256']
    fast_export['scope']='Same W8/A18 checkpoint/weights; extra tap-major depthwise read view for HLS parallelism'
    (ROOT/'reports'/f'T_{FAST}_hls_export.json').write_text(json.dumps(fast_export,indent=2))
    tb=ROOT/'01_hls'/f'tb_{FAST}';tb.mkdir()
    original_tb=(ROOT/'01_hls'/f'tb_{VARIANT}'/'T_tb_vitseg.cpp').read_text()
    (tb/'T_tb_vitseg.cpp').write_text(original_tb.replace(f'../src_{VARIANT}/vitseg_core.h',f'../src_{FAST}/vitseg_core.h'))
    params=ROOT/'parameters'/FAST;params.mkdir(parents=True)
    for b in range(4):(params/f'hls_wbank{b}.bin').write_bytes((ROOT/'parameters'/VARIANT/f'hls_wbank{b}.bin').read_bytes())
    print('FAST_HLS_SOURCE_PREPARED',FAST,report['fast_cpp_sha256'],flush=True)
if __name__=='__main__':main()
