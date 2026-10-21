"""W8/A18 software-boundary validation on frozen 5000 validation frames only.

Run: python T_validate_quantization.py --variant dw
Not a bit-accurate HLS/RTL or board result.
"""
import argparse,json,sys,time
from pathlib import Path
import numpy as np,torch

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT
sys.path.insert(0,str(ROOT/'support'))
sys.path.insert(0,str(ROOT/'source'))
from training_utils import FrozenData,packet_metrics,summarize,sha,atomic_json
from evaluation_utils import configure_torch
from T_vit_depthwise import build
from T_quant_vit_depthwise import QuantViTSeg


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=['dw','dw_dilated'],required=True);args=ap.parse_args()
    out=ROOT/'reports'/f'T_{args.variant}_w8a18_validation.json'
    if out.exists():raise RuntimeError(f'Refuse overwrite: {out}')
    run=ROOT/'runs'/args.variant
    status=json.loads((run/'status.json').read_text())
    assert status['status']=='complete'
    checkpoint_path=run/'best.pth';assert sha(checkpoint_path)==status['best_sha256']
    rec=json.loads((ROOT/'reports'/f'T_{args.variant}_validation_thresholds.json').read_text())
    assert rec['checkpoint_sha256']==sha(checkpoint_path)
    threshold=rec['selected_threshold']
    configure_torch()
    data=FrozenData(torch.device('cuda:0'))
    checkpoint=torch.load(checkpoint_path,map_location='cpu',weights_only=True)
    model=build(args.variant).cuda().eval();model.load_state_dict(checkpoint['model_state_dict'])
    identity=QuantViTSeg(model,enabled=False).cuda().eval()
    quant=QuantViTSeg(model,enabled=True).cuda().eval()
    ids=data.val[:4];x,*_=data.batch(ids)
    original_fastpath=torch.backends.mha.get_fastpath_enabled()
    with torch.inference_mode():
        torch.backends.mha.set_fastpath_enabled(False)
        reference=model(x)
        identity_error=float((reference-identity(x)).abs().max())
        torch.backends.mha.set_fastpath_enabled(original_fastpath)
    if identity_error>.001:raise RuntimeError(f'Unquantized explicit interpreter mismatch {identity_error}')
    quant._tracking=True
    rows={'FP32':[],'W8A18_boundary':[]};disagreement=0;mae=0.;maximum=0.;pixels=0;t0=time.perf_counter()
    with torch.inference_mode():
        for off in range(0,len(data.val),4):
            inds=data.val[off:off+4];x,*_=data.batch(inds)
            a=model(x).squeeze(1);b=quant(x).squeeze(1)
            delta=(a-b).abs();mae+=float(delta.sum(dtype=torch.float64));maximum=max(maximum,float(delta.max()));pixels+=a.numel()
            am=(a.sigmoid()>threshold).cpu().numpy();bm=(b.sigmoid()>threshold).cpu().numpy()
            disagreement+=int((am!=bm).sum())
            for idx,ma,mb in zip(inds,am,bm):
                for tag,mask in [('FP32',ma),('W8A18_boundary',mb)]:
                    metric=packet_metrics(mask,data.coords[idx],data.states[idx])
                    metric.update(cache_row=int(idx),category=str(data.categories[idx]));rows[tag].append(metric)
            if off%500==0:print('QUANT_VALIDATION',args.variant,off,flush=True)
    report=dict(status='complete',variant=args.variant,checkpoint_sha256=sha(checkpoint_path),frames=len(data.val),
                threshold=threshold,fp32= summarize(rows['FP32']),w8a18_boundary=summarize(rows['W8A18_boundary']),
                unquantized_interpreter_sample_max_abs=identity_error,logit_mae=mae/pixels,logit_max_abs=maximum,
                mask_disagreement=disagreement/pixels,activation_clipping=quant.counters,
                seconds=time.perf_counter()-t0,test_accessed=False,
                scope='Software W8/A18 boundary simulation; FP32 LayerNorm/Softmax/GELU internals. Not bit-accurate HLS, RTL, board or clock result.')
    atomic_json(report,out)
    print('QUANT_VALIDATION_COMPLETE',args.variant,json.dumps({'fp32_all':report['fp32']['ALL'],'quant_all':report['w8a18_boundary']['ALL']}),flush=True)

if __name__=='__main__':main()
