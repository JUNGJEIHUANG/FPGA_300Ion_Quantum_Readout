"""Frozen evaluation primitives; coordinates enter scoring only."""
import cv2, numpy as np, torch
from scipy.optimize import linear_sum_assignment
from training_utils import summarize

def predictions(mask,xy):
    _,_,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8),connectivity=4)
    ids=np.flatnonzero((stats[:,cv2.CC_STAT_AREA]>=2)&(stats[:,cv2.CC_STAT_AREA]<=45));ids=ids[ids!=0]
    centers=cent[ids].astype(np.float32);pred=np.zeros(len(xy),np.uint8)
    if len(centers):
        distance=np.sqrt(((xy[:,None,:]-centers[None,:,:])**2).sum(-1))
        rr,cc=linear_sum_assignment(np.where(distance<=3.2,distance,1e6))
        pred[rr[distance[rr,cc]<=3.2]]=1
    return pred,centers,stats[ids,cv2.CC_STAT_AREA]

def detailed_summary(rows):
    groups=summarize(rows)
    for category,g in groups.items():
        rr=rows if category=='ALL' else [r for r in rows if r['category']==category]
        n=sum(r.get('isotope_sites',0) for r in rr);c=sum(r.get('isotope_correct',0) for r in rr)
        g.update(precision=g['tp']/(g['tp']+g['fp']) if g['tp']+g['fp'] else None,
                 recall=g['tp']/(g['tp']+g['fn']) if g['tp']+g['fn'] else None,
                 isotope_sites=n,isotope_correct=c,isotope_accuracy=c/n if n else None,
                 mean_count_abs_error=sum(r['count_abs_error'] for r in rr)/len(rr))
    return groups

def configure_torch():
    torch.set_num_threads(8);torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    torch.backends.cudnn.deterministic=True;torch.backends.cudnn.benchmark=False
