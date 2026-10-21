"""Identical spatial assignment for CCL packets and IPM/ROI bright outputs."""
import numpy as np
from scipy.optimize import linear_sum_assignment

def score_centers(centers,xy,truth,isotope=None):
    centers=np.asarray(centers,np.float32).reshape(-1,2);pred=np.zeros(len(truth),np.uint8)
    if len(centers):
        distance=np.sqrt(((xy[:,None,:]-centers[None,:,:])**2).sum(-1))
        rr,cc=linear_sum_assignment(np.where(distance<=3.2,distance,1e6));pred[rr[distance[rr,cc]<=3.2]]=1
    tp=int(((pred==1)&(truth==1)).sum());fp=int(((pred==1)&(truth==0)).sum());fn=int(((pred==0)&(truth==1)).sum());tn=int(((pred==0)&(truth==0)).sum())
    extra=len(centers)-int(pred.sum());exact=int(np.array_equal(pred,truth))
    isotope=np.zeros(len(truth),np.uint8) if isotope is None else isotope
    metrics=dict(tp=tp,fp=fp,fn=fn,tn=tn,extra_packets=extra,state_exact=exact,record_exact=int(exact and extra==0),frames=1,
                 isotope_sites=int(isotope.sum()),isotope_correct=int(((pred==truth)&(isotope>0)).sum()),
                 predicted_count=len(centers),true_count=int(truth.sum()),count_abs_error=abs(len(centers)-int(truth.sum())))
    return metrics,pred

def mapping_coverage(centers,xy):
    distance=np.sqrt(((xy[:,None,:]-np.asarray(centers,np.float32)[None,:,:])**2).sum(-1))
    rr,cc=linear_sum_assignment(np.where(distance<=3.2,distance,1e6))
    return int((distance[rr,cc]<=3.2).sum())
