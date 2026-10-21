from pathlib import Path
import csv,hashlib,importlib.util,json,warnings,os
import cv2,numpy as np,torch
from torch.nn import functional as F
from scipy.optimize import linear_sum_assignment

ROOT=Path(__file__).resolve().parent.parent
PARENT=Path(os.environ.get('DWVIT_TRAIN_DATA', str(ROOT/'local_data/train_validation')))
MODEL_SOURCE=ROOT.parent/'nets/DWViT.py'
CFG=dict(image_size=(88,456),patch_size=8,embed_dim=64,depth=2,num_heads=2,mlp_ratio=2,dropout=0.1)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def make_model():
    spec=importlib.util.spec_from_file_location('continued_vit',MODEL_SOURCE);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    with warnings.catch_warnings():warnings.simplefilter('ignore');net=mod.DWViT(dilation=2)
    assert sum(p.numel() for p in net.parameters())==376321
    return net
def atomic_save(obj,path):
    path=Path(path);temp=path.with_suffix(path.suffix+'.tmp');torch.save(obj,temp);temp.replace(path)
def atomic_json(obj,path):
    path=Path(path);temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(obj,indent=2),encoding='utf-8');temp.replace(path)

class FrozenData:
    def __init__(self,device):
        cache=PARENT/'cache';self.device=device
        receipt=json.loads((PARENT/'reports/training_data_receipt.json').read_text(encoding='utf-8'))
        if sha(cache/'split_manifest.csv')!=receipt['manifest_sha256']:raise ValueError('Frozen manifest changed')
        self.receipt=receipt
        self.images=np.load(cache/'images.npy',mmap_mode='r');self.coords=np.load(cache/'xy.npy',mmap_mode='r');self.states=np.load(cache/'states.npy',mmap_mode='r')
        self.train=np.load(cache/'training_indices.npy');self.val=np.load(cache/'validation_indices.npy')
        rows=list(csv.DictReader((cache/'split_manifest.csv').open(encoding='utf-8-sig')))
        self.categories=np.asarray([r['category'] for r in rows])
        if len(self.train)!=20000 or len(self.val)!=5000 or np.intersect1d(self.train,self.val).size:raise ValueError('Unexpected frozen membership')
        self.offsets=torch.cartesian_prod(torch.arange(-2,3,device=device),torch.arange(-2,3,device=device)).long()
        self.cross=torch.tensor([[0.,0.],[1.,0.],[-1.,0.],[0.,1.],[0.,-1.]],device=device)
        self.peak=torch.cartesian_prod(torch.arange(-1,2,device=device),torch.arange(-1,2,device=device)).float()
    def batch(self,indices):
        device=self.device
        x=torch.from_numpy(np.asarray(self.images[indices]).copy()).pin_memory().to(device,non_blocking=True).float().unsqueeze(1)/255.
        xy=torch.from_numpy(np.asarray(self.coords[indices]).copy()).to(device)
        st=torch.from_numpy(np.asarray(self.states[indices]).copy()).to(device).float()
        pix=torch.floor(xy).long().unsqueeze(2)+self.offsets.view(1,1,25,2)
        valid=((pix.float()-xy.unsqueeze(2)).square().sum(-1)<=2.25)&(st.unsqueeze(-1)>0)&(pix[...,0]>=0)&(pix[...,0]<456)&(pix[...,1]>=0)&(pix[...,1]<88)
        flat=torch.arange(len(indices),device=device).view(-1,1,1)*(88*456)+pix[...,1]*456+pix[...,0]
        target=torch.zeros(len(indices)*88*456,device=device);target.scatter_(0,flat[valid],1.)
        def grid(offset):
            g=xy.unsqueeze(2)+offset.view(1,1,-1,2);g[...,0]=g[...,0]/455*2-1;g[...,1]=g[...,1]/87*2-1;return g
        mixed=torch.tensor(self.categories[indices]=='150D_150B',device=device)
        return x,target.view(-1,1,88,456),st,grid(self.cross),grid(self.peak),mixed

def objective(logits,target,states,cross_grid,peak_grid,mixed):
    # Retain the original pilot's full dense objective. Add only a mild negative-site term.
    dense=F.binary_cross_entropy_with_logits(logits,target,pos_weight=torch.tensor(4.,device=logits.device),reduction='none').mean((1,2,3))
    prob=logits.sigmoid()
    dice=1-(2*(prob*target).sum((1,2,3))+1)/(prob.sum((1,2,3))+target.sum((1,2,3))+1)
    cross=F.grid_sample(prob,cross_grid,align_corners=True).squeeze(1).mean(-1)
    peaks=F.grid_sample(logits,peak_grid,align_corners=True).squeeze(1).amax(-1)
    site=F.binary_cross_entropy(cross.clamp(1e-6,1.-1e-6),states,reduction='none').mean(1)
    dark=(states==0).float()
    dark_peak=(F.softplus(peaks)*dark).sum(1)/dark.sum(1).clamp(min=1.)
    per_frame=dense+dice+.25*site+.10*dark_peak
    frame_weight=torch.where(mixed,1.5,1.)
    loss=(per_frame*frame_weight).sum()/frame_weight.sum()
    return loss,dict(dense=float(dense.detach().mean()),dice=float(dice.detach().mean()),site=float(site.detach().mean()),dark_peak=float(dark_peak.detach().mean()))

def packet_metrics(mask,xy,truth):
    n,_,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8),connectivity=4)
    ids=np.flatnonzero((stats[:,cv2.CC_STAT_AREA]>=2)&(stats[:,cv2.CC_STAT_AREA]<=45));ids=ids[ids!=0]
    centers=cent[ids].astype(np.float32);pred=np.zeros(len(truth),np.uint8)
    if len(centers):
        distance=np.sqrt(((xy[:,None,:]-centers[None,:,:])**2).sum(-1))
        rr,cc=linear_sum_assignment(np.where(distance<=3.2,distance,1e6));rr=rr[distance[rr,cc]<=3.2];pred[rr]=1
    tp=int(((pred==1)&(truth==1)).sum());fp=int(((pred==1)&(truth==0)).sum());fn=int(((pred==0)&(truth==1)).sum());tn=int(((pred==0)&(truth==0)).sum())
    extra=len(centers)-int(pred.sum())
    return dict(tp=tp,fp=fp,fn=fn,tn=tn,extra_packets=extra,state_exact=int(np.array_equal(pred,truth)),record_exact=int(np.array_equal(pred,truth) and extra==0),frames=1)

def summarize(rows):
    out={}
    for group in ['ALL']+sorted({r['category'] for r in rows}):
        rr=rows if group=='ALL' else [r for r in rows if r['category']==group]
        sums={k:sum(r[k] for r in rr) for k in ['tp','fp','fn','tn','extra_packets','state_exact','record_exact','frames']}
        tp,fp,fn,tn=(sums[k] for k in ['tp','fp','fn','tn'])
        out[group]={**sums,'site_accuracy':(tp+tn)/max(1,tp+fp+fn+tn),
                    'false_bright_rate':fp/max(1,fp+tn),'false_dark_rate':fn/max(1,fn+tp),
                    'state_exact_rate':sums['state_exact']/len(rr),'record_exact_rate':sums['record_exact']/len(rr),
                    'packet_errors':fp+fn+sums['extra_packets']}
    return out

def validate(model,data,batch_size=32,threshold=.5,save_path=None):
    model.eval();rows=[];total_loss=0.
    with torch.inference_mode():
        for start in range(0,len(data.val),batch_size):
            inds=data.val[start:start+batch_size];x,y,st,cross,peak,mixed=data.batch(inds)
            logits=model(x);loss,_=objective(logits,y,st,cross,peak,mixed);total_loss+=float(loss)*len(inds)
            masks=(logits.sigmoid().squeeze(1)>threshold).cpu().numpy().astype(np.uint8)
            for idx,mask in zip(inds,masks):
                row=packet_metrics(mask,data.coords[idx],data.states[idx]);row.update(cache_row=int(idx),category=str(data.categories[idx]));rows.append(row)
    out=dict(objective=total_loss/len(data.val),threshold=threshold,groups=summarize(rows))
    if save_path:atomic_json(dict(summary=out,per_frame=rows),save_path)
    return out

def selection_key(validation):
    mixed=validation['groups']['150D_150B'];all_=validation['groups']['ALL']
    return (mixed['packet_errors'],-mixed['state_exact'],mixed['fp'],all_['packet_errors'],validation['objective'])
