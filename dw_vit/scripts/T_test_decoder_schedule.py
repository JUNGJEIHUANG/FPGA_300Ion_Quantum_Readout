"""Unit-check parity-scheduled three-row dilated decoder against PyTorch."""
import numpy as np,torch
from torch.nn import functional as F
def scheduled(source,weight,dilation):
    h,w,c=source.shape;ring=np.zeros((3,w,c),np.float64);out=np.zeros((h,w,c),np.float64)
    def row(y):ring[(y//dilation)%3]=source[y]
    for parity in range(dilation):
        row(parity)
        for y in range(parity,h,dilation):
            if y+dilation<h:row(y+dilation)
            for x in range(w):
                for ch in range(c):
                    total=0.
                    for ky in range(3):
                        for kx in range(3):
                            yy=y+(ky-1)*dilation;xx=x+(kx-1)*dilation
                            if 0<=yy<h and 0<=xx<w:
                                total+=ring[(yy//dilation)%3,xx,ch]*weight[ch,ky,kx]
                    out[y,x,ch]=total
    return out
def main():
    rng=np.random.default_rng(20261021)
    for dilation in (1,2):
        source=rng.normal(size=(8,10,4));weight=rng.normal(size=(4,3,3))
        a=scheduled(source,weight,dilation)
        b=F.conv2d(torch.from_numpy(source).permute(2,0,1).unsqueeze(0),
                   torch.from_numpy(weight).unsqueeze(1),groups=4,
                   padding=dilation,dilation=dilation).squeeze(0).permute(1,2,0).numpy()
        error=float(np.max(np.abs(a-b)))
        if error>1e-12:raise RuntimeError(f'dilation={dilation} error={error}')
        print('DECODER_SCHEDULE_PASS',dilation,error,flush=True)
if __name__=='__main__':main()
