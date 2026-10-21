"""Exact-value train/val GPU cache and removal of unused detached loss logging."""
import ast,inspect,time
import numpy as np,torch
import training_utils as common
BASE_DATA=common.FrozenData
BASE_OBJECTIVE=common.objective

tree=ast.parse(inspect.getsource(BASE_OBJECTIVE));function=tree.body[0]
assert isinstance(function.body[-1],ast.Return)
assert isinstance(function.body[-1].value,ast.Tuple)
assert isinstance(function.body[-1].value.elts[0],ast.Name) and function.body[-1].value.elts[0].id=='loss'
unchanged_loss_ast=ast.dump(ast.Module(body=function.body[:-1],type_ignores=[]))
function.body[-1].value=ast.Tuple(elts=[ast.Name(id='loss',ctx=ast.Load()),ast.Dict(keys=[],values=[])],ctx=ast.Load())
ast.fix_missing_locations(tree)
namespace=dict(vars(common));exec(compile(tree,'objective_without_unused_detached_statistics','exec'),namespace)
FAST_OBJECTIVE=namespace['objective']

class CachedData(BASE_DATA):
    def __init__(self,device):
        super().__init__(device)
        if device.type!='cuda':raise RuntimeError('Acceleration requires CUDA')
        n=len(self.images);free,total=torch.cuda.mem_get_info(device)
        needed=n*(88*456*2+300+300*5*2*4+300*9*2*4+1)
        if free<needed+1536*2**20:raise RuntimeError(f'Insufficient free VRAM for cache and1.5GiB model/workspace reserve: free={free}, cache={needed}')
        start=time.perf_counter()
        self._images=torch.empty((n,88,456),dtype=torch.uint8,device=device)
        self._targets=torch.empty_like(self._images)
        self._states=torch.empty((n,300),dtype=torch.uint8,device=device)
        self._cross=torch.empty((n,300,5,2),dtype=torch.float32,device=device)
        self._peak=torch.empty((n,300,9,2),dtype=torch.float32,device=device)
        self._mixed=torch.empty((n,),dtype=torch.bool,device=device)
        for off in range(0,n,128):
            ids=np.arange(off,min(off+128,n));_,y,st,cross,peak,mixed=BASE_DATA.batch(self,ids)
            self._images[off:off+len(ids)].copy_(torch.from_numpy(np.asarray(self.images[ids]).copy()).to(device))
            self._targets[off:off+len(ids)].copy_(y.squeeze(1).to(torch.uint8))
            self._states[off:off+len(ids)].copy_(st.to(torch.uint8))
            self._cross[off:off+len(ids)].copy_(cross);self._peak[off:off+len(ids)].copy_(peak);self._mixed[off:off+len(ids)].copy_(mixed)
        torch.cuda.synchronize()
        self.cache_receipt=dict(cache_bytes=needed,cache_gib=needed/2**30,cache_seconds=time.perf_counter()-start,
                                source='Frozen25000 train/validation cache only; no benchmark images',input_storage='uint8',
                                target_storage='uint8 exact binary masks',grid_storage='FP32, computed by original batch method')
        print('READ_ONLY_GPU_CACHE_READY',self.cache_receipt,flush=True)
    def batch(self,indices):
        ids=torch.as_tensor(np.asarray(indices),device=self.device,dtype=torch.long)
        return (self._images[ids].float().unsqueeze(1)/255.,self._targets[ids].float().unsqueeze(1),
                self._states[ids].float(),self._cross[ids],self._peak[ids],self._mixed[ids])
