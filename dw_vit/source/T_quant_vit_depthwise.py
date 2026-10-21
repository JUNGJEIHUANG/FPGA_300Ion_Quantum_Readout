"""W8/A18 software-boundary diagnostic. NOT a bit-accurate HLS/RTL simulator.

Matrix weights: signed W8, per-tensor power-of-two steps from trained weights.
Stored activations/position/LayerNorm coefficients: signed 18-bit Q10.8, AP_RND-like
ties toward +infinity and saturation. Bias: signed 32-bit Q16.16. Attention
probabilities use unsigned 18-bit Q1.17, not coarse Q10.8.
Accumulators, LayerNorm, Softmax and GELU are still computed in FP32 here.
"""
import math
import torch
from torch import nn
from torch.nn import functional as F

class QuantViTSeg(nn.Module):
    def __init__(self,model,enabled=True):
        super().__init__();self.cfg=dict(embed_dim=64,depth=2,num_heads=2,mlp_ratio=2);self.enabled=enabled
        self.dilation=model.decoder[2][0].dilation[0]
        assert self.dilation in (1,2)
        self.scales={};self.counters={};self._names={};self._tracking=False
        for i,(name,param) in enumerate(model.named_parameters()):
            value=param.detach().clone()
            if value.ndim>=2 and name!='pos_embed':
                mx=float(value.abs().max());step=2.**math.ceil(math.log2(mx/127.)) if mx else 2.**-8
                q=torch.clamp(torch.floor(value/step+.5),-128,127)*step
                self.scales[name]=dict(bits=8,step=step,format='signed int8 * power_of_two_step')
            elif name.endswith('bias') and 'norm' not in name:
                step=2.**-16;q=torch.clamp(torch.floor(value/step+.5),-(2**31),2**31-1)*step
                self.scales[name]=dict(bits=32,step=step,format='signed Q16.16')
            else:
                step=2.**-8;q=torch.clamp(torch.floor(value/step+.5),-(2**17),2**17-1)*step
                self.scales[name]=dict(bits=18,step=step,format='signed Q10.8')
            key=f'p{i}';self.register_buffer(key,q if enabled else value);self._names[name]=key
    def p(self,name):return getattr(self,self._names[name])
    def q(self,x,name,prob=False):
        if not self.enabled:return x
        step=2.**(-17 if prob else -8)
        lo,hi=(0,2**18-1) if prob else (-(2**17),2**17-1)
        raw=torch.floor(x/step+.5)
        if self._tracking:
            c=self.counters.setdefault(name,dict(clipped=0,elements=0))
            c['clipped']+=int(((raw<lo)|(raw>hi)).sum());c['elements']+=raw.numel()
        return torch.clamp(raw,lo,hi)*step
    def linear(self,x,base):return self.q(F.linear(x,self.p(base+'.weight'),self.p(base+'.bias')),base)
    def norm(self,x,base):
        return self.q(F.layer_norm(x,(64,),self.p(base+'.weight'),self.p(base+'.bias'),1e-5),base)
    def forward(self,x):
        x=self.q(x,'input')
        x=self.q(F.conv2d(x,self.p('patch_embed.weight'),self.p('patch_embed.bias'),stride=8),'patch_embed')
        x=x.flatten(2).transpose(1,2)
        x=self.q(x+self.p('pos_embed'),'add_pos')
        for layer in range(2):
            base=f'encoder.layers.{layer}'
            z=self.norm(x,base+'.norm1')
            qkv=self.q(F.linear(z,self.p(base+'.self_attn.in_proj_weight'),self.p(base+'.self_attn.in_proj_bias')),base+'.qkv')
            q,k,v=qkv.chunk(3,-1);B,N,D=q.shape
            q=q.view(B,N,2,32).transpose(1,2);k=k.view(B,N,2,32).transpose(1,2);v=v.view(B,N,2,32).transpose(1,2)
            scores=(q@k.transpose(-2,-1))*(32.**-.5)
            prob=self.q(scores.softmax(-1),base+'.softmax_probability',prob=True)
            ctx=self.q(prob@v,base+'.attention_context').transpose(1,2).reshape(B,N,D)
            z=self.linear(ctx,base+'.self_attn.out_proj')
            x=self.q(x+z,base+'.residual1')
            z=self.norm(x,base+'.norm2')
            z=self.linear(z,base+'.linear1');z=self.q(F.gelu(z),base+'.gelu')
            z=self.linear(z,base+'.linear2');x=self.q(x+z,base+'.residual2')
        x=self.norm(x,'encoder.norm')
        x=x.transpose(1,2).reshape(x.shape[0],64,11,57)
        x=self.q(F.conv_transpose2d(x,self.p('decoder.0.weight'),self.p('decoder.0.bias'),stride=8),'decoder.0')
        x=self.q(F.gelu(x),'decoder.1')
        x=self.q(F.conv2d(x,self.p('decoder.2.0.weight'),self.p('decoder.2.0.bias'),
                          padding=self.dilation,dilation=self.dilation,groups=64),'decoder.2.depthwise')
        x=self.q(F.conv2d(x,self.p('decoder.2.1.weight'),self.p('decoder.2.1.bias')),'decoder.2.pointwise')
        x=self.q(F.gelu(x),'decoder.3')
        return self.q(F.conv2d(x,self.p('decoder.4.weight'),self.p('decoder.4.bias')),'decoder.4')
