from pathlib import Path
import importlib.util
import torch
p=Path(__file__).resolve().parents[2]/'nets/DWViT.py'
spec=importlib.util.spec_from_file_location('released_dwvit',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
torch.set_num_threads(4)
for dilation in (1,2):
    model=m.DWViT(dilation=dilation).eval()
    assert sum(p.numel() for p in model.parameters()) == 376321
    assert model.decoder[2][0].groups == 64
    assert model.decoder[2][0].dilation == (dilation,dilation)
    with torch.inference_mode(): out=model(torch.zeros(1,1,88,456))
    assert out.shape == (1,1,88,456) and torch.isfinite(out).all()
print('PASS: two DW-ViT dilation configurations, parameter counts, and output shapes; no hardware accessed.')
