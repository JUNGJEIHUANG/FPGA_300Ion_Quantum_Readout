"""Portable adapter retaining the exact experiment variant names."""
from pathlib import Path
import importlib.util
p = Path(__file__).resolve().parents[2] / 'nets' / 'DWViT.py'
spec = importlib.util.spec_from_file_location('released_dw_vit', p)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
DWViT = mod.DWViT

def build(variant):
    return DWViT(dilation={'dw': 1, 'dw_dilated': 2}[variant])
