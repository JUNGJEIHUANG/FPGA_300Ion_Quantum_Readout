"""Recheck Csim receipt source/checkpoint before expensive HLS synthesis."""
import argparse,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=['dw','dw_dilated','dw_dilated_fast'],required=True);a=ap.parse_args()
    receipt=json.loads((ROOT/'reports'/f'T_{a.variant}_csim_pass.json').read_text())
    export=json.loads((ROOT/'reports'/f'T_{a.variant}_hls_export.json').read_text())
    assert receipt['status']=='passed' and receipt['checkpoint_sha256']==export['checkpoint_sha256']
    assert receipt['source_sha256']==sha(ROOT/'01_hls'/f'src_{a.variant}'/'vitseg_core.cpp')
    assert receipt['parameter_sha256']==sha(ROOT/'01_hls'/f'src_{a.variant}'/'vit_parameters.h')
    assert receipt['logits_sha256']==sha(ROOT/'reports'/f'T_csim_{a.variant}_mixed_logits_q18.bin')
    print('CSIM_GATE_RECHECK_PASS',a.variant)
if __name__=='__main__':main()
