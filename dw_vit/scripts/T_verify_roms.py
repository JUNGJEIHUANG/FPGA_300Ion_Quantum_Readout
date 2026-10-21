"""Recheck every generated weight-ROM word against frozen checkpoint export."""
import argparse,json,re,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=['dw','dw_dilated','dw_dilated_fast'],required=True);a=ap.parse_args()
    report=json.loads((ROOT/'reports'/f'T_{a.variant}_rom_pack.json').read_text())
    export=json.loads((ROOT/'reports'/f'T_{a.variant}_hls_export.json').read_text())
    rtl=ROOT/'02_vivado'/a.variant/'T_rtl_snapshot'
    assert report['checkpoint_sha256']==export['checkpoint_sha256']
    assert sha(rtl/'vitseg_core.v')==report['packed_core_rtl_sha256']
    assert sha(rtl/'T_weight_rom_chunk.v')==report['chunk_module_sha256']
    for b in range(4):
        originals=(ROOT/'parameters'/a.variant/f'hls_wbank{b}.bin').read_bytes()
        import numpy as np
        words=np.frombuffer(originals,dtype='<u4')
        text=[]
        for k in range(report['banks'][b]['chunks']):
            p=rtl/f'T_W_BANK{b}_chunk{k:02d}.dat'
            vals=p.read_text().split()
            assert len(vals)==1024 and all(re.fullmatch('[0-9a-fA-F]{8}',v) for v in vals)
            text.extend(vals)
        reconstructed=np.asarray([int(x,16) for x in text[:len(words)]],dtype='<u4')
        if not np.array_equal(reconstructed,words):raise RuntimeError(f'ROM mismatch bank {b}')
        if any(int(x,16)!=0 for x in text[len(words):]):raise RuntimeError('Nonzero padding')
    print('ROM_EXHAUSTIVE_CHECK_PASS',a.variant,export['words_per_bank']*4,flush=True)
if __name__=='__main__':main()
