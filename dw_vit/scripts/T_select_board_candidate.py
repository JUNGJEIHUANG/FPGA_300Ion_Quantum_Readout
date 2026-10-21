"""Freeze one board pilot candidate from validation metrics before test reads."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    out=ROOT/'reports/T_board_candidate_pretest.json'
    if out.exists():raise RuntimeError('Refuse to overwrite pretest selection')
    rows={}
    for variant in ('dw','dw_dilated'):
        p=ROOT/'reports'/f'T_{variant}_validation_thresholds.json'
        r=json.loads(p.read_text())
        if r['status']!='frozen' or r['test_accessed']:raise RuntimeError('Not validation-only')
        g=r['selected_validation'];m=g['150D_150B'];a=g['ALL']
        key=(m['packet_errors'],a['packet_errors'],m['fp'],0 if variant=='dw' else 1)
        rows[variant]=dict(checkpoint_sha256=r['checkpoint_sha256'],threshold=r['selected_threshold'],
                           selection_key=list(key),validation_mixed=m,validation_all=a)
    chosen=min(rows,key=lambda v:rows[v]['selection_key'])
    result=dict(status='pretest_frozen',candidate=chosen,criterion='Mixed validation packet errors, then all errors, then mixed false-bright; tie favors non-dilated',
                variants=rows,test_accessed=False,hardware_programmed=False,
                note='W8/A18, HLS, resources, and routed timing remain independent gates')
    out.write_text(json.dumps(result,indent=2))
    print('BOARD_CANDIDATE_FROZEN_PRETEST',chosen,flush=True)
if __name__=='__main__':main()
