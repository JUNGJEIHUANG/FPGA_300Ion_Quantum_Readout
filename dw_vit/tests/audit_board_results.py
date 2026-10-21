"""Offline audit of a complete 2000-frame DW-ViT FPGA replay.

Reads locally held board records and frozen benchmark truth for SCORING ONLY.
No data acquisition, inference, training, threshold selection, or device I/O.
Published outputs contain aggregate metrics and per-frame output hashes, not
image pixels, truth arrays, centroids, or individual state predictions.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse, hashlib, json, sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'support'))
from packet_scoring import score_centers


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def aggregate(rows):
    output = {}
    for name in ['ALL'] + sorted({r['category'] for r in rows}):
        part = rows if name == 'ALL' else [r for r in rows if r['category'] == name]
        keys = ['tp','fp','fn','tn','extra_packets','state_exact','record_exact','frames','packet_errors']
        g = {k:sum(r[k] for r in part) for k in keys}
        g.update(site_accuracy=(g['tp']+g['tn'])/max(1,g['tp']+g['fp']+g['fn']+g['tn']),
                 false_bright_rate=g['fp']/max(1,g['fp']+g['tn']),
                 false_dark_rate=g['fn']/max(1,g['fn']+g['tp']),
                 state_exact_rate=g['state_exact']/g['frames'],
                 record_exact_rate=g['record_exact']/g['frames'],
                 precision=g['tp']/(g['tp']+g['fp']) if g['tp']+g['fp'] else None,
                 recall=g['tp']/(g['tp']+g['fn']) if g['tp']+g['fn'] else None,
                 isotope_sites=sum(r['isotope_sites'] for r in part),
                 isotope_correct=sum(r['isotope_correct'] for r in part),
                 mean_count_abs_error=sum(r['count_abs_error'] for r in part)/len(part))
        g['isotope_accuracy']=g['isotope_correct']/g['isotope_sites'] if g['isotope_sites'] else None
        output[name]=g
    return output


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ['run-dir','fixture-dir','checkpoint','bitstream','attestation','output','output-hashes']:
        ap.add_argument('--'+name,required=True,type=Path)
    a=ap.parse_args();run=a.run_dir.resolve()
    summary_path=run/'T_summary.json';log_path=run/'T_frames.jsonl'
    summary=json.loads(summary_path.read_text());att=json.loads(a.attestation.read_text())
    assert summary['status']=='complete' and summary['completed']==summary['requested']==2000
    assert summary['variant']=='dw_dilated_fast'
    checkpoint_hash=sha(a.checkpoint);bitstream_hash=sha(a.bitstream)
    assert checkpoint_hash==att['checkpoint_sha256']==summary['checkpoint_sha256']
    assert bitstream_hash==att['bitstream_sha256']==summary['bitstream_sha256']
    fixture_path=a.fixture_dir/'reports/T_inputs_frozen.json';fixture=json.loads(fixture_path.read_text())
    assert fixture['status']=='frozen' and fixture['frames']==2000
    cache_hashes={}
    for name,expected in fixture['cache_sha256'].items():
        actual=sha(a.fixture_dir/'cache'/name);assert actual==expected,name;cache_hashes[name]=actual
    xy=np.load(a.fixture_dir/'cache/T_truth_xy.npy',mmap_mode='r')
    truth=np.load(a.fixture_dir/'cache/T_truth_states.npy',mmap_mode='r')
    isotope=np.load(a.fixture_dir/'cache/T_isotope.npy',mmap_mode='r')
    assert truth.shape==(2000,300) and xy.shape==(2000,300,2)
    rows=[json.loads(s) for s in log_path.read_text().splitlines() if s.strip()]
    assert len(rows)==2000
    assert sorted(r['id'] for r in rows)==list(range(2000))
    assert sorted(r['run_id'] for r in rows)==list(range(2000))
    hashes=[];rescored=[]
    for row in rows:
        i=row['id'];expected=fixture['images'][i]
        assert expected['id']==i and row['group']==expected['group']
        assert row['input_written']==row['input_consumed']==40128
        assert row['input_signature'].lower()==expected['signature'].lower()
        assert row['transport_passed'] is True
        assert row['input_starvation_cycles']==row['output_backpressure_cycles']==row['forced_pause_cycles']==0
        assert row['partial_upload_then_reset'] is False
        file=run/Path(row['words_file']).name
        assert file.resolve().parent==run
        digest=sha(file);assert digest==row['words_sha256']
        words=np.array([int(w,16) for w in file.read_text().split()],dtype='<u4').reshape(-1,3)
        assert len(words)==row['count']==len(row['centers'])==len(row['areas'])
        centers=words[:,:2].copy().view('<f4');areas=words[:,2]
        assert np.array_equal(centers,np.asarray(row['centers'],np.float32).reshape(-1,2))
        assert np.array_equal(areas,np.asarray(row['areas'],np.uint32))
        assert np.isfinite(centers).all()
        if len(words):
            assert ((centers[:,0]>=0)&(centers[:,0]<456)&(centers[:,1]>=0)&(centers[:,1]<88)).all()
            assert ((areas>=2)&(areas<=45)).all()
        metric,prediction=score_centers(centers,xy[i],truth[i],isotope[i])
        assert np.array_equal(prediction,np.asarray(row['site_prediction'],np.uint8))
        for k,v in metric.items():
            assert v==row['metrics'][k],(i,k,v,row['metrics'].get(k))
        metric.update(category=row['group'],packet_errors=metric['fp']+metric['fn']+metric['extra_packets']);rescored.append(metric)
        hashes.append({'frame_id':i,'category':row['group'],'record_count':row['count'],'sha256':digest})
    groups=aggregate(rescored)
    assert set(groups)==set(summary['groups'])
    for group,g in groups.items():
        for k,v in g.items():
            original=summary['groups'][group][k]
            assert (v is None and original is None) or (v is not None and original is not None and abs(v-original)<1e-12),(group,k,v,original)
    assert [groups[k]['frames'] for k in sorted(groups) if k!='ALL']==[250,250,1000,100,400]
    report={'status':'pass','audit_utc':datetime.now(timezone.utc).isoformat(),
            'variant':'dw_dilated_fast','frames':2000,'unique_frame_ids':2000,
            'checkpoint_sha256':checkpoint_hash,'bitstream_sha256':bitstream_hash,
            'source_summary_sha256':sha(summary_path),'source_frames_log_sha256':sha(log_path),
            'frozen_fixture_sha256':sha(fixture_path),'frozen_cache_sha256':cache_hashes,
            'raw_output_files_hash_verified':2000,'raw_record_decode_verified':2000,
            'input_counts_and_signatures_verified':2000,'transport_passed_frames':2000,
            'offline_site_predictions_recomputed':2000,'offline_aggregate_summary_matches':True,
            'groups':groups,'ground_truth_used_for_offline_scoring_only':True,
            'checkpoint_or_threshold_changed':False,'hardware_operations_triggered':False,
            'scope':'Complete previously exposed 2000-frame FPGA dataset replay with USB-preloaded input. Not live-ion capture, sensor calibration, direct 300-bit board output, FP32 equivalence, or full C/RTL bit equivalence.'}
    hashes.sort(key=lambda x:x['frame_id'])
    for path,value in [(a.output,report),(a.output_hashes,{'frames':2000,'contains_raw_outputs_or_truth':False,'records':hashes})]:
        assert not path.exists(),f'Refuse overwrite: {path}'
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(json.dumps(value,indent=2).encode())
    print(json.dumps({'status':'pass','frames':2000,'all':groups['ALL'],'mixed':groups['03_pi2_binomial_1000']},indent=2))

if __name__=='__main__':
    main()
