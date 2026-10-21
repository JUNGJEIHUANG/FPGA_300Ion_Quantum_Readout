"""Measure an attested decoder variant on the camera FPGA via preloaded USB replay.

Run: D:\anaconda3\python.exe -u T_board_replay.py --mode pilot
First programs a verified bitstream via a separate Tcl flow; this script never
programs hardware. Pilot is five frames, not an accuracy estimate.
"""
import argparse,ctypes,hashlib,importlib.util,json,statistics,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OLD=Path(r'E:\ISCAS_3\vit0392_required_experiments_20261020')
SPEC=importlib.util.spec_from_file_location('old_preloaded_transport',OLD/'scripts/T_run_preloaded.py')
old=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(old)
sys.path.insert(0,str(OLD.parent/'small_baselines_seed1_20261020/scripts'))
from packet_scoring import score_centers
from evaluation_utils import detailed_summary
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,obj):
    p=Path(p);temp=p.with_suffix(p.suffix+'.tmp');temp.write_text(json.dumps(obj,indent=2));temp.replace(p)

class Candidate(old.Preloaded):
    def __init__(self,session,checkpoint_sha256,raw_threshold):
        super().__init__(session);self.expected_sha=checkpoint_sha256;self.raw_threshold=raw_threshold
    def probe(self):
        if self.read(0x100)[0]!=0x56495431 or self.read(0x128)[0]!=0x50524c33:raise RuntimeError('Not preloaded ViTSeg fixture')
        value=self.read(0x124)[0]
        if value>>16!=0x0812 or ((value&0xffff)^0x8000)-0x8000!=self.raw_threshold:
            raise RuntimeError(f'Numerical format or threshold mismatch {value:08x}')
        if self.read(0x130)[0]!=40128:raise RuntimeError('Wrong input capacity')
        digest=''.join(f'{x:08x}' for x in self.read(0x140,8))
        if digest!=self.expected_sha:raise RuntimeError(f'Checkpoint metadata mismatch {digest}')
        return digest

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=['pilot','full'],default='pilot');ap.add_argument('--resume',action='store_true')
    ap.add_argument('--implementation',choices=['serial','fast'],default='serial');args=ap.parse_args()
    fast=args.implementation=='fast'
    binding=json.loads((ROOT/'reports'/('T_fast_camera_binding.json' if fast else 'T_camera_build_binding.json')).read_text())
    variant=binding['variant'];checkpoint_sha=binding['checkpoint_sha256']
    bit=ROOT/'output'/('T_camera_vit_depthwise_fast.bit' if fast else 'T_camera_vit_depthwise.bit')
    if not bit.exists():raise RuntimeError('No routed bitstream')
    programming=ROOT/'reports'/('T_program_camera_fast_receipt.txt' if fast else 'T_program_camera_receipt.txt')
    if not programming.exists():raise RuntimeError('No programming receipt')
    if 'PROGRAM_HW_DEVICES_RETURNED_SUCCESS' not in programming.read_text():
        raise RuntimeError('Programming receipt does not confirm success')
    attestation=json.loads((ROOT/'reports'/('T_camera_fast_bitstream_attestation.json' if fast else 'T_camera_bitstream_attestation.json')).read_text())
    if attestation['checkpoint_sha256']!=checkpoint_sha or attestation['bitstream_sha256']!=sha(bit):
        raise RuntimeError('Bitstream attestation mismatch')
    m=json.loads((OLD/'reports/T_inputs_frozen.json').read_text())
    for name,digest in m['cache_sha256'].items():
        if sha(OLD/'cache'/name)!=digest:raise RuntimeError('Frozen benchmark cache changed')
    raw=np.load(OLD/'cache/T_raw_q8_u16.npy',mmap_mode='r')
    xy=np.load(OLD/'cache/T_truth_xy.npy',mmap_mode='r')
    truth=np.load(OLD/'cache/T_truth_states.npy',mmap_mode='r')
    isotope=np.load(OLD/'cache/T_isotope.npy',mmap_mode='r')
    out=ROOT/'reports'/f'T_board_{variant}_{args.mode}'
    out.mkdir(exist_ok=args.resume)
    schedule=[0,250,500,1500,1600] if args.mode=='pilot' else list(range(2000))
    log=out/'T_frames.jsonl'
    rows=[json.loads(x) for x in log.read_text().splitlines()] if log.exists() else []
    done={r['run_id'] for r in rows}
    ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
    session=old.usb.DllSession(old.usb.HOST/'third_party/cobra_dlls/gp_native_fx3.dll',0,5000)
    try:
        dev=Candidate(session,checkpoint_sha,binding['threshold_raw'])
        dev.probe()
        for run_id,i in enumerate(schedule):
            if run_id in done:continue
            row=dev.frame(m['images'][i],raw[i],out,run_id)
            if not row['fpga_latency_qualified']:raise RuntimeError(f'Unqualified timing frame {i}')
            if fast:
                old_words=ROOT/'reports/T_board_dw_dilated_pilot'/f'T_run_{run_id:04d}_image_{i:04d}_words.hex'
                if old_words.exists():
                    row['same_model_serial_board_words_bitexact']=Path(row['words_file']).read_bytes()==old_words.read_bytes()
                    if not row['same_model_serial_board_words_bitexact']:
                        raise RuntimeError(f'Fast hardware output differs from serial implementation on frame {i}')
            metric,pred=score_centers(row['centers'],xy[i],truth[i],isotope[i]);metric.update(id=i,category=row['group'])
            row['metrics']=metric;row['site_prediction']=pred.tolist()
            with log.open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n');f.flush()
            rows.append(row)
            times=[r['latency_ms']['fpga_kernel'] for r in rows]
            summary=dict(status='complete' if len(rows)==len(schedule) else 'partial',mode=args.mode,
                         variant=variant,checkpoint_sha256=checkpoint_sha,bitstream_sha256=sha(bit),
                         completed=len(rows),requested=len(schedule),
                         latency_ms=dict(mean=float(np.mean(times)),p50=float(np.median(times)),
                                         p95=float(np.percentile(times,95)),maximum=max(times)),
                         groups=detailed_summary([r['metrics'] for r in rows]),
                         timing_scope='FPGA start to ap_done, input preloaded, no external stalls, USB excluded',
                         accuracy_scope='Previously exposed benchmark; pilot not representative' if args.mode=='pilot' else 'Previously exposed full2000 benchmark')
            write(out/'T_summary.json',summary)
            print('BOARD_FRAME',variant,i,'kernel_ms',times[-1],len(rows),'/',len(schedule),flush=True)
    finally:
        session.close();ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
if __name__=='__main__':main()
