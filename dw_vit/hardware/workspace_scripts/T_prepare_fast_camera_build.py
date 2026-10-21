"""Prepare a second camera route from the frozen same-metadata camera prelink.

The trained checkpoint, threshold, bridge, camera source and RTL interface are
unchanged. Only the bit-exact depthwise hardware schedule is replaced.
"""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
FAST='dw_dilated_fast'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def once(s,a,b):
    if s.count(a)!=1:raise RuntimeError(f'Expected one Tcl anchor: {a} found {s.count(a)}')
    return s.replace(a,b)
def main():
    base=json.loads((ROOT/'reports/T_camera_build_binding.json').read_text())
    fast=json.loads((ROOT/'reports/T_fast_depthwise_source.json').read_text())
    regression=json.loads((ROOT/'reports/T_fast_csim_bitexact.json').read_text())
    if base['variant']!='dw_dilated' or base['checkpoint_sha256']!=fast['checkpoint_sha256']:
        raise RuntimeError('Camera metadata or trained checkpoint changed')
    if regression['status']!='pass' or not all(regression['bitexact_files'].values()):
        raise RuntimeError('No full-frame C-model bitexact check')
    core=ROOT/'02_vivado'/FAST/f'T_vit_{FAST}_synth.dcp'
    prelink=ROOT/'02_vivado/T_preload_camera_prelink.dcp'
    if not core.exists() or not prelink.exists():raise RuntimeError('Missing fast core or camera prelink DCP')
    source=(ROOT/'scripts/T_build_camera.tcl').read_text()
    source=once(source,'if {$variant ni {dw dw_dilated}}','if {$variant ni {dw_dilated_fast}}')
    source=once(source,'T_preload_camera_prelink.dcp','__PRESERVE_PRELINK__')
    source=source.replace('T_preload_camera_','T_fast_camera_')
    source=once(source,'__PRESERVE_PRELINK__','T_preload_camera_prelink.dcp')
    source=once(source,'T_preload_implementation_gate.txt','T_fast_implementation_gate.txt')
    source=once(source,'T_camera_prelink_constraints.xdc','T_fast_prelink_constraints.xdc')
    source=once(source,'T_camera_vit_depthwise.bit','T_camera_vit_depthwise_fast.bit')
    source=once(source,'T_camera_vit_depthwise.ltx','T_camera_vit_depthwise_fast.ltx')
    dest=ROOT/'scripts/T_build_camera_fast.tcl'
    if dest.exists():raise RuntimeError('Refuse overwrite')
    dest.write_text(source,encoding='utf-8')
    receipt=dict(status='prepared_only',variant=FAST,checkpoint_sha256=base['checkpoint_sha256'],
                 threshold_raw=base['threshold_raw'],fast_core_sha256=sha(core),prelink_sha256=sha(prelink),
                 source_camera_bridge_sha256=sha(ROOT/'rtl/T_vit_replay_bridge.sv'),
                 build_script_sha256=sha(dest),previous_camera_bitstream_sha256=sha(ROOT/'output/T_camera_vit_depthwise.bit'),
                 original_camera_project_modified=False,programmed=False)
    (ROOT/'reports/T_fast_camera_binding.json').write_text(json.dumps(receipt,indent=2))
    print('FAST_CAMERA_BUILD_PREPARED_FROM_PRELINK_NOT_ROUTED',flush=True)
if __name__=='__main__':main()
