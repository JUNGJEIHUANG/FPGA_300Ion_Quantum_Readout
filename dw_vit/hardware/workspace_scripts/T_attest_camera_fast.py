"""Attest routed bit-exact fast HLS implementation before volatile programming."""
import hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    binding=json.loads((ROOT/'reports/T_fast_camera_binding.json').read_text())
    prior=json.loads((ROOT/'reports/T_camera_bitstream_attestation.json').read_text())
    assert binding['checkpoint_sha256']==prior['checkpoint_sha256']
    assert binding['threshold_raw']==prior['threshold_raw']
    core=ROOT/'02_vivado/dw_dilated_fast/T_vit_dw_dilated_fast_synth.dcp'
    prelink=ROOT/'02_vivado/T_preload_camera_prelink.dcp'
    assert sha(core)==binding['fast_core_sha256'] and sha(prelink)==binding['prelink_sha256']
    assert sha(ROOT/'rtl/T_vit_replay_bridge.sv')==binding['source_camera_bridge_sha256']
    assert sha(ROOT/'scripts/T_build_camera_fast.tcl')==binding['build_script_sha256']
    assert sha(ROOT/'output/T_camera_vit_depthwise.bit')==binding['previous_camera_bitstream_sha256']
    gate=(ROOT/'reports/T_fast_implementation_gate.txt').read_text()
    if 'CAMERA_PRESERVING_IMPLEMENTATION_TIMING_DRC_PASS' not in gate:raise RuntimeError('No implementation pass marker')
    setup=float(re.search(r'SETUP_SLACK_NS\s+([\-0-9.]+)',gate).group(1))
    hold=float(re.search(r'HOLD_SLACK_NS\s+([\-0-9.]+)',gate).group(1))
    if setup<0 or hold<0:raise RuntimeError('Timing violation')
    bit=ROOT/'output/T_camera_vit_depthwise_fast.bit';ltx=ROOT/'output/T_camera_vit_depthwise_fast.ltx'
    if not bit.exists() or not ltx.exists():raise RuntimeError('Missing fast routed outputs')
    result=dict(status='ready_for_volatile_programming',variant='dw_dilated_fast',
                checkpoint_sha256=binding['checkpoint_sha256'],threshold_raw=binding['threshold_raw'],
                bitstream_sha256=sha(bit),ltx_sha256=sha(ltx),setup_slack_ns=setup,
                hold_slack_ns=hold,previous_serial_bitstream_sha256=sha(ROOT/'output/T_camera_vit_depthwise.bit'),
                old_known_good_bitstream_sha256=prior['baseline_recovery_bitstream_sha256'],
                algorithm_values_changed=False,fpga_programmed=False)
    out=ROOT/'reports/T_camera_fast_bitstream_attestation.json'
    if out.exists() and json.loads(out.read_text())!=result:raise RuntimeError('Attestation changed')
    out.write_text(json.dumps(result,indent=2))
    print('FAST_CAMERA_BIT_ATTESTED_NOT_PROGRAMMED',result['bitstream_sha256'],flush=True)
if __name__=='__main__':main()
