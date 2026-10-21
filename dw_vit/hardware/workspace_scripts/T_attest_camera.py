"""Bind routed bitstream to frozen variant before volatile JTAG programming."""
import hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OLD=Path(r'E:\ISCAS_3\vit0392_required_experiments_20261020')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    binding=json.loads((ROOT/'reports/T_camera_build_binding.json').read_text())
    variant=binding['variant']
    export=json.loads((ROOT/'reports'/f'T_{variant}_hls_export.json').read_text())
    core=ROOT/'02_vivado'/variant/f'T_vit_{variant}_synth.dcp'
    assert binding['checkpoint_sha256']==export['checkpoint_sha256']
    assert sha(core)==binding['ooc_checkpoint_sha256']
    assert sha(ROOT/'rtl/T_vit_replay_bridge.sv')==binding['bridge_sha256']
    assert sha(ROOT/'scripts/T_build_camera.tcl')==binding['build_script_sha256']
    bit=ROOT/'output/T_camera_vit_depthwise.bit';ltx=ROOT/'output/T_camera_vit_depthwise.ltx'
    gate=(ROOT/'reports/T_preload_implementation_gate.txt').read_text()
    assert bit.exists() and ltx.exists() and 'CAMERA_PRESERVING_IMPLEMENTATION_TIMING_DRC_PASS' in gate
    setup=float(re.search(r'SETUP_SLACK_NS\s+([\-0-9.]+)',gate).group(1))
    hold=float(re.search(r'HOLD_SLACK_NS\s+([\-0-9.]+)',gate).group(1))
    if setup<0 or hold<0:raise RuntimeError('Timing failed')
    baseline=OLD/'output/T_camera_vit0392_preloaded.bit'
    assert sha(baseline)=='9f2abaabe29cc4023eac68b91235e1b447d6cad99c7d8370117df68a2ec8b08d'
    receipt=dict(status='ready_for_volatile_programming',variant=variant,checkpoint_sha256=export['checkpoint_sha256'],
                 threshold_raw=export['threshold_raw'],bitstream_sha256=sha(bit),ltx_sha256=sha(ltx),
                 setup_slack_ns=setup,hold_slack_ns=hold,baseline_recovery_bitstream_sha256=sha(baseline),
                 original_camera_project_untouched=True,flash_untouched=True,board_tested=False)
    p=ROOT/'reports/T_camera_bitstream_attestation.json'
    if p.exists() and json.loads(p.read_text())!=receipt:raise RuntimeError('Attestation changed')
    p.write_text(json.dumps(receipt,indent=2))
    print('CAMERA_VARIANT_BIT_ATTESTED_NOT_PROGRAMMED',variant,receipt['bitstream_sha256'],flush=True)
if __name__=='__main__':main()
