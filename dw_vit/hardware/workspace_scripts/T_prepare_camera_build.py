"""Bind one validated decoder variant to isolated camera replay RTL/build.

Run: D:\anaconda3\python.exe T_prepare_camera_build.py --variant dw
This prepares source only. It does not synthesize or program the camera.
"""
import argparse,hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OLD=Path(r'E:\ISCAS_3\vit0392_required_experiments_20261020')
BASE=Path(r'E:\ISCAS_3\vit0392_fpga_deploy_20261020')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def once(s,old,new):
    if s.count(old)!=1:raise RuntimeError(f'Expected one anchor: {old[:80]} found {s.count(old)}')
    return s.replace(old,new)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=['dw','dw_dilated'],required=True);args=ap.parse_args()
    export=json.loads((ROOT/'reports'/f'T_{args.variant}_hls_export.json').read_text())
    csim=json.loads((ROOT/'reports'/f'T_{args.variant}_csim_pass.json').read_text())
    pack=json.loads((ROOT/'reports'/f'T_{args.variant}_rom_pack.json').read_text())
    if csim['status']!='passed' or csim['checkpoint_sha256']!=export['checkpoint_sha256'] or pack['checkpoint_sha256']!=export['checkpoint_sha256']:
        raise RuntimeError('Unbound checkpoint, HLS Csim or ROM pack')
    core=ROOT/'02_vivado'/args.variant/f'T_vit_{args.variant}_synth.dcp'
    if not core.exists():raise RuntimeError('OOC core checkpoint missing')
    bridge=ROOT/'rtl/T_vit_replay_bridge.sv'
    text=bridge.read_text()
    old='57763af960b944beb884e7a4b6198ea7fac6897fadd9ff1381cee89317088bc8'
    text=once(text,old,export['checkpoint_sha256'])
    raw=export['threshold_raw']&0xffff
    text=once(text,"32'h08120033; // W8/A18 candidate; raw logit threshold=51.",
              f"32'h0812{raw:04x}; // W8/A18 candidate; signed raw logit threshold={export['threshold_raw']}.")
    bridge.write_text(text,encoding='utf-8')
    build=(OLD/'scripts/T_build_preload_full.tcl').read_text()
    build=once(build,'set mode full','set variant $env(T_VIT_VARIANT)\nif {$variant ni {dw dw_dilated}} {error "Missing selected decoder variant"}\nset mode full')
    build=once(build,'set core {E:/ISCAS_3/vit0392_fpga_deploy_20261020/02_vivado/T_vit_ooc_packed_hier/T_vit_ooc_packed_hier_synth.dcp}',
               'set core [file join $root 02_vivado $variant T_vit_${variant}_synth.dcp]')
    build=once(build,'puts [exec {D:/anaconda3/python.exe} -I {E:/ISCAS_3/vit0392_fpga_deploy_20261020/scripts/T_verify_weight_roms.py}]',
               'puts [exec {D:/anaconda3/python.exe} -I [file join $root scripts T_verify_roms.py] --variant $variant]')
    build=once(build,'[file join $cam T_preload_camera.xpr]','[file join $cam T_depthwise_camera.xpr]')
    build=once(build,'[file join $out T_camera_vit0392_preloaded.bit]','[file join $out T_camera_vit_depthwise.bit]')
    build=once(build,'[file join $out T_camera_vit0392_preloaded.ltx]','[file join $out T_camera_vit_depthwise.ltx]')
    dest=ROOT/'scripts/T_build_camera.tcl'
    if dest.exists():raise RuntimeError('Refusing to overwrite camera build script')
    dest.write_text(build,encoding='utf-8')
    # Copy only the known compact-camera CDC exception script into this copy.
    cdc=ROOT/'scripts/T_compact_observer_constraints.tcl'
    cdc.write_bytes((OLD/'scripts/T_compact_observer_constraints.tcl').read_bytes())
    receipt=dict(status='prepared_only',variant=args.variant,checkpoint_sha256=export['checkpoint_sha256'],
                 threshold_raw=export['threshold_raw'],ooc_checkpoint_sha256=sha(core),
                 bridge_sha256=sha(bridge),build_script_sha256=sha(dest),
                 original_camera_project_modified=False,prior_deployed_bitstream_modified=False,programmed=False)
    (ROOT/'reports/T_camera_build_binding.json').write_text(json.dumps(receipt,indent=2))
    print('CAMERA_BUILD_PREPARED_NOT_SYNTHESIZED',args.variant,flush=True)
if __name__=='__main__':main()
