"""Exhaustively compare packed 1024-word ROMs to HLS native ROM at clock/CE."""
import argparse,json,re,shutil,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=['dw','dw_dilated','dw_dilated_fast'],required=True);a=ap.parse_args()
    export=json.loads((ROOT/'reports'/f'T_{a.variant}_hls_export.json').read_text())
    source=ROOT/'01_hls'/f'vitseg_{a.variant}'/'solution1/syn/verilog'
    packed=ROOT/'02_vivado'/a.variant/'T_rtl_snapshot'
    work=ROOT/'reports'/f'T_rom_equivalence_{a.variant}'
    if work.exists():raise RuntimeError('Refuse overwrite ROM test workspace')
    work.mkdir()
    n=export['words_per_bank'];chunks=(n+1023)//1024
    files=[packed/'T_weight_rom_chunk.v'];instances=[];checks=[]
    for b in range(4):
        oldfiles=list(source.glob(f'*DOT_BLOCK_W_BANK{b}_ROM*.v'))
        newfiles=list(packed.glob(f'*DOT_BLOCK_W_BANK{b}_ROM*.v'))
        if len(oldfiles)!=1 or len(newfiles)!=1:raise RuntimeError('Missing ROM leaf')
        old=oldfiles[0].read_text();name=re.search(r'module\s+(\w+)',old).group(1)
        renamed=re.sub(r'\bmodule\s+'+re.escape(name)+r'\b','module '+name+'_ref',old,count=1)
        ref=work/f'{name}_ref.v';ref.write_text(renamed);files.extend([ref,newfiles[0]])
        dat=re.search(r'\$readmemh\("\./([^"/]+)"',old).group(1)
        shutil.copy2(source/dat,work/dat)
        for k in range(chunks):shutil.copy2(packed/f'T_W_BANK{b}_chunk{k:02d}.dat',work/f'T_W_BANK{b}_chunk{k:02d}.dat')
        instances.append(f"wire [31:0] got{b},ref{b};\n{name} n{b}(.clk(clk),.reset(1'b0),.address0(address),.ce0(ce),.q0(got{b}));\n{name}_ref r{b}(.clk(clk),.reset(1'b0),.address0(address),.ce0(ce),.q0(ref{b}));")
        checks.append(f'if(got{b}!==ref{b}) $fatal(1,"bank {b} mismatch address=%0d got=%h expected=%h",address,got{b},ref{b});')
    tb=['`timescale 1ns/1ps','module T_tb_weight_roms;reg clk=0;always #5 clk=~clk;reg [14:0] address=0;reg ce=0;',*instances,
        'task check;begin',*checks,'end endtask',
        f'integer i;initial begin repeat(2) @(negedge clk);for(i=0;i<{n};i=i+1)begin @(negedge clk);ce=1;address=i;@(posedge clk);#1;check;',
        f"if(i%7==0)begin @(negedge clk);ce=0;address={n-1}-i;@(posedge clk);#1;check;end end",
        '$display("T_PACKED_WEIGHT_ROM_TEST_PASS");$finish;end',
        'initial begin #1000000;$fatal(1,"ROM test timeout");end endmodule']
    tbfile=work/'T_tb_weight_roms.sv';tbfile.write_text('\n'.join(tb))
    output=work/'sim.out'
    compile=subprocess.run(['iverilog','-g2012','-o',str(output),str(tbfile)]+[str(p) for p in files],cwd=work,capture_output=True,text=True)
    if compile.returncode:raise RuntimeError('iverilog failed: '+compile.stdout+compile.stderr)
    simulation=subprocess.run(['vvp',str(output)],cwd=work,capture_output=True,text=True)
    if simulation.returncode or 'T_PACKED_WEIGHT_ROM_TEST_PASS' not in simulation.stdout:
        raise RuntimeError('ROM simulation failed: '+simulation.stdout+simulation.stderr)
    receipt=dict(status='pass',variant=a.variant,words_per_bank=n,banks=4,all_legal_addresses_exhaustive=True,
                 clock_enable_stalls_tested=True,read_latency_preserved=True,
                 scope='ROM leaf equivalence only; not full-core RTL or board equivalence')
    (ROOT/'reports'/f'T_rom_equivalence_{a.variant}.json').write_text(json.dumps(receipt,indent=2))
    print('ROM_RTL_EQUIVALENCE_PASS',a.variant,n*4,flush=True)
if __name__=='__main__':main()
