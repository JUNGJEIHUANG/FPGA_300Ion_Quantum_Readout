"""Split generated W8 ROMs into one-cycle 1024-word blocks, isolated by variant.

Run: python T_pack_roms.py --variant dw
Requires HLS synthesis. Does not alter HLS source or checkpoint weights.
"""
import argparse,hashlib,json,re,shutil
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=['dw','dw_dilated','dw_dilated_fast'],required=True);args=ap.parse_args()
    report=json.loads((ROOT/'reports'/f'T_{args.variant}_hls_export.json').read_text())
    words=report['words_per_bank'];chunks=(words+1023)//1024
    source=ROOT/'01_hls'/f'vitseg_{args.variant}'/'solution1'/'syn'/'verilog'
    dest=ROOT/'02_vivado'/args.variant/'T_rtl_snapshot'
    if not (source/'vitseg_core.v').exists():raise RuntimeError('Missing HLS RTL')
    if dest.exists():raise RuntimeError(f'Refuse overwrite: {dest}')
    dest.mkdir(parents=True)
    for p in source.iterdir():
        if p.is_file():shutil.copy2(p,dest/p.name)
    chunk_module='''// Hierarchical one-cycle 1024x32 ROM, exact CE and address behavior.
(* keep_hierarchy="yes" *) module T_weight_rom_chunk(input wire clk,ce,input wire [9:0] address,output reg [31:0] data);
parameter INIT_FILE="";
(* rom_style="block" *) reg [31:0] mem[0:1023];
initial $readmemh(INIT_FILE,mem);
always @(posedge clk) if(ce) data<=mem[address];
endmodule
'''
    (dest/'T_weight_rom_chunk.v').write_text(chunk_module,encoding='ascii')
    records=[]
    for b in range(4):
        files=list(dest.glob(f'*DOT_BLOCK_W_BANK{b}_ROM*.v'))
        if len(files)!=1:raise RuntimeError(f'Expected one W_BANK{b} ROM leaf, got {files}')
        path=files[0];old=path.read_text();name=re.search(r'module\s+(\w+)',old).group(1)
        dat=re.search(r'\$readmemh\("\./([^"/]+)"',old).group(1)
        original=(source/dat).read_text().split()
        if len(original)!=words or any(not re.fullmatch('[0-9a-fA-F]{8}',v) for v in original):
            raise RuntimeError(f'Unexpected ROM data in bank {b}')
        code=[f'// Exact ROM banking; {words} words, {chunks} chunks.',
              f'module {name}(address0,ce0,q0,reset,clk);',
              f'parameter DataWidth=32, AddressWidth=15, AddressRange={words};',
              'input [AddressWidth-1:0] address0;input ce0,reset,clk;output reg [DataWidth-1:0] q0;',
              'reg [4:0] selected;',
              'always @(posedge clk) if(ce0) selected<=address0[14:10];']
        reconstructed=[]
        for k in range(chunks):
            block=original[k*1024:(k+1)*1024];reconstructed.extend(block)
            filename=f'T_W_BANK{b}_chunk{k:02d}.dat'
            (dest/filename).write_text('\n'.join(block+['00000000']*(1024-len(block)))+'\n',encoding='ascii')
            code += [f'wire [DataWidth-1:0] bank_q{k};',
                     f'(* keep_hierarchy="yes" *) T_weight_rom_chunk #(.INIT_FILE("./{filename}")) u_chunk{k}(.clk(clk),.ce(ce0 && address0[14:10]==5\'d{k}),.address(address0[9:0]),.data(bank_q{k}));']
        assert reconstructed==original
        code += ['always @* begin',' case(selected)']
        code += [f" 5'd{k}:q0=bank_q{k};" for k in range(chunks)]
        code += [" default:q0={DataWidth{1'b0}};",' endcase','end','endmodule']
        path.write_text('\n'.join(code)+'\n',encoding='ascii')
        records.append(dict(bank=b,words=words,chunks=chunks,original_data_sha256=sha(source/dat),
                            leaf_sha256=sha(path),all_words_inverse_verified=True))
    result=dict(status='generated',variant=args.variant,checkpoint_sha256=report['checkpoint_sha256'],
                source_hls_rtl_sha256=sha(source/'vitseg_core.v'),packed_core_rtl_sha256=sha(dest/'vitseg_core.v'),
                chunk_module_sha256=sha(dest/'T_weight_rom_chunk.v'),banks=records,
                read_latency_cycles=1,clock_enable_preserved=True,
                scope='ROM restructuring only; OOC synthesis, route and hardware tests pending')
    out=ROOT/'reports'/f'T_{args.variant}_rom_pack.json'
    if out.exists():raise RuntimeError(f'Refuse overwrite: {out}')
    out.write_text(json.dumps(result,indent=2))
    print('ROM_PACK_COMPLETE',args.variant,words,chunks,flush=True)

if __name__=='__main__':main()
