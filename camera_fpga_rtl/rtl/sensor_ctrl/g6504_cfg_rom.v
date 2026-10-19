`timescale 1ns/1ps

module g6504_cfg_rom #(
    parameter integer DEPTH = 598,
    parameter integer IDX_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH),
    parameter         MEM_FILE = "g6504_sensor_init_external_exposure.mem"
)(
    input  wire [IDX_W-1:0]         index,
    output wire [14:0]              addr,
    output wire [7:0]               data
);

    reg [23:0] mem [0:DEPTH-1];

    initial begin
        $readmemh(MEM_FILE, mem);
    end

    wire [23:0] raw_word = mem[index];

    assign addr = raw_word[22:8];
    assign data = raw_word[7:0];

endmodule
