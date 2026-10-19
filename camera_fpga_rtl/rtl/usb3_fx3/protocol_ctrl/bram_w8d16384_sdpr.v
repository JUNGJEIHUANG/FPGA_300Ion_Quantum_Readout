`timescale 1ns/1ps

// Portable replacement for the vendor blk_mem_gen instance used by
// sensor_protocol.  The two clocks intentionally match the original simple
// dual-port RAM: byte writes on port A and synchronous byte reads on port B.
module bram_w8d16384_sdpr (
    input  wire        clka,
    input  wire [0:0]  wea,
    input  wire [13:0] addra,
    input  wire [7:0]  dina,
    input  wire        clkb,
    input  wire        enb,
    input  wire [13:0] addrb,
    output reg  [7:0]  doutb
);
    (* ram_style = "block" *) reg [7:0] mem [0:16383];

    always @(posedge clka) begin
        if (wea[0])
            mem[addra] <= dina;
    end

    always @(posedge clkb) begin
        if (enb)
            doutb <= mem[addrb];
    end
endmodule
