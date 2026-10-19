`timescale 1ns/1ps
// Pack 16-channel x 12-bit pixel data into 32-bit words for USB TX.
// Each valid beat of i_dat carries 16 pixels; 8 output words are produced
// (2 pixels per word, each pixel zero-padded to 16 bit):
//   word[31:16] = {4'b0, pixel[2k+1][11:0]}
//   word[15:0]  = {4'b0, pixel[2k  ][11:0]}
// Input clock: clk_220M.  Output feeds an async FIFO.
module pixel_12to16 #(
    parameter CHAN_NUM  = 16,
    parameter DAT_WIDTH = 12
)(
    input                              clk,
    input                              rst,
    input  [CHAN_NUM-1:0]              i_dat_vld,   // train_dat_vld
    input  [CHAN_NUM*DAT_WIDTH-1:0]    i_dat,       // train_dat
    output reg                         o_wr_en,
    output reg [31:0]                  o_dout
);

localparam PAIRS = CHAN_NUM / 2;  // 8

reg [CHAN_NUM*DAT_WIDTH-1:0] dat_hold;
reg [3:0]                    pair_idx;
reg                          busy;

always @(posedge clk) begin
    if (rst) begin
        busy     <= 1'b0;
        pair_idx <= 4'd0;
        o_wr_en  <= 1'b0;
        o_dout   <= 32'd0;
        dat_hold <= 'd0;
    end else begin
        if (!busy) begin
            o_wr_en <= 1'b0;
            if (&i_dat_vld) begin       // all 16 channels valid simultaneously
                dat_hold <= i_dat;
                busy     <= 1'b1;
                pair_idx <= 4'd0;
            end
        end else begin
            o_dout[15:0]  <= {4'b0, dat_hold[(pair_idx*2  )*DAT_WIDTH +: DAT_WIDTH]};
            o_dout[31:16] <= {4'b0, dat_hold[(pair_idx*2+1)*DAT_WIDTH +: DAT_WIDTH]};
            o_wr_en <= 1'b1;
            if (pair_idx == PAIRS - 1) begin
                pair_idx <= 4'd0;
                busy     <= 1'b0;
            end else begin
                pair_idx <= pair_idx + 4'd1;
            end
        end
    end
end

endmodule
