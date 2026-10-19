`timescale 1ns / 1ps
module ddr_dout_buf_0 (
    input  wire         clk_wr,
    input  wire         rst_wr,
    input  wire         clk_rd,
    input  wire         rst_rd,
    input  wire         i_cmd_frame_req,
    input  wire [9:0]   i_lval_len_wr,
    input  wire         i_din_vld,
    input  wire [511:0] i_din,
    output wire         o_din_req,
    input  wire [15:0]  i_dout_rd_en,
    output wire [15:0]  o_dout_vld,
    output wire [255:0] o_dout,
    output wire         o_dout_rdy
);
    ddr_dout_buf #(
        .INTERLACE_EN    (1),
        .BRAM_DEPTH      (1024),
        .BRAM_ADDR_WIDTH (10),
        .DAT_WIDTH_IN    (16),
        .DAT_WIDTH_OUT   (16),
        .DOUBLE_IN_EN    (1),
        .CHAN_NUM_IN     (32),
        .CHAN_NUM_OUT    (16)
    ) inst (
        .clk_wr          (clk_wr),
        .rst_wr          (rst_wr),
        .clk_rd          (clk_rd),
        .rst_rd          (rst_rd),
        .i_cmd_frame_req (i_cmd_frame_req),
        .i_lval_len_wr   (i_lval_len_wr),
        .i_din_vld       (i_din_vld),
        .i_din           (i_din),
        .o_din_req       (o_din_req),
        .i_dout_rd_en    (i_dout_rd_en),
        .o_dout_vld      (o_dout_vld),
        .o_dout          (o_dout),
        .o_dout_rdy      (o_dout_rdy)
    );
endmodule