`timescale 1ns / 1ps
module image_dat_tx_0 (
    input  wire         clk,
    input  wire         rst,
    input  wire         i_cmd_frame_req,
    input  wire [15:0]  i_lval_len,
    input  wire [15:0]  i_win_row_len,
    input  wire [15:0]  i_win_col_len,
    input  wire [15:0]  i_frame_num_tx,
    input  wire [12:0]  i_tx_fifo_cnt,
    input  wire         i_din_rdy,
    input  wire [15:0]  i_din_vld,
    input  wire [255:0] i_din,
    output wire [15:0]  o_din_rd_en,
    output wire         o_dout_vld,
    output wire [31:0]  o_dout,
    output wire         o_tx_row_done,
    output wire         o_img2tx_tx_en,
    output wire         o_frame_done
);
    image_dat_tx #(
        .INTERLACE_EN       (1'b1),
        .TX_FIFO_CNT_WIDTH  (13),
        .CHAN_NUM           (16),
        .FVAL_GEN_EN        (1'b0)
    ) inst (
        .clk                (clk),
        .rst                (rst),
        .i_cmd_frame_req    (i_cmd_frame_req),
        .i_lval_len         (i_lval_len),
        .i_win_row_len      (i_win_row_len),
        .i_win_row_len_fval (16'd0),
        .i_win_col_len      (i_win_col_len),
        .i_frame_num_tx     (i_frame_num_tx),
        .i_tx_fifo_cnt      (i_tx_fifo_cnt),
        .i_din_rdy          (i_din_rdy),
        .i_din_vld          (i_din_vld),
        .i_din              (i_din),
        .o_din_rd_en        (o_din_rd_en),
        .o_dout_fval        (),
        .o_dout_vld         (o_dout_vld),
        .o_dout             (o_dout),
        .o_tx_row_done      (o_tx_row_done),
        .o_img2tx_tx_en     (o_img2tx_tx_en),
        .o_frame_done       (o_frame_done)
    );
endmodule
