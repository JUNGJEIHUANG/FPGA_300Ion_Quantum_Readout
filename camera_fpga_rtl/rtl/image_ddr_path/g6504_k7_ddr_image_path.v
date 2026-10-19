`timescale 1ns / 1ps
// K7_Demo-compatible DDR image path for G6504 bring-up.
//
// This module deliberately reuses the known-working K7 DDR architecture:
//   lvds_imagebuffer 384b row beat -> image_ddr_driver/mig_driver -> MIG
//   MIG read data -> 12b-to-16b pad -> ddr_dout_buf_0 -> image_dat_tx_0 -> USB stream
//
// Geometry for GSENSE6504 STD mode (matching the official imageprocess path):
//   272 x 384-bit beats per DDR row group, 512 row groups/frame.
//   Each group contains four 2176-pixel display rows (8704 pixels/group).

module g6504_k7_ddr_image_path #(
    parameter [15:0] LVAL_LEN_DDR    = 16'd272,
    parameter [15:0] WIN_ROW_LEN     = 16'd512,
    parameter [15:0] WIN_COL_LEN_TX  = 16'd8704,
    parameter [15:0] FRAME_NUM       = 16'd1
)(
    input  wire        clk_img,
    input  wire        rst_img,
    input  wire        clk_sys,
    input  wire        rst_sys,
    input  wire        clk_200m,
    input  wire        i_cmd_frame_req,
    input  wire        i_wide_vld,
    input  wire [383:0] i_wide_dat,
    input  wire [1:0]  i_hdr_mode_en,
    input  wire [15:0] i_lval_len_ddr,
    input  wire [15:0] i_win_row_len,
    input  wire [15:0] i_win_col_len,
    input  wire [15:0] i_frame_num,
    input  wire [15:0] i_frame_num_tx,
    // Match K7_Demo: expose only the lower 13 count bits so image_dat_tx
    // throttles near 8K and keeps the upper half of the 16K FIFO as margin.
    input  wire [12:0] i_tx_fifo_cnt,

    output wire        o_img_dat_vld,
    output wire [31:0] o_img_dat,
    output wire        o_img_tx_en,
    output wire        o_frame_done,

    output wire        o_ui_clk,
    output wire        o_ui_rst,
    output wire        o_init_calib_complete,
    output wire        o_ddr_wr_done,
    output wire        o_ddr_rd_req,
    output wire        o_dbg_rd_req_ui,
    output wire        o_ddr_dat_vld,
    output wire [383:0] o_ddr_dat, // Observation-only, native clk_ui data
    output wire        o_rowbuf_rdy,
    output wire        o_rowbuf_vld_any,
    output wire        o_rowbuf_rd_en_any,
    output wire        o_tx_row_done,
    output wire [31:0] o_dbg0,
    output wire [31:0] o_dbg1,

    output wire [15:0] o_DDR3_ADDR,
    output wire [2:0]  o_DDR3_BA,
    output wire        o_DDR3_CAS_N,
    output wire        o_DDR3_CK_N,
    output wire        o_DDR3_CK_P,
    output wire        o_DDR3_CKE,
    output wire        o_DDR3_RAS_N,
    output wire        o_DDR3_RESET_N,
    output wire        o_DDR3_WE_N,
    inout  wire [63:0] io_DDR3_DQ,
    inout  wire [7:0]  io_DDR3_DQS_N,
    inout  wire [7:0]  io_DDR3_DQS_P,
    output wire        o_DDR3_CS_N,
    output wire [7:0]  o_DDR3_DM,
    output wire        o_DDR3_ODT
);

    wire        clk_ui;
    wire        rst_ui;
    wire        init_calib_complete;
    wire        ddr_wr_done;
    wire        ddr_rd_req;
    wire        ddr_dat_vld;
    wire [383:0] ddr_dat;

    wire        rowbuf_rdy;
    wire [15:0] rowbuf_vld;
    wire [255:0] rowbuf_dout;
    wire [15:0] rowbuf_rd_en;
    wire        tx_row_done;
    wire [15:0] cfg_lval_len_ddr = (i_lval_len_ddr != 16'd0) ? i_lval_len_ddr : LVAL_LEN_DDR;
    wire [15:0] cfg_win_row_len  = (i_win_row_len  != 16'd0) ? i_win_row_len  : WIN_ROW_LEN;
    wire [15:0] cfg_win_col_len  = (i_win_col_len  != 16'd0) ? i_win_col_len  : (WIN_COL_LEN_TX >> 2);
    wire [15:0] cfg_frame_num    = (i_frame_num    != 16'd0) ? i_frame_num    : FRAME_NUM;
    wire [15:0] cfg_frame_num_tx = (i_frame_num_tx != 16'd0) ? i_frame_num_tx : cfg_frame_num;
    wire [15:0] cfg_lval_len_tx  = cfg_lval_len_ddr << 1;
    wire [15:0] cfg_win_col_tx   = cfg_win_col_len << 2;
    assign o_ui_clk              = clk_ui;
    assign o_ui_rst              = rst_ui;
    assign o_ddr_wr_done         = ddr_wr_done;
    assign o_ddr_rd_req          = ddr_rd_req;
    assign o_ddr_dat_vld         = ddr_dat_vld;
    assign o_ddr_dat             = ddr_dat;
    assign o_rowbuf_rdy          = rowbuf_rdy;
    assign o_rowbuf_vld_any      = |rowbuf_vld;
    assign o_rowbuf_rd_en_any    = |rowbuf_rd_en;
    assign o_tx_row_done         = tx_row_done;

    function [511:0] pad_32x12_to_32x16;
        input [383:0] din;
        integer i;
        begin
            pad_32x12_to_32x16 = 512'd0;
            for (i = 0; i < 32; i = i + 1) begin
                pad_32x12_to_32x16[i*16 +: 16] = {4'b0000, din[i*12 +: 12]};
            end
        end
    endfunction

    image_ddr_driver #(
        .IMAGE_DAT_WIDTH(384)
    ) u_image_ddr_driver (
        .clk_rx                (clk_img),
        .rst_rx                (rst_img),
        .clk_200m              (clk_200m),
        .rst_200m              (rst_sys),
        .i_cmd_frame_req       (i_cmd_frame_req),
        .i_image_type          (3'b001),
        .i_hdr_mode_en         (i_hdr_mode_en),
        .i_lval_len            (cfg_lval_len_ddr),
        .i_win_row_len         (cfg_win_row_len),
        .i_frame_num           (cfg_frame_num),
        .i_dat_vld             (i_wide_vld),
        .i_dat                 (i_wide_dat),
        .i_ddr_rd_req          (ddr_rd_req),
        .o_dbg_rd_req_ui       (o_dbg_rd_req_ui),
        .o_dat_vld             (ddr_dat_vld),
        .o_dat                 (ddr_dat),
        .o_ddr_wr_done         (ddr_wr_done),
        .clk_ui                (clk_ui),
        .rst_ui                (rst_ui),
        .o_init_calib_complete (init_calib_complete),
        .o_DDR3_ADDR           (o_DDR3_ADDR),
        .o_DDR3_BA             (o_DDR3_BA),
        .o_DDR3_CAS_N          (o_DDR3_CAS_N),
        .o_DDR3_CK_N           (o_DDR3_CK_N),
        .o_DDR3_CK_P           (o_DDR3_CK_P),
        .o_DDR3_CKE            (o_DDR3_CKE),
        .o_DDR3_RAS_N          (o_DDR3_RAS_N),
        .o_DDR3_RESET_N        (o_DDR3_RESET_N),
        .o_DDR3_WE_N           (o_DDR3_WE_N),
        .io_DDR3_DQ            (io_DDR3_DQ),
        .io_DDR3_DQS_N         (io_DDR3_DQS_N),
        .io_DDR3_DQS_P         (io_DDR3_DQS_P),
        .o_DDR3_CS_N           (o_DDR3_CS_N),
        .o_DDR3_DM             (o_DDR3_DM),
        .o_DDR3_ODT            (o_DDR3_ODT)
    );

    ddr_dout_buf_0 u_ddr_dout_buf_0 (
        .clk_wr          (clk_ui),
        .rst_wr          (rst_ui),
        .clk_rd          (clk_sys),
        .rst_rd          (rst_sys),
        .i_cmd_frame_req (i_cmd_frame_req),
        .i_lval_len_wr   (cfg_lval_len_ddr[9:0]),
        .i_din_vld       (ddr_dat_vld),
        .i_din           (pad_32x12_to_32x16(ddr_dat)),
        .o_din_req       (ddr_rd_req),
        .i_dout_rd_en    (rowbuf_rd_en),
        .o_dout_vld      (rowbuf_vld),
        .o_dout          (rowbuf_dout),
        .o_dout_rdy      (rowbuf_rdy)
    );

    image_dat_tx_0 u_image_dat_tx_0 (
        .clk              (clk_sys),
        .rst              (rst_sys),
        .i_cmd_frame_req  (i_cmd_frame_req),
        // Official imageprocess_top doubles lval_len after the 32->16
        // channel DDR output buffer and multiplies sensor width by four.
        .i_lval_len       (cfg_lval_len_tx),
        .i_win_row_len    (cfg_win_row_len),
        .i_win_col_len    (cfg_win_col_tx),
        .i_frame_num_tx   (cfg_frame_num_tx),
        .i_tx_fifo_cnt    (i_tx_fifo_cnt),
        .i_din_rdy        (rowbuf_rdy),
        .i_din_vld        (rowbuf_vld),
        .i_din            (rowbuf_dout),
        .o_din_rd_en      (rowbuf_rd_en),
        .o_dout_vld       (o_img_dat_vld),
        .o_dout           (o_img_dat),
        .o_tx_row_done    (tx_row_done),
        .o_img2tx_tx_en   (o_img_tx_en),
        .o_frame_done     (o_frame_done)
    );

    reg [31:0] wide_in_total;
    reg [31:0] ddr_out_total;
    reg [31:0] usb_word_total;
    reg [31:0] rd_req_total;
    reg        ddr_rd_req_d;

    assign o_dbg0 = wide_in_total;
    assign o_dbg1 = ddr_out_total;

    assign o_init_calib_complete = init_calib_complete;

    always @(posedge clk_img) begin
        if (rst_img || i_cmd_frame_req)
            wide_in_total <= 32'd0;
        else if (i_wide_vld)
            wide_in_total <= wide_in_total + 32'd1;
    end

    always @(posedge clk_ui) begin
        if (rst_ui) begin
            ddr_out_total <= 32'd0;
            rd_req_total  <= 32'd0;
            ddr_rd_req_d  <= 1'b0;
        end else begin
            ddr_rd_req_d <= ddr_rd_req;
            if (ddr_dat_vld)
                ddr_out_total <= ddr_out_total + 32'd1;
            if (ddr_rd_req && !ddr_rd_req_d)
                rd_req_total <= rd_req_total + 32'd1;
        end
    end

    always @(posedge clk_sys) begin
        if (rst_sys || i_cmd_frame_req)
            usb_word_total <= 32'd0;
        else if (o_img_dat_vld)
            usb_word_total <= usb_word_total + 32'd1;
    end

endmodule
