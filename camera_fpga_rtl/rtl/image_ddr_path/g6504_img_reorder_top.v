`timescale 1ns/1ps

// g6504_img_reorder_top
// ---------------------------------------------------------------------------
// Ported K7_Demo hardware image-reorder pipeline for the self_developed
// GSENSE6504 bring-up, replacing g6504_beat_gather + g6504_frame_assembler.
//
// Chain:
//   train_dat[191:0]/train_dat_vld[15:0]  (16 lanes x 12b, clk_220M)
//     -> lane swap  (self_developed low8=BOT/high8=TOP  ->  K7 low8=TOP/high8=BOT)
//     -> lval_synccode_top_0  (per-lane sync gate, aligns all 16 lanes into
//                              contiguous back-to-back valid beats)
//     -> lvds_imagebuffer     (chan_mult x2 + chan_top_bot_synrd_out:
//                              the actual lane->raster reorder, 384b out)
//     -> 384->32 serializer   (each 384b beat = 32 px = 16 USB words)
//     -> o_dout[31:0]/o_wr_en  (feeds existing img_cdc_fifo, clk_220M write side)
//
// All logic runs in a single clk_220M domain (clk_rx=clk_tx): the reorder
// BRAMs become same-clock simple-dual-port, avoiding any CDC.
//
// Frame-start marker (2026-07-02): the reordered pixel stream carries no
// sensor sync codes any more (lval_synccode/lvds_imagebuffer consume them
// internally), so a PC receiver blindly reshaping the raw byte stream at a
// fixed row width has no way to find where a real frame actually begins --
// concatenating multiple triggered frames without per-frame boundaries
// causes each frame's rows to land at a different phase, which looks like a
// slow diagonal roll across the assembled image. FRAME_MARK_WORD is a
// sentinel value that can never occur in real pixel data (packed pixel
// words always have zero upper nibbles: {4'b0,px,4'b0,px}), emitted
// FRAME_MARK_LEN times immediately before the first serialized word of the
// first beat following a rising edge of i_cmd_frame_req.
//
// Fixed config (K7 GSENSE6504 STD-mode defaults, sensor_protocol.v):
//   n_mux=8, w_mux=1088, lval_len(in)=1088, lvalout_len(buf)=1088,
//   flip_v=0, flip_h=0, hdr=STD(0), SOL_INCREASE_EN=0 (in the IP),
//   sync codes FFF/000/000/{SOF=800, SOL=AB0}.
module g6504_img_reorder_top #(
    parameter [15:0] WIN_ROW_LEN = 16'd2100,  // frame height (SOL_INCREASE=0 -> minor use)
    parameter [15:0] LVAL_LEN    = 16'd1088,  // valid pixels per line from sensor
    parameter [15:0] LVALOUT_LEN = 16'd1088,  // valid pixels per line into image buffer
    parameter [15:0] FRAME_NUM   = 16'd1
)(
    input  wire         clk,           // clk_220M
    input  wire         rst,           // active high
    input  wire         i_cmd_frame_req,
    input  wire [11:0]  i_synccode_1st,
    input  wire [11:0]  i_synccode_2nd,
    input  wire [11:0]  i_synccode_3rd,
    input  wire [11:0]  i_synccode_4th_sof,
    input  wire [11:0]  i_synccode_4th_sol,
    input  wire [15:0]  i_lval_len,
    input  wire [15:0]  i_lvalout_len,
    input  wire [15:0]  i_win_row_len,
    input  wire [15:0]  i_frame_num,
    input  wire [1:0]   i_hdr_mode_en,
    input  wire [1:0]   i_flip_v_en,
    input  wire         i_flip_h_en,
    input  wire [4:0]   i_n_mux,
    input  wire [15:0]  i_w_mux,
    input  wire [15:0]  i_dat_vld,     // train_dat_vld (lane0..15)
    input  wire [191:0] i_dat,         // train_dat, low8=BOT high8=TOP
    output wire         o_wr_en,       // to img_cdc_fifo
    output wire [31:0]  o_dout,        // two 12b pixels: [15:0]=low px, [31:16]=high px
    output wire         o_busy,        // serializer/FIFO backpressure (debug)
    output wire         o_wide_vld,    // K7-style wide output before local serializer
    output wire [383:0] o_wide_dat,    // 32 pixels x 12b from lvds_imagebuffer
    // Direct pre-lvds_imagebuffer observability: these are the synchronized
    // 16-lane beats consumed by u_lvds_imagebuffer in the same clk domain.
    output wire [191:0] o_lane_fifo_din,
    output wire [15:0] o_lane_fifo_wen, o_lane_fifo_rst,
    output wire         o_sync_vld,
    output wire [191:0] o_sync_dat,
    output wire [31:0]  o_sync_dbg,
    output wire [191:0] o_lval_fifo_dout,
    output wire [63:0]  o_lval_internal_dbg,
    output wire [63:0]  o_lval_lane0_dbg,
    output wire [63:0]  o_lval_lane8_dbg,
    output wire [31:0]  o_imgbuf_dbg,
    output wire [31:0]  o_dbg          // debug: drop/fifo/serializer status
);

    localparam [31:0] FRAME_MARK_WORD = 32'hFFFF_FFFF;  // never a valid packed-pixel word
    localparam integer FRAME_MARK_LEN = 8;               // 8 words = 32 bytes, easy to find

    // Keep the official physical-lane order.  The XDC maps lanes 0..7 to TOP
    // and lanes 8..15 to BOT, exactly as in the working CC2405 project.
    wire [191:0] din_sw = i_dat;
    wire [15:0]  vld_sw = i_dat_vld;
    wire [15:0] cfg_lval_len    = (i_lval_len    != 16'd0) ? i_lval_len    : LVAL_LEN;
    wire [15:0] cfg_lvalout_len = (i_lvalout_len != 16'd0) ? i_lvalout_len : LVALOUT_LEN;
    wire [15:0] cfg_win_row_len = (i_win_row_len != 16'd0) ? i_win_row_len : WIN_ROW_LEN;
    wire [15:0] cfg_frame_num   = (i_frame_num   != 16'd0) ? i_frame_num   : FRAME_NUM;
    wire [4:0]  cfg_n_mux       = (i_n_mux       != 5'd0)  ? i_n_mux       : 5'd8;
    wire [15:0] cfg_w_mux       = (i_w_mux       != 16'd0) ? i_w_mux       : 16'd1088;

    // --- Stage 1: per-lane sync gate -> aligned 16-lane contiguous beats ---
    wire         sync_vld;
    wire [191:0] sync_dat;
    wire [31:0]  sync_dbg;
    wire [191:0] lval_fifo_dout_dbg;
    wire [63:0]  lval_internal_dbg;

    lval_synccode_top #(
        .LVDS_PAIRS           (16),
        .DAT_WIDTH            (12),
        .SOL_INCREASE_EN      (1'b0),
        .FIFO_DEPTH           (8192),
        .FIFO_DATACOUNT_WIDTH (13),
        .CHAN_VLD_1ST         (0),
        .CHAN_VLD_2ND         (8)
    ) u_lval_synccode_top (
        .clk                (clk),
        .rst                (rst),
        .i_cmd_frame_req    (i_cmd_frame_req),
        .i_synccode_1st     ((i_synccode_1st != 12'd0) ? i_synccode_1st : 12'hFFF),
        .i_synccode_2nd     (i_synccode_2nd),
        .i_synccode_3rd     (i_synccode_3rd),
        .i_synccode_4th_sof ((i_synccode_4th_sof != 12'd0) ? i_synccode_4th_sof : 12'h800),
        .i_synccode_4th_sol ((i_synccode_4th_sol != 12'd0) ? i_synccode_4th_sol : 12'hAB0),
        .i_lval_len         (cfg_lval_len),
        .i_win_row_len      (cfg_win_row_len),
        .i_frame_num        (cfg_frame_num),
        .i_dat_vld          (vld_sw),
        .i_dat              (din_sw),
        .o_fifo_din_dbg     (o_lane_fifo_din),
        .o_fifo_wen_dbg     (o_lane_fifo_wen),
        .o_fifo_rst_dbg     (o_lane_fifo_rst),
        .o_dat_vld          (sync_vld),
        .o_dat              (sync_dat),
        .o_dbg              (sync_dbg),
        .o_fifo_dout_dbg    (lval_fifo_dout_dbg),
        .o_internal_dbg     (lval_internal_dbg),
        .o_lane0_dbg        (o_lval_lane0_dbg),
        .o_lane8_dbg        (o_lval_lane8_dbg)
    );

    // --- Stage 2: lane -> raster reorder (K7 lvds_imagebuffer) ---
    wire         img_vld;
    wire [383:0] img_dat;   // 16ch x 2 x 12b = 32 pixels
    wire [31:0]  imgbuf_dbg;

    lvds_imagebuffer #(
        .CHAN_NUM  (16),
        .DAT_WIDTH (12)
    ) u_lvds_imagebuffer (
        .clk_rx          (clk),
        .rst_rx          (rst),
        .clk_tx          (clk),
        .rst_tx          (rst),
        .i_cmd_frame_req (i_cmd_frame_req),
        .i_hdr_mode_en   (i_hdr_mode_en),
        .i_flip_v_en     (i_flip_v_en),
        .i_flip_h_en     (i_flip_h_en),
        .i_n_mux         (cfg_n_mux),
        .i_w_mux         (cfg_w_mux[13:0]),
        .i_lvalout_len   (cfg_lvalout_len),
        .i_din_vld       (sync_vld),
        .i_din           (sync_dat),
        .o_chan_vld      (),
        .o_dout_vld      (img_vld),
        .o_dout          (img_dat),
        .o_dbg           (imgbuf_dbg)
    );

    // --- 384b burst FIFO (absorb per-line burst before serializing) ---
    wire         fifo_full;
    wire         fifo_empty;
    wire [383:0] fifo_dout;
    wire         fifo_rd_en;
    wire         ser_busy;
    wire         fifo_wr_en;
    wire         img_drop;
    reg  [23:0] img_drop_cnt;

    assign o_busy = fifo_full;
    assign fifo_wr_en = img_vld & ~fifo_full;
    assign img_drop   = img_vld &  fifo_full;

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE    ("0"),
        .ECC_MODE            ("no_ecc"),
        .FIFO_MEMORY_TYPE    ("block"),
        .FIFO_READ_LATENCY   (0),
        .FIFO_WRITE_DEPTH    (2048),
        .FULL_RESET_VALUE    (0),
        .PROG_EMPTY_THRESH   (10),
        .PROG_FULL_THRESH    (240),
        .RD_DATA_COUNT_WIDTH (12),
        .READ_DATA_WIDTH     (384),
        .READ_MODE           ("fwft"),
        .SIM_ASSERT_CHK      (0),
        .USE_ADV_FEATURES    ("0000"),
        .WAKEUP_TIME         (0),
        .WRITE_DATA_WIDTH    (384),
        .WR_DATA_COUNT_WIDTH (12)
    ) u_burst_fifo (
        .almost_empty (), .almost_full (), .data_valid (), .dbiterr (),
        .dout         (fifo_dout),
        .empty        (fifo_empty),
        .full         (fifo_full),
        .overflow     (), .prog_empty (), .prog_full (), .rd_data_count (),
        .rd_rst_busy  (), .sbiterr (), .underflow (), .wr_ack (),
        .wr_data_count(), .wr_rst_busy (),
        .din          (img_dat),
        .injectdbiterr(1'b0), .injectsbiterr(1'b0),
        .rd_en        (fifo_rd_en),
        .rst          (rst),
        .sleep        (1'b0),
        .wr_clk       (clk),
        .wr_en        (fifo_wr_en)
    );

    // Count any image-buffer beat that arrives while the local 384b burst
    // FIFO is full.  This is a non-destructive debug counter used to verify
    // whether the self_developed 384b->32b serializer path is losing data.
    always @(posedge clk) begin
        if (rst || i_cmd_frame_req) begin
            img_drop_cnt <= 24'd0;
        end else if (img_drop && (img_drop_cnt != 24'hFF_FFFF)) begin
            img_drop_cnt <= img_drop_cnt + 24'd1;
        end
    end

    // Serializer states are declared before the frame-start logic that reads
    // ser_state.  This avoids declaration-order ambiguity in stricter tools.
    localparam [1:0] S_IDLE = 2'd0, S_MARK = 2'd1, S_SER = 2'd2;
    reg  [1:0]   ser_state;

    // --- Frame-start detection: latch on i_cmd_frame_req rising edge, clear
    //     the moment the marker burst for that frame has been scheduled. ---
    reg cmd_frame_req_d;
    reg frame_start_pending;
    always @(posedge clk) begin
        if (rst) begin
            cmd_frame_req_d    <= 1'b0;
            frame_start_pending<= 1'b0;
        end else begin
            cmd_frame_req_d <= i_cmd_frame_req;
            if (i_cmd_frame_req && !cmd_frame_req_d)
                frame_start_pending <= 1'b1;
            else if (ser_state == S_IDLE && fifo_rd_en && frame_start_pending)
                frame_start_pending <= 1'b0;
        end
    end

    // --- 384 -> 32 serializer: one 384b beat = 32 px = 16 words (2 px/word) ---
    // Word k (k=0..15): {4'b0, px[2k+1], 4'b0, px[2k]}, px i = beat[i*12 +: 12].
    // S_MARK emits FRAME_MARK_LEN sentinel words before the first beat of a
    // new frame; S_SER serializes the already-popped 384b beat as before.
    reg  [383:0] ser_shreg;
    reg  [3:0]   ser_cnt;      // 0..15
    reg  [3:0]   mark_cnt;     // 0..FRAME_MARK_LEN-1
    reg          ser_wr_en_r;
    reg  [31:0]  ser_dout_r;

    assign ser_busy   = (ser_state != S_IDLE);
    // Pull a new beat when idle and FIFO has data.
    assign fifo_rd_en = !fifo_empty && (ser_state == S_IDLE);

    always @(posedge clk) begin
        if (rst) begin
            ser_state   <= S_IDLE;
            ser_cnt     <= 4'd0;
            mark_cnt    <= 4'd0;
            ser_shreg   <= 384'd0;
            ser_wr_en_r <= 1'b0;
            ser_dout_r  <= 32'd0;
        end else begin
            ser_wr_en_r <= 1'b0;
            case (ser_state)
                S_IDLE: begin
                    if (fifo_rd_en) begin
                        ser_shreg <= fifo_dout;
                        if (frame_start_pending) begin
                            mark_cnt  <= 4'd0;
                            ser_state <= S_MARK;
                        end else begin
                            ser_cnt   <= 4'd0;
                            ser_state <= S_SER;
                        end
                    end
                end

                S_MARK: begin
                    ser_dout_r  <= FRAME_MARK_WORD;
                    ser_wr_en_r <= 1'b1;
                    if (mark_cnt == FRAME_MARK_LEN-1) begin
                        ser_cnt   <= 4'd0;
                        ser_state <= S_SER;
                    end else begin
                        mark_cnt <= mark_cnt + 4'd1;
                    end
                end

                S_SER: begin
                    // emit two pixels per cycle from the low end of shreg
                    ser_dout_r  <= {4'b0, ser_shreg[23:12], 4'b0, ser_shreg[11:0]};
                    ser_wr_en_r <= 1'b1;
                    ser_shreg   <= {24'd0, ser_shreg[383:24]};  // shift out 2 px
                    if (ser_cnt == 4'd15) begin
                        ser_state <= S_IDLE;
                    end
                    ser_cnt <= ser_cnt + 4'd1;
                end

                default: ser_state <= S_IDLE;
            endcase
        end
    end

    assign o_wr_en = ser_wr_en_r;
    assign o_dout  = ser_dout_r;
    assign o_wide_vld = img_vld;
    assign o_wide_dat = img_dat;
    assign o_sync_vld = sync_vld;
    assign o_sync_dat = sync_dat;
    assign o_sync_dbg = sync_dbg;
    assign o_lval_fifo_dout = lval_fifo_dout_dbg;
    assign o_lval_internal_dbg = lval_internal_dbg;
    assign o_imgbuf_dbg = imgbuf_dbg;
    assign o_dbg   = {
        img_drop_cnt,  // [31:8]
        img_drop,      // [7]
        img_vld,       // [6]
        fifo_full,     // [5]
        fifo_wr_en,    // [4]
        fifo_rd_en,    // [3]
        fifo_empty,    // [2]
        ser_busy,      // [1]
        ser_wr_en_r    // [0]
    };

endmodule
