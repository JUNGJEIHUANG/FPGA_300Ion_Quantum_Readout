`timescale 1ns/1ps

// Simple DDR-backed chunk buffer for the current G6504 image stream.
// Data path:
//   img_cdc_fifo 32b stream (MIG ui_clk domain)
//     -> pack 16 x 32b to 512b
//     -> sequential MIG write
//     -> pipelined MIG read after one chunk is written
//     -> 512b async FIFO to clk_sys/USB domain
//     -> serialize 512b to 32b on the clk_sys side
//
// Read-path rework (2026-07-02): a hardware ILA/PC capture showed the
// original design -- issue ONE read command, then sit in a dedicated state
// waiting for that command's own app_rd_data_valid before issuing the next
// -- can hang indefinitely (app_rd_data_valid simply never arrives for a
// stuck command, with nothing else in flight to keep the pipe alive).
// K7_Demo's mig_driver.v (known-working reference, same mig_k7_mem_bar_512b
// core) never does this: it pipelines read commands back-to-back as fast as
// app_rdy allows, and treats app_rd_data_valid as a free-running consumer
// signal completely decoupled from command issuance. This module now mirrors
// that: S_READ_CMD issues commands in a tight loop, and a separate always
// block captures every app_rd_data_valid beat into a 512-bit-wide async
// FIFO whenever it appears, regardless of FSM state.
//
// This is intentionally simpler than K7_Demo imageprocess_top/mig_driver:
// no HDR, no row jump, no averaging, no multi-frame reorder.
module ddr_simple_frame_buffer #(
    parameter integer FRAME_BEATS_512  = 20'd65536,  // 65536 * 64B = 4 MiB per captured chunk
    parameter integer WRITE_IDLE_TIMEOUT = 24'd1000000, // ui_clk cycles: empty gap that closes current frame
    parameter integer TX_ALMOST_FULL   = 15'd14000
)(
    input  wire        clk_sys,
    input  wire        rst_sys,
    input  wire        clk_img,
    input  wire        rst_img,
    input  wire        clk_200m,
    input  wire        i_cmd_frame_req,

    // K7-style wide image stream source; buffered before MIG and written as
    // one 384-bit image beat per 512-bit DDR beat, matching K7_Demo's
    // image_ddr_driver convention of zero-padding to MIG width.
    input  wire        i_wide_vld,
    input  wire [383:0] i_wide_dat,

    // Legacy 32-bit stream source; kept for port compatibility/debug only in
    // the K7-style path.
    input  wire        i_in_empty,
    input  wire [31:0] i_in_dout,
    output reg         o_in_rd_en,

    // USB TX FIFO backpressure, clk_sys domain.
    input  wire [14:0] i_tx_fifo_cnt,
    output reg         o_img_dat_vld,
    output reg  [31:0] o_img_dat,
    output wire        o_img_tx_en,

    // MIG user clock/reset exported so upstream FIFO can use ui_clk as rd_clk.
    output wire        o_ui_clk,
    output wire        o_ui_rst,
    output wire        o_init_calib_complete,

    // DDR3 physical interface, same naming/width as K7_Demo.
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
    output wire        o_DDR3_ODT,
    output wire [15:0] o_dbg,

    // ILA debug signals in ui_clk domain.
    output wire [2:0]  o_ila_state,
    output wire        o_ila_init_calib_complete,
    output wire        o_ila_ui_rst,
    output wire        o_ila_app_en,
    output wire        o_ila_app_rdy,
    output wire [2:0]  o_ila_app_cmd,
    output wire [29:0] o_ila_app_addr,
    output wire        o_ila_app_wdf_wren,
    output wire        o_ila_app_wdf_rdy,
    output wire        o_ila_app_wdf_end,
    output wire [31:0] o_ila_app_wdf_data_lo,
    output wire        o_ila_app_rd_data_valid,
    output wire        o_ila_app_rd_data_end,
    output wire [31:0] o_ila_app_rd_data_lo,
    output wire        o_ila_i_in_empty,
    output wire        o_ila_o_in_rd_en,
    output wire        o_ila_out_fifo_wr_en,
    output wire        o_ila_out_fifo_full,
    output wire        o_ila_out_fifo_empty,
    output wire        o_ila_out_fifo_almost_full,
    output wire        o_ila_out_fifo_rd_en,
    output wire [14:0] o_ila_tx_fifo_cnt,
    output wire [19:0] o_ila_wr_beat_cnt,
    output wire [19:0] o_ila_rd_cmd_cnt,
    output wire [19:0] o_ila_rd_data_cnt
);

localparam [2:0] CMD_WRITE = 3'd0;
localparam [2:0] CMD_READ  = 3'd1;

localparam [1:0] S_WAIT_CAL  = 2'd0;
localparam [1:0] S_WRITE     = 2'd1;
localparam [1:0] S_WR_ISSUE  = 2'd2;  // command + write-data fired together
localparam [1:0] S_READ_CMD  = 2'd3;  // pipelined read-command issue
localparam [25:0] IDLE_CLOSE_CYCLES = 26'd50000000;

function [511:0] pack_wide12_to_usb512;
    input [383:0] din;
    integer i;
    begin
        pack_wide12_to_usb512 = 512'd0;
        for (i = 0; i < 16; i = i + 1) begin
            pack_wide12_to_usb512[i*32 +: 32] = {
                4'b0000, din[(2*i+1)*12 +: 12],
                4'b0000, din[(2*i  )*12 +: 12]
            };
        end
    end
endfunction

wire        ui_clk;
wire        ui_rst;
wire        init_calib_complete;
wire        app_rdy;
wire        app_wdf_rdy;
reg  [29:0] app_addr;
reg  [2:0]  app_cmd;
reg         app_en;
reg  [511:0] app_wdf_data;
reg         app_wdf_wren;
reg         app_wdf_end;
wire [511:0] app_rd_data;
wire        app_rd_data_valid;
wire        app_rd_data_end;

assign o_ui_clk = ui_clk;
assign o_ui_rst = ui_rst;
assign o_init_calib_complete = init_calib_complete;

reg [1:0]   state;
reg [3:0]   pack_cnt;
reg [511:0] pack_data;
reg         rd_en_d1;
reg         fifo2ddr_rd_en;
wire        fifo2ddr_empty;
wire [383:0] fifo2ddr_dout;
wire [11:0] fifo2ddr_rd_cnt;
reg [19:0]  wr_beat_cnt;
reg [27:0]  wr_addr;
reg [27:0]  rd_addr;
reg [19:0]  read_target_cnt; // actual completed writes in the current frame/chunk
reg [19:0]  rd_cmd_cnt;   // read commands issued this chunk
reg [19:0]  rd_data_cnt;  // read data beats actually captured this chunk
reg         wr_cmd_done;  // S_WR_ISSUE: this beat's command already accepted
reg         wr_data_done; // S_WR_ISSUE: this beat's write-data already accepted
reg [25:0]  write_empty_cnt;
reg         rd_data_cnt_clr;
reg [4:0]   frame_rst_img_cnt;
reg         frame_rst_img;
reg [2:0]   frame_rst_ui_sync;
wire        frame_rst_ui = frame_rst_ui_sync[2];

wire        out_fifo_full;
wire        out_fifo_empty;
wire        out_fifo_almost_full;
wire [511:0] out_fifo_dout;
wire        out_fifo_wr_en = app_rd_data_valid && !out_fifo_full;
wire        out_fifo_rd_en;

assign o_ila_state               = {1'b0, state};
assign o_ila_init_calib_complete = init_calib_complete;
assign o_ila_ui_rst              = ui_rst;
assign o_ila_app_en              = app_en;
assign o_ila_app_rdy             = app_rdy;
assign o_ila_app_cmd             = app_cmd;
assign o_ila_app_addr            = app_addr;
assign o_ila_app_wdf_wren        = app_wdf_wren;
assign o_ila_app_wdf_rdy         = app_wdf_rdy;
assign o_ila_app_wdf_end         = app_wdf_end;
assign o_ila_app_wdf_data_lo     = app_wdf_data[31:0];
assign o_ila_app_rd_data_valid   = app_rd_data_valid;
assign o_ila_app_rd_data_end     = app_rd_data_end;
assign o_ila_app_rd_data_lo      = app_rd_data[31:0];
assign o_ila_i_in_empty          = fifo2ddr_empty;
assign o_ila_o_in_rd_en          = fifo2ddr_rd_en;
assign o_ila_out_fifo_wr_en      = out_fifo_wr_en;
assign o_ila_out_fifo_full       = out_fifo_full;
assign o_ila_out_fifo_empty      = out_fifo_empty;
assign o_ila_out_fifo_almost_full= out_fifo_almost_full;
assign o_ila_out_fifo_rd_en      = out_fifo_rd_en;
assign o_ila_tx_fifo_cnt         = i_tx_fifo_cnt;
assign o_ila_wr_beat_cnt         = wr_beat_cnt;
assign o_ila_rd_cmd_cnt          = rd_cmd_cnt;
assign o_ila_rd_data_cnt         = rd_data_cnt;

assign o_dbg = {
    state,               // [15:14]
    app_rdy,             // [13]
    app_wdf_rdy,         // [12]
    app_en,              // [11]
    app_wdf_wren,        // [10]
    app_rd_data_valid,   // [9]
    out_fifo_empty,      // [8]
    out_fifo_full,       // [7]
    out_fifo_almost_full,// [6]
    fifo2ddr_empty,      // [5]
    fifo2ddr_rd_en,      // [4]
    pack_cnt[3:0]        // [3:0]
};

// K7_Demo-compatible per-frame reset.  K7 resets the image-to-DDR FIFO and
// MIG frame-control FSM for ~16 clocks after i_cmd_frame_req.  Keep the same
// contract here so each requested frame starts from DDR address 0 and no
// previous-frame residue remains in the input FIFO.
always @(posedge clk_img) begin
    if (rst_img) begin
        frame_rst_img_cnt <= 5'd0;
        frame_rst_img     <= 1'b0;
    end else if (i_cmd_frame_req) begin
        frame_rst_img_cnt <= 5'd0;
        frame_rst_img     <= 1'b1;
    end else if (frame_rst_img) begin
        if (frame_rst_img_cnt >= 5'd15) begin
            frame_rst_img <= 1'b0;
        end else begin
            frame_rst_img_cnt <= frame_rst_img_cnt + 5'd1;
        end
    end
end

always @(posedge ui_clk) begin
    if (ui_rst) begin
        frame_rst_ui_sync <= 3'b111;
    end else begin
        frame_rst_ui_sync <= {frame_rst_ui_sync[1:0], frame_rst_img};
    end
end

// K7_Demo reference: lvds_imagebuffer 384-bit output crosses into MIG UI
// clock through a 384-bit independent-clock FIFO, then each beat is written
// to the 512-bit MIG port with the upper 128 bits zero-padded.
xpm_fifo_async #(
    .CASCADE_HEIGHT      (0),
    .CDC_SYNC_STAGES     (2),
    .DOUT_RESET_VALUE    ("0"),
    .ECC_MODE            ("no_ecc"),
    .FIFO_MEMORY_TYPE    ("auto"),
    .FIFO_READ_LATENCY   (0),
    .FIFO_WRITE_DEPTH    (2048),
    .FULL_RESET_VALUE    (0),
    .PROG_EMPTY_THRESH   (10),
    .PROG_FULL_THRESH    (1800),
    .RD_DATA_COUNT_WIDTH (12),
    .READ_DATA_WIDTH     (384),
    .READ_MODE           ("fwft"),
    .RELATED_CLOCKS      (0),
    .SIM_ASSERT_CHK      (0),
    .USE_ADV_FEATURES    ("0000"),
    .WAKEUP_TIME         (0),
    .WRITE_DATA_WIDTH    (384),
    .WR_DATA_COUNT_WIDTH (12)
) u_img_to_ddr_fifo (
    .almost_empty  (),
    .almost_full   (),
    .data_valid    (),
    .dbiterr       (),
    .dout          (fifo2ddr_dout),
    .empty         (fifo2ddr_empty),
    .full          (),
    .overflow      (),
    .prog_empty    (),
    .prog_full     (),
    .rd_data_count (fifo2ddr_rd_cnt),
    .rd_rst_busy   (),
    .sbiterr       (),
    .underflow     (),
    .wr_ack        (),
    .wr_data_count (),
    .wr_rst_busy   (),
    .din           (i_wide_dat),
    .injectdbiterr (1'b0),
    .injectsbiterr (1'b0),
    .rd_clk        (ui_clk),
    .rd_en         (fifo2ddr_rd_en),
    .rst           (rst_sys | rst_img | ui_rst | frame_rst_img),
    .sleep         (1'b0),
    .wr_clk        (clk_img),
    .wr_en         (i_wide_vld)
);

// --- Serialize 512b DDR-read beats back to the 32b USB stream ---
reg  [383:0] ser_data;
reg  [3:0]   ser_cnt;
reg          ser_busy;
wire         ser_can_pop = !out_fifo_empty && !ser_busy && (i_tx_fifo_cnt < TX_ALMOST_FULL);

assign out_fifo_rd_en = ser_can_pop;
assign o_img_tx_en    = !out_fifo_empty || ser_busy;

always @(posedge clk_sys) begin
    if (rst_sys) begin
        o_img_dat_vld <= 1'b0;
        o_img_dat     <= 32'd0;
            ser_data      <= 384'd0;
        ser_cnt       <= 4'd0;
        ser_busy      <= 1'b0;
    end else begin
        o_img_dat_vld <= 1'b0;
        if (ser_can_pop) begin
            ser_data <= out_fifo_dout[383:0];
            ser_cnt  <= 4'd0;
            ser_busy <= 1'b1;
        end else if (ser_busy) begin
            o_img_dat_vld <= 1'b1;
            o_img_dat     <= {4'b0, ser_data[23:12], 4'b0, ser_data[11:0]};
            ser_data      <= {24'd0, ser_data[383:24]};
            if (ser_cnt == 4'd15) begin
                ser_busy <= 1'b0;
            end
            ser_cnt <= ser_cnt + 4'd1;
        end
    end
end

// Cross DDR-read 512b beats back to the USB clock domain. One MIG read
// beat = one FIFO word, written the instant app_rd_data_valid pulses --
// no per-command wait, no unpack state machine on the write side.
xpm_fifo_async #(
    .CASCADE_HEIGHT      (0),
    .CDC_SYNC_STAGES     (2),
    .DOUT_RESET_VALUE    ("0"),
    .ECC_MODE            ("no_ecc"),
    .FIFO_MEMORY_TYPE    ("auto"),
    .FIFO_READ_LATENCY   (0),
    .FIFO_WRITE_DEPTH    (512),
    .FULL_RESET_VALUE    (0),
    .PROG_EMPTY_THRESH   (10),
    .PROG_FULL_THRESH    (448),
    .RD_DATA_COUNT_WIDTH (10),
    .READ_DATA_WIDTH     (512),
    .READ_MODE           ("fwft"),
    .RELATED_CLOCKS      (0),
    .SIM_ASSERT_CHK      (0),
    .USE_ADV_FEATURES    ("0002"),
    .WAKEUP_TIME         (0),
    .WRITE_DATA_WIDTH    (512),
    .WR_DATA_COUNT_WIDTH (10)
) u_ddr_to_usb_fifo (
    .almost_empty  (),
    .almost_full   (out_fifo_almost_full),
    .data_valid    (),
    .dbiterr       (),
    .dout          (out_fifo_dout),
    .empty         (out_fifo_empty),
    .full          (out_fifo_full),
    .overflow      (),
    .prog_empty    (),
    .prog_full     (),
    .rd_data_count (),
    .rd_rst_busy   (),
    .sbiterr       (),
    .underflow     (),
    .wr_ack        (),
    .wr_data_count (),
    .wr_rst_busy   (),
    .din           (app_rd_data),
    .injectdbiterr (1'b0),
    .injectsbiterr (1'b0),
    .rd_clk        (clk_sys),
    .rd_en         (out_fifo_rd_en),
    .rst           (rst_sys | ui_rst),
    .sleep         (1'b0),
    .wr_clk        (ui_clk),
    .wr_en         (out_fifo_wr_en)
);

// Free-running read-data capture: independent of `state`, mirrors
// K7_Demo's image_ddr_driver.v wiring app_rd_data_valid straight to a
// downstream consumer with no FSM gating. Counts beats actually received
// so the command-issue side (S_READ_CMD) knows when a chunk is truly done,
// as opposed to merely "commands issued" which can outrun data return.
always @(posedge ui_clk) begin
    if (ui_rst) begin
        rd_data_cnt <= 20'd0;
    end else if (state == S_WAIT_CAL || rd_data_cnt_clr) begin
        rd_data_cnt <= 20'd0;
    end else if (app_rd_data_valid && !out_fifo_full) begin
        rd_data_cnt <= rd_data_cnt + 20'd1;
    end
end

// MIG-side write/read command FSM.
// Write: command (app_en/app_rdy) and write-data (app_wdf_wren/app_wdf_rdy)
// are fired together in S_WR_ISSUE and require BOTH ready signals high in
// the same cycle, matching K7_Demo's mig_driver.v combinational write-fire
// pattern instead of splitting them into two sequential states.
// Read: S_READ_CMD pipelines commands back-to-back as fast as app_rdy
// allows (K7_Demo's s_rd pattern) instead of waiting for each command's
// own data before issuing the next.
always @(posedge ui_clk) begin
    if (ui_rst || frame_rst_ui) begin
        state          <= S_WAIT_CAL;
        o_in_rd_en     <= 1'b0;
        rd_en_d1       <= 1'b0;
        fifo2ddr_rd_en <= 1'b0;
        pack_cnt       <= 4'd0;
        pack_data      <= 512'd0;
        wr_beat_cnt    <= 20'd0;
        wr_addr        <= 28'd0;
        rd_addr        <= 28'd0;
        read_target_cnt<= 20'd0;
        rd_cmd_cnt     <= 20'd0;
        wr_cmd_done    <= 1'b0;
        wr_data_done   <= 1'b0;
        write_empty_cnt<= 26'd0;
        rd_data_cnt_clr<= 1'b0;
        app_addr       <= 30'd0;
        app_cmd        <= CMD_WRITE;
        app_en         <= 1'b0;
        app_wdf_data   <= 512'd0;
        app_wdf_wren   <= 1'b0;
        app_wdf_end    <= 1'b0;
    end else begin
        o_in_rd_en     <= 1'b0;
        fifo2ddr_rd_en <= 1'b0;
        app_en         <= 1'b0;
        app_wdf_wren   <= 1'b0;
        app_wdf_end    <= 1'b0;
        rd_en_d1       <= o_in_rd_en;
        rd_data_cnt_clr<= 1'b0;

        case (state)
            S_WAIT_CAL: begin
                pack_cnt    <= 4'd0;
                wr_beat_cnt <= 20'd0;
                wr_addr     <= 28'd0;
                rd_addr     <= 28'd0;
                read_target_cnt <= 20'd0;
                rd_cmd_cnt  <= 20'd0;
                write_empty_cnt <= 26'd0;
                if (init_calib_complete)
                    state <= S_WRITE;
            end

            S_WRITE: begin
                // K7-style wide path: pop one 384-bit lvds_imagebuffer beat
                // and write it as one zero-padded 512-bit MIG beat.  The
                // lvds_imagebuffer/K7 ordering is preserved here; PC-side
                // pixel-format fixes must be done only after the upstream
                // frame length is correct.
                if (!fifo2ddr_empty) begin
                    fifo2ddr_rd_en <= 1'b1;
                    app_wdf_data   <= {128'd0, fifo2ddr_dout};
                    app_addr       <= wr_addr;
                    app_cmd        <= CMD_WRITE;
                    pack_cnt       <= 4'd0;
                    write_empty_cnt<= 26'd0;
                    state          <= S_WR_ISSUE;
                end else if (wr_beat_cnt != 20'd0) begin
                    // If the sensor ROI/test mode produces fewer beats than
                    // the nominal full-frame target, close the frame after a
                    // long image-side idle gap and read out exactly what was
                    // written.  The threshold is far longer than the observed
                    // inter-burst gap (~2.4k clk_220M cycles) but short versus
                    // human-triggered frame spacing, so it will not split a
                    // normal frame.
                    if (write_empty_cnt >= IDLE_CLOSE_CYCLES) begin
                        read_target_cnt <= wr_beat_cnt;
                        rd_addr         <= 28'd0;
                        rd_cmd_cnt      <= 20'd0;
                        rd_data_cnt_clr <= 1'b1;
                        app_addr        <= 30'd0;
                        app_cmd         <= CMD_READ;
                        state           <= S_READ_CMD;
                    end else begin
                        write_empty_cnt <= write_empty_cnt + 26'd1;
                    end
                end
            end

            S_WR_ISSUE: begin
                // Command (app_en/app_rdy) and write-data (app_wdf_wren/
                // app_wdf_rdy) handshakes are independent and can complete
                // on different cycles. Latch each acceptance separately and
                // stop re-asserting that one signal once accepted, so a
                // command that lands before its data (or vice versa) can
                // never be re-issued/re-pushed on a later cycle while still
                // waiting for the other side -- that hazard silently
                // corrupted the write stream (hardware testing 2026-07-02).
                app_en       <= !wr_cmd_done;
                app_wdf_wren <= !wr_data_done;
                app_wdf_end  <= !wr_data_done;
                if (app_rdy)
                    wr_cmd_done <= 1'b1;
                if (app_wdf_rdy)
                    wr_data_done <= 1'b1;
                if ((wr_cmd_done || app_rdy) && (wr_data_done || app_wdf_rdy)) begin
                    wr_cmd_done  <= 1'b0;
                    wr_data_done <= 1'b0;
                    wr_addr     <= wr_addr + 28'd8;
                    wr_beat_cnt <= wr_beat_cnt + 20'd1;
                    if (wr_beat_cnt == FRAME_BEATS_512[19:0] - 20'd1) begin
                        read_target_cnt <= FRAME_BEATS_512[19:0];
                        rd_addr         <= 28'd0;
                        rd_cmd_cnt      <= 20'd0;
                        rd_data_cnt_clr <= 1'b1;
                        app_addr        <= 30'd0;
                        app_cmd         <= CMD_READ;
                        state           <= S_READ_CMD;
                    end else begin
                        state           <= S_WRITE;
                    end
                end
            end

            S_READ_CMD: begin
                // Keep issuing read commands back-to-back while there is
                // buffer room and commands remain, without ever waiting for
                // any individual command's data to return first.
                if (rd_cmd_cnt < read_target_cnt && !out_fifo_almost_full) begin
                    app_addr <= rd_addr;
                    app_cmd  <= CMD_READ;
                    app_en   <= 1'b1;
                    if (app_rdy) begin
                        rd_addr    <= rd_addr + 28'd8;
                        rd_cmd_cnt <= rd_cmd_cnt + 20'd1;
                    end
                end
                // Only start the next chunk's write once every issued
                // command's data has actually been captured.
                if (rd_cmd_cnt >= read_target_cnt && rd_data_cnt >= read_target_cnt) begin
                    wr_addr         <= 28'd0;
                    wr_beat_cnt     <= 20'd0;
                    read_target_cnt <= 20'd0;
                    write_empty_cnt <= 26'd0;
                    pack_cnt        <= 4'd0;
                    state           <= S_WRITE;
                end
            end

            default: state <= S_WAIT_CAL;
        endcase
    end
end

mig_k7_mem_bar_512b u_mig_k7_mem_bar_512b (
    .ddr3_addr           (o_DDR3_ADDR),
    .ddr3_ba             (o_DDR3_BA),
    .ddr3_cas_n          (o_DDR3_CAS_N),
    .ddr3_ck_n           (o_DDR3_CK_N),
    .ddr3_ck_p           (o_DDR3_CK_P),
    .ddr3_cke            (o_DDR3_CKE),
    .ddr3_ras_n          (o_DDR3_RAS_N),
    .ddr3_reset_n        (o_DDR3_RESET_N),
    .ddr3_we_n           (o_DDR3_WE_N),
    .ddr3_dq             (io_DDR3_DQ),
    .ddr3_dqs_n          (io_DDR3_DQS_N),
    .ddr3_dqs_p          (io_DDR3_DQS_P),
    .ddr3_cs_n           (o_DDR3_CS_N),
    .ddr3_dm             (o_DDR3_DM),
    .ddr3_odt            (o_DDR3_ODT),
    .init_calib_complete (init_calib_complete),
    .app_addr            (app_addr),
    .app_cmd             (app_cmd),
    .app_en              (app_en),
    .app_wdf_data        (app_wdf_data),
    .app_wdf_end         (app_wdf_end),
    .app_wdf_wren        (app_wdf_wren),
    .app_rd_data         (app_rd_data),
    .app_rd_data_end     (app_rd_data_end),
    .app_rd_data_valid   (app_rd_data_valid),
    .app_rdy             (app_rdy),
    .app_wdf_rdy         (app_wdf_rdy),
    .app_sr_req          (1'b0),
    .app_ref_req         (1'b0),
    .app_zq_req          (1'b0),
    .app_sr_active       (),
    .app_ref_ack         (),
    .app_zq_ack          (),
    .ui_clk              (ui_clk),
    .ui_clk_sync_rst     (ui_rst),
    .app_wdf_mask        (64'd0),
    .sys_clk_i           (clk_200m),
    .clk_ref_i           (clk_200m),
    .device_temp         (),
    .sys_rst             (rst_sys)
);

endmodule
