`timescale 1ns/1ps

// Re-align per-lane skewed valids into whole 16-lane beats, then buffer bursts
// in a deep FIFO so the frame assembler never loses data.
//
// Why this exists (measured on hardware with ILA, 2026-07-02):
//   1) The 16 lane CDC FIFOs after training run independent read enables, so
//      one sensor beat's valids can arrive split across adjacent clk_220M
//      cycles (e.g. train_dat_vld = 0xEFEF then 0x1010).  The frame assembler
//      requires all 16 valids in the same cycle and silently drops split
//      beats, which also breaks SOF/SOL sync-code detection.
//   2) The assembler needs 8 cycles per beat but line bursts deliver one beat
//      per cycle, so ~7/8 of in-line beats were dropped.  A line is ~144
//      beats followed by ~11 us of idle, so a 512-deep beat FIFO fully
//      absorbs each burst and the assembler drains it between lines.
module g6504_beat_gather #(
    parameter integer LANES      = 16,
    parameter integer DAT_WIDTH  = 12,
    parameter integer FIFO_DEPTH = 512,
    parameter [11:0]  IDLE_WORD  = 12'h86E  // sensor training/idle pattern (regs 0x405/0x406)
)(
    input  wire                       clk,
    input  wire                       rst,
    input  wire [LANES-1:0]           i_dat_vld,
    input  wire [LANES*DAT_WIDTH-1:0] i_dat,
    input  wire                       i_rd_ready,   // downstream can accept a beat this cycle
    output wire [LANES-1:0]           o_dat_vld,
    output wire [LANES*DAT_WIDTH-1:0] o_dat,
    output reg  [15:0]                o_drop_cnt    // beats lost to FIFO overflow (debug)
);

// --- Stage 1: gather skewed lane valids into one whole beat ---
reg  [LANES*DAT_WIDTH-1:0] hold;
reg  [LANES-1:0]           got;
wire [LANES-1:0]           got_now       = got | i_dat_vld;
wire                       beat_complete = &got_now;

wire [LANES*DAT_WIDTH-1:0] beat_mux;
genvar g;
generate
    for (g = 0; g < LANES; g = g + 1) begin : g_beat_mux
        assign beat_mux[g*DAT_WIDTH +: DAT_WIDTH] =
            i_dat_vld[g] ? i_dat[g*DAT_WIDTH +: DAT_WIDTH]
                         : hold[g*DAT_WIDTH +: DAT_WIDTH];
    end
endgenerate

integer k;
always @(posedge clk) begin
    if (rst) begin
        got  <= {LANES{1'b0}};
        hold <= {LANES*DAT_WIDTH{1'b0}};
    end else if (beat_complete) begin
        got <= {LANES{1'b0}};
    end else begin
        got <= got_now;
        for (k = 0; k < LANES; k = k + 1) begin
            if (i_dat_vld[k])
                hold[k*DAT_WIDTH +: DAT_WIDTH] <= i_dat[k*DAT_WIDTH +: DAT_WIDTH];
        end
    end
end

// --- Idle filter: never enqueue sensor idle/training beats ---
// Between frames the sensor streams IDLE_WORD on all lanes at a rate far
// above the assembler's drain rate; without this filter the FIFO sits
// permanently full and randomly drops frame beats (including sync codes).
// Sync codes and real pixels never have all 16 lanes equal to IDLE_WORD.
wire beat_is_idle = &{
    (beat_mux[ 0*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 1*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 2*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 3*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 4*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 5*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 6*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 7*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 8*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[ 9*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[10*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[11*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[12*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[13*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[14*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD),
    (beat_mux[15*DAT_WIDTH +: DAT_WIDTH] == IDLE_WORD)};

// --- Stage 2: burst-absorbing beat FIFO (FWFT) ---
wire                       fifo_full;
wire                       fifo_empty;
wire [LANES*DAT_WIDTH-1:0] fifo_dout;
wire                       fifo_wr_en = beat_complete && !beat_is_idle && !fifo_full;
wire                       fifo_rd_en = i_rd_ready && !fifo_empty;

always @(posedge clk) begin
    if (rst)
        o_drop_cnt <= 16'd0;
    else if (beat_complete && !beat_is_idle && fifo_full)
        o_drop_cnt <= o_drop_cnt + 16'd1;
end

xpm_fifo_sync #(
    .CASCADE_HEIGHT      (0),
    .DOUT_RESET_VALUE    ("0"),
    .ECC_MODE            ("no_ecc"),
    .FIFO_MEMORY_TYPE    ("block"),
    .FIFO_READ_LATENCY   (0),
    .FIFO_WRITE_DEPTH    (FIFO_DEPTH),
    .FULL_RESET_VALUE    (0),
    .PROG_EMPTY_THRESH   (10),
    .PROG_FULL_THRESH    (FIFO_DEPTH-16),
    .RD_DATA_COUNT_WIDTH (10),
    .READ_DATA_WIDTH     (LANES*DAT_WIDTH),
    .READ_MODE           ("fwft"),
    .SIM_ASSERT_CHK      (0),
    .USE_ADV_FEATURES    ("0000"),
    .WAKEUP_TIME         (0),
    .WRITE_DATA_WIDTH    (LANES*DAT_WIDTH),
    .WR_DATA_COUNT_WIDTH (10)
) u_beat_fifo (
    .almost_empty  (),
    .almost_full   (),
    .data_valid    (),
    .dbiterr       (),
    .dout          (fifo_dout),
    .empty         (fifo_empty),
    .full          (fifo_full),
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
    .din           (beat_mux),
    .injectdbiterr (1'b0),
    .injectsbiterr (1'b0),
    .rd_en         (fifo_rd_en),
    .rst           (rst),
    .sleep         (1'b0),
    .wr_clk        (clk),
    .wr_en         (fifo_wr_en)
);

// Present the beat only on the cycle it is consumed so the downstream
// all-lanes-valid sampling and the FIFO advance stay in lock step.
assign o_dat     = fifo_dout;
assign o_dat_vld = {LANES{fifo_rd_en}};

endmodule
