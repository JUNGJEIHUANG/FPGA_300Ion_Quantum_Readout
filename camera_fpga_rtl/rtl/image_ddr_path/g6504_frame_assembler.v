`timescale 1ns/1ps

// GSENSE6504 first-pass frame assembler / sync detector.
// This version is intentionally lightweight for on-board validation:
//   1) detect four-word sync codes FFF,000,000,{800,9D0,AB0,B60};
//   2) pass both sensor sync codes and pixel payload to DDR/USB;
//   3) pack 16 lanes x 12-bit into eight 32-bit words, two pixels per word.
// Lane mapping is kept unchanged here:
//   lane[0..7]  = BOT_CHANNEL_OUT<0..7>
//   lane[8..15] = TOP_CHANNEL_OUT<0..7>
// PC software can use the documented STD-mode order to do final 2176x2100 reshape.
module g6504_frame_assembler #(
    parameter integer LANES      = 16,
    parameter integer DAT_WIDTH  = 12,
    parameter integer SEGMENTS   = 8,
    parameter integer SEG_WIDTH  = 272,
    parameter integer IMG_WIDTH  = 2176,
    parameter integer BOT_BASE   = 0,
    parameter integer TOP_BASE   = 8,
    parameter         BIG_END_EN = 1'b0
)(
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         i_gate,
    input  wire [LANES-1:0]             i_dat_vld,
    input  wire [LANES*DAT_WIDTH-1:0]   i_dat,

    output reg                          o_wr_en,
    output reg  [31:0]                  o_dout,
    output wire                         o_busy,
    output reg                          o_frame_active,
    output reg                          o_line_active,
    output reg                          o_frame_done,
    output reg  [12:0]                  o_x,
    output reg  [12:0]                  o_y,
    output reg  [3:0]                   o_phase,
    output reg  [15:0]                  o_status
);

localparam [11:0] SYNC_W0  = 12'hFFF;
localparam [11:0] SYNC_W1  = 12'h000;
localparam [11:0] SYNC_W2  = 12'h000;
localparam [11:0] CODE_SOF = 12'h800;
localparam [11:0] CODE_EOF = 12'h9D0;
localparam [11:0] CODE_SOL = 12'hAB0;
localparam [11:0] CODE_EOL = 12'hB60;

reg [LANES*DAT_WIDTH-1:0] pipe0;
reg [LANES*DAT_WIDTH-1:0] pipe1;
reg [LANES*DAT_WIDTH-1:0] pipe2;
reg [1:0]                 pipe_count;
reg [LANES*DAT_WIDTH-1:0] dat_hold;
reg [LANES*DAT_WIDTH-1:0] sync_hold1;
reg [LANES*DAT_WIDTH-1:0] sync_hold2;
reg [LANES*DAT_WIDTH-1:0] sync_hold3;
reg [1:0]                 sync_emit_idx;
reg [1:0]                 sync_emit_left;
reg [3:0]                 pair_idx;
reg                       busy;
reg [12:0]                pix_x;
reg [12:0]                pix_y;

assign o_busy = busy;

function [11:0] bit_order;
    input [11:0] din;
    integer b;
    begin
        if (BIG_END_EN) begin
            for (b = 0; b < 12; b = b + 1)
                bit_order[b] = din[11-b];
        end else begin
            bit_order = din;
        end
    end
endfunction

function [11:0] lane_word;
    input [LANES*DAT_WIDTH-1:0] beat;
    input integer lane;
    begin
        lane_word = bit_order(beat[lane*DAT_WIDTH +: DAT_WIDTH]);
    end
endfunction

wire valid_all = i_gate && (&i_dat_vld);
wire [11:0] ref0 = lane_word(pipe0, 0);
wire [11:0] ref1 = lane_word(pipe1, 0);
wire [11:0] ref2 = lane_word(pipe2, 0);
wire [11:0] refc = lane_word(i_dat, 0);
wire is_code = (refc == CODE_SOF) || (refc == CODE_EOF) || (refc == CODE_SOL) || (refc == CODE_EOL);
wire sync_now = (pipe_count == 2'd3) &&
                (ref0 == SYNC_W0) && (ref1 == SYNC_W1) && (ref2 == SYNC_W2) && is_code;

always @(posedge clk) begin
    if (rst || !i_gate) begin
        pipe0          <= {LANES*DAT_WIDTH{1'b0}};
        pipe1          <= {LANES*DAT_WIDTH{1'b0}};
        pipe2          <= {LANES*DAT_WIDTH{1'b0}};
        pipe_count     <= 2'd0;
        dat_hold       <= {LANES*DAT_WIDTH{1'b0}};
        sync_hold1     <= {LANES*DAT_WIDTH{1'b0}};
        sync_hold2     <= {LANES*DAT_WIDTH{1'b0}};
        sync_hold3     <= {LANES*DAT_WIDTH{1'b0}};
        sync_emit_idx  <= 2'd0;
        sync_emit_left <= 2'd0;
        pair_idx       <= 4'd0;
        busy           <= 1'b0;
        pix_x          <= 13'd0;
        pix_y          <= 13'd0;
        o_wr_en        <= 1'b0;
        o_dout         <= 32'd0;
        o_frame_active <= 1'b0;
        o_line_active  <= 1'b0;
        o_frame_done   <= 1'b0;
        o_x            <= 13'd0;
        o_y            <= 13'd0;
        o_phase        <= 4'd0;
        o_status       <= 16'd0;
    end else begin
        o_wr_en      <= 1'b0;
        o_frame_done <= 1'b0;

        if (busy) begin
            o_dout[15:0]  <= {4'b0, lane_word(dat_hold, pair_idx*2)};
            o_dout[31:16] <= {4'b0, lane_word(dat_hold, pair_idx*2 + 1)};
            o_wr_en       <= 1'b1;
            o_x           <= pix_x;
            o_y           <= pix_y;

            if (pair_idx == (LANES/2)-1) begin
                pair_idx <= 4'd0;
                if (sync_emit_left != 2'd0) begin
                    case (sync_emit_idx)
                        2'd1: dat_hold <= sync_hold1;
                        2'd2: dat_hold <= sync_hold2;
                        default: dat_hold <= sync_hold3;
                    endcase
                    sync_emit_idx  <= sync_emit_idx + 2'd1;
                    sync_emit_left <= sync_emit_left - 2'd1;
                    busy           <= 1'b1;
                end else begin
                    busy <= 1'b0;
                end
                if (pix_x >= IMG_WIDTH-1) begin
                    pix_x <= 13'd0;
                    pix_y <= pix_y + 13'd1;
                end else begin
                    pix_x <= pix_x + LANES[12:0];
                end
            end else begin
                pair_idx <= pair_idx + 4'd1;
            end
        end else if (valid_all) begin
            if (sync_now) begin
                case (refc)
                    CODE_SOF: begin
                        o_frame_active <= 1'b1;
                        o_line_active  <= 1'b0;
                        pix_x          <= 13'd0;
                        pix_y          <= 13'd0;
                        o_phase        <= 4'd0;
                        o_status[4]    <= 1'b1;
                    end
                    CODE_SOL: begin
                        o_line_active <= 1'b1;
                        pix_x         <= 13'd0;
                        o_phase       <= 4'd0;
                        o_status[5]   <= 1'b1;
                    end
                    CODE_EOL: begin
                        o_line_active <= 1'b0;
                        pix_x         <= 13'd0;
                        pix_y         <= pix_y + 13'd1;
                        o_phase       <= 4'd0;
                        o_status[6]   <= 1'b1;
                    end
                    CODE_EOF: begin
                        o_frame_active <= 1'b0;
                        o_line_active  <= 1'b0;
                        o_frame_done   <= 1'b1;
                        o_status[7]    <= 1'b1;
                    end
                    default: begin
                    end
                endcase
            end

            // SOF is the write trigger.  Before SOF, discard training/idle words.
            // Once in frame, pass payload and the complete four-beat sync sequence
            // FFF/000/000/{SOF,EOF,SOL,EOL} downstream.
            if (sync_now && ((refc == CODE_SOF) || o_frame_active)) begin
                dat_hold       <= pipe0;
                sync_hold1     <= pipe1;
                sync_hold2     <= pipe2;
                sync_hold3     <= i_dat;
                sync_emit_idx  <= 2'd1;
                sync_emit_left <= 2'd3;
                busy           <= 1'b1;
                pair_idx       <= 4'd0;
            end else if (o_frame_active) begin
                dat_hold <= i_dat;
                busy     <= 1'b1;
                pair_idx <= 4'd0;
            end

            if (pipe_count == 2'd3) begin
                pipe0 <= pipe1;
                pipe1 <= pipe2;
                pipe2 <= i_dat;
            end else begin
                if (pipe_count == 2'd0) begin
                    pipe0 <= i_dat;
                end else if (pipe_count == 2'd1) begin
                    pipe1 <= i_dat;
                end else begin
                    pipe2 <= i_dat;
                end
                pipe_count <= pipe_count + 2'd1;
            end
        end
    end
end

endmodule
