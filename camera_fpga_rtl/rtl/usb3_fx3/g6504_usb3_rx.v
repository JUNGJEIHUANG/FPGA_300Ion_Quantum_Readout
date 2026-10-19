`timescale 1ns/1ps

// FX3 slave-FIFO receive engine.  This follows K7_Demo/usb30_rx.v: the GPIF
// receive socket is A=3 (A[1:0]=2'b11), control/data are sampled on the
// falling edge of the 100-MHz FX3 clock, and two setup cycles are discarded.
module g6504_usb3_rx (
    input  wire        clk,
    input  wire        rst,
    input  wire        i_FX3_FLAGA,
    input  wire [31:0] i_FX3_DB,
    output reg         o_FX3_SLCS_N,
    output reg         o_FX3_SLRD_N,
    output reg         o_FX3_SLOE_N,
    output wire [1:0]  o_FX3_A,
    output reg         o_active,
    output reg         o_word_valid,
    output reg  [31:0] o_word
);
    reg [9:0]  word_count;
    reg [31:0] data_d;

    assign o_FX3_A = 2'b11;

    always @(negedge clk or posedge rst) begin
        if (rst) begin
            o_FX3_SLCS_N <= 1'b1;
            o_FX3_SLRD_N <= 1'b1;
            o_FX3_SLOE_N <= 1'b1;
            o_active     <= 1'b0;
            o_word_valid <= 1'b0;
            o_word       <= 32'd0;
            word_count   <= 10'd0;
            data_d       <= 32'd0;
        end else begin
            o_word_valid <= 1'b0;
            data_d       <= i_FX3_DB;
            if (!o_active) begin
                if (i_FX3_FLAGA) begin
                    o_active     <= 1'b1;
                    o_FX3_SLCS_N <= 1'b0;
                    o_FX3_SLRD_N <= 1'b0;
                    o_FX3_SLOE_N <= 1'b0;
                    word_count   <= 10'd0;
                end else begin
                    o_FX3_SLCS_N <= 1'b1;
                    o_FX3_SLRD_N <= 1'b1;
                    o_FX3_SLOE_N <= 1'b1;
                end
            end else if (!i_FX3_FLAGA) begin
                o_active     <= 1'b0;
                o_FX3_SLCS_N <= 1'b1;
                o_FX3_SLRD_N <= 1'b1;
                o_FX3_SLOE_N <= 1'b1;
                word_count   <= 10'd0;
            end else begin
                o_FX3_SLCS_N <= 1'b0;
                o_FX3_SLRD_N <= 1'b0;
                o_FX3_SLOE_N <= 1'b0;
                word_count <= word_count + 1'b1;
                if (word_count >= 10'd2) begin
                    o_word       <= data_d;
                    o_word_valid <= 1'b1;
                end
            end
        end
    end
endmodule
