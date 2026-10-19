`timescale 1ns/1ps

// 8-N-1 UART transmitter with a one-byte ready/valid interface.
module g6504_uart_tx #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 921_600
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       data_valid,
    input  wire [7:0] data_in,
    output wire       data_ready,
    output reg        uart_tx
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD/2) / BAUD;
    localparam [1:0] ST_IDLE = 2'd0, ST_START = 2'd1,
                     ST_DATA = 2'd2, ST_STOP = 2'd3;
    reg [1:0] state;
    reg [31:0] bit_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift_reg;

    assign data_ready = (state == ST_IDLE);

    always @(posedge clk) begin
        if (rst) begin
            state     <= ST_IDLE;
            bit_cnt   <= 32'd0;
            bit_idx   <= 3'd0;
            shift_reg <= 8'd0;
            uart_tx   <= 1'b1;
        end else begin
            case (state)
                ST_IDLE: begin
                    uart_tx <= 1'b1;
                    bit_cnt <= 32'd0;
                    if (data_valid) begin
                        shift_reg <= data_in;
                        bit_idx   <= 3'd0;
                        bit_cnt   <= (CLKS_PER_BIT > 0) ? CLKS_PER_BIT - 1 : 0;
                        uart_tx   <= 1'b0;
                        state     <= ST_START;
                    end
                end
                ST_START: begin
                    uart_tx <= 1'b0;
                    if (bit_cnt == 0) begin
                        bit_cnt <= (CLKS_PER_BIT > 0) ? CLKS_PER_BIT - 1 : 0;
                        uart_tx <= shift_reg[0];
                        state   <= ST_DATA;
                    end else bit_cnt <= bit_cnt - 1'b1;
                end
                ST_DATA: begin
                    uart_tx <= shift_reg[bit_idx];
                    if (bit_cnt == 0) begin
                        bit_cnt <= (CLKS_PER_BIT > 0) ? CLKS_PER_BIT - 1 : 0;
                        if (bit_idx == 3'd7) begin
                            uart_tx <= 1'b1;
                            state   <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else bit_cnt <= bit_cnt - 1'b1;
                end
                ST_STOP: begin
                    uart_tx <= 1'b1;
                    if (bit_cnt == 0) state <= ST_IDLE;
                    else bit_cnt <= bit_cnt - 1'b1;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
