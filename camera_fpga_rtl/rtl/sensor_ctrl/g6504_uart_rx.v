`timescale 1ns/1ps

// 8-N-1 UART receiver. The input is asynchronous to clk and is synchronized
// before the bit-timing state machine samples it.
module g6504_uart_rx #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 921_600
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       uart_rx,
    output reg        byte_valid,
    output reg [7:0]  byte_data
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD/2) / BAUD;
    localparam integer HALF_BIT     = (CLKS_PER_BIT / 2);
    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_START = 2'd1;
    localparam [1:0] ST_DATA  = 2'd2;
    localparam [1:0] ST_STOP  = 2'd3;

    (* ASYNC_REG = "TRUE" *) reg rx_meta;
    (* ASYNC_REG = "TRUE" *) reg rx_sync;
    reg [1:0] state;
    reg [31:0] bit_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift_reg;

    always @(posedge clk) begin
        if (rst) begin
            rx_meta    <= 1'b1;
            rx_sync    <= 1'b1;
            state      <= ST_IDLE;
            bit_cnt    <= 32'd0;
            bit_idx    <= 3'd0;
            shift_reg  <= 8'd0;
            byte_valid <= 1'b0;
            byte_data  <= 8'd0;
        end else begin
            rx_meta    <= uart_rx;
            rx_sync    <= rx_meta;
            byte_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    bit_cnt <= 32'd0;
                    if (!rx_sync) begin
                        state   <= ST_START;
                        bit_cnt <= (HALF_BIT > 0) ? HALF_BIT - 1 : 0;
                    end
                end
                ST_START: begin
                    if (bit_cnt == 0) begin
                        if (!rx_sync) begin
                            state   <= ST_DATA;
                            bit_idx <= 3'd0;
                            bit_cnt <= (CLKS_PER_BIT > 0) ? CLKS_PER_BIT - 1 : 0;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end else begin
                        bit_cnt <= bit_cnt - 1'b1;
                    end
                end
                ST_DATA: begin
                    if (bit_cnt == 0) begin
                        shift_reg[bit_idx] <= rx_sync;
                        bit_cnt <= (CLKS_PER_BIT > 0) ? CLKS_PER_BIT - 1 : 0;
                        if (bit_idx == 3'd7) begin
                            state <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        bit_cnt <= bit_cnt - 1'b1;
                    end
                end
                ST_STOP: begin
                    if (bit_cnt == 0) begin
                        if (rx_sync) begin
                            byte_data  <= shift_reg;
                            byte_valid <= 1'b1;
                        end
                        state <= ST_IDLE;
                    end else begin
                        bit_cnt <= bit_cnt - 1'b1;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
