`timescale 1ns/1ps

// UART transport for SENSOR register access.
// Request: A5 5A CMD SEQ ADDR_H ADDR_L DATA CRC8 (8 bytes)
// Reply:   5A A5 SEQ STATUS DATA CRC8 (6 bytes)
module g6504_uart_sensor_uart #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 921_600
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       uart_rx,
    output wire       uart_tx,
    output reg        req_valid,
    output reg        req_write,
    output reg [14:0] req_addr,
    output reg [7:0]  req_wdata,
    input  wire       req_ready,
    input  wire       rsp_valid,
    input  wire [2:0] rsp_status,
    input  wire [7:0] rsp_data
);
    localparam [2:0] P_IDLE=3'd0, P_SYNC2=3'd1, P_BODY=3'd2;
    reg [2:0] pstate;
    reg [2:0] byte_idx;
    reg [7:0] frame [0:6];
    reg pending;
    reg [7:0] pending_seq;
    reg [7:0] active_seq;
    reg [7:0] tx_byte;
    reg tx_valid;
    wire tx_ready;
    wire rx_valid;
    wire [7:0] rx_byte;
    reg [2:0] reply_idx;
    reg reply_active;
    reg [7:0] reply [0:5];

    function [7:0] crc8;
        input [7:0] crc_in;
        input [7:0] dat;
        integer k;
        reg [7:0] c;
        begin
            c = crc_in ^ dat;
            for (k=0; k<8; k=k+1)
                c = c[7] ? ((c << 1) ^ 8'h07) : (c << 1);
            crc8 = c;
        end
    endfunction

    g6504_uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst(rst), .uart_rx(uart_rx),
        .byte_valid(rx_valid), .byte_data(rx_byte)
    );
    g6504_uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst(rst), .data_valid(tx_valid), .data_in(tx_byte),
        .data_ready(tx_ready), .uart_tx(uart_tx)
    );

    always @(posedge clk) begin
        if (rst) begin
            pstate       <= P_IDLE;
            byte_idx     <= 3'd0;
            pending      <= 1'b0;
            pending_seq  <= 8'd0;
            active_seq   <= 8'd0;
            req_valid    <= 1'b0;
            req_write    <= 1'b0;
            req_addr     <= 15'd0;
            req_wdata    <= 8'd0;
            tx_valid     <= 1'b0;
            tx_byte      <= 8'd0;
            reply_idx    <= 3'd0;
            reply_active <= 1'b0;
        end else begin
            req_valid <= 1'b0;
            tx_valid  <= 1'b0;

            if (rx_valid) begin
                case (pstate)
                    P_IDLE: begin
                        if (rx_byte == 8'hA5) pstate <= P_SYNC2;
                    end
                    P_SYNC2: begin
                        if (rx_byte == 8'h5A) begin
                            byte_idx <= 3'd2;
                            frame[0] <= 8'hA5;
                            frame[1] <= 8'h5A;
                            pstate   <= P_BODY;
                        end else if (rx_byte != 8'hA5) pstate <= P_IDLE;
                    end
                    P_BODY: begin
                        if (byte_idx < 3'd7) frame[byte_idx] <= rx_byte;
                        if (byte_idx == 3'd7) begin
                            if (!pending && (frame[2] == 8'h01 || frame[2] == 8'h02) &&
                                (crc8(crc8(crc8(crc8(crc8(crc8(crc8(8'd0, frame[0]), frame[1]), frame[2]), frame[3]), frame[4]), frame[5]), frame[6]) == rx_byte)) begin
                                req_write <= (frame[2] == 8'h01);
                                req_addr  <= {frame[4][6:0], frame[5]};
                                req_wdata <= frame[6];
                                pending_seq <= frame[3];
                                pending   <= 1'b1;
                            end
                            pstate <= P_IDLE;
                        end else byte_idx <= byte_idx + 1'b1;
                    end
                    default: pstate <= P_IDLE;
                endcase
            end

            if (pending && req_ready) begin
                req_valid <= 1'b1;
                pending   <= 1'b0;
                active_seq <= pending_seq;
            end

            if (rsp_valid && !reply_active) begin
                reply[0] <= 8'h5A;
                reply[1] <= 8'hA5;
                reply[2] <= active_seq;
                reply[3] <= {5'd0, rsp_status};
                reply[4] <= rsp_data;
                reply[5] <= crc8(crc8(crc8(crc8(crc8(8'd0, 8'h5A), 8'hA5), active_seq), {5'd0, rsp_status}), rsp_data);
                reply_idx    <= 3'd0;
                reply_active <= 1'b1;
            end
            if (reply_active && tx_ready) begin
                tx_valid <= 1'b1;
                tx_byte  <= reply[reply_idx];
                if (reply_idx == 3'd5) reply_active <= 1'b0;
                else reply_idx <= reply_idx + 1'b1;
            end
        end
    end
endmodule
