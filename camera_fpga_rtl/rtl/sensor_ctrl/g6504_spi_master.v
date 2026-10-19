`timescale 1ns/1ps

// GSENSE6504BSI 4-wire SPI master. The transaction format and burst
// behaviour follow K7_Demo/sensor_4line_spi_drive: one R/W bit, one 15-bit
// start address, then N consecutive 8-bit register values while XCE remains
// asserted. Write data is prefetched one byte ahead so a synchronous BRAM
// may be used by the caller.
module g6504_spi_master (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        SPI_EN,
    input  wire        rw,
    input  wire [14:0] addr,
    input  wire [15:0] length,
    input  wire [7:0]  data,
    output reg         data_req,
    output reg  [15:0] data_index,
    output reg         busy,
    output reg         done,
    output reg  [7:0]  rd_data,
    output reg  [15:0] rd_index,
    output reg         rd_valid,
    output reg         sensor_xce,
    output reg         sensor_sck,
    output reg         sensor_sdi,
    input  wire        sensor_sdo
);
    // clk=100 MHz, divisor=9 -> SCK=5 MHz, as in the K7 demo.
    localparam [2:0] ST_IDLE     = 3'd0,
                     ST_START    = 3'd1,
                     ST_HEADER   = 3'd2,
                     ST_DATA     = 3'd3,
                     ST_XCE_HOLD = 3'd4,
                     ST_XCE_HIGH = 3'd5;

    reg [2:0]  state;
    reg [3:0]  div_cnt;
    reg [15:0] header_shift;
    reg [4:0]  header_bits;
    reg [2:0]  data_bit;
    reg [15:0] byte_index;
    reg [15:0] length_latched;
    reg [7:0]  wr_byte;
    reg [7:0]  rd_shift;
    reg        rw_latched;

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= ST_IDLE; div_cnt <= 4'd0; header_shift <= 16'd0;
            header_bits <= 5'd0; data_bit <= 3'd0; byte_index <= 16'd0;
            length_latched <= 16'd0; wr_byte <= 8'd0; rd_shift <= 8'd0;
            rw_latched <= 1'b1; data_req <= 1'b0; data_index <= 16'd0;
            busy <= 1'b0; done <= 1'b0; rd_data <= 8'd0;
            rd_index <= 16'd0; rd_valid <= 1'b0;
            sensor_xce <= 1'b1; sensor_sck <= 1'b1; sensor_sdi <= 1'b0;
        end else begin
            done <= 1'b0;
            rd_valid <= 1'b0;
            data_req <= 1'b0;
            case (state)
                ST_IDLE: begin
                    busy <= 1'b0; div_cnt <= 4'd0;
                    sensor_xce <= 1'b1; sensor_sck <= 1'b1; sensor_sdi <= 1'b0;
                    if (SPI_EN && (length != 16'd0)) begin
                        busy <= 1'b1; rw_latched <= rw; length_latched <= length;
                        header_shift <= {rw, addr}; header_bits <= 5'd0;
                        data_bit <= 3'd0; byte_index <= 16'd0; rd_shift <= 8'd0;
                        data_index <= 16'd0; data_req <= rw;
                        sensor_xce <= 1'b0; state <= ST_START;
                    end
                end
                ST_START: begin
                    // Half-SCK setup time also covers synchronous BRAM latency.
                    if (div_cnt == 4'd9) begin
                        div_cnt <= 4'd0; wr_byte <= data; state <= ST_HEADER;
                    end else div_cnt <= div_cnt + 1'b1;
                end
                ST_HEADER: begin
                    if (div_cnt == 4'd9) begin
                        div_cnt <= 4'd0;
                        if (sensor_sck) begin
                            sensor_sck <= 1'b0; sensor_sdi <= header_shift[15];
                            header_shift <= {header_shift[14:0], 1'b0};
                        end else begin
                            sensor_sck <= 1'b1;
                            if (header_bits == 5'd15) begin
                                data_bit <= 3'd0;
                                if (rw_latched && (length_latched > 16'd1)) begin
                                    data_index <= 16'd1; data_req <= 1'b1;
                                end
                                state <= ST_DATA;
                            end else header_bits <= header_bits + 1'b1;
                        end
                    end else div_cnt <= div_cnt + 1'b1;
                end
                ST_DATA: begin
                    if (div_cnt == 4'd9) begin
                        div_cnt <= 4'd0;
                        if (sensor_sck) begin
                            sensor_sck <= 1'b0;
                            sensor_sdi <= rw_latched ? wr_byte[7-data_bit] : 1'b0;
                        end else begin
                            sensor_sck <= 1'b1;
                            if (!rw_latched) rd_shift <= {rd_shift[6:0], sensor_sdo};
                            if (data_bit == 3'd7) begin
                                if (!rw_latched) begin
                                    rd_data <= {rd_shift[6:0], sensor_sdo};
                                    rd_index <= byte_index; rd_valid <= 1'b1;
                                end
                                if (byte_index + 1'b1 >= length_latched) begin
                                    state <= ST_XCE_HOLD;
                                end else begin
                                    byte_index <= byte_index + 1'b1; data_bit <= 3'd0;
                                    rd_shift <= 8'd0;
                                    if (rw_latched) begin
                                        wr_byte <= data;
                                        if (byte_index + 2 < length_latched) begin
                                            data_index <= byte_index + 2; data_req <= 1'b1;
                                        end
                                    end
                                end
                            end else data_bit <= data_bit + 1'b1;
                        end
                    end else div_cnt <= div_cnt + 1'b1;
                end
                ST_XCE_HOLD: begin
                    sensor_xce <= 1'b0; sensor_sck <= 1'b1; sensor_sdi <= 1'b0;
                    if (div_cnt == 4'd9) begin
                        div_cnt <= 4'd0; sensor_xce <= 1'b1; state <= ST_XCE_HIGH;
                    end else div_cnt <= div_cnt + 1'b1;
                end
                ST_XCE_HIGH: begin
                    sensor_xce <= 1'b1; sensor_sck <= 1'b1; sensor_sdi <= 1'b0;
                    if (div_cnt == 4'd9) begin
                        div_cnt <= 4'd0; busy <= 1'b0; done <= 1'b1; state <= ST_IDLE;
                    end else div_cnt <= div_cnt + 1'b1;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
