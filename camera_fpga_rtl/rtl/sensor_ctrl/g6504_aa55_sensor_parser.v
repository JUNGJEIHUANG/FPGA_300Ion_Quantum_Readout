`timescale 1ns/1ps

// K7-compatible AA55 decoder for GSENSE6504BSI register access.
// SENSOR packets (header type != 0) load the address-indexed register BRAM.
// FPGA packets (header type == 0) use protocol addresses 1=start address,
// 2=length and 0=command; command 4 writes and command 5 reads a burst.
// A completion ACK/read result is emitted only after the SPI transaction ends.
module g6504_aa55_sensor_parser #(
    parameter integer SENSOR_DEPTH = 16384,
    parameter integer MAX_READ_LEN = 1024
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        i_word_valid,
    input  wire [31:0] i_word,

    output reg         o_burst_valid,
    output reg         o_burst_write,
    output reg  [14:0] o_burst_addr,
    output reg  [15:0] o_burst_len,
    output reg  [7:0]  o_burst_wdata,
    input  wire        i_burst_wreq,
    input  wire [15:0] i_burst_windex,
    input  wire        i_burst_ready,
    input  wire        i_burst_rsp_valid,
    input  wire [2:0]  i_burst_rsp_status,
    input  wire [7:0]  i_burst_rdata,
    input  wire [15:0] i_burst_rindex,
    input  wire        i_burst_rvalid,

    output reg         o_tx_en,
    output reg         o_tx_valid,
    output reg  [31:0] o_tx_word,
    output reg         o_packet_ack,
    output reg         o_packet_error,
    output wire [2:0]  o_debug_state
);
    localparam [3:0] S_IDLE      = 4'd0,
                     S_INFO      = 4'd1,
                     S_DATA      = 4'd2,
                     S_SUM       = 4'd3,
                     S_ISSUE     = 4'd4,
                     S_WAIT      = 4'd5,
                     S_TX_ARM    = 4'd6,
                     S_TX_HEADER = 4'd7,
                     S_TX_INFO   = 4'd8,
                     S_TX_DATA   = 4'd9,
                     S_TX_END    = 4'd10;

    reg [3:0]  state;
    reg [15:0] data_len;
    reg [15:0] data_idx;
    reg [31:0] checksum;
    reg        sensor_packet;
    reg [7:0]  ack_content;
    reg [7:0]  ack_mode;
    reg [15:0] fpga_reg_addr;
    reg [15:0] fpga_reg_len;
    reg [15:0] fpga_command;
    reg [15:0] tx_index;
    reg [7:0]  response_code;

    (* ram_style = "block" *) reg [7:0] sensor_mem [0:SENSOR_DEPTH-1];
    (* ram_style = "block" *) reg [7:0] result_mem [0:MAX_READ_LEN-1];

    assign o_debug_state = state[2:0];

    always @(posedge clk) begin
        // K7 sensor_protocol uses a synchronous BRAM read port.
        if (i_burst_wreq)
            o_burst_wdata <= sensor_mem[(o_burst_addr[13:0] + i_burst_windex[13:0])];
        if (i_burst_rvalid && (i_burst_rindex < MAX_READ_LEN))
            result_mem[i_burst_rindex] <= i_burst_rdata;

        if (rst) begin
            state <= S_IDLE; data_len <= 16'd0; data_idx <= 16'd0;
            checksum <= 32'd0; sensor_packet <= 1'b0;
            ack_content <= 8'd0; ack_mode <= 8'd0;
            fpga_reg_addr <= 16'd0; fpga_reg_len <= 16'd0;
            fpga_command <= 16'd0; tx_index <= 16'd0;
            response_code <= 8'd0;
            o_burst_valid <= 1'b0; o_burst_write <= 1'b0;
            o_burst_addr <= 15'd0; o_burst_len <= 16'd0;
            o_burst_wdata <= 8'd0;
            o_tx_en <= 1'b0; o_tx_valid <= 1'b0; o_tx_word <= 32'd0;
            o_packet_ack <= 1'b0; o_packet_error <= 1'b0;
        end else begin
            o_tx_valid <= 1'b0;
            o_packet_ack <= 1'b0;
            o_packet_error <= 1'b0;
            case (state)
                S_IDLE: begin
                    o_tx_en <= 1'b0;
                    o_burst_valid <= 1'b0;
                    if (i_word_valid && (i_word[15:0] == 16'hAA55)) begin
                        checksum <= i_word;
                        sensor_packet <= (i_word[23:16] != 8'd0);
                        fpga_reg_addr <= 16'd0;
                        fpga_reg_len <= 16'd0;
                        fpga_command <= 16'd0;
                        state <= S_INFO;
                    end
                end
                S_INFO: begin
                    if (i_word_valid) begin
                        checksum <= checksum + i_word;
                        ack_content <= i_word[31:24];
                        ack_mode <= i_word[23:16];
                        data_len <= i_word[15:0];
                        data_idx <= 16'd0;
                        state <= (i_word[15:0] == 16'd0) ? S_SUM : S_DATA;
                    end
                end
                S_DATA: begin
                    if (i_word_valid) begin
                        checksum <= checksum + i_word;
                        if (sensor_packet) begin
                            if (i_word[29:16] < SENSOR_DEPTH)
                                sensor_mem[i_word[29:16]] <= i_word[7:0];
                            else
                                o_packet_error <= 1'b1;
                        end else begin
                            case (i_word[31:16])
                                16'h0000: fpga_command  <= i_word[15:0];
                                16'h0001: fpga_reg_addr <= i_word[15:0];
                                16'h0002: fpga_reg_len  <= i_word[15:0];
                                default: ;
                            endcase
                        end
                        if (data_idx + 1'b1 >= data_len)
                            state <= S_SUM;
                        else
                            data_idx <= data_idx + 1'b1;
                    end
                end
                S_SUM: begin
                    if (i_word_valid) begin
                        if (checksum != i_word) begin
                            o_packet_error <= 1'b1;
                            state <= S_IDLE;
                        end else if (sensor_packet) begin
                            // SENSOR packet completion means BRAM loading is done;
                            // SPI execution is launched by the following FPGA cmd.
                            o_packet_ack <= 1'b1;
                            if (ack_mode == 8'h01) begin
                                response_code <= 8'h00;
                                o_burst_write <= 1'b1;
                                state <= S_TX_ARM;
                            end else state <= S_IDLE;
                        end else if (((fpga_command == 16'h0004) ||
                                      (fpga_command == 16'h0005)) &&
                                     (fpga_reg_len != 16'd0) &&
                                     ({1'b0, fpga_reg_addr[13:0]} + fpga_reg_len <= SENSOR_DEPTH) &&
                                     ((fpga_command != 16'h0005) ||
                                      (fpga_reg_len <= MAX_READ_LEN))) begin
                            o_burst_write <= (fpga_command == 16'h0004);
                            o_burst_addr <= fpga_reg_addr[14:0];
                            o_burst_len <= fpga_reg_len;
                            response_code <= (fpga_command == 16'h0004) ? 8'h83 : 8'h84;
                            state <= S_ISSUE;
                        end else begin
                            o_packet_error <= 1'b1;
                            state <= S_IDLE;
                        end
                    end
                end
                S_ISSUE: begin
                    o_burst_valid <= 1'b1;
                    if (o_burst_valid && i_burst_ready) begin
                        o_burst_valid <= 1'b0;
                        state <= S_WAIT;
                    end
                end
                S_WAIT: begin
                    if (i_burst_rsp_valid) begin
                        o_packet_ack <= (i_burst_rsp_status == 3'd0);
                        if (i_burst_rsp_status != 3'd0) begin
                            o_packet_error <= 1'b1;
                            state <= S_IDLE;
                        end else if ((ack_mode == 8'h01) || !o_burst_write) begin
                            state <= S_TX_ARM;
                        end else state <= S_IDLE;
                    end
                end
                S_TX_ARM: begin
                    // Assert tx_en before the first FIFO word.
                    o_tx_en <= 1'b1;
                    tx_index <= 16'd0;
                    state <= S_TX_HEADER;
                end
                S_TX_HEADER: begin
                    o_tx_valid <= 1'b1;
                    o_tx_word <= 32'h0000AA55;
                    state <= S_TX_INFO;
                end
                S_TX_INFO: begin
                    o_tx_valid <= 1'b1;
                    o_tx_word <= {response_code, 8'h00,
                                  (response_code == 8'h84) ? o_burst_len : 16'd0};
                    if ((response_code == 8'h84) && (o_burst_len != 16'd0))
                        state <= S_TX_DATA;
                    else
                        state <= S_TX_END;
                end
                S_TX_DATA: begin
                    o_tx_valid <= 1'b1;
                    o_tx_word <= {24'd0, result_mem[tx_index]};
                    if (tx_index + 1'b1 >= o_burst_len)
                        state <= S_TX_END;
                    else
                        tx_index <= tx_index + 1'b1;
                end
                S_TX_END: begin
                    o_tx_en <= 1'b0;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
