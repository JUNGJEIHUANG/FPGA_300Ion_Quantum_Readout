`timescale 1ns/1ps

// CC2405 command executor for the official Gpixel AA55 protocol.
//
// Packet framing/checksum validation is intentionally delegated to the
// unmodified vendor rx_pkg_decoder.  This block executes the decoded command
// stream against the self-developed power/SPI/training/frame datapath and
// emits responses in the format consumed by cc2405_official_control.py.
module g6504_official_command_bridge #(
    parameter integer SENSOR_DEPTH = 16384,
    parameter integer MAX_READ_LEN = 1024
)(
    input  wire        clk,
    input  wire        rst,

    input  wire [7:0]  i_ack_content,
    input  wire [7:0]  i_ack_mode,
    input  wire        i_prot_sensor_vld,
    input  wire [15:0] i_prot_sensor_addr,
    input  wire [15:0] i_prot_sensor_dat,
    input  wire        i_prot_fpga_vld,
    input  wire [15:0] i_prot_fpga_addr,
    input  wire [15:0] i_prot_fpga_dat,

    output reg         o_pc_active,
    output reg         o_power_on_level,
    output reg         o_cmd_sensor_reset,
    output reg         o_cmd_train,
    output reg         o_cmd_frame_req,
    output reg         o_cmd_on_seq,

    input  wire        i_power_done,
    input  wire        i_power_off_done,
    input  wire        i_capture_ready,

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

    input  wire        i_on_seq_ready,
    input  wire        i_on_seq_done,

    input  wire        i_train_done,
    input  wire        i_train_err,
    input  wire        i_fifo_train_wen,
    input  wire [7:0]  i_fifo_train_din,

    input  wire [15:0] i_frame_columns,
    input  wire [15:0] i_frame_rows,
    input  wire [15:0] i_frame_count,
    input  wire [7:0]  i_pixel_width,

    output reg         o_tx_en,
    output reg         o_tx_valid,
    output reg  [31:0] o_tx_word,
    output reg         o_packet_ack,
    output reg         o_packet_error,
    output wire [2:0]  o_debug_state
);
    localparam [3:0] S_IDLE           = 4'd0,
                     S_BURST_ISSUE    = 4'd1,
                     S_BURST_WAIT     = 4'd2,
                     S_POWER_ON_WAIT  = 4'd3,
                     S_POWER_OFF_WAIT = 4'd4,
                     S_ONSEQ_ISSUE    = 4'd5,
                     S_ONSEQ_WAIT     = 4'd6,
                     S_TRAIN_WAIT     = 4'd7,
                     S_TX_ARM         = 4'd8,
                     S_TX_HEADER      = 4'd9,
                     S_TX_INFO        = 4'd10,
                     S_TX_DATA        = 4'd11,
                     S_FRAME_WORD2    = 4'd12,
                     S_FRAME_WORD3    = 4'd13,
                     S_FRAME_WORD4    = 4'd14,
                     S_TX_END         = 4'd15;

    localparam [1:0] RSP_NONE  = 2'd0,
                     RSP_READ  = 2'd1,
                     RSP_TRAIN = 2'd2,
                     RSP_FRAME = 2'd3;

    reg [3:0] state;
    reg [15:0] fpga_reg_addr;
    reg [15:0] fpga_reg_len;
    reg [7:0]  response_code;
    reg [15:0] response_len;
    reg [1:0]  response_kind;
    reg [15:0] tx_index;
    reg [7:0]  train_count;
    integer k;

    (* ram_style = "block" *) reg [7:0] sensor_mem [0:SENSOR_DEPTH-1];
    (* ram_style = "block" *) reg [7:0] read_result_mem [0:MAX_READ_LEN-1];
    (* ram_style = "distributed" *) reg [7:0] train_result_mem [0:111];

    assign o_debug_state = state[2:0];

    always @(posedge clk) begin
        if (rst) begin
            state              <= S_IDLE;
            fpga_reg_addr      <= 16'd0;
            fpga_reg_len       <= 16'd1;
            response_code      <= 8'd0;
            response_len       <= 16'd0;
            response_kind      <= RSP_NONE;
            tx_index           <= 16'd0;
            train_count        <= 8'd0;
            o_pc_active        <= 1'b0;
            o_power_on_level   <= 1'b0;
            o_cmd_sensor_reset <= 1'b0;
            o_cmd_train        <= 1'b0;
            o_cmd_frame_req    <= 1'b0;
            o_cmd_on_seq       <= 1'b0;
            o_burst_valid      <= 1'b0;
            o_burst_write      <= 1'b0;
            o_burst_addr       <= 15'd0;
            o_burst_len        <= 16'd0;
            o_burst_wdata      <= 8'd0;
            o_tx_en            <= 1'b0;
            o_tx_valid         <= 1'b0;
            o_tx_word          <= 32'd0;
            o_packet_ack       <= 1'b0;
            o_packet_error     <= 1'b0;
            for (k = 0; k < 112; k = k + 1)
                train_result_mem[k] <= 8'd0;
        end else begin
            o_cmd_sensor_reset <= 1'b0;
            o_cmd_train        <= 1'b0;
            o_cmd_frame_req    <= 1'b0;
            o_cmd_on_seq       <= 1'b0;
            o_tx_valid         <= 1'b0;
            o_packet_ack       <= 1'b0;
            o_packet_error     <= 1'b0;

            // Match the synchronous BRAM contract used by the existing Sensor
            // SPI burst engine: data is presented one clock after its request.
            if (i_burst_wreq)
                o_burst_wdata <= sensor_mem[(o_burst_addr[13:0] +
                                             i_burst_windex[13:0])];
            if (i_burst_rvalid && (i_burst_rindex < MAX_READ_LEN))
                read_result_mem[i_burst_rindex] <= i_burst_rdata;
            if (i_fifo_train_wen && (train_count < 8'd112)) begin
                train_result_mem[train_count] <= i_fifo_train_din;
                train_count <= train_count + 1'b1;
            end
            if (i_prot_sensor_vld && (i_prot_sensor_addr < SENSOR_DEPTH))
                sensor_mem[i_prot_sensor_addr[13:0]] <= i_prot_sensor_dat[7:0];

            if (i_prot_sensor_vld || i_prot_fpga_vld)
                o_pc_active <= 1'b1;

            // Addresses 1/2 are the official Sensor SPI range registers.
            // All FPGA words are also forwarded in parallel to fpga_protocol.
            if (i_prot_fpga_vld) begin
                case (i_prot_fpga_addr)
                    16'h0001: fpga_reg_addr <= i_prot_fpga_dat;
                    16'h0002: fpga_reg_len  <= i_prot_fpga_dat;
                    default: ;
                endcase
            end

            case (state)
                S_IDLE: begin
                    o_tx_en       <= 1'b0;
                    o_burst_valid <= 1'b0;
                    tx_index      <= 16'd0;

                    // Address zero is also present as system_common=0 in the
                    // embedded FPGA CSV. It is a command only when the packet
                    // requests a result ACK. Otherwise an unsolicited 0xFF
                    // would corrupt the next USB transaction.
                    if (i_prot_fpga_vld && (i_prot_fpga_addr == 16'h0000) &&
                        (i_ack_mode != 8'd0)) begin
                        response_len  <= 16'd0;
                        response_kind <= RSP_NONE;
                        case (i_prot_fpga_dat)
                            16'h0001: begin
                                o_power_on_level <= 1'b1;
                                state <= i_power_done ? S_TX_ARM : S_POWER_ON_WAIT;
                                response_code <= 8'h80;
                            end
                            16'h0002: begin
                                o_power_on_level <= 1'b0;
                                state <= i_power_off_done ? S_TX_ARM : S_POWER_OFF_WAIT;
                                response_code <= 8'h81;
                            end
                            16'h0003: begin
                                o_cmd_sensor_reset <= 1'b1;
                                response_code <= 8'h82;
                                state <= S_TX_ARM;
                            end
                            16'h0004, 16'h0005: begin
                                if ((fpga_reg_len == 16'd0) ||
                                    ({1'b0, fpga_reg_addr[13:0]} + fpga_reg_len > SENSOR_DEPTH) ||
                                    ((i_prot_fpga_dat == 16'h0005) &&
                                     (fpga_reg_len > MAX_READ_LEN))) begin
                                    response_code <= 8'hFF;
                                    o_packet_error <= 1'b1;
                                    state <= S_TX_ARM;
                                end else begin
                                    o_burst_write <= (i_prot_fpga_dat == 16'h0004);
                                    o_burst_addr  <= fpga_reg_addr[14:0];
                                    o_burst_len   <= fpga_reg_len;
                                    response_code <= (i_prot_fpga_dat == 16'h0004) ? 8'h83 : 8'h84;
                                    response_len  <= (i_prot_fpga_dat == 16'h0004) ? 16'd0 : fpga_reg_len;
                                    response_kind <= (i_prot_fpga_dat == 16'h0004) ? RSP_NONE : RSP_READ;
                                    state <= S_BURST_ISSUE;
                                end
                            end
                            16'h0006: begin
                                train_count <= 8'd0;
                                for (k = 0; k < 112; k = k + 1)
                                    train_result_mem[k] <= 8'd0;
                                o_cmd_train <= 1'b1;
                                response_code <= 8'h85;
                                response_len <= 16'd112;
                                response_kind <= RSP_TRAIN;
                                state <= S_TRAIN_WAIT;
                            end
                            16'h0007: begin
                                if (i_capture_ready) begin
                                    o_cmd_frame_req <= 1'b1;
                                    response_code <= 8'h86;
                                    response_kind <= RSP_FRAME;
                                    state <= S_TX_ARM;
                                end else begin
                                    response_code <= 8'hFF;
                                    o_packet_error <= 1'b1;
                                    state <= S_TX_ARM;
                                end
                            end
                            16'h1006: begin
                                response_code <= 8'hAF;
                                state <= S_ONSEQ_ISSUE;
                            end
                            default: begin
                                // Unsupported commands receive the official error ACK.
                                response_code <= 8'hFF;
                                o_packet_error <= 1'b1;
                                state <= S_TX_ARM;
                            end
                        endcase
                    end
                end

                S_BURST_ISSUE: begin
                    // Present VALID for at least one complete clock.  READY is
                    // normally already high in ST_READY; testing it on the
                    // same edge that first raises this registered VALID would
                    // otherwise create a zero-cycle request at the receiver.
                    if (!o_burst_valid) begin
                        o_burst_valid <= 1'b1;
                    end else if (i_burst_ready) begin
                        o_burst_valid <= 1'b0;
                        state <= S_BURST_WAIT;
                    end
                end

                S_BURST_WAIT: begin
                    if (i_burst_rsp_valid) begin
                        if (i_burst_rsp_status == 3'd0) begin
                            o_packet_ack <= 1'b1;
                        end else begin
                            response_code <= 8'hFF;
                            response_len <= 16'd0;
                            response_kind <= RSP_NONE;
                            o_packet_error <= 1'b1;
                        end
                        state <= S_TX_ARM;
                    end
                end

                S_POWER_ON_WAIT: begin
                    if (i_power_done) begin
                        o_packet_ack <= 1'b1;
                        state <= S_TX_ARM;
                    end
                end

                S_POWER_OFF_WAIT: begin
                    if (i_power_off_done) begin
                        o_packet_ack <= 1'b1;
                        state <= S_TX_ARM;
                    end
                end

                S_ONSEQ_ISSUE: begin
                    o_cmd_on_seq <= 1'b1;
                    if (i_on_seq_ready)
                        state <= S_ONSEQ_WAIT;
                end

                S_ONSEQ_WAIT: begin
                    if (i_on_seq_done) begin
                        o_packet_ack <= 1'b1;
                        state <= S_TX_ARM;
                    end
                end

                S_TRAIN_WAIT: begin
                    // train_err can still describe the preceding run;
                    // train_done marks completion of this request and the
                    // availability of the complete 112-byte result.
                    if (i_train_done) begin
                        o_packet_ack <= 1'b1;
                        state <= S_TX_ARM;
                    end
                end

                S_TX_ARM: begin
                    // The vendor USB mux samples tx_en before the first word.
                    o_tx_en  <= 1'b1;
                    tx_index <= 16'd0;
                    state    <= S_TX_HEADER;
                end

                S_TX_HEADER: begin
                    o_tx_valid <= 1'b1;
                    o_tx_word  <= 32'h0000_AA55;
                    state      <= S_TX_INFO;
                end

                S_TX_INFO: begin
                    o_tx_valid <= 1'b1;
                    o_tx_word  <= {response_code, 8'h00,
                                   (response_kind == RSP_FRAME) ? 16'd0 : response_len};
                    if (response_kind == RSP_FRAME)
                        state <= S_FRAME_WORD2;
                    else if (response_len != 16'd0)
                        state <= S_TX_DATA;
                    else
                        state <= S_TX_END;
                end

                S_TX_DATA: begin
                    o_tx_valid <= 1'b1;
                    if (response_kind == RSP_READ)
                        o_tx_word <= {24'd0, read_result_mem[tx_index]};
                    else
                        o_tx_word <= {24'd0, train_result_mem[tx_index]};
                    if (tx_index + 1'b1 >= response_len)
                        state <= S_TX_END;
                    else
                        tx_index <= tx_index + 1'b1;
                end

                // Official 20-byte frame header: AA55, info, geometry, frame
                // count, pixel width. Pixel data follows through img2tx.
                S_FRAME_WORD2: begin
                    o_tx_valid <= 1'b1;
                    o_tx_word  <= {i_frame_rows, i_frame_columns};
                    state      <= S_FRAME_WORD3;
                end
                S_FRAME_WORD3: begin
                    o_tx_valid <= 1'b1;
                    o_tx_word  <= {16'd0, i_frame_count};
                    state      <= S_FRAME_WORD4;
                end
                S_FRAME_WORD4: begin
                    o_tx_valid <= 1'b1;
                    o_tx_word  <= {24'd0, i_pixel_width};
                    state      <= S_TX_END;
                end

                S_TX_END: begin
                    o_tx_en <= 1'b0;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // Keep these fields in the interface because the vendor decoder exposes
    // them and future commands may use them. Current PC operations always ask
    // for completion-time ACKs; command execution above intentionally waits
    // for the corresponding done condition.
    wire _unused_ack = &{1'b0, i_ack_content, i_ack_mode};

endmodule
