`timescale 1ns/1ps

module g6504_sensor_ctrl #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer CFG_DEPTH = 598, // must match line count of MEM_FILE
    parameter integer CFG_TO_PLL_TO_US = 5_000, // Config done -> SPI PLL_EN=1, 5 ms min
    parameter integer PLL_TO_US = 5_000,        // SPI PLL_EN=1 -> SPI STREAM_EN=1, 5 ms min
    parameter         UART_ONLY = 1'b0,         // 1: do not read MEM_FILE; configure through host UART
    parameter         MEM_FILE = "g6504_sensor_init_external_exposure.mem"
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        power_done,
    input  wire        config_upload,
    input  wire        read_reg,
    input  wire [14:0] reg_address,

    // Runtime UART/host access. Requests are accepted only in ST_READY.
    input  wire        host_req_valid,
    input  wire        host_req_write,
    input  wire [14:0] host_req_addr,
    input  wire [7:0]  host_req_wdata,
    output wire        host_req_ready,
    output reg         host_rsp_valid,
    output reg  [2:0]  host_rsp_status,
    output reg  [7:0]  host_rsp_rdata,

    // K7-compatible burst command interface used by the USB AA55 path.
    input  wire        host_burst_valid,
    input  wire        host_burst_write,
    input  wire [14:0] host_burst_addr,
    input  wire [15:0] host_burst_len,
    input  wire [7:0]  host_burst_wdata,
    output wire        host_burst_wreq,
    output wire [15:0] host_burst_windex,
    output wire        host_burst_ready,
    output reg         host_burst_rsp_valid,
    output reg  [2:0]  host_burst_rsp_status,
    output reg  [7:0]  host_burst_rdata,
    output reg  [15:0] host_burst_rindex,
    output reg         host_burst_rvalid,

    // Official CC2405 command 0x1006. After the uploaded mode table has been
    // written, execute the two sensor on-register writes (PLL then STREAM).
    input  wire        host_on_seq_valid,
    output wire        host_on_seq_ready,
    output reg         host_on_seq_done,
    input  wire [7:0]  host_pll_reg_base,
    input  wire [7:0]  host_stream_reg_base,
    input  wire [31:0] host_pwr_seq_wait0,
    input  wire [31:0] host_pwr_seq_wait1,

    // Kept for port compatibility. Training is now started externally via the
    // VIO command (vio_cmd_train) in the top level, so this FSM no longer waits
    // on the training result and these inputs are unused here.
    input  wire        train_done,
    input  wire        train_err,

    // Internal status flags: the corresponding sensor registers have been written.
    output reg         pll_en,
    output reg         stream_en,

    output wire        sensor_xce,
    output wire        sensor_sck,
    output wire        sensor_sdi,
    input  wire        sensor_sdo,

    output reg         cfg_done,
    output reg         read_done,
    output reg [7:0]   read_data,
    output reg [3:0]   sensor_state,
    output wire        sensor_spi_en,
    output wire        sensor_spi_done
);

    localparam integer US_TICKS = CLK_HZ / 1_000_000;
    localparam integer CFG_TO_PLL_TO_TICKS = CFG_TO_PLL_TO_US * US_TICKS;
    localparam integer PLL_TO_TICKS = PLL_TO_US * US_TICKS;
    localparam integer IDX_W = (CFG_DEPTH <= 2) ? 1 : $clog2(CFG_DEPTH);

    // Sensor internal register writes after the main init ROM.
    localparam [14:0] REG_PLL_EN_ADDR    = 15'h03EA;
    localparam [7:0]  REG_PLL_EN_DATA    = 8'hEA; // original 8'hAA | 8'h40, set bit6 PLL_EN=1
    localparam [14:0] REG_STREAM_EN_ADDR = 15'h0000;
    localparam [7:0]  REG_STREAM_EN_DATA = 8'h01; // set bit0 STREAM_EN=1

    localparam [3:0] ST_IDLE            = 4'd0;
    localparam [3:0] ST_SPI_WRITE_INIT  = 4'd1;
    localparam [3:0] ST_INIT_TO_PLL     = 4'd2;
    localparam [3:0] ST_WRITE_PLL       = 4'd3;
    localparam [3:0] ST_PLL_TO_STREAM   = 4'd4;
    localparam [3:0] ST_WRITE_STREAM    = 4'd5;
    localparam [3:0] ST_READY           = 4'd6;
    localparam [3:0] ST_SPI_READ        = 4'd7;
    localparam [3:0] ST_HOST_SPI        = 4'd8;
    localparam [3:0] ST_HOST_RESP       = 4'd9;
    localparam [3:0] ST_HOST_BURST      = 4'd10;
    localparam [3:0] ST_HOST_BURST_RESP = 4'd11;
    localparam [3:0] ST_STREAM_WAIT     = 4'd12;
    localparam [3:0] ST_HOST_CFG_TO_PLL = 4'd13;

    reg [3:0]       sensor_state_n;
    reg [IDX_W-1:0] index;
    reg [31:0]      delay_cnt;

    reg config_upload_d;
    reg read_reg_d;
    reg spi_en;
    reg spi_started;
    reg host_write_latched;
    reg [14:0] host_addr_latched;
    reg [7:0]  host_wdata_latched;
    reg        host_burst_write_latched;
    reg [14:0] host_burst_addr_latched;
    reg [15:0] host_burst_len_latched;
    reg [7:0]  host_burst_first_wdata;

    wire config_upload_rise = config_upload & ~config_upload_d;
    wire read_reg_rise      = read_reg & ~read_reg_d;
    assign host_req_ready   = power_done && (sensor_state == ST_READY);
    assign host_burst_ready = power_done && (sensor_state == ST_READY);
    assign host_on_seq_ready = power_done && (sensor_state == ST_READY);

    wire       spi_done;
    assign sensor_spi_en = spi_en;
    assign sensor_spi_done = spi_done;
    wire [7:0] spi_rdata;
    wire       spi_rd_valid;
    wire [15:0] spi_rd_index;
    wire       spi_data_req;
    wire [15:0] spi_data_index;
    wire [14:0] rom_addr;
    wire [7:0]  rom_data;

    wire spi_rw = (sensor_state == ST_SPI_READ) ? 1'b0 :
                  (sensor_state == ST_HOST_BURST) ? host_burst_write_latched :
                  (sensor_state == ST_HOST_SPI) ? host_write_latched : 1'b1;

    wire [14:0] spi_addr =
        (sensor_state == ST_SPI_READ)     ? reg_address :
        (sensor_state == ST_HOST_BURST)   ? host_burst_addr_latched :
        (sensor_state == ST_HOST_SPI)     ? host_addr_latched :
        (sensor_state == ST_WRITE_PLL)    ? REG_PLL_EN_ADDR :
        (sensor_state == ST_WRITE_STREAM) ? REG_STREAM_EN_ADDR :
                                            rom_addr;

    // Match official on_reg_ctrl: retain the mode-specific CSV bits and
    // force only PLL_EN/STREAM_EN. HDR uses 0x8A -> 0xCA, while STD uses
    // 0xAA -> 0xEA.
    wire [7:0] host_pll_on_data =
        {(host_pll_reg_base != 8'd0) ? host_pll_reg_base[7] : REG_PLL_EN_DATA[7],
         1'b1,
         (host_pll_reg_base != 8'd0) ? host_pll_reg_base[5:0] : REG_PLL_EN_DATA[5:0]};
    wire [7:0] host_stream_on_data = {host_stream_reg_base[7:1], 1'b1};
    wire [31:0] on_wait0_cycles = (host_pwr_seq_wait0 != 32'd0) ?
                                  host_pwr_seq_wait0 : CFG_TO_PLL_TO_TICKS;
    wire [31:0] on_wait1_cycles = (host_pwr_seq_wait1 != 32'd0) ?
                                  host_pwr_seq_wait1 : PLL_TO_TICKS;

    wire [7:0] spi_wdata =
        (sensor_state == ST_HOST_BURST)   ? host_burst_wdata :
        (sensor_state == ST_HOST_SPI)     ? host_wdata_latched :
        (sensor_state == ST_WRITE_PLL)    ? host_pll_on_data :
        (sensor_state == ST_WRITE_STREAM) ? host_stream_on_data :
                                            rom_data;

    wire [15:0] spi_length = (sensor_state == ST_HOST_BURST) ?
                             host_burst_len_latched : 16'd1;
    assign host_burst_wreq   = (sensor_state == ST_HOST_BURST) && spi_data_req;
    assign host_burst_windex = spi_data_index;

    generate
        if (!UART_ONLY) begin : gen_rom_config
            g6504_cfg_rom #(
                .DEPTH(CFG_DEPTH),
                .MEM_FILE(MEM_FILE)
            ) u_cfg_rom (
                .index(index),
                .addr(rom_addr),
                .data(rom_data)
            );
        end else begin : gen_uart_only_config
            assign rom_addr = 15'd0;
            assign rom_data = 8'd0;
        end
    endgenerate

    g6504_spi_master u_spi (
        .clk(clk),
        .rst_n(rst_n),
        .SPI_EN(spi_en),
        .rw(spi_rw),
        .addr(spi_addr),
        .length(spi_length),
        .data(spi_wdata),
        .data_req(spi_data_req),
        .data_index(spi_data_index),
        .busy(),
        .done(spi_done),
        .rd_data(spi_rdata),
        .rd_index(spi_rd_index),
        .rd_valid(spi_rd_valid),
        .sensor_xce(sensor_xce),
        .sensor_sck(sensor_sck),
        .sensor_sdi(sensor_sdi),
        .sensor_sdo(sensor_sdo)
    );

    // FSM state register
    always @(posedge clk) begin
        if (!rst_n) begin
            sensor_state <= ST_IDLE;
        end else begin
            sensor_state <= sensor_state_n;
        end
    end

    // FSM next-state logic
    always @(*) begin
        sensor_state_n = sensor_state;

        case (sensor_state)
            ST_IDLE: begin
                if (power_done) begin
                    if (UART_ONLY)
                        sensor_state_n = ST_READY;
                    else if (config_upload_rise)
                        sensor_state_n = ST_SPI_WRITE_INIT;
                end
            end

            ST_SPI_WRITE_INIT: begin
                if (!power_done) begin
                    sensor_state_n = ST_IDLE;
                end else if (spi_done) begin
                    if (index == CFG_DEPTH-1) begin
                        sensor_state_n = ST_INIT_TO_PLL;
                    end
                end
            end

            ST_INIT_TO_PLL: begin
                if (!power_done) begin
                    sensor_state_n = ST_IDLE;
                end else if (delay_cnt >= CFG_TO_PLL_TO_TICKS-1) begin
                    sensor_state_n = ST_WRITE_PLL;
                end
            end

            ST_WRITE_PLL: begin
                if (!power_done) begin
                    sensor_state_n = ST_IDLE;
                end else if (spi_started && spi_done) begin
                    sensor_state_n = ST_PLL_TO_STREAM;
                end
            end

            ST_PLL_TO_STREAM: begin
                if (!power_done) begin
                    sensor_state_n = ST_IDLE;
                end else if (delay_cnt >= on_wait0_cycles-1) begin
                    sensor_state_n = ST_WRITE_STREAM;
                end
            end

            ST_WRITE_STREAM: begin
                if (!power_done) begin
                    sensor_state_n = ST_IDLE;
                end else if (spi_started && spi_done) begin
                    sensor_state_n = ST_STREAM_WAIT;
                end
            end

            ST_STREAM_WAIT: begin
                if (!power_done)
                    sensor_state_n = ST_IDLE;
                else if (delay_cnt >= on_wait1_cycles-1)
                    sensor_state_n = ST_READY;
            end

            ST_HOST_CFG_TO_PLL: begin
                if (!power_done)
                    sensor_state_n = ST_IDLE;
                else if (delay_cnt >= CFG_TO_PLL_TO_TICKS-1)
                    sensor_state_n = ST_WRITE_PLL;
            end

            ST_READY: begin
                if (!power_done) begin
                    sensor_state_n = ST_IDLE;
                end else if (host_on_seq_valid) begin
                    // Enforce Figure 23: wait at least 5 ms after the host
                    // configuration upload before asserting PLL_EN.  The
                    // following states then wait wait0 before STREAM_EN and
                    // wait1 before acknowledging the sequence.
                    sensor_state_n = ST_HOST_CFG_TO_PLL;
                end else if (host_burst_valid) begin
                    sensor_state_n = ST_HOST_BURST;
                end else if (host_req_valid) begin
                    sensor_state_n = ST_HOST_SPI;
                end else if (read_reg_rise) begin
                    sensor_state_n = ST_SPI_READ;
                end
            end

            ST_SPI_READ: begin
                if (!power_done) begin
                    sensor_state_n = ST_IDLE;
                end else if (spi_done) begin
                    sensor_state_n = ST_READY;
                end
            end

            ST_HOST_SPI: begin
                if (!power_done) begin
                    sensor_state_n = ST_IDLE;
                end else if (spi_started && spi_done) begin
                    sensor_state_n = ST_HOST_RESP;
                end
            end

            ST_HOST_RESP: begin
                sensor_state_n = ST_READY;
            end

            ST_HOST_BURST: begin
                if (!power_done)
                    sensor_state_n = ST_IDLE;
                else if (spi_started && spi_done)
                    sensor_state_n = ST_HOST_BURST_RESP;
            end

            ST_HOST_BURST_RESP: begin
                sensor_state_n = ST_READY;
            end

            default: begin
                sensor_state_n = ST_IDLE;
            end
        endcase
    end

    // FSM outputs and datapath
    always @(posedge clk) begin
        if (!rst_n) begin
            index           <= {IDX_W{1'b0}};
            delay_cnt       <= 32'd0;
            config_upload_d <= 1'b0;
            read_reg_d      <= 1'b0;
            pll_en          <= 1'b0;
            stream_en       <= 1'b0;
            cfg_done        <= 1'b0;
            read_done       <= 1'b0;
            read_data       <= 8'd0;
            spi_en          <= 1'b0;
            spi_started     <= 1'b0;
            host_write_latched <= 1'b0;
            host_addr_latched  <= 15'd0;
            host_wdata_latched <= 8'd0;
            host_rsp_valid     <= 1'b0;
            host_rsp_status    <= 3'd0;
            host_rsp_rdata     <= 8'd0;
            host_burst_write_latched <= 1'b0;
            host_burst_addr_latched  <= 15'd0;
            host_burst_len_latched   <= 16'd0;
            host_burst_first_wdata   <= 8'd0;
            host_burst_rsp_valid     <= 1'b0;
            host_burst_rsp_status    <= 3'd0;
            host_burst_rdata         <= 8'd0;
            host_burst_rindex        <= 16'd0;
            host_burst_rvalid        <= 1'b0;
            host_on_seq_done         <= 1'b0;
        end else begin
            config_upload_d <= config_upload;
            read_reg_d      <= read_reg;
            spi_en          <= 1'b0; // default: SPI_EN is a one-cycle pulse
            host_rsp_valid  <= 1'b0;
            host_burst_rsp_valid <= 1'b0;
            host_burst_rvalid    <= 1'b0;
            host_on_seq_done     <= 1'b0;

            case (sensor_state)
                ST_IDLE: begin
                    index           <= {IDX_W{1'b0}};
                    delay_cnt       <= 32'd0;
                    pll_en          <= 1'b0;
                    stream_en       <= 1'b0;
                    cfg_done        <= 1'b0;
                    read_done       <= 1'b0;
                    spi_started     <= 1'b0;
                    if (!UART_ONLY && power_done && config_upload_rise) begin
                        spi_en <= 1'b1; // start first ROM write
                    end
                end

                ST_SPI_WRITE_INIT: begin
                    if (spi_done) begin
                        if (index == CFG_DEPTH-1) begin
                            index     <= {IDX_W{1'b0}};
                            delay_cnt <= 32'd0;
                        end else begin
                            index  <= index + 1'b1;
                            spi_en <= 1'b1; // start next ROM write; new index is visible next clock
                        end
                    end
                end

                ST_INIT_TO_PLL: begin
                    if (delay_cnt >= CFG_TO_PLL_TO_TICKS-1) begin
                        delay_cnt <= 32'd0;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_WRITE_PLL: begin
                    if (!spi_started) begin
                        spi_en      <= 1'b1; // start write 0x03EA = 0xEA
                        spi_started <= 1'b1;
                    end else if (spi_done) begin
                        spi_started <= 1'b0;
                        pll_en      <= 1'b1;
                        delay_cnt   <= 32'd0;
                    end
                end

                ST_PLL_TO_STREAM: begin
                    if (delay_cnt >= on_wait0_cycles-1) begin
                        delay_cnt <= 32'd0;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_WRITE_STREAM: begin
                    if (!spi_started) begin
                        spi_en      <= 1'b1; // start write 0x0000 = 0x01
                        spi_started <= 1'b1;
                    end else if (spi_done) begin
                        spi_started <= 1'b0;
                        stream_en   <= 1'b1;
                        delay_cnt   <= 32'd0;
                    end
                end

                ST_STREAM_WAIT: begin
                    if (delay_cnt >= on_wait1_cycles-1) begin
                        delay_cnt        <= 32'd0;
                        cfg_done         <= 1'b1;
                        host_on_seq_done <= 1'b1;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_HOST_CFG_TO_PLL: begin
                    if (delay_cnt >= CFG_TO_PLL_TO_TICKS-1) begin
                        delay_cnt <= 32'd0;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_READY: begin
                    if (host_on_seq_valid) begin
                        // Start the fixed configuration-to-PLL interval from
                        // a known value on every official on-sequence request.
                        delay_cnt   <= 32'd0;
                        spi_started <= 1'b0;
                    end else if (host_burst_valid) begin
                        host_burst_write_latched <= host_burst_write;
                        host_burst_addr_latched  <= host_burst_addr;
                        host_burst_len_latched   <= host_burst_len;
                        host_burst_first_wdata   <= host_burst_wdata;
                        spi_started              <= 1'b0;
                    end else if (host_req_valid) begin
                        host_write_latched <= host_req_write;
                        host_addr_latched  <= host_req_addr;
                        host_wdata_latched <= host_req_wdata;
                        spi_started        <= 1'b0;
                    end else if (read_reg_rise) begin
                        read_done <= 1'b0;
                        spi_en    <= 1'b1;
                    end
                end

                ST_SPI_READ: begin
                    if (spi_done) begin
                        if (spi_rd_valid) begin
                            read_data <= spi_rdata;
                        end
                        read_done <= 1'b1;
                    end
                end

                ST_HOST_SPI: begin
                    if (!spi_started) begin
                        spi_en      <= 1'b1;
                        spi_started <= 1'b1;
                    end else if (spi_done) begin
                        spi_started  <= 1'b0;
                        host_rsp_status <= 3'd0;
                        if (!host_write_latched && spi_rd_valid)
                            host_rsp_rdata <= spi_rdata;
                    end
                end

                ST_HOST_RESP: begin
                    host_rsp_valid <= 1'b1;
                    if (host_write_latched && (host_addr_latched == REG_PLL_EN_ADDR))
                        pll_en <= host_wdata_latched[6];
                    if (host_write_latched && (host_addr_latched == REG_STREAM_EN_ADDR))
                        stream_en <= host_wdata_latched[0];
                end

                ST_HOST_BURST: begin
                    if (!spi_started) begin
                        spi_en <= 1'b1;
                        spi_started <= 1'b1;
                    end
                    if (spi_rd_valid) begin
                        host_burst_rdata  <= spi_rdata;
                        host_burst_rindex <= spi_rd_index;
                        host_burst_rvalid <= 1'b1;
                    end
                    if (spi_started && spi_done) begin
                        spi_started <= 1'b0;
                        host_burst_rsp_status <= 3'd0;
                    end
                end

                ST_HOST_BURST_RESP: begin
                    host_burst_rsp_valid <= 1'b1;
                    if (host_burst_write_latched &&
                        (host_burst_addr_latched == REG_PLL_EN_ADDR))
                        pll_en <= host_burst_first_wdata[6];
                    if (host_burst_write_latched &&
                        (host_burst_addr_latched == REG_STREAM_EN_ADDR))
                        stream_en <= host_burst_first_wdata[0];
                end

                default: begin
                    index           <= {IDX_W{1'b0}};
                    delay_cnt       <= 32'd0;
                    spi_started     <= 1'b0;
                end
            endcase
        end
    end

endmodule
