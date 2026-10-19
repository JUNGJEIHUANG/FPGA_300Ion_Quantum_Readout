`timescale 1ns/1ps

// GSEN6504BSI bring-up with VIO control:
//   Power ON -> 6-step rails -> SYS_RST_N -> CLK_REF -> Config -> PLL_EN -> STREAM
module g6504_bringup_top #(
    parameter integer SYS_CLK_HZ   = 100_000_000,
    parameter integer POWER_STEP   = 6,
    parameter         MEM_FILE     = "g6504_sensor_init_external_exposure_5ms_gain3.mem",
    parameter         UART_CONFIG_ONLY = 1'b1, // 1: no FPGA SENSOR ROM; host USB AA55/UART supplies writes
    parameter         DEBUG_BYPASS_DDR = 1'b0, // 1: sensor/pixel -> USB directly (simple test, DDR out of path); 0: enable DDR frame buffer
    parameter         DEBUG_PC_STITCH_USB = 1'b0, // 1 with DEBUG_BYPASS_DDR: send sync-tagged raw stream to FX3 for PC-side stitching
    parameter integer VSYNC_PULSE_US = 2000 // exposure request T0 = 2 ms.
                                               // Readout does NOT happen during the pulse: rows start after the VSYNC
                                               // falling edge and take ~2100 x TLINE (~27.3ms) to complete.
)(
    input  wire        i_CLK_OSC_IN_P,
    input  wire        i_CLK_OSC_IN_N,
    input  wire        i_FPGA_RST_N,    //Pure external input (on-board reset button)

    // External differential 8-N-1 UART, matching K7_Demo's OS serial pins.
    // The PC side must use an LVDS/RS-422 compatible transceiver.
    input  wire        i_OS_SERTC_P,
    input  wire        i_OS_SERTC_N,
    output wire        o_OS_SERTFG_P,
    output wire        o_OS_SERTFG_N,

    output wire        o_CLK_REF, //40M
    output wire        o_PLL_IN_P, //40M LVDS, sensor data-interface 
    output wire        o_PLL_IN_N,

    output wire        o_SYS_RST_N,//SENSOR RESET
    output wire        o_VSYNC,
    output wire        o_SPI_XCE,
    output wire        o_SPI_SCK,
    output wire        o_SPI_SDI,
    input  wire        i_SPI_SDO,

    output wire        o_HSYNC,
    output wire        o_EN_VCC4V5,
    output wire        o_EN_VCC2V5,
    output wire        o_EN_VCC3V3,
    output wire        o_EN_VCC_LT,
    output wire        o_EN_VCC,
    output wire        o_EN_VCC5V5,
    output wire        o_BUS_TRANS_OE_N,
    output wire        o_EN_VDDA,
    output wire        o_EN_VDDIO,
    output wire        o_EN_VDDD,
    output wire        o_EN_VDDAD,
    output wire        o_EN_VDDPLL,
    output wire        o_EN_VDDSF,
    output wire        o_EN_H,
    output wire        o_EN_L,
    output wire        o_EN_BIAS,
    output wire        o_EN_VDDQ,
    output wire        o_OTP_CTRL,
    output wire        o_PLL_EN,
    output wire        o_SEL,
    output wire        o_SEL3,
    output wire        o_SEL2,
    output wire        o_SEL1,
    output wire        o_SEL0,
    output wire        o_CLK_SEL,
    output wire        o_SDATA,
    output wire        o_SCLK,
    output wire        o_FSEL1,
    output wire        o_FSEL0,
    output wire        o_SLAVE_CTRL,
    output wire        o_CTRL_RELAY,

    input  wire        i_CLK_DDR_IN_P,
    input  wire        i_CLK_DDR_IN_N,
    input  wire [15:0] i_SENSOR_DATA_P,
    input  wire [15:0] i_SENSOR_DATA_N,

    // USB 3.0 / FX3 Interface
    output wire        o_FX3_PCLK,
    input  wire        i_FX3_FLAGA,
    input  wire        i_FX3_FLAGB,
    output wire        o_FX3_SLCS_N,
    output wire        o_FX3_SLWR_N,
    output wire        o_FX3_SLRD_N,
    output wire        o_FX3_SLOE_N,
    output wire        o_FX3_PKTEND_N,
    output wire [1:0]  o_FX3_A,
    inout  wire [31:0] io_FX3_DB,
    output wire        o_USB_RESET,

    // DDR3 interface (K7_Demo compatible MIG)
    output wire [15:0] o_DDR3_ADDR,
    output wire [2:0]  o_DDR3_BA,
    output wire        o_DDR3_CAS_N,
    output wire        o_DDR3_CK_N,
    output wire        o_DDR3_CK_P,
    output wire        o_DDR3_CKE,
    output wire        o_DDR3_RAS_N,
    output wire        o_DDR3_RESET_N,
    output wire        o_DDR3_WE_N,
    inout  wire [63:0] io_DDR3_DQ,
    inout  wire [7:0]  io_DDR3_DQS_N,
    inout  wire [7:0]  io_DDR3_DQS_P,
    output wire        o_DDR3_CS_N,
    output wire [7:0]  o_DDR3_DM,
    output wire        o_DDR3_ODT
);

    wire clk_idly;
    wire [0:0] config_upload;
    wire [0:0] read_reg;
    wire [15:0] reg_address; // VIO Reg_Address, use reg_address[14:0] for SPI address
    wire [3:0] sensor_state;
    wire       power_done;
    wire       cfg_done;
    wire       read_done;
    wire [7:0] read_data;
    wire       uart_req_valid;
    wire       uart_req_write;
    wire [14:0] uart_req_addr;
    wire [7:0] uart_req_wdata;
    wire       uart_req_ready;
    wire       uart_rsp_valid;
    wire [2:0] uart_rsp_status;
    wire [7:0] uart_rsp_data;
    wire       uart_rx_single;
    wire       uart_tx_single;
    wire       usb_rx_word_valid;
    wire [31:0] usb_rx_word;
    wire       usb_rx_fifo_rd_en;
    wire [31:0] usb_rx_fifo_rd_dat;
    wire       usb_rx_fifo_empty;
    wire [7:0] usb_ack_content;
    wire [7:0] usb_ack_mode;
    wire       prot_sensor_vld;
    wire [15:0] prot_sensor_addr;
    wire [15:0] prot_sensor_dat;
    wire       prot_fpga_vld;
    wire [15:0] prot_fpga_addr;
    wire [15:0] prot_fpga_dat;
    wire       pc_control_active;
    wire       pc_power_on_level;
    wire       pc_cmd_sensor_reset;
    wire       pc_cmd_train;
    wire       pc_cmd_frame_req;
    wire       pc_cmd_on_seq;
    wire       sensor_on_seq_ready;
    wire       sensor_on_seq_done;
    wire [12:0] official_tx_fifo_cnt;
    wire       usb_sensor_burst_valid;
    wire       usb_sensor_burst_write;
    wire [14:0] usb_sensor_burst_addr;
    wire [15:0] usb_sensor_burst_len;
    wire [7:0]  usb_sensor_burst_wdata;
    wire       usb_sensor_burst_wreq;
    wire [15:0] usb_sensor_burst_windex;
    wire       usb_sensor_burst_ready;
    wire       usb_sensor_burst_rsp_valid;
    wire [2:0] usb_sensor_burst_rsp_status;
    wire [7:0] usb_sensor_burst_rdata;
    wire [15:0] usb_sensor_burst_rindex;
    wire       usb_sensor_burst_rvalid;
    wire       usb_sensor_tx_en;
    wire       usb_sensor_tx_valid;
    wire [31:0] usb_sensor_tx_word;
    wire       usb_sensor_packet_ack;
    wire       usb_sensor_packet_error;
    wire [2:0]  usb_sensor_parser_state;
    wire       sensor_spi_en_dbg;
    wire       sensor_spi_done_dbg;
    wire       fx3_tx_word_strobe;
    wire       fx3_tx_word_accepted;
    wire       fx3_tx_word_is_image;
    wire [31:0] fx3_tx_word;
    wire [14:0] fx3_tag_fifo_count;
    wire       fx3_tag_fifo_full;
    wire       fx3_tag_fifo_empty;
    wire       host_req_valid;
    wire       host_req_write;
    wire [14:0] host_req_addr;
    wire [7:0]  host_req_wdata;
    wire       host_req_ready;

    // Dynamic mode values decoded by the official Gpixel register maps.
    wire [7:0]   pc_adc_depth;
    wire [11:0]  pc_synccode_1st;
    wire [11:0]  pc_synccode_2nd;
    wire [11:0]  pc_synccode_3rd;
    wire [11:0]  pc_synccode_4th_sof;
    wire [11:0]  pc_synccode_4th_sol;
    wire         pc_reg_msb_first_en;
    wire [1:0]   pc_reg_hdr_mode_en;
    wire [1:0]   pc_reg_flip_v_en;
    wire         pc_reg_flip_h_en;
    wire [15:0]  pc_reg_frame_num;
    wire [15:0]  pc_reg_line_time;
    wire [15:0]  pc_reg_training_word;
    wire [4:0]   pc_reg_n_mux;
    wire [15:0]  pc_reg_w_mux;
    wire [15:0]  pc_win_row_len;
    wire [15:0]  pc_win_row_len_ddr;
    wire [15:0]  pc_win_col_len;
    wire [15:0]  pc_lval_len;
    wire [15:0]  pc_lval_len_ddr;
    wire [15:0]  pc_lval_len_buf;
    wire [15:0]  pc_chan_vld;
    wire [7:0]   pc_reg_stream_en;
    wire [7:0]   pc_reg_pll_en;
    wire         fifo_train_wen;
    wire [7:0]   fifo_train_din;
    wire [POWER_STEP*32-1:0] pc_time_pwr_on_clk;
    wire [POWER_STEP*32-1:0] pc_time_pwr_off_clk;
    wire [31:0]  pc_pwr_seq_wait0;
    wire [31:0]  pc_pwr_seq_wait1;
    wire [31:0]  pc_pwr_seq_wait2;
    wire [31:0]  pc_inte_linetime;
    wire [15:0]  pc_inte_clkpix;
    wire [15:0]  pc_frame_num_tx =
        (pc_reg_hdr_mode_en == 2'd0) ? pc_reg_frame_num : (pc_reg_frame_num << 1);

    // K7_Demo uses these same differential pins and LVDS_25 I/O standard.
    // serial_comm_out in K7_Demo inverts the internal UART level before the
    // OBUFDS; preserve that polarity here for compatibility.
    IBUFDS #(
        .DIFF_TERM    ("TRUE"),
        .IBUF_LOW_PWR ("TRUE"),
        .IOSTANDARD   ("LVDS_25")
    ) u_uart_rx_ibufds (
        .I  (i_OS_SERTC_P),
        .IB (i_OS_SERTC_N),
        .O  (uart_rx_single)
    );

    OBUFDS #(
        .IOSTANDARD ("LVDS_25"),
        .SLEW       ("SLOW")
    ) u_uart_tx_obufds (
        .I  (~uart_tx_single),
        .O  (o_OS_SERTFG_P),
        .OB (o_OS_SERTFG_N)
    );
    wire       train_done;
    wire       train_err;
    wire clk_osc;
    wire clk_200M;
    wire clk_220M;
    wire clk_100M;
    wire mmcm0_locked;
    wire clk_ref_unbuf;
    wire pll_ok;
    wire [14:0] tx_fifo_cnt;
    wire [31:0] usb_dbg;
    wire [31:0] usb_tx_word_total;
    wire [31:0] usb_pkt_total;
    wire [31:0] flagb_low_total;
    reg  [31:0] ddr_to_usb_word_total;
    reg tp100_vld;
    (* ASYNC_REG="TRUE" *) reg rst_sys_meta, rst_sys;
    
// PCB OSCILLAR INPUT
    IBUFDS #(
        .DIFF_TERM("FALSE"),
        .IBUF_LOW_PWR("TRUE"),
        .IOSTANDARD("DIFF_SSTL15")
    ) u_sys_clk_ibufds (
        .I(i_CLK_OSC_IN_P),
        .IB(i_CLK_OSC_IN_N),
        .O(clk_osc)
    );
    
    wire clk_sys = clk_100M;

    clk_wiz_0 mmcm_0 (
        .clk_200M (clk_200M),
        .clk_220M (clk_220M),
        .clk_100M (clk_100M),
        .clk_40M (clk_ref_unbuf),
        .reset    (1'b0),
        .locked   (mmcm0_locked),
        .clk_in1  (clk_osc)
    );

    wire mmcm1_locked;

    clk_wiz_1 mmcm_1
    (
     // Clock out ports
     .clk_out1(clk_idly),     // output 300M
     // Status and control signals
     .reset(1'b0),
     .locked(mmcm1_locked),       // output locked
    // Clock in ports
     .clk_in1(clk_osc)      // input clk_in1
 );

 wire rst_idly;
 assign rst_idly = ~mmcm1_locked;

 (* IODELAY_GROUP = "GROUP1ST" *) IDELAYCTRL u_idc_train (
 .RDY    (),
 .REFCLK (clk_idly),
 .RST    (rst_idly)
);


// Center VIO
    wire vio_cmd_power_on;
    wire cmd_power_on = pc_control_active ? pc_power_on_level : vio_cmd_power_on;
    wire cmd_train;
    wire vio_cmd_train;
    wire vio_vsync_req;
    reg  img_tx_en;

    vio_0 u_vio (
        .clk(clk_sys),

        // VIO keeps only slow control/status. DDR/MIG handshakes should be observed by ILA in ui_clk domain.
        .probe_in0(power_done),                                      // [0] sensor power sequence done
        .probe_in1(cfg_done),                                        // [0] SPI configuration done
        .probe_in2(pll_ok),                                          // [0] sensor PLL/data clock OK
        .probe_in3({train_done, train_err, img_tx_en,
                    i_FX3_FLAGB, o_FX3_SLCS_N, o_FX3_SLWR_N}),        // [5]train_done [4]train_err [3]img_tx_en [2]FLAGB [1]SLCS_N [0]SLWR_N
        .probe_in4(tx_fifo_cnt),                                     // [14:0] USB TX FIFO fill level

        .probe_out0(vio_cmd_power_on),
        .probe_out1(config_upload),
        .probe_out2(read_reg),      // unused by image path in this build; keep low during normal capture
        .probe_out3(reg_address),   // [15:0]
        .probe_out4(vio_cmd_train), // one-shot training trigger only
        .probe_out5(vio_vsync_req)  // rising edge triggers fixed-width VSYNC pulse
    );

    assign cmd_train = vio_cmd_train | pc_cmd_train;

    // External exposure VSYNC pulse generator.
    // VIO probe_out5 is a trigger request, not the VSYNC level itself:
    // rising edge -> o_VSYNC high for the exposure-register duration -> auto low.
    localparam integer VSYNC_PULSE_CYCLES = (SYS_CLK_HZ / 1_000_000) * VSYNC_PULSE_US;
    // Sensor LINE_TIME uses 20 ns units. clk_sys uses 10 ns cycles.
    wire [47:0] pc_vsync_cycles_wide = pc_inte_linetime * pc_reg_line_time * 2 + pc_inte_clkpix;
    wire [31:0] active_vsync_cycles =
        (pc_control_active && pc_vsync_cycles_wide != 0) ? pc_vsync_cycles_wide[31:0] : VSYNC_PULSE_CYCLES;
    reg                     vio_vsync_req_d;
    reg                     vsync_pulse_active;
    reg [31:0]              vsync_pulse_cnt;
    reg                     vsync_out_r;
    reg                     rio_frame_req_tgl_sys;
    reg                     train_started; // latched one-shot training request
    wire                    frame_req_level = vio_vsync_req | pc_cmd_frame_req;
    wire                    vio_vsync_rise = frame_req_level & ~vio_vsync_req_d;
    // T0 for the latency measurement: the request passed every exposure gate
    // and is accepted by the VSYNC generator on this clk_sys edge.
    wire                    exposure_start_event = vio_vsync_rise && train_started &&
                                                   !train_err && !vsync_pulse_active;

    assign o_VSYNC = vsync_out_r;

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            vio_vsync_req_d    <= 1'b0;
            vsync_pulse_active <= 1'b0;
            vsync_pulse_cnt    <= 32'd0;
            vsync_out_r        <= 1'b0;
            rio_frame_req_tgl_sys <= 1'b0;
        end else begin
            vio_vsync_req_d <= frame_req_level;

            // Accept a VSYNC trigger after training has been requested and no
            // training error is reported. train_done is a one-cycle pulse and is
            // not used as a level gate here.
            if (exposure_start_event) begin
                vsync_pulse_active <= 1'b1;
                vsync_pulse_cnt    <= 32'd0;
                vsync_out_r        <= 1'b1;
                rio_frame_req_tgl_sys <= ~rio_frame_req_tgl_sys;
            end else if (vsync_pulse_active) begin
                if (vsync_pulse_cnt >= active_vsync_cycles - 1) begin
                    vsync_pulse_active <= 1'b0;
                    vsync_pulse_cnt    <= 32'd0;
                    vsync_out_r        <= 1'b0;
                end else begin
                    vsync_pulse_cnt <= vsync_pulse_cnt + 1'b1;
                    vsync_out_r     <= 1'b1;
                end
            end else begin
                vsync_out_r <= 1'b0;
            end
        end
    end


// Achieve rst_sys from i_FPGA_RST_N or mmcm1 Locked.1 is reset, 0 is normal

    reg        rst_raw;
    reg [9:0]  cnt_rst;
    always @(posedge clk_osc) begin
        if (!mmcm0_locked || !mmcm1_locked) begin  // removed !i_FPGA_RST_N: may be undriven on G6504_060
            cnt_rst <= 10'd0;
            rst_raw <= 1'b1;
        end else if (cnt_rst < 10'd1023) begin
            cnt_rst <= cnt_rst + 10'd1;
            rst_raw <= 1'b1;
        end else begin
            rst_raw <= 1'b0;
        end
    end
    

    always @(posedge clk_sys) begin
        if (rst_raw) begin
            rst_sys_meta <= 1'b1;
            rst_sys      <= 1'b1;
        end else begin
            rst_sys_meta <= 1'b0;
            rst_sys      <= rst_sys_meta;
        end
    end


// Power on module
    wire [POWER_STEP-1:0] power_bus;
    wire clk_ref_en;
    localparam integer CYCLES_PER_US    = SYS_CLK_HZ / 1_000_000;
    localparam integer CYCLES_PER_MS    = SYS_CLK_HZ / 1_000;

    // board/pre-rail -> VDDIO -> VDDAD+VDDD -> VDDA ->
    // VDDSF/VDDCH/VRSTH/VTXH -> VTXL. Use 1 ms minimum between
    // power steps. After the last rail, TO 2 us minimum before SYS_RST_N.
    localparam [31:0] TIME_PWR_ON_0 = 32'd2 * CYCLES_PER_MS; // step0 -> step1, 1 ms min
    localparam [31:0] TIME_PWR_ON_1 = 32'd2 * CYCLES_PER_MS; // step1 -> step2, 1 ms min
    localparam [31:0] TIME_PWR_ON_2 = 32'd2 * CYCLES_PER_MS; // step2 -> step3, 1 ms min
    localparam [31:0] TIME_PWR_ON_3 = 32'd2 * CYCLES_PER_MS; // step3 -> step4, 1 ms min
    localparam [31:0] TIME_PWR_ON_4 = 32'd2 * CYCLES_PER_MS; // step4 -> step5, 1 ms min
    localparam [31:0] TIME_PWR_ON_5 = 32'd4 * CYCLES_PER_US; // last rail -> SYS_RST_N, 2 us min

    wire [POWER_STEP*32-1:0] fallback_time_pwr_on_clk = {
        TIME_PWR_ON_5, TIME_PWR_ON_4, TIME_PWR_ON_3,
        TIME_PWR_ON_2, TIME_PWR_ON_1, TIME_PWR_ON_0
    };
    // Once the PC protocol is active, use the official FPGA register bank's
    // power timing values. Before that, retain the safe bring-up defaults.
    wire [POWER_STEP*32-1:0] time_pwr_on_clk =
        pc_control_active ? pc_time_pwr_on_clk : fallback_time_pwr_on_clk;

    power_rst_seq #(
        .POWER_STEP(POWER_STEP),
        .POWERON_SYS2CLK(2*CYCLES_PER_US),  // SYS_RST_N -> CLK_REF, 2 us min
        .POWERON_CLK2DONE(1*CYCLES_PER_US), // CLK_REF -> SPI config allowed, 1 us min
        .POWEROFF_SYS2CLK(2*CYCLES_PER_US),
        .POWEROFF_CLK2PWR(2*CYCLES_PER_US),
        .POWEROFF_STEP_TIME(1000*CYCLES_PER_US)
    ) u_power_rst_seq (
        .clk                    (clk_sys),
        .rst                    (rst_sys),
        .i_cmd_power_on         (cmd_power_on),
        .i_time_pwr_on_clk      (time_pwr_on_clk),
        .i_time_pwr_off_clk     (pc_time_pwr_off_clk),
        .o_POWER_BUS            (power_bus),
        .o_SYS_RST_N            (o_SYS_RST_N),
        .o_clk_ref_en           (clk_ref_en),
        .o_power_done           (power_done)
    );

    // clk_ref_en is generated in clk_sys(100 MHz), while BUFGCE.CE is sampled
    // by the 40 MHz CLK_REF input domain. Synchronize CE into the 40 MHz domain
    // before driving BUFGCE.CE.
    (* ASYNC_REG = "TRUE" *) reg clk_ref_en_sync1 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg clk_ref_en_sync2 = 1'b0;

    always @(posedge clk_ref_unbuf) begin  //clk_ref_unbuf is 40M
        clk_ref_en_sync1 <= clk_ref_en;
        clk_ref_en_sync2 <= clk_ref_en_sync1;
    end

    BUFGCE u_bufgce_clk_ref (
        .I(clk_ref_unbuf),
        .CE(clk_ref_en_sync2),
        .O(o_CLK_REF)
    );

    // 40 MHz LVDS clock forwarded to the sensor's data-interface PLL input.
    // K7 Demo drives this on o_PLL_IN_P/N (pin F11); without it the sensor PLL
    // has no reference, the high-speed LVDS clock/data never lock, and training
    // always fails (train_err=1). Gate with the same CLK_REF enable.
    wire clk_pll;
    BUFGCE u_bufgce_pll (
        .I(clk_ref_unbuf),
        .CE(clk_ref_en_sync2),
        .O(clk_pll)
    );
    OBUFDS #(
        .IOSTANDARD("DEFAULT"),
        .SLEW("SLOW")
    ) u_obufds_pll (
        .I(clk_pll),
        .O(o_PLL_IN_P),
        .OB(o_PLL_IN_N)
    );

    // K7 CC2405 EVK rail mapping (GSEN6504BSI)
    assign o_EN_VCC4V5      = power_bus[0];
    assign o_EN_VCC2V5      = power_bus[0];
    assign o_EN_VCC3V3      = 1'b1;
    assign o_EN_VCC_LT      = power_bus[0];
    assign o_EN_VCC         = power_bus[0];
    assign o_EN_VCC5V5      = power_bus[0];
    assign o_BUS_TRANS_OE_N = ~power_bus[1];
    assign o_EN_VDDIO       = power_bus[1];
    assign o_EN_VDDAD       = power_bus[2];
    assign o_EN_VDDD        = power_bus[2];
    assign o_EN_VDDPLL      = power_bus[2];
    wire pll_en;
    wire stream_en;
    assign o_PLL_EN         = pll_en;
    // Match the official CC2405 peripheral configuration.  The board-level
    // IDT 8T49N004I must be programmed after its enable rail is asserted;
    // merely supplying the 40 MHz reference does not configure its outputs.
    assign o_SEL  = 1'b0;
    assign o_SEL0 = 1'b0;
    assign o_SEL1 = 1'b0;
    assign o_SEL2 = 1'b0;
    assign o_SEL3 = 1'b0;

    idt_8t49n004i u_idt_8t49n004i (
        .clk           (clk_sys),
        .rst           (rst_sys),
        .i_cmd_set_pll (o_PLL_EN),
        .o_CLK_SEL     (o_CLK_SEL),
        .o_FSEL0       (o_FSEL0),
        .o_FSEL1       (o_FSEL1),
        .o_IDT_8T_SCL  (o_SCLK),
        .o_IDT_8T_SDA  (o_SDATA)
    );
    assign o_EN_VDDA        = power_bus[3];
    assign o_EN_VDDSF       = power_bus[4];
    assign o_EN_H           = power_bus[4];
    assign o_EN_L           = power_bus[5];
    assign o_EN_BIAS        = power_bus[5];

    assign o_EN_VDDQ        = 1'b0;
    assign o_OTP_CTRL       = 1'b0;
    assign o_HSYNC          = 1'b0;
    assign o_SLAVE_CTRL     = 1'b1;
    assign o_CTRL_RELAY     = 1'b0;

    g6504_uart_sensor_uart #(
        .CLK_HZ(SYS_CLK_HZ),
        .BAUD(921_600)
    ) u_uart_sensor (
        .clk        (clk_sys),
        .rst        (rst_sys),
        .uart_rx    (uart_rx_single),
        .uart_tx    (uart_tx_single),
        .req_valid  (uart_req_valid),
        .req_write  (uart_req_write),
        .req_addr   (uart_req_addr),
        .req_wdata  (uart_req_wdata),
        .req_ready  (host_req_ready),
        .rsp_valid  (uart_rsp_valid),
        .rsp_status (uart_rsp_status),
        .rsp_data   (uart_rsp_data)
    );

    // UART retains the single-register interface. USB uses the independent
    // K7-compatible burst port and has priority inside g6504_sensor_ctrl.
    assign host_req_valid = uart_req_valid;
    assign host_req_write = uart_req_write;
    assign host_req_addr  = uart_req_addr;
    assign host_req_wdata = uart_req_wdata;

    // Keep the official vendor decoder unchanged. It owns packet framing,
    // receive-FIFO latency and checksum validation.
    rx_pkg_decoder u_rx_pkg_decoder (
        .clk                (clk_sys),
        .rst                (rst_sys),
        .i_rx_fifo_rd_dat   (usb_rx_fifo_rd_dat),
        .o_rx_fifo_rd_en    (usb_rx_fifo_rd_en),
        .i_rx_fifo_rd_empty (usb_rx_fifo_empty),
        .o_ack_content      (usb_ack_content),
        .o_ack_mode         (usb_ack_mode),
        .o_prot_sensor_vld  (prot_sensor_vld),
        .o_prot_sensor_addr (prot_sensor_addr),
        .o_prot_sensor_dat  (prot_sensor_dat),
        .o_prot_fpga_vld    (prot_fpga_vld),
        .o_prot_fpga_addr   (prot_fpga_addr),
        .o_prot_fpga_dat    (prot_fpga_dat)
    );

    // Official Sensor register semantics and derived image geometry. Sensor
    // packet words update this shadow map at the same time as the SPI BRAM.
    sensor_protocol #(
        .REG_DAT_WIDTH(8), .DAT_PAIRS(16), .DAT_WIDTH(12)
    ) u_pc_sensor_protocol (
        .clk                    (clk_sys),
        .rst                    (rst_sys || !o_SYS_RST_N),
        .clk_spi                (clk_sys),
        .i_prot_vld             (prot_sensor_vld),
        .i_prot_addr            (prot_sensor_addr),
        .i_prot_dat             (prot_sensor_dat),
        .i_reg_ren              (1'b0),
        .i_reg_raddr            (16'd0),
        .o_reg_rdat             (),
        .o_adc_depth            (pc_adc_depth),
        .o_synccode_1st         (pc_synccode_1st),
        .o_synccode_2nd         (pc_synccode_2nd),
        .o_synccode_3rd         (pc_synccode_3rd),
        .o_synccode_4th_sof     (pc_synccode_4th_sof),
        .o_synccode_4th_sol     (pc_synccode_4th_sol),
        .o_reg_sync_invert_en   (),
        .o_reg_master           (),
        .o_reg_hdr_mode_en      (pc_reg_hdr_mode_en),
        .o_reg_frame_num        (pc_reg_frame_num),
        .o_reg_line_time        (pc_reg_line_time),
        .o_reg_win_row_len      (),
        .o_reg_flip_v_en        (pc_reg_flip_v_en),
        .o_reg_msb_first_en     (pc_reg_msb_first_en),
        .o_reg_flip_h_en        (pc_reg_flip_h_en),
        .o_reg_n_mux            (pc_reg_n_mux),
        .o_reg_training_word    (pc_reg_training_word),
        .o_reg_w_mux            (pc_reg_w_mux),
        .o_reg_pwr_mode         (),
        .o_reg_stream_en        (pc_reg_stream_en),
        .o_reg_pll_en           (pc_reg_pll_en),
        .o_reg_otp_prog_en      (),
        .o_reg_pga_en           (),
        .o_win_row_len          (pc_win_row_len),
        .o_win_row_len_ddr      (pc_win_row_len_ddr),
        .o_win_row_len_lp       (),
        .o_win_col_len          (pc_win_col_len),
        .o_lval_len             (pc_lval_len),
        .o_lval_len_ddr         (pc_lval_len_ddr),
        .o_lval_len_buf         (pc_lval_len_buf),
        .o_chan_vld             (pc_chan_vld)
    );

    // Official FPGA register map. The current self image datapath consumes the
    // power-related outputs immediately; additional mode outputs can be wired
    // incrementally without changing the PC ABI.
    fpga_protocol #(.POWER_STEP(POWER_STEP)) u_pc_fpga_protocol (
        .clk                 (clk_sys),
        .rst                 (rst_sys),
        .i_prot_vld          (prot_fpga_vld),
        .i_prot_addr         (prot_fpga_addr),
        .i_prot_dat          (prot_fpga_dat),
        .o_time_pwr_on_clk   (pc_time_pwr_on_clk),
        .o_time_pwr_off_clk  (pc_time_pwr_off_clk),
        .o_pwr_seq_wait0     (pc_pwr_seq_wait0),
        .o_pwr_seq_wait1     (pc_pwr_seq_wait1),
        .o_pwr_seq_wait2     (pc_pwr_seq_wait2),
        .o_inte_linetime     (pc_inte_linetime),
        .o_inte_clkpix       (pc_inte_clkpix)
    );

    g6504_official_command_bridge u_official_command_bridge (
        .clk                 (clk_sys),
        .rst                 (rst_sys),
        .i_ack_content       (usb_ack_content),
        .i_ack_mode          (usb_ack_mode),
        .i_prot_sensor_vld   (prot_sensor_vld),
        .i_prot_sensor_addr  (prot_sensor_addr),
        .i_prot_sensor_dat   (prot_sensor_dat),
        .i_prot_fpga_vld     (prot_fpga_vld),
        .i_prot_fpga_addr    (prot_fpga_addr),
        .i_prot_fpga_dat     (prot_fpga_dat),
        .o_pc_active         (pc_control_active),
        .o_power_on_level    (pc_power_on_level),
        .o_cmd_sensor_reset  (pc_cmd_sensor_reset),
        .o_cmd_train         (pc_cmd_train),
        .o_cmd_frame_req     (pc_cmd_frame_req),
        .o_cmd_on_seq        (pc_cmd_on_seq),
        .i_power_done        (power_done),
        .i_power_off_done    ((~|power_bus) && !o_SYS_RST_N && !clk_ref_en),
        .i_capture_ready     (cfg_done && train_started && !train_err),
        .o_burst_valid       (usb_sensor_burst_valid),
        .o_burst_write       (usb_sensor_burst_write),
        .o_burst_addr        (usb_sensor_burst_addr),
        .o_burst_len         (usb_sensor_burst_len),
        .o_burst_wdata       (usb_sensor_burst_wdata),
        .i_burst_wreq        (usb_sensor_burst_wreq),
        .i_burst_windex      (usb_sensor_burst_windex),
        .i_burst_ready       (usb_sensor_burst_ready),
        .i_burst_rsp_valid   (usb_sensor_burst_rsp_valid),
        .i_burst_rsp_status  (usb_sensor_burst_rsp_status),
        .i_burst_rdata       (usb_sensor_burst_rdata),
        .i_burst_rindex      (usb_sensor_burst_rindex),
        .i_burst_rvalid      (usb_sensor_burst_rvalid),
        .i_on_seq_ready      (sensor_on_seq_ready),
        .i_on_seq_done       (sensor_on_seq_done),
        .i_train_done        (train_done),
        .i_train_err         (train_err),
        .i_fifo_train_wen    (fifo_train_wen),
        .i_fifo_train_din    (fifo_train_din),
        .i_frame_columns     ((pc_win_col_len != 16'd0) ? pc_win_col_len : 16'd2176),
        .i_frame_rows        ((pc_win_row_len != 16'd0) ? pc_win_row_len : 16'd2048),
        .i_frame_count       ((pc_frame_num_tx != 16'd0) ? pc_frame_num_tx : 16'd1),
        .i_pixel_width       ((pc_adc_depth != 8'd0) ? pc_adc_depth : 8'd12),
        .o_tx_en             (usb_sensor_tx_en),
        .o_tx_valid          (usb_sensor_tx_valid),
        .o_tx_word           (usb_sensor_tx_word),
        .o_packet_ack        (usb_sensor_packet_ack),
        .o_packet_error      (usb_sensor_packet_error),
        .o_debug_state       (usb_sensor_parser_state)
    );

    assign usb_rx_word_valid = usb_rx_fifo_rd_en;
    assign usb_rx_word       = usb_rx_fifo_rd_dat;

   
    g6504_sensor_ctrl #(
        .CLK_HZ(SYS_CLK_HZ),
        .CFG_DEPTH(598),
        .UART_ONLY(UART_CONFIG_ONLY),
        .MEM_FILE(MEM_FILE)
    ) u_sensor_ctrl (
        .clk               (clk_sys),
        .rst_n             (~rst_sys),
        .power_done        (power_done),
        .config_upload     (config_upload[0]),
        .read_reg          (1'b0),
        .reg_address       (reg_address[14:0]),
        .host_req_valid    (host_req_valid),
        .host_req_write    (host_req_write),
        .host_req_addr     (host_req_addr),
        .host_req_wdata    (host_req_wdata),
        .host_req_ready    (host_req_ready),
        .host_rsp_valid    (uart_rsp_valid),
        .host_rsp_status   (uart_rsp_status),
        .host_rsp_rdata    (uart_rsp_data),
        .host_burst_valid  (usb_sensor_burst_valid),
        .host_burst_write  (usb_sensor_burst_write),
        .host_burst_addr   (usb_sensor_burst_addr),
        .host_burst_len    (usb_sensor_burst_len),
        .host_burst_wdata  (usb_sensor_burst_wdata),
        .host_burst_wreq   (usb_sensor_burst_wreq),
        .host_burst_windex (usb_sensor_burst_windex),
        .host_burst_ready  (usb_sensor_burst_ready),
        .host_burst_rsp_valid  (usb_sensor_burst_rsp_valid),
        .host_burst_rsp_status (usb_sensor_burst_rsp_status),
        .host_burst_rdata  (usb_sensor_burst_rdata),
        .host_burst_rindex (usb_sensor_burst_rindex),
        .host_burst_rvalid (usb_sensor_burst_rvalid),
        .host_on_seq_valid (pc_cmd_on_seq),
        .host_on_seq_ready (sensor_on_seq_ready),
        .host_on_seq_done  (sensor_on_seq_done),
        .host_pll_reg_base (pc_reg_pll_en),
        .host_stream_reg_base(pc_reg_stream_en),
        .host_pwr_seq_wait0(pc_pwr_seq_wait0),
        .host_pwr_seq_wait1(pc_pwr_seq_wait1),
        .train_done        (train_done),
        .train_err         (train_err),
        .pll_en            (pll_en),
        .stream_en         (stream_en),
        .sensor_xce        (o_SPI_XCE),
        .sensor_sck        (o_SPI_SCK),
        .sensor_sdi        (o_SPI_SDI),
        .sensor_sdo        (i_SPI_SDO),
        .cfg_done          (cfg_done),
        .read_done         (read_done),
        .read_data         (read_data),
        .sensor_state      (sensor_state),
        .sensor_spi_en     (sensor_spi_en_dbg),
        .sensor_spi_done   (sensor_spi_done_dbg)
    );

    wire [0:0] clk_rx;
    wire [0:0] clk_ddr;
    wire [0:0] rst_rx;
    wire       clk_check_pll;

    wire [15:0]  train_dat_vld;
    wire [191:0] train_dat;
    wire [63:0]  train_lane8_align_dbg;

    clock_rx #(
        .CLKDDR_PAIRS(1)
    ) u_clock_rx (
        .i_CLK_DDR_IN_P ({i_CLK_DDR_IN_P}),
        .i_CLK_DDR_IN_N ({i_CLK_DDR_IN_N}),
        .i_sys_rst_n    ( o_SYS_RST_N),
        .o_clk_check_pll(clk_check_pll),
        .o_clk_rx       (clk_rx),
        .o_clk_ddr      (clk_ddr),
        .o_rst_rx       (rst_rx)
    );

    wire [19:0] pll_freq_cnt;
    wire        pll_meas_valid;

    pll_freq_check #(
        .GATE_CYCLES(100_000),  // 1ms @100MHz
        .CNT_MIN    (147_000),  // ???????clk_rx??50M ??????? ??????????????????????
        .CNT_MAX    (153_000),
        .CNT_W      (20)
    ) u_pll_freq_check (
        .clk_ref  (clk_sys),
        .rst      (rst_sys),
        .clk_meas (clk_rx[0]),   // ??????????clk_rx(150M, ??????????????; ????????????????????clk_check_pll
        .valid    (pll_meas_valid),
        .freq_cnt (pll_freq_cnt),
        .pll_ok   (pll_ok)
    );

    reg rst_220M;
    reg [9:0] cnt_rst_glb;
    
    always @(posedge clk_220M or negedge mmcm0_locked) begin
        if (!mmcm0_locked) begin
            cnt_rst_glb <= 10'd0;
            rst_220M    <= 1'b1;
        end else if (cnt_rst_glb[9] == 1'b0) begin
            cnt_rst_glb <= cnt_rst_glb + 1'b1;
            rst_220M    <= 1'b1;
        end else begin
            rst_220M    <= 1'b0;
        end
    end

    localparam [3:0] TRAIN_LOAD_IDLE        = 4'd0;
    localparam [3:0] TRAIN_LOAD_MSB_FIRST   = 4'd1;
    localparam [3:0] TRAIN_LOAD_ADC_DEPTH   = 4'd2;
    localparam [3:0] TRAIN_LOAD_WORD        = 4'd3;
    localparam [3:0] TRAIN_LOAD_STABLE_LEN  = 4'd4;
    localparam [3:0] TRAIN_LOAD_CHAN_VLD    = 4'd5;
    localparam [3:0] TRAIN_LOAD_PULSE_HOLD  = 4'd6;

    reg        cmd_train_dly;
    reg [3:0]  train_load_state;
    reg [3:0]  train_pulse_hold_cnt;
    reg        train_cmd_loaded;
    reg        train_msb_first_en;
    reg [7:0]  train_adc_depth;
    reg [15:0] train_word;
    reg [6:0]  train_stable_len_half;
    reg [15:0] train_chan_vld;

    wire cmd_train_rise = cmd_train & ~cmd_train_dly ;  // ??& pll_ok

    // Training parameters must be presented from registers.  A train command
    // first loads these registers one item per clk_sys cycle, then generates a
    // registered start pulse for training_top_0.  training_top_0 starts on the
    // falling edge of i_cmd_start_train, so the registers are already stable.
    always @(posedge clk_sys) begin
        if (rst_sys) begin
            cmd_train_dly          <= 1'b0;
            train_load_state       <= TRAIN_LOAD_IDLE;
            train_pulse_hold_cnt   <= 4'd0;
            train_cmd_loaded       <= 1'b0;
            train_started          <= 1'b0;
            train_msb_first_en     <= 1'b0;
            train_adc_depth        <= 8'd0;
            train_word             <= 16'd0;
            train_stable_len_half  <= 7'd0;
            train_chan_vld         <= 16'd0;
        end else begin
            cmd_train_dly <= cmd_train;

            // When sensor power is turned off, allow a fresh training on next power-up.
            if (!cmd_power_on) begin
                train_started <= 1'b0;
            end

            case (train_load_state)
                TRAIN_LOAD_IDLE: begin
                    train_cmd_loaded     <= 1'b0;
                    train_pulse_hold_cnt <= 4'd0;
                    // Only the first vio_cmd_train rising edge starts training.
                    // Later 0->1 edges are image_gate only and must not retrigger training.
                    if (cmd_train_rise && (!train_started || pc_cmd_train)) begin
                        train_started    <= 1'b1;
                        train_load_state <= TRAIN_LOAD_MSB_FIRST;
                    end
                end

                TRAIN_LOAD_MSB_FIRST: begin
                    train_msb_first_en <= pc_reg_msb_first_en;
                    train_cmd_loaded   <= 1'b1;
                    train_load_state   <= TRAIN_LOAD_ADC_DEPTH;
                end

                TRAIN_LOAD_ADC_DEPTH: begin
                    train_adc_depth  <= (pc_adc_depth != 8'd0) ? pc_adc_depth : 8'd12;
                    train_load_state <= TRAIN_LOAD_WORD;
                end

                TRAIN_LOAD_WORD: begin

                    train_word       <= (pc_reg_training_word != 16'd0) ?
                                        pc_reg_training_word : 16'h086E;
                    train_load_state <= TRAIN_LOAD_STABLE_LEN;
                end

                TRAIN_LOAD_STABLE_LEN: begin
                    // Official CC2405 STD top.v uses stable_len_half=5 for
                    // this LVDS data rate.  A smaller eye-margin value can
                    // pass the static training word yet corrupt/clamp image
                    // samples during readout.
                    train_stable_len_half <= 7'd5;
                    train_load_state      <= TRAIN_LOAD_CHAN_VLD;
                end

                TRAIN_LOAD_CHAN_VLD: begin
                    train_chan_vld   <= (pc_chan_vld != 16'd0) ? pc_chan_vld : 16'hFFFF;
                    train_load_state <= TRAIN_LOAD_PULSE_HOLD;
                end

                TRAIN_LOAD_PULSE_HOLD: begin
                    if (train_pulse_hold_cnt == 4'd7) begin
                        train_cmd_loaded     <= 1'b0;
                        train_pulse_hold_cnt <= 4'd0;
                        train_load_state     <= TRAIN_LOAD_IDLE;
                    end else begin
                        train_pulse_hold_cnt <= train_pulse_hold_cnt + 1'b1;
                    end
                end

                default: begin
                    train_cmd_loaded <= 1'b0;
                    train_load_state <= TRAIN_LOAD_IDLE;
                end
            endcase
        end
    end

    training_top u_training_top_0 (
        .clk_rx            (clk_rx),       // 150M Hz
        .clk_ddr           (clk_ddr),    //600M Hz
        .rst_rx            (rst_rx),  

        .clk_sys           (clk_sys),    //100M Hz
        .rst_sys           (rst_sys),

        .clk_glb           (clk_220M),
        .rst_glb           (rst_220M),

        .i_SENSOR_DATA_P   (i_SENSOR_DATA_P),
        .i_SENSOR_DATA_N   (i_SENSOR_DATA_N),

        .i_msb_first_en    (train_msb_first_en),

        .i_adc_depth       (train_adc_depth),

        .i_training_word   (train_word),

        .i_stable_len_half (train_stable_len_half),

        .i_chan_vld        (train_chan_vld),

        .i_cmd_start_train (train_cmd_loaded),

        .o_dat_vld         (train_dat_vld),
        .o_dat             (train_dat),
        .o_train_done      (train_done),
        .o_train_err       (train_err),
        .o_fifo_train_wen  (fifo_train_wen),
        .o_fifo_train_din  (fifo_train_din),
        .o_lane8_align_dbg  (train_lane8_align_dbg)
    );

    // =========================================================================
    // USB 3.0 Image Transmission Path
    // pixel_12to16 (clk_220M) -> async FIFO -> usb30_interface (clk_100M)
    // =========================================================================

    // --- USB test pattern: 12-bit block image, bypass sensor/training path ---
    // Set TEST_PATTERN=1 to bypass training_top for USB link verification.
    // Generates a 2048x2048 12-bit grayscale image divided into 8x8 blocks.
    // Block gray value increases in row-major order from top-left:
    //   top-left=0, top-right=448, bottom-left=3584, bottom-right=4032.
    localparam TEST_PATTERN = 0;
    localparam BYPASS_DDR  = DEBUG_BYPASS_DDR;  // first validate sensor/pixel -> USB before DDR cache

    reg [11:0]  tp_cnt;
    reg [191:0] tp_dat;
    reg [15:0]  tp_vld;
    reg [19:0]  tp_div;   // slow down to ~220M/16 = ~13.75M pixels/s per ch

    always @(posedge clk_220M) begin
        if (rst_220M) begin
            tp_cnt <= 12'd0;
            tp_div <= 20'd0;
            tp_vld <= 16'd0;
            tp_dat <= 192'd0;
        end else begin
            if (tp_div == 20'd15) begin   // 16 cycles per valid beat
                tp_div <= 20'd0;
                tp_cnt <= tp_cnt + 12'd1;
                tp_vld <= 16'hFFFF;
                // same counter value on all 16 channels
                tp_dat <= {16{tp_cnt}};
            end else begin
                tp_div <= tp_div + 20'd1;
                tp_vld <= 16'd0;
            end
        end
    end

    // Image path enable is not an exposure control.
    // Exposure is controlled by o_VSYNC; frame writeout is started by SOF inside
    // g6504_frame_assembler.  This gate only allows the trained sensor stream to
    // reach the frame parser after configuration/training are valid.
    // train_done is only a one-cycle pulse, so use the latched train_started
    // plus train_err status as the persistent stream gate.
    wire image_gate = cfg_done && train_started && !train_err;
    wire [15:0]  pser_vld = TEST_PATTERN ? tp_vld : (image_gate ? train_dat_vld : 16'd0);
    wire [191:0] pser_dat = TEST_PATTERN ? tp_dat : train_dat;

    // --- Frame assembler ---
    // Sensor STD-mode path: filter SOF/SOL/EOL/EOF and assemble 8-row stripes in row-major order.
    // TEST_PATTERN keeps the old raw serializer path because the test ramp has no sensor sync codes.
    wire        raw_ser_wr_en;
    wire [31:0] raw_ser_dout;
    wire        pc_stitch_wr_en;
    wire [31:0] pc_stitch_dout;
    wire        pc_stitch_busy;
    wire        pc_stitch_frame_active;
    wire        pc_stitch_line_active;
    wire        pc_stitch_frame_done;
    wire [12:0] pc_stitch_x;
    wire [12:0] pc_stitch_y;
    wire [3:0]  pc_stitch_phase;
    wire [15:0] pc_stitch_status;
    // Ported K7 hardware reorder pipeline (replaces beat_gather + frame_assembler):
    //   train_dat -> lane swap -> lval_synccode_top -> lvds_imagebuffer -> 384->32
    wire        rio_wr_en;
    wire [31:0] rio_dout;
    wire        rio_busy;
    wire [31:0] rio_dbg;
    wire        rio_wide_vld;
    wire [383:0] rio_wide_dat;
    wire        rio_sync_vld;
    wire [191:0] rio_sync_dat;
    wire [191:0] lane_fifo_din;
    wire [15:0] lane_fifo_wen, lane_fifo_rst;
    wire [31:0] rio_sync_dbg;
    wire [31:0] rio_imgbuf_dbg;
    wire [191:0] rio_lval_fifo_dout;
    wire [63:0] rio_lval_internal_dbg;
    wire [63:0] rio_lval_lane0_dbg;
    wire [63:0] rio_lval_lane8_dbg;

    wire        direct_pc_stitch = DEBUG_PC_STITCH_USB && BYPASS_DDR && !TEST_PATTERN;
    wire        ser_wr_en = TEST_PATTERN ? raw_ser_wr_en : (direct_pc_stitch ? pc_stitch_wr_en : rio_wr_en);
    wire [31:0] ser_dout  = TEST_PATTERN ? raw_ser_dout  : (direct_pc_stitch ? pc_stitch_dout  : rio_dout);

    // Per-frame reset pulse for the reorder pipeline.
    // K7_Demo drives lval_synccode/lvds_imagebuffer with the original
    // cmd_frame_req, not with an edge recovered from o_VSYNC.  Keep the same
    // contract here: when the VIO frame request is accepted, toggle a sys-clock
    // event and resynchronize it into clk_220M as a clean one-cycle pulse.
    // Do NOT derive this reset from o_VSYNC falling edge; that resets the
    // reorder pipeline at the end of exposure and can put SOF/SOL parsing in
    // the wrong frame phase.
    reg  [2:0] rio_frame_req_sync_220;
    always @(posedge clk_220M) begin
        if (rst_220M)
            rio_frame_req_sync_220 <= 3'b000;
        else
            rio_frame_req_sync_220 <= {rio_frame_req_sync_220[1:0], rio_frame_req_tgl_sys};
    end
    wire rio_cmd_frame_req = rio_frame_req_sync_220[2] ^ rio_frame_req_sync_220[1];

    pixel_12to16 #(
        .CHAN_NUM (16),
        .DAT_WIDTH(12)
    ) u_pixel_ser_raw (
        .clk      (clk_220M),
        .rst      (rst_220M),
        .i_dat_vld(pser_vld),
        .i_dat    (pser_dat),
        .o_wr_en  (raw_ser_wr_en),
        .o_dout   (raw_ser_dout)
    );

    g6504_frame_assembler #(
        .LANES     (16),
        .DAT_WIDTH (12),
        .IMG_WIDTH (2176),
        .BOT_BASE  (0),
        .TOP_BASE  (8),
        .BIG_END_EN(1'b0)
    ) u_pc_stitch_frame_assembler (
        .clk           (clk_220M),
        .rst           (rst_220M || !image_gate),
        .i_gate        (image_gate),
        .i_dat_vld     (pser_vld),
        .i_dat         (pser_dat),
        .o_wr_en       (pc_stitch_wr_en),
        .o_dout        (pc_stitch_dout),
        .o_busy        (pc_stitch_busy),
        .o_frame_active(pc_stitch_frame_active),
        .o_line_active (pc_stitch_line_active),
        .o_frame_done  (pc_stitch_frame_done),
        .o_x           (pc_stitch_x),
        .o_y           (pc_stitch_y),
        .o_phase       (pc_stitch_phase),
        .o_status      (pc_stitch_status)
    );

    g6504_img_reorder_top #(
        // Match the official STD sensor_protocol geometry.  The sensor emits
        // 2048 display rows, but the 32-channel image buffer/DDR path handles
        // them as 512 row groups (4 display rows per group).
        .WIN_ROW_LEN    (16'd512),
        .LVAL_LEN    (16'd1088),
        .LVALOUT_LEN (16'd1088),
        .FRAME_NUM   (16'd1)
    ) u_img_reorder (
        .clk            (clk_220M),
        .rst            (rst_220M || !image_gate),
        .i_cmd_frame_req(rio_cmd_frame_req),
        .i_synccode_1st (pc_synccode_1st),
        .i_synccode_2nd (pc_synccode_2nd),
        .i_synccode_3rd (pc_synccode_3rd),
        .i_synccode_4th_sof(pc_synccode_4th_sof),
        .i_synccode_4th_sol(pc_synccode_4th_sol),
        .i_lval_len     (pc_lval_len),
        .i_lvalout_len  (pc_lval_len_buf),
        .i_win_row_len  (pc_win_row_len),
        .i_frame_num    (pc_reg_frame_num),
        .i_hdr_mode_en  (pc_reg_hdr_mode_en),
        .i_flip_v_en    (pc_reg_flip_v_en),
        .i_flip_h_en    (pc_reg_flip_h_en),
        .i_n_mux        (pc_reg_n_mux),
        .i_w_mux        (pc_reg_w_mux),
        .i_dat_vld      (pser_vld),
        .i_dat          (pser_dat),
        .o_wr_en        (rio_wr_en),
        .o_dout         (rio_dout),
        .o_busy         (rio_busy),
        .o_wide_vld     (rio_wide_vld),
        .o_wide_dat     (rio_wide_dat),
        .o_lane_fifo_din(lane_fifo_din),
        .o_lane_fifo_wen(lane_fifo_wen),
        .o_lane_fifo_rst(lane_fifo_rst),
        .o_sync_vld     (rio_sync_vld),
        .o_sync_dat     (rio_sync_dat),
        .o_sync_dbg      (rio_sync_dbg),
        .o_imgbuf_dbg    (rio_imgbuf_dbg),
        .o_lval_fifo_dout(rio_lval_fifo_dout),
        .o_lval_internal_dbg(rio_lval_internal_dbg),
        .o_lval_lane0_dbg(rio_lval_lane0_dbg),
        .o_lval_lane8_dbg(rio_lval_lane8_dbg),
        .o_dbg          (rio_dbg)
    );

    // --- Async FIFO: wr 32bit@clk_220M, rd 32bit@clk_100M, depth 4096 ---
    // Vivado IP: FIFO Generator, Indep Clocks Block RAM, Standard FIFO
    //   Write Width=32, Read Width=32, Write Depth=4096
    //   Enable Reset Synchronization=YES, Reset Type=Asynchronous
    //   Data Count=Read Data Count, Read Data Count Width=13
    wire [31:0] afifo_dout;
    wire        afifo_rd_en;
    wire        afifo_full;
    wire        afifo_empty;
    wire [12:0] afifo_rd_cnt;

    wire        ddr_ui_clk;
    wire        ddr_ui_rst;
    wire        ddr_init_done;
    wire        ddr_in_rd_en;
    wire        ddr_img_dat_vld;
    wire [31:0] ddr_img_dat;
    wire        ddr_img_tx_en;
    wire [15:0] ddr_dbg;
    wire        usb_tx_fifo_ren;
    wire        usb_tx_fifo_full;
    wire        usb_tx_fifo_empty;
    wire        usb_tx_fifo_wr_en;


    img_cdc_fifo u_img_cdc_fifo (
        .rst          (rst_220M),
        .wr_clk       (clk_220M),
        .din          (ser_dout),
        .wr_en        (BYPASS_DDR ? (ser_wr_en & ~afifo_full) : 1'b0),
        .full         (afifo_full),
        .rd_clk       (BYPASS_DDR ? clk_100M : ddr_ui_clk),
        .rd_en        (afifo_rd_en),
        .dout         (afifo_dout),
        .empty        (afifo_empty),
        .rd_data_count(afifo_rd_cnt)
    );

    // --- 100 MHz domain test image (TEST_PATTERN=1 bypass: skip 220M domain) ---
    // Each 2048x2048 frame is preceded by one G6504-style SOF sync code:
    //   12-bit words: FFFh, 000h, 000h, 800h
    //   USB 32-bit words, low 16-bit pixel first: 0000_0FFFh, 0800_0000h
    localparam integer TP_IMG_W       = 2048;
    localparam integer TP_IMG_H       = 2048;
    localparam integer TP_BLOCK_SIZE  = 256;   // 2048 / 8
    localparam integer TP_WORDS_LINE  = 1024;  // 2 pixels per 32-bit USB word

    reg [10:0] tp100_x;       // even pixel x: 0,2,4,...,2046
    reg [10:0] tp100_y;       // row: 0..2047
    reg [3:0]  tp100_div;
    reg [1:0]  tp100_sof_idx; // 0,1: emit SOF words; 2: emit image payload
    reg [31:0] tp100_word;
    wire [2:0]  tp100_blk_x = tp100_x[10:8];   // /256
    wire [2:0]  tp100_blk_y = tp100_y[10:8];   // /256
    wire [5:0]  tp100_blk_i = {tp100_blk_y, 3'b000} + {3'b000, tp100_blk_x};
    wire [11:0] tp100_gray  = {tp100_blk_i, 6'b000}; // block index * 64

    always @(posedge clk_100M) begin
        if (rst_sys) begin
            tp100_x   <= 11'd0;
            tp100_y   <= 11'd0;
            tp100_div <= 4'd0;
            tp100_sof_idx <= 2'd0;
            tp100_word <= 32'd0;
            tp100_vld <= 1'b0;
        end else begin
            if (tp100_div == 4'd15) begin
                tp100_div <= 4'd0;
                tp100_vld <= 1'b1;
                if (tp100_sof_idx == 2'd0) begin
                    tp100_word <= 32'h0000_0FFF;
                    tp100_sof_idx <= 2'd1;
                end else if (tp100_sof_idx == 2'd1) begin
                    tp100_word <= 32'h0800_0000;
                    tp100_sof_idx <= 2'd2;
                end else begin
                    tp100_word <= {4'b0, tp100_gray, 4'b0, tp100_gray};
                    if (tp100_x == 11'd2046) begin
                        tp100_x <= 11'd0;
                        if (tp100_y == 11'd2047) begin
                            tp100_y <= 11'd0;
                            tp100_sof_idx <= 2'd0;
                        end else begin
                            tp100_y <= tp100_y + 11'd1;
                        end
                    end else begin
                        tp100_x <= tp100_x + 11'd2;
                    end
                end
            end else begin
                tp100_div <= tp100_div + 4'd1;
                tp100_vld <= 1'b0;
            end
        end
    end

    // --- DDR3 frame/chunk buffer between image CDC FIFO and USB TX ---
    assign afifo_rd_en = (TEST_PATTERN == 0) ? (BYPASS_DDR ? (!afifo_empty && (tx_fifo_cnt < 15'd14000)) : 1'b0) : 1'b0;

    wire        ddr_k7_wr_done;
    wire        ddr_k7_rd_req;
    wire        ddr_k7_rd_req_ui;
    wire        ddr_k7_dat_vld;
    wire [383:0] ddr_k7_dat;
    wire        ddr_k7_rowbuf_rdy;
    wire        ddr_k7_rowbuf_vld_any;
    wire        ddr_k7_rowbuf_rd_en_any;
    wire        ddr_k7_tx_row_done;
    wire        ddr_k7_frame_done;
    wire [31:0] ddr_k7_dbg0;
    wire [31:0] ddr_k7_dbg1;

    // --- K7_Demo gold-standard DDR image path ---
    // Directly reuses K7 image_ddr_driver/mig_driver for DDR input and
    // ddr_dout_buf_0 + image_dat_tx_0 for row-level DDR output.
    g6504_k7_ddr_image_path #(
        // Official STD imageprocess geometry:
        // 272 x 384-bit beats per DDR row group, 512 row groups per frame.
        // Each group contains four 2176-pixel display rows, so the USB row
        // length at this stage is 4*2176=8704 pixels.  Total payload remains
        // 2176*2048*2 = 8,912,896 bytes.
        .LVAL_LEN_DDR   (16'd272),
        .WIN_ROW_LEN    (16'd512),
        .WIN_COL_LEN_TX (16'd8704),
        .FRAME_NUM      (16'd1)
    ) u_g6504_k7_ddr_image_path (
        .clk_img              (clk_220M),
        .rst_img              (rst_220M || !image_gate),
        .clk_sys              (clk_100M),
        .rst_sys              (rst_sys),
        .clk_200m             (clk_200M),
        .i_cmd_frame_req      (rio_cmd_frame_req),
        .i_wide_vld           (image_gate ? rio_wide_vld : 1'b0),
        .i_wide_dat           (rio_wide_dat),
        .i_hdr_mode_en        (pc_reg_hdr_mode_en),
        .i_lval_len_ddr       (pc_lval_len_ddr),
        .i_win_row_len        (pc_win_row_len_ddr),
        .i_win_col_len        (pc_win_col_len),
        .i_frame_num          (pc_reg_frame_num),
        .i_frame_num_tx       (pc_frame_num_tx),
        .i_tx_fifo_cnt        (tx_fifo_cnt[12:0]),
        .o_img_dat_vld        (ddr_img_dat_vld),
        .o_img_dat            (ddr_img_dat),
        .o_img_tx_en          (ddr_img_tx_en),
        .o_frame_done         (ddr_k7_frame_done),
        .o_ui_clk             (ddr_ui_clk),
        .o_ui_rst             (ddr_ui_rst),
        .o_init_calib_complete(ddr_init_done),
        .o_ddr_wr_done        (ddr_k7_wr_done),
        .o_ddr_rd_req         (ddr_k7_rd_req),
        .o_dbg_rd_req_ui      (ddr_k7_rd_req_ui),
        .o_ddr_dat_vld        (ddr_k7_dat_vld),
        .o_ddr_dat            (ddr_k7_dat),
        .o_rowbuf_rdy         (ddr_k7_rowbuf_rdy),
        .o_rowbuf_vld_any     (ddr_k7_rowbuf_vld_any),
        .o_rowbuf_rd_en_any   (ddr_k7_rowbuf_rd_en_any),
        .o_tx_row_done        (ddr_k7_tx_row_done),
        .o_dbg0               (ddr_k7_dbg0),
        .o_dbg1               (ddr_k7_dbg1),
        .o_DDR3_ADDR          (o_DDR3_ADDR),
        .o_DDR3_BA            (o_DDR3_BA),
        .o_DDR3_CAS_N         (o_DDR3_CAS_N),
        .o_DDR3_CK_N          (o_DDR3_CK_N),
        .o_DDR3_CK_P          (o_DDR3_CK_P),
        .o_DDR3_CKE           (o_DDR3_CKE),
        .o_DDR3_RAS_N         (o_DDR3_RAS_N),
        .o_DDR3_RESET_N       (o_DDR3_RESET_N),
        .o_DDR3_WE_N          (o_DDR3_WE_N),
        .io_DDR3_DQ           (io_DDR3_DQ),
        .io_DDR3_DQS_N        (io_DDR3_DQS_N),
        .io_DDR3_DQS_P        (io_DDR3_DQS_P),
        .o_DDR3_CS_N          (o_DDR3_CS_N),
        .o_DDR3_DM            (o_DDR3_DM),
        .o_DDR3_ODT           (o_DDR3_ODT)
    );

    assign ddr_dbg = {
        ddr_k7_wr_done,          // [15]
        ddr_k7_rd_req,           // [14]
        ddr_k7_dat_vld,          // [13]
        ddr_k7_rowbuf_rdy,       // [12]
        ddr_k7_rowbuf_vld_any,   // [11]
        ddr_k7_rowbuf_rd_en_any, // [10]
        ddr_k7_tx_row_done,      // [9]
        ddr_k7_frame_done,       // [8]
        ddr_init_done,           // [7]
        7'd0
    };


    // Register output for img2tx path
    reg        img_dat_vld;
    reg [31:0] img_dat;

    always @(posedge clk_100M) begin
        if (rst_sys) begin
            img_dat_vld <= 1'b0;
            img_dat     <= 32'd0;
            img_tx_en   <= 1'b0;
        end else if (TEST_PATTERN) begin
            img_dat_vld <= tp100_vld;
            img_dat     <= tp100_word;
            img_tx_en   <= 1'b1;
        end else if (BYPASS_DDR) begin
            img_dat_vld <= afifo_rd_en;
            img_dat     <= afifo_dout;
            img_tx_en   <= !afifo_empty;
        end else begin
            img_dat_vld <= ddr_img_dat_vld;
            img_dat     <= ddr_img_dat;
            img_tx_en   <= ddr_img_tx_en;
        end
    end

    // Count words handed from DDR/output selector into the USB TX FIFO path.
    // This stays in clk_100M with img_dat_vld, so it measures the actual PC-bound word stream.
    always @(posedge clk_100M) begin
        if (rst_sys)
            ddr_to_usb_word_total <= 32'd0;
        else if (img_dat_vld)
            ddr_to_usb_word_total <= ddr_to_usb_word_total + 32'd1;
    end

    // --- USB 3.0 Interface ---
    // Use a namespaced, otherwise unmodified copy of the known-working vendor
    // transport. The receive FIFO is consumed by rx_pkg_decoder above.
    gpx_usb30_interface_official u_usb30_interface (
        .clk              (clk_100M),
        .rst              (rst_sys),
        .i_fullauto_en     (1'b0),
        .i_fullauto_vld    (1'b0),
        .i_fullauto_dat    (32'd0),
        .i_FX3_FLAGA      (i_FX3_FLAGA),
        .i_FX3_FLAGB      (i_FX3_FLAGB),
        .o_FX3_SLCS_N     (o_FX3_SLCS_N),
        .o_FX3_SLWR_N     (o_FX3_SLWR_N),
        .o_FX3_SLRD_N     (o_FX3_SLRD_N),
        .o_FX3_SLOE_N     (o_FX3_SLOE_N),
        .o_FX3_PKTEND_N   (o_FX3_PKTEND_N),
        .o_FX3_A          (o_FX3_A),
        .io_FX3_DB        (io_FX3_DB),
        .o_USB_RESET      (o_USB_RESET),
        .i_rx_fifo_rd_en  (usb_rx_fifo_rd_en),
        .o_rx_fifo_rd_dat (usb_rx_fifo_rd_dat),
        .o_rx_fifo_rd_empty(usb_rx_fifo_empty),
        .i_ack2tx_tx_en   (1'b0),
        .i_ack2tx_dat_vld (1'b0),
        .i_ack2tx_dat     (32'd0),
        .i_rlt2tx_tx_en   (usb_sensor_tx_en),
        .i_rlt2tx_dat_vld (usb_sensor_tx_valid),
        .i_rlt2tx_dat     (usb_sensor_tx_word),
        .i_img2tx_tx_en   (img_tx_en),
        .i_img2tx_dat_vld (img_dat_vld),
        .i_img2tx_dat     (img_dat),
        .o_tx_fifo_cnt    (official_tx_fifo_cnt),
        .o_dbg_tx_word_strobe(fx3_tx_word_strobe),
        .o_dbg_tx_word_accepted(fx3_tx_word_accepted),
        .o_dbg_tx_word_is_image(fx3_tx_word_is_image),
        .o_dbg_tx_word    (fx3_tx_word),
        .o_dbg_tag_fifo_count(fx3_tag_fifo_count),
        .o_dbg_tag_fifo_full(fx3_tag_fifo_full),
        .o_dbg_tag_fifo_empty(fx3_tag_fifo_empty)
    );

    assign tx_fifo_cnt          = {2'b00, official_tx_fifo_cnt};
    assign usb_tx_fifo_ren      = 1'b0;
    assign usb_tx_fifo_full     = 1'b0;
    assign usb_tx_fifo_empty    = (official_tx_fifo_cnt == 13'd0);
    assign usb_tx_fifo_wr_en    = img_dat_vld | usb_sensor_tx_valid;
    assign usb_dbg              = {16'd0, official_tx_fifo_cnt, i_FX3_FLAGB,
                                   usb_sensor_tx_en, img_tx_en};
    assign usb_tx_word_total    = 32'd0;
    assign usb_pkt_total        = 32'd0;
    assign flagb_low_total      = 32'd0;

    // Match the known-working K7_Demo GPIF clock topology exactly.
    assign o_FX3_PCLK = clk_100M;

    // =========================================================================
    // Exposure-to-FX3 timing instrumentation.  The geometry comes from the
    // same configured values used to construct the official AA55 frame header.
    wire [15:0] timing_frame_columns = (pc_win_col_len != 16'd0) ? pc_win_col_len : 16'd2176;
    wire [15:0] timing_frame_rows    = (pc_win_row_len != 16'd0) ? pc_win_row_len : 16'd2048;
    wire [15:0] timing_frame_count   = (pc_frame_num_tx != 16'd0) ? pc_frame_num_tx : 16'd1;
    // Debug geometry pipeline: configuration settles before FRAME_REQ.
    // Break the cascaded multiplier path before crossing to the DDR UI clock.
    reg [31:0] timing_pixels_per_subframe;
    reg [47:0] timing_pixel_containers;
    always @(posedge clk_sys) begin
        if (rst_sys) begin
            timing_pixels_per_subframe <= 0;
            timing_pixel_containers <= 0;
        end else begin
            timing_pixels_per_subframe <= timing_frame_columns * timing_frame_rows;
            timing_pixel_containers <= timing_pixels_per_subframe * timing_frame_count;
        end
    end
    
    wire [31:0] timing_expected_image_words = timing_pixel_containers[32:1];

    g6504_rough_ila u_rough_timing (
        .clk(clk_sys), .rst(rst_sys), .vsync(vsync_out_r),
        .rx_clk(clk_220M), .rx_rst(rst_220M), .rx_start(rio_cmd_frame_req),
        .lane_fifo_wen(lane_fifo_wen), .lane_fifo_rst(lane_fifo_rst),
        .ddr_in_valid(image_gate && rio_wide_vld),
        .ui_clk(ddr_ui_clk), .ui_rst(ddr_ui_rst),
        .request_toggle(rio_frame_req_tgl_sys), .ddr_out_valid(ddr_k7_dat_vld),
        .expected_fx3_words(timing_expected_image_words),
        .fx3_strobe(fx3_tx_word_strobe),
        .fx3_data(fx3_tx_word),
        .lane_fifo_din(lane_fifo_din), .ddr_in_data(rio_wide_dat), .ddr_out_data(ddr_k7_dat),
        .ddr_read_request(ddr_k7_rd_req_ui), .ddr_calibrated(ddr_init_done),
        .ddr_read_request_sys(ddr_k7_rd_req), .fx3_flagb(i_FX3_FLAGB),
        .fx3_slcs_n(o_FX3_SLCS_N), .fx3_slwr_n(o_FX3_SLWR_N),
        .fx3_slrd_n(o_FX3_SLRD_N), .fx3_sloe_n(o_FX3_SLOE_N),
        .fx3_pktend_n(o_FX3_PKTEND_N), .fx3_addr(o_FX3_A)
    );
endmodule
