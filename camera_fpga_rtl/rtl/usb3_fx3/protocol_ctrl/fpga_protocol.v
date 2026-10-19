`timescale 1ns / 1ps
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : fpga_protocol
// --File      : fpga_protocol.v
// --Author    : 
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : 
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2022.07.14   danyang.li      create
// --1.1       2022.12.12   Weihan.li       Modify
// --1.2       2024.04.30   Guangcai Ding   Modify Code Style
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module fpga_protocol #(
    parameter    POWER_STEP    = 6    // 1--30, max30
)(
    // clock & reset
    input                                    clk                    ,
    input                                    rst                    ,

    // protocol input signa
    input                                   i_prot_vld             ,    // FPGA protocol from PC(rx_pkg_decoder.v), valid of address and data
    input            [15:0]                 i_prot_addr            ,    // FPGA protocol from PC(rx_pkg_decoder.v), address
    input            [15:0]                 i_prot_dat             ,    // FPGA protocol from PC(rx_pkg_decoder.v), data

    // Used for universal
    output    reg                           o_cmd_bus_vld          ,    // command valid
    output    reg    [15:0]                 o_cmd_bus              ,    // command
    output    reg    [15:0]                 o_reg_addr             ,    // Start address for write and read SPI operation
    output    reg    [15:0]                 o_reg_len              ,    // Length of write and read SPI operation, (actuals length).
    output    reg    [15:0]                 o_frame_num            ,    // Sensor output frame numbers ,use it if sensor has no this register           ,
    output    reg    [15:0]                 o_dp_control           ,    // DPs’ address mapping and data. group: [15:12], addr: [11:10], instruction: [9:8], data: [7:0]
    output    reg                           o_otp_dat_vld          ,    // clk domain. o_otp_dat valid flag
    output    reg    [7:0]                  o_otp_dat              ,    // clk domain. Configuring OTP’s data. Totally 8-byte for OTP,64-byte for FTP. send order from low to high
    output    reg                           o_dac_dat_vld          ,    // clk domain. o_dac_dat valid flag
    output    reg    [15:0]                 o_dac_dat              ,    // clk domain. Configuring dac’s data //detail refer to <FPGA-PC Comunication Protocol>
    output    reg    [15:0]                 o_testimage_fpga_typ   ,    // 1: transfer FPGA test image to PC, 0: transfer Sensor image to PC, 
    output    reg                           o_i2c_en               ,    // way of write and read sensor registers. 1: I2C, 0:SPI
    output    reg                           o_slamode              ,    // slave address control, for I2C.
    output    reg                           o_image_lag_en         ,    // test image lag enable
    output    reg                           o_FAN_CTRL             ,    // fan control. default is 1
    output    reg                           o_TEXP_OTP_EN          ,    // used to distinguish When multiple signals(input and output) share a pin
    output    reg                           o_image_ctr_en         ,    // 0: burst mode,1: continue mode
    output    reg                           o_switch_bit_en        ,    // switch FPGA program enable
    output    reg    [3:0]                  o_switch_bit_num       ,    // switch FPGA program, zynq:0to15, FPGA: Configure values according to requirements
    output    reg                           o_shift_en             ,    // shift enable
    output    reg    [10:0]                 o_time_out_frame_s     ,    // +20s is the max wait time from frame request to ddr received done
    output    reg    [15:0]                 o_lag_frame_start      ,
    output    reg    [15:0]                 o_lag_frame_num        ,
    output    reg    [7:0]                  o_lag_vld_lane_sel     ,
    output           [31:0]                 o_time_sensor_rst_clk  ,
    output           [31:0]                 o_time_sensor_set_clk  ,
    output           [POWER_STEP*32-1:0]    o_time_pwr_on_clk      ,
    output           [POWER_STEP*32-1:0]    o_time_pwr_off_clk     ,
    output    reg                           o_flash_din_vld        ,
    output    reg    [7:0]                  o_flash_din            ,
    output    reg    [3:0]                  o_flash_function       ,
    output    reg    [15:0]                 o_flash_len            ,
    output    reg    [47:0]                 o_lag_on_delay         ,
    output    reg    [47:0]                 o_lag_on_time          ,

    // Used for different project
    output    reg    [31:0]                 o_frame_linetime       ,    // texp vsync
    output    reg    [15:0]                 o_frame_clkpix         ,    // texp vsync
    output    reg    [31:0]                 o_inte_linetime        ,    // texp vsync
    output    reg    [15:0]                 o_inte_clkpix          ,    // texp vsync
    output    reg                           o_trigger_pulse_en     ,
    output    reg    [1:0]                  o_freq_sel             ,
    output    reg                           o_lp_mode_en           ,
    output    reg    [15:0]                 o_lp_win_len_time      ,
    output    reg    [15:0]                 o_lp_mode_offset0      ,
    output    reg    [31:0]                 o_lp_mode_twait        ,
    output    reg    [31:0]                 o_lp_mode_twarming     ,
    output    reg    [15:0]                 o_lp_line_time         ,
    output    reg    [15:0]                 o_lp_hsync_wait0       ,
    output    reg    [15:0]                 o_lp_hsync_wait1       ,
    output    reg    [31:0]                 o_pwr_seq_wait0        ,    // power on seq
    output    reg    [31:0]                 o_pwr_seq_wait1        ,    // power on seq
    output    reg    [31:0]                 o_pwr_seq_wait2        ,    // power on seq
    output    reg                           o_reg_test_mode_en     ,

    // Used for auto operation
    output    reg    [15:0]                 o_auto_step_en         ,    // when auto configuration, bit2=1: rd default enable, bit3=1: power on enbanle 
    output    reg    [15:0]                 o_image_type           ,    // 0x0001:  usb 16 bit size original image output 
                                                                        // 0x0002:  usb 16 bit size average image output
                                                                        // 0x0004:  usb 16 bit size noise image output (x 128).
                                                                        // 0x000C:  usb 16 bit size noise image output, keep integers delate decimals 
    output    reg    [15:0]                 o_image_gain           ,    // not all sensors need it.  0x0001: LG, 0x0002: HG, 0x0003: LG&HG, 0x0004: STD 
    output    reg    [15:0]                 o_frame_start          ,    // number of frame drops at begin, 0 to o_frame_num-1
    output    reg    [15:0]                 o_frame_num_vld        ,    // effective frames received, FPGA transfer to PC
    output    reg                           o_sub_eb_en            ,    // 1: open sub EB function, 0: close sub EB function
    output    reg    [15:0]                 o_fpga_project         ,    // when a FPGA project is compatible with different sensor, use it to distinguish
    output    reg    [15:0]                 o_mode_sel             ,    // auto configuration operating config .ini selection
    output    reg                           o_white_balance_en     ,    // white balance enable. 1: open, 0:close
    output    reg                           o_ccm_en               ,    // CCM enable. 1: open, 0:close
    output    reg                           o_color_composite_en   ,    // color composite enable. 1: open, 0:close
    output    reg    [15:0]                 o_check_pll_times      ,    // check PLL times in standard frequency
    output    reg    [7:0]                  o_opt_brt              ,    // brightness of OPT light source, shared by 4 channels
    output    reg    [2:0]                  o_opt_chan             ,    // OPT channel to be set,Chan1=1,chan2=2,chan3=3,chan4=4
    output    reg    [5:0]                  o_os_group_en          ,
    output    reg    [47:0]                 o_os_idrive
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

reg     [3:0]                  cnt_switch_pulse    ;
reg     [15:0]                 time_sensor_rst_1us ;
reg     [15:0]                 time_sensor_set_1us ;
reg     [POWER_STEP*16-1:0]    time_pwr_on_100us   ;
reg     [POWER_STEP*16-1:0]    time_pwr_off_100us  ;
reg                            cmd_bus_vld_pre     ;
reg     [1:0]                  cmd_bus_vld_pre_dly ;
reg     [15:0]                 cmd_bus_pre         ;
reg     [31:0]                 cnt_cmd_bus         ;
reg                            fsm_cmd             ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

assign    o_time_sensor_rst_clk    = 100 * time_sensor_rst_1us;    // value 100 is base on frequency of input clk of time_sensor_rst_clk.v
assign    o_time_sensor_set_clk    = 100 * time_sensor_set_1us;    // value 100 is base on frequency of input clk of time_sensor_rst_clk.v

genvar    i;
generate
    for (i = 0; i < POWER_STEP; i = i+1) begin:pwr
        assign    o_time_pwr_on_clk[(i+1)*32-1:i*32]     = 10000*time_pwr_on_100us [(i+1)*16-1:i*16];    // value 10000 is base on frequency of input clk of time_sensor_rst_clk.v
        assign    o_time_pwr_off_clk[(i+1)*32-1:i*32]    = 10000*time_pwr_off_100us[(i+1)*16-1:i*16];    // value 10000 is base on frequency of input clk of time_sensor_rst_clk.v
    end
endgenerate

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main code
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------- decoder the fpga data ------------------------------------------------------

// Automatic command delay 1ms
always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        o_cmd_bus_vld      <= 1'b0;
        o_cmd_bus          <= 'd0 ;
        cmd_bus_vld_pre    <= 1'b0;
        cmd_bus_pre        <= 'd0 ;
        cnt_cmd_bus        <= 'd0 ;
        fsm_cmd            <= 1'b0;
    end
    else begin
        case(fsm_cmd)
            1'b0:   begin
                        if((i_prot_addr[15:0] == 16'h0000) && (i_prot_vld == 1'b1)) begin
                            if(i_prot_dat == 16'h1001) begin
                                cmd_bus_vld_pre    <= 1'b1;
                                cmd_bus_pre        <= i_prot_dat;
                                fsm_cmd            <= 1'b1;
                            end
                            else begin
                                o_cmd_bus_vld      <= 1'd1;
                                o_cmd_bus          <= i_prot_dat;
                            end
                        end
                        else begin
                            o_cmd_bus_vld      <= 1'b0;
                            cmd_bus_vld_pre    <= 1'b0;
                            cnt_cmd_bus        <= 'd0;
                        end
                    end
            1'b1:   begin
                        if(cnt_cmd_bus == 'd100_000) begin    // 1ms
                            o_cmd_bus_vld      <= 1'b1;
                            o_cmd_bus          <= cmd_bus_pre;
                            fsm_cmd            <= 1'b0;
                        end
                        else begin
                            cnt_cmd_bus        <= cnt_cmd_bus + 1'b1;
                        end
                    end
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        o_otp_dat_vld    <= 1'b0;
    end
    else begin
        if(i_prot_addr[15:0] == 16'h0006 && i_prot_vld == 1'b1) begin
            o_otp_dat_vld    <= 1'b1;
        end
        else begin
            o_otp_dat_vld    <= 1'b0;
        end
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        o_dac_dat_vld    <= 1'b0;
    end
    else begin
        if(i_prot_addr[15:0] == 16'h0005 && i_prot_vld == 1'b1) begin
            o_dac_dat_vld    <= 1'b1;
        end
        else begin
            o_dac_dat_vld    <= 1'b0;
        end
    end
end

always @(posedge clk or posedge rst)begin
    if (rst == 1) begin 
        o_switch_bit_en        <= 1'b0;
        cnt_switch_pulse       <= 'd0;
    end
    else begin 
        if(i_prot_addr[15:0] == 16'h000E && i_prot_vld == 1) begin
            o_switch_bit_en        <= 1'b1;
            cnt_switch_pulse       <= 'd0;
        end
        else if(cnt_switch_pulse == 'd7) begin
            o_switch_bit_en        <= 1'b0;
        end
        else begin
            cnt_switch_pulse <= cnt_switch_pulse + 1'b1 ;
        end
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        // o_cmd_bus                   <= 16'd0;
        o_reg_addr                     <= 16'd0;
        o_reg_len                      <= 16'd1;
        o_frame_num                    <= 16'd1;
        o_dp_control                   <= 16'd0;
        o_otp_dat                      <= 8'd0;
        o_dac_dat                      <= 16'd0;
        o_testimage_fpga_typ           <= 'd0;
        o_i2c_en                       <= 1'b0;
        o_slamode                      <= 1'b0;
        o_image_lag_en                 <= 1'b0;
        o_FAN_CTRL                     <= 1'b0;
        o_TEXP_OTP_EN                  <= 1'b0;
        o_image_ctr_en                 <= 1'b0;
        o_switch_bit_num               <= 4'd0;
        o_shift_en                     <= 1'b0;
        o_time_out_frame_s             <= 'd0;
        o_lag_frame_start              <= 'd0;
        o_lag_frame_num                <= 'd0;
        o_lag_vld_lane_sel             <= 'd0;
        time_sensor_rst_1us            <= 'd2;     // unit 1us
        time_sensor_set_1us            <= 'd2;     // unit 1us
        time_pwr_on_100us[15:0]        <= 16'd50;  // unit 100us  // power on 1st     // CC2405 GSEN6504BSI EVK    EN_VCC_LT  --> EN_VDDIO
        time_pwr_on_100us[31:16]       <= 16'd9;   // unit 100us  // power on 2nd     // CC2405 GSEN6504BSI EVK    EN_VDDIO   --> EN_VDDD
        time_pwr_on_100us[47:32]       <= 16'd27;  // unit 100us  // power on 3rd     // CC2405 GSEN6504BSI EVK    EN_VDDD    --> EN_VDDA
        time_pwr_on_100us[63:48]       <= 16'd28;  // unit 100us  // power on 4th     // CC2405 GSEN6504BSI EVK    EN_VDDA    --> EN_VDDSF
        time_pwr_on_100us[79:64]       <= 16'd29;  // unit 100us  // power on 5th     // CC2405 GSEN6504BSI EVK    EN_VDDSF   --> EN_VDDBIAS
        time_pwr_on_100us[95:80]       <= 16'd120;  // unit 100us  // power on 6th    // CC2405 GSEN6504BSI EVK    EN_VDDBIAS --> SYS_RST_N
        // time_pwr_on_100us[111:96]      <= 16'd8;   // unit 100us  // power on 7th
        // time_pwr_on_100us[127:112]     <= 16'd150; // unit 100us  // power on 8th
        // time_pwr_on_100us[143:128]     <= 16'd; // unit 100us  // power on 9th
        // time_pwr_on_100us[159:144]     <= 16'd; // unit 100us  // power on 10th
        // time_pwr_on_100us[175:160]     <= 16'd; // unit 100us  // power on 11th
        // time_pwr_on_100us[191:176]     <= 16'd; // unit 100us  // power on 12th
        // time_pwr_on_100us[207:192]     <= 16'd; // unit 100us  // power on 13th
        // time_pwr_on_100us[223:208]     <= 16'd; // unit 100us  // power on 14th
        // time_pwr_on_100us[239:224]     <= 16'd; // unit 100us  // power on 15th
        // time_pwr_on_100us[255:240]     <= 16'd; // unit 100us  // power on 16th
        // time_pwr_on_100us[271:256]     <= 16'd; // unit 100us  // power on 17th
        // time_pwr_on_100us[287:272]     <= 16'd; // unit 100us  // power on 18th
        // time_pwr_on_100us[303:288]     <= 16'd; // unit 100us  // power on 19th
        // time_pwr_on_100us[319:304]     <= 16'd; // unit 100us  // power on 20th
        // time_pwr_on_100us[335:320]     <= 16'd; // unit 100us  // power on 21th
        // time_pwr_on_100us[351:336]     <= 16'd; // unit 100us  // power on 22th
        // time_pwr_on_100us[367:352]     <= 16'd; // unit 100us  // power on 23th
        // time_pwr_on_100us[383:368]     <= 16'd; // unit 100us  // power on 24th
        // time_pwr_on_100us[399:384]     <= 16'd; // unit 100us  // power on 25th
        // time_pwr_on_100us[415:400]     <= 16'd; // unit 100us  // power on 26th
        // time_pwr_on_100us[431:416]     <= 16'd; // unit 100us  // power on 27th
        // time_pwr_on_100us[447:432]     <= 16'd; // unit 100us  // power on 28th
        // time_pwr_on_100us[463:448]     <= 16'd; // unit 100us  // power on 29th
        // time_pwr_on_100us[479:464]     <= 16'd; // unit 100us  // power on 30th
        // time_pwr_on_100us[495:480]     <= 16'd; // unit 100us  // power on 31th
        // time_pwr_on_100us[511:496]     <= 16'd; // unit 100us  // power on 32th 
        time_pwr_off_100us[15:0]       <= 16'd500; // unit 100us  // power off last             // CC2405 GSEN6504BSI EVK    EN_VCC_LT --> power off done
        time_pwr_off_100us[31:16]      <= 16'd150; // unit 100us  // power off (last - 1)       // CC2405 GSEN6504BSI EVK    EN_VDDIO  --> EN_VCC_LT
        time_pwr_off_100us[47:32]      <= 16'd420; // unit 100us  // power off (last - 2)       // CC2405 GSEN6504BSI EVK    EN_VDDD   --> EN_VDDIO
        time_pwr_off_100us[63:48]      <= 16'd500; // unit 100us  // power off (last - 3)       // CC2405 GSEN6504BSI EVK    EN_VDDA   --> EN_VDDD
        time_pwr_off_100us[79:64]      <= 16'd550; // unit 100us  // power off (last - 4)       // CC2405 GSEN6504BSI EVK    EN_VDDSF  --> EN_VDDA
        time_pwr_off_100us[95:80]      <= 16'd220; // unit 100us  // power off (last - 5)       // CC2405 GSEN6504BSI EVK    VDDBIAS   --> EN_VDDSF
        // time_pwr_off_100us[111:96]     <= 16'd500; // unit 100us  // power off (last - 6)
        // time_pwr_off_100us[127:112]    <= 16'd720;  // unit 100us  // power off (last - 7)
        // time_pwr_off_100us[143:128]    <= 16'd; // unit 100us  // power off (last - 8 )
        // time_pwr_off_100us[159:144]    <= 16'd; // unit 100us  // power off (last - 9 )
        // time_pwr_off_100us[175:160]    <= 16'd; // unit 100us  // power off (last - 10)
        // time_pwr_off_100us[191:176]    <= 16'd; // unit 100us  // power off (last - 11)
        // time_pwr_off_100us[207:192]    <= 16'd; // unit 100us  // power off (last - 12)
        // time_pwr_off_100us[223:208]    <= 16'd; // unit 100us  // power off (last - 13)
        // time_pwr_off_100us[239:224]    <= 16'd; // unit 100us  // power off (last - 14)
        // time_pwr_off_100us[255:240]    <= 16'd; // unit 100us  // power off (last - 15)
        // time_pwr_off_100us[271:256]    <= 16'd; // unit 100us  // power off (last - 16)
        // time_pwr_off_100us[287:272]    <= 16'd; // unit 100us  // power off (last - 17)
        // time_pwr_off_100us[303:288]    <= 16'd; // unit 100us  // power off (last - 18)
        // time_pwr_off_100us[319:304]    <= 16'd; // unit 100us  // power off (last - 19)
        // time_pwr_off_100us[335:320]    <= 16'd; // unit 100us  // power off (last - 20)
        // time_pwr_off_100us[351:336]    <= 16'd; // unit 100us  // power off (last - 21)
        // time_pwr_off_100us[367:352]    <= 16'd; // unit 100us  // power off (last - 22)
        // time_pwr_off_100us[383:368]    <= 16'd; // unit 100us  // power off (last - 23)
        // time_pwr_off_100us[399:384]    <= 16'd; // unit 100us  // power off (last - 24)
        // time_pwr_off_100us[415:400]    <= 16'd; // unit 100us  // power off (last - 25)
        // time_pwr_off_100us[431:416]    <= 16'd; // unit 100us  // power off (last - 26)
        // time_pwr_off_100us[447:432]    <= 16'd; // unit 100us  // power off (last - 27)
        // time_pwr_off_100us[463:448]    <= 16'd; // unit 100us  // power off (last - 28)
        // time_pwr_off_100us[479:464]    <= 16'd; // unit 100us  // power off (last - 29)
        // time_pwr_off_100us[495:480]    <= 16'd; // unit 100us  // power off (last - 30)
        // time_pwr_off_100us[511:496]    <= 16'd; // unit 100us  // power off (last - 31)

        o_flash_din_vld                <= 1'b0;
        o_flash_din                    <= 'd0;
        o_flash_function               <= 'd0;
        o_flash_len                    <= 'd0;
        o_lag_on_delay                 <= 'd0;
        o_lag_on_time                  <= 'd0;

        o_frame_linetime               <= 32'd90000;
        o_frame_clkpix                 <= 16'd0;
        o_inte_linetime                <= 32'd5000;
        o_inte_clkpix                  <= 16'd0;

        o_trigger_pulse_en             <= 1'b0;
        o_freq_sel                     <= 'd0;

        o_lp_mode_en                   <= 1'b1;
        o_lp_mode_offset0              <= 16'd100;
        o_lp_win_len_time              <= 16'd525;
        o_lp_mode_twait                <= 32'd76924;
        o_lp_mode_twarming             <= 32'd1539;
        o_lp_line_time                 <= 16'd325;
        o_lp_hsync_wait0               <= 16'd2;
        o_lp_hsync_wait1               <= 16'd1;

        o_pwr_seq_wait0                <= 32'd500_000;
        o_pwr_seq_wait1                <= 32'd20_000_000;
        o_pwr_seq_wait2                <= 32'd20_000_000;

        o_reg_test_mode_en             <= 1'b0;

        o_auto_step_en                 <= 'd12;
        o_image_type                   <= 'd1;
        o_image_gain                   <= 'd0;
        o_frame_start                  <= 'd0;
        o_frame_num_vld                <= 'd0;
        o_sub_eb_en                    <= 1'b0;
        o_fpga_project                 <= 'd0;
        o_mode_sel                     <= 'd0;
        o_white_balance_en             <= 1'b0;
        o_ccm_en                       <= 1'b0;
        o_color_composite_en           <= 1'b0;
        o_check_pll_times              <= 'd1000;
        o_opt_brt                      <= 'd0;
        o_opt_chan                     <= 'd0;
        o_os_group_en                  <= 'd1;
        o_os_idrive                    <= 'h00_00_00_00_00_67;
    end
    else begin
        o_flash_din_vld                <= 1'b0;
        if(i_prot_vld) begin 
            case(i_prot_addr[15:0])
                // 16'h0000:   o_cmd_bus                    <= i_prot_dat;
                // Used for universal [from 0x0001 to 0x00FF]                 // note:can only be added in sequence and cannot be modified
                16'h0001:   o_reg_addr                      <= i_prot_dat;
                16'h0002:   o_reg_len                       <= i_prot_dat;
                16'h0003:   o_frame_num                     <= i_prot_dat;    // Sensor output frame numbers ,use it if sensor has no this register
                16'h0004:   o_dp_control                    <= i_prot_dat;
                16'h0005:   o_dac_dat                       <= i_prot_dat;
                16'h0006:   o_otp_dat                       <= i_prot_dat[7:0];
                16'h0007:   o_testimage_fpga_typ            <= i_prot_dat[15:0];
                16'h0008:   o_i2c_en                        <= i_prot_dat[0];
                16'h0009:   o_slamode                       <= i_prot_dat[0];
                16'h000A:   o_image_lag_en                  <= i_prot_dat[0];
                16'h000B:   o_FAN_CTRL                      <= i_prot_dat[0];
                16'h000C:   o_TEXP_OTP_EN                   <= i_prot_dat[0];
                16'h000D:   o_image_ctr_en                  <= i_prot_dat[0];
                16'h000E:   o_switch_bit_num                <= i_prot_dat[3:0];
                16'h000F:   o_shift_en                      <= i_prot_dat[0];
                16'h0011:   o_time_out_frame_s              <= i_prot_dat[10:0];
                16'h0012:   o_lag_frame_start               <= i_prot_dat[15:0];
                16'h0013:   o_lag_frame_num                 <= i_prot_dat[15:0];
                16'h0014:   o_lag_vld_lane_sel              <= i_prot_dat[7:0];
                16'h0015:   time_sensor_rst_1us             <= i_prot_dat[15:0];
                16'h0016:   time_sensor_set_1us             <= i_prot_dat[15:0];
                16'h0017:   time_pwr_on_100us[15:0]         <= i_prot_dat[15:0]; // power on 1st
                16'h0018:   time_pwr_on_100us[31:16]        <= i_prot_dat[15:0]; // power on 2nd
                16'h0019:   time_pwr_on_100us[47:32]        <= i_prot_dat[15:0]; // power on 3rd
                16'h001A:   time_pwr_on_100us[63:48]        <= i_prot_dat[15:0]; // power on 4th
                16'h001B:   time_pwr_on_100us[79:64]        <= i_prot_dat[15:0]; // power on 5th
                16'h001C:   time_pwr_on_100us[95:80]        <= i_prot_dat[15:0]; // power on 6th
                // 16'h001D:   time_pwr_on_100us[111:96]       <= i_prot_dat[15:0]; // power on 7th
                // 16'h001E:   time_pwr_on_100us[127:112]      <= i_prot_dat[15:0]; // power on 8th
                // 16'h001F:   time_pwr_on_100us[143:128]      <= i_prot_dat[15:0]; // power on 9th
                // 16'h0020:   time_pwr_on_100us[159:144]      <= i_prot_dat[15:0]; // power on 10th
                // 16'h0021:   time_pwr_on_100us[175:160]      <= i_prot_dat[15:0]; // power on 11th
                // 16'h0022:   time_pwr_on_100us[191:176]      <= i_prot_dat[15:0]; // power on 12th
                // 16'h0023:   time_pwr_on_100us[207:192]      <= i_prot_dat[15:0]; // power on 13th
                // 16'h0024:   time_pwr_on_100us[223:208]      <= i_prot_dat[15:0]; // power on 14th
                // 16'h0025:   time_pwr_on_100us[239:224]      <= i_prot_dat[15:0]; // power on 15th
                // 16'h0026:   time_pwr_on_100us[255:240]      <= i_prot_dat[15:0]; // power on 16th
                // 16'h0027:   time_pwr_on_100us[271:256]      <= i_prot_dat[15:0]; // power on 17th
                // 16'h0028:   time_pwr_on_100us[287:272]      <= i_prot_dat[15:0]; // power on 18th
                // 16'h0029:   time_pwr_on_100us[303:288]      <= i_prot_dat[15:0]; // power on 19th
                // 16'h002A:   time_pwr_on_100us[319:304]      <= i_prot_dat[15:0]; // power on 20th
                // 16'h002B:   time_pwr_on_100us[335:320]      <= i_prot_dat[15:0]; // power on 21th
                // 16'h002C:   time_pwr_on_100us[351:336]      <= i_prot_dat[15:0]; // power on 22th
                // 16'h002D:   time_pwr_on_100us[367:352]      <= i_prot_dat[15:0]; // power on 23th
                // 16'h002E:   time_pwr_on_100us[383:368]      <= i_prot_dat[15:0]; // power on 24th
                // 16'h002F:   time_pwr_on_100us[399:384]      <= i_prot_dat[15:0]; // power on 25th
                // 16'h0030:   time_pwr_on_100us[415:400]      <= i_prot_dat[15:0]; // power on 26th
                // 16'h0031:   time_pwr_on_100us[431:416]      <= i_prot_dat[15:0]; // power on 27th
                // 16'h0032:   time_pwr_on_100us[447:432]      <= i_prot_dat[15:0]; // power on 28th
                // 16'h0033:   time_pwr_on_100us[463:448]      <= i_prot_dat[15:0]; // power on 29th
                // 16'h0034:   time_pwr_on_100us[479:464]      <= i_prot_dat[15:0]; // power on 30th
                // 16'h0035:   time_pwr_on_100us[495:480]      <= i_prot_dat[15:0]; // power on 31th
                // 16'h0036:   time_pwr_on_100us[511:496]      <= i_prot_dat[15:0]; // power on 32th 
                16'h0037:   time_pwr_off_100us[15:0]        <= i_prot_dat[15:0]; // power off 1st
                16'h0038:   time_pwr_off_100us[31:16]       <= i_prot_dat[15:0]; // power off 2nd
                16'h0039:   time_pwr_off_100us[47:32]       <= i_prot_dat[15:0]; // power off 3rd
                16'h003A:   time_pwr_off_100us[63:48]       <= i_prot_dat[15:0]; // power off 4th
                16'h003B:   time_pwr_off_100us[79:64]       <= i_prot_dat[15:0]; // power off 5th
                16'h003C:   time_pwr_off_100us[95:80]       <= i_prot_dat[15:0]; // power off 6th
                // 16'h003D:   time_pwr_off_100us[111:96]      <= i_prot_dat[15:0]; // power off 7th
                // 16'h003E:   time_pwr_off_100us[127:112]     <= i_prot_dat[15:0]; // power off 8th
                // 16'h003F:   time_pwr_off_100us[143:128]     <= i_prot_dat[15:0]; // power off 9th
                // 16'h0040:   time_pwr_off_100us[159:144]     <= i_prot_dat[15:0]; // power off 10th
                // 16'h0041:   time_pwr_off_100us[175:160]     <= i_prot_dat[15:0]; // power off 11th
                // 16'h0042:   time_pwr_off_100us[191:176]     <= i_prot_dat[15:0]; // power off 12th
                // 16'h0043:   time_pwr_off_100us[207:192]     <= i_prot_dat[15:0]; // power off 13th
                // 16'h0044:   time_pwr_off_100us[223:208]     <= i_prot_dat[15:0]; // power off 14th
                // 16'h0045:   time_pwr_off_100us[239:224]     <= i_prot_dat[15:0]; // power off 15th
                // 16'h0046:   time_pwr_off_100us[255:240]     <= i_prot_dat[15:0]; // power off 16th
                // 16'h0047:   time_pwr_off_100us[271:256]     <= i_prot_dat[15:0]; // power off 17th
                // 16'h0048:   time_pwr_off_100us[287:272]     <= i_prot_dat[15:0]; // power off 18th
                // 16'h0049:   time_pwr_off_100us[303:288]     <= i_prot_dat[15:0]; // power off 19th
                // 16'h004A:   time_pwr_off_100us[319:304]     <= i_prot_dat[15:0]; // power off 20th
                // 16'h004B:   time_pwr_off_100us[335:320]     <= i_prot_dat[15:0]; // power off 21th
                // 16'h004C:   time_pwr_off_100us[351:336]     <= i_prot_dat[15:0]; // power off 22th
                // 16'h004D:   time_pwr_off_100us[367:352]     <= i_prot_dat[15:0]; // power off 23th
                // 16'h004E:   time_pwr_off_100us[383:368]     <= i_prot_dat[15:0]; // power off 24th
                // 16'h004F:   time_pwr_off_100us[399:384]     <= i_prot_dat[15:0]; // power off 25th
                // 16'h0050:   time_pwr_off_100us[415:400]     <= i_prot_dat[15:0]; // power off 26th
                // 16'h0051:   time_pwr_off_100us[431:416]     <= i_prot_dat[15:0]; // power off 27th
                // 16'h0052:   time_pwr_off_100us[447:432]     <= i_prot_dat[15:0]; // power off 28th
                // 16'h0053:   time_pwr_off_100us[463:448]     <= i_prot_dat[15:0]; // power off 29th
                // 16'h0054:   time_pwr_off_100us[479:464]     <= i_prot_dat[15:0]; // power off 30th
                // 16'h0055:   time_pwr_off_100us[495:480]     <= i_prot_dat[15:0]; // power off 31th
                // 16'h0056:   time_pwr_off_100us[511:496]     <= i_prot_dat[15:0]; // power off 32th 

                16'h005B:   begin
                                o_flash_din_vld             <= 1'b1;
                                o_flash_din                 <= i_prot_dat[7:0];
                            end
                16'h005C:   o_flash_function                <= i_prot_dat[3:0];
                16'h005D:   o_flash_len                     <= i_prot_dat;

                16'h0060:   o_lag_on_delay[15:0]            <= i_prot_dat;
                16'h0061:   o_lag_on_delay[31:16]           <= i_prot_dat;
                16'h0062:   o_lag_on_delay[47:32]           <= i_prot_dat;
                16'h0063:   o_lag_on_time[15:0]             <= i_prot_dat;
                16'h0064:   o_lag_on_time[31:16]            <= i_prot_dat;
                16'h0065:   o_lag_on_time[47:32]            <= i_prot_dat;

                //Used for individual project  [from 0x0100 to 0x0FFF]                     // note:Address is used at will
                16'h0100:   o_frame_linetime[15:0]          <= i_prot_dat;
                16'h0101:   o_frame_linetime[31:16]         <= i_prot_dat;
                16'h0102:   o_frame_clkpix[15:0]            <= i_prot_dat;
                16'h0103:   o_inte_linetime[15:0]           <= i_prot_dat;
                16'h0104:   o_inte_linetime[31:16]          <= i_prot_dat;
                16'h0105:   o_inte_clkpix[15:0]             <= i_prot_dat;
                16'h0106:   o_trigger_pulse_en              <= i_prot_dat[0];
                16'h0107:   o_freq_sel                      <= i_prot_dat[1:0];            // 0 : o_clk_texp = 120M  ; 1 : o_clk_texp = 150M ; clk_ref = 40M 

                16'h0200:   o_lp_mode_en                    <= i_prot_dat[0];
                16'h021D:   o_lp_win_len_time               <= i_prot_dat;
                16'h021E:   o_lp_mode_offset0               <= i_prot_dat;
                16'h021F:   o_lp_mode_twait[15:0]           <= i_prot_dat;
                16'h0220:   o_lp_mode_twait[31:16]          <= i_prot_dat;
                16'h0221:   o_lp_mode_twarming[15:0]        <= i_prot_dat;
                16'h0222:   o_lp_mode_twarming[31:16]       <= i_prot_dat;
                16'h0223:   o_lp_line_time                  <= i_prot_dat;
                16'h0224:   o_lp_hsync_wait0                <= i_prot_dat;
                16'h0225:   o_lp_hsync_wait1                <= i_prot_dat;

                16'h0300:   o_pwr_seq_wait0[15:0]           <= i_prot_dat;
                16'h0301:   o_pwr_seq_wait0[31:16]          <= i_prot_dat[7:0];
                16'h0302:   o_pwr_seq_wait1[15:0]           <= i_prot_dat;
                // Keep the complete high word.  A 200 ms wait at 100 MHz is
                // 20,000,000 = 0x0131_2D00 and therefore requires bit 24.
                16'h0303:   o_pwr_seq_wait1[31:16]          <= i_prot_dat;
                16'h0304:   o_pwr_seq_wait2[15:0]           <= i_prot_dat;
                16'h0305:   o_pwr_seq_wait2[31:16]          <= i_prot_dat[7:0];

                16'h0400:   o_reg_test_mode_en              <= i_prot_dat[0];

                //Used for auto operation [from 0x1000 to 0x1FFF]                          // note:can only be added in sequence and cannot be modified
                16'h1000:   o_auto_step_en                  <= i_prot_dat;                 // bit2:rd default enable,  bit3:power on enbanle
                16'h1001:   o_image_type                    <= i_prot_dat;                 // 0x0001: usb 16 bit size original image output
                                                                                           // 0x0002: usb 16 bit size average image output,
                                                                                           // 0x0004: usb 16 bit size noise image output (x 128) 
                                                                                           // 0x000B: usb 16 bit size noise image output , keep integers delate decimals
                16'h1002:   o_image_gain                    <= i_prot_dat;                 // 1:LG,  2:HG,  3:LG&HG,  4:STD
                16'h1003:   o_frame_start                   <= i_prot_dat;
                16'h1004:   o_frame_num_vld                 <= i_prot_dat;                 // It is better to set it to the actual width, and the high position is supplemented by 0
                16'h1005:   o_sub_eb_en                     <= i_prot_dat[0];
                16'h1006:   o_fpga_project                  <= i_prot_dat;                 // when a FPGA project is compatible with different sensor, use it to distinguish 
                16'h1007:   o_mode_sel                      <= i_prot_dat;                 // Auto configuration operating  config .ini selection
                16'h1008:   o_white_balance_en              <= i_prot_dat[0];
                16'h1009:   o_ccm_en                        <= i_prot_dat[0];
                16'h100A:   o_color_composite_en            <= i_prot_dat[0];
                16'h100B:   o_check_pll_times               <= i_prot_dat;
                16'h100C:   o_opt_brt                       <= i_prot_dat;
                16'h100D:   o_opt_chan                      <= i_prot_dat[2:0];
                16'h100E:   o_os_group_en[5:0]              <= i_prot_dat[5:0];
                16'h100F:   o_os_idrive[7:0]                <= i_prot_dat[7:0];
                16'h1010:   o_os_idrive[15:8]               <= i_prot_dat[7:0];
                16'h1011:   o_os_idrive[23:16]              <= i_prot_dat[7:0];
                16'h1012:   o_os_idrive[31:24]              <= i_prot_dat[7:0];
                16'h1013:   o_os_idrive[39:32]              <= i_prot_dat[7:0];
                16'h1014:   o_os_idrive[47:40]              <= i_prot_dat[7:0];
                default: ;
            endcase
        end

        if(cmd_bus_vld_pre_dly == 2'b01) begin
            if(cmd_bus_pre == 16'h1001) begin
                if(o_mode_sel < 10) begin    // 0~9: 100MHz 12bit
                    o_freq_sel    <= 'd0;    // 100 MHz
                end
                else begin
                    o_freq_sel    <= 'd1;
                end
            end
        end
        
    end
end

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        cmd_bus_vld_pre_dly    <= 'd0;
    end
    else begin
        cmd_bus_vld_pre_dly    <= {cmd_bus_vld_pre_dly[0],cmd_bus_vld_pre};
    end
end

endmodule
