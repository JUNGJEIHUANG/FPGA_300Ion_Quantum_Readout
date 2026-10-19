`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//                  --  Copyright (c) 2012 Gpixel, Technology Co. Ltd.      --
//                  --          All Rights Reserved.                        --
//------------------------------------------------------------------------------------------------------------------
//  -- Module       : train_word_align
//  -- File         : train_word_align_7serial.v
//  -- Author       : tianjiao.hou@gpixel.com 
//------------------------------------------------------------------------------------------------------------------
//  -- Function     : Bit and Word alignment control
//------------------------------------------------------------------------------------------------------------------
//  -- Hisoty       :
//  -- Ver          Date        By              Description 
//  -- 1.0          2022.05.30  Tianjiao Hou    creat 
//  -- 1.1          2022.06.15  Danyang Li      modify style 
//  -- 1.2          2022.06.29  Danyang Li      Three FSM create 
//  -- 1.4          2022.12.22  Tianjiao Hou    combine zynq and k7,define zynq or not(K7) in define  module or top module
//  -- 1.5          2023.05.24  Tianjiao Hou    as train bit&word align top, Original function in align_ctrl.v
//  -- 1.6          2023.08.23  Tianjiao Hou    1.change file name from train_word_align.v to train_word_align_7serial.v,only for 7serial
//                                              2.change the meaning represented by the i_adc_depth value
//                                              3.add 9bit and 11bit adc depth
//------------------------------------------------------------------------------------------------------------------

module train_word_align_debug#(
    parameter    DAT_8BIT_EN            = 1'b0       , // 8bit serial to parral enable 
    parameter    DAT_9BIT_EN            = 1'b0       , // 9bit serial to parral enable
    parameter    DAT_10BIT_EN           = 1'b0       , // 10bit serial to parral enable
    parameter    DAT_11BIT_EN           = 1'b0       , // 11bit serial to parral enable
    parameter    DAT_12BIT_EN           = 1'b1       , // 12bit serial to parral enable
    parameter    DAT_14BIT_EN           = 1'b0       , // 14bit serial to parral enable
    parameter    DAT_16BIT_EN           = 1'b0       , // 16bit serial to parral enable
    parameter    DAT_WIDTH              = 12         , // max data width  // 8/10/12/14/16 supported 
    parameter    TAP_MAX                = 31         , // zynq:255. K7:31, hided when generating IP core
                                                       // 1/2 real zynq taps //zynq taps from 0 to 511  //iodelay3 512tap=2ns(0.0004*512tap)
                                                       // K7 tap from 0 to 31 //idelayctrl 200M : 78ps/1tap, 300M : 52ps/1tap, 400M : 39ps/1tap
    parameter    GROUP                  = "GROUP1ST" , // IODELAY_GROUP of IDELAYE2
    parameter    REFCLK_FREQUENCY       = 300          // REFCLK_FREQUENCY of IDELAYE2, 200M:78ps/tap,300M:52ps/tap,400M:39ps/tap
)
(
    input                               clk_rx               , // frequency is 1/4 clk_ddr
    input                               clk_ddr              , // frequency is sensor output lvds clk: DDR CLK
    input                               rst_rx               , // clk_rx domain
    input                               clk_glb              , // sampling clock for output data: o_dat_vld and o_dat
    input                               rst_glb              , // clk_glb domain
    input                               i_SENSOR_DATA_P      , // sensor output P of lvds data
    input                               i_SENSOR_DATA_N      , // sensor output N of lvds data
    input            [7:0]              i_adc_depth          , // 8:8bit,9:9bit,10:10bit,11:11bit,12:12bit, 14:14bit,16:16bit 
    input                               i_cmd_start_train    , // command of start training 
    input            [DAT_WIDTH-1:0]    i_training_word      , // trainging word
    input            [6:0]              i_stable_len_half    , // half of one bit's stable length, //It is best to test according to different projects
    input                               i_msb_first_en       , // serial data(i_SENSOR_DATA) output sequence, 1:msb , 0:lsb
    output    reg                       o_dat_vld            , // o_dat valid flag, clk_glb domain
    output    reg    [DAT_WIDTH-1:0]    o_dat                , // output data, clk_glb domain
    output                              o_train_done         , // training done flag, maintain high for 16 clk_rx
    output           [7:0]              o_loc_eye_start      , // clk_rx domain. start location of i_SENSOR_DATA's stable eye
    output           [7:0]              o_loc_eye_mid        , // clk_rx domain. sample location of i_SENSOR_DATA
    output           [7:0]              o_loc_eye_end        , // clk_rx domain. end location of i_SENSOR_DATA's stable eye
    output           [7:0]              o_cnt_timeout        , // clk_rx domain. times of error align, word align error times:o_cnt_timeout[3:0], bit align error times:o_cnt_timeout[7:4]
    output           [4:0]              o_loc_word           , // clk_rx domain. times of bit moving in word align
    output                              o_loc_ok               , // clk_rx domain. align successful flag,  1:correct, 0:error
    output wire [63:0]                  o_debug               // internal deserializer/FIFO debug
    );
    
    
//------------------------------------------------------------------------------------------------------------------
//parameter
//------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

wire             rst_idelay       ;
wire             idelay_ld        ;
wire   [7:0]     idelay_valuein   ;
wire             bitslip          ;
wire             bitslip_change   ;
wire   [4:0]     rd_data_count    ;
reg              rd_en            ;
reg              rd_en_dly        ;
wire             dat_des_vld      ;
reg             dat_des_vld_dly  ;  // 鏂板锛氬欢杩焩ld淇″彿浼犵粰align_ctrl
wire   [15:0]    dat_des          ;
wire   [15:0]    fifo_dout        ;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
// Internal debug is intentionally exposed for one selected lane (lane 8 at the
// training_top wrapper). The low fields are the raw ISERDES word and its valid
// flag; the remaining fields show the CDC FIFO read timing and training controls.
assign o_debug = {
    7'd0, rst_glb, rst_rx, i_cmd_start_train, o_loc_ok, rst_idelay, idelay_ld,
    idelay_valuein, bitslip_change, bitslip, rd_data_count, rd_en_dly, rd_en,
    dat_des_vld_dly, dat_des_vld, fifo_dout, dat_des
};

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------
always @(posedge clk_glb or posedge rst_glb) begin
    if(rst_glb == 1'b1) begin   
        rd_en   <= 1'b0;
    end
    else begin
        if(rd_data_count > 'd4) begin   
            rd_en   <= 1'b1;
        end
        else begin
            rd_en   <= 1'b0;
        end
    end
end

always @(posedge clk_glb or posedge rst_glb) begin
    if(rst_glb == 1) begin
        o_dat        <= 'd0;
        rd_en_dly    <= 1'b0;
        o_dat_vld    <= 1'b0;
    end
    else begin
        o_dat        <= fifo_dout[DAT_WIDTH-1:0];
        rd_en_dly    <= rd_en;
        o_dat_vld    <= rd_en_dly;
    end
end

always @(posedge clk_rx) begin
    dat_des_vld_dly <= dat_des_vld;  // 鏂板锛氬悓姝ュ埌clk_rx鏃堕挓鍩燂紝鍖归厤align_ctrl閲囨牱寤惰繜
end

//--------------------------------------------------------------------------------------------------------------------
//Instance
//--------------------------------------------------------------------------------------------------------------------

//K7 and ZYNQ can share this module
align_ctrl_debug#(
    .DAT_WIDTH               (DAT_WIDTH            ),
    .TAP_MAX                 (TAP_MAX              )
)    u_align_ctrl(
    .clk                     (clk_rx               ),
    .rst                     (rst_rx               ),
    .i_adc_depth             (i_adc_depth          ),
    .i_cmd_start_train       (i_cmd_start_train    ),
    .i_training_word         (i_training_word      ),
    .i_stable_len_half       (i_stable_len_half    ),
    .i_dat                   (dat_des              ),
    .i_dat_vld               (dat_des_vld_dly      ),
    .o_rst_idelay            (rst_idelay           ),
    .o_idelay_ld             (idelay_ld            ),
    .o_idelay_valuein        (idelay_valuein       ),
    .o_bitslip               (bitslip              ),
    .o_bitslip_change        (bitslip_change       ),
    .o_train_done            (o_train_done         ),
    .o_loc_eye_start         (o_loc_eye_start      ),
    .o_loc_eye_mid           (o_loc_eye_mid        ),
    .o_loc_eye_end           (o_loc_eye_end        ),
    .o_cnt_timeout           (o_cnt_timeout        ),
    .o_loc_word              (o_loc_word           ),
    .o_loc_ok                (o_loc_ok             )
);


//K7 and ZYNQ can share this module io;
//des_8bit_to_adcdepth under iser2des is same, but others in iser2des need to be modified(K7: IDELAYE2 and ISERDESE2, ZYNQ : IDELAYE3 and ISERDESE3)
iser2des_debug#(
    .GROUP                    (GROUP            ), 
    .REFCLK_FREQUENCY         (REFCLK_FREQUENCY ), 
    .DAT_8BIT_EN              (DAT_8BIT_EN      ), 
    .DAT_9BIT_EN              (DAT_9BIT_EN     ),  
    .DAT_10BIT_EN             (DAT_10BIT_EN    ),  
    .DAT_11BIT_EN             (DAT_11BIT_EN    ),  
    .DAT_12BIT_EN             (DAT_12BIT_EN     ), 
    .DAT_14BIT_EN             (DAT_14BIT_EN     ),
    .DAT_16BIT_EN             (DAT_16BIT_EN     )
)    u_iser2des(                               
    .clk_rx                   (clk_rx           ),
    .clk_ddr                  (clk_ddr          ),
    .rst_rx                   (rst_rx           ),
    .i_SENSOR_DATA_P          (i_SENSOR_DATA_P  ),
    .i_SENSOR_DATA_N          (i_SENSOR_DATA_N  ),
    .i_adc_depth              (i_adc_depth      ),
    .i_rst_idelay             (rst_idelay       ),
    .i_idelay_ld              (idelay_ld        ),
    .i_idelay_valuein         (idelay_valuein   ),
    .i_bitslip                (bitslip          ),
    .i_bitslip_change         (bitslip_change   ),
    .i_msb_first_en           (i_msb_first_en   ),
    .o_dat_vld                (dat_des_vld      ),
    .o_dat                    (dat_des          )
);

// Independent Clocks Distributed RAM   
// Standard FIFO
// Write/Read width: 16
// Write/Read depth: 16
// ResetPin, Enable Reset Synchronization : select
// Valid Flag : select
// Read Data Count : select 
fifo_generator_0    u_fifo_w16d32_icdr_st(
    .rst              (rst_rx          ),
    .wr_clk           (clk_rx          ),
    .rd_clk           (clk_glb         ),
    .din              (dat_des         ),
    .wr_en            (dat_des_vld     ),
    .rd_en            (rd_en           ),
    .dout             (fifo_dout       ),
    .full             (                ),
    .empty            (                ),
    .rd_data_count    (rd_data_count   ) 
); 

endmodule

