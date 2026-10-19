`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//                  --  Copyright (c) 2012 Gpixel, Technology Co. Ltd.      --  
//                  --          All Rights Reserved.                        --
//------------------------------------------------------------------------------------------------------------------
//  -- Module       : iser2des
//  -- File         : iser2des_7serial.v
//  -- Author       : Tianjiao Hou
//------------------------------------------------------------------------------------------------------------------
//  -- Function     : serial to parallel .Compatible with 8bit,10bit,12bit
//------------------------------------------------------------------------------------------------------------------
//  -- Hisoty       :
//  -- Ver          Date        By              Description 
//  -- 1.0          2022.05.30  Tianjiao Hou    creat 
//  -- 1.1          2022.06.16  Danyang Li      modify style
//  -- 1.2          2022.06.28  Danyang Li      modify style,i_msb_first_en added
//  -- 1.4          2022.12.22  Tianjiao Hou    combine zynq and k7,define zynq or not(K7) in define  module or top module
//  -- 1.5          2023.07.01  Tianjiao Hou    only 7serial, combine 8/10/12/14/16
//  -- 1.6          2023.08.11  Tianjiao Hou    1.change the meaning represented by the i_adc_depth value
//                                              2.combine 8/9/10/11/12/14/16,add 9bit and 11bit adc depth,
//------------------------------------------------------------------------------------------------------------------

module iser2des#(
    parameter    DAT_8BIT_EN            = 1'b0        , // 8bit serial to parral enable 
    parameter    DAT_9BIT_EN            = 1'b0        , // 9bit serial to parral enable
    parameter    DAT_10BIT_EN           = 1'b0        , // 10bit serial to parral enable
    parameter    DAT_11BIT_EN           = 1'b0        , // 11bit serial to parral enable
    parameter    DAT_12BIT_EN           = 1'b1        , // 12bit serial to parral enable
    parameter    DAT_14BIT_EN           = 1'b0        , // 14bit serial to parral enable
    parameter    DAT_16BIT_EN           = 1'b0        , // 16bit serial to parral enable
    parameter    GROUP                  = "GROUP1ST"  , // IODELAY_GROUP of IDELAYE2
    parameter    REFCLK_FREQUENCY       = 300           // REFCLK_FREQUENCY of IDELAYE2, 200M:78ps/tap,300M:52ps/tap,400M:39ps/tap
)
(
    input               clk_rx           , // frequency is 1/4 clk_ddr                      // 'CLK' of IDELAYE, 'CLKDIV' of ISERDESE,
    input               clk_ddr          , // frequency is sensor output lvds clk: DDR CLK  // 'CLK' of ISERDESE,
    input               rst_rx           , // clk_rx domain
    input               i_SENSOR_DATA_P  , // sensor output P of lvds data
    input               i_SENSOR_DATA_N  , // sensor output N of lvds data
    input     [7:0]     i_adc_depth      , // 8:8bit,9:9bit,10:10bit,11:11bit,12:12bit, 14:14bit,16:16bit 
    input               i_rst_idelay     , // for 'RST' of IDELAYE
    input               i_idelay_ld      , // for 'LOAD' of IDELAYE
    input     [7:0]     i_idelay_valuein , // for 'CNTVALUEIN' of IDELAYE3
    input               i_bitslip        , // bit moved flag for word align
    input               i_bitslip_change , // ISERDESE is 1 to 8,so i_bitslip can only move out of 8 possibilities, i_bitslip_change equal to 0 and 1 make 16 possibilities
    input               i_msb_first_en   , // serial data(i_SENSOR_DATA) output sequence, 1:msb , 0:lsb
    output              o_dat_vld        , // o_dat valid flag, clk_rx domain
    output    [15:0]    o_dat              // output data, clk_rx domain
);
 


//fpga_zynq
   
//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

wire             data_in       ;
wire             data_serial   ;
wire             icascade1     ;
wire             icascade2     ;
wire    [7:0]    dat_parallel  ;

//--------------------------------------------------------------------------------------------------------------------
//Instance
//--------------------------------------------------------------------------------------------------------------------

 IBUFDS #(
    .DIFF_TERM                ("TRUE"           ),    // Differential Termination
    .IBUF_LOW_PWR             ("FALSE"          ),    // Low power="TRUE", Highest performance="FALSE"
    .IOSTANDARD               ("LVDS_25"        )     // Specify the input I/O standard
) IBUFDS_IBUF_DATA (
    .O                        (data_in          ),    // Buffer output
    .I                        (i_SENSOR_DATA_P  ),    // Diff_p buffer input (connect directly to top-level port)
    .IB                       (i_SENSOR_DATA_N  )     // Diff_n buffer input (connect directly to top-level port)
);


//(* IODELAY_GROUP = "GROUP" *) IDELAYE2 #(
(* IODELAY_GROUP = GROUP *) IDELAYE2 #(
    .CINVCTRL_SEL             ("FALSE"          ),
    .DELAY_SRC                ("IDATAIN"        ),
    .HIGH_PERFORMANCE_MODE    ("FALSE"          ),
    .IDELAY_TYPE              ("VAR_LOAD"       ),
    .IDELAY_VALUE             (0                ),
    .PIPE_SEL                 ("FALSE"          ),
    .REFCLK_FREQUENCY         (REFCLK_FREQUENCY ),//Modified according to idelayctrl
    .SIGNAL_PATTERN           ("DATA"           )
)
u_IDELAYE2 (
    .CNTVALUEOUT              (                      ),
    .DATAOUT                  (data_serial           ),
    .C                        (clk_rx                ),
    .CE                       (1'b0                  ),
    .CINVCTRL                 (1'b0                  ),
    .CNTVALUEIN               (i_idelay_valuein[4:0] ),
    .DATAIN                   (1'b0                  ),
    .IDATAIN                  (data_in               ),
    .INC                      (1'b0                  ),
    .LD                       (i_idelay_ld           ),
    .LDPIPEEN                 (1'b0                  ),
    .REGRST                   (i_rst_idelay          )
 );

ISERDESE2 # (
    .DATA_RATE                ("DDR"            ),
    .DATA_WIDTH               (8                ),
    .INTERFACE_TYPE           ("NETWORKING"     ), 
    .DYN_CLKDIV_INV_EN        ("FALSE"          ),
    .DYN_CLK_INV_EN           ("FALSE"          ),
    .NUM_CE                   (2                ),
    .OFB_USED                 ("FALSE"          ),
    .IOBDELAY                 ("IFD"            ),                                // Use input at DDLY to output the data on Q
    .SERDES_MODE              ("MASTER"         ))
u_iserdese2 (
    .Q1                       (dat_parallel[7] ),
    .Q2                       (dat_parallel[6] ),
    .Q3                       (dat_parallel[5] ),
    .Q4                       (dat_parallel[4] ),
    .Q5                       (dat_parallel[3] ),
    .Q6                       (dat_parallel[2] ),
    .Q7                       (dat_parallel[1] ),
    .Q8                       (dat_parallel[0] ),
    .SHIFTOUT1                (icascade1        ),
    .SHIFTOUT2                (icascade2        ),
    .BITSLIP                  (i_bitslip        ),
    .CE1                      (1'b1             ),
    .CE2                      (1'b1             ),
    .CLK                      (clk_ddr          ),
    .CLKB                     (~clk_ddr         ),
    .CLKDIV                   (clk_rx           ),
    .CLKDIVP                  (1'b0             ),
    .D                        (1'b0             ),
    .DDLY                     (data_serial      ),
    .RST                      (i_rst_idelay     ),
    .SHIFTIN1                 (1'b0             ),
    .SHIFTIN2                 (1'b0             ),
    .DYNCLKDIVSEL             (1'b0             ),
    .DYNCLKSEL                (1'b0             ),
    .OFB                      (1'b0             ),
    .OCLK                     (1'b0             ),
    .OCLKB                    (1'b0             ),
    .O                        (                 )
); 

des_8bit_to_adcdepth#(
    .DAT_8BIT_EN              (DAT_8BIT_EN     ), 
    .DAT_9BIT_EN              (DAT_9BIT_EN     ),
    .DAT_10BIT_EN             (DAT_10BIT_EN    ),
    .DAT_11BIT_EN             (DAT_11BIT_EN    ),
    .DAT_12BIT_EN             (DAT_12BIT_EN    ), 
    .DAT_14BIT_EN             (DAT_14BIT_EN    ),
    .DAT_16BIT_EN             (DAT_16BIT_EN    )
) u_des_8bit_to_adcdepth(
    .clk                      (clk_rx           ),
    .rst                      (rst_rx           ),
    .rst_idelay               (i_rst_idelay     ),
    .i_bitslip_change         (i_bitslip_change ),
    .i_adc_depth              (i_adc_depth      ),
    .i_dat                    (dat_parallel     ),
    .i_msb_first_en           (i_msb_first_en   ),
    .o_dat_vld                (o_dat_vld        ),
    .o_dat                    (o_dat            )
);  

endmodule
