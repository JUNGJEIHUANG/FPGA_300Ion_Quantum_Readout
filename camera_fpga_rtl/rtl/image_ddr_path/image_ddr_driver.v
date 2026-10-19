`timescale 1ns / 1ps
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : image_ddr_driver
// --File      : image_ddr_driver.v
// --Author    : guangcai.ding@gpixel.com
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : image_ddr_driver module.
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2023.11.03   Tianjiao Hou    creat
// --1.0       2024.04.25   Guangcai Ding   modify code style
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module image_ddr_driver #(
    parameter    IMAGE_DAT_WIDTH    = 448
)(
    input                                 clk_rx                ,    // Input data clock
    input                                 rst_rx                ,    // Input reset
    input                                 clk_200m              ,    // MIG SYS CLK
    input                                 rst_200m              ,    // MIG SYS RST
    input                                 i_cmd_frame_req       ,    // Frame request
    input     [2:0]                       i_image_type          ,    // Output image types, bit 0: ORIG image, bit 1: DIV image, bit 2: STDEV image, bit 3: STDEV output cut enable
    input     [1:0]                       i_hdr_mode_en         ,
    input     [15:0]                      i_lval_len            ,    // lval_length
    input     [15:0]                      i_win_row_len         ,    // line number
    input     [15:0]                      i_frame_num           ,    // Frame number
    input                                 i_dat_vld             ,    // Input data line valid
    input     [IMAGE_DAT_WIDTH - 1:0]     i_dat                 ,    // Input data
    output                                o_dbg_rd_req_ui,
    input                                 i_ddr_rd_req          ,
    output                                o_dat_vld             ,
    output    [IMAGE_DAT_WIDTH - 1:0]     o_dat                 ,
    output                                o_ddr_wr_done         ,    // DDR write done , same to frame_rx_done
    output                                clk_ui                ,
    output                                rst_ui                ,
    output                                o_init_calib_complete,
    output    [15:0]                      o_DDR3_ADDR           ,    // DDR3 ports
    output    [2:0]                       o_DDR3_BA             ,
    output                                o_DDR3_CAS_N          ,
    output                                o_DDR3_CK_N           ,
    output                                o_DDR3_CK_P           ,
    output                                o_DDR3_CKE            ,
    output                                o_DDR3_RAS_N          ,
    output                                o_DDR3_RESET_N        ,
    output                                o_DDR3_WE_N           ,
    inout     [63:0]                      io_DDR3_DQ            ,
    inout     [7:0]                       io_DDR3_DQS_N         ,
    inout     [7:0]                       io_DDR3_DQS_P         ,
    output                                o_DDR3_CS_N           ,
    output    [7:0]                       o_DDR3_DM             ,
    output                                o_DDR3_ODT
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

wire                               init_calib_complete  ;
wire                               app_rdy              ;
wire                               app_wdf_rdy          ;
wire    [28:0]                     app_addr             ;
wire    [2:0]                      app_cmd              ;
wire                               app_en               ;
reg     [3:0]                      cmd_frame_req_rx_dly ;
reg     [4:0]                      cnt_frame_rst_rx     ;
reg                                frame_rst_rx         ;
wire                               fifo2ddr_ren         ;
wire    [IMAGE_DAT_WIDTH - 1:0]    fifo2ddr_dout        ;
wire                               fifo2ddr_empty       ;
wire    [8:0]                      fifo2ddr_rd_cnt      ;
wire    [511-IMAGE_DAT_WIDTH:0]    dat_unused           ;    // IMAGE_DAT_WIDTH < 512

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

assign o_init_calib_complete = init_calib_complete;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk_rx  or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        cnt_frame_rst_rx    <= 'd0;
        frame_rst_rx        <= 1'b1;
    end
    else begin
        if(cmd_frame_req_rx_dly[3:2] == 2'b01) begin
            cnt_frame_rst_rx    <= 'd0;
            frame_rst_rx        <= 1'b1;
        end
        else begin
            if(cnt_frame_rst_rx >= 'd15) begin
                frame_rst_rx    <= 1'b0;
            end
            else begin
                cnt_frame_rst_rx    <= cnt_frame_rst_rx + 1'b1;
            end
        end
    end
end

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Indepentent Clocks Block RAM
// First Word Fall Through
// Write/Read width: 448
// Write/Read depth: 512
// ResetPin: select
// Enable Reset Synchronization: select
// Enable Safety Circuit: not select
// ResetType: Asynchronous Reset
// Read Data Count : select
fifo_w384d512_icbr_fwft    fifo2ddr_dat_buf
(
    .rst                   (frame_rst_rx   ),
    .wr_clk                (clk_rx         ),
    .rd_clk                (clk_ui         ),
    .din                   (i_dat          ),
    .wr_en                 (i_dat_vld      ),
    .rd_en                 (fifo2ddr_ren   ),
    .dout                  (fifo2ddr_dout  ),
    .full                  (               ),
    .empty                 (fifo2ddr_empty ),
    .rd_data_count         (fifo2ddr_rd_cnt)
);

mig_driver    inst_mig_driver
(
    .clk                   (clk_ui             ),    // Output MIG generated clock
    .rst                   (rst_ui             ),    // Output MIG generated clock reset signal
    .i_cmd_frame_req       (i_cmd_frame_req    ),
    .i_image_type          (i_image_type       ),    // Image type
    .i_hdr_mode_en         (i_hdr_mode_en      ),
    .i_lval_len            (i_lval_len         ),
    .i_win_row_len         (i_win_row_len      ),
    .i_frame_num           (i_frame_num        ),
    .i_fifo2ddr_empty      (fifo2ddr_empty     ),    // FIFO empty signal
    .i_fifo2ddr_rd_cnt     (fifo2ddr_rd_cnt    ),    // FIFO read count
    .o_fifo2ddr_ren        (fifo2ddr_ren       ),    // FIFO read enable
    .o_ddr_wr_done         (o_ddr_wr_done      ),
    .i_ddr_rd_req          (i_ddr_rd_req       ),
    .o_dbg_rd_req_ui       (o_dbg_rd_req_ui    ),    // Transmitted data finshed signal
    .i_init_calib_complete (init_calib_complete),    //MIG ports
    .i_app_rdy             (app_rdy            ),
    .i_app_wdf_rdy         (app_wdf_rdy        ),
    .o_app_addr            (app_addr           ),
    .o_app_cmd             (app_cmd            ),
    .o_app_en              (app_en             )
);

mig_k7_mem_bar_512b    mig_7series_512b
(
    // Memory interface portsendmodule
    .ddr3_addr             (o_DDR3_ADDR                                  ),    // output [15:0]
    .ddr3_ba               (o_DDR3_BA                                    ),    // output [2:0]
    .ddr3_cas_n            (o_DDR3_CAS_N                                 ),    // output
    .ddr3_ck_n             (o_DDR3_CK_N                                  ),    // output [0:0]
    .ddr3_ck_p             (o_DDR3_CK_P                                  ),    // output [0:0]
    .ddr3_cke              (o_DDR3_CKE                                   ),    // output [0:0]
    .ddr3_ras_n            (o_DDR3_RAS_N                                 ),    // output
    .ddr3_reset_n          (o_DDR3_RESET_N                               ),    // output
    .ddr3_we_n             (o_DDR3_WE_N                                  ),    // output
    .ddr3_dq               (io_DDR3_DQ                                   ),    // inout [63:0]
    .ddr3_dqs_n            (io_DDR3_DQS_N                                ),    // inout [7:0]
    .ddr3_dqs_p            (io_DDR3_DQS_P                                ),    // inout [7:0]
    .ddr3_cs_n             (o_DDR3_CS_N                                  ),    // output [0:0]
    .ddr3_dm               (o_DDR3_DM                                    ),    // output [7:0]
    .ddr3_odt              (o_DDR3_ODT                                   ),    // output [0:0]
    .init_calib_complete   (init_calib_complete                          ),    // output
    // Application interface ports
    .app_addr              ({1'b0,app_addr}                              ),    // input [29:0]
    .app_cmd               (app_cmd                                      ),    // input [2:0]
    .app_en                (app_en                                       ),    // input
    .app_wdf_data          ({{(512-IMAGE_DAT_WIDTH){1'b0}},fifo2ddr_dout}),    // input [511:0]
    .app_wdf_end           (fifo2ddr_ren                                 ),    // input
    .app_wdf_wren          (fifo2ddr_ren                                 ),    // input
    .app_rd_data           ({dat_unused,o_dat}                           ),    // output [511:0]
    .app_rd_data_end       (                                             ),    // output
    .app_rd_data_valid     (o_dat_vld                                    ),    // output
    .app_rdy               (app_rdy                                      ),    // output
    .app_wdf_rdy           (app_wdf_rdy                                  ),    // output
    .app_sr_req            (1'b0                                         ),    // input
    .app_ref_req           (1'b0                                         ),    // input
    .app_zq_req            (1'b0                                         ),    // input
    .app_sr_active         (                                             ),    // output
    .app_ref_ack           (                                             ),    // output
    .app_zq_ack            (                                             ),    // output
    .ui_clk                (clk_ui                                       ),    // output
    .ui_clk_sync_rst       (rst_ui                                       ),    // output
    .app_wdf_mask          (64'd0                                        ),    // input [63:0]
    .device_temp           (                                             ),    // output [11:0]
    // System Clock Ports
    .sys_clk_i             (clk_200m                                     ),    // input
    // Reference Clock Ports                                             
    .clk_ref_i             (clk_200m                                     ),    // input
    .sys_rst               (rst_200m                                     )     // active high
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        cmd_frame_req_rx_dly    <= 'd0;
    end
    else begin
        cmd_frame_req_rx_dly    <= {cmd_frame_req_rx_dly[2:0],i_cmd_frame_req};
    end
end

endmodule
