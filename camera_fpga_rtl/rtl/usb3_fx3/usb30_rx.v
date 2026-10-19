`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//------------------------------------------------------------------------------------------------------------------
// --Module    : usb30_rx
// --File      : usb30_rx.v
// --Author    : 
//------------------------------------------------------------------------------------------------------------------
// --Function  : receive CMD and data from PC
//------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2022.07.20   danyang.li      create
// --1.1       2023.01.03   Tianjiao Hou    mofify:code style,
//------------------------------------------------------------------------------------------------------------------
module usb30_rx(
    input                      clk               , // 100MHz
    input                      rst               ,
    input                      i_fullauto_en     , // negedeg clk domain, fully automatic enable,continuous 1 When working on fully automatic, 
    input                      i_fullauto_vld    , // negedeg clk domain, fully automatic valid flag of i_fullauto_dat
    input            [31:0]    i_fullauto_dat    , // negedeg clk domain, fully automatic data 
    input                      i_FX3_FLAGA       , // USB3.0 Interface  
    output    reg              o_FX3_SLCS_N      , // USB3.0 Interface     //Slave FIFO Select siginal
    output    reg              o_FX3_SLWR_N      , // USB3.0 Interface     //Slave FIFO Enable siginal
    output    reg              o_FX3_SLRD_N      , // USB3.0 Interface     //Slave FIFO Read enable
    output    reg              o_FX3_SLOE_N      , // USB3.0 Interface     //Slave FIFO Output enable
    output    reg              o_FX3_PKTEND_N    , // USB3.0 Interface     //Package Finish Siginal
    output           [1 :0]    o_FX3_A           , // USB3.0 Interface     //The Operated FIFO ADDRESS
    input            [31:0]    i_FX3_DB          , // USB3.0 Interface     //Input data bus
    input                      i_rd_en           , // rx_fifo read enable,
    output           [31:0]    o_rd_dat          , // rx_fifo data out.  Read mode of rx_fifo is Standard FIFO, so rx_fifo read latency is 1
    output                     o_rd_empty          // rx_fifo empty flag
);

//------------------------------------------------------------------------------------------------------------------
//parameter
//------------------------------------------------------------------------------------------------------------------

localparam     fx3_idle     = 1'b0,
               fx3_read     = 1'b1;
localparam     fx3_on       = 1'b0;
localparam     fx3_off      = 1'b1;

//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

reg    [9 :0]    num_data          ;
reg              fsm_fx3_rx        ;
reg    [31:0]    fx3_rdb           ;
reg              fx3_rdb_en        ;
reg              fullauto_en_1dly  ;
reg              fullauto_en_2dly  ;
reg              fullauto_vld_1dly ;
reg              fullauto_vld_2dly ;
reg    [31:0]    fullauto_dat_1dly ;
reg    [31:0]    fullauto_dat_2dly ;
reg              rx_fifo_wen       ;
reg    [31:0]    rx_fifo_din       ;

//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------

assign    o_FX3_A = 2'b11;

//always @(posedge clk_n or posedge rst) begin
always @(negedge clk or posedge rst) begin
    if(rst == 1) begin
        o_FX3_SLCS_N    <= fx3_off;
        o_FX3_SLWR_N    <= fx3_off;
        o_FX3_SLRD_N    <= fx3_off;
        o_FX3_SLOE_N    <= fx3_off;
        o_FX3_PKTEND_N  <= fx3_off;
        fsm_fx3_rx      <= fx3_idle;
        fx3_rdb         <= 32'd0;
        fx3_rdb_en      <= 1'b0;
        num_data        <= 10'd0;
    end
    else begin
        fx3_rdb         <= i_FX3_DB;
        case(fsm_fx3_rx)
            fx3_idle:    begin
                            if(i_FX3_FLAGA == 1'b1)begin
                                fsm_fx3_rx       <= fx3_read;
                                o_FX3_SLCS_N     <= fx3_on;
                                o_FX3_SLRD_N     <= fx3_on;
                                o_FX3_SLOE_N     <= fx3_on;
                                o_FX3_SLWR_N     <= fx3_off;
                                o_FX3_PKTEND_N   <= fx3_off;
                            end
                            else begin
                                fsm_fx3_rx       <= fx3_idle;
                                o_FX3_SLCS_N     <= fx3_off;
                                o_FX3_SLWR_N     <= fx3_off;
                                o_FX3_SLRD_N     <= fx3_off;
                                o_FX3_SLOE_N     <= fx3_off;
                                o_FX3_PKTEND_N   <= fx3_off;
                                fx3_rdb          <= 32'd0; 
                                fx3_rdb_en       <= 1'b0;
                                num_data         <= 10'd0;
                            end
                
                        end
            fx3_read:   begin
                            num_data             <= num_data + 1'b1;
                            if(i_FX3_FLAGA == 1'b0) begin
                                o_FX3_SLCS_N     <= fx3_off;
                                o_FX3_SLWR_N     <= fx3_off;
                                o_FX3_SLRD_N     <= fx3_off;
                                o_FX3_SLOE_N     <= fx3_off;
                                o_FX3_PKTEND_N   <= fx3_off;
                                fsm_fx3_rx       <= fx3_idle;
                                fx3_rdb_en       <= 1'b0;
                            end
                            else begin
                                if(num_data >= 10'd2) begin
                                    fx3_rdb_en   <= 1'b1;
                                end
                            end
                        end
        endcase
    end
end

always @(negedge clk or posedge rst) begin
    if(rst == 1) begin 
        rx_fifo_wen      <= 1'b0;
        rx_fifo_din      <= 'd0 ; 
    end
    else begin 
        if(fullauto_en_2dly==1'b1)begin
            rx_fifo_wen      <= fullauto_vld_2dly ;
            rx_fifo_din      <= fullauto_dat_2dly ;
        end
        else begin
            rx_fifo_wen      <= fx3_rdb_en;
            rx_fifo_din      <= fx3_rdb   ;
        end
    end
end
//--------------------------------------------------------------------------------------------------------------------
//Instance
//--------------------------------------------------------------------------------------------------------------------
//Basic:        Native 
//              Inpendent Clock Block RAM 
//Native Ports: Standard FIFO 
//              Write/read width 32 bit
//              Depth 16384
//              Reset Pin: select
//              Enable Reset Synchronization : select
//              Enable Safety Circuit : not select
fifo_w32d16384_icbr_st    u_fifo_w32d16384_icbr_st (
    .rst          (rst              ),
    .wr_clk       (~clk             ),
    .rd_clk       (clk              ),
    .din          (rx_fifo_din      ),
    .wr_en        (rx_fifo_wen      ),
    .rd_en        (i_rd_en          ),
    .dout         (o_rd_dat         ),
    .full         (                 ),
    .empty        (o_rd_empty       ) 
);
//--------------------------------------------------------------------------------------------------------------------
//CDC
//--------------------------------------------------------------------------------------------------------------------
always @(negedge clk or posedge rst) begin
    if(rst == 1) begin
        fullauto_en_1dly   <= 1'b0;
        fullauto_en_2dly   <= 1'b0;
        fullauto_vld_1dly  <= 1'b0;
        fullauto_vld_2dly  <= 1'b0;
        fullauto_dat_1dly  <= 'd0;
        fullauto_dat_2dly  <= 'd0; 
    end
    else begin
        fullauto_en_1dly         <= i_fullauto_en ;
        fullauto_en_2dly         <= fullauto_en_1dly ;
        fullauto_vld_1dly        <= i_fullauto_vld;
        fullauto_vld_2dly        <= fullauto_vld_1dly;
        fullauto_dat_1dly        <= i_fullauto_dat;
        fullauto_dat_2dly        <= fullauto_dat_1dly;
    end
end

endmodule