`timescale 1ns / 1ps
//--------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//---------------------------------------------------------------------------------------------------------------
// --Module    : usb30_tx
// --File      : usb30_tx.v
// --Author    : 
//---------------------------------------------------------------------------------------------------------------
// --Function  : 
// --------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2022.07.24   danyang.li      modify
// --1.1       2022.12.28   Tianjiao Hou    mofify:code style,
//---------------------------------------------------------------------------------------------------------------
`define USB_PKT_SIZE    4096 
module usb30_tx(
    input                      clk           , //100MHz
    input                      rst           ,
    //----------------
    output    reg              o_tx_fifo_ren , // tx_fifo(fifo_w32d8192_ccbr_st in usb30_interface.v) read enable
    input            [31:0]    i_tx_fifo_dout, // tx_fifo data out
    input            [14:0]    i_tx_fifo_cnt , // tx_fifo data_count (14-bit IP output, zero when full)
    input                      i_tx_fifo_full, // asserted when the 16384-word TX FIFO is full
    input                      i_tx_en       , // 1: outside module transter data to tx_fifo, continuous high in once transmission 
    //----usb30--------------
    input                      i_FX3_FLAGB   , // USB3.0 Interface 
    output                     o_FX3_SLCS_N  , // USB3.0 Interface 
    output                     o_FX3_SLWR_N  , // USB3.0 Interface 
    output                     o_FX3_SLRD_N  , // USB3.0 Interface 
    output                     o_FX3_SLOE_N  , // USB3.0 Interface 
    output                     o_FX3_PKTEND_N, // USB3.0 Interface 
    output           [1:0]     o_FX3_A       , // USB3.0 Interface 
    output           [31:0]    o_FX3_DB      , // USB3.0 Interface 
    
    output    reg              o_usb30_tx    , // 1: transfer to FX3/usb3.0 , 0: can receive from FX3/usb3.0
    output           [31:0]    o_dbg           // debug: state/flag/write/pkt/delay/fifo status
);

//------------------------------------------------------------------------------------------------------------------
//parameter
//------------------------------------------------------------------------------------------------------------------

parameter   idle               =3'd0,
            rd_fifo            =3'd1,
            rd_end             =3'd2,
            delay_vs           =3'd3,
            check_delay        =3'd4,
            check_full         =3'd5,
            begin_transfer     =3'd6;

//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

reg              usb_wr;
reg              usb_pkt,usb_pkt_buf;
reg              usb_wr_full;
reg    [15:0]    delay_cnt;
reg    [1:0]     usb_tx_en_dly;
reg    [2:0]     usb_state=idle;
wire   [14:0]    tx_fifo_occupancy;

// The FIFO IP data_count wraps to zero at full; full carries the missing MSB.
assign tx_fifo_occupancy = i_tx_fifo_full ? 15'd16384 : i_tx_fifo_cnt;

//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------

assign     o_FX3_A           = 2'b00; 
assign     o_FX3_SLCS_N      = 1'b0;
assign     o_FX3_SLRD_N      = 1'b1;
assign     o_FX3_SLOE_N      = 1'b1;
assign     o_FX3_SLWR_N      = ~usb_wr;
assign     o_FX3_PKTEND_N    = ~usb_pkt;
assign     o_FX3_DB[31:0]    = i_tx_fifo_dout;
assign     o_dbg             = {3'd0,
                                (tx_fifo_occupancy >= `USB_PKT_SIZE),
                                (tx_fifo_occupancy == 15'd0),
                                i_tx_en,
                                delay_cnt,
                                usb_pkt_buf,
                                usb_pkt,
                                usb_wr,
                                o_tx_fifo_ren,
                                usb_wr_full,
                                i_FX3_FLAGB,
                                o_usb30_tx,
                                usb_state};

always @(posedge clk)begin
    if(rst)begin
        usb_state            <= idle;
        delay_cnt            <= 16'd0;
        usb_pkt              <= 1'b0;
        usb_pkt_buf          <= 1'b0;
        usb_wr               <= 1'b0;
        usb_wr_full          <= 1'b0;
        usb_tx_en_dly        <= 'd0;
        o_tx_fifo_ren        <= 1'b0;
        o_usb30_tx           <= 1'b0;
    end
    else begin
        usb_wr               <= o_tx_fifo_ren;
        usb_wr_full          <= ~i_FX3_FLAGB;
        usb_tx_en_dly        <= {usb_tx_en_dly[0],i_tx_en};
        case(usb_state)
            idle:            begin
                                o_usb30_tx          <=1'b0;
                                usb_pkt              <=1'b0;
                                usb_pkt_buf          <=1'b0;
                                delay_cnt            <=16'd0;
                                o_tx_fifo_ren        <=1'b0;
                                if(tx_fifo_occupancy>=`USB_PKT_SIZE)begin
                                    usb_state            <=begin_transfer;
                                    o_usb30_tx           <=1'b1;
                                end
                                else if(usb_tx_en_dly==2'b10)begin
                                    usb_state            <=begin_transfer;
                                    o_usb30_tx           <=1'b1;
                                end
                            end
            begin_transfer:    begin
                                // Do not launch the first packet until the selected
                                // FX3 producer socket reports writable.  The legacy
                                // FSM only checked FLAGB between packets, so an
                                // autonomous source could write one packet before a
                                // host bulk-IN request existed and wedge the link.
                                if((tx_fifo_occupancy>=`USB_PKT_SIZE) && i_FX3_FLAGB)begin
                                    usb_state       <= rd_fifo;
                                    delay_cnt       <= `USB_PKT_SIZE;
                                end
                                else if((!i_tx_en) && (tx_fifo_occupancy>0) && i_FX3_FLAGB)begin
                                    usb_pkt_buf     <= 1'b1;
                                    usb_state       <= rd_fifo;
                                    delay_cnt       <= tx_fifo_occupancy;
                                end
                                else if((!i_tx_en) && (tx_fifo_occupancy==0))
                                    usb_state       <= delay_vs;
                                else 
                                    usb_state       <= begin_transfer;
                            end
            rd_fifo:        begin
                                if(delay_cnt>0)begin
                                    o_tx_fifo_ren        <= 1'b1;
                                    delay_cnt            <= delay_cnt-1'b1;
                                end
                                else begin
                                    usb_pkt              <= usb_pkt_buf;
                                    o_tx_fifo_ren        <= 1'b0;
                                    usb_state            <= (usb_pkt_buf)?delay_vs:check_delay;
                                end
                            end
            check_delay:    begin
                                usb_pkt                 <= 1'b0;
                                usb_pkt_buf             <= 1'b0;
                                delay_cnt               <= delay_cnt+1'b1;
                                if(delay_cnt==10)begin
                                    usb_state           <= check_full;
                                end
                            end
            check_full:        begin
                                if(usb_wr_full)
                                    usb_state           <= check_full;
                                else
                                    usb_state           <= begin_transfer;
                            end
            delay_vs:       begin
                                o_usb30_tx             <= 1'b1;
                                usb_pkt                 <= 1'b0;
                                usb_pkt_buf             <= 1'b0;
                                delay_cnt               <= delay_cnt+1'b1;
                                if(delay_cnt==50)begin
                                    usb_state               <= idle;
                                    o_usb30_tx              <= 1'b0;
                                end
                            end
            default:        usb_state  <= idle;
        endcase
    end
end
endmodule
