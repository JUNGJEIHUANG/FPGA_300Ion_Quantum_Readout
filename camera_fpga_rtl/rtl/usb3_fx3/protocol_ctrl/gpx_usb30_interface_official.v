`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//------------------------------------------------------------------------------------------------------------------
// --Module    : usb30_interface
// --File      : usb30_interface.v
// --Author    : 
//------------------------------------------------------------------------------------------------------------------
// --Function  : USB3.0 interface 
//------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2022.07.12   Danyang.li      modify style,add comment
// --1.1       2022.12.28   Tianjiao Hou    mofify:code style. ack_mode02 deleted.
//------------------------------------------------------------------------------------------------------------------
module gpx_usb30_interface_official(
    input                      clk                     , // FX3 interface used clock, 100MHz fixed
    input                      rst                     , 
    input                      i_fullauto_en           , // negedeg clk domain, fully automatic enable,continuous 1 When working on fully automatic, 
    input                      i_fullauto_vld          , // negedeg clk domain, fully automatic valid flag of i_fullauto_dat
    input            [31:0]    i_fullauto_dat          , // negedeg clk domain, fully automatic data 
    input                      i_FX3_FLAGA             , // USB3.0 Interface  
    input                      i_FX3_FLAGB             , // USB3.0 Interface  
    output                     o_FX3_SLCS_N            , // USB3.0 Interface  
    output                     o_FX3_SLWR_N            , // USB3.0 Interface  
    output                     o_FX3_SLRD_N            , // USB3.0 Interface  
    output                     o_FX3_SLOE_N            , // USB3.0 Interface  
    output                     o_FX3_PKTEND_N          , // USB3.0 Interface  
    output           [1:0]     o_FX3_A                 , // USB3.0 Interface  
    inout            [31:0]    io_FX3_DB               , // USB3.0 Interface  
    output    reg              o_USB_RESET             , // USB3.0 Interface  
    input                      i_rx_fifo_rd_en         , // rx_fifo data out, rx_fifo read latency is 1
    output           [31:0]    o_rx_fifo_rd_dat        , // rx_fifo read enable 
    output                     o_rx_fifo_rd_empty      , // rx_fifo empty flag 
    input                      i_ack2tx_tx_en          , // 1: when ACK without result, transter i_ack2tx_dat to tx_fifo. continuous high in once transmission 
                                                         // 0: when ACK with result or image
    input                      i_ack2tx_dat_vld        , // valid flag of i_ack2tx_dat
    input            [31:0]    i_ack2tx_dat            , // ack data to usb(tx_fifo)
    input                      i_rlt2tx_tx_en          , // 1:transter i_ack2tx_dat to tx_fifo. continuous high in once transmission 
    input                      i_rlt2tx_dat_vld        , // valid flag of i_rlt2tx_dat
    input            [31:0]    i_rlt2tx_dat            , // command result data to usb(tx_fifo)
    input                      i_img2tx_tx_en          , // 1:transter i_img2tx_dat to tx_fifo. continuous high in once transmission 
    input                      i_img2tx_dat_vld        , // valid flag of i_img2tx_dat
    input            [31:0]    i_img2tx_dat            , // image data to usb(tx_fifo)
    output           [12:0]    o_tx_fifo_cnt           , // tx_fifo data count, low 13 bits used by vendor TX FSM
    output                     o_dbg_tx_word_strobe    , // FPGA-driven GPIF write edge, independent of FLAGB
    output                     o_dbg_tx_word_accepted  , // GPIF write accepted by FX3 (SLWR_N=0 and FLAGB=1)
    output                     o_dbg_tx_word_is_image  , // tag aligned with the word currently driven to FX3
    output           [31:0]    o_dbg_tx_word           ,
    output           [14:0]    o_dbg_tag_fifo_count    ,
    output                     o_dbg_tag_fifo_full     ,
    output                     o_dbg_tag_fifo_empty
);
//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

wire                  fx3_flaga_rx;
wire                  fx3_slcs_n_rx;
wire                  fx3_slcs_n_tx;
wire                  fx3_slwr_n_rx;
wire                  fx3_slwr_n_tx;
wire                  fx3_slrd_n_rx;
wire                  fx3_slrd_n_tx;
wire                  fx3_sloe_n_rx;
wire                  fx3_sloe_n_tx;
wire                  fx3_pktend_n_rx;
wire                  fx3_pktend_n_tx;
wire    [1:0]         fx3_a_rx;
wire    [1:0]         fx3_a_tx;
wire    [31:0]        fx3_db_tx;
wire    [31:0]        fx3_db_rx;
reg     [19:0]        cnt_rst;
wire                  usb30_tx; 
reg                   tx_en;
reg                   dat2tx_dat_vld;
reg     [31:0]        dat2tx_dat;
reg                   dat2tx_is_image;
wire                  tx_fifo_ren;
wire    [31:0]        tx_fifo_dout;
wire    [32:0]        tx_fifo_dout_tagged;
wire                  tx_fifo_full;
wire                  tx_fifo_empty;
wire    [13:0]        tx_fifo_count_raw;
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// assign     io_FX3_DB          = (usb30_tx == 1'b0) ? 32'hzzzzzzzz    : fx3_db_tx;
// assign     fx3_db_rx          = (usb30_tx == 1'b0) ? io_FX3_DB       : 32'd0;
assign     fx3_flaga_rx       = (usb30_tx == 1'b0) ? i_FX3_FLAGA     : 1'b0;
assign     o_FX3_SLCS_N       = (usb30_tx == 1'b0) ? fx3_slcs_n_rx   : fx3_slcs_n_tx;
assign     o_FX3_SLWR_N       = (usb30_tx == 1'b0) ? fx3_slwr_n_rx   : fx3_slwr_n_tx;
assign     o_FX3_SLRD_N       = (usb30_tx == 1'b0) ? fx3_slrd_n_rx   : fx3_slrd_n_tx;
assign     o_FX3_SLOE_N       = (usb30_tx == 1'b0) ? fx3_sloe_n_rx   : fx3_sloe_n_tx;
assign     o_FX3_PKTEND_N     = (usb30_tx == 1'b0) ? fx3_pktend_n_rx : fx3_pktend_n_tx;
assign     o_FX3_A            = (usb30_tx == 1'b0) ? fx3_a_rx        : fx3_a_tx;
assign     o_dbg_tx_word_strobe = usb30_tx && !fx3_slcs_n_tx && !fx3_slwr_n_tx && (fx3_a_tx == 2'b00);
assign     o_dbg_tx_word_accepted = usb30_tx && !fx3_slcs_n_tx && !fx3_slwr_n_tx &&
                                    (fx3_a_tx == 2'b00) && i_FX3_FLAGB;
assign     o_dbg_tx_word_is_image = tx_fifo_dout_tagged[32];
assign     o_dbg_tx_word          = fx3_db_tx;
assign     o_dbg_tag_fifo_count   = tx_fifo_full ? 15'd16384 : {1'b0, tx_fifo_count_raw};
assign     o_dbg_tag_fifo_full    = tx_fifo_full;
assign     o_dbg_tag_fifo_empty   = tx_fifo_empty;
assign     o_tx_fifo_cnt          = tx_fifo_count_raw[12:0];
assign     tx_fifo_dout           = tx_fifo_dout_tagged[31:0];

    //------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------

always @(posedge clk) begin 
    if(rst == 1'b1) begin
        tx_en            <= 1'b0;
        dat2tx_dat_vld   <= 1'b0;
        dat2tx_dat       <= 32'd0;
        dat2tx_is_image  <= 1'b0;
    end
    else begin 
        tx_en            <= i_rlt2tx_tx_en | i_img2tx_tx_en | i_ack2tx_tx_en;
        if(i_rlt2tx_dat_vld == 1'b1) begin
            dat2tx_dat_vld    <= 1'b1;
            dat2tx_dat        <= i_rlt2tx_dat;
            dat2tx_is_image   <= 1'b0;
        end
        else if(i_img2tx_dat_vld == 1'b1) begin
            dat2tx_dat_vld    <= 1'b1;
            dat2tx_dat        <= i_img2tx_dat;
            dat2tx_is_image   <= 1'b1;
        end
        else if(i_ack2tx_dat_vld == 1'b1) begin
            dat2tx_dat_vld    <= 1'b1;
            dat2tx_dat        <= i_ack2tx_dat;
            dat2tx_is_image   <= 1'b0;
        end
        else begin
            dat2tx_dat_vld    <= 1'b0;
            dat2tx_dat        <= 32'd0;
            dat2tx_is_image   <= 1'b0;
        end
    end
end

always @(posedge clk) begin //reset FX3 for 1ms after system power on 
    if(rst == 1'b1) begin
        o_USB_RESET        <= 1'b0;
        cnt_rst        <= 12'd0;
    end
    else begin
        if(cnt_rst >= 'd100000) begin
           o_USB_RESET     <= 1'b1;
        end
        else begin
            o_USB_RESET    <= 1'b0;
            cnt_rst    <= cnt_rst + 1'b1;
        end
    end
end

//--------------------------------------------------------------------------------------------------------------------
//Instance
//--------------------------------------------------------------------------------------------------------------------

gpx_usb30_rx_official   u_usb30_rx(
    .clk                    (clk               ),
    .rst                    (rst               ),
    .i_fullauto_en          (i_fullauto_en     ),
    .i_fullauto_vld         (i_fullauto_vld    ),
    .i_fullauto_dat         (i_fullauto_dat    ),
    .i_FX3_FLAGA            (fx3_flaga_rx      ),
    .o_FX3_SLCS_N           (fx3_slcs_n_rx     ),
    .o_FX3_SLWR_N           (fx3_slwr_n_rx     ),
    .o_FX3_SLRD_N           (fx3_slrd_n_rx     ),
    .o_FX3_SLOE_N           (fx3_sloe_n_rx     ),
    .o_FX3_PKTEND_N         (fx3_pktend_n_rx   ),
    .o_FX3_A                (fx3_a_rx          ),
    .i_FX3_DB               (fx3_db_rx         ),
    .i_rd_en                (i_rx_fifo_rd_en   ),
    .o_rd_dat               (o_rx_fifo_rd_dat  ),
    .o_rd_empty             (o_rx_fifo_rd_empty)
    );


gpx_usb30_tx_official    u_usb30_tx(
    .clk                    (clk               ),
    .rst                    (rst               ),
    .o_tx_fifo_ren          (tx_fifo_ren       ),
    .i_tx_fifo_dout         (tx_fifo_dout      ),
    .i_tx_fifo_cnt          (o_tx_fifo_cnt     ),
    .i_tx_en                (tx_en             ),
    .o_usb30_tx             (usb30_tx          ),
    .i_FX3_FLAGB            (i_FX3_FLAGB       ),
    .o_FX3_SLCS_N           (fx3_slcs_n_tx     ),
    .o_FX3_SLWR_N           (fx3_slwr_n_tx     ),
    .o_FX3_SLRD_N           (fx3_slrd_n_tx     ),
    .o_FX3_SLOE_N           (fx3_sloe_n_tx     ),
    .o_FX3_PKTEND_N         (fx3_pktend_n_tx   ),
    .o_FX3_A                (fx3_a_tx          ), 
    .o_FX3_DB               (fx3_db_tx         )
);



// Dedicated 33-bit TX FIFO. Bit 32 is an atomic image/non-image tag and bits
// 31:0 are the unchanged vendor payload. Keeping data and tag in the same
// generated FIFO avoids a one-cycle boundary ambiguity between unlike FIFOs.
fifo_w33d16384_ccbr_st    u_fifo_w33d16384_ccbr_st( //FIFO,tx to USB
    .clk                    (clk               ), // USB interface 100MHz clock
    .rst                    (rst               ), 
    .din                    ({dat2tx_is_image, dat2tx_dat}),
    .wr_en                  (dat2tx_dat_vld    ),
    .full                   (tx_fifo_full      ),
    .rd_en                  (tx_fifo_ren       ),
    .dout                   (tx_fifo_dout_tagged),
    .empty                  (tx_fifo_empty     ),
    .data_count             (tx_fifo_count_raw ),
    .wr_rst_busy            (                  ),
    .rd_rst_busy            (                  )
);

genvar    i;
generate
    for (i = 0; i < 32; i = i+1) begin:loop
        IOBUF IOBUF_db (
           .O   (fx3_db_rx[i]   ),
           .I   (fx3_db_tx[i]   ),
           .IO  (io_FX3_DB[i]   ),
           .T   (~usb30_tx      )  // 1:input, 0:output 
        );
    end
endgenerate


endmodule
