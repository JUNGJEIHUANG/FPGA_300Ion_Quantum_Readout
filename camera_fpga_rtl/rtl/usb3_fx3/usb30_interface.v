`timescale 1ns / 1ps
// USB3.0 FX3 interface.  TX carries image/results; RX carries the official
// Gpixel AA55 command packets.
module usb30_interface(
    input                      clk                     ,
    input                      rst                     ,
    input                      i_FX3_FLAGA             ,
    input                      i_FX3_FLAGB             ,
    output                     o_FX3_SLCS_N            ,
    output                     o_FX3_SLWR_N            ,
    output                     o_FX3_SLRD_N            ,
    output                     o_FX3_SLOE_N            ,
    output                     o_FX3_PKTEND_N          ,
    output           [1:0]     o_FX3_A                 ,
    inout            [31:0]    io_FX3_DB               ,
    output    reg              o_USB_RESET             ,
    input                      i_ack2tx_tx_en          ,
    input                      i_ack2tx_dat_vld        ,
    input            [31:0]    i_ack2tx_dat            ,
    input                      i_rlt2tx_tx_en          ,
    input                      i_rlt2tx_dat_vld        ,
    input            [31:0]    i_rlt2tx_dat            ,
    input                      i_img2tx_tx_en          ,
    input                      i_img2tx_dat_vld        ,
    input            [31:0]    i_img2tx_dat            ,
    output           [14:0]    o_tx_fifo_cnt           ,
    output                     o_tx_fifo_ren           ,
    output                     o_tx_fifo_full          ,
    output                     o_tx_fifo_empty         ,
    output                     o_tx_fifo_wr_en         ,
    output           [31:0]    o_usb_dbg               ,
    output           [31:0]    o_usb_tx_word_total     ,
    output           [31:0]    o_usb_pkt_total         ,
    output           [31:0]    o_flagb_low_total
    ,output    reg              o_rx_word_valid
    ,output    reg     [31:0]    o_rx_word
    ,output                     o_dbg_rx_flaga
    ,output                     o_dbg_fx3_flaga_rx
    ,output                     o_dbg_usb30_tx
    ,output                     o_dbg_rx_fifo_rd_en
    ,output                     o_dbg_rx_fifo_empty
    ,output           [31:0]    o_dbg_rx_fifo_rd_dat
    ,output           [1:0]     o_dbg_rx_a
);

wire    [31:0]    fx3_db_tx;
wire              fx3_slcs_n_tx;
wire              fx3_slwr_n_tx;
wire              fx3_slrd_n_tx;
wire              fx3_sloe_n_tx;
wire              fx3_pktend_n_tx;
wire    [1:0]     fx3_a_tx;
wire              tx_fifo_ren;
wire    [31:0]    tx_fifo_dout;
wire              usb30_tx;
wire              fx3_flaga_rx;
wire              fx3_slcs_n_rx;
wire              fx3_slwr_n_rx;
wire              fx3_slrd_n_rx;
wire              fx3_sloe_n_rx;
wire              fx3_pktend_n_rx;
wire       [1:0]  fx3_a_rx;
wire       [31:0] fx3_db_rx;
wire       [31:0] rx_fifo_rd_dat;
wire              rx_fifo_rd_empty;
reg               rx_fifo_rd_en;
reg               rx_fifo_rd_accept_d;
wire              tx_fifo_full;
wire              tx_fifo_empty;
wire    [13:0]    tx_fifo_data_count;
wire    [31:0]    usb_tx_dbg;
wire              tx_fifo_wr_en;
reg               tx_fifo_ren_d;
reg     [31:0]    usb_tx_word_total;
reg     [31:0]    usb_pkt_total;
reg     [31:0]    flagb_low_total;
reg               tx_en;
reg               dat2tx_dat_vld;
reg    [31:0]     dat2tx_dat;
reg    [25:0]     cnt_rst;
reg    [14:0]     fifo_cnt_r;

always @(posedge clk) begin
    if (rst) begin
        tx_en          <= 1'b0;
        dat2tx_dat_vld <= 1'b0;
        dat2tx_dat     <= 32'd0;
    end else begin
        tx_en <= i_rlt2tx_tx_en | i_img2tx_tx_en | i_ack2tx_tx_en;
        if (i_rlt2tx_dat_vld) begin
            dat2tx_dat_vld <= 1'b1;
            dat2tx_dat     <= i_rlt2tx_dat;
        end else if (i_img2tx_dat_vld) begin
            dat2tx_dat_vld <= 1'b1;
            dat2tx_dat     <= i_img2tx_dat;
        end else if (i_ack2tx_dat_vld) begin
            dat2tx_dat_vld <= 1'b1;
            dat2tx_dat     <= i_ack2tx_dat;
        end else begin
            dat2tx_dat_vld <= 1'b0;
            dat2tx_dat     <= 32'd0;
        end
    end
end

always @(posedge clk) begin
    if (rst) begin
        o_USB_RESET <= 1'b0;
        cnt_rst     <= 26'd0;
    end else begin
        if (cnt_rst >= 26'd100000)   // ~1ms @100MHz, matches working K7_Demo
            o_USB_RESET <= 1'b1;
        else begin
            o_USB_RESET <= 1'b0;
            cnt_rst     <= cnt_rst + 1'b1;
        end
    end
end

// Use the FIFO IP data_count as the TX count, same principle as K7_Demo.
// The local counter could desynchronise if wr_en is asserted while full.
assign tx_fifo_wr_en = dat2tx_dat_vld & ~tx_fifo_full;
assign o_tx_fifo_cnt = {1'b0, tx_fifo_data_count};
assign o_tx_fifo_ren = tx_fifo_ren;
assign o_tx_fifo_full= tx_fifo_full;
assign o_tx_fifo_empty= tx_fifo_empty;
assign o_tx_fifo_wr_en= tx_fifo_wr_en;
assign o_usb_dbg     = {tx_fifo_full, tx_fifo_empty, tx_fifo_wr_en, tx_fifo_ren,
                        i_FX3_FLAGB, tx_en, o_tx_fifo_cnt[13:0], usb_tx_dbg[11:0]};
assign o_usb_tx_word_total = usb_tx_word_total;
assign o_usb_pkt_total     = usb_pkt_total;
assign o_flagb_low_total   = flagb_low_total;
assign o_dbg_rx_flaga        = i_FX3_FLAGA;
assign o_dbg_fx3_flaga_rx    = fx3_flaga_rx;
assign o_dbg_usb30_tx        = usb30_tx;
assign o_dbg_rx_fifo_rd_en   = rx_fifo_rd_en;
assign o_dbg_rx_fifo_empty   = rx_fifo_rd_empty;
assign o_dbg_rx_fifo_rd_dat  = rx_fifo_rd_dat;
assign o_dbg_rx_a            = fx3_a_rx;

// USB-side cumulative debug counters in clk domain.
// usb_tx_word_total: words actually read from TX FIFO toward FX3.
// usb_pkt_total: one count at the end of each TX burst/packet.
// flagb_low_total: cycles where FX3 reports not writable/full.
always @(posedge clk) begin
    if (rst) begin
        tx_fifo_ren_d     <= 1'b0;
        usb_tx_word_total <= 32'd0;
        usb_pkt_total     <= 32'd0;
        flagb_low_total   <= 32'd0;
    end else begin
        tx_fifo_ren_d <= tx_fifo_ren;
        if (tx_fifo_ren)
            usb_tx_word_total <= usb_tx_word_total + 32'd1;
        if (tx_fifo_ren_d && !tx_fifo_ren)
            usb_pkt_total <= usb_pkt_total + 32'd1;
        if (!i_FX3_FLAGB)
            flagb_low_total <= flagb_low_total + 32'd1;
    end
end

// K7-Demo RX frontend: FX3 is sampled on the PCLK falling edge and data crosses
// into the system-clock domain through the same 32-bit independent-clock FIFO.
assign fx3_flaga_rx = (usb30_tx == 1'b0) ? i_FX3_FLAGA : 1'b0;
assign o_FX3_SLCS_N   = (usb30_tx == 1'b0) ? fx3_slcs_n_rx   : fx3_slcs_n_tx;
assign o_FX3_SLWR_N   = (usb30_tx == 1'b0) ? fx3_slwr_n_rx   : fx3_slwr_n_tx;
assign o_FX3_SLRD_N   = (usb30_tx == 1'b0) ? fx3_slrd_n_rx   : fx3_slrd_n_tx;
assign o_FX3_SLOE_N   = (usb30_tx == 1'b0) ? fx3_sloe_n_rx   : fx3_sloe_n_tx;
assign o_FX3_PKTEND_N = (usb30_tx == 1'b0) ? fx3_pktend_n_rx : fx3_pktend_n_tx;
assign o_FX3_A        = (usb30_tx == 1'b0) ? fx3_a_rx        : fx3_a_tx;

usb30_rx u_usb30_rx (
    .clk              (clk),
    .rst              (rst),
    .i_fullauto_en    (1'b0),
    .i_fullauto_vld   (1'b0),
    .i_fullauto_dat   (32'd0),
    .i_FX3_FLAGA      (fx3_flaga_rx),
    .o_FX3_SLCS_N     (fx3_slcs_n_rx),
    .o_FX3_SLWR_N     (fx3_slwr_n_rx),
    .o_FX3_SLRD_N     (fx3_slrd_n_rx),
    .o_FX3_SLOE_N     (fx3_sloe_n_rx),
    .o_FX3_PKTEND_N   (fx3_pktend_n_rx),
    .o_FX3_A          (fx3_a_rx),
    .i_FX3_DB         (fx3_db_rx),
    .i_rd_en          (rx_fifo_rd_en),
    .o_rd_dat         (rx_fifo_rd_dat),
    .o_rd_empty       (rx_fifo_rd_empty)
);

// The K7 FIFO has one read-cycle latency.  Adapt it to the existing parser's
// ready/valid input without bypassing the clock-domain crossing.
always @(posedge clk) begin
    if (rst) begin
        rx_fifo_rd_accept_d <= 1'b0;
        rx_fifo_rd_en   <= 1'b0;
        o_rx_word_valid <= 1'b0;
        o_rx_word       <= 32'd0;
    end else begin
        rx_fifo_rd_accept_d <= rx_fifo_rd_en & ~rx_fifo_rd_empty;
        rx_fifo_rd_en        <= ~rx_fifo_rd_empty;
        o_rx_word_valid      <= rx_fifo_rd_accept_d;
        if (rx_fifo_rd_accept_d)
            o_rx_word <= rx_fifo_rd_dat;
    end
end

usb30_tx u_usb30_tx (
    .clk            (clk           ),
    .rst            (rst           ),
    .o_tx_fifo_ren  (tx_fifo_ren   ),
    .i_tx_fifo_dout (tx_fifo_dout  ),
    .i_tx_fifo_cnt  (o_tx_fifo_cnt ),
    .i_tx_fifo_full (tx_fifo_full  ),
    .i_tx_en        (tx_en         ),
    .o_usb30_tx     (usb30_tx      ),
    .i_FX3_FLAGB    (i_FX3_FLAGB   ),
    .o_FX3_SLCS_N   (fx3_slcs_n_tx   ),
    .o_FX3_SLWR_N   (fx3_slwr_n_tx   ),
    .o_FX3_SLRD_N   (fx3_slrd_n_tx   ),
    .o_FX3_SLOE_N   (fx3_sloe_n_tx   ),
    .o_FX3_PKTEND_N (fx3_pktend_n_tx ),
    .o_FX3_A        (fx3_a_tx        ),
    .o_FX3_DB       (fx3_db_tx     ),
    .o_dbg          (usb_tx_dbg    )
);

// TX FIFO: common clock 100MHz, 32bit x 16384
fifo_w32d16384_ccbr_st u_fifo_tx (
    .clk   (clk           ),
    .rst   (rst           ),
    .din   (dat2tx_dat    ),
    .wr_en (tx_fifo_wr_en ),
    .full  (tx_fifo_full  ),
    .rd_en (tx_fifo_ren   ),
    .dout  (tx_fifo_dout  ),
    .empty (tx_fifo_empty ),
    .data_count(tx_fifo_data_count)
);

// IOBUF: output when usb30_tx=1, hi-Z when idle
genvar i;
generate
    for (i = 0; i < 32; i = i+1) begin: loop
        IOBUF u_iobuf_db (
            .O  (fx3_db_rx[i]),
            .I  (fx3_db_tx[i]),
            .IO (io_FX3_DB[i]),
            .T  (~usb30_tx)
        );
    end
endgenerate

endmodule






