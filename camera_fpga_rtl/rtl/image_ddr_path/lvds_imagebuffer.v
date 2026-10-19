`timescale 1ns / 1ps
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : lvds_imagebuffer
// --File      : lvds_imagebuffer.v
// --Author    : guangcai.ding@gpixel.com
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : lvds_imagebuffer module.
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2025.02.13   Guangcai Ding   Creat
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module lvds_imagebuffer #(
    parameter    CHAN_NUM       = 16   ,
    parameter    DAT_WIDTH      = 12   ,
    parameter    BRAM_DEPTH     = 4096
)
(
    input                                   clk_rx            ,
    input                                   rst_rx            ,
    input                                   clk_tx            ,
    input                                   rst_tx            ,
    input                                   i_cmd_frame_req   ,
    input     [1:0]                         i_hdr_mode_en     ,
    input     [1:0]                         i_flip_v_en       ,
    input                                   i_flip_h_en       ,
    input     [4:0]                         i_n_mux           ,
    input     [13:0]                        i_w_mux           ,
    input     [15:0]                        i_lvalout_len     ,
    input                                   i_din_vld         ,
    input     [CHAN_NUM*DAT_WIDTH-1:0]      i_din             ,    // bot [15:8] ; top [7:0] 
    output    [CHAN_NUM-1:0]                o_chan_vld        ,
    output                                  o_dout_vld        ,
    output    [CHAN_NUM*2*DAT_WIDTH-1:0]    o_dout            ,
    output    [31:0]                        o_dbg
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

localparam    BRAM_ADDR_WIDTH    = $clog2(BRAM_DEPTH) ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

wire    [CHAN_NUM/2-1:0]                        bram_rd_en   ;
wire    [CHAN_NUM/2*(BRAM_ADDR_WIDTH-2)-1:0]    bram_rd_addr ;
wire                                            pipe_flag_top;
wire                                            row1_done_top;
wire                                            pipe_flag_bot;
wire                                            row1_done_bot;
wire    [CHAN_NUM*2*DAT_WIDTH-1:0]              dat_out_top  ;
wire    [CHAN_NUM*2*DAT_WIDTH-1:0]              dat_out_bot  ;
wire    [31:0]                                  synrd_dbg    ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

chan_mult    u_chan_mult_top
(
    .clk_rx                 (clk_rx                         ),
    .rst_rx                 (rst_rx                         ),
    .clk_tx                 (clk_tx                         ),
    .i_cmd_frame_req        (i_cmd_frame_req                ),
    .i_din_vld              (i_din_vld                      ),
    .i_din                  (i_din[CHAN_NUM*DAT_WIDTH/2-1:0]),
    .i_flip_h_en            (i_flip_h_en                    ),
    .i_n_mux                (i_n_mux                        ),
    .i_w_mux                (i_w_mux                        ),
    .i_lvalout_len          (i_lvalout_len                  ),
    .i_bram_rd_en           (bram_rd_en                     ),
    .i_bram_rd_addr         (bram_rd_addr                   ),
    .o_pipe_flag            (pipe_flag_top                  ),
    .o_row1_done            (row1_done_top                  ),
    .o_chan_vld             (o_chan_vld[CHAN_NUM/2-1:0]     ),
    .o_dout                 (dat_out_top                    )
);

chan_mult    u_chan_mult_bot
(
    .clk_rx                 (clk_rx                                          ),
    .rst_rx                 (rst_rx                                          ),
    .clk_tx                 (clk_tx                                          ),
    .i_cmd_frame_req        (i_cmd_frame_req                                 ),
    .i_din_vld              (i_din_vld                                       ),
    .i_din                  (i_din[CHAN_NUM*DAT_WIDTH-1:CHAN_NUM*DAT_WIDTH/2]),
    .i_flip_h_en            (i_flip_h_en                                     ),
    .i_n_mux                (i_n_mux                                         ),
    .i_w_mux                (i_w_mux                                         ),
    .i_lvalout_len          (i_lvalout_len                                   ),
    .i_bram_rd_en           (bram_rd_en                                      ),
    .i_bram_rd_addr         (bram_rd_addr                                    ),
    .o_pipe_flag            (pipe_flag_bot                                   ),
    .o_row1_done            (row1_done_bot                                   ),
    .o_chan_vld             (o_chan_vld[CHAN_NUM-1:CHAN_NUM/2]               ),
    .o_dout                 (dat_out_bot                                     )
);

chan_top_bot_synrd_out    u_chan_top_bot_synrd_out
(
    .clk                    (clk_tx         ),
    .rst                    (rst_tx         ),
    .i_cmd_frame_req        (i_cmd_frame_req),
    .i_hdr_mode_en          (i_hdr_mode_en  ),
    .i_flip_v_en            (i_flip_v_en    ),
    .i_flip_h_en            (i_flip_h_en    ),
    .o_bram_rd_en           (bram_rd_en     ),
    .o_bram_rd_addr         (bram_rd_addr   ),
    .i_pipe_flag_top        (pipe_flag_top  ),
    .i_row1_done_top        (row1_done_top  ),
    .i_pipe_flag_bot        (pipe_flag_bot  ),
    .i_row1_done_bot        (row1_done_bot  ),
    .i_dat_out_top          (dat_out_top    ),
    .i_dat_out_bot          (dat_out_bot    ),
    .o_dout_vld             (o_dout_vld     ),
    .o_dout                 (o_dout         ),
    .o_dbg                  (synrd_dbg      )
);

assign o_dbg = {
    synrd_dbg[31:16],   // [31:16] chan_top_bot_synrd_out high-level state
    i_cmd_frame_req,    // [15]
    i_din_vld,          // [14]
    o_dout_vld,         // [13]
    row1_done_top,      // [12]
    row1_done_bot,      // [11]
    pipe_flag_top,      // [10]
    pipe_flag_bot,      // [9]
    o_chan_vld[7:0]     // [8:1]
    ,synrd_dbg[0]       // [0]
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

endmodule
