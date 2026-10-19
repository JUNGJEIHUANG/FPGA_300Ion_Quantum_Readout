`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : ddr_dout_buf
// --File      : ddr_dout_buf.v
// --Author    : weihan.li@gpixel.com
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : Transmit data buffer module
//                 1. BRAM Buffer operation is pipeline
//                 2. Support transmitted mode is interlace mode & serial mode
//                      - mode sample:
//                          interlace mode: (INTERLACE_EN=1)
//                                                  col 0 per chan     col 1 per chan          col lval-1 per chan
//                                    Channel 0    :  Pix<0>            , Pix<n>            , ... Pix<n*(lval-1)>
//                                    Channel 1    :  Pix<1>            , Pix<n+1>          , ... Pix<n*(lval-1)+1>
//                                      ...            ...                  ...                       ...
//                                    Channel n-1  :  Pix<n-1>          , Pix<2*n-1>         , ...Pix<n*lval-1>
//                          serial mode:   (INTERLACE_EN=0)
//                                    Channel 0    :  Pix<0>            , Pix<2>            , ... Pix<2*lval-2>
//                                    Channel 1    :  Pix<1>            , Pix<3>            , ... Pix<2*lval-1>
//                                      ...            ...                  ...               ...
//                                    Channel n-2  :  Pix<(n-2)*lval>   , Pix<(n-2)*lval+2> , ... Pix<n*lval-2>
//                                    Channel n-1  :  Pix<(n-2)*lval+1> , Pix<(n-2)*lval+3> , ... Pix<n*lval-1> 
//                 3. Output channel number can be reduced
//                      - DOUBLE_IN_EN = 0 : o_dout's format is same to i_din as above
//                      - DOUBLE_IN_EN = 1 : i_din as above, o_dout as follows 
//                           interlace mode: (INTERLACE_EN=1)
//                                                     col 0 per chan      col 1 per chan      col lval-1 per chan       col lval per chan                  col 2*lval-1 per chan     
//                                     Channel 0      :  Pix<0>            , Pix<n/2>          , Pix<n*(lval-1)/2>       , Pix<n*lval/2>          ,   ...   , Pix<n*(2*lval-1)/2>    
//                                     Channel 1      :  Pix<1>            , Pix<n/2+1>        , Pix<n*(lval-1)/2+1>     , Pix<n*lval/2+1>        ,   ...   , Pix<n*(2*lval-1)/2+1>  
//                                       ...            ...                     ...                ...                       ...                  ,   ...   ,    ...                 
//                                     Channel n/2-1  :  Pix<n/2-1>        , Pix<n-1>          , Pix<n*lval/2-1>         , Pix<n*(lval+1)/2>      ,   ...   , Pix<n*lva-1>      
//                           serial mode:   (INTERLACE_EN=0)
//                                     Channel 0      :  Pix<0>            , Pix<2>            , Pix<2*lval-2>           , Pix<2*lval>            ,   ....  , Pix<4*lval-2>
//                                     Channel 1      :  Pix<1>            , Pix<3>            , Pix<2*lval-1>           , Pix<2*lval+1>          ,   ....  , Pix<4*lval-1>
//                                       ...            ...                  ...               ....                  ...               ....
//                                     Channel n/2-2  :  Pix<(n-4)*lval>   , Pix<(n-4)*lval+2> , Pix<(n-2)*lval-2>       , Pix<(n-2)*lval>        ,   ....  , Pix<n*lval-2> 
//                                     Channel n/2-1  :  Pix<(n-4)*lval+1> , Pix<(n-4)*lval+3> , Pix<(n-2)*lval-1>       , Pix<(n-2)*lval+1>      ,   ....  , Pix<n*lval-1> 
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2023.11.16   Weihan.li       Creat
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module ddr_dout_buf #(
    parameter    INTERLACE_EN    = 1    , // Channel output as pixel interlace mode enable
    parameter    BRAM_DEPTH      = 1024 , // 1024, 2048, 4096: 2**BRAM_ADDR_WIDTH // >=2*i_lval_len_wr, BRAM Buffer is pipeline
    parameter    BRAM_ADDR_WIDTH = 10   , // 10, 11, 12  // bit width of BRAM's address
    parameter    DAT_WIDTH_IN    = 12   , // Data width of data in(i_din), maximum is 16
    parameter    DAT_WIDTH_OUT   = 16   , // Data width of data out(o_dout),16 fixed
    parameter    DOUBLE_IN_EN    = 1    , // Output channel reduce enable, 1: input channel number is 2 times than output channel number; 0: input channel number is equal to output channel number
    parameter    CHAN_NUM_IN     = 32   , // Input channel number  //not editable, CHAN_NUM_IN= CHAN_NUM_OUT*(DOUBLE_IN_EN+1)
    parameter    CHAN_NUM_OUT    = 16    ) // Output channel number // CHAN_NUM_OUT%2=0
(
// Clock & reset
    input                                                clk_wr          , // used to recive data from DDR4
    input                                                rst_wr          , // used to recive data from DDR4
    input                                                clk_rd          , // used to tranfer data to USB
    input                                                rst_rd          , // used to tranfer data to USB
    input                                                i_cmd_frame_req , // command of frame requeset,rising edge effective  
// Configuration
    input            [BRAM_ADDR_WIDTH-1:0]               i_lval_len_wr   , // length of one row of i_din; // DOUBLE_IN_EN=0: one row of  o_dout;  DOUBLE_IN_EN=1: 1/2 row of of o_dout;
// Data in
    input                                                i_din_vld       , // valid flag of i_din
    input            [CHAN_NUM_IN*DAT_WIDTH_IN-1:0]      i_din           , // data from axi_top from axi_top/image_ddr_driver
    output    reg                                        o_din_req       , // BRAM is ready to receive next row of input data //clk_rd clock domain
// Data read out
    input            [CHAN_NUM_OUT-1:0]                  i_dout_rd_en    , // read enbale of o_dout
    output    reg    [CHAN_NUM_OUT-1:0]                  o_dout_vld      , // valid flag of o_dout, have 4 clock delay with i_dout_rd_en
    output    reg    [CHAN_NUM_OUT*DAT_WIDTH_OUT-1:0]    o_dout          , // output data
    output    reg                                        o_dout_rdy        // BRAM is ready to output data, ready to receive i_dout_rd_en
);

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Reset
reg     [3:0]                   cmd_frame_req_wr_dly         ;
reg     [4:0]                   cnt_frame_req_wr             ;
reg                             frame_rst_wr                 ;
reg     [3:0]                   cmd_frame_req_rd_dly         ;
reg     [4:0]                   cnt_frame_req_rd             ;
reg                             frame_rst_rd                 ;

// Write counter & BRAM write ports
reg     [BRAM_ADDR_WIDTH:0]     cnt_bram_wr                  ;    // Write in BRAM counter
reg                             bram_wr_en                   ;    // MIG read out buffer BRAM
reg     [CHAN_NUM_IN*16-1:0]    bram_wr_dat                  ;
reg     [BRAM_ADDR_WIDTH:0]     bram_wr_addr                 ;
reg     [2:0]                   bram_wr_en_dly               ;
// Read counter & BRAM read ports
reg     [BRAM_ADDR_WIDTH-1:0]   cnt_rd_len                   ;
reg     [3:0]                   chan_rd_flag                 ;
reg     [CHAN_NUM_IN-1:0]       bram_rd_en                   ;
reg     [BRAM_ADDR_WIDTH:0]     bram_rd_addr[CHAN_NUM_IN-1:0];
wire    [CHAN_NUM_IN*16-1:0]    bram_rd_dat                  ;    // Buffer BRAM read out data, data width pre channel is 16 bit
// Delay
reg     [CHAN_NUM_OUT-1:0]      rd_en_1dly                   ;
reg     [CHAN_NUM_OUT-1:0]      rd_en_2dly                   ;

// CDC
reg     [BRAM_ADDR_WIDTH-1:0]   lval_len_wr_1dly             ;
reg     [BRAM_ADDR_WIDTH-1:0]   lval_len_wr_2dly             ;
reg     [BRAM_ADDR_WIDTH-1:0]   lval_len_wr_3dly             ;
reg     [BRAM_ADDR_WIDTH-1:0]   lval_len_rd_1dly             ;   // length of read BRAM, equal to lval_len_wr.  not length of o_dout&o_dout_vld
reg     [BRAM_ADDR_WIDTH-1:0]   lval_len_rd_2dly             ;
reg     [BRAM_ADDR_WIDTH-1:0]   lval_len_rd_3dly             ;

// BRAM IP pipeline check
reg                             wr_flag                      ;
reg     [3:0]                   wr_flag_dly                  ;
reg                             rd_flag                      ;
reg     [3:0]                   rd_flag_dly                  ;
reg     [1:0]                   bram_buf_rdy                 ;
reg                             bram_full                    ;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
// -- Reset generator
//--------------------------------------------------------------------------------
always @(posedge clk_wr or posedge rst_wr) begin
    if(rst_wr == 1'b1) begin
        cnt_frame_req_wr            <= 'd0;
        frame_rst_wr                <= 1'b1;
    end
    else begin
        if(cmd_frame_req_wr_dly[3:2] == 2'b01) begin
            cnt_frame_req_wr            <= 'd15;
            frame_rst_wr                <= 1'b1;
        end
        else begin
            if(cnt_frame_req_wr == 'd0)begin
                frame_rst_wr                <= 1'b0;
            end
            else begin
                cnt_frame_req_wr            <= cnt_frame_req_wr - 1'b1;
            end
        end
    end
end

always @(posedge clk_rd or posedge rst_rd) begin
    if(rst_rd == 1'b1) begin
        cnt_frame_req_rd            <= 'd0;
        frame_rst_rd                <= 1'b1;
    end
    else begin
        if(cmd_frame_req_rd_dly[3:2] == 2'b01)begin 
            cnt_frame_req_rd            <= 'd15;
            frame_rst_rd                <= 1'b1;
        end
        else begin
            if(cnt_frame_req_rd == 'd0)begin
                frame_rst_rd                <= 1'b0;
            end
            else begin
                cnt_frame_req_rd            <= cnt_frame_req_rd - 1'b1;
            end
        end
    end
end

//--------------------------------------------------------------------------------
// -- Output mapping
//--------------------------------------------------------------------------------
always @(posedge clk_rd or posedge frame_rst_rd) begin
    if(frame_rst_rd == 1'b1) begin
        bram_rd_en                  <= 'd0;
        rd_en_1dly                  <= 'd0;
        rd_en_2dly                  <= 'd0;
        o_dout_vld                  <= 'd0;
        o_dout                      <= 'd0;
        cnt_rd_len                  <= 'd0;
        chan_rd_flag                <= 'd0;
    end
    else begin
        o_dout_vld                  <= rd_en_2dly;
        rd_en_2dly                  <= rd_en_1dly;
        // Input channel number is equal to output channel number
        if(DOUBLE_IN_EN == 0) begin
            bram_rd_en                  <= i_dout_rd_en;
            rd_en_1dly                  <= bram_rd_en;
            o_dout                      <= bram_rd_dat;
        end
        // Input channel number is 2 times to output channel number
        else begin
            chan_rd_flag[3:1]           <= chan_rd_flag[2:0];
            // When input data mode is interlace mode
            if(INTERLACE_EN == 1'b1) begin
                // Flip flag when half channel read out
                if(i_dout_rd_en[CHAN_NUM_OUT-1] == 1'b1) begin
                    chan_rd_flag[0]             <= ~chan_rd_flag[0];
                end
            end
            // When input data mode is serial mode
            else begin
                // Flip flag when read out
                if(i_dout_rd_en > 'd0) begin
                    if(cnt_rd_len >= lval_len_rd_3dly - 1'b1) begin
                        cnt_rd_len                  <= 'd0;
                        chan_rd_flag[0]             <= ~chan_rd_flag[0];
                    end
                    else begin
                        cnt_rd_len                  <= cnt_rd_len + 1'b1;
                    end
                end
            end
            // Read out half BRAM base on flag state
            if(chan_rd_flag[0] == 1'b0) begin
                bram_rd_en[CHAN_NUM_OUT-1:0]    <= i_dout_rd_en;
                bram_rd_en[CHAN_NUM_IN-1:CHAN_NUM_OUT]<= 'd0;
            end
            else begin
                bram_rd_en[CHAN_NUM_OUT-1:0]    <= 'd0;
                bram_rd_en[CHAN_NUM_IN-1:CHAN_NUM_OUT]<= i_dout_rd_en;
            end
            // Valid signal mapping
            if(chan_rd_flag[1] == 1'b0) begin
                rd_en_1dly                  <= bram_rd_en[CHAN_NUM_OUT-1:0];
            end
            else begin
                rd_en_1dly                  <= bram_rd_en[CHAN_NUM_IN-1:CHAN_NUM_OUT];
            end
            // Data out mapping
            if(chan_rd_flag[3] == 1'b0) begin
                o_dout                      <= bram_rd_dat[CHAN_NUM_OUT*16-1:0];
            end
            else begin
                o_dout                      <= bram_rd_dat[CHAN_NUM_IN*16-1:CHAN_NUM_OUT*16];
            end
        end
    end
end

//--------------------------------------------------------------------------------
// -- BRAM write address controller
//--------------------------------------------------------------------------------
always @(posedge clk_wr or posedge frame_rst_wr) begin
    if(frame_rst_wr == 1'b1) begin
        cnt_bram_wr                 <= 'd0;
        bram_wr_en                  <= 1'b0;
        bram_wr_addr                <= 'd0;
    end
    else begin
        bram_wr_addr                <= cnt_bram_wr;
        if(i_din_vld == 1'b1) begin
            bram_wr_en                  <= 1'b1;
            // Pipeline, address equal 2 times lval length
            if(cnt_bram_wr < 2*lval_len_wr_3dly - 1'b1) begin
                cnt_bram_wr                 <= cnt_bram_wr + 1'b1;
            end
            else begin
                cnt_bram_wr                 <= 'd0;
            end
        end
        else begin
            bram_wr_en                  <= 1'b0;
        end
    end
end

//--------------------------------------------------------------------------------
// -- BRAM write in data configuration via channels
//--------------------------------------------------------------------------------
genvar i;
generate
    for(i = 0; i < CHAN_NUM_IN/2; i = i + 1) begin: BRAM_wr_map
        always @(posedge clk_wr or posedge frame_rst_wr) begin
            if(frame_rst_wr == 1'b1) begin
                bram_wr_dat[i*32+31:i*32]   <= 'd0;
            end
            else begin
                // Serial output channel reduce mode write in data mapping
                if((INTERLACE_EN == 1'b0) && (DOUBLE_IN_EN == 1'b1)) begin
                /* - Data in[i*2](two chan) into BRAM[i](two BRAM) - */
                    if(i < CHAN_NUM_IN/4) begin
                        if(DAT_WIDTH_IN == 16) begin
                            bram_wr_dat[i*32+31:i*32]   <= i_din[(i*4+2)*DAT_WIDTH_IN-1:(i*4)*DAT_WIDTH_IN];    // ((i*2)+1)*(DAT_WIDTH_IN*2)-1 : (i*2)*(DAT_WIDTH_IN*2)
                        end
                        else begin
                            bram_wr_dat[i*32+31:i*32+16]<= {{(16-DAT_WIDTH_IN){1'b0}}, i_din[(i*4+2)*DAT_WIDTH_IN-1:(i*4+1)*DAT_WIDTH_IN]};    // ((i*2)+1)*(DAT_WIDTH_IN*2)-1 : (i*2)*(DAT_WIDTH_IN*2)+DAT_WIDTH_IN
                            bram_wr_dat[i*32+15:i*32]   <= {{(16-DAT_WIDTH_IN){1'b0}}, i_din[(i*4+1)*DAT_WIDTH_IN-1:(i*4)*DAT_WIDTH_IN]};    // (i*2)*(DAT_WIDTH_IN*2)+DAT_WIDTH_IN-1 : (i*2)*(DAT_WIDTH_IN*2)
                        end
                    end
                /* - Data in[i*2-CHAN_NUM_IN/4+1](two chan) into BRAM[i](two BRAM) - */
                    else if(i < CHAN_NUM_IN/2) begin
                        if(DAT_WIDTH_IN == 16) begin
                            bram_wr_dat[i*32+31:i*32]   <= i_din[(i*4-CHAN_NUM_IN+4)*DAT_WIDTH_IN-1:(i*4-CHAN_NUM_IN+2)*DAT_WIDTH_IN];    // (((i-CHAN_NUM_IN/4)*2)+1)*(DAT_WIDTH_IN*2)+DAT_WIDTH_IN*2-1 : ((((i-CHAN_NUM_IN/4)*2)+1)*(DAT_WIDTH_IN*2)
                        end
                        else begin
                            bram_wr_dat[i*32+31:i*32+16]<= {{(16-DAT_WIDTH_IN){1'b0}}, i_din[(i*4-CHAN_NUM_IN+4)*DAT_WIDTH_IN-1:(i*4-CHAN_NUM_IN+3)*DAT_WIDTH_IN]};    // ((((i-CHAN_NUM_IN/4)*2)+1)+1)*(DAT_WIDTH_IN*2)-1 : ((((i-CHAN_NUM_IN/4)*2)+1)*(DAT_WIDTH_IN*2)+DAT_WIDTH_IN
                            bram_wr_dat[i*32+15:i*32]   <= {{(16-DAT_WIDTH_IN){1'b0}}, i_din[(i*4-CHAN_NUM_IN+3)*DAT_WIDTH_IN-1:(i*4-CHAN_NUM_IN+2)*DAT_WIDTH_IN]};    // ((((i-CHAN_NUM_IN/4)*2)+1)*(DAT_WIDTH_IN*2)+DAT_WIDTH_IN-1 : ((((i-CHAN_NUM_IN/4)*2)+1)*(DAT_WIDTH_IN*2)
                        end
                    end
                end
                // Other mode write in data mapping
                else begin
                    if(DAT_WIDTH_IN == 16) begin
                        bram_wr_dat[i*32+31:i*32]   <= i_din[(2*i+2)*DAT_WIDTH_IN-1:(2*i)*DAT_WIDTH_IN];
                    end
                    else begin
                        bram_wr_dat[i*32+31:i*32+16]<= {{(16-DAT_WIDTH_IN){1'b0}}, i_din[(2*i+2)*DAT_WIDTH_IN-1:(2*i+1)*DAT_WIDTH_IN]};
                        bram_wr_dat[i*32+15:i*32]   <= {{(16-DAT_WIDTH_IN){1'b0}}, i_din[(2*i+1)*DAT_WIDTH_IN-1:(2*i)*DAT_WIDTH_IN]};
                    end 
                end
            end
        end
    end
endgenerate

//--------------------------------------------------------------------------------
// -- BRAM read address controller via channels
//--------------------------------------------------------------------------------
genvar j;
generate
    for(j = 0; j < CHAN_NUM_IN; j = j + 1) begin: BRAM_rd_map
        always @(posedge clk_rd or posedge frame_rst_rd) begin
            if(frame_rst_rd == 1'b1) begin
                bram_rd_addr[j]             <= 'd0;
            end
            else begin
                if(bram_rd_en[j] == 1'b1) begin
                    // Pipeline, address equal 2 times lval length
                    if(bram_rd_addr[j] < 2*lval_len_rd_3dly - 1'b1) begin
                        bram_rd_addr[j]             <= bram_rd_addr[j] + 'd1;
                    end
                    else begin
                        bram_rd_addr[j]             <= 'd0;
                    end
                end
            end
        end
    end
endgenerate

//--------------------------------------------------------------------------------
// -- Write pipeline flag
//--------------------------------------------------------------------------------
always @(posedge clk_wr or posedge frame_rst_wr) begin    // Write in flag, pull up - write first half done, pull down - write second half done
    if(frame_rst_wr == 1'b1) begin
        wr_flag                     <= 1'b0;
    end
    else begin
        if(bram_wr_addr == 'b0) begin
            wr_flag                     <= 1'b0;
        end
        else if(bram_wr_addr == lval_len_wr_3dly) begin
            wr_flag                     <= 1'b1;
        end
    end
end

//--------------------------------------------------------------------------------
// -- Read pipeline flag
//--------------------------------------------------------------------------------
always @(posedge clk_rd or posedge frame_rst_rd) begin    // Read out flag, pull up - read first half done, pull done - write second half done
    if(frame_rst_rd == 1'b1) begin
        rd_flag                     <= 1'b0;
    end
    else begin
        if(bram_rd_addr[CHAN_NUM_IN-1] == 'b0) begin
            rd_flag                     <= 1'b0;
        end
        else if(bram_rd_addr[CHAN_NUM_IN-1] == lval_len_rd_3dly) begin
            rd_flag                     <= 1'b1;
        end
    end
end

//--------------------------------------------------------------------------------
// -- Read pipeline stage check
//--------------------------------------------------------------------------------
always @(posedge clk_rd or posedge frame_rst_rd) begin
    if(frame_rst_rd == 1'b1) begin
        bram_buf_rdy                <= 'd0;
    end
    else begin
        // BRAM first half
        if(wr_flag_dly[3:2] == 2'b01) begin    // BRAM first half ready
            bram_buf_rdy[0]             <= 1'b1;
        end
        else if(rd_flag_dly[3:2] == 2'b01) begin    // BRAM first half read out
            bram_buf_rdy[0]             <= 1'b0;
        end
        // BRAM second half
        if(wr_flag_dly[3:2] == 2'b10) begin    // BRAM second half ready
            bram_buf_rdy[1]             <= 1'b1;
        end
        else if(rd_flag_dly[3:2] == 2'b10) begin    // BRAM second half read out
            bram_buf_rdy[1]             <= 1'b0;
        end
    end
end

//--------------------------------------------------------------------------------
// -- BRAM write in request generator
//--------------------------------------------------------------------------------
always @(posedge clk_rd or posedge frame_rst_rd) begin
    if(frame_rst_rd == 1'b1) begin
        o_din_req                <= 1'b0;
        bram_full                <= 1'b0;
    end
    else begin
        // When first half write in done
        if(wr_flag_dly[3:2] == 2'b01) begin
            if(o_din_req == 1'b1) begin    // When request signal pull up already, pull it done
                o_din_req                <= 1'b0;
                bram_full                <= 1'b1;
            end
            else if(bram_buf_rdy[1] == 1'b0) begin    // When second half BRAM is empty, pull up request
                o_din_req                <= 1'b1;
            end
            else begin
                bram_full                <= 1'b1;    // Request signal buffer
            end
        end
        // When second half write in done
        else if(wr_flag_dly[3:2] == 2'b10) begin
            if(o_din_req == 1'b1) begin    // When request signal pull up already, pull it done
                o_din_req                <= 1'b0;
                bram_full                <= 1'b1;
            end
            else if(bram_buf_rdy[0] == 1'b0) begin    // When first half BRAM is empty, pull up request
                o_din_req                <= 1'b1;
            end
            else begin
                bram_full                <= 1'b1;    // Request signal buffer
            end
        end
        // When requset buffer is actived
        else if(bram_full == 1'b1) begin
            if(bram_buf_rdy != 2'b11) begin    // When two half of BRAM are not full either
                bram_full                <= 1'b0;
                o_din_req                <= 1'b1;
            end
        end
        else if(o_din_req == 1'b1) begin
            if(bram_wr_en_dly[2] == 1'b1) begin
                o_din_req                   <= 1'b0;
            end
        end
    end
end

//--------------------------------------------------------------------------------
// -- BRAM read out enable generator
//--------------------------------------------------------------------------------
always @(posedge clk_rd or posedge frame_rst_rd) begin
    if(frame_rst_rd == 1'b1) begin
        o_dout_rdy                   <= 1'b0;
    end
    else begin
        // When a write operation done, pull up o_dout_rdy
        if(wr_flag_dly[3] != wr_flag_dly[2]) begin
            o_dout_rdy                   <= 1'b1;
        end
        // When first half BRAM read out
        else if(rd_flag_dly[3:2] == 2'b01) begin
            o_dout_rdy                   <= bram_buf_rdy[1];
        end
        // When second half BRAM read out
        else if(rd_flag_dly[3:2] == 2'b10) begin
            o_dout_rdy                   <= bram_buf_rdy[0];
        end
    end
end

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

genvar k;
generate
    for(k = 0; k < CHAN_NUM_IN; k = k + 1) begin: bram_ddr_rdat_buf
        if(BRAM_DEPTH == 1024) begin
            // Simple Dual Port RAM
            // Write 16 width, 1024 depth
            // Read 16 width, 1024 depth
            // Port A always enable
            // Port B always enable
            // Primitives output register selected
            bram_w16d1024_sdpr    u_bram_dat_buffer    // Pipeline
            (
                .clka  (clk_wr                              ),
                .wea   (bram_wr_en                          ),
                .addra (bram_wr_addr[BRAM_ADDR_WIDTH-1:0]   ),
                .dina  (bram_wr_dat[k*16+15:k*16]           ),
                .clkb  (clk_rd                              ),
                .addrb (bram_rd_addr[k][BRAM_ADDR_WIDTH-1:0]),
                .doutb (bram_rd_dat[k*16+15:k*16]           )
            );
        end
        else if(BRAM_DEPTH == 2048) begin
            // Simple Dual Port RAM
            // Write 16 width, 2048 depth
            // Read 16 width, 2048 depth
            // Port A always enable
            // Port B always enable
            // Primitives output register selected
            bram_w16d2048_sdpr    u_bram_dat_buffer    // Pipeline
            (
                .clka  (clk_wr                              ),
                .wea   (bram_wr_en                          ),
                .addra (bram_wr_addr[BRAM_ADDR_WIDTH-1:0]   ),
                .dina  (bram_wr_dat[k*16+15:k*16]           ),
                .clkb  (clk_rd                              ),
                .addrb (bram_rd_addr[k][BRAM_ADDR_WIDTH-1:0]),
                .doutb (bram_rd_dat[k*16+15:k*16]           )
            );
        end
        else if(BRAM_DEPTH == 4096) begin
            // Simple Dual Port RAM
            // Write 16 width, 4096 depth
            // Read 16 width, 4096 depth
            // Port A always enable
            // Port B always enable
            // Primitives output register selected
            bram_w16d4096_sdpr    u_bram_dat_buffer    // Pipeline
            (
                .clka  (clk_wr                              ),
                .wea   (bram_wr_en                          ),
                .addra (bram_wr_addr[BRAM_ADDR_WIDTH-1:0]   ),
                .dina  (bram_wr_dat[k*16+15:k*16]           ),
                .clkb  (clk_rd                              ),
                .addrb (bram_rd_addr[k][BRAM_ADDR_WIDTH-1:0]),
                .doutb (bram_rd_dat[k*16+15:k*16]           )
            );
        end
    end
endgenerate

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk_wr or posedge rst_wr) begin
    if(rst_wr == 1'b1) begin
        cmd_frame_req_wr_dly        <= 'd0;
    end
    else begin
        cmd_frame_req_wr_dly        <= {cmd_frame_req_wr_dly[2:0], i_cmd_frame_req};
    end
end

always @(posedge clk_rd or posedge rst_rd) begin
    if(rst_rd == 1'b1) begin
        cmd_frame_req_rd_dly        <= 'd0;
    end
    else begin
        cmd_frame_req_rd_dly        <= {cmd_frame_req_rd_dly[2:0], i_cmd_frame_req};
    end
end

always @(posedge clk_wr or posedge rst_wr) begin
    if(rst_wr == 1'b1) begin
        lval_len_wr_1dly            <= 'd0;
        lval_len_wr_2dly            <= 'd0;
        lval_len_wr_3dly            <= 'd0;
    end
    else begin
        lval_len_wr_1dly            <= i_lval_len_wr;
        lval_len_wr_2dly            <= lval_len_wr_1dly;
        lval_len_wr_3dly            <= lval_len_wr_2dly;
    end
end

always @(posedge clk_rd or posedge rst_rd) begin
    if(rst_rd == 1'b1) begin
        lval_len_rd_1dly            <= 'd0;
        lval_len_rd_2dly            <= 'd0;
        lval_len_rd_3dly            <= 'd0;
    end
    else begin
        lval_len_rd_1dly            <= i_lval_len_wr;
        lval_len_rd_2dly            <= lval_len_rd_1dly;
        lval_len_rd_3dly            <= lval_len_rd_2dly;
    end
end

always @(posedge clk_rd or posedge frame_rst_rd) begin
    if(frame_rst_rd == 1'b1) begin
        wr_flag_dly                 <= 'd0;
        rd_flag_dly                 <= 'd0;
        bram_wr_en_dly              <= 'd0;
    end
    else begin
        wr_flag_dly                 <= {wr_flag_dly[2:0], wr_flag};
        rd_flag_dly                 <= {rd_flag_dly[2:0], rd_flag};
        bram_wr_en_dly              <= {bram_wr_en_dly[1:0], bram_wr_en};
    end
end


endmodule