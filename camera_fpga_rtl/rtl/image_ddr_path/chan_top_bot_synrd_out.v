`timescale 1ns / 1ps
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : chan_top_bot_synrd_out
// --File      : chan_top_bot_synrd_out.v
// --Author    : guangcai.ding@gpixel.com
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : chan_top_bot_synrd_out module.
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2025.03.06   Guangcai Ding   Creat
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module chan_top_bot_synrd_out #(
    parameter    CHAN_NUM           = 8       ,
    parameter    DAT_WIDTH          = 12      ,
    parameter    BRAM_ADDR_WIDTH    = 10
)(
    input                                          clk                ,
    input                                          rst                ,
    input                                          i_cmd_frame_req    ,
    input            [1:0]                         i_hdr_mode_en      ,
    input            [1:0]                         i_flip_v_en        ,
    input                                          i_flip_h_en        ,
    output    reg    [7:0]                         o_bram_rd_en       ,
    output           [8*BRAM_ADDR_WIDTH-1:0]       o_bram_rd_addr     ,
    input                                          i_pipe_flag_top    ,
    input                                          i_row1_done_top    ,
    input                                          i_pipe_flag_bot    ,
    input                                          i_row1_done_bot    ,
    input            [CHAN_NUM*4*DAT_WIDTH-1:0]    i_dat_out_top      ,
    input            [CHAN_NUM*4*DAT_WIDTH-1:0]    i_dat_out_bot      ,
    output    reg                                  o_dout_vld         ,
    output    reg    [CHAN_NUM*4*DAT_WIDTH-1:0]    o_dout             ,
    output           [31:0]                         o_dbg
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

localparam    BRAM_RD_LEN    = 272;
localparam    BUF_RD_LEN     = 68 ;
localparam    UN_VLD_LEN     = 2  ;

localparam    s_mult_idle    = 'd0;
localparam    s_init_1st     = 'd1;
localparam    s_init_2nd     = 'd2;
localparam    s_mult_addr    = 'd3;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

reg     [1:0]                       cs_mult                   ;
reg     [1:0]                       ns_mult                   ;
wire    [DAT_WIDTH-1:0]             buf_dat_top[CHAN_NUM*4-1:0];
wire    [DAT_WIDTH-1:0]             buf_dat_bot[CHAN_NUM*4-1:0];
reg     [3:0]                       cmd_frame_req_dly          ;
reg     [4:0]                       cnt_frame_req              ;
reg     [1:0]                       hdr_mode_en_1dly           ;
reg     [1:0]                       hdr_mode_en_2dly           ;
reg     [1:0]                       hdr_mode_en_3dly           ;
reg     [1:0]                       flip_v_en_1dly             ;
reg     [1:0]                       flip_v_en_2dly             ;
reg     [1:0]                       flip_v_en_3dly             ;
reg     [3:0]                       flip_h_en_dly              ;
reg                                 frame_rst                  ;
reg     [15:0]                      cnt_dat_len                ;    // Write in data number counter
reg     [1:0]                       pipe_flag_top_dly          ;
reg     [1:0]                       pipe_flag_bot_dly          ;
reg                                 row1_done_top_dly          ;
reg                                 row1_done_bot_dly          ;
reg                                 flag_rd                    ;
reg     [2:0]                       cnt_chan                   ;
reg     [BRAM_ADDR_WIDTH-1:0]       bram_rd_addr[CHAN_NUM-1:0] ;

reg                                 buf_wr_en                  ;
reg     [12:0]                      buf_wr_addr                ;
reg     [12:0]                      buf_wr_addr_ch[CHAN_NUM-1:0];
integer                             wr_addr_i                  ;
reg     [DAT_WIDTH-1:0]             buf_wr_dat[CHAN_NUM-1:0]   ;
reg     [12:0]                      cnt_wr_buf                 ;
reg                                 buf_wr_pipe                ;
reg     [1:0]                       buf_wr_pipe_dly            ;
reg                                 buf_row1_done              ;
reg                                 buf_row1_done_dly          ;
reg     [7:0]                       buf_rd_en                  ;
reg     [7:0]                       buf_rd_en_dly              ;
reg                                 buf_dout_vld               ;
reg     [7:0]                       buf_rd_addr[CHAN_NUM-1:0]  ;
wire    [383:0]                     buf_rd_dat[7:0]            ;
reg                                 buf_rd_flag                ;
reg     [12:0]                      cnt_rd_buf                 ;
reg                                 buf_wr_en_pre              ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

genvar i;
generate
    for(i = 0; i < CHAN_NUM*4; i = i + 1) begin    // Input data buffer into 2D array
        assign    buf_dat_top[i]    = i_dat_out_top[(i+1)*DAT_WIDTH-1:i*DAT_WIDTH];
        assign    buf_dat_bot[i]    = i_dat_out_bot[(i+1)*DAT_WIDTH-1:i*DAT_WIDTH];
    end
endgenerate

genvar k;
generate
    for(k = 0; k < CHAN_NUM ; k = k + 1) begin    // Input data buffer into 2D array
        assign    o_bram_rd_addr[(k+1)*BRAM_ADDR_WIDTH-1:k*BRAM_ADDR_WIDTH]    = bram_rd_addr[k];
    end
endgenerate

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Generate frame_rst
always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        cnt_frame_req    <= 'd0;
        frame_rst        <= 1'b1;
    end
    else begin
        if(cmd_frame_req_dly[3:2] == 2'b01)begin
            cnt_frame_req    <= 'd15;
            frame_rst        <= 1'b1;
        end
        else begin
            if(cnt_frame_req == 'd0)begin
                frame_rst    <= 1'b0;
            end
            else begin
                cnt_frame_req    <= cnt_frame_req - 1'b1;
            end
        end
    end
end

// Read data from chan_mult
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        cs_mult    <= s_mult_idle;
    end
    else begin
        cs_mult    <= ns_mult;
    end
end

always @(*) begin
    case(cs_mult)
        s_mult_idle:   begin
                            // K7_Demo gold-standard behavior: accept row-group
                            // retrigger directly from row_done/pipe_flag.
                            if((row1_done_top_dly == 1'b1) && (row1_done_bot_dly == 1'b1) && ((pipe_flag_top_dly == 2'b01) || (pipe_flag_bot_dly == 2'b01))) begin
                                ns_mult    = s_init_1st;
                            end
                            else if((row1_done_top_dly == 1'b1) && (row1_done_bot_dly == 1'b1) && ((pipe_flag_top_dly == 2'b10) || (pipe_flag_bot_dly == 2'b10))) begin
                                ns_mult    = s_init_2nd;
                            end
                            else begin
                                ns_mult    = s_mult_idle;
                            end
                        end
        s_init_1st:     begin
                            ns_mult    = s_mult_addr;
                        end 
        s_init_2nd:     begin
                            ns_mult    = s_mult_addr;
                        end
        s_mult_addr:     begin
                            if(flag_rd == 1'b0) begin
                                ns_mult    = s_mult_idle;
                            end
                            else begin
                                ns_mult    = s_mult_addr;
                            end
                        end
        default:        begin
                            ns_mult    = s_mult_idle;
                        end
    endcase
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        flag_rd            <= 1'b0;
        cnt_dat_len        <= 'd0;
        cnt_chan           <= 'd0;
        o_bram_rd_en       <= 8'b0000_0000;
        bram_rd_addr[0]    <= 'd0;
        bram_rd_addr[1]    <= 'd0;
        bram_rd_addr[2]    <= 'd0;
        bram_rd_addr[3]    <= 'd0;
        bram_rd_addr[4]    <= 'd0;
        bram_rd_addr[5]    <= 'd0;
        bram_rd_addr[6]    <= 'd0;
        bram_rd_addr[7]    <= 'd0;
    end
    else begin
        case(cs_mult)
            s_mult_idle:    begin
                                flag_rd            <= 1'b0;
                                cnt_dat_len        <= 'd0;
                                cnt_chan           <= 'd0;
                                o_bram_rd_en       <= 8'b0000_0000;
                                bram_rd_addr[0]    <= 'd0;
                                bram_rd_addr[1]    <= 'd0;
                                bram_rd_addr[2]    <= 'd0;
                                bram_rd_addr[3]    <= 'd0;
                                bram_rd_addr[4]    <= 'd0;
                                bram_rd_addr[5]    <= 'd0;
                                bram_rd_addr[6]    <= 'd0;
                                bram_rd_addr[7]    <= 'd0;
                            end
            s_init_1st:     begin
                                flag_rd         <= 1'b1;
                                cnt_dat_len     <= 'd0;

                                if(flip_h_en_dly[3] == 1'b1) begin
                                    o_bram_rd_en       <= 8'b1000_0000;
                                    bram_rd_addr[0]    <= 'd271;
                                    bram_rd_addr[1]    <= 'd271;
                                    bram_rd_addr[2]    <= 'd271;
                                    bram_rd_addr[3]    <= 'd271;
                                    bram_rd_addr[4]    <= 'd271;
                                    bram_rd_addr[5]    <= 'd271;
                                    bram_rd_addr[6]    <= 'd271;
                                    bram_rd_addr[7]    <= 'd271;
                                end
                                else begin
                                    o_bram_rd_en       <= 8'b0000_0001;
                                    bram_rd_addr[0]    <= 'd0;
                                    bram_rd_addr[1]    <= 'd0;
                                    bram_rd_addr[2]    <= 'd0;
                                    bram_rd_addr[3]    <= 'd0;
                                    bram_rd_addr[4]    <= 'd0;
                                    bram_rd_addr[5]    <= 'd0;
                                    bram_rd_addr[6]    <= 'd0;
                                    bram_rd_addr[7]    <= 'd0;
                                end
                            end
            s_init_2nd:     begin
                                flag_rd         <= 1'b1;
                                cnt_dat_len     <= 'd0;
                                if(flip_h_en_dly[3] == 1'b1) begin
                                    o_bram_rd_en       <= 8'b1000_0000;
                                    bram_rd_addr[0]    <= 'd783;
                                    bram_rd_addr[1]    <= 'd783;
                                    bram_rd_addr[2]    <= 'd783;
                                    bram_rd_addr[3]    <= 'd783;
                                    bram_rd_addr[4]    <= 'd783;
                                    bram_rd_addr[5]    <= 'd783;
                                    bram_rd_addr[6]    <= 'd783;
                                    bram_rd_addr[7]    <= 'd783;
                                end
                                else begin
                                    o_bram_rd_en       <= 8'b0000_0001;
                                    bram_rd_addr[0]    <= 'd512;
                                    bram_rd_addr[1]    <= 'd512;
                                    bram_rd_addr[2]    <= 'd512;
                                    bram_rd_addr[3]    <= 'd512;
                                    bram_rd_addr[4]    <= 'd512;
                                    bram_rd_addr[5]    <= 'd512;
                                    bram_rd_addr[6]    <= 'd512;
                                    bram_rd_addr[7]    <= 'd512;
                                end
                            end
            s_mult_addr:    begin
                                if(cnt_dat_len == BRAM_RD_LEN - 'd1) begin
                                    if(cnt_chan == 'd7) begin
                                        flag_rd         <= 1'b0;
                                        cnt_dat_len     <= 'd0;
                                        o_bram_rd_en    <= 8'b0000_0000;
                                        cnt_chan        <= 'd0;
                                    end
                                    else begin
                                        cnt_dat_len    <= 'd0;
                                        cnt_chan       <= cnt_chan + 1'b1;
                                    end
                                    
                                    if(flip_h_en_dly[3] == 1'b1) begin
                                        case(cnt_chan)
                                            'd0:    o_bram_rd_en    <= 8'b0100_0000;
                                            'd1:    o_bram_rd_en    <= 8'b0010_0000;
                                            'd2:    o_bram_rd_en    <= 8'b0001_0000;
                                            'd3:    o_bram_rd_en    <= 8'b0000_1000;
                                            'd4:    o_bram_rd_en    <= 8'b0000_0100;
                                            'd5:    o_bram_rd_en    <= 8'b0000_0010;
                                            'd6:    o_bram_rd_en    <= 8'b0000_0001;
                                            'd7:    o_bram_rd_en    <= 8'b0000_0000;
                                        endcase
                                    end
                                    else begin
                                        case(cnt_chan)
                                            'd0:    o_bram_rd_en    <= 8'b0000_0010;
                                            'd1:    o_bram_rd_en    <= 8'b0000_0100;
                                            'd2:    o_bram_rd_en    <= 8'b0000_1000;
                                            'd3:    o_bram_rd_en    <= 8'b0001_0000;
                                            'd4:    o_bram_rd_en    <= 8'b0010_0000;
                                            'd5:    o_bram_rd_en    <= 8'b0100_0000;
                                            'd6:    o_bram_rd_en    <= 8'b1000_0000;
                                            'd7:    o_bram_rd_en    <= 8'b0000_0000;
                                        endcase
                                    end
                                end
                                else begin
                                    cnt_dat_len             <= cnt_dat_len + 1'b1;
                                end

                                if(flip_h_en_dly[3] == 1'b1) begin
                                    case(cnt_chan)
                                        'd0:    bram_rd_addr[7]    <= bram_rd_addr[7] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd1:    bram_rd_addr[6]    <= bram_rd_addr[6] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd2:    bram_rd_addr[5]    <= bram_rd_addr[5] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd3:    bram_rd_addr[4]    <= bram_rd_addr[4] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd4:    bram_rd_addr[3]    <= bram_rd_addr[3] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd5:    bram_rd_addr[2]    <= bram_rd_addr[2] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd6:    bram_rd_addr[1]    <= bram_rd_addr[1] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd7:    bram_rd_addr[0]    <= bram_rd_addr[0] + 1'b1 - 2 * flip_h_en_dly[3];
                                    endcase
                                end
                                else begin
                                    case(cnt_chan)
                                        'd0:    bram_rd_addr[0]    <= bram_rd_addr[0] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd1:    bram_rd_addr[1]    <= bram_rd_addr[1] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd2:    bram_rd_addr[2]    <= bram_rd_addr[2] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd3:    bram_rd_addr[3]    <= bram_rd_addr[3] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd4:    bram_rd_addr[4]    <= bram_rd_addr[4] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd5:    bram_rd_addr[5]    <= bram_rd_addr[5] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd6:    bram_rd_addr[6]    <= bram_rd_addr[6] + 1'b1 - 2 * flip_h_en_dly[3];
                                        'd7:    bram_rd_addr[7]    <= bram_rd_addr[7] + 1'b1 - 2 * flip_h_en_dly[3];
                                    endcase
                                end
                            end
        endcase
    end
end

// Buffer write enable
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_wr_en_pre    <= 1'b0;
    end
    else begin
        if(o_bram_rd_en != 8'b0000_0000) begin
            buf_wr_en_pre    <= 1'b1;
        end
        else begin
            buf_wr_en_pre    <= 1'b0;
        end
    end
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_wr_en    <= 1'b0;
    end
    else begin
        buf_wr_en    <= buf_wr_en_pre;
    end
end

// Buffer write address
// Replicate the write address per BRAM lane.  The original single
// buf_wr_addr net drove every generated BRAM addra port and became a
// high-fanout 220 MHz route-critical path after implementation.
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_wr_addr    <= 'd0;
        cnt_wr_buf     <= 'd0;
        for(wr_addr_i = 0; wr_addr_i < CHAN_NUM; wr_addr_i = wr_addr_i + 1) begin
            buf_wr_addr_ch[wr_addr_i] <= 'd0;
        end
    end
    else begin
        if((row1_done_top_dly == 1'b1) && (row1_done_bot_dly == 1'b1) && ((pipe_flag_top_dly == 2'b01)||(pipe_flag_bot_dly == 2'b01))) begin
            buf_wr_addr    <= 'd0;
            cnt_wr_buf     <= 'd0;
            for(wr_addr_i = 0; wr_addr_i < CHAN_NUM; wr_addr_i = wr_addr_i + 1) begin
                buf_wr_addr_ch[wr_addr_i] <= 'd0;
            end
        end
        else if((row1_done_top_dly == 1'b1) && (row1_done_bot_dly == 1'b1) && ((pipe_flag_top_dly == 2'b10)||(pipe_flag_bot_dly == 2'b10))) begin
            buf_wr_addr    <= 'd4096;
            cnt_wr_buf     <= 'd0;
            for(wr_addr_i = 0; wr_addr_i < CHAN_NUM; wr_addr_i = wr_addr_i + 1) begin
                buf_wr_addr_ch[wr_addr_i] <= 'd4096;
            end
        end
        else begin
            if(buf_wr_en == 1'b1) begin
                buf_wr_addr    <= buf_wr_addr + 1'b1;
                cnt_wr_buf     <= cnt_wr_buf + 1'b1;
                for(wr_addr_i = 0; wr_addr_i < CHAN_NUM; wr_addr_i = wr_addr_i + 1) begin
                    buf_wr_addr_ch[wr_addr_i] <= buf_wr_addr + 1'b1;
                end
            end
            else begin
                buf_wr_addr    <= buf_wr_addr;
                cnt_wr_buf     <= cnt_wr_buf;
                for(wr_addr_i = 0; wr_addr_i < CHAN_NUM; wr_addr_i = wr_addr_i + 1) begin
                    buf_wr_addr_ch[wr_addr_i] <= buf_wr_addr_ch[wr_addr_i];
                end
            end
        end
    end
end

// buffer write data
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_wr_dat[2]     <= 'd0;    // top
        buf_wr_dat[6]     <= 'd0;    // top
        buf_wr_dat[3]     <= 'd0;    // top
        buf_wr_dat[7]     <= 'd0;    // top
        buf_wr_dat[0]     <= 'd0;    // bot
        buf_wr_dat[4]     <= 'd0;    // bot
        buf_wr_dat[1]     <= 'd0;    // bot
        buf_wr_dat[5]     <= 'd0;    // bot
    end
    else begin
        if(flip_v_en_3dly == 2'd1)begin    // flip_v enable
            if(hdr_mode_en_3dly == 2'd0) begin    // STD mode
                if((cnt_wr_buf >= 'd0) && (cnt_wr_buf < BRAM_RD_LEN - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1)begin
                        buf_wr_dat[2]     <= buf_dat_bot[30];
                        buf_wr_dat[6]     <= buf_dat_bot[31];
                        buf_wr_dat[3]     <= buf_dat_bot[28];
                        buf_wr_dat[7]     <= buf_dat_bot[29];
                        buf_wr_dat[0]     <= buf_dat_top[30];
                        buf_wr_dat[4]     <= buf_dat_top[31];
                        buf_wr_dat[1]     <= buf_dat_top[28];
                        buf_wr_dat[5]     <= buf_dat_top[29];
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[2];
                        buf_wr_dat[6]     <= buf_dat_bot[3];
                        buf_wr_dat[3]     <= buf_dat_bot[0];
                        buf_wr_dat[7]     <= buf_dat_bot[1];
                        buf_wr_dat[0]     <= buf_dat_top[2];
                        buf_wr_dat[4]     <= buf_dat_top[3];
                        buf_wr_dat[1]     <= buf_dat_top[0];
                        buf_wr_dat[5]     <= buf_dat_top[1];
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 2 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[26];
                        buf_wr_dat[6]     <= buf_dat_bot[27];
                        buf_wr_dat[3]     <= buf_dat_bot[24];
                        buf_wr_dat[7]     <= buf_dat_bot[25];
                        buf_wr_dat[0]     <= buf_dat_top[26];
                        buf_wr_dat[4]     <= buf_dat_top[27];
                        buf_wr_dat[1]     <= buf_dat_top[24];
                        buf_wr_dat[5]     <= buf_dat_top[25];
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[6];
                        buf_wr_dat[6]     <= buf_dat_bot[7];
                        buf_wr_dat[3]     <= buf_dat_bot[4];
                        buf_wr_dat[7]     <= buf_dat_bot[5];
                        buf_wr_dat[0]     <= buf_dat_top[6];
                        buf_wr_dat[4]     <= buf_dat_top[7];
                        buf_wr_dat[1]     <= buf_dat_top[4];
                        buf_wr_dat[5]     <= buf_dat_top[5];
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 2 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 3 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[22];
                        buf_wr_dat[6]     <= buf_dat_bot[23];
                        buf_wr_dat[3]     <= buf_dat_bot[20];
                        buf_wr_dat[7]     <= buf_dat_bot[21];
                        buf_wr_dat[0]     <= buf_dat_top[22];
                        buf_wr_dat[4]     <= buf_dat_top[23];
                        buf_wr_dat[1]     <= buf_dat_top[20];
                        buf_wr_dat[5]     <= buf_dat_top[21];
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[10];
                        buf_wr_dat[6]     <= buf_dat_bot[11];
                        buf_wr_dat[3]     <= buf_dat_bot[8];
                        buf_wr_dat[7]     <= buf_dat_bot[9];
                        buf_wr_dat[0]     <= buf_dat_top[10];
                        buf_wr_dat[4]     <= buf_dat_top[11];
                        buf_wr_dat[1]     <= buf_dat_top[8];
                        buf_wr_dat[5]     <= buf_dat_top[9];
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 3 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 4 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[18];
                        buf_wr_dat[6]     <= buf_dat_bot[19];
                        buf_wr_dat[3]     <= buf_dat_bot[16];
                        buf_wr_dat[7]     <= buf_dat_bot[17];
                        buf_wr_dat[0]     <= buf_dat_top[18];
                        buf_wr_dat[4]     <= buf_dat_top[19];
                        buf_wr_dat[1]     <= buf_dat_top[16];
                        buf_wr_dat[5]     <= buf_dat_top[17];
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[14];
                        buf_wr_dat[6]     <= buf_dat_bot[15];
                        buf_wr_dat[3]     <= buf_dat_bot[12];
                        buf_wr_dat[7]     <= buf_dat_bot[13];
                        buf_wr_dat[0]     <= buf_dat_top[14];
                        buf_wr_dat[4]     <= buf_dat_top[15];
                        buf_wr_dat[1]     <= buf_dat_top[12];
                        buf_wr_dat[5]     <= buf_dat_top[13];
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 4 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 5 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[14];
                        buf_wr_dat[6]     <= buf_dat_bot[15];
                        buf_wr_dat[3]     <= buf_dat_bot[12];
                        buf_wr_dat[7]     <= buf_dat_bot[13];
                        buf_wr_dat[0]     <= buf_dat_top[14];
                        buf_wr_dat[4]     <= buf_dat_top[15];
                        buf_wr_dat[1]     <= buf_dat_top[12];
                        buf_wr_dat[5]     <= buf_dat_top[13];
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[18];
                        buf_wr_dat[6]     <= buf_dat_bot[19];
                        buf_wr_dat[3]     <= buf_dat_bot[16];
                        buf_wr_dat[7]     <= buf_dat_bot[17];
                        buf_wr_dat[0]     <= buf_dat_top[18];
                        buf_wr_dat[4]     <= buf_dat_top[19];
                        buf_wr_dat[1]     <= buf_dat_top[16];
                        buf_wr_dat[5]     <= buf_dat_top[17];
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 5 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 6 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[10];
                        buf_wr_dat[6]     <= buf_dat_bot[11]; 
                        buf_wr_dat[3]     <= buf_dat_bot[8];
                        buf_wr_dat[7]     <= buf_dat_bot[9]; 
                        buf_wr_dat[0]     <= buf_dat_top[10];
                        buf_wr_dat[4]     <= buf_dat_top[11];
                        buf_wr_dat[1]     <= buf_dat_top[8];
                        buf_wr_dat[5]     <= buf_dat_top[9]; 
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[22];
                        buf_wr_dat[6]     <= buf_dat_bot[23];
                        buf_wr_dat[3]     <= buf_dat_bot[20];
                        buf_wr_dat[7]     <= buf_dat_bot[21];
                        buf_wr_dat[0]     <= buf_dat_top[22];
                        buf_wr_dat[4]     <= buf_dat_top[23];
                        buf_wr_dat[1]     <= buf_dat_top[20];
                        buf_wr_dat[5]     <= buf_dat_top[21];
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 6 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 7 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[6];
                        buf_wr_dat[6]     <= buf_dat_bot[7];
                        buf_wr_dat[3]     <= buf_dat_bot[4];
                        buf_wr_dat[7]     <= buf_dat_bot[5];
                        buf_wr_dat[0]     <= buf_dat_top[6];
                        buf_wr_dat[4]     <= buf_dat_top[7];
                        buf_wr_dat[1]     <= buf_dat_top[4];
                        buf_wr_dat[5]     <= buf_dat_top[5];
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[26];
                        buf_wr_dat[6]     <= buf_dat_bot[27];
                        buf_wr_dat[3]     <= buf_dat_bot[24];
                        buf_wr_dat[7]     <= buf_dat_bot[25];
                        buf_wr_dat[0]     <= buf_dat_top[26];
                        buf_wr_dat[4]     <= buf_dat_top[27];
                        buf_wr_dat[1]     <= buf_dat_top[24];
                        buf_wr_dat[5]     <= buf_dat_top[25];
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 7 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 8 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[2];
                        buf_wr_dat[6]     <= buf_dat_bot[3];
                        buf_wr_dat[3]     <= buf_dat_bot[0];
                        buf_wr_dat[7]     <= buf_dat_bot[1];
                        buf_wr_dat[0]     <= buf_dat_top[2];
                        buf_wr_dat[4]     <= buf_dat_top[3];
                        buf_wr_dat[1]     <= buf_dat_top[0];
                        buf_wr_dat[5]     <= buf_dat_top[1];
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[30];
                        buf_wr_dat[6]     <= buf_dat_bot[31];
                        buf_wr_dat[3]     <= buf_dat_bot[28];
                        buf_wr_dat[7]     <= buf_dat_bot[29];
                        buf_wr_dat[0]     <= buf_dat_top[30];
                        buf_wr_dat[4]     <= buf_dat_top[31];
                        buf_wr_dat[1]     <= buf_dat_top[28];
                        buf_wr_dat[5]     <= buf_dat_top[29];
                    end
                end
                else begin
                    buf_wr_dat[2]     <= 'd0;
                    buf_wr_dat[6]     <= 'd0;
                    buf_wr_dat[3]     <= 'd0;
                    buf_wr_dat[7]     <= 'd0;
                    buf_wr_dat[0]     <= 'd0;
                    buf_wr_dat[4]     <= 'd0;
                    buf_wr_dat[1]     <= 'd0;
                    buf_wr_dat[5]     <= 'd0;
                end
            end
            else begin    // HDR mode
                if((cnt_wr_buf >= 'd0) && (cnt_wr_buf < BRAM_RD_LEN - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1)begin
                        buf_wr_dat[2]     <= buf_dat_bot[30];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[31];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[28];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[29];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[30];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[31];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[28];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[29];    // 2L
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[2];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[3];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[0];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[1];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[2];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[3];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[0];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[1];    // 2L
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 2 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[26];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[27];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[24];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[25];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[26];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[27];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[24];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[25];    // 2L
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[6];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[7];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[4];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[5];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[6];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[7];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[4];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[5];    // 2L
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 2 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 3 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[22];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[23];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[20];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[21];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[22];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[23];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[20];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[21];    // 2L
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[10];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[11];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[8];     // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[9];     // 0L
                        buf_wr_dat[0]     <= buf_dat_top[10];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[11];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[8];     // 2H
                        buf_wr_dat[5]     <= buf_dat_top[9];     // 2L
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 3 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 4 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[18];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[19];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[16];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[17];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[18];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[19];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[16];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[17];    // 2L
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[14];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[15];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[12];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[13];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[14];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[15];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[12];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[13];    // 2L
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 4 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 5 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[14];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[15];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[12];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[13];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[14];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[15];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[12];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[13];    // 2L
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[18];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[19];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[16];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[17];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[18];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[19];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[16];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[17];    // 2L
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 5 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 6 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[10];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[11];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[8];     // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[9];     // 0L
                        buf_wr_dat[0]     <= buf_dat_top[10];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[11];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[8];     // 2H
                        buf_wr_dat[5]     <= buf_dat_top[9];     // 2L
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[22];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[23];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[20];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[21];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[22];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[23];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[20];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[21];    // 2L
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 6 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 7 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[6];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[7];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[4];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[5];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[6];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[7];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[4];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[5];    // 2L
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[26];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[27];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[24];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[25];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[26];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[27];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[24];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[25];    // 2L
                    end
                end
                else if((cnt_wr_buf >= BRAM_RD_LEN * 7 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 8 - 1'b1)) begin
                    if(flip_h_en_dly[3] == 1'b1) begin
                        buf_wr_dat[2]     <= buf_dat_bot[2];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[3];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[0];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[1];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[2];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[3];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[0];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[1];    // 2L
                    end
                    else begin
                        buf_wr_dat[2]     <= buf_dat_bot[30];    // 1H
                        buf_wr_dat[6]     <= buf_dat_bot[31];    // 1L
                        buf_wr_dat[3]     <= buf_dat_bot[28];    // 0H
                        buf_wr_dat[7]     <= buf_dat_bot[29];    // 0L
                        buf_wr_dat[0]     <= buf_dat_top[30];    // 3H
                        buf_wr_dat[4]     <= buf_dat_top[31];    // 3L
                        buf_wr_dat[1]     <= buf_dat_top[28];    // 2H
                        buf_wr_dat[5]     <= buf_dat_top[29];    // 2L
                    end
                end
                else begin
                    buf_wr_dat[2]     <= 'd0;
                    buf_wr_dat[6]     <= 'd0;
                    buf_wr_dat[3]     <= 'd0;
                    buf_wr_dat[7]     <= 'd0;
                    buf_wr_dat[0]     <= 'd0;
                    buf_wr_dat[4]     <= 'd0;
                    buf_wr_dat[1]     <= 'd0;
                    buf_wr_dat[5]     <= 'd0;
                end
            end
        end
        else begin    // flip_v disable
            if((cnt_wr_buf >= 'd0) && (cnt_wr_buf < BRAM_RD_LEN - 1'b1)) begin
                if(flip_h_en_dly[3] == 1'b1)begin    // flip_h enable
                    buf_wr_dat[2]     <= buf_dat_top[28];
                    buf_wr_dat[6]     <= buf_dat_top[29];
                    buf_wr_dat[3]     <= buf_dat_top[30];
                    buf_wr_dat[7]     <= buf_dat_top[31];
                    buf_wr_dat[0]     <= buf_dat_bot[28];
                    buf_wr_dat[4]     <= buf_dat_bot[29];
                    buf_wr_dat[1]     <= buf_dat_bot[30];
                    buf_wr_dat[5]     <= buf_dat_bot[31];
                end
                else begin    // flip_h disable
                    buf_wr_dat[2]     <= buf_dat_top[0];
                    buf_wr_dat[6]     <= buf_dat_top[1];
                    buf_wr_dat[3]     <= buf_dat_top[2];
                    buf_wr_dat[7]     <= buf_dat_top[3];
                    buf_wr_dat[0]     <= buf_dat_bot[0];
                    buf_wr_dat[4]     <= buf_dat_bot[1];
                    buf_wr_dat[1]     <= buf_dat_bot[2];
                    buf_wr_dat[5]     <= buf_dat_bot[3];
                end
            end
            else if((cnt_wr_buf >= BRAM_RD_LEN - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 2 - 1'b1)) begin
                if(flip_h_en_dly[3] == 1'b1) begin
                    buf_wr_dat[2]     <= buf_dat_top[24];
                    buf_wr_dat[6]     <= buf_dat_top[25];
                    buf_wr_dat[3]     <= buf_dat_top[26];
                    buf_wr_dat[7]     <= buf_dat_top[27];
                    buf_wr_dat[0]     <= buf_dat_bot[24];
                    buf_wr_dat[4]     <= buf_dat_bot[25];
                    buf_wr_dat[1]     <= buf_dat_bot[26];
                    buf_wr_dat[5]     <= buf_dat_bot[27];
                end
                else begin
                    buf_wr_dat[2]     <= buf_dat_top[4];
                    buf_wr_dat[6]     <= buf_dat_top[5];
                    buf_wr_dat[3]     <= buf_dat_top[6];
                    buf_wr_dat[7]     <= buf_dat_top[7];
                    buf_wr_dat[0]     <= buf_dat_bot[4];
                    buf_wr_dat[4]     <= buf_dat_bot[5];
                    buf_wr_dat[1]     <= buf_dat_bot[6];
                    buf_wr_dat[5]     <= buf_dat_bot[7];
                end
            end
            else if((cnt_wr_buf >= BRAM_RD_LEN * 2 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 3 - 1'b1)) begin
                if(flip_h_en_dly[3] == 1'b1) begin
                    buf_wr_dat[2]     <= buf_dat_top[20];
                    buf_wr_dat[6]     <= buf_dat_top[21];
                    buf_wr_dat[3]     <= buf_dat_top[22];
                    buf_wr_dat[7]     <= buf_dat_top[23];
                    buf_wr_dat[0]     <= buf_dat_bot[20];
                    buf_wr_dat[4]     <= buf_dat_bot[21];
                    buf_wr_dat[1]     <= buf_dat_bot[22];
                    buf_wr_dat[5]     <= buf_dat_bot[23];
                end
                else begin
                    buf_wr_dat[2]     <= buf_dat_top[8];
                    buf_wr_dat[6]     <= buf_dat_top[9];
                    buf_wr_dat[3]     <= buf_dat_top[10];
                    buf_wr_dat[7]     <= buf_dat_top[11];
                    buf_wr_dat[0]     <= buf_dat_bot[8];
                    buf_wr_dat[4]     <= buf_dat_bot[9];
                    buf_wr_dat[1]     <= buf_dat_bot[10];
                    buf_wr_dat[5]     <= buf_dat_bot[11];
                end
            end
            else if((cnt_wr_buf >= BRAM_RD_LEN * 3 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 4 - 1'b1)) begin
                if(flip_h_en_dly[3] == 1'b1) begin
                    buf_wr_dat[2]     <= buf_dat_top[16];
                    buf_wr_dat[6]     <= buf_dat_top[17];
                    buf_wr_dat[3]     <= buf_dat_top[18];
                    buf_wr_dat[7]     <= buf_dat_top[19];
                    buf_wr_dat[0]     <= buf_dat_bot[16];
                    buf_wr_dat[4]     <= buf_dat_bot[17];
                    buf_wr_dat[1]     <= buf_dat_bot[18];
                    buf_wr_dat[5]     <= buf_dat_bot[19];
                end
                else begin
                    buf_wr_dat[2]     <= buf_dat_top[12];
                    buf_wr_dat[6]     <= buf_dat_top[13];
                    buf_wr_dat[3]     <= buf_dat_top[14];
                    buf_wr_dat[7]     <= buf_dat_top[15];
                    buf_wr_dat[0]     <= buf_dat_bot[12];
                    buf_wr_dat[4]     <= buf_dat_bot[13];
                    buf_wr_dat[1]     <= buf_dat_bot[14];
                    buf_wr_dat[5]     <= buf_dat_bot[15];
                end
            end
            else if((cnt_wr_buf >= BRAM_RD_LEN * 4 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 5 - 1'b1)) begin
                if(flip_h_en_dly[3] == 1'b1) begin
                    buf_wr_dat[2]     <= buf_dat_top[12];
                    buf_wr_dat[6]     <= buf_dat_top[13];
                    buf_wr_dat[3]     <= buf_dat_top[14];
                    buf_wr_dat[7]     <= buf_dat_top[15];
                    buf_wr_dat[0]     <= buf_dat_bot[12];
                    buf_wr_dat[4]     <= buf_dat_bot[13];
                    buf_wr_dat[1]     <= buf_dat_bot[14];
                    buf_wr_dat[5]     <= buf_dat_bot[15];
                end
                else begin
                    buf_wr_dat[2]     <= buf_dat_top[16];
                    buf_wr_dat[6]     <= buf_dat_top[17];
                    buf_wr_dat[3]     <= buf_dat_top[18];
                    buf_wr_dat[7]     <= buf_dat_top[19];
                    buf_wr_dat[0]     <= buf_dat_bot[16];
                    buf_wr_dat[4]     <= buf_dat_bot[17];
                    buf_wr_dat[1]     <= buf_dat_bot[18];
                    buf_wr_dat[5]     <= buf_dat_bot[19];
                end
            end
            else if((cnt_wr_buf >= BRAM_RD_LEN * 5 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 6 - 1'b1)) begin
                if(flip_h_en_dly[3] == 1'b1) begin
                    buf_wr_dat[2]     <= buf_dat_top[8];
                    buf_wr_dat[6]     <= buf_dat_top[9];
                    buf_wr_dat[3]     <= buf_dat_top[10];
                    buf_wr_dat[7]     <= buf_dat_top[11];
                    buf_wr_dat[0]     <= buf_dat_bot[8];
                    buf_wr_dat[4]     <= buf_dat_bot[9];
                    buf_wr_dat[1]     <= buf_dat_bot[10];
                    buf_wr_dat[5]     <= buf_dat_bot[11];
                end
                else begin
                    buf_wr_dat[2]     <= buf_dat_top[20];
                    buf_wr_dat[6]     <= buf_dat_top[21];
                    buf_wr_dat[3]     <= buf_dat_top[22];
                    buf_wr_dat[7]     <= buf_dat_top[23];
                    buf_wr_dat[0]     <= buf_dat_bot[20];
                    buf_wr_dat[4]     <= buf_dat_bot[21];
                    buf_wr_dat[1]     <= buf_dat_bot[22];
                    buf_wr_dat[5]     <= buf_dat_bot[23];
                end
            end
            else if((cnt_wr_buf >= BRAM_RD_LEN * 6 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 7 - 1'b1)) begin
                if(flip_h_en_dly[3] == 1'b1) begin
                    buf_wr_dat[2]     <= buf_dat_top[4];
                    buf_wr_dat[6]     <= buf_dat_top[5];
                    buf_wr_dat[3]     <= buf_dat_top[6];
                    buf_wr_dat[7]     <= buf_dat_top[7];
                    buf_wr_dat[0]     <= buf_dat_bot[4];
                    buf_wr_dat[4]     <= buf_dat_bot[5];
                    buf_wr_dat[1]     <= buf_dat_bot[6];
                    buf_wr_dat[5]     <= buf_dat_bot[7];
                end
                else begin
                    buf_wr_dat[2]     <= buf_dat_top[24];
                    buf_wr_dat[6]     <= buf_dat_top[25];
                    buf_wr_dat[3]     <= buf_dat_top[26];
                    buf_wr_dat[7]     <= buf_dat_top[27];
                    buf_wr_dat[0]     <= buf_dat_bot[24];
                    buf_wr_dat[4]     <= buf_dat_bot[25];
                    buf_wr_dat[1]     <= buf_dat_bot[26];
                    buf_wr_dat[5]     <= buf_dat_bot[27];
                end
            end
            else if((cnt_wr_buf >= BRAM_RD_LEN * 7 - 1'b1) && (cnt_wr_buf < BRAM_RD_LEN * 8 - 1'b1)) begin
                if(flip_h_en_dly[3] == 1'b1) begin
                    buf_wr_dat[2]     <= buf_dat_top[0];
                    buf_wr_dat[6]     <= buf_dat_top[1];
                    buf_wr_dat[3]     <= buf_dat_top[2];
                    buf_wr_dat[7]     <= buf_dat_top[3];
                    buf_wr_dat[0]     <= buf_dat_bot[0];
                    buf_wr_dat[4]     <= buf_dat_bot[1];
                    buf_wr_dat[1]     <= buf_dat_bot[2];
                    buf_wr_dat[5]     <= buf_dat_bot[3];
                end
                else begin
                    buf_wr_dat[2]     <= buf_dat_top[28];
                    buf_wr_dat[6]     <= buf_dat_top[29];
                    buf_wr_dat[3]     <= buf_dat_top[30];
                    buf_wr_dat[7]     <= buf_dat_top[31];
                    buf_wr_dat[0]     <= buf_dat_bot[28];
                    buf_wr_dat[4]     <= buf_dat_bot[29];
                    buf_wr_dat[1]     <= buf_dat_bot[30];
                    buf_wr_dat[5]     <= buf_dat_bot[31];
                end
            end
            else begin
                buf_wr_dat[2]     <= 'd0;
                buf_wr_dat[6]     <= 'd0;
                buf_wr_dat[3]     <= 'd0;
                buf_wr_dat[7]     <= 'd0;
                buf_wr_dat[0]     <= 'd0;
                buf_wr_dat[4]     <= 'd0;
                buf_wr_dat[1]     <= 'd0;
                buf_wr_dat[5]     <= 'd0;
            end
        end
    end
end

// Buffer write pipe & one row done
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_wr_pipe     <= 1'b0;
        buf_row1_done   <= 1'b0;
    end
    else begin
        if(buf_wr_en == 1'b1) begin
            if(cnt_wr_buf < 'd2175) begin
                buf_row1_done   <= 1'b0;  
            end
            else begin
                buf_row1_done   <= 1'b1;
                buf_wr_pipe     <= ~buf_wr_pipe;
            end
        end

    end
end

// Read BRAM operation
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_rd_flag       <= 1'b0;
        cnt_rd_buf        <= 'd0;
        buf_rd_en         <= 8'b0000_0000;
        buf_rd_addr[0]    <= 'd0;
        buf_rd_addr[1]    <= 'd0;
        buf_rd_addr[2]    <= 'd0;
        buf_rd_addr[3]    <= 'd0;
        buf_rd_addr[4]    <= 'd0;
        buf_rd_addr[5]    <= 'd0;
        buf_rd_addr[6]    <= 'd0;
        buf_rd_addr[7]    <= 'd0;
    end
    else begin
        if((buf_row1_done_dly == 1'b1) && (buf_wr_pipe_dly == 2'b01)) begin
            buf_rd_flag       <= 1'b1;
            cnt_rd_buf        <= 'd0;
            buf_rd_en         <= 8'b0000_0001;
            buf_rd_addr[0]    <= 'd0;
            buf_rd_addr[1]    <= 'd0;
            buf_rd_addr[2]    <= 'd0;
            buf_rd_addr[3]    <= 'd0;
            buf_rd_addr[4]    <= 'd0;
            buf_rd_addr[5]    <= 'd0;
            buf_rd_addr[6]    <= 'd0;
            buf_rd_addr[7]    <= 'd0;
        end
        else if((buf_row1_done_dly == 1'b1) && (buf_wr_pipe_dly == 2'b10)) begin
            buf_rd_flag       <= 1'b1;
            cnt_rd_buf        <= 'd0;
            buf_rd_en         <= 8'b0000_0001;
            buf_rd_addr[0]    <= 'd128;
            buf_rd_addr[1]    <= 'd128;
            buf_rd_addr[2]    <= 'd128;
            buf_rd_addr[3]    <= 'd128;
            buf_rd_addr[4]    <= 'd128;
            buf_rd_addr[5]    <= 'd128;
            buf_rd_addr[6]    <= 'd128;
            buf_rd_addr[7]    <= 'd128;
        end
        else if(buf_rd_flag == 1'b1)begin
            if((cnt_rd_buf >= 'd0) && (cnt_rd_buf < BUF_RD_LEN - 1'b1)) begin
                buf_rd_en         <= 8'b0000_0001;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if((cnt_rd_buf >= BUF_RD_LEN - 1'b1) && (cnt_rd_buf < BUF_RD_LEN * 2 - 1'b1)) begin
                buf_rd_en         <= 8'b0000_0010;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if((cnt_rd_buf >= BUF_RD_LEN * 2 - 1'b1) && (cnt_rd_buf < BUF_RD_LEN * 3 - 1'b1)) begin
                buf_rd_en         <= 8'b0000_0100;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if((cnt_rd_buf >= BUF_RD_LEN * 3 - 1'b1) && (cnt_rd_buf < BUF_RD_LEN * 4 - 1'b1)) begin
                buf_rd_en         <= 8'b0000_1000;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if((cnt_rd_buf >= BUF_RD_LEN * 4 - 1'b1) && (cnt_rd_buf < BUF_RD_LEN * 4 + UN_VLD_LEN - 1'b1)) begin
                buf_rd_en         <= 8'b0000_0000;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if((cnt_rd_buf >= BUF_RD_LEN * 4 + UN_VLD_LEN - 1'b1) && (cnt_rd_buf < BUF_RD_LEN * 5 + UN_VLD_LEN - 1'b1)) begin
                buf_rd_en         <= 8'b0001_0000;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if((cnt_rd_buf >= BUF_RD_LEN * 5 + UN_VLD_LEN - 1'b1) && (cnt_rd_buf < BUF_RD_LEN * 6 + UN_VLD_LEN - 1'b1)) begin
                buf_rd_en         <= 8'b0010_0000;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if((cnt_rd_buf >= BUF_RD_LEN * 6 + UN_VLD_LEN - 1'b1) && (cnt_rd_buf < BUF_RD_LEN * 7 + UN_VLD_LEN - 1'b1)) begin
                buf_rd_en         <= 8'b0100_0000;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if((cnt_rd_buf >= BUF_RD_LEN * 7 + UN_VLD_LEN - 1'b1) && (cnt_rd_buf < BUF_RD_LEN * 8 + UN_VLD_LEN - 1'b1)) begin
                buf_rd_en         <= 8'b1000_0000;
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end
            else if(cnt_rd_buf == BUF_RD_LEN * 8 + UN_VLD_LEN - 1'b1) begin
                buf_rd_flag       <= 1'b0;
                cnt_rd_buf        <= 'd0;
                buf_rd_en         <= 8'b0000_0000;
            end
            else begin
                cnt_rd_buf        <= cnt_rd_buf + 1'b1;
            end

            case(buf_rd_en)
                8'b0000_0001:   buf_rd_addr[0]    <= buf_rd_addr[0] + 1'b1;
                8'b0000_0010:   buf_rd_addr[1]    <= buf_rd_addr[1] + 1'b1;
                8'b0000_0100:   buf_rd_addr[2]    <= buf_rd_addr[2] + 1'b1;
                8'b0000_1000:   buf_rd_addr[3]    <= buf_rd_addr[3] + 1'b1;
                8'b0001_0000:   buf_rd_addr[4]    <= buf_rd_addr[4] + 1'b1;
                8'b0010_0000:   buf_rd_addr[5]    <= buf_rd_addr[5] + 1'b1;
                8'b0100_0000:   buf_rd_addr[6]    <= buf_rd_addr[6] + 1'b1;
                8'b1000_0000:   buf_rd_addr[7]    <= buf_rd_addr[7] + 1'b1;
                default:        begin
                                    buf_rd_addr[0]    <= buf_rd_addr[0];
                                    buf_rd_addr[1]    <= buf_rd_addr[1];
                                    buf_rd_addr[2]    <= buf_rd_addr[2];
                                    buf_rd_addr[3]    <= buf_rd_addr[3];
                                    buf_rd_addr[4]    <= buf_rd_addr[4];
                                    buf_rd_addr[5]    <= buf_rd_addr[5];
                                    buf_rd_addr[6]    <= buf_rd_addr[6];
                                    buf_rd_addr[7]    <= buf_rd_addr[7];
                                end
            endcase
        end
    end
end

// buf_dout_vld
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_dout_vld    <= 1'b0;
    end
    else begin
        if(buf_rd_en != 8'b0000_0000) begin
            buf_dout_vld    <= 1'b1;
        end
        else begin
            buf_dout_vld    <= 1'b0;
        end
    end
end

// o_dout_vld
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        o_dout_vld    <= 1'b0;
    end
    else begin
        o_dout_vld    <= buf_dout_vld;
    end
end

// buf_rd_en_dly
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_rd_en_dly    <= 'd0;
    end
    else begin
        buf_rd_en_dly    <= buf_rd_en;
    end
end

// o_dout
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        o_dout    <= 'd0;
    end
    else begin
        case(buf_rd_en_dly)
            8'b0000_0001:   begin
                                o_dout    <= buf_rd_dat[0];
                            end
            8'b0000_0010:   begin
                                o_dout    <= buf_rd_dat[1];
                            end
            8'b0000_0100:   begin
                                o_dout    <= buf_rd_dat[2];
                            end
            8'b0000_1000:   begin
                                o_dout    <= buf_rd_dat[3];
                            end
            8'b0001_0000:   begin
                                o_dout    <= buf_rd_dat[4];
                            end
            8'b0010_0000:   begin
                                o_dout    <= buf_rd_dat[5];
                            end
            8'b0100_0000:   begin
                                o_dout    <= buf_rd_dat[6];
                            end
            8'b1000_0000:   begin
                                o_dout    <= buf_rd_dat[7];
                            end
            default:        begin
                                o_dout    <= buf_rd_dat[0];
                            end
        endcase
    end
end

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

genvar j;
generate
    for(j = 0; j < CHAN_NUM; j = j + 1) begin: loop_BRAM
        // Interface Type                   Native
        // Memory Type                      Simple Dual Port RAM
        // Write Width                      12
        // Write Depth                      8192
        // Write Operation Type             No Change
        // Write Enable Port Type           Always Enable
        // Read Width                       384
        // Read Depth                       256
        // Read Operation Type              Write First
        // Read Enable Port Type            Use ENB Pin
        // Read Primitives Output Register  No
        // Read Core Output Register        No
        // Read RSTB Pin                    No
        bram_w12d8192_w384d256_sdpr    chan_buf
        (
            .clka   (clk           ),
            .wea    (buf_wr_en     ),
            .addra  (buf_wr_addr_ch[j]),
            .dina   (buf_wr_dat[j] ),
            .clkb   (clk           ),
            .enb    (buf_rd_en[j]  ),
            .addrb  (buf_rd_addr[j]),
            .doutb  (buf_rd_dat[j] )
        );
    end
endgenerate

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        cmd_frame_req_dly    <= 'd0;
    end
    else begin
        cmd_frame_req_dly    <= {cmd_frame_req_dly[2:0],i_cmd_frame_req};
    end
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        flip_v_en_1dly    <= 'd0;
        flip_v_en_2dly    <= 'd0;
        flip_v_en_3dly    <= 'd0;
    end
    else begin
        flip_v_en_1dly    <= i_flip_v_en   ;
        flip_v_en_2dly    <= flip_v_en_1dly;
        flip_v_en_3dly    <= flip_v_en_2dly;
    end
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        hdr_mode_en_1dly    <= 'd0;
        hdr_mode_en_2dly    <= 'd0;
        hdr_mode_en_3dly    <= 'd0;
    end
    else begin
        hdr_mode_en_1dly    <= i_hdr_mode_en   ;
        hdr_mode_en_2dly    <= hdr_mode_en_1dly;
        hdr_mode_en_3dly    <= hdr_mode_en_2dly;
    end
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        flip_h_en_dly    <= 'd0;
    end
    else begin
        flip_h_en_dly    <= {flip_h_en_dly[2:0],i_flip_h_en};
    end
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        pipe_flag_top_dly    <= 'd0;
        pipe_flag_bot_dly    <= 'd0;
    end
    else begin
        pipe_flag_top_dly    <= {pipe_flag_top_dly[0],i_pipe_flag_top};
        pipe_flag_bot_dly    <= {pipe_flag_bot_dly[0],i_pipe_flag_bot};
    end
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        row1_done_top_dly    <= 1'b0;
        row1_done_bot_dly    <= 1'b0;
    end
    else begin
        row1_done_top_dly    <= i_row1_done_top;
        row1_done_bot_dly    <= i_row1_done_bot;
    end
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        buf_wr_pipe_dly      <= 'd0;
        buf_row1_done_dly    <= 'd0;
    end
    else begin
        buf_wr_pipe_dly      <= {buf_wr_pipe_dly[0],buf_wr_pipe};
        buf_row1_done_dly    <= buf_row1_done;
    end
end

assign o_dbg = {
    frame_rst,           // [31]
    o_dout_vld,          // [30]
    buf_dout_vld,        // [29]
    buf_row1_done,       // [28]
    i_row1_done_top,     // [27]
    i_row1_done_bot,     // [26]
    i_pipe_flag_top,     // [25]
    i_pipe_flag_bot,     // [24]
    cs_mult,             // [23:22]
    cnt_chan,            // [21:19]
    cnt_wr_buf[8:0],     // [18:10]
    cnt_rd_buf[8:0],     // [9:1]
    flag_rd              // [0]
};

endmodule
