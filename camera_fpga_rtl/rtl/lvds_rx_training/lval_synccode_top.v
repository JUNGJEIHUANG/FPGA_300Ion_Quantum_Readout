`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//------------------------------------------------------------------------------------------------------------------
// --Module    : lval_synccode_top
// --File      : lval_synccode_top.v
// --Author    : tianjiao.hou@gpixel.com 
//------------------------------------------------------------------------------------------------------------------
// --Function  : Synchronize all channels and generate lval(o_dat_vld)
//------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// -- 1.0      2022.05.30   Tianjiao Hou    creat 
// -- 1.1      2022.06.22   Danyang Li      modify 
// -- 1.3      2022.11.01   Tianjiao Hou    modify 
// -- 1.4      2023.05.10   Tianjiao Hou    i_cmd_frame_req added, synccode as input for universal use, sol ynamically changing support
//------------------------------------------------------------------------------------------------------------------
module lval_synccode_top#(
    parameter    LVDS_PAIRS             = 8     , // sensor channel number
    parameter    DAT_WIDTH              = 12    , // data width supported : 12/14/16
    parameter    SOL_INCREASE_EN        = 1'b0  , // synccode_4th_sol dynamically changing based on which line it is // eg.SOL<N> : (AB0+N)h
                                                  //  when SOL_INCREASE_EN = 1, 'dynamically forth word of sync code' is from i_synccode_4th_sol to 'i_synccode_4th_sol + i_win_row_len - 1'
    parameter    FIFO_DEPTH             = 8192  , // >i_lval_len // FIFO depth:1024/2048/4096/8192/16384
    parameter    FIFO_DATACOUNT_WIDTH   = 13    , // width of FIFO's data count //2^FIFO_DATACOUNT_WIDTH=FIFO_DEPTH
    parameter    CHAN_VLD_1ST           = 0     , // effective channel that will not be merged
    parameter    CHAN_VLD_2ND           = 4       // effective channel, no matter it is same to CHAN_VLD_1ST, or merged, or not merged
)
(
    input                                          clk                 ,
    input                                          rst                 ,
    input                                          i_cmd_frame_req     , // cmd_frame_req,rising edge effective  
    input            [DAT_WIDTH-1:0]               i_synccode_1st      , // first word of sync code
    input            [DAT_WIDTH-1:0]               i_synccode_2nd      , // second word of sync code
    input            [DAT_WIDTH-1:0]               i_synccode_3rd      , // third word of sync code
    input            [DAT_WIDTH-1:0]               i_synccode_4th_sof  , // forth word of sync code, start of frame  //not used when SOL_INCREASE_EN=1
    input            [DAT_WIDTH-1:0]               i_synccode_4th_sol  , // forth word of sync code, start of line
    input            [15:0]                        i_lval_len          , // valid pixel number in one line
    input            [15:0]                        i_win_row_len       , // row(line) number in one frame
    input            [15:0]                        i_frame_num         , // frame number per i_cmd_frame_req
    input            [LVDS_PAIRS-1:0]              i_dat_vld           , // i_dat valid flag
    input            [LVDS_PAIRS*DAT_WIDTH-1:0]    i_dat               , // input data
    output wire [LVDS_PAIRS*DAT_WIDTH-1:0] o_fifo_din_dbg,
    output wire [LVDS_PAIRS-1:0] o_fifo_wen_dbg, o_fifo_rst_dbg,
    output    reg                                  o_dat_vld           , // o_dat valid flag
    output    reg    [LVDS_PAIRS*DAT_WIDTH-1:0]    o_dat               , // output data
    output           [31:0]                         o_dbg,
    output           [191:0]                        o_fifo_dout_dbg,
    output           [63:0]                         o_internal_dbg,
    output           [63:0]                         o_lane0_dbg,
    output           [63:0]                         o_lane8_dbg
    );
//-------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------

localparam    s_idle        = 'd0 ,
              s_gen         = 'd1 ,
              s_end         = 'd2 ;

//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------


reg     [15:0]                             lval_len_1dly               ;
reg     [15:0]                             lval_len_2dly               ;
reg     [15:0]                             win_row_len_1dly            ;
reg     [15:0]                             win_row_len_2dly            ;
reg     [15:0]                             frame_num_1dly              ;
reg     [15:0]                             frame_num_2dly              ;
(*use_dsp="yes"*)wire    [31:0]            row_all_calc                ;
reg     [31:0]                             row_all_m1                  ;
reg     [31:0]                             cnt_row                     ;
reg     [3:0]                              cmd_frame_req_dly           ;
reg     [15:0]                             cnt_lval_len                ;
reg     [1:0]                              fsm_lval                    ;
reg                                        fifo_ren                    ;
wire    [LVDS_PAIRS*DAT_WIDTH-1:0]         fifo_dout                   ;
wire    [FIFO_DATACOUNT_WIDTH-1:0]         data_count[LVDS_PAIRS-1:0]  ;
wire    [63:0]                              lane_dbg[LVDS_PAIRS-1:0]    ;
reg                                        fifo_ren_1dly               ;
reg                                        fifo_ren_2dly               ;
reg     [LVDS_PAIRS*DAT_WIDTH-1:0]         fifo_dout_1dly              ;
reg     [4:0]                              cnt_frame_rst               ;
reg                                        frame_rst                   ;
reg                                        frame_rst_dly               ;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

assign    row_all_calc = win_row_len_2dly * frame_num_2dly;

//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------

always @(posedge clk  or posedge rst)begin
    if(rst == 1'b1)begin  
        cnt_frame_rst    <= 'd0;
        frame_rst_dly    <= 1'b0;
        frame_rst        <= 1'b0;
    end
    else begin 
        frame_rst_dly    <= frame_rst;
        if(cmd_frame_req_dly[3:2] == 2'b01)begin 
            cnt_frame_rst    <= 'd0;
            frame_rst        <= 1'b1;
        end
        else begin
            if(cnt_frame_rst >= 'd15)begin
                frame_rst        <= 1'b0;
            end
            else begin
                cnt_frame_rst    <= cnt_frame_rst + 1'b1;
            end
        end
    end
end 

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1)begin 
        row_all_m1         <= 'd0;
    end
    else begin
        row_all_m1         <= (row_all_calc == 32'd0) ? 32'd0 : (row_all_calc - 1'b1);
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1)begin 
        cnt_lval_len       <= 'd0;
        cnt_row            <= 'd0;
        fifo_ren           <= 1'b0;
        fsm_lval           <= s_idle;
    end
    else begin
        case(fsm_lval)
            s_idle: if((frame_rst == 1'b0)&&(frame_rst_dly == 1'b1))begin 
                        fsm_lval           <= s_gen;
                        cnt_row            <= 'd0;
                        fifo_ren           <= 1'b0;
                    end
            s_gen:  if((frame_rst == 1'b1)&&(frame_rst_dly == 1'b0))begin 
                        fsm_lval           <= s_idle;
                        fifo_ren           <= 1'b0;
                    end
                    else if((data_count[CHAN_VLD_1ST] >= lval_len_2dly[FIFO_DATACOUNT_WIDTH-1:0])||(data_count[CHAN_VLD_2ND]>=lval_len_2dly[FIFO_DATACOUNT_WIDTH-1:0]))begin //  ||   base on clock tree of different project 
                        fifo_ren           <= 1'b1;
                        cnt_lval_len       <= lval_len_2dly - 1'b1;
                        fsm_lval           <= s_end;
                    end
            s_end:  begin
                        if((frame_rst == 1'b1)&&(frame_rst_dly == 1'b0))begin 
                            fsm_lval           <= s_idle;
                            fifo_ren           <= 1'b0;
                        end
                        else if(cnt_lval_len == 'd0)begin
                            fifo_ren           <= 1'b0;
                            if(cnt_row == row_all_m1) begin
                                fsm_lval           <= s_idle;
                            end
                            else begin
                                cnt_row            <= cnt_row + 1'b1;
                                fsm_lval           <= s_gen;
                            end
                        end
                        else begin
                            cnt_lval_len       <= cnt_lval_len - 1'b1;
                        end 
                    end 
            default: fsm_lval           <= s_idle;
        endcase  
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        fifo_ren_1dly     <= 1'b0;
        fifo_ren_2dly     <= 1'b0;
        o_dat_vld         <= 1'b0;
        fifo_dout_1dly    <= {LVDS_PAIRS*DAT_WIDTH{1'b0}};
        o_dat             <= {LVDS_PAIRS*DAT_WIDTH{1'b0}};
    end
    else begin
        fifo_ren_1dly     <= fifo_ren;
        fifo_ren_2dly     <= fifo_ren_1dly;
        o_dat_vld         <= fifo_ren_2dly;
        fifo_dout_1dly    <= fifo_dout;
        o_dat             <= fifo_dout_1dly;
    end
end

//--------------------------------------------------------------------------------------------------------------------
//Instance
//--------------------------------------------------------------------------------------------------------------------
// base on clock tree of different project 
genvar    i;
generate
    for (i = 0; i < LVDS_PAIRS; i = i+1) begin:loop
        lval_synccode#(
            .DAT_WIDTH            (DAT_WIDTH               ), 
            .SOL_INCREASE_EN      (SOL_INCREASE_EN         ),  
            .FIFO_DEPTH           (FIFO_DEPTH              ), 
            .FIFO_DATACOUNT_WIDTH (FIFO_DATACOUNT_WIDTH    )
        )  u_lval_synncode(
            .clk                  (clk                                      ),
            .rst                  (rst                                      ),
            .i_cmd_frame_req      (i_cmd_frame_req                          ),
            .i_synccode_1st       (i_synccode_1st                           ), 
            .i_synccode_2nd       (i_synccode_2nd                           ), 
            .i_synccode_3rd       (i_synccode_3rd                           ), 
            .i_synccode_4th_sof   (i_synccode_4th_sof                       ), 
            .i_synccode_4th_sol   (i_synccode_4th_sol                       ),  
            .i_lval_len           (i_lval_len                               ),
            .i_win_row_len        (i_win_row_len                            ),
            .i_dat_vld            (i_dat_vld[i]                             ),
            .i_dat                (i_dat[DAT_WIDTH*(i+1)-1:DAT_WIDTH*i]     ),
            .i_fifo_ren           (fifo_ren                                 ),
            .o_fifo_dout          (fifo_dout[DAT_WIDTH*(i+1)-1:DAT_WIDTH*i] ),
            .o_data_count         (data_count[i]                            ),
            .o_fifo_din_dbg       (o_fifo_din_dbg[DAT_WIDTH*(i+1)-1:DAT_WIDTH*i]),
            .o_fifo_wen_dbg       (o_fifo_wen_dbg[i]),
            .o_fifo_rst_dbg       (o_fifo_rst_dbg[i]),
            .o_debug              (lane_dbg[i]                              )
        );
    end
endgenerate

assign o_fifo_dout_dbg = fifo_dout;
assign o_lane0_dbg = lane_dbg[0];
assign o_lane8_dbg = lane_dbg[8];
assign o_internal_dbg = {
    7'd0, data_count[1], data_count[0], fifo_ren, fsm_lval,
    cnt_row[11:0], cnt_lval_len[13:0], frame_rst, frame_rst_dly
};

assign o_dbg = {
    frame_rst,           // [31]
    frame_rst_dly,       // [30]
    o_dat_vld,           // [29]
    fifo_ren,            // [28]
    fsm_lval,            // [27:26]
    cnt_row[11:0],       // [25:14]
    cnt_lval_len[13:0]   // [13:0]
};

//----------------------------------------------------------
// CDC
//----------------------------------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1)begin 
        cmd_frame_req_dly    <= 'd0;
    end
    else begin
        cmd_frame_req_dly    <= {cmd_frame_req_dly[2:0],i_cmd_frame_req};
    end
end


always @(posedge clk  or posedge rst)begin
    if(rst == 1'b1)begin  
        lval_len_1dly              <= 'd0; 
        lval_len_2dly              <= 'd0;  
    end
    else begin 
        lval_len_1dly              <= i_lval_len   ;
        lval_len_2dly              <= lval_len_1dly;
    end
end 

always @(posedge clk  or posedge rst)begin
    if(rst == 1'b1)begin  
        win_row_len_1dly              <= 'd0; 
        win_row_len_2dly              <= 'd0;  
    end
    else begin 
        win_row_len_1dly              <= i_win_row_len   ;
        win_row_len_2dly              <= win_row_len_1dly;
    end
end 

always @(posedge clk  or posedge rst)begin
    if(rst == 1'b1)begin  
        frame_num_1dly              <= 'd0; 
        frame_num_2dly              <= 'd0;  
    end
    else begin 
        frame_num_1dly              <= i_frame_num   ;
        frame_num_2dly              <= frame_num_1dly;
    end
end 


endmodule
