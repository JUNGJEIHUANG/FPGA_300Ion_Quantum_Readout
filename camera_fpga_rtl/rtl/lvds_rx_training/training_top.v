`timescale 1ns / 1ps
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : training_top
// --File      : training_top.v
// --Author    : guangcai.ding@gpixel.com
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : training_top module.
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// -- 1.0      2022.05.30   Tianjiao Hou    Creat 
// -- 1.1      2022.06.15   Danyang Li      Modify style 
// -- 1.2      2022.06.28   Danyang Li      Three state FSM create
// -- 1.3      2022.11.01   Tianjiao Hou    Modify: 1.o_fifo_train_din from 6 to 7, chan_sel added. 2. o_train_done    change to pulse of 16 clk
// -- 1.4      2024.04.28   Guangcai Ding   Modify style
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module training_top #(
    parameter    CLKDDR_PAIRS    = 1  ,
    parameter    DAT_PAIRS       = 16 ,
    parameter    DAT_WIDTH       = 12  
)(
    input            [CLKDDR_PAIRS-1:0]           clk_rx              ,
    input            [CLKDDR_PAIRS-1:0]           clk_ddr             ,
    input            [CLKDDR_PAIRS-1:0]           rst_rx              ,
    input                                         clk_sys             ,
    input                                         rst_sys             ,
    input                                         clk_glb             ,
    input                                         rst_glb             ,
    input            [DAT_PAIRS-1:0]              i_SENSOR_DATA_P     ,
    input            [DAT_PAIRS-1:0]              i_SENSOR_DATA_N     ,
    input                                         i_msb_first_en      ,
    input            [7:0]                        i_adc_depth         ,    // 8:8bit,9:9bit,10:10bit,11:11bit,12:12bit, 14:14bit,16:16bit 
    input            [15:0]                       i_training_word     ,
    input            [6:0]                        i_stable_len_half   ,
    input            [DAT_PAIRS-1:0]              i_chan_vld          ,
    input                                         i_cmd_start_train   ,
    output           [DAT_PAIRS-1:0]              o_dat_vld           ,
    output           [DAT_PAIRS*DAT_WIDTH-1:0]    o_dat               ,
    output    reg                                 o_train_done        ,
    output    reg                                 o_train_err         ,
    output    reg                                 o_fifo_train_wen    ,
    output    reg    [7:0]                        o_fifo_train_din,
    output           [63:0]                       o_lane8_align_dbg
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

localparam    s_wr_idle        = 4'd0  ,
              s_TO_time      = 4'd1  ,
              s_wr_chan        = 4'd2  ,
              s_wr_start       = 4'd3  ,
              s_wr_mid         = 4'd4  ,
              s_wr_end         = 4'd5  ,
              s_wr_word        = 4'd6  ,
              s_wr_times       = 4'd7  ,
              s_wr_ok          = 4'd8  ,
              s_chan_num       = 4'd9  ,
              s_done           = 4'd10 ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

(*async_reg="true"*)reg     [3:0]    cmd_start_train_dly       ;
reg     [DAT_PAIRS-1:0]     chan_vld_1dly                      ;
reg     [DAT_PAIRS-1:0]     chan_vld_2dly                      ;
reg     [DAT_PAIRS-1:0]     chan_vld_3dly                      ;
wire    [DAT_WIDTH-1:0]     data_par_2d[DAT_PAIRS-1:0]         ;
wire    [63:0]              align_dbg[DAT_PAIRS-1:0]          ;
wire    [7:0]               loc_eye_start[DAT_PAIRS-1:0]       ;
reg     [7:0]               loc_eye_start_1dly[DAT_PAIRS-1:0]  ;
reg     [7:0]               loc_eye_start_2dly[DAT_PAIRS-1:0]  ;
wire    [7:0]               loc_eye_mid[DAT_PAIRS-1:0]         ;
reg     [7:0]               loc_eye_mid_1dly[DAT_PAIRS-1:0]    ;
reg     [7:0]               loc_eye_mid_2dly[DAT_PAIRS-1:0]    ;
wire    [7:0]               loc_eye_end[DAT_PAIRS-1:0]         ;
reg     [7:0]               loc_eye_end_1dly[DAT_PAIRS-1:0]    ;
reg     [7:0]               loc_eye_end_2dly[DAT_PAIRS-1:0]    ;
wire    [7:0]               cnt_timeout[DAT_PAIRS-1:0]         ;
reg     [7:0]               cnt_timeout_1dly[DAT_PAIRS-1:0]    ;
reg     [7:0]               cnt_timeout_2dly[DAT_PAIRS-1:0]    ;
wire    [4:0]               loc_word[DAT_PAIRS-1:0]            ;
reg     [4:0]               loc_word_1dly[DAT_PAIRS-1:0]       ;
reg     [4:0]               loc_word_2dly[DAT_PAIRS-1:0]       ;
wire    [DAT_PAIRS-1:0]     loc_ok                             ;
reg     [DAT_PAIRS-1:0]     loc_ok_1dly                        ;
reg     [DAT_PAIRS-1:0]     loc_ok_2dly                        ;
reg     [DAT_PAIRS-1:0]     loc_err                            ;
reg     [DAT_PAIRS-1:0]     train_rlt                          ;
wire    [DAT_PAIRS-1:0]     train_done                         ;
reg     [DAT_PAIRS-1:0]     train_done_1dly                    ;
reg     [DAT_PAIRS-1:0]     train_done_2dly                    ;
reg     [3:0]               ns_train_top,cs_train_top          ;
reg     [7:0]               chan_sel                           ;
reg     [23:0]              cnt_TO_time                      ;
reg     [7:0]               cnt_done                           ;
reg                         state_next                         ;
reg                         state_back                         ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
assign o_lane8_align_dbg = align_dbg[8];

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk_sys or posedge rst_sys) begin
   if(rst_sys == 1)begin
        loc_err    <= 'd0;
    end
    else begin
        loc_err    <= ~loc_ok_1dly;
    end
end

always @(posedge clk_sys or posedge rst_sys) begin
   if(rst_sys == 1)begin
        train_rlt    <= 'd0;
    end
    else begin
        train_rlt    <= chan_vld_3dly&loc_err;
    end
end

always @(posedge clk_sys or posedge rst_sys) begin
   if(rst_sys == 1)begin
        o_train_err    <= 1'b0;
    end
    else begin 
        if(train_rlt != {DAT_PAIRS{1'b0}})begin
            o_train_err    <= 1'b1;
        end
        else begin
            o_train_err    <= 1'b0;
        end
    end
end

genvar    j;
generate
    for (j = 0; j < DAT_PAIRS; j = j+1) begin:loop_dly
        always @(posedge clk_sys or posedge rst_sys) begin
            if(rst_sys == 1) begin
                loc_eye_start_1dly[j]      <= 'd0;
                loc_eye_start_2dly[j]      <= 'd0;
                loc_eye_mid_1dly[j]        <= 'd0;
                loc_eye_mid_2dly[j]        <= 'd0;
                loc_eye_end_1dly[j]        <= 'd0;
                loc_eye_end_2dly[j]        <= 'd0;
                cnt_timeout_1dly[j]        <= 'd0;
                cnt_timeout_2dly[j]        <= 'd0;
                loc_word_1dly[j]           <= 'd0;
                loc_word_2dly[j]           <= 'd0;
                loc_ok_1dly[j]             <= 1'b0;
                loc_ok_2dly[j]             <= 1'b0;
                train_done_1dly[j]         <= 1'b0;
                train_done_2dly[j]         <= 1'b0;
            end
            else begin
                loc_eye_start_1dly[j]      <= loc_eye_start[j];
                loc_eye_start_2dly[j]      <= loc_eye_start_1dly[j];
                loc_eye_mid_1dly[j]        <= loc_eye_mid[j];
                loc_eye_mid_2dly[j]        <= loc_eye_mid_1dly[j];
                loc_eye_end_1dly[j]        <= loc_eye_end[j];
                loc_eye_end_2dly[j]        <= loc_eye_end_1dly[j];
                cnt_timeout_1dly[j]        <= cnt_timeout[j];
                cnt_timeout_2dly[j]        <= cnt_timeout_1dly[j];
                loc_word_1dly[j]           <= loc_word[j];
                loc_word_2dly[j]           <= loc_word_1dly[j];
                loc_ok_1dly[j]             <= loc_ok[j];
                loc_ok_2dly[j]             <= loc_ok_1dly[j];
                train_done_1dly[j]         <= train_done[j];
                train_done_2dly[j]         <= train_done_1dly[j];
            end
        end
    end
endgenerate

always @(posedge clk_sys or posedge rst_sys) begin
    if(rst_sys == 1) begin
        cs_train_top    <= s_wr_idle;
    end
    else begin
        cs_train_top    <= ns_train_top;
    end
end

always @(*) begin
    case (cs_train_top)
        s_wr_idle:  begin
                        if(state_next == 1'b1) begin
                            ns_train_top    = s_TO_time;
                        end
                        else begin
                            ns_train_top    = s_wr_idle;
                        end
                    end
        s_TO_time:begin
                        if(state_next == 1'b1) begin 
                            ns_train_top   = s_wr_chan;
                        end
                        else begin
                            ns_train_top    = s_TO_time;
                        end
                    end
        s_wr_chan:  begin
                        ns_train_top    = s_wr_start;
                    end
        s_wr_start: begin
                        ns_train_top    = s_wr_mid;
                    end
        s_wr_mid:   begin
                        ns_train_top    = s_wr_end;
                    end
        s_wr_end:   begin
                        ns_train_top    = s_wr_word;
                    end
        s_wr_word:  begin
                        ns_train_top    = s_wr_times;
                    end
        s_wr_times: begin
                        ns_train_top    = s_wr_ok;
                    end
        s_wr_ok:    begin
                        ns_train_top    = s_chan_num;
                    end
        s_chan_num: begin
                        if(state_back == 1'b1) begin
                            ns_train_top    = s_wr_chan;
                        end
                        else if(state_next == 1'b1) begin
                            ns_train_top    = s_done;
                        end
                        else begin
                            ns_train_top    = s_chan_num;
                        end
                    end
        s_done:     begin
                        if(state_next == 1'b1) begin
                            ns_train_top    = s_wr_idle;
                        end
                        else begin
                            ns_train_top    = s_done;
                        end 
                    end
        default:    ns_train_top    = s_wr_idle;
    endcase
end

always @(posedge clk_sys or posedge rst_sys) begin
    if(rst_sys == 1) begin
        o_fifo_train_din    <= 8'd0;
        o_fifo_train_wen    <= 1'b0;
        chan_sel            <= 'd0 ;
        cnt_TO_time       <= 'd0;
        state_next          <= 1'b0;
        state_back          <= 1'b0;
        cnt_done            <= 'd0;
        o_train_done        <= 1'b0;
    end
    else begin
        case(cs_train_top)
            s_wr_idle:      begin
                                if(state_next == 1'b0) begin
                                    if((cmd_start_train_dly[3:2] == 2'b10)) begin
                                        state_next       <= 1'b1;
                                        chan_sel         <= 'd0;
                                        cnt_TO_time    <= 'd0;
                                        cnt_done         <= 'd0;
                                        o_train_done     <= 1'b0;
                                    end
                                end
                                else begin
                                    state_next    <= 1'b0;
                                end
                            end
            s_TO_time:    begin
                                if(state_next == 1'b0) begin
                                    if((cnt_TO_time > 'd255) && (train_done_2dly == {DAT_PAIRS{1'b1}})) begin
                                        state_next    <= 1'b1;
                                    end
                                    else if(cnt_TO_time == 'hFFFFFF)begin
                                        state_next    <= 1'b1;
                                    end
                                    else begin
                                        cnt_TO_time    <= cnt_TO_time + 1'b1;
                                    end
                                end
                                else begin
                                    state_next    <= 1'b0;
                                end
                            end
            s_wr_chan:      begin
                                o_fifo_train_din    <= chan_sel;
                                o_fifo_train_wen    <= 1'b1;
                            end 
            s_wr_start:     begin
                                o_fifo_train_din    <= loc_eye_start_2dly[chan_sel];
                                o_fifo_train_wen    <= 1'b1;
                            end  
            s_wr_mid:       begin
                                o_fifo_train_din    <= loc_eye_mid_2dly[chan_sel];
                                o_fifo_train_wen    <= 1'b1;
                            end
            s_wr_end:       begin
                                o_fifo_train_din    <= loc_eye_end_2dly[chan_sel];
                                o_fifo_train_wen    <= 1'b1;
                            end
            s_wr_word:      begin
                                o_fifo_train_din    <= {3'd0,loc_word_2dly[chan_sel]};
                                o_fifo_train_wen    <= 1'b1;
                            end
            s_wr_times:     begin
                                o_fifo_train_din    <= cnt_timeout_2dly[chan_sel];
                                o_fifo_train_wen    <= 1'b1;
                            end
            s_wr_ok:        begin
                                o_fifo_train_din    <= {7'b0000000, loc_ok_2dly[chan_sel]};
                                o_fifo_train_wen    <= 1'b1;
                            end
            s_chan_num:     begin
                                if((state_next == 1'b0)&&(state_back == 1'b0))begin
                                    o_fifo_train_wen    <= 1'b0;
                                    if(chan_sel == DAT_PAIRS-1)begin
                                        chan_sel      <= 'd0;
                                        state_next    <= 1'b1;
                                    end
                                    else begin
                                        state_back    <= 1'b1;
                                        chan_sel      <= chan_sel + 1'b1;
                                    end
                                end
                                else begin
                                    state_next    <= 1'b0;
                                    state_back    <= 1'b0;
                                end
                            end
            s_done:         begin
                                if(state_next == 1'b0) begin
                                    if (cnt_done == 'd15) begin
                                        state_next      <= 1'b1;
                                        cnt_done        <= 'd0;
                                        o_train_done    <= 1'b0;
                                    end
                                    else begin
                                        o_train_done    <= 1'b1;
                                        cnt_done        <= cnt_done + 1'b1;
                                    end
                                end
                                else begin
                                    state_next    <= 1'b0;
                                end
                            end
        endcase
   end
end

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// base on clock tree of different project 
genvar    i;
generate
    for (i = 0; i < DAT_PAIRS/2; i = i+1) begin:loop_train_clk0
        //train_word_align_7serial_0    u_train_word_align(
        train_word_align_debug u_train_word_align(
            .clk_rx                  (clk_rx[0]                           ),
            .clk_ddr                 (clk_ddr[0]                          ),
            .rst_rx                  (rst_rx[0]                           ),
            .clk_glb                 (clk_glb                             ),
            .rst_glb                 (rst_glb                             ),
            .i_SENSOR_DATA_P         (i_SENSOR_DATA_P[i]                  ),
            .i_SENSOR_DATA_N         (i_SENSOR_DATA_N[i]                  ),
            .i_adc_depth             (i_adc_depth                         ),
            .i_cmd_start_train       (i_cmd_start_train                   ),
            .i_training_word         (i_training_word[DAT_WIDTH-1:0]      ),
            .i_stable_len_half       (i_stable_len_half                   ),
            .i_msb_first_en          (i_msb_first_en                      ),
            .o_dat_vld               (o_dat_vld[i]                        ),
            .o_dat                   (data_par_2d[i]                      ),
            .o_train_done            (train_done[i]                       ),
            .o_loc_eye_start         (loc_eye_start[i]                    ),
            .o_loc_eye_mid           (loc_eye_mid[i]                      ),
            .o_loc_eye_end           (loc_eye_end[i]                      ),
            .o_cnt_timeout           (cnt_timeout[i]                      ),
            .o_loc_word              (loc_word[i]                         ),
            .o_loc_ok                (loc_ok[i]                           ),
            .o_debug                 (align_dbg[i]                    )
        );
        assign    o_dat[(i+1) * DAT_WIDTH - 1:i * DAT_WIDTH]    = data_par_2d[i];
    end
endgenerate

generate
    for (i = DAT_PAIRS/2; i < DAT_PAIRS; i = i+1) begin:loop_train_clk1
        train_word_align_debug u_train_word_align(
            .clk_rx                  (clk_rx[0]                           ),
            .clk_ddr                 (clk_ddr[0]                          ),
            .rst_rx                  (rst_rx[0]                           ),
            .clk_glb                 (clk_glb                             ),
            .rst_glb                 (rst_glb                             ),
            .i_SENSOR_DATA_P         (i_SENSOR_DATA_P[i]                  ),
            .i_SENSOR_DATA_N         (i_SENSOR_DATA_N[i]                  ),
            .i_adc_depth             (i_adc_depth                         ),
            .i_cmd_start_train       (i_cmd_start_train                   ),
            .i_training_word         (i_training_word[DAT_WIDTH-1:0]      ),
            .i_stable_len_half       (i_stable_len_half                   ),
            .i_msb_first_en          (i_msb_first_en                      ),
            .o_dat_vld               (o_dat_vld[i]                        ),
            .o_dat                   (data_par_2d[i]                      ),
            .o_train_done            (train_done[i]                       ),
            .o_loc_eye_start         (loc_eye_start[i]                    ),
            .o_loc_eye_mid           (loc_eye_mid[i]                      ),
            .o_loc_eye_end           (loc_eye_end[i]                      ),
            .o_cnt_timeout           (cnt_timeout[i]                      ),
            .o_loc_word              (loc_word[i]                         ),
            .o_loc_ok                (loc_ok[i]                           ),
            .o_debug                 (align_dbg[i]                    )
        );
        assign    o_dat[(i+1) * DAT_WIDTH - 1:i * DAT_WIDTH]    = data_par_2d[i];
    end
endgenerate

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk_sys or posedge rst_sys) begin
    if(rst_sys == 1) begin 
        cmd_start_train_dly    <= 'd0;
    end
    else begin
        cmd_start_train_dly    <= {cmd_start_train_dly[2:0],i_cmd_start_train}; 
    end
end


always @(posedge clk_sys or posedge rst_sys) begin
   if(rst_sys == 1)begin  
        chan_vld_1dly    <= 'd0;
        chan_vld_2dly    <= 'd0;
        chan_vld_3dly    <= 'd0;
    end
    else begin 
        chan_vld_1dly    <= i_chan_vld;
        chan_vld_2dly    <= chan_vld_1dly;
        chan_vld_3dly    <= chan_vld_2dly;
    end
end

endmodule


// ---- Inlined lane-8 debug implementation (unique module names) ----
//------------------------------------------------------------------------------------------------------------------
//                  --  Copyright (c) 2012 Gpixel, Technology Co. Ltd.      --
//                  --          All Rights Reserved.                        --
//------------------------------------------------------------------------------------------------------------------
//  -- Module       : align_ctrl
//  -- File         : align_ctrl.v
//  -- Author       : tianjiao.hou@gpixel.com 
//------------------------------------------------------------------------------------------------------------------
//  -- Function     : Bit and Word alignment control
//------------------------------------------------------------------------------------------------------------------
//  -- Hisoty       :
//  -- Ver          Date        By              Description 
//  -- 1.0          2022.05.30  Tianjiao Hou    creat 
//  -- 1.1          2022.06.15  Danyang Li      modify style 
//  -- 1.2          2022.06.29  Danyang Li      Three FSM create 
//  -- 1.4          2022.12.22  Tianjiao Hou    combine zynq and k7,define zynq or not(K7) in define  module or top module
//  -- 1.5          2023.05.24  Tianjiao Hou    module name changed,improve training algorithm, split train_word_align into align_ctrl, iser2des and fifo_w12d16_icdr_st
//  -- 1.6          2023.08.11  Tianjiao Hou    1.change the meaning represented by the i_adc_depth value
//                                              2.add 9bit and 11bit adc depth
//------------------------------------------------------------------------------------------------------------------

module align_ctrl_debug#(
    parameter    TAP_MAX                = 31   , // k7:32, //zynq:255 when 'CNTVALUEIN' of ISERDESE3 is {o_idelay_valuein,1'b0}
    parameter    DAT_WIDTH              = 12     // max data width  // 8/10/12/14/16 supported
)
(
    input                               clk                  , // frequency is 1/4 clk_ddr, clk_ddr is sensor output lvds clk
    input                               rst                  , // clk_rx domain
    input            [7:0]              i_adc_depth          , // 8:8bit,9:9bit,10:10bit,11:11bit,12:12bit, 14:14bit,16:16bit 
    input                               i_cmd_start_train    , // command of start training 
    input            [DAT_WIDTH-1:0]    i_training_word      , // trainging word
    input            [6:0]              i_stable_len_half    , // half of one bit's stable length, //It is best to test according to different projects
    input            [15:0]             i_dat                , // sensor lvds data after serial parallel conversion
    input                               i_dat_vld            ,  // 新增：数据有效标志
    output    reg                       o_rst_idelay         , // for 'RST' of IDELAYE
    output    reg                       o_idelay_ld          , // for 'LOAD' of IDELAYE
    output    reg    [7:0]              o_idelay_valuein     , // for 'CNTVALUEIN' of IDELAYE3
    output    reg                       o_bitslip            , // bit moved flag for word align
    output    reg                       o_bitslip_change     , // ISERDESE is 1 to 8,so o_bitslip can only move out of 8 possibilities, o_bitslip_change equal to 0 and 1 make 16 possibilities
    output    reg                       o_train_done         , // training done flag, maintain high for 16 clk_rx
    output    reg    [7:0]              o_loc_eye_start      , // clk_rx domain. start location of i_SENSOR_DATA's stable eye
    output    reg    [7:0]              o_loc_eye_mid        , // clk_rx domain. sample location of i_SENSOR_DATA
    output    reg    [7:0]              o_loc_eye_end        , // clk_rx domain. end location of i_SENSOR_DATA's stable eye
    output    reg    [7:0]              o_cnt_timeout        , // clk_rx domain. times of error align, word align error times:o_cnt_timeout[3:0], bit align error times:o_cnt_timeout[7:4]
    output    reg    [4:0]              o_loc_word           , // clk_rx domain. times of bit moving in word align
    output    reg                       o_loc_ok               // clk_rx domain. align successful flag,  1:correct, 0:error
);

//------------------------------------------------------------------------------------------------------------------
//parameter
//------------------------------------------------------------------------------------------------------------------


localparam    s_idle             = 4'd0 ,
              s_rst              = 4'd1 ,
              s_eye_ld           = 4'd2 ,
              s_eye_sample       = 4'd3 ,
              s_eye_start        = 4'd4 ,
              s_eye_check        = 4'd5 ,
              s_eye_cal          = 4'd6 ,
              s_valuein_ld       = 4'd7 ,
              s_word_align       = 4'd8 ,
              s_static_test      = 4'd9 ,
              s_done             = 4'd10 ;


//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

(*async_reg="true"*)reg    [3:0]       cmd_start_train_dly;
reg    [7:0]     adc_depth_1dly                ;
reg    [7:0]     adc_depth_2dly                ;
reg    [6:0]     stable_len_half_1dly          ;
reg    [6:0]     stable_len_half_2dly          ;
reg    [15:0]    training_word_1dly            ;
reg    [15:0]    training_word_2dly            ; 
reg    [15:0]    training_word                 ;
reg              even_bit_tp                   ;
reg    [3:0]     ns_train_align,cs_train_align ; 
reg    [15:0]    data_dly                      ;
reg    [15:0]    data_curr                     ;
reg    [15:0]    data_curr_pre                 ;
reg    [15:0]    data_ld_pre                   ;
reg              even_bit                      ;
reg              state_next                    ;
reg              state_back                    ;
reg              state_jump                    ;
reg    [5:0]     cnt_align                     ;
reg    [7:0]     cnt_samples                   ;
reg    [7:0]     cnt_bit_steps                 ;
reg    [7:0]     cnt_bit_stable                ;
reg    [7:0]     eye_start                     ;
reg    [7:0]     eye_end                       ;
reg    [7:0]     eye_stable_temp               ;
reg    [7:0]     eye_start_temp                ;
reg    [7:0]     eye_end_temp                  ;
reg              data_stable                   ;
reg              data_stable_start             ;
reg    [3:0]     cnt_bit_err                   ;
reg    [3:0]     cnt_word_err                  ;
reg    [4:0]     cnt_bitslip_step              ;
reg    [11:0]    cnt_static                    ;
reg    [3:0]     cnt_done                      ;


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin  
        training_word         <= 'd0; 
    end
    else begin
        case(adc_depth_2dly) // 8:8bit,9:9bit,10:10bit,11:11bit,12:12bit, 14:14bit,16:16bit 
            'd8:     training_word         <= {8'd0,training_word_2dly[7:0]};
            'd9:     training_word         <= {7'd0,training_word_2dly[8:0]};
            'd10:    training_word         <= {6'd0,training_word_2dly[9:0]};
            'd11:    training_word         <= {5'd0,training_word_2dly[10:0]};
            'd12:    training_word         <= {4'd0,training_word_2dly[11:0]};
            'd14:    training_word         <= {2'd0,training_word_2dly[13:0]};
            'd16:    training_word         <= training_word_2dly;
            default: training_word         <= 'hFFFF;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin  
        even_bit_tp         <= 1'b0; 
    end
    else begin
        even_bit_tp         <= ^training_word;//even parity
    end
end


always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        cs_train_align    <= s_idle;
    end
    else begin
        cs_train_align    <= ns_train_align;
    end
end

always @(*) begin
    case (cs_train_align)
        s_idle :        begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_rst;
                            end
                            else begin
                                ns_train_align <= s_idle;
                            end
                        end
        s_rst :         begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_eye_ld;
                            end
                            else begin
                                ns_train_align <= s_rst;
                            end
                        end
        s_eye_ld :      begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_eye_sample;
                            end
                            else begin
                                ns_train_align <= s_eye_ld;
                            end
                        end
        s_eye_sample :  begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_eye_start;
                            end
                            else begin
                                ns_train_align <= s_eye_sample;
                            end
                        end
        s_eye_start :   begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_eye_check;
                            end
                            else begin
                                ns_train_align <= s_eye_start;
                            end
                        end
        s_eye_check :   begin
                            if(state_back == 1'b1) begin
                                ns_train_align <= s_eye_ld;
                            end
                            else if(state_next == 1'b1) begin
                                ns_train_align <= s_eye_cal;
                            end
                            else begin
                                ns_train_align <= s_eye_check;
                            end
                        end
        s_eye_cal :     begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_valuein_ld;
                            end
                            else if(state_back == 1'b1) begin
                                ns_train_align <= s_rst;
                            end
                            else if(state_jump == 1'b1) begin
                                ns_train_align <= s_done;
                            end
                            else begin
                                ns_train_align <= s_eye_cal;
                            end
                        end
        s_valuein_ld :  begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_word_align;
                            end
                            else begin
                                ns_train_align <= s_valuein_ld;
                            end
                        end
        s_word_align :  begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_static_test;
                            end
                            else begin
                                ns_train_align <= s_word_align;
                            end
                        end
        s_static_test : begin
                            if(state_back == 1'b1) begin
                                ns_train_align <= s_rst;
                            end
                            else if(state_next == 1'b1) begin
                                ns_train_align <= s_done;
                            end
                            else begin
                                ns_train_align <= s_static_test;
                            end
                        end
        s_done :        begin
                            if(state_next == 1'b1) begin
                                ns_train_align <= s_idle;
                            end
                            else begin
                                ns_train_align <= s_done;
                            end
                        end
        default:        ns_train_align    <= s_idle;
    endcase
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        o_loc_eye_start    <= 'd0;
        o_loc_eye_mid      <= 'd0;
        o_loc_eye_end      <= 'd0;
        o_loc_word         <= 'd0;
        o_cnt_timeout      <= 'd0;
        o_loc_ok           <= 1'b0;
        o_train_done       <= 1'b0;
        data_dly           <= 'd0;
        data_curr          <= 'd0;
        data_curr_pre      <= 'd0;
        even_bit           <= 1'b0;
        state_next         <= 1'b0;
        state_back         <= 1'b0;
        state_jump         <= 1'b0; 
        data_ld_pre        <= 'd0; 
        o_rst_idelay       <= 1'b0;
        o_idelay_ld        <= 1'b0;
        o_idelay_valuein     <= 'd0;
        cnt_align          <= 'd0;
        cnt_samples        <= 'd0;
        
        cnt_bit_steps      <= 'd0;
        cnt_bit_stable     <= 'd0;
        eye_start          <= 'd0; 
        eye_end            <= 'd0;
        eye_stable_temp    <= 'd0;
        eye_start_temp     <= 'd0; 
        eye_end_temp       <= 'd0;
        data_stable_start  <= 1'b0; 
        data_stable        <= 1'b0; 
        cnt_bit_err        <= 'd0;
        o_bitslip          <= 1'b0; 
        o_bitslip_change   <= 1'b0;
        cnt_bitslip_step   <= 'd0;
        cnt_word_err       <= 'd0;
        cnt_static         <= 'd0;
        cnt_done           <= 'd0;
    end
    else begin
        if(i_dat_vld) begin                    // 新增：这一行
            data_dly           <= i_dat;       // 原来的三行
            data_curr          <= data_dly;    // 
            data_curr_pre      <= data_curr;   // 
        end                                    // 新增：这一行
        even_bit           <= ^data_curr; 
        case(cs_train_align)
            s_idle :        begin
                                if(state_next == 1'b0) begin
                                    if(cmd_start_train_dly[3:2] == 2'b01) begin
                                        state_next         <= 1'b1;
                                        state_back         <= 1'b0;
                                        state_jump         <= 1'b0;
                                        o_rst_idelay       <= 1'b0;
                                        cnt_align          <= 'd0; 
                                        cnt_done           <= 'd0;
                                        o_bitslip          <= 1'b0;
                                        cnt_bit_err        <= 'd0;
                                        cnt_word_err       <= 'd0;
                                        o_loc_ok           <= 1'b0;
                                        o_train_done       <= 1'b0;
                                    end
                                end
                                else begin
                                    state_next         <= 1'b0;
                                end
                            end
            s_rst :         begin
                                if(state_next == 1'b0) begin 
                                    cnt_bit_steps      <= 'd0;
                                    cnt_bit_stable     <= 'd0;
                                    cnt_samples        <= 'd0;
                                    eye_start          <= 'd0; 
                                    eye_end            <= 'd0;
                                    eye_stable_temp    <= 'd0;
                                    eye_start_temp     <= 'd0; 
                                    eye_end_temp       <= 'd0;
                                    o_bitslip_change   <= 1'b0;
                                    cnt_bitslip_step   <= 'd0;
                                    cnt_static         <= 'd0;
                                    if(cnt_align == 'd15)begin    //wait idelay stable
                                        state_next        <= 1'b1;
                                        o_idelay_valuein    <= 'd0;
                                        data_ld_pre       <= 'd0;    //first CNTVALUEIN euqal to 0, so no previous CNTVALUEIN
                                        cnt_align         <= 'd0; 
                                    end
                                    else if(cnt_align == 'd0)begin    //reset idelay, iserdes
                                        o_rst_idelay      <= 1'b1;
                                        cnt_align         <= cnt_align + 1'b1;
                                    end
                                    else begin
                                        o_rst_idelay      <= 1'b0;
                                        cnt_align         <= cnt_align + 1'b1;
                                    end 
                                end
                                else begin
                                    state_next         <= 1'b0;
                                end
                            end
            s_eye_ld :      begin
                                if(state_next == 1'b0) begin
                                    if(cnt_align == 'd15)begin    //wait idelay stable
                                        state_next        <= 1'b1;
                                        data_stable_start <= 1'b0;
                                        data_stable       <= 1'b1;
                                        cnt_align         <= 'd0; 
                                    end
                                    else if(cnt_align == 'd2)begin    //load idelay CNTVALUEIN to 0
                                        o_idelay_ld       <= 1'b1;
                                        cnt_align         <= cnt_align + 1'b1;
                                    end
                                    else begin
                                        o_idelay_ld       <= 1'b0;
                                        cnt_align         <= cnt_align + 1'b1;
                                    end 
                                end
                                else begin
                                    state_next         <= 1'b0;
                                end
                            end
            s_eye_sample :  begin                                                                     // check whether data is stable and  even parity is right
                                if(state_next == 1'b0) begin                               
                                    if (cnt_samples == 'd255) begin                                    //sample times 
                                        cnt_samples    <= 'd0; 
                                        state_next     <= 1'b1;
                                    end
                                    else begin
                                        cnt_samples    <= cnt_samples + 1'b1;
                                    end
                                    if((even_bit != even_bit_tp)||(data_curr != data_curr_pre))begin  
                                        data_stable       <= 1'b0;
                                    end
                                end
                                else begin
                                    state_next         <= 1'b0;
                                end
                            end
            s_eye_start :   begin                                                                     // check whether data is stable when CNTVALUEIN = 0 
                                if(state_next == 1'b0) begin                               
                                    state_next         <= 1'b1;
                                    if (data_stable == 1'b0) begin 
                                        data_stable_start  <= 1'b0;
                                        data_ld_pre        <= 'd0;
                                    end
                                    else if(data_curr != data_ld_pre)begin 
                                        data_stable_start  <= 1'b1;
                                        data_ld_pre        <= data_curr;
                                    end
                                    else begin
                                        data_stable_start  <= 1'b0;
                                        data_ld_pre        <= data_curr; 
                                    end
                                end
                                else begin
                                    state_next         <= 1'b0;
                                end
                            end 
            s_eye_check :   begin
                                if((state_next == 1'b0)&&(state_back == 1'b0))begin
                                    if(cnt_bit_steps == TAP_MAX) begin
                                        state_next         <= 1'b1;
                                        cnt_bit_steps      <= 'd0; 
                                    end
                                    else begin
                                        state_back         <= 1'b1;
                                        cnt_bit_steps      <= cnt_bit_steps + 1'b1; 
                                    end
                                    
                                    o_idelay_valuein    <= o_idelay_valuein + 1'b1;                   //delay tap + 1
                                    
                                    if (data_stable_start == 1'b1) begin  
                                        cnt_bit_stable     <= 'd1;
                                        eye_start          <= cnt_bit_steps; 
                                        eye_end            <= cnt_bit_steps;
                                    end
                                    else if (data_stable == 1'b1) begin  
                                        cnt_bit_stable     <= cnt_bit_stable + 1'b1; 
                                        eye_end            <= cnt_bit_steps; 
                                    end
                                    else begin  
                                        cnt_bit_stable     <= 'd0;
                                    end 
                                end
                                else begin
                                    state_next         <= 1'b0;
                                    state_back         <= 1'b0;
                                    if(cnt_bit_stable > eye_stable_temp) begin
                                        eye_stable_temp    <= cnt_bit_stable;
                                        eye_start_temp     <= eye_start     ; 
                                        eye_end_temp       <= eye_end       ;
                                    end 
                                    else begin
                                        eye_stable_temp    <= eye_stable_temp;
                                        eye_start_temp     <= eye_start_temp ; 
                                        eye_end_temp       <= eye_end_temp   ;
                                    end 
                                end
                            end
            s_eye_cal :     begin
                                if((state_next == 1'b0)&&(state_back == 1'b0)&&(state_jump == 1'b0))begin
                                    cnt_bit_steps    <= 'd0;
                                    o_loc_eye_start    <= eye_start_temp;
                                    o_loc_eye_end      <= eye_end_temp; 
                                    if(eye_stable_temp <= stable_len_half_2dly) begin
                                        o_loc_eye_mid      <= (eye_start_temp>>1) + (eye_end_temp>>1) + (eye_start_temp[0]^eye_end_temp[0]);
                                        if(cnt_bit_err == 'd7) begin
                                            state_jump          <= 1'b1;
                                            o_loc_word          <= 'd0;
                                            o_cnt_timeout       <= {cnt_bit_err,cnt_word_err};
                                            o_loc_ok            <= 1'b0;
                                            o_train_done        <= 1'b1;
                                        end
                                        else begin
                                            state_back          <= 1'b1;
                                            cnt_bit_err         <= cnt_bit_err + 1'b1;
                                        end
                                    end
                                    else begin
                                        state_next          <= 1'b1;
                                        if(eye_stable_temp >= 2*stable_len_half_2dly) begin
                                            o_loc_eye_mid    <= (eye_start_temp>>1) + (eye_end_temp>>1) + (eye_start_temp[0]^eye_end_temp[0]);
                                        end
                                        else if(eye_start_temp== 'd0) begin
                                            o_loc_eye_mid    <= eye_end_temp - stable_len_half_2dly ;
                                        end
                                        else if(eye_end_temp== TAP_MAX) begin
                                            o_loc_eye_mid    <= eye_start_temp + stable_len_half_2dly ;
                                        end
                                        else begin
                                            o_loc_eye_mid    <= (eye_start_temp>>1) + (eye_end_temp>>1) + (eye_start_temp[0]^eye_end_temp[0]);
                                        end
                                    end
                                end
                                else begin
                                    state_next         <= 1'b0;
                                    state_back         <= 1'b0;  
                                    state_jump         <= 1'b0;  
                                end
                            end 
            s_valuein_ld :  begin
                                if(state_next == 1'b0) begin 
                                    state_next          <= 1'b1;
                                    o_idelay_ld         <= 1'b1;
                                    o_idelay_valuein    <= o_loc_eye_mid;
                                    cnt_bitslip_step    <= 'd0;
                                end
                                else begin
                                    o_idelay_ld         <= 1'b0;
                                    state_next          <= 1'b0;
                                end
                            end
            s_word_align :  begin                                                       // match training word  , iserdes needs two clk to finish  bitslip operation
                                if(state_next == 1'b0) begin                               
                                    if(cnt_align == 'd15) begin
                                        cnt_align            <= 'd0;
                                        if((data_curr == training_word)||(cnt_bitslip_step == 'd16)) begin //word align: cnt_bitslip_step=0 to 15;  cnt_bitslip_step=16 is error
                                            cnt_static       <= 'd0; 
                                            state_next       <= 1'b1;
                                        end
                                        else begin
                                            o_bitslip        <= 1'b1;
                                            cnt_bitslip_step <= cnt_bitslip_step + 1'b1;
                                        end
                                    end
                                    else begin
                                        o_bitslip            <= 1'b0;
                                        cnt_align            <= cnt_align + 1'b1;
                                        if(cnt_bitslip_step < 'd8) begin
                                            o_bitslip_change  <= 1'b0;
                                        end
                                        else  begin
                                            o_bitslip_change  <= 1'b1;
                                        end
                                    end
                                end
                                else begin 
                                    state_next          <= 1'b0;
                                end
                            end
            s_static_test : begin
                                if((state_next == 1'b0)&&(state_back == 1'b0))begin
                                    if(data_curr != training_word) begin  
                                        if (cnt_word_err == 'd7) begin
                                            state_next         <= 1'b1;
                                            o_loc_word         <= cnt_bitslip_step; 
                                            o_cnt_timeout      <= {cnt_bit_err,cnt_word_err};
                                            o_loc_ok           <= 1'b0;
                                        end
                                        else begin
                                            state_back         <= 1'b1;
                                            cnt_bit_err        <= 'd0;
                                            cnt_word_err       <= cnt_word_err + 3'd1;  
                                        end
                                    end
                                    else begin
                                        if (cnt_static == 'd4095) begin
                                            state_next         <= 1'b1;
                                            cnt_static         <= 'd0;
                                            o_loc_word         <= cnt_bitslip_step; 
                                            o_cnt_timeout      <= {cnt_bit_err,cnt_word_err};
                                            o_loc_ok           <= 1'b1; 
                                        end
                                        else begin
                                            cnt_static            <= cnt_static + 1'b1; 
                                        end
                                    end
                                end
                                else begin
                                    state_next         <= 1'b0;
                                    state_back         <= 1'b0;  
                                end
                            end
            s_done :        begin 
                                if(state_next == 1'b0) begin                               
                                    if (cnt_done == 'd15) begin
                                        state_next         <= 1'b1;
                                        cnt_done           <= 'd0;
                                        o_train_done       <= 1'b1;
                                    end
                                    else begin
                                        cnt_done           <= cnt_done + 1'b1; 
                                    end
                                end
                                else begin
                                    state_next         <= 1'b0; 
                                end
                            end
        endcase
    end
end

//--------------------------------------------------------------------------------------------------------------------
//CDC
//--------------------------------------------------------------------------------------------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin   
        cmd_start_train_dly   <= 'd0;
    end
    else begin
        cmd_start_train_dly   <= {cmd_start_train_dly[2:0],i_cmd_start_train}; 
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin   
        adc_depth_1dly             <= 'd0; 
        adc_depth_2dly             <= 'd0; 
    end
    else begin 
        adc_depth_1dly             <= i_adc_depth; 
        adc_depth_2dly             <= adc_depth_1dly; 
    end
end
always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin   
        stable_len_half_1dly             <= 'd0; 
        stable_len_half_2dly             <= 'd0; 
    end
    else begin 
        stable_len_half_1dly             <= i_stable_len_half; 
        stable_len_half_2dly             <= stable_len_half_1dly; 
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin   
        training_word_1dly             <= 'd0; 
        training_word_2dly             <= 'd0; 
    end
    else begin 
        training_word_1dly             <= i_training_word; 
        training_word_2dly             <= training_word_1dly; 
    end
end


endmodule


// ---- Inlined lane-8 debug implementation (unique module names) ----
`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//                  --  Copyright (c) 2012 Gpixel, Technology Co. Ltd.      --  
//                  --          All Rights Reserved.                        --
//------------------------------------------------------------------------------------------------------------------
//  -- Module       : des_8bit_to_adcdepth
//  -- File         : des_8bit_to_adcdepth.v
//  -- Author       : Tianjiao Hou
//------------------------------------------------------------------------------------------------------------------
//  -- Function     : 8bit to 8,10,12bit, incompatible 16bit
//------------------------------------------------------------------------------------------------------------------
//  -- Hisoty       :
//  -- Ver          Date        By              Description 
//  -- 1.0          2022.05.30  Tianjiao Hou    creat 
//  -- 1.1          2022.06.16  Danyang Li      modify style
//  -- 1.2          2022.06.28  Danyang Li      modify
//  -- 1.4          2022.12.22  Tianjiao Hou    modify style
//  -- 1.5          2023.07.01  Tianjiao Hou    combine 8/9/10/11/12/14/16
//  -- 1.6          2023.08.11  Tianjiao Hou    1.change the meaning represented by the i_adc_depth value
//                                              2.add 9bit and 11bit adc depth
//------------------------------------------------------------------------------------------------------------------
module des_8bit_to_adcdepth_debug#(
    parameter    DAT_8BIT_EN            = 1'b0    , // 8bit serial to parral enable 
    parameter    DAT_9BIT_EN            = 1'b1    , // 9bit serial to parral enable
    parameter    DAT_10BIT_EN           = 1'b1    , // 10bit serial to parral enable
    parameter    DAT_11BIT_EN           = 1'b1    , // 11bit serial to parral enable
    parameter    DAT_12BIT_EN           = 1'b1    , // 12bit serial to parral enable
    parameter    DAT_14BIT_EN           = 1'b0    , // 14bit serial to parral enable
    parameter    DAT_16BIT_EN           = 1'b0      // 16bit serial to parral enable
)
(
    input                      clk              , // input clk_rx of module iser2des
    input                      rst              , // input rst_rx of module iser2des
    input                      rst_idelay       , // input i_rst_idelay of iser2des, 'RST' of IDELAYE
    input                      i_bitslip_change , // ISERDESE of module iser2des  is 1 to 8,so input i_bitslip of module iser2des can only move out of 8 possibilities, i_bitslip_change equal to 0 and 1 make 16 possibilities
    input            [7:0]     i_adc_depth      , // 8:8bit,9:9bit,10:10bit,11:11bit,12:12bit, 14:14bit,16:16bit 
    input            [7:0]     i_dat            , // data after serial parallel conversion and bitslip
    input                      i_msb_first_en   , // serial data(i_SENSOR_DATA) output sequence, 1:msb , 0:lsb
    output    reg              o_dat_vld        , // o_dat valid flag, clk domain
    output    reg    [15:0]    o_dat              // output data, clk domain, 16bit fixed no matter DAT_XBIT_EN is 1 or 0, low bit effective
);
 
//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

reg    [2:0]     msb_first_en_dly;
reg    [7:0]     adc_depth_1dly;
reg    [7:0]     adc_depth_2dly;
reg    [7:0]     adc_depth_3dly;
reg    [7:0]     dat_in_1dly;
reg    [7:0]     dat_in_2dly;
reg    [7:0]     dat_in_3dly;
reg    [15:0]    dat_out_1;
reg    [15:0]    dat_out_2; 
reg    [3:0]     rd_state;
reg              dat_vld;

//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------

always @(posedge clk) begin
    dat_in_1dly    <= i_dat;
    dat_in_2dly    <= dat_in_1dly;
    dat_in_3dly    <= dat_in_2dly;
end

always @(posedge clk) begin
    o_dat_vld   <= dat_vld;
    if(msb_first_en_dly[2] == 1)begin
        case (adc_depth_3dly)
            'd8:begin
                    if(DAT_8BIT_EN == 1'b1)begin
                        if(i_bitslip_change)
                            o_dat     <= {8'd0,dat_out_2[0],dat_out_2[1],dat_out_2[2],dat_out_2[3],dat_out_2[4],dat_out_2[5],dat_out_2[6],dat_out_2[7]};
                        else
                            o_dat     <= {8'd0,dat_out_1[0],dat_out_1[1],dat_out_1[2],dat_out_1[3],dat_out_1[4],dat_out_1[5],dat_out_1[6],dat_out_1[7]};
                    end
                 end
            'd9:begin 
                    if(DAT_9BIT_EN == 1'b1)begin
                        if(i_bitslip_change)
                            o_dat     <= {7'd0,dat_out_2[0],dat_out_2[1],dat_out_2[2],dat_out_2[3],dat_out_2[4],dat_out_2[5],dat_out_2[6],dat_out_2[7],dat_out_2[8]};
                        else
                            o_dat     <= {7'd0,dat_out_1[0],dat_out_1[1],dat_out_1[2],dat_out_1[3],dat_out_1[4],dat_out_1[5],dat_out_1[6],dat_out_1[7],dat_out_1[8]};
                     end
               end
            'd10:begin 
                    if(DAT_10BIT_EN == 1'b1)begin
                        if(i_bitslip_change)
                            o_dat     <= {6'd0,dat_out_2[0],dat_out_2[1],dat_out_2[2],dat_out_2[3],dat_out_2[4],dat_out_2[5],dat_out_2[6],dat_out_2[7],dat_out_2[8],dat_out_2[9]};
                        else
                            o_dat     <= {6'd0,dat_out_1[0],dat_out_1[1],dat_out_1[2],dat_out_1[3],dat_out_1[4],dat_out_1[5],dat_out_1[6],dat_out_1[7],dat_out_1[8],dat_out_1[9]};
                     end
               end
            'd11:begin 
                    if(DAT_11BIT_EN == 1'b1)begin
                        if(i_bitslip_change)
                            o_dat     <= {5'd0,dat_out_2[0],dat_out_2[1],dat_out_2[2],dat_out_2[3],dat_out_2[4],dat_out_2[5],dat_out_2[6],dat_out_2[7],dat_out_2[8],dat_out_2[9],dat_out_2[10]};
                        else
                            o_dat     <= {5'd0,dat_out_1[0],dat_out_1[1],dat_out_1[2],dat_out_1[3],dat_out_1[4],dat_out_1[5],dat_out_1[6],dat_out_1[7],dat_out_1[8],dat_out_1[9],dat_out_1[10]};
                     end
               end
            'd12:begin  
                    if(DAT_12BIT_EN == 1'b1)begin
                        if(i_bitslip_change)
                            o_dat     <= {4'd0,dat_out_2[0],dat_out_2[1],dat_out_2[2],dat_out_2[3],dat_out_2[4],dat_out_2[5],dat_out_2[6],dat_out_2[7],dat_out_2[8],dat_out_2[9],dat_out_2[10],dat_out_2[11]};
                        else
                            o_dat     <= {4'd0,dat_out_1[0],dat_out_1[1],dat_out_1[2],dat_out_1[3],dat_out_1[4],dat_out_1[5],dat_out_1[6],dat_out_1[7],dat_out_1[8],dat_out_1[9],dat_out_1[10],dat_out_1[11]};
                    end
                end
            'd14:begin  
                    if(DAT_14BIT_EN == 1'b1)begin
                        if(i_bitslip_change)
                            o_dat     <= {2'd0,dat_out_2[0],dat_out_2[1],dat_out_2[2],dat_out_2[3],dat_out_2[4],dat_out_2[5],dat_out_2[6],dat_out_2[7],dat_out_2[8],dat_out_2[9],dat_out_2[10],dat_out_2[11],dat_out_2[12],dat_out_2[13]};
                        else
                            o_dat     <= {2'd0,dat_out_1[0],dat_out_1[1],dat_out_1[2],dat_out_1[3],dat_out_1[4],dat_out_1[5],dat_out_1[6],dat_out_1[7],dat_out_1[8],dat_out_1[9],dat_out_1[10],dat_out_1[11],dat_out_1[12],dat_out_1[13]};
                    end
                end
            'd16:begin  
                    if(DAT_16BIT_EN == 1'b1)begin
                        if(i_bitslip_change)
                            o_dat     <= {dat_out_2[0],dat_out_2[1],dat_out_2[2],dat_out_2[3],dat_out_2[4],dat_out_2[5],dat_out_2[6],dat_out_2[7],dat_out_2[8],dat_out_2[9],dat_out_2[10],dat_out_2[11],dat_out_2[12],dat_out_2[13],dat_out_2[14],dat_out_2[15]};
                        else
                            o_dat     <= {dat_out_1[0],dat_out_1[1],dat_out_1[2],dat_out_1[3],dat_out_1[4],dat_out_1[5],dat_out_1[6],dat_out_1[7],dat_out_1[8],dat_out_1[9],dat_out_1[10],dat_out_1[11],dat_out_1[12],dat_out_1[13],dat_out_1[14],dat_out_1[15]};
                    end
                end
            default:o_dat <= 'd0 ;
        endcase
    end
    else begin
        if(i_bitslip_change)
            o_dat     <= dat_out_2;
        else
            o_dat     <= dat_out_1;
    end

end 

always @(posedge clk or posedge rst_idelay) begin
    if(rst_idelay == 1'b1) begin
        dat_out_1      <= 'd0; 
        dat_out_2      <= 'd0; 
        rd_state       <= 'd0; 
        dat_vld        <= 1'b0;
    end 
    else begin 
        case(adc_depth_3dly)//bit0=1:8bit, bit1=1:10bit, bit3=1:12bit, bit4=1:14bit,, bit5=1:16bit
            'd8:begin
                    if(DAT_8BIT_EN == 1'b1)begin
                        dat_out_1      <= {8'd0,i_dat};
                        dat_out_2      <= {8'd0,dat_in_1dly}; 
                        dat_vld        <= 1'b1;
                    end
                end
            'd9:begin 
                    if(DAT_9BIT_EN == 1'b1)begin
                        case(rd_state)
                            'd0:begin  
                                    dat_out_1      <= {7'd0,i_dat[0],dat_in_1dly};
                                    dat_out_2      <= {7'd0,dat_in_1dly[0],dat_in_2dly};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd1:begin  
                                    dat_out_1      <= {7'd0,i_dat[1:0],dat_in_1dly[7:1]};
                                    dat_out_2      <= {7'd0,dat_in_1dly[1:0],dat_in_2dly[7:1]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd2:begin  
                                    dat_out_1      <= {7'd0,i_dat[2:0],dat_in_1dly[7:2]};
                                    dat_out_2      <= {7'd0,dat_in_1dly[2:0],dat_in_2dly[7:2]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd3:begin  
                                    dat_out_1      <= {7'd0,i_dat[3:0],dat_in_1dly[7:3]};
                                    dat_out_2      <= {7'd0,dat_in_1dly[3:0],dat_in_2dly[7:3]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd4:begin  
                                    dat_out_1      <= {7'd0,i_dat[4:0],dat_in_1dly[7:4]};
                                    dat_out_2      <= {7'd0,dat_in_1dly[4:0],dat_in_2dly[7:4]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd5:begin  
                                    dat_out_1      <= {7'd0,i_dat[5:0],dat_in_1dly[7:5]};
                                    dat_out_2      <= {7'd0,dat_in_1dly[5:0],dat_in_2dly[7:5]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd6:begin  
                                    dat_out_1      <= {7'd0,i_dat[6:0],dat_in_1dly[7:6]};
                                    dat_out_2      <= {7'd0,dat_in_1dly[6:0],dat_in_2dly[7:6]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd7:begin  
                                    dat_out_1      <= {7'd0,i_dat[7:0],dat_in_1dly[7]};
                                    dat_out_2      <= {7'd0,dat_in_1dly[7:0],dat_in_2dly[7]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end 
                            'd8:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                            default:begin
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                        endcase 
                    end
                end
            'd10:begin 
                    if(DAT_10BIT_EN == 1'b1)begin
                        case(rd_state)
                            'd0:begin  
                                    dat_out_1      <= {6'd0,i_dat[1:0],dat_in_1dly};
                                    dat_out_2      <= {6'd0,dat_in_1dly[1:0],dat_in_2dly};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd1:begin  
                                    dat_out_1      <= {6'd0,i_dat[3:0],dat_in_1dly[7:2]};
                                    dat_out_2      <= {6'd0,dat_in_1dly[3:0],dat_in_2dly[7:2]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd2:begin  
                                    dat_out_1      <= {6'd0,i_dat[5:0],dat_in_1dly[7:4]};
                                    dat_out_2      <= {6'd0,dat_in_1dly[5:0],dat_in_2dly[7:4]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd3:begin  
                                    dat_out_1      <= {6'd0,i_dat,dat_in_1dly[7:6]};
                                    dat_out_2      <= {6'd0,dat_in_1dly,dat_in_2dly[7:6]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd4:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                            default:begin
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                        endcase 
                    end
                end
            'd11:begin 
                    if(DAT_11BIT_EN == 1'b1)begin
                        case(rd_state)
                            'd0:begin  
                                    dat_out_1      <= {5'd0,i_dat[2:0],dat_in_1dly};
                                    dat_out_2      <= {5'd0,dat_in_1dly[2:0],dat_in_2dly};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd1:begin  
                                    dat_out_1      <= {5'd0,i_dat[5:0],dat_in_1dly[7:3]};
                                    dat_out_2      <= {5'd0,dat_in_1dly[5:0],dat_in_2dly[7:3]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd2:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b0;
                                end
                            'd3:begin  
                                    dat_out_1      <= {5'd0,i_dat[0],dat_in_1dly,dat_in_2dly[7:6]};
                                    dat_out_2      <= {5'd0,dat_in_1dly[0],dat_in_2dly,dat_in_3dly[7:6]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd4:begin  
                                    dat_out_1      <= {5'd0,i_dat[3:0],dat_in_1dly[7:1]};
                                    dat_out_2      <= {5'd0,dat_in_1dly[3:0],dat_in_2dly[7:1]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd5:begin  
                                    dat_out_1      <= {5'd0,i_dat[6:0],dat_in_1dly[7:4]};
                                    dat_out_2      <= {5'd0,dat_in_1dly[6:0],dat_in_2dly[7:4]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd6:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b0;
                                end
                            'd7:begin  
                                    dat_out_1      <= {5'd0,i_dat[1:0],dat_in_1dly,dat_in_2dly[7]};
                                    dat_out_2      <= {5'd0,dat_in_1dly[1:0],dat_in_2dly,dat_in_3dly[7]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd8:begin  
                                    dat_out_1      <= {5'd0,i_dat[4:0],dat_in_1dly[7:2]};
                                    dat_out_2      <= {5'd0,dat_in_1dly[4:0],dat_in_2dly[7:2]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end 
                            'd9:begin  
                                    dat_out_1      <= {5'd0,i_dat,dat_in_1dly[7:5]};
                                    dat_out_2      <= {5'd0,dat_in_1dly,dat_in_2dly[7:5]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end 
                            'd10:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                            default:begin
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                        endcase 
                    end
                end
            'd12:begin  
                    if(DAT_12BIT_EN == 1'b1)begin
                        case(rd_state)
                            'd0:begin  
                                    dat_out_1      <= {4'd0,i_dat[3:0],dat_in_1dly};
                                    dat_out_2      <= {4'd0,dat_in_1dly[3:0],dat_in_2dly};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd1:begin  
                                    dat_out_1      <= {4'd0,i_dat,dat_in_1dly[7:4]};
                                    dat_out_2      <= {4'd0,dat_in_1dly,dat_in_2dly[7:4]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd2:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                            default:begin
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                        endcase
                    end
                end 
            'd14:begin 
                    if(DAT_14BIT_EN == 1'b1)begin
                        case(rd_state)
                            'd0:begin  
                                    dat_out_1      <= {2'd0,i_dat[5:0],dat_in_1dly};
                                    dat_out_2      <= {2'd0,dat_in_1dly[5:0],dat_in_2dly};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd1:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b0;
                                end
                            'd2:begin  
                                    dat_out_1      <= {2'd0,i_dat[3:0],dat_in_1dly,dat_in_2dly[7:6]};
                                    dat_out_2      <= {2'd0,dat_in_1dly[3:0],dat_in_2dly,dat_in_3dly[7:6]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd3:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b0;
                                end
                            'd4:begin  
                                    dat_out_1      <= {2'd0,i_dat[1:0],dat_in_1dly,dat_in_2dly[7:4]};
                                    dat_out_2      <= {2'd0,dat_in_1dly[1:0],dat_in_2dly,dat_in_3dly[7:4]};
                                    rd_state        <= rd_state + 1'b1;
                                    dat_vld         <= 1'b1;
                                end
                            'd5:begin  
                                    dat_out_1      <= {2'd0,i_dat,dat_in_1dly[7:2]};
                                    dat_out_2      <= {2'd0,dat_in_1dly,dat_in_2dly[7:2]};
                                    rd_state       <= rd_state + 1'b1;
                                    dat_vld        <= 1'b1;
                                end
                            'd6:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                            default:begin
                                    rd_state        <= 'd0;
                                    dat_vld         <= 1'b0;
                                end
                        endcase 
                    end
                end
            'd16:begin  
                    if(DAT_16BIT_EN == 1'b1)begin
                        case(rd_state)
                            'd0:begin  
                                    dat_out_1      <= {i_dat,dat_in_1dly};
                                    dat_out_2      <= {dat_in_1dly,dat_in_2dly};
                                    rd_state       <= 'd1;
                                    dat_vld        <= 1'b1;
                                end
                            'd1:begin  
                                    dat_out_1      <= dat_out_1;
                                    dat_out_2      <= dat_out_2;
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                            default:begin
                                    rd_state       <= 'd0;
                                    dat_vld        <= 1'b0;
                                end
                        endcase
                    end
                end 
            default:begin  
                        dat_out_1      <= 'd0;
                        dat_out_2      <= 'd0;
                        rd_state       <= 'd0;
                        dat_vld        <= 1'b0;
                    end
        endcase
    end
end


//--------------------------------------------------------------------------------------------------------------------
//CDC
//--------------------------------------------------------------------------------------------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        msb_first_en_dly   <= 'd0;
    end
    else begin
        msb_first_en_dly   <= {msb_first_en_dly[1:0],i_msb_first_en}; 
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        adc_depth_1dly             <= 'd0; 
        adc_depth_2dly             <= 'd0; 
        adc_depth_3dly             <= 'd0; 
    end
    else begin 
        adc_depth_1dly             <= i_adc_depth; 
        adc_depth_2dly             <= adc_depth_1dly; 
        if((DAT_8BIT_EN == 1'b0)&&(adc_depth_2dly == 'd8))begin
            adc_depth_3dly             <= 'd0; 
        end 
        else if((DAT_9BIT_EN == 1'b0)&&(adc_depth_2dly == 'd9))begin
            adc_depth_3dly             <= 'd0; 
        end 
        else if((DAT_10BIT_EN == 1'b0)&&(adc_depth_2dly == 'd10))begin
            adc_depth_3dly             <= 'd0; 
        end 
        else if((DAT_11BIT_EN == 1'b0)&&(adc_depth_2dly == 'd11))begin
            adc_depth_3dly             <= 'd0; 
        end 
        else if((DAT_12BIT_EN == 1'b0)&&(adc_depth_2dly == 'd12))begin
            adc_depth_3dly             <= 'd0; 
        end 
        else if((DAT_14BIT_EN == 1'b0)&&(adc_depth_2dly == 'd14))begin
            adc_depth_3dly             <= 'd0; 
        end 
        else if((DAT_16BIT_EN == 1'b0)&&(adc_depth_2dly == 'd16))begin
            adc_depth_3dly             <= 'd0; 
        end 
        else begin
            adc_depth_3dly             <= adc_depth_2dly; 
        end 
    end
end

endmodule


// ---- Inlined lane-8 debug implementation (unique module names) ----
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

module iser2des_debug#(
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

des_8bit_to_adcdepth_debug#(
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


// ---- Inlined lane-8 debug implementation (unique module names) ----
`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//                  --  Copyright (c) 2012 Gpixel, Technology Co. Ltd.      --
//                  --          All Rights Reserved.                        --
//------------------------------------------------------------------------------------------------------------------
//  -- Module       : train_word_align
//  -- File         : train_word_align_7serial.v
//  -- Author       : tianjiao.hou@gpixel.com 
//------------------------------------------------------------------------------------------------------------------
//  -- Function     : Bit and Word alignment control
//------------------------------------------------------------------------------------------------------------------
//  -- Hisoty       :
//  -- Ver          Date        By              Description 
//  -- 1.0          2022.05.30  Tianjiao Hou    creat 
//  -- 1.1          2022.06.15  Danyang Li      modify style 
//  -- 1.2          2022.06.29  Danyang Li      Three FSM create 
//  -- 1.4          2022.12.22  Tianjiao Hou    combine zynq and k7,define zynq or not(K7) in define  module or top module
//  -- 1.5          2023.05.24  Tianjiao Hou    as train bit&word align top, Original function in align_ctrl.v
//  -- 1.6          2023.08.23  Tianjiao Hou    1.change file name from train_word_align.v to train_word_align_7serial.v,only for 7serial
//                                              2.change the meaning represented by the i_adc_depth value
//                                              3.add 9bit and 11bit adc depth
//------------------------------------------------------------------------------------------------------------------

module train_word_align_debug#(
    parameter    DAT_8BIT_EN            = 1'b0       , // 8bit serial to parral enable 
    parameter    DAT_9BIT_EN            = 1'b0       , // 9bit serial to parral enable
    parameter    DAT_10BIT_EN           = 1'b0       , // 10bit serial to parral enable
    parameter    DAT_11BIT_EN           = 1'b0       , // 11bit serial to parral enable
    parameter    DAT_12BIT_EN           = 1'b1       , // 12bit serial to parral enable
    parameter    DAT_14BIT_EN           = 1'b0       , // 14bit serial to parral enable
    parameter    DAT_16BIT_EN           = 1'b0       , // 16bit serial to parral enable
    parameter    DAT_WIDTH              = 12         , // max data width  // 8/10/12/14/16 supported 
    parameter    TAP_MAX                = 31         , // zynq:255. K7:31, hided when generating IP core
                                                       // 1/2 real zynq taps //zynq taps from 0 to 511  //iodelay3 512tap=2ns(0.0004*512tap)
                                                       // K7 tap from 0 to 31 //idelayctrl 200M : 78ps/1tap, 300M : 52ps/1tap, 400M : 39ps/1tap
    parameter    GROUP                  = "GROUP1ST" , // IODELAY_GROUP of IDELAYE2
    parameter    REFCLK_FREQUENCY       = 300          // REFCLK_FREQUENCY of IDELAYE2, 200M:78ps/tap,300M:52ps/tap,400M:39ps/tap
)
(
    input                               clk_rx               , // frequency is 1/4 clk_ddr
    input                               clk_ddr              , // frequency is sensor output lvds clk: DDR CLK
    input                               rst_rx               , // clk_rx domain
    input                               clk_glb              , // sampling clock for output data: o_dat_vld and o_dat
    input                               rst_glb              , // clk_glb domain
    input                               i_SENSOR_DATA_P      , // sensor output P of lvds data
    input                               i_SENSOR_DATA_N      , // sensor output N of lvds data
    input            [7:0]              i_adc_depth          , // 8:8bit,9:9bit,10:10bit,11:11bit,12:12bit, 14:14bit,16:16bit 
    input                               i_cmd_start_train    , // command of start training 
    input            [DAT_WIDTH-1:0]    i_training_word      , // trainging word
    input            [6:0]              i_stable_len_half    , // half of one bit's stable length, //It is best to test according to different projects
    input                               i_msb_first_en       , // serial data(i_SENSOR_DATA) output sequence, 1:msb , 0:lsb
    output    reg                       o_dat_vld            , // o_dat valid flag, clk_glb domain
    output    reg    [DAT_WIDTH-1:0]    o_dat                , // output data, clk_glb domain
    output                              o_train_done         , // training done flag, maintain high for 16 clk_rx
    output           [7:0]              o_loc_eye_start      , // clk_rx domain. start location of i_SENSOR_DATA's stable eye
    output           [7:0]              o_loc_eye_mid        , // clk_rx domain. sample location of i_SENSOR_DATA
    output           [7:0]              o_loc_eye_end        , // clk_rx domain. end location of i_SENSOR_DATA's stable eye
    output           [7:0]              o_cnt_timeout        , // clk_rx domain. times of error align, word align error times:o_cnt_timeout[3:0], bit align error times:o_cnt_timeout[7:4]
    output           [4:0]              o_loc_word           , // clk_rx domain. times of bit moving in word align
    output                              o_loc_ok               , // clk_rx domain. align successful flag,  1:correct, 0:error
    output wire [63:0]                  o_debug               // internal deserializer/FIFO debug
    );
    
    
//------------------------------------------------------------------------------------------------------------------
//parameter
//------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

wire             rst_idelay       ;
wire             idelay_ld        ;
wire   [7:0]     idelay_valuein   ;
wire             bitslip          ;
wire             bitslip_change   ;
wire   [4:0]     rd_data_count    ;
reg              rd_en            ;
reg              rd_en_dly        ;
wire             dat_des_vld      ;
reg             dat_des_vld_dly  ;  // 鏂板锛氬欢杩焩ld淇″彿浼犵粰align_ctrl
wire   [15:0]    dat_des          ;
wire   [15:0]    fifo_dout        ;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
// Internal debug is intentionally exposed for one selected lane (lane 8 at the
// training_top wrapper). The low fields are the raw ISERDES word and its valid
// flag; the remaining fields show the CDC FIFO read timing and training controls.
assign o_debug = {
    7'd0, rst_glb, rst_rx, i_cmd_start_train, o_loc_ok, rst_idelay, idelay_ld,
    idelay_valuein, bitslip_change, bitslip, rd_data_count, rd_en_dly, rd_en,
    dat_des_vld_dly, dat_des_vld, fifo_dout, dat_des
};

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------
always @(posedge clk_glb or posedge rst_glb) begin
    if(rst_glb == 1'b1) begin   
        rd_en   <= 1'b0;
    end
    else begin
        if(rd_data_count > 'd4) begin   
            rd_en   <= 1'b1;
        end
        else begin
            rd_en   <= 1'b0;
        end
    end
end

always @(posedge clk_glb or posedge rst_glb) begin
    if(rst_glb == 1) begin
        o_dat        <= 'd0;
        rd_en_dly    <= 1'b0;
        o_dat_vld    <= 1'b0;
    end
    else begin
        o_dat        <= fifo_dout[DAT_WIDTH-1:0];
        rd_en_dly    <= rd_en;
        o_dat_vld    <= rd_en_dly;
    end
end

always @(posedge clk_rx) begin
    dat_des_vld_dly <= dat_des_vld;  // 鏂板锛氬悓姝ュ埌clk_rx鏃堕挓鍩燂紝鍖归厤align_ctrl閲囨牱寤惰繜
end

//--------------------------------------------------------------------------------------------------------------------
//Instance
//--------------------------------------------------------------------------------------------------------------------

//K7 and ZYNQ can share this module
align_ctrl_debug#(
    .DAT_WIDTH               (DAT_WIDTH            ),
    .TAP_MAX                 (TAP_MAX              )
)    u_align_ctrl(
    .clk                     (clk_rx               ),
    .rst                     (rst_rx               ),
    .i_adc_depth             (i_adc_depth          ),
    .i_cmd_start_train       (i_cmd_start_train    ),
    .i_training_word         (i_training_word      ),
    .i_stable_len_half       (i_stable_len_half    ),
    .i_dat                   (dat_des              ),
    .i_dat_vld               (dat_des_vld_dly      ),
    .o_rst_idelay            (rst_idelay           ),
    .o_idelay_ld             (idelay_ld            ),
    .o_idelay_valuein        (idelay_valuein       ),
    .o_bitslip               (bitslip              ),
    .o_bitslip_change        (bitslip_change       ),
    .o_train_done            (o_train_done         ),
    .o_loc_eye_start         (o_loc_eye_start      ),
    .o_loc_eye_mid           (o_loc_eye_mid        ),
    .o_loc_eye_end           (o_loc_eye_end        ),
    .o_cnt_timeout           (o_cnt_timeout        ),
    .o_loc_word              (o_loc_word           ),
    .o_loc_ok                (o_loc_ok             )
);


//K7 and ZYNQ can share this module io;
//des_8bit_to_adcdepth under iser2des is same, but others in iser2des need to be modified(K7: IDELAYE2 and ISERDESE2, ZYNQ : IDELAYE3 and ISERDESE3)
iser2des_debug#(
    .GROUP                    (GROUP            ), 
    .REFCLK_FREQUENCY         (REFCLK_FREQUENCY ), 
    .DAT_8BIT_EN              (DAT_8BIT_EN      ), 
    .DAT_9BIT_EN              (DAT_9BIT_EN     ),  
    .DAT_10BIT_EN             (DAT_10BIT_EN    ),  
    .DAT_11BIT_EN             (DAT_11BIT_EN    ),  
    .DAT_12BIT_EN             (DAT_12BIT_EN     ), 
    .DAT_14BIT_EN             (DAT_14BIT_EN     ),
    .DAT_16BIT_EN             (DAT_16BIT_EN     )
)    u_iser2des(                               
    .clk_rx                   (clk_rx           ),
    .clk_ddr                  (clk_ddr          ),
    .rst_rx                   (rst_rx           ),
    .i_SENSOR_DATA_P          (i_SENSOR_DATA_P  ),
    .i_SENSOR_DATA_N          (i_SENSOR_DATA_N  ),
    .i_adc_depth              (i_adc_depth      ),
    .i_rst_idelay             (rst_idelay       ),
    .i_idelay_ld              (idelay_ld        ),
    .i_idelay_valuein         (idelay_valuein   ),
    .i_bitslip                (bitslip          ),
    .i_bitslip_change         (bitslip_change   ),
    .i_msb_first_en           (i_msb_first_en   ),
    .o_dat_vld                (dat_des_vld      ),
    .o_dat                    (dat_des          )
);

// Independent Clocks Distributed RAM   
// Standard FIFO
// Write/Read width: 16
// Write/Read depth: 16
// ResetPin, Enable Reset Synchronization : select
// Valid Flag : select
// Read Data Count : select 
fifo_generator_0    u_fifo_w16d32_icdr_st(
    .rst              (rst_rx          ),
    .wr_clk           (clk_rx          ),
    .rd_clk           (clk_glb         ),
    .din              (dat_des         ),
    .wr_en            (dat_des_vld     ),
    .rd_en            (rd_en           ),
    .dout             (fifo_dout       ),
    .full             (                ),
    .empty            (                ),
    .rd_data_count    (rd_data_count   ) 
); 

endmodule

