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
