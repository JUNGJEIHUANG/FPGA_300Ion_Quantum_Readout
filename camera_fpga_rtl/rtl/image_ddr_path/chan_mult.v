`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : chan_mult
// --File      : chan_mult.v
// --Author    : weihan.li@gpixel.com
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : Channel multiplexing operation
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2022.09.13   Weihan.li       creat
// --1.1       2025.02.13   Guangcai Ding   Modify : CC2405
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module chan_mult #(
    parameter    CHAN_NUM           = 8    ,
    parameter    DAT_WIDTH          = 12   ,
    parameter    BRAM_DEPTH         = 4096 ,
    parameter    BRAM_ADDR_WIDTH    = 12                                          // Minimum number of a channel output data
)(
    input                                                  clk_rx            ,    // Data trained accompanying clock
    input                                                  rst_rx            ,    // Data trained accompanying clock reset signal, high active
    input                                                  clk_tx            ,    // Data trained accompanying clock
    input                                                  i_cmd_frame_req   ,    // Reset before each frame request
    input                                                  i_din_vld         ,    // Data trained's valid signal
    input            [CHAN_NUM*DAT_WIDTH-1:0]              i_din             ,    // Data trained input
    input                                                  i_flip_h_en       ,
    input            [4:0]                                 i_n_mux           ,    // Channel multiplexing operation channel's valid number
    input            [13:0]                                i_w_mux           ,    // Channel multiplexing operation channel's output valid length
    input            [15:0]                                i_lvalout_len     , 
    input            [CHAN_NUM-1:0]                        i_bram_rd_en      ,    // BRAM read enable
    input            [CHAN_NUM*(BRAM_ADDR_WIDTH-2)-1:0]    i_bram_rd_addr    ,    // BRAM read address	
    output    reg                                          o_pipe_flag       ,    // Pipeline operation sign
    output    reg                                          o_row1_done       ,    // Write into BRAM one row data done
    output    reg    [CHAN_NUM-1:0]                        o_chan_vld        ,    // chan valid for training.v 
    output           [CHAN_NUM*4*DAT_WIDTH-1:0]            o_dout                 // Output data
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

localparam    s_init_idle        = 'd0 ,
              s_init_calc        = 'd1 ,
              s_init_assign      = 'd2 ,
              s_init_end         = 'd3 ;

localparam    BRAM_DEPTH_HALF    = BRAM_DEPTH / 2 ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

reg     [3:0]                    cmd_frame_req_dly          ;
reg     [4:0]                    cnt_frame_req              ;
reg                              frame_rst                  ;
reg     [3:0]                    frame_rst_dly              ;
reg                              flip_h_en_1dly             ;
reg                              flip_h_en_2dly             ;
reg                              flip_h_en_3dly             ;
reg                              flip_h_en_calc             ;
reg                              flip_h_en                  ;
reg     [15:0]                   lvalout_len_1dly           ;
reg     [15:0]                   lvalout_len_2dly           ;
reg     [15:0]                   lvalout_len_3dly           ;
reg     [15:0]                   lvalout_len_calc           ;
reg     [15:0]                   lvalout_len                ;
reg     [4:0]                    n_mux_1dly                 ;
reg     [4:0]                    n_mux_2dly                 ;
reg     [4:0]                    n_mux_3dly                 ;
reg     [4:0]                    n_mux_calc                 ;
reg     [4:0]                    n_mux                      ;
reg     [13:0]                   w_mux_1dly                 ;
reg     [13:0]                   w_mux_2dly                 ;
reg     [13:0]                   w_mux_3dly                 ;
reg     [13:0]                   w_mux_calc                 ;
reg     [13:0]                   w_mux                      ;
reg     [15:0]                   dat_unvld_len              ;
reg     [15:0]                   cnt_unvld_len              ;
reg     [12:0]                   cnt_rst_calc               ;
reg     [3:0]                    rst_calc_dly               ;
reg                              rst_calc                   ;
reg     [15:0]                   wr_addr_end_even           ;
reg     [15:0]                   wr_addr_end_odd            ;
reg     [15:0]                   wr_addr_start_even         ;
reg     [15:0]                   wr_addr_start_odd          ;
wire    [DAT_WIDTH-1:0]          dat_array[CHAN_NUM-1:0]    ;    // Input data buffer into 2D array format
wire    [BRAM_ADDR_WIDTH-3:0]    bram_rd_addr[CHAN_NUM-1:0] ;    // Input data buffer into 2D array format
reg     [1:0]                    cs_init,ns_init            ;
reg                              state_flag_next            ;    // FSM state change to next state sign
reg                              state_flag_back            ;    // FSM state change to previous state sign 
reg     [4:0]                    cnt_nMux                   ;
reg     [13:0]                   cnt_wMux                   ;
reg                              init_flag                  ;    // Mapping initial operation
reg                              init_flag_1dly             ;
reg     [CHAN_NUM-1:0]           chan_vld_calc              ;    // Sensor output data valid channel sign buffer
reg     [BRAM_ADDR_WIDTH-1:0]    wr_addr_calc[CHAN_NUM-1:0] ;    // Data mapping beginning address 
reg     [5:0]                    dat_chan_calc[CHAN_NUM-1:0];    // Data mapping beginning address 
reg     [CHAN_NUM-1:0]           chan_vld_buf               ;    // Sensor output data valid channel sign buffer
reg     [BRAM_ADDR_WIDTH-1:0]    wr_addr_buf[CHAN_NUM-1:0]  ;    // Data mapping beginning address buffer
reg     [5:0]                    dat_chan_buf[CHAN_NUM-1:0] ;    // Data mapping beginning address buffer
reg     [CHAN_NUM-1:0]           chan_vld                   ;    // Sensor output data valid channel sign
reg     [BRAM_ADDR_WIDTH-1:0]    wr_addr_init[CHAN_NUM-1:0] ;    // Data mapping beginning address
reg     [5:0]                    dat_chan_init[CHAN_NUM-1:0];    // Data mapping beginning address
wire    [15:0]                   divisor                    ;    // Divider IP ports
wire    [15:0]                   dividend                   ;
reg                              div_val_in                 ;
wire    [15:0]                   div_dat_out                ;
wire    [15:0]                   remaind_out                ;
wire                             div_val_out                ;
reg     [15:0]                   div_dat_out_pre            ;
reg     [15:0]                   remaind_out_pre            ;
reg     [CHAN_NUM-1:0]           bram_val                   ;    // BRAM actived sign
reg     [CHAN_NUM-1:0]           bram_wr_en                 ;    // BRAM write enable
reg     [BRAM_ADDR_WIDTH-1:0]    wr_addr_pre[CHAN_NUM-1:0]  ;
reg     [BRAM_ADDR_WIDTH-1:0]    bram_wr_addr[CHAN_NUM-1:0] ;    // BRAM write address
reg     [DAT_WIDTH-1:0]          bram_dat_in[CHAN_NUM-1:0]  ;    // BRAM write in data
reg     [5:0]                    dat_chan[CHAN_NUM-1:0]     ;    // Sensor output channel mapping number
reg                              row1_done_1dly             ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

genvar i;
generate
    for(i = 0; i < CHAN_NUM; i = i + 1) begin    // Input data buffer into 2D array
        assign    dat_array[i]       = i_din[(i+1)*DAT_WIDTH-1:i*DAT_WIDTH];
        assign    bram_rd_addr[i]    = i_bram_rd_addr[(i+1)*(BRAM_ADDR_WIDTH-2)-1:i*(BRAM_ADDR_WIDTH-2)];
    end
endgenerate

assign    divisor  = lvalout_len_3dly;         // Divisor assign

assign    dividend = cnt_nMux * w_mux_calc;    // Dividend assign

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// frame_rst
always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1)begin 
        cnt_frame_req    <= 'd0;
        frame_rst        <= 1'b1;
    end
    else begin  
        if(cmd_frame_req_dly[3:2] == 2'b01)begin 
            cnt_frame_req      <= 'd15;
            frame_rst          <= 1'b1;
        end
        else begin
            if(cnt_frame_req == 'd0)begin
                frame_rst          <= 1'b0;
            end
            else begin
                cnt_frame_req      <= cnt_frame_req - 1'b1;
            end
        end
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1)begin 
        frame_rst_dly    <= 'd0; 
    end
    else begin
        frame_rst_dly    <= {frame_rst_dly[2:0],frame_rst};
    end
end

// calculate
always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        n_mux_calc          <= 'd0 ;
        w_mux_calc          <= 'd0 ;
        lvalout_len_calc    <= 'd0 ;
        flip_h_en_calc      <= 1'b0;
        rst_calc            <= 1'b1;
        cnt_rst_calc        <= 'd0 ;
    end
    else begin
        if((n_mux_calc != n_mux_3dly)||(w_mux_calc != w_mux_3dly)||(flip_h_en_calc != flip_h_en_3dly)||(lvalout_len_calc != lvalout_len_3dly))begin
            rst_calc            <= 1'b1            ;
            n_mux_calc          <= n_mux_3dly      ;
            w_mux_calc          <= w_mux_3dly      ;
            lvalout_len_calc    <= lvalout_len_3dly;
            flip_h_en_calc      <= flip_h_en_3dly  ;
            cnt_rst_calc        <= 'd31            ;    // must larger than 16, because div_val_out is 16clk delay after div_val_in
        end
        else begin
            if(cnt_rst_calc == 'd0)begin
                rst_calc        <= 1'b0;
            end
            else begin
                cnt_rst_calc    <= cnt_rst_calc - 1'b1;
            end
        end
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1)begin 
        rst_calc_dly    <= 'hF; 
    end
    else begin
        rst_calc_dly    <= {rst_calc_dly[2:0],rst_calc};
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        flip_h_en       <= 1'b0;
        n_mux           <= 'd0;
        w_mux           <= 'd0;
        lvalout_len     <= 'd0;
    end
    else begin
        if(frame_rst_dly[3:2] == 2'b01) begin 
            flip_h_en       <= flip_h_en_calc ; 
            n_mux           <= n_mux_calc      ;
            w_mux           <= w_mux_calc      ;
            lvalout_len     <= lvalout_len_calc;
        end
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        wr_addr_end_even               <= 'd0;
        wr_addr_end_odd                <= 'd0;
        wr_addr_start_even             <= 'd0;
        wr_addr_start_odd              <= 'd0;
        dat_unvld_len                  <= 'd0;
    end
    else begin 
        if(flip_h_en == 1'b1) begin
            wr_addr_end_even               <= 'd0;
            wr_addr_end_odd                <= BRAM_DEPTH_HALF;
            wr_addr_start_even             <= lvalout_len - 1'b1;
            wr_addr_start_odd              <= BRAM_DEPTH_HALF + lvalout_len - 1'b1;
            dat_unvld_len                  <= w_mux * n_mux - CHAN_NUM * lvalout_len;
        end
        else begin
            wr_addr_end_even               <= lvalout_len - 1'b1;
            wr_addr_end_odd                <= BRAM_DEPTH_HALF + lvalout_len - 1'b1;
            wr_addr_start_even             <= 'd0;
            wr_addr_start_odd              <= BRAM_DEPTH_HALF;
            dat_unvld_len                  <= 'd0;
        end 
    end
end

// FSM init operation

always @(posedge clk_rx or posedge rst_calc) begin
    if(rst_calc == 1'b1) begin
        cs_init    <= s_init_idle;
    end
    else begin
        cs_init    <= ns_init;
    end
end

always @(*) begin
    case(cs_init)
        s_init_idle:    begin
                            if(state_flag_next == 1'b1) begin
                                ns_init    = s_init_calc;
                            end
                            else begin
                                ns_init    = s_init_idle;
                            end
                        end
        s_init_calc:    begin
                            if(state_flag_next == 1'b1) begin
                                ns_init    = s_init_assign;
                            end
                            else begin
                                ns_init    = s_init_calc;
                            end
                        end  
        s_init_assign:  begin
                            if(state_flag_next == 1'b1) begin
                                ns_init    = s_init_idle;
                            end
                            else if(state_flag_back == 1'b1) begin
                                ns_init    = s_init_calc;
                            end
                            else begin
                                ns_init    = s_init_assign;
                            end
                        end
        s_init_end:     ns_init    = s_init_end;
    endcase
end

always @(posedge clk_rx or posedge rst_calc) begin
    if(rst_calc== 1'b1) begin
        cnt_nMux                <= 'd0;
        chan_vld_calc           <= 'd0;
        div_dat_out_pre         <= 'd0;
        remaind_out_pre         <= 'd0;
        init_flag               <= 1'b0;
        div_val_in              <= 1'b0;
        state_flag_next         <= 1'b0;
        state_flag_back         <= 1'b0;
    end
    else begin
        case(cs_init)
            s_init_idle:    begin
                                if(state_flag_next == 1'b0) begin    // Calculating
                                    div_dat_out_pre         <= CHAN_NUM - 1'b1;
                                    remaind_out_pre         <= lvalout_len_calc;
                                    div_val_in              <= 1'b0;
                                    if(flip_h_en_calc== 1'b1) begin
                                        cnt_nMux                <= n_mux_calc - 1'b1;
                                    end
                                    else begin
                                        cnt_nMux                <= 'd0;
                                    end
                                    if(rst_calc_dly[3:2] == 2'b10) begin 
                                        chan_vld_calc           <= 'd0;
                                        init_flag               <= 1'b0;
                                        state_flag_next         <= 1'b1;
                                    end 
                                end
                                else begin 
                                    state_flag_next         <= 1'b0;
                                end
                            end
            s_init_calc:    begin
                                if(state_flag_next == 1'b0) begin    // Calculating
                                    div_val_in              <= 1'b1;
                                    state_flag_next         <= 1'b1;
                                end
                                else begin
                                    div_val_in              <= 1'b0;
                                    state_flag_next         <= 1'b0;
                                end
                            end
            s_init_assign:  begin
                                if((state_flag_next == 1'b0) && (state_flag_back == 1'b0)) begin
                                    if(div_val_out == 1'b1) begin
                                        if(remaind_out == 'd0) begin
                                            remaind_out_pre  <= lvalout_len_calc;
                                            div_dat_out_pre  <= div_dat_out - 1'b1;
                                        end
                                        else begin
                                            remaind_out_pre  <= remaind_out;
                                            div_dat_out_pre  <= div_dat_out;
                                        end
                                        if(div_dat_out < CHAN_NUM) begin    // Assigning calculator's result
                                            if(flip_h_en_calc == 1'b1) begin
                                                chan_vld_calc[div_dat_out_pre]     <= 1'b1;
                                                dat_chan_calc[div_dat_out_pre]     <= div_dat_out;
                                                wr_addr_calc[div_dat_out_pre]      <= remaind_out_pre - 1'b1;
                                            end
                                            else begin
                                                chan_vld_calc[div_dat_out]         <= 1'b1;
                                                dat_chan_calc[div_dat_out]         <= div_dat_out;
                                                wr_addr_calc[div_dat_out]          <= remaind_out;
                                            end
                                        end

                                        if(flip_h_en_calc == 1'b1) begin
                                            if(cnt_nMux == 'd0) begin    // When all channel calculated done, back to IDLE state
                                                state_flag_next         <= 1'b1;
                                            end
                                            else begin    // Preparing to calculate next channel
                                                cnt_nMux                <= cnt_nMux - 1'b1;
                                                state_flag_back         <= 1'b1;
                                            end
                                        end
                                        else begin 
                                            if(cnt_nMux >= n_mux_calc - 1'b1) begin    // When all channel calculated done, back to IDLE state
                                                state_flag_next         <= 1'b1;
                                            end
                                            else begin    // Preparing to calculate next channel
                                                cnt_nMux                <= cnt_nMux + 1'b1;
                                                state_flag_back         <= 1'b1;
                                            end
                                        end
                                    end
                                end
                                else if(state_flag_next == 1'b1) begin
                                    init_flag               <= 1'b1;
                                    state_flag_next         <= 1'b0;
                                end
                                else begin
                                    state_flag_back         <= 1'b0;
                                end
                            end
        endcase
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        init_flag_1dly    <= 1'b0;
    end
    else begin
        init_flag_1dly    <= init_flag;
    end
end

// o_chan_vld : for training.v
always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        o_chan_vld    <= 'hFFFF;
    end
    else begin
        if((init_flag == 1'b1) && (init_flag_1dly == 1'b0)) begin
            o_chan_vld    <= chan_vld_calc; 
        end
    end
end

// o_row1_done : Counting input lval , Counting a row length
always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        cnt_wMux                  <= 'd0;
        o_row1_done               <= 1'b0;
        o_pipe_flag               <= 1'b0;
    end
    else if(frame_rst == 1'b1) begin
        cnt_wMux                  <= 'd0;
        o_row1_done               <= 1'b1;
        o_pipe_flag               <= 1'b0;
    end
    else begin
        if(i_din_vld == 1'b1) begin
            if(cnt_wMux < w_mux - 1) begin
                cnt_wMux        <= cnt_wMux + 1'b1;
                o_row1_done     <= 1'b0;
            end
            else begin
                cnt_wMux       <= 'd0;
                o_row1_done    <= 1'b1;
                o_pipe_flag    <= ~o_pipe_flag;
            end
        end
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        row1_done_1dly          <= 1'b0;
    end
    else if(frame_rst == 1'b1) begin
        row1_done_1dly          <= 1'b0;
    end
    else begin
        row1_done_1dly          <= o_row1_done;
    end
end

genvar j;
generate
    for(j = 0; j < CHAN_NUM; j = j + 1) begin
        always @(posedge clk_rx or posedge rst_rx) begin
            if(rst_rx == 1'b1) begin
                chan_vld_buf[j]                    <= 1'b0;
                wr_addr_buf[j]                     <= 'd0;
                dat_chan_buf[j]                    <= 'd0;
            end
            else begin
                if((init_flag == 1'b1) && (init_flag_1dly == 1'b0)) begin
                    chan_vld_buf[j]                <= chan_vld_calc[j];
                    wr_addr_buf[j]                 <= wr_addr_calc[j] ;
                    dat_chan_buf[j]                <= dat_chan_calc[j];
                end
            end
        end
        
        always @(posedge clk_rx or posedge rst_rx) begin
            if(rst_rx == 1'b1) begin
                chan_vld[j]                        <= 1'b0;
                wr_addr_init[j]                    <= 'd0;
                dat_chan_init[j]                   <= 'd0;
            end
            else begin
                if(frame_rst_dly[3:2] == 2'b01) begin
                    chan_vld[j]                    <= chan_vld_buf[j];
                    wr_addr_init[j]                <= wr_addr_buf[j] ;
                    dat_chan_init[j]               <= dat_chan_buf[j];
                end
            end
        end
        
        always @(posedge clk_rx or posedge rst_rx) begin
            if(rst_rx == 1'b1) begin
                bram_val[j]                        <= 1'b0;
                bram_wr_en[j]                      <= 1'b0;
                wr_addr_pre[j]                     <= 'd0;
                bram_wr_addr[j]                    <= 'd0;
                bram_dat_in[j]                     <= 'd0;
                dat_chan[j]                        <= 'd0;
                if(j == 7)begin
                    cnt_unvld_len                  <= 'd0;
                end
            end
            else if((row1_done_1dly == 1'b0) && (o_row1_done == 1'b1)) begin    // Assign initial value operation, run when initial registers value changed, wrote a row finished or a frame reset sign comes
                dat_chan[j]                        <= dat_chan_init[j];
                bram_wr_en[j]                      <= 1'b0;
                bram_val[j]                        <= chan_vld[j];
                if(j == 7)begin
                    cnt_unvld_len                  <= 'd0;
                end

                if(chan_vld[j] == 1'b1) begin    // When BRAM actived
                    if(o_pipe_flag == 1'b0) begin    // Assign initial address
                        wr_addr_pre[j]             <= wr_addr_init[j];
                    end
                    else begin
                        wr_addr_pre[j]             <= wr_addr_init[j] + BRAM_DEPTH_HALF;
                    end
                end
                else begin
                    if(o_pipe_flag == 1'b0) begin    // Unactive BRAM assign 0 or BRAM_DEPTH_HALF
                        wr_addr_pre[j]             <= wr_addr_start_even;
                    end
                    else begin
                        wr_addr_pre[j]             <= wr_addr_start_odd;
                    end
                end
            end
            else begin
                if(bram_val[j] == 1'b1) begin    // When BRAM actived
                    if((bram_wr_en[j] == 1'b1) && ((bram_wr_addr[j] == wr_addr_end_even) || (bram_wr_addr[j] == wr_addr_end_odd))) begin    // When write in address achieved last address
                        bram_wr_en[j]              <= 1'b0;
                        bram_dat_in[j]             <= 'd0;
                        bram_val[j]                <= 1'b0;    // BRAM setting to unactived
                        
                        if(o_pipe_flag == 1'b0) begin    // Address setting to 0 or BRAM_DEPTH_HALF
                            wr_addr_pre[j]         <= wr_addr_start_even;
                            bram_wr_addr[j]        <= wr_addr_start_even;
                        end
                        else begin
                            wr_addr_pre[j]         <= wr_addr_start_odd;
                            bram_wr_addr[j]        <= wr_addr_start_odd;
                        end
                    end
                    else if(i_din_vld == 1'b1) begin    // Writing into BRAM
                        if(j == 7)begin
                            if(cnt_unvld_len < dat_unvld_len)begin
                                bram_wr_en[j]      <= 1'b0;
                                cnt_unvld_len      <= cnt_unvld_len + 1'b1;
                            end
                            else begin
                                bram_wr_en[j]      <= 1'b1;
                                wr_addr_pre[j]     <= wr_addr_pre[j] + 1'b1 - 2 * flip_h_en;
                                bram_wr_addr[j]    <= wr_addr_pre[j];
                            end
                        end
                        else begin
                            bram_wr_en[j]          <= 1'b1;
                            wr_addr_pre[j]         <= wr_addr_pre[j] + 1'b1 - 2 * flip_h_en;
                            bram_wr_addr[j]        <= wr_addr_pre[j];
                        end
                        bram_dat_in[j]             <= dat_array[dat_chan[j]];
                    end
                    else begin
                        bram_wr_en[j]              <= 1'b0;
                    end
                end
                else begin 
                    if(flip_h_en == 1'b1) begin
                        if(j < CHAN_NUM - 1) begin    // When BRAM unactived
                           if(((wr_addr_pre[j+1] == wr_addr_end_even) || (wr_addr_pre[j+1] == wr_addr_end_odd)) && (cnt_wMux < w_mux - 2) && (i_din_vld == 1'b1)) begin    // When write operation unfinished but previous BRAM write finished
                               dat_chan[j]         <= dat_chan[j+1];    // Channel mapping
                               bram_val[j]         <= 1'b1;    // Change BRAM to actived
                           end
                        end
                    end
                    else begin
                        if(j > 0) begin    // When BRAM unactived
                           if(((wr_addr_pre[j-1] == wr_addr_end_even) || (wr_addr_pre[j-1] == wr_addr_end_odd)) && (cnt_wMux < w_mux - 2) && (i_din_vld == 1'b1)) begin    // When write operation unfinished but previous BRAM write finished
                               dat_chan[j]         <= dat_chan[j-1];    // Channel mapping
                               bram_val[j]         <= 1'b1;    // Change BRAM to actived
                           end
                        end
                    end
                end
            end
        end
    end
endgenerate

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Algorithm Type        Radix2
// Operand Sign          Unsigned
// Dividend Width        16
// Dividend TLAST        No
// Dividend TUSER        No
// Divisor Width         16
// Divisor TLAST         No
// Divisor TUSER         No
// Raminder Type         Remainder
// Clocks per Division   1
// Flow Control          Non Blocking
// Latency Configuration Automatic
div_16_16_Radix2    div_init_value
(
    .aclk                   (clk_rx                    ),    // input wire aclk
    .s_axis_divisor_tvalid  (div_val_in                ),    // input wire s_axis_divisor_tvalid
    .s_axis_divisor_tdata   (divisor                   ),    // input wire [15 : 0] s_axis_divisor_tdata
    .s_axis_dividend_tvalid (div_val_in                ),    // input wire s_axis_dividend_tvalid
    .s_axis_dividend_tdata  (dividend                  ),    // input wire [15 : 0] s_axis_dividend_tdata
    .m_axis_dout_tvalid     (div_val_out               ),    // output wire m_axis_dout_tvalid
    .m_axis_dout_tdata      ({div_dat_out, remaind_out})     // output wire [31 : 0] m_axis_dout_tdata
);

genvar k;
generate
    for(k = 0; k < CHAN_NUM; k = k + 1) begin: loop_BRAM
        // Interface Type                   Native
        // Memory Type                      Simple Dual Port RAM
        // Write Width                      12
        // Write Depth                      1024
        // Write Operation Type             No Change
        // Write Enable Port Type           Always Enable
        // Read Width                       24
        // Read Depth                       512
        // Read Operation Type              Write First
        // Read Enable Port Type            Use ENB Pin
        // Read Primitives Output Register  No
        // Read Core Output Register        No
        // Read RSTB Pin                    No
        bram_w12d4096_w48d1024_sdpr    chan_multiplex
        (
            .clka   (clk_rx                                   ),
            .wea    (bram_wr_en[k]                            ),
            .addra  (bram_wr_addr[k]                          ),
            .dina   (bram_dat_in[k]                           ),
            .clkb   (clk_tx                                   ),
            .enb    (i_bram_rd_en[k]                          ),
            .addrb  (bram_rd_addr[k]                          ),
            .doutb  (o_dout[(k+1)*DAT_WIDTH*4-1:k*DAT_WIDTH*4])
        );
    end
endgenerate

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1)begin 
        cmd_frame_req_dly    <= 'd0; 
    end
    else begin
        cmd_frame_req_dly    <= {cmd_frame_req_dly[2:0],i_cmd_frame_req};
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        flip_h_en_1dly    <= 1'b0;
        flip_h_en_2dly    <= 1'b0;
        flip_h_en_3dly    <= 1'b0;
    end
    else begin
        flip_h_en_1dly    <= i_flip_h_en   ;
        flip_h_en_2dly    <= flip_h_en_1dly;
        flip_h_en_3dly    <= flip_h_en_2dly; 
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        w_mux_1dly    <= 'd0;
        w_mux_2dly    <= 'd0;
        w_mux_3dly    <= 'd0;
    end
    else begin
        w_mux_1dly    <= i_w_mux   ;
        w_mux_2dly    <= w_mux_1dly;
        w_mux_3dly    <= w_mux_2dly; 
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        n_mux_1dly    <= 'd0;
        n_mux_2dly    <= 'd0;
        n_mux_3dly    <= 'd0;
    end
    else begin
        n_mux_1dly    <= i_n_mux   ;
        n_mux_2dly    <= n_mux_1dly;
        n_mux_3dly    <= n_mux_2dly;
    end
end

always @(posedge clk_rx or posedge rst_rx) begin
    if(rst_rx == 1'b1) begin
        lvalout_len_1dly    <= 'd0;
        lvalout_len_2dly    <= 'd0;
        lvalout_len_3dly    <= 'd0;
    end
    else begin
        lvalout_len_1dly    <= i_lvalout_len     ;
        lvalout_len_2dly    <= lvalout_len_1dly  ;
        lvalout_len_3dly    <= lvalout_len_2dly  ;
    end
end

endmodule
