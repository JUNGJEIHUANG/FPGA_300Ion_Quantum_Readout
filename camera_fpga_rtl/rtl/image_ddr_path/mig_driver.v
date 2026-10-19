`timescale 1ns / 1ps
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : mig_driver
// --File      : mig_driver.v
// --Author    : weihan.li@gpixel.com
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : MIG driver, write in and read out operation
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2023.01.13   Weihan.li       creat
// --1.1       2023.04.19   Weihan.li       Delet frame request reset signal
// --1.1       2023.05.10   Weihan.li       Add reset signals
// --1.2       2023.05.17   Weihan.li       Remove read command
// --1.2       2024.05.04   Guangcai Ding   modify code style
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module mig_driver
(
    input                       clk                    ,
    input                       rst                    ,
    input                       i_cmd_frame_req        ,
    input             [2:0]     i_hdr_mode_en          ,
    input             [2:0]     i_image_type           ,
    input             [15:0]    i_lval_len             ,
    input             [15:0]    i_win_row_len          ,
    input             [15:0]    i_frame_num            ,
    input                       i_fifo2ddr_empty       ,    // FIFO(before DDR)  ports
    input             [8:0]     i_fifo2ddr_rd_cnt      ,
    output                      o_fifo2ddr_ren         ,
    output     reg              o_ddr_wr_done          ,
    output                      o_dbg_rd_req_ui, // Native UI-domain row-request edge
    input                       i_ddr_rd_req           ,    // Read & transmitted out a row finished signals----NO
    input                       i_init_calib_complete  ,
    input                       i_app_rdy              ,
    input                       i_app_wdf_rdy          ,
    output     reg    [28:0]    o_app_addr             ,
    output     reg    [2:0]     o_app_cmd              ,
    output                      o_app_en
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

localparam    s_mig_init    = 'd0,
              s_idle        = 'd1,
              s_wr          = 'd2,
              s_wr_wait     = 'd3,
              s_rd          = 'd4,
              s_rd_wait     = 'd5;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

(*use_dsp="yes" *)wire    [31:0]    lval_num             ;
(*use_dsp="yes" *)wire    [19:0]    addr_jump_row        ;    // Jump a row
(*use_dsp="yes" *)wire    [35:0]    addr_jump_frame      ;    // Jump a frame
(*use_dsp="yes" *)wire    [35:0]    addr_jump_frame_half ;    // Jump half frame
reg    [3:0]     cmd_frame_req_dly       ;
reg    [4:0]     cnt_frame_rst           ;
reg              frame_rst               ;
reg    [2:0]     image_type_1dly         ;
reg    [2:0]     image_type_2dly         ;
reg    [2:0]     image_type_3dly         ;
reg    [15:0]    lval_len_1dly           ;
reg    [15:0]    lval_len_2dly           ;
reg    [15:0]    lval_len_3dly           ;
reg    [2:0]     hdr_mode_en_1dly        ;
reg    [2:0]     hdr_mode_en_2dly        ;
reg    [2:0]     hdr_mode_en_3dly        ;
reg    [15:0]    win_row_len_1dly        ;
reg    [15:0]    win_row_len_2dly        ;
reg    [15:0]    win_row_len_3dly        ;
reg    [15:0]    win_row_len_3dly_half   ;
reg    [15:0]    frame_num_1dly          ;
reg    [15:0]    frame_num_2dly          ;
reg    [15:0]    frame_num_3dly          ;
(*ASYNC_REG="true" *)reg   [3:0]     ddr_rd_req_dly ;
assign o_dbg_rd_req_ui = (ddr_rd_req_dly[3:2] == 2'b01);
reg    [33:0]    real_lval_num           ;    // Totally read out data numbers
reg    [2:0]     cs_mig,ns_mig           ;
reg              state_flag_back         ;    // FSM state change flag
reg              state_flag_next         ;    // FSM state change flag
reg              rd_org_done             ;
reg              rd_div_done             ;
reg              rd_sf1_done             ;
reg              flag_wr_odd             ;
reg              mig_wr_flag             ;    // MIG initial done flag
reg              app_en_tmp              ;
reg    [29:0]    addr_row_start          ;
reg    [15:0]    cnt_app_in              ;
reg    [15:0]    cnt_app_out             ;
reg    [33:0]    cnt_total_lval          ;
reg    [15:0]    cnt_row                 ;
reg    [15:0]    cnt_frame               ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

assign    lval_num                = win_row_len_3dly * frame_num_3dly;
assign    addr_jump_row           = lval_len_3dly * 4'd8;
assign    addr_jump_frame         = lval_len_3dly * 4'd8 * (win_row_len_3dly - 1'b1);
assign    addr_jump_frame_half    = lval_len_3dly * 4'd8 * (win_row_len_3dly_half - 1'b1); // for HDR

assign    o_fifo2ddr_ren          = mig_wr_flag & ~i_fifo2ddr_empty & i_app_rdy & i_app_wdf_rdy;

assign    o_app_en                = (o_app_cmd == 'd0) ? o_fifo2ddr_ren : (i_app_rdy & app_en_tmp);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk  or posedge rst) begin
    if(rst == 1'b1)begin
        cnt_frame_rst    <= 'd0;
        frame_rst        <= 1'b1;
    end
    else begin
        if(cmd_frame_req_dly[3:2] == 2'b01) begin
            cnt_frame_rst    <= 'd0;
            frame_rst        <= 1'b1;
        end
        else begin
            if(cnt_frame_rst >= 'd15) begin
                frame_rst    <= 1'b0;
            end
            else begin
                cnt_frame_rst    <= cnt_frame_rst + 1'b1;
            end
        end
    end
end

always @(posedge clk  or posedge rst) begin
    if(rst == 1'b1) begin
        real_lval_num    <= 'd0;
    end
    else begin
        case(image_type_3dly)
            3'b000: begin    // Protected
                        real_lval_num    = lval_num;
                    end
            3'b001: begin    // ORG image
                        real_lval_num    = lval_num;
                    end
            3'b010: begin    // AVG image
                        real_lval_num    = lval_num;
                    end
            3'b100: begin    // STDEV image
                        real_lval_num    = lval_num * 2;
                    end
            3'b011: begin    // ORG & AVG image
                        real_lval_num    = lval_num * 2;
                    end
            3'b101: begin    // ORG & STDEV image
                        real_lval_num    = lval_num * 3;
                    end
            3'b110: begin    // AVG & STDEV image
                        real_lval_num    = lval_num * 3;
                    end
            3'b111: begin    // ORG & AVG & STDEV image
                        real_lval_num    = lval_num * 4;
                    end
        endcase
    end
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        cs_mig    <= s_mig_init;
    end
    else begin
        cs_mig    <= ns_mig;
    end
end

always @(*) begin
    case(cs_mig)
        s_mig_init: begin
                        if(state_flag_next == 1'b1) begin
                            ns_mig    = s_idle;
                        end
                        else begin
                            ns_mig    = s_mig_init;
                        end
                    end
        s_idle:     begin
                        if(state_flag_next == 1'b1) begin
                            ns_mig    = s_wr;
                        end
                        else begin
                            ns_mig    = s_idle;
                        end
                    end
        s_wr:       begin
                        if(state_flag_back == 1'b1) begin
                            ns_mig    = s_wr_wait;
                        end
                        else if(state_flag_next == 1'b1) begin
                            ns_mig    = s_rd;
                        end
                        else begin
                            ns_mig    = s_wr;
                        end
                    end
        s_wr_wait:  begin
                        ns_mig    = s_idle;
                    end
        s_rd:       begin
                        if(state_flag_next == 1'b1) begin
                            ns_mig    = s_rd_wait;
                        end
                        else begin
                            ns_mig    = s_rd;
                        end
                    end
        s_rd_wait:     begin
                        if(state_flag_back == 1'b1) begin
                            ns_mig    = s_rd;
                        end
                        else if(state_flag_next == 1'b1) begin
                            ns_mig    = s_mig_init;
                        end
                        else begin
                            ns_mig    = s_rd_wait;
                        end
                    end
        default:    begin
                        ns_mig    = s_mig_init;
                    end
    endcase
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        addr_row_start      <= 'd0;
        o_app_addr          <= 'd0;
        o_app_cmd           <= 'd0;
        app_en_tmp          <= 1'b0;
        mig_wr_flag         <= 1'b0;
        cnt_app_in          <= 'd0;
        cnt_app_out         <= 'd0;
        cnt_total_lval      <= 'd0;
        cnt_row             <= 'd0;
        cnt_frame           <= 'd0;
        rd_org_done         <= 1'b0;
        rd_div_done         <= 1'b0;
        rd_sf1_done         <= 1'b0;
        state_flag_back     <= 1'b0;
        state_flag_next     <= 1'b0;
        flag_wr_odd         <= 1'b0;
        o_ddr_wr_done       <= 1'b0;
    end
    else begin
        case(cs_mig)
            s_mig_init: begin
                            if(state_flag_next == 1'b0) begin
                                if(i_init_calib_complete == 1'b1) begin
                                    state_flag_next     <= 1'b1;
                                end
                            end
                            else begin
                                addr_row_start     <= 'd0;
                                o_app_addr         <= 'd0;
                                o_app_cmd          <= 'd0;
                                app_en_tmp         <= 1'b0;

                                mig_wr_flag        <= 1'b0;
                                flag_wr_odd        <= 1'b0;
                                cnt_app_in         <= 'd0;
                                cnt_app_out        <= 'd0;
                                cnt_total_lval     <= 'd0;
                                cnt_row            <= 'd0;
                                cnt_frame          <= 'd0;

                                rd_org_done        <= 1'b0;
                                rd_div_done        <= 1'b0;
                                rd_sf1_done        <= 1'b0;

                                state_flag_back    <= 1'b0;
                                state_flag_next    <= 1'b0;
                                o_ddr_wr_done      <= 1'b0;
                            end
                        end
            s_idle:     begin
                            if(state_flag_next == 1'b0) begin
                                if(i_fifo2ddr_rd_cnt >= 'd20) begin
                                    state_flag_next    <= 1'b1;
                                end

                                rd_sf1_done    <= 1'b0;

                                if(image_type_3dly[0] == 1'b0) begin
                                    rd_org_done    <= 1'b1;
                                end
                                else begin
                                    rd_org_done    <= 1'b0;
                                end

                                if(image_type_3dly[1] == 1'b0) begin
                                    rd_div_done    <= 1'b1;
                                end
                                else begin
                                    rd_div_done    <= 1'b0;
                                end
                            end
                            else begin
                                addr_row_start     <= 'd0;
                                cnt_app_in         <= 'd0;
                                o_app_cmd          <= 'd0;
                                mig_wr_flag        <= 1'b1;
                                state_flag_next    <= 1'b0;
                            end
                        end
            s_wr:       begin
                            if((state_flag_back == 1'b0) && (state_flag_next == 1'b0)) begin
                                if(o_fifo2ddr_ren == 1'b1) begin
                                    o_app_addr    <= o_app_addr + 'd8;

                                    if(cnt_app_in == lval_len_3dly - 1'b1) begin
                                        mig_wr_flag    <= 1'b0;
                                        cnt_app_in     <= 'd0;
                                        flag_wr_odd    <= ~flag_wr_odd;
                                        
                                        if(cnt_row == win_row_len_3dly - 1'b1) begin
                                            cnt_row    <= 'd0;
                                            if(cnt_frame == frame_num_3dly - 1'b1) begin
                                                state_flag_next    <= 1'b1;
                                                o_ddr_wr_done      <= 1'b1;
                                                cnt_frame          <= 'd0;
                                                o_app_cmd          <= 'd1;
                                            end
                                            else begin
                                                state_flag_back    <= 1'b1;
                                                cnt_frame          <= cnt_frame + 1'b1;
                                            end
                                        end
                                        else begin
                                            state_flag_back    <= 1'b1;
                                            cnt_row            <= cnt_row + 1'b1;
                                        end
                                    end
                                    else begin
                                        cnt_app_in    <= cnt_app_in + 1'b1;
                                    end
                                end
                            end
                            else begin
                                state_flag_next    <= 1'b0;
                                state_flag_back    <= 1'b0;
                                if(state_flag_next == 1'b1) begin
                                    o_app_addr    <= 'd0;
                                    app_en_tmp    <= 1'b1;
                                end
                            end
                        end
            s_wr_wait:  begin
                            if(hdr_mode_en_3dly != 2'b00) begin    // 12bit HDR
                                if(cnt_row == 'd0) begin
                                    o_app_addr            <= o_app_addr;
                                end
                                else begin
                                    if(flag_wr_odd == 1'b1) begin
                                        o_app_addr            <= o_app_addr + addr_jump_frame_half[28:0]; 
                                    end
                                    else begin
                                        o_app_addr            <= o_app_addr - addr_jump_frame_half[28:0] - addr_jump_row;
                                    end
                                end
                            end
                            else begin
                                o_app_addr    <= o_app_addr;
                            end
                        end
            s_rd:       begin
                            if(state_flag_next == 1'b0) begin
                                if(i_app_rdy == 1'b1) begin
                                    o_app_addr    <= o_app_addr + 'd8;

                                    if(cnt_app_out == lval_len_3dly - 1'b1) begin
                                        state_flag_next    <= 1'b1;
                                        app_en_tmp         <= 1'b0;
                                        cnt_app_out        <= 'd0;
                                    end
                                    else begin
                                        cnt_app_out    <= cnt_app_out + 1'b1;
                                    end
                                end
                            end
                            else begin
                                state_flag_next    <= 1'b0;
                            end
                        end
            s_rd_wait:  begin
                            if((state_flag_back == 1'b0) && (state_flag_next == 1'b0)) begin
                                if(ddr_rd_req_dly[3:2] == 2'b01) begin
                                    if(cnt_total_lval == real_lval_num - 1'b1) begin    // Read out finished
                                        state_flag_next    <= 1'b1;
                                        cnt_total_lval     <= 'd0;
                                        o_app_cmd          <= 'd0;
                                        o_app_addr         <= 'd0;
                                        addr_row_start     <= 'd0;
                                        cnt_row            <= 'd0;
                                        cnt_frame          <= 'd0;
                                        rd_org_done        <= 1'b0;
                                        rd_div_done        <= 1'b0;
                                        rd_sf1_done        <= 1'b0;
                                    end
                                    else begin    // Address jump
                                        state_flag_back    <= 1'b1;
                                        cnt_total_lval     <= cnt_total_lval + 1'b1;

                                        if(rd_org_done == 1'b0) begin    // ORG image read out follow order
                                            if(cnt_total_lval < lval_num - 1'b1) begin
                                                o_app_addr    <= o_app_addr;
                                            end
                                            else begin
                                                o_app_addr     <= 'd0;
                                                rd_org_done    <= 1'b1;
                                            end
                                        end
                                        else if(rd_div_done == 1'b0) begin    // AVG image first read out same row in each frames, and then going to next row
                                            if(cnt_frame < frame_num_3dly - 1'b1) begin
                                                cnt_frame     <= cnt_frame + 1'b1;
                                                o_app_addr    <= o_app_addr + addr_jump_frame[28:0];    // Address jumping to next frame's same row
                                                
                                                if(cnt_frame == 'd0) begin
                                                    addr_row_start    <= o_app_addr;    // Recording the next row's start address
                                                end
                                            end
                                            else begin    // All frames' row read out, jump to first frame's next row
                                                cnt_frame     <= 'd0;
                                                o_app_addr    <= addr_row_start;
                                                
                                                if(cnt_row < win_row_len_3dly - 1'b1) begin
                                                    cnt_row    <= cnt_row + 1'b1;
                                                end
                                                else begin
                                                    cnt_row        <= 'd0;
                                                    o_app_addr     <= 'd0;
                                                    rd_div_done    <= 1'b1;
                                                end
                                            end
                                        end
                                        else begin
                                            if(cnt_frame < frame_num_3dly - 1'b1) begin    // Read out same row data in each frame, running twice
                                                cnt_frame     <= cnt_frame + 1'b1;
                                                o_app_addr    <= o_app_addr + addr_jump_frame[28:0];

                                                if(cnt_frame == 'd0) begin
                                                    if(rd_sf1_done == 1'b0) begin
                                                        addr_row_start    <= o_app_addr - addr_jump_row;    // Recording the same row's start address
                                                    end
                                                    else begin
                                                        addr_row_start    <= o_app_addr;    // Recording the next row's start address
                                                    end
                                                end
                                            end
                                            else begin    // All frames' row read out, jump to record row's start address
                                                cnt_frame      <= 'd0;
                                                o_app_addr     <= addr_row_start;
                                                rd_sf1_done    <= ~rd_sf1_done;
                                            end
                                        end
                                    end
                                end
                            end
                            else if(state_flag_next == 1'b1) begin
                                state_flag_next    <= 1'b0;
                            end
                            else begin
                                state_flag_back    <= 1'b0;
                                app_en_tmp         <= 1'b1;
                            end
                        end
        endcase
    end
end

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk  or posedge rst)begin
    if(rst == 1'b1)begin
        cmd_frame_req_dly    <= 'd0;
    end
    else begin
        cmd_frame_req_dly    <= {cmd_frame_req_dly[2:0],i_cmd_frame_req};
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        lval_len_1dly    <= 'd0;
        lval_len_2dly    <= 'd0;
        lval_len_3dly    <= 'd0;
    end
    else begin
        lval_len_1dly    <= i_lval_len;
        lval_len_2dly    <= lval_len_1dly;
        lval_len_3dly    <= lval_len_2dly;
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        image_type_1dly    <= 'd0;
        image_type_2dly    <= 'd0;
        image_type_3dly    <= 'd0;
    end
    else begin
        image_type_1dly    <= i_image_type;
        image_type_2dly    <= image_type_1dly;
        image_type_3dly    <= image_type_2dly;
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        hdr_mode_en_1dly    <= 'd0;
        hdr_mode_en_2dly    <= 'd0;
        hdr_mode_en_3dly    <= 'd0;
    end
    else begin
        hdr_mode_en_1dly    <= i_hdr_mode_en;
        hdr_mode_en_2dly    <= hdr_mode_en_1dly;
        hdr_mode_en_3dly    <= hdr_mode_en_2dly;
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        win_row_len_1dly         <= 'd0;
        win_row_len_2dly         <= 'd0;
        win_row_len_3dly         <= 'd0;
        win_row_len_3dly_half    <= 'd0;
    end
    else begin
        win_row_len_1dly         <= i_win_row_len;
        win_row_len_2dly         <= win_row_len_1dly;
        win_row_len_3dly         <= win_row_len_2dly;
        win_row_len_3dly_half    <= win_row_len_2dly[15:1];
    end
end
always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        frame_num_1dly    <= 'd0;
        frame_num_2dly    <= 'd0;
        frame_num_3dly    <= 'd0;
    end
    else begin
        frame_num_1dly    <= i_frame_num;
        frame_num_2dly    <= frame_num_1dly;
        frame_num_3dly    <= frame_num_2dly;
    end
end


always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        ddr_rd_req_dly    <= 'd0;
    end
    else begin
        ddr_rd_req_dly    <= {ddr_rd_req_dly[2:0], i_ddr_rd_req};
    end
end

endmodule