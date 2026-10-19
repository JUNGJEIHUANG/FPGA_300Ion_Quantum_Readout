`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : image_dat_tx
// --File      : image_dat_tx.v
// --Author    : Weihan Li
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : Data transmit out function
//
//               Transmitting data to USB protocol buffer FIFO
//
//               Two pixels transmitted as a 4-byte unit a time
//
//               Transmitting data length per-column decided by input column length
//
//               Support transmitted mode is interlace mode & serial mode
//               - mode sample:
//                   interlace mode:(INTERLACE_EN=1)
//                                         col 0 per chan     col 1 per chan          col lval-1 per chan
//                             Channel 0    :  Pix<0>            , Pix<n>            , ... Pix<n*(lval-1)>
//                             Channel 1    :  Pix<1>            , Pix<n+1>          , ... Pix<n*(lval-1)+1>
//                               ...            ...                  ...                       ...
//                             Channel n-1 :  Pix<n-1>          , Pix<2*n-1>         , ... Pix<n*lval-1>
//                   serial mode:(INTERLACE_EN=0)
//                             Channel 0    :  Pix<0>            , Pix<2>            , ... Pix<2*lval-2>
//                             Channel 1    :  Pix<1>            , Pix<3>            , ... Pix<2*lval-1>
//                               ...            ...                  ...               ...
//                             Channel n-2  :  Pix<(n-2)*lval>   , Pix<(n-2)*lval+2> , ... Pix<n*lval-2>
//                             Channel n-1  :  Pix<(n-2)*lval+1> , Pix<(n-2)*lval+3> , ... Pix<n*lval-1>
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Hisoty    :
// --Ver       Date         By              Description
// --1.0       2023.11.16   Weihan.li       Creat
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module image_dat_tx #(
    parameter    FVAL_GEN_EN           = 1'b0  , // generate o_dout_fval flag, 1:CameraLink@media_2chan*16bit and no FIFO in next module, 0:others
    parameter    INTERLACE_EN          = 1'b1  , // Channel output as pixel interlace mode enable
    parameter    TX_FIFO_CNT_WIDTH     = 13    , // FIFO count width：13,14,15
    parameter    CHAN_NUM              = 32    ) // Channel number, CHAN_NUM%2=0
(
// Clock & reset
    input                                         clk                 , // same to clk_rd of ddr_dout_buf.v,
    input                                         rst                 , // same to rst_rd of ddr_dout_buf.v,
    input                                         i_cmd_frame_req     , // command of frame requeset,rising edge effective  
// Configuration
    input            [15:0]                       i_lval_len          , // length of one row of i_din_vld&o_dout_vld
    input            [15:0]                       i_win_row_len       , // window row length number
    input            [15:0]                       i_win_row_len_fval  , // cameralink lval's number of fval
                                                                        // only for CameraLink@media_2chan*16bit. hidden in USB or other CameraLink mode
    input            [15:0]                       i_win_col_len       , // column length of one row
    input            [15:0]                       i_frame_num_tx      , // frame number transmitted to USB
    input            [TX_FIFO_CNT_WIDTH-1:0]      i_tx_fifo_cnt       , // USB FIFO's wr_data_count, hidden in CameraLink@media_2chan*16bit mode
// Data in
    input                                         i_din_rdy           , // i_din is ready
    input            [CHAN_NUM-1:0]               i_din_vld           , // valid flag of i_din,
    input            [CHAN_NUM*16-1:0]            i_din               , // input data 
    output    reg    [CHAN_NUM-1:0]               o_din_rd_en         , // read enbale of i_din
// Data out
    output    reg                                 o_dout_fval         , // only for CameraLink@media_2chan*16bit. hidden in USB or other CameraLink mode
                                                                        // CameraLink protocol: to generate the correct o_dout_fval, i_tx_fifo_cnt must be set to 0.
    output    reg                                 o_dout_vld          , // valid flag of o_dout
    output    reg    [31:0]                       o_dout              , // output data 
    output    reg                                 o_tx_row_done       , // one row output data is transfered done
    output    reg                                 o_img2tx_tx_en      , // Flag for transmitting o_dout. pull high when starting and pull down when all i_frame_num_tx completed
                                                                        // hidden in CameraLink@media_2chan*16bit mode
    output    reg                                 o_frame_done          // Flag for transmitting all i_frame_num_tx completed, 16clk pulse width high
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

localparam    s_tx_idle  = 'd0,
              s_tx_run   = 'd1,
              s_tx_done  = 'd2;

localparam    chan_width = $clog2(CHAN_NUM);    // Width of Channels registers

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Reset
reg     [3:0]               cmd_frame_req_dly      ;
reg     [4:0]               cnt_frame_req          ;
reg                         frame_rst              ;

// CDC
reg     [15:0]              lval_len_1dly          ;
reg     [15:0]              lval_len_2dly          ;
reg     [15:0]              lval_len_3dly          ;
reg     [15:0]              win_row_len_1dly       ;
reg     [15:0]              win_row_len_2dly       ;
reg     [15:0]              win_row_len_3dly       ;
reg     [15:0]              win_col_len_1dly       ;
reg     [15:0]              win_col_len_2dly       ; 
reg     [15:0]              frame_num_1dly         ;
reg     [15:0]              frame_num_2dly         ;
reg     [15:0]              frame_num_3dly         ;

reg     [15:0]              win_row_len_fval_1dly  ;
reg     [15:0]              win_row_len_fval_2dly  ;
reg     [15:0]              win_row_len_fval_3dly  ;
reg     [15:0]              cnt_lval               ;
reg                         fval                   ;

// Input data to array
reg     [15:0]              dat_array[CHAN_NUM-1:0];
reg                         dat_vld                ;

// Counter
reg     [15:0]              counter_len            ;    // Valid transmitted data number
reg     [15:0]              total_len              ;    // Totally data number
reg     [15:0]              cnt_lval_wr            ;
reg     [15:0]              cnt_lval_rd            ;
reg     [4:0]               cnt_done               ;
reg     [15:0]              counter                ;
// Channel number related
reg                         chan_change_en         ;    // Channel selected change enable
reg     [chan_width-1:0]    chan1_num              ;    // Read enable channels
reg     [chan_width-1:0]    chan2_num              ;    // Read enable channels
reg     [chan_width-1:0]    vld_chan1_num          ;    // Input valid channels
reg     [chan_width-1:0]    vld_chan2_num          ;    // Input valid channels
// Number row & frame
reg     [15:0]              cnt_row                ;
reg     [15:0]              cnt_frame              ;
// Done flag
reg     [3:0]               cnt_op_done            ;

// Output related
reg                         dat2usb_rd_en          ;
reg                         dat2usb_vld            ;
reg     [31:0]              dat2usb                ;

// FSM
reg     [1:0]               cs_tx                  ;
reg     [1:0]               ns_tx                  ;
reg                         state_flag_run         ;
reg                         state_flag_done        ;
reg                         state_flag_end         ;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
// -- Reset generator
//--------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin 
        cnt_frame_req           <= 'd0;
        frame_rst               <= 1'b1;
    end
    else begin
        if(cmd_frame_req_dly[3:2] == 2'b01) begin
            cnt_frame_req           <= 'd15;
            frame_rst               <= 1'b1;
        end
        else begin
            if(cnt_frame_req == 'd0) begin
                frame_rst               <= 1'b0;
            end
            else begin
                cnt_frame_req           <= cnt_frame_req - 1'b1;
            end
        end
    end
end

//--------------------------------------------------------------------------------
// -- Input data processing
//--------------------------------------------------------------------------------
genvar i;
generate
    for(i = 0; i < CHAN_NUM; i = i + 1) begin: dat_2Dbuf    // Data buffer as 2D array
        always @(posedge clk) begin
            dat_array[i]            <= i_din[i*16+15:i*16];
        end
    end
endgenerate

//--------------------------------------------------------------------------------
// -- Valid signal generator
//--------------------------------------------------------------------------------
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        counter                 <= 'd0;
        dat_vld                 <= 1'b0;
    end
    else begin
        if(i_din_vld != 'd0) begin
            // Effectical data
            if(counter <= counter_len - 1'b1) begin
                dat_vld                 <= 1'b1;
                if(counter >= total_len - 1'b1) begin
                    counter                 <= 'd0;
                end
                else begin
                    counter                 <= counter + 1'b1;
                end
            end
            // blank data
            else begin
                dat_vld                 <= 1'b0;
                if(counter >= total_len - 1'b1) begin
                    counter                 <= 'd0;
                end
                else begin
                    counter                 <= counter + 1'b1;
                end
            end
        end
        else begin
            dat_vld                 <= 1'b0;
        end
    end
end

//--------------------------------------------------------------------------------
// -- FSM
//--------------------------------------------------------------------------------
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        cs_tx                   <= s_tx_idle;
    end
    else begin
        cs_tx                   <= ns_tx;
    end
end

always @(*) begin
    case(cs_tx)
        s_tx_idle:  begin
                        if(state_flag_run == 1'b1) begin    // Tx operation start
                            ns_tx                   = s_tx_run;
                        end
                        else begin
                            ns_tx                   = s_tx_idle;
                        end
                    end
        s_tx_run:   begin
                        if(state_flag_done == 1'b1) begin    // All channels' data tx done
                            ns_tx                   = s_tx_done;
                        end
                        else begin
                            ns_tx                   = s_tx_run;
                        end
                    end
        s_tx_done:  begin
                        if(state_flag_end == 1'b1) begin    // Tx done
                            ns_tx                   = s_tx_idle;
                        end
                        else begin
                            ns_tx                   = s_tx_done;
                        end
                    end 
        default:    begin
                        ns_tx                   = s_tx_idle;
                    end 
    endcase
end

always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        o_din_rd_en             <= 'd0;
        o_tx_row_done           <= 1'b0;
        chan1_num               <= 'd0;
        chan2_num               <= 'd1;
        state_flag_run          <= 1'b0;
        state_flag_done         <= 1'b0;
        state_flag_end          <= 1'b0;
        cnt_lval_wr             <= 'd0;
        cnt_done                <= 'd0;
        chan_change_en          <= 1'b0;
    end
    else begin
        case(cs_tx)
            s_tx_idle:  begin
                            if(state_flag_run == 1'b0) begin
                                if(i_din_rdy == 1'b1) begin    // Tx operation start
                                    state_flag_run          <= 1'b1; 
                                end
                            end
                            else begin
                                chan1_num               <= 'd0;
                                chan2_num               <= 'd1;
                                state_flag_run          <= 1'b0;
                            end
                        end
            s_tx_run:   begin
                            if(state_flag_done == 1'b0) begin
                                if(dat2usb_rd_en == 1'b1) begin
                                    // When input data mode is interlace mode
                                    if(INTERLACE_EN == 1'b1) begin
                                        if(chan1_num < CHAN_NUM - 2) begin    // Read enable channels change
                                            chan_change_en          <= 1'b1;
                                            chan1_num               <= chan1_num + 'd2;
                                            chan2_num               <= chan2_num + 'd2;
                                        end
                                        else begin    // When all channels read out once
                                            chan_change_en          <= 1'b0;
                                            chan1_num               <= 'd0;
                                            chan2_num               <= 'd1;
                                            if(cnt_lval_wr < lval_len_3dly - 1'b1) begin    // Count lval length
                                                cnt_lval_wr             <= cnt_lval_wr + 1'b1;
                                            end
                                            else begin    // All data read out
                                                cnt_lval_wr             <= 'd0;
                                                state_flag_done         <= 1'b1;
                                            end
                                        end
                                        // Channel read enable control
                                        // Current two channels' data read enable
                                        o_din_rd_en[chan1_num]  <= 1'b1;
                                        o_din_rd_en[chan2_num]  <= 1'b1;
                                        if(chan_change_en == 1'b1) begin    // Channels' traverse going on
                                            // Forward two channels' data read disable
                                            o_din_rd_en[chan1_num-2]<= 1'b0;
                                            o_din_rd_en[chan2_num-2]<= 1'b0;
                                        end
                                        else begin    // A round Channels' traverse done
                                            // When multi-channels(channel number larger than 3), disable last two channels
                                            if(CHAN_NUM > 3) begin
                                                o_din_rd_en[CHAN_NUM-2] <= 1'b0;
                                                o_din_rd_en[CHAN_NUM-1] <= 1'b0;
                                            end
                                        end
                                    end
                                    // When input data mode is serial mode
                                    else begin
                                        if(cnt_lval_wr < lval_len_3dly - 1'b1) begin    // Count lval length
                                            cnt_lval_wr             <= cnt_lval_wr + 1'b1;
                                            chan_change_en          <= 1'b0;
                                        end
                                        else begin    // Channels all read out
                                            cnt_lval_wr             <= 'd0;
                                            chan_change_en          <= 1'b1;
                                            if(chan1_num < CHAN_NUM - 2) begin    // Read enable channels change
                                                chan1_num               <= chan1_num + 'd2;
                                                chan2_num               <= chan2_num + 'd2;
                                            end
                                            else begin    // When all channels read out once
                                                chan1_num               <= 'd0;
                                                chan2_num               <= 'd1;
                                                state_flag_done         <= 1'b1;
                                            end
                                        end
                                        // Channel read enable control
                                        // Current two channels' data read enable
                                        o_din_rd_en[chan1_num]  <= 1'b1;
                                        o_din_rd_en[chan2_num]  <= 1'b1;
                                        if(chan_change_en == 1'b1) begin    // Channels' traverse going on
                                            // Forward two channels' data read disable
                                            o_din_rd_en[chan1_num-2]<= 1'b0;
                                            o_din_rd_en[chan2_num-2]<= 1'b0;
                                        end
                                    end
                                end
                                else begin
                                    o_din_rd_en             <= 'd0;
                                end
                            end
                            else begin
                                chan_change_en          <= 1'b0;
                                o_din_rd_en             <= 'd0;
                                cnt_done                <= 'd0;
                                state_flag_done         <= 1'b0;
                            end
                        end
            s_tx_done:  begin
                            if(state_flag_end == 1'b0) begin
                                if(cnt_done < 'd31) begin
                                    cnt_done                <= cnt_done + 1'b1;
                                    if(cnt_done > 'd15) begin
                                        o_tx_row_done           <= 1'b1;
                                    end
                                end
                                else begin
                                    cnt_done                <= 'd0;
                                    state_flag_end          <= 1'b1;
                                end
                            end
                            else begin
                                o_tx_row_done           <= 1'b0;
                                state_flag_end          <= 1'b0;
                            end
                        end
        endcase
    end
end

//--------------------------------------------------------------------------------
// -- Input data matching
//--------------------------------------------------------------------------------
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        vld_chan1_num           <= 'd0;
        vld_chan2_num           <= 'd1;
        if(INTERLACE_EN == 1'b0) begin
            cnt_lval_rd             <= 'd0;
        end
        dat2usb_vld             <= 1'b0;
        dat2usb                 <= 'd0;
    end
    else begin
        dat2usb_vld             <= dat_vld;
        dat2usb[15:0]           <= dat_array[vld_chan1_num];
        dat2usb[31:16]          <= dat_array[vld_chan2_num];
        if(dat_vld == 1'b1) begin
            // When input data mode is interlace mode
            if(INTERLACE_EN == 1'b1) begin
                if(vld_chan2_num < CHAN_NUM - 1) begin
                    vld_chan1_num           <= vld_chan1_num + 'd2;
                    vld_chan2_num           <= vld_chan2_num + 'd2;
                end
                else begin
                    vld_chan1_num           <= 'd0;
                    vld_chan2_num           <= 'd1;
                end
            end
            // When input data mode is serial mode
            else begin
                if(cnt_lval_rd < lval_len_3dly - 1'b1) begin
                    cnt_lval_rd             <= cnt_lval_rd + 1'b1;
                end
                else begin
                    cnt_lval_rd             <= 'd0;
                    if(vld_chan2_num < CHAN_NUM - 1) begin
                        vld_chan1_num           <= vld_chan1_num + 'd2;
                        vld_chan2_num           <= vld_chan2_num + 'd2;
                    end
                    else begin
                        vld_chan1_num           <= 'd0;
                        vld_chan2_num           <= 'd1;
                    end
                end
            end
        end
        else begin
            if(state_flag_end == 1'b1) begin
                if(INTERLACE_EN == 1'b0) begin
                    cnt_lval_rd             <= 'd0;
                end
                vld_chan1_num           <= 'd0;
                vld_chan2_num           <= 'd1;
            end
        end
    end
end

//--------------------------------------------------------------------------------
// -- FIFO empty space determine
//--------------------------------------------------------------------------------
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        dat2usb_rd_en           <= 1'b0;
    end
    else begin
        if(FVAL_GEN_EN == 1'b1) begin
            dat2usb_rd_en           <= 1'b1;
        end
        else begin
            if(i_tx_fifo_cnt < ({TX_FIFO_CNT_WIDTH{1'b1}} - 'd200)) begin
                dat2usb_rd_en           <= 1'b1;
            end
            else begin
                dat2usb_rd_en           <= 1'b0;
            end
        end
    end
end

//--------------------------------------------------------------------------------
// -- Output register
//--------------------------------------------------------------------------------
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        o_dout_vld               <= 1'b0;
        o_dout                   <= 'd0;
    end
    else begin
        o_dout_vld               <= dat2usb_vld;
        o_dout                   <= dat2usb;
    end
end

generate 
    if(FVAL_GEN_EN == 1'b1) begin
        always @(posedge clk or posedge frame_rst) begin
            if(frame_rst == 1'b1) begin
                o_dout_fval        <= 1'b0;
                fval               <= 1'b0;
                cnt_lval           <= 'd0; 
            end
            else begin 
                o_dout_fval        <= fval;
                if((dat_vld==1'b1)&&(dat2usb_vld==1'b0))begin
                    fval    <= 1'b1;
                end
                else if((dat_vld==1'b0)&&(dat2usb_vld==1'b1))begin
                    if(cnt_lval < win_row_len_fval_3dly - 1'b1)begin
                        cnt_lval      <= cnt_lval + 1'b1;
                    end
                    else begin
                        cnt_lval      <= 'd0;
                        fval          <= 1'b0;
                    end
                end  
            end
        end
    end
endgenerate

//--------------------------------------------------------------------------------
// -- Done flag & transmit enable signal generator
//--------------------------------------------------------------------------------
always @(posedge clk or posedge frame_rst) begin
    if(frame_rst == 1'b1) begin
        o_img2tx_tx_en          <= 1'b0;
        o_frame_done            <= 1'b0;
        cnt_row                 <= 'd0;
        cnt_frame               <= 'd0;
        cnt_op_done             <= 'd0;
    end
    else begin
        // Counting transmit times
        if(state_flag_end == 1'b1) begin
            if(cnt_row < win_row_len_3dly - 1'b1) begin
                cnt_row                 <= cnt_row + 1'b1;
            end
            else begin
                cnt_row                 <= 'd0;
                if(cnt_frame < frame_num_3dly - 1'b1) begin
                    cnt_frame               <= cnt_frame + 1'b1;
                end
                // Pull done transmit enable signal and pull up done flag after all frame data transmit done
                else begin
                    cnt_frame               <= 'd0;
                    o_img2tx_tx_en          <= 1'b0;
                    o_frame_done            <= 1'b1;
                end
            end
        end
        // Pull up transmit enable signal before transimtting start
        else if(state_flag_run == 1'b1) begin
            o_img2tx_tx_en          <= 1'b1;
        end
        // Pull down done flag after it pull up 15 clock
        else if(o_frame_done == 1'b1) begin
            if(cnt_op_done < 'd15) begin
                cnt_op_done             <= cnt_op_done + 1'b1;
            end
            else begin
                cnt_op_done             <= 'd0;
                o_frame_done            <= 1'b0;
            end
        end
    end
end

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Instance
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CDC
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        cmd_frame_req_dly       <= 'd0;
    end
    else begin
        cmd_frame_req_dly       <= {cmd_frame_req_dly[2:0], i_cmd_frame_req};
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        lval_len_1dly           <= 'd0;
        lval_len_2dly           <= 'd0;
        lval_len_3dly           <= 'd0;
        total_len               <= 'd0;
    end
    else begin
        lval_len_1dly           <= i_lval_len;
        lval_len_2dly           <= lval_len_1dly;
        lval_len_3dly           <= lval_len_2dly;
        total_len               <= lval_len_2dly * (CHAN_NUM / 2);
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        win_row_len_1dly        <= 'd0;
        win_row_len_2dly        <= 'd0;
        win_row_len_3dly        <= 'd0;
    end
    else begin
        win_row_len_1dly        <= i_win_row_len;
        win_row_len_2dly        <= win_row_len_1dly;
        win_row_len_3dly        <= win_row_len_2dly;
    end
end
 
generate 
    if(FVAL_GEN_EN == 1'b1) begin
        always @(posedge clk or posedge rst) begin
            if(rst == 1'b1) begin 
                win_row_len_fval_1dly        <= 'd0;
                win_row_len_fval_2dly        <= 'd0;
                win_row_len_fval_3dly        <= 'd0; 
            end
            else begin 
                win_row_len_fval_1dly        <= i_win_row_len_fval;
                win_row_len_fval_2dly        <= win_row_len_fval_1dly;
                win_row_len_fval_3dly        <= win_row_len_fval_2dly; 
            end
        end  
    end
endgenerate 

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        win_col_len_1dly        <= 'd0;
        win_col_len_2dly        <= 'd0;
        counter_len             <= 'd0;
    end
    else begin
        win_col_len_1dly        <= i_win_col_len;
        win_col_len_2dly        <= win_col_len_1dly;
        counter_len             <=(win_col_len_2dly + 'd1) / 2;
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        frame_num_1dly         <= 'd0;
        frame_num_2dly         <= 'd0;
        frame_num_3dly         <= 'd0;
    end
    else begin
        frame_num_1dly         <= i_frame_num_tx;
        frame_num_2dly         <= frame_num_1dly;
        frame_num_3dly         <= frame_num_2dly;
    end
end


endmodule