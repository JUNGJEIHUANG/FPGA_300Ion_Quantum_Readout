`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//------------------------------------------------------------------------------------------------------------------
// --Module    : lval_synncode
// --File      : lval_synncode.v
// --Author    : tianjiao.hou@gpixel.com 
//------------------------------------------------------------------------------------------------------------------
// --Function  : receive data between synccode_start and synccode_end 
//------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// -- 1.0      2022.05.30   Tianjiao Hou    creat 
// -- 1.1      2022.06.22   Danyang Li      modify 
// -- 1.3      2022.11.01   Tianjiao Hou    modify 
// -- 1.4      2023.05.10   Tianjiao Hou    i_cmd_frame_req added, synccode as input for universal use, sol ynamically changing support
//------------------------------------------------------------------------------------------------------------------
module lval_synccode#(
    parameter    DAT_WIDTH              = 12   ,   // data width supported : 12/14/16
    parameter    SOL_INCREASE_EN        = 1'b1 ,   // synccode_4th_sol dynamically changing based on which line it is // eg.SOL<N> : (AB0+N)h
                                                   // when SOL_INCREASE_EN = 1, 'dynamically forth word of sync code' is from i_synccode_4th_sol to 'i_synccode_4th_sol + i_win_row_len - 1'
    parameter    FIFO_DEPTH             = 4096 ,   // >i_lval_len // FIFO depth:1024/2048/4096/8192/16384
    parameter    FIFO_DATACOUNT_WIDTH   = 12       // width of FIFO's data count //2^FIFO_DATACOUNT_WIDTH=FIFO_DEPTH
)(
    input                                   clk                  ,
    input                                   rst                  ,
    input                                   i_cmd_frame_req      , // cmd_frame_req,rising edge effective  
    input     [DAT_WIDTH-1:0]               i_synccode_1st       , // first word of sync code
    input     [DAT_WIDTH-1:0]               i_synccode_2nd       , // second word of sync code
    input     [DAT_WIDTH-1:0]               i_synccode_3rd       , // third word of sync code
    input     [DAT_WIDTH-1:0]               i_synccode_4th_sof   , // forth word of sync code, start of frame  //not used when SOL_INCREASE_EN=1
    input     [DAT_WIDTH-1:0]               i_synccode_4th_sol   , // forth word of sync code, start of line
    input     [15:0]                        i_lval_len           , // valid pixel number in one line
    input     [15:0]                        i_win_row_len        , // row(line) number in one frame
    input                                   i_dat_vld            , // i_dat valid flag
    input     [DAT_WIDTH-1:0]               i_dat                , // input data
    input                                   i_fifo_ren           , // fifo read enable
    output    [DAT_WIDTH-1:0]               o_fifo_dout          , // fifo output data
    output    [FIFO_DATACOUNT_WIDTH-1:0]    o_data_count,         // data count in FIFO
    output    [DAT_WIDTH-1:0]               o_fifo_din_dbg,
    output                                  o_fifo_wen_dbg,
    output                                  o_fifo_rst_dbg,
    output    [63:0]                         o_debug
); 

//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------

reg    [15:0]             lval_len_1dly          ;
reg    [15:0]             lval_len_2dly          ;
reg    [15:0]             win_row_len_1dly       ;
reg    [15:0]             win_row_len_2dly       ; 
reg    [DAT_WIDTH-1:0]    synccode_1st_1dly      ;
reg    [DAT_WIDTH-1:0]    synccode_2nd_1dly      ;
reg    [DAT_WIDTH-1:0]    synccode_3rd_1dly      ;
reg    [DAT_WIDTH-1:0]    synccode_4th_sof_1dly  ;
reg    [DAT_WIDTH-1:0]    synccode_4th_sol_1dly  ;
reg    [DAT_WIDTH-1:0]    synccode_1st_2dly      ;
reg    [DAT_WIDTH-1:0]    synccode_2nd_2dly      ;
reg    [DAT_WIDTH-1:0]    synccode_3rd_2dly      ;
reg    [DAT_WIDTH-1:0]    synccode_4th_sof_2dly  ;
reg    [DAT_WIDTH-1:0]    synccode_4th_sol_2dly  ;
reg    [2:0]              FlagSyncCode           ;
reg    [1:0]              FlagSyncCodeID         ;
reg    [2:0]              PrevSyncCodes          ;
reg                       FlagSyncCode_SOL       ;
reg                       FlagSyncCode_SOF       ;
reg                       dat_vld_dly            ;
reg                       Initiator_Valid_Fifo   ;
reg    [DAT_WIDTH-1:0]    Initiator_Data_Fifo    ;
reg                       Fsm_Valid_Gen          ;
reg                       Sync_Valid             ;
reg    [DAT_WIDTH-1:0]    dat_in_1dly            ;
reg    [DAT_WIDTH-1:0]    dat_in_2dly            ;
reg    [15:0]             CntPixelPerLine        ;
wire   [15:0]             fifo_dout              ;
reg    [3:0]              cmd_frame_req_dly      ;
reg    [4:0]              cnt_fifo_rst           ;
reg                       fifo_rst               ;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

assign    o_fifo_dout = fifo_dout[DAT_WIDTH-1:0];
// Observation only: exactly the data / wr_en / reset driving u_fifo_synccode.
assign o_fifo_din_dbg = Initiator_Data_Fifo;
assign o_fifo_wen_dbg = Initiator_Valid_Fifo;
assign o_fifo_rst_dbg = fifo_rst;
assign    o_debug = {
    12'd0, i_dat, Initiator_Data_Fifo, 4'd0,
    i_dat_vld, Initiator_Valid_Fifo, Sync_Valid,
    FlagSyncCode_SOL, FlagSyncCode_SOF, Fsm_Valid_Gen,
    CntPixelPerLine[7:0]
};

//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------

always @(posedge clk  or posedge rst)begin
    if(rst == 1'b1)begin 
        cnt_fifo_rst    <= 'd0;
        fifo_rst        <= 1'b0;
    end
    else begin 
        if(cmd_frame_req_dly[3:2] == 2'b01)begin 
            cnt_fifo_rst    <= 'd0;
            fifo_rst        <= 1'b1;
        end
        else begin
            if(cnt_fifo_rst >= 'd15)begin
                fifo_rst        <= 1'b0;
            end
            else begin
                cnt_fifo_rst    <= cnt_fifo_rst + 1'b1;
            end
        end
    end
end 

always @(posedge clk  or posedge rst) begin
    if(rst == 1'b1)begin 
        FlagSyncCode      <= 3'd0;
        FlagSyncCodeID    <= 2'd0;
    end
    else begin 

        if(i_dat_vld ==1'b1)begin
            if(i_dat == synccode_1st_2dly)
                FlagSyncCode[0]      <= 1'b1;
            else
                FlagSyncCode[0]      <= 1'b0;
            if(i_dat == synccode_2nd_2dly)
                FlagSyncCode[1]      <= 1'b1;
            else
                FlagSyncCode[1]      <= 1'b0;
            if(i_dat == synccode_3rd_2dly)
                FlagSyncCode[2]      <= 1'b1;
            else
                FlagSyncCode[2]      <= 1'b0;  
                
            if (SOL_INCREASE_EN == 1'b0) begin
                if(i_dat == synccode_4th_sol_2dly) 
                    FlagSyncCodeID[0]    <= 1'b1;
                else
                    FlagSyncCodeID[0]    <= 1'b0;
                if(i_dat == synccode_4th_sof_2dly) 
                    FlagSyncCodeID[1]    <= 1'b1;
                else
                    FlagSyncCodeID[1]    <= 1'b0;
            end
            else begin
                if(i_dat >= synccode_4th_sol_2dly && i_dat <= synccode_4th_sol_2dly + win_row_len_2dly - 1'b1)
                    FlagSyncCodeID[0]    <= 1'b1;
                else
                    FlagSyncCodeID[0]    <= 1'b0;
                FlagSyncCodeID[1]    <= 1'b0;
            end
        end
    end
end 

always @(posedge clk  or posedge rst)begin
    if(rst == 1'b1)begin 
        PrevSyncCodes       <= 3'd0;
        FlagSyncCode_SOL    <= 1'b0;
        FlagSyncCode_SOF    <= 1'b0;
        dat_vld_dly         <= 1'b0;
    end
    else begin
        dat_vld_dly         <= i_dat_vld;
        if(dat_vld_dly == 1'b1) begin
            PrevSyncCodes[0]    <= FlagSyncCode[0];
            PrevSyncCodes[1]    <= FlagSyncCode[1] & PrevSyncCodes[0];
            PrevSyncCodes[2]    <= FlagSyncCode[2] & PrevSyncCodes[1];
            FlagSyncCode_SOL    <= PrevSyncCodes[2] & FlagSyncCodeID[0];
            FlagSyncCode_SOF    <= PrevSyncCodes[2] & FlagSyncCodeID[1];
        end
    end
end

always @(posedge clk  or posedge rst) begin
    if(rst == 1'b1)begin 
        dat_in_1dly             <= 'd0;
        dat_in_2dly             <= 'd0;
        Fsm_Valid_Gen           <= 1'b0;
        Sync_Valid              <= 1'b0;
        Initiator_Valid_Fifo    <= 1'b0;
        Initiator_Data_Fifo     <= 'd0;
        CntPixelPerLine         <= 'd0;
    end
    else begin
        dat_in_1dly             <= i_dat;
        dat_in_2dly             <= dat_in_1dly;
        Initiator_Data_Fifo     <= dat_in_2dly;
        Initiator_Valid_Fifo    <= Sync_Valid;
        case(Fsm_Valid_Gen)
            1'b0:    begin
                        if(dat_vld_dly == 1'b1) begin
                            CntPixelPerLine      <= 'd0;
                            if((FlagSyncCode_SOL == 1'b1)||(FlagSyncCode_SOF == 1'b1)) begin
                                Sync_Valid       <= 1'b1;
                                Fsm_Valid_Gen    <= 1'b1;
                            end
                            else begin
                                Sync_Valid       <= 1'b0;
                                Fsm_Valid_Gen    <= 1'b0;
                            end
                        end
                    end
            1'b1:    begin
                        if(dat_vld_dly == 1'b1) begin
                            if(CntPixelPerLine    == lval_len_2dly - 1'b1) begin
                                Fsm_Valid_Gen      <= 1'b0;
                                Sync_Valid         <= 1'b0;
                                CntPixelPerLine    <= 'd0;
                            end
                            else begin
                                CntPixelPerLine    <= CntPixelPerLine + 1'b1;
                                Sync_Valid         <= 1'b1;
                            end
                        end
                        else begin
                            Sync_Valid             <= 1'b0;
                        end
                    end
        endcase
    end
end

//--------------------------------------------------------------------------------------------------------------------
//Instance
//--------------------------------------------------------------------------------------------------------------------

generate
    if (FIFO_DEPTH == 1024) begin 
        // Common Clock Block RAM   
        // Standard FIFO
        // Write/Read width: 16
        // Write/Read depth: 1024
        // ResetPin: select
        // ResetType: Asynchronous Reset
        // Enable Safety Circuit: not select
        // Read Data Count : select 
        if (DAT_WIDTH == 16) begin 
             fifo_w16d1024_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout                         ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
        else begin 
             fifo_w16d1024_ccbr_st    u_fifo_synccode (
                .clk              ( clk                                          ),
                .rst              ( fifo_rst                                     ),
                .din              ( {{(16-DAT_WIDTH){1'b0}},Initiator_Data_Fifo} ),
                .wr_en            ( Initiator_Valid_Fifo                         ),
                .rd_en            ( i_fifo_ren                                   ),
                .dout             ( fifo_dout                                    ),
                .full             (                                              ),
                .empty            (                                              ),
                .data_count       ( o_data_count                                 )
            );
        end
    end
    else if (FIFO_DEPTH == 16'd2048) begin 
        // Common Clock Block RAM   
        // Standard FIFO
        // Write/Read width: 16
        // Write/Read depth: 2048
        // ResetPin: select
        // ResetType: Asynchronous Reset
        // Enable Safety Circuit: not select
        // Read Data Count : select 
        if (DAT_WIDTH == 16) begin 
             fifo_w16d2048_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout                         ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
        else begin 
             fifo_w16d2048_ccbr_st    u_fifo_synccode (
                .clk              ( clk                                          ),
                .rst              ( fifo_rst                                     ),
                .din              ( {{(16-DAT_WIDTH){1'b0}},Initiator_Data_Fifo} ),
                .wr_en            ( Initiator_Valid_Fifo                         ),
                .rd_en            ( i_fifo_ren                                   ),
                .dout             ( fifo_dout                                    ),
                .full             (                                              ),
                .empty            (                                              ),
                .data_count       ( o_data_count                                 )
            );
        end
    end 
    else if (FIFO_DEPTH == 16'd4096) begin 
        // Common Clock Block RAM   
        // Standard FIFO
        // Write/Read width: 12/16/16
        // Write/Read depth: 4096
        // ResetPin: select
        // ResetType: Asynchronous Reset
        // Enable Safety Circuit: not select
        // Read Data Count : select 
        if (DAT_WIDTH == 12) begin 
             fifo_w12d4096_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout[DAT_WIDTH-1:0]          ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
        else if (DAT_WIDTH == 14) begin 
             fifo_w16d4096_ccbr_st    u_fifo_synccode (
                .clk              ( clk                                          ),
                .rst              ( fifo_rst                                     ),
                .din              ( {{(16-DAT_WIDTH){1'b0}},Initiator_Data_Fifo} ),
                .wr_en            ( Initiator_Valid_Fifo                         ),
                .rd_en            ( i_fifo_ren                                   ),
                .dout             ( fifo_dout                                    ),
                .full             (                                              ),
                .empty            (                                              ),
                .data_count       ( o_data_count                                 )
            );
        end
        else begin 
             fifo_w16d4096_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout                         ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
    end
    else if (FIFO_DEPTH == 16'd8192) begin 
        // Common Clock Block RAM   
        // Standard FIFO
        // Write/Read width: 12/14/16
        // Write/Read depth: 8192
        // ResetPin: select
        // ResetType: Asynchronous Reset
        // Enable Safety Circuit: not select
        // Read Data Count : select  
        if (DAT_WIDTH == 12) begin 
             fifo_w12d8192_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout[DAT_WIDTH-1:0]          ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
        else if (DAT_WIDTH == 14) begin 
             fifo_w14d8192_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout[DAT_WIDTH-1:0]          ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
        else begin 
             fifo_w16d8192_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout[DAT_WIDTH-1:0]          ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
    end 
    else if (FIFO_DEPTH == 16'd16384) begin 
        // Common Clock Block RAM   
        // Standard FIFO
        // Write/Read width: 12/14/16
        // Write/Read depth: 16384
        // ResetPin: select
        // ResetType: Asynchronous Reset
        // Enable Safety Circuit: not select
        // Read Data Count : select 
        if (DAT_WIDTH == 12) begin 
             fifo_w12d16384_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout[DAT_WIDTH-1:0]          ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
        else if (DAT_WIDTH == 14) begin 
             fifo_w14d16384_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                          ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout[DAT_WIDTH-1:0]          ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end
        else begin 
             fifo_w16d16384_ccbr_st    u_fifo_synccode (
                .clk              ( clk                               ),
                .rst              ( fifo_rst                         ),
                .din              ( Initiator_Data_Fifo               ),
                .wr_en            ( Initiator_Valid_Fifo              ),
                .rd_en            ( i_fifo_ren                        ),
                .dout             ( fifo_dout[DAT_WIDTH-1:0]          ),
                .full             (                                   ),
                .empty            (                                   ),
                .data_count       ( o_data_count                      )
            );
        end 
    end 
endgenerate

//--------------------------------------------------------------------------------------------------------------------
//CDC
//--------------------------------------------------------------------------------------------------------------------

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
        synccode_1st_1dly                  <= 'd0; 
        synccode_2nd_1dly                  <= 'd0; 
        synccode_3rd_1dly                  <= 'd0; 
        synccode_4th_sof_1dly              <= 'd0; 
        synccode_4th_sol_1dly              <= 'd0; 
        synccode_1st_2dly                  <= 'd0; 
        synccode_2nd_2dly                  <= 'd0; 
        synccode_3rd_2dly                  <= 'd0; 
        synccode_4th_sof_2dly              <= 'd0; 
        synccode_4th_sol_2dly              <= 'd0;  
    end
    else begin 
        synccode_1st_1dly                  <= i_synccode_1st    ;
        synccode_2nd_1dly                  <= i_synccode_2nd    ;
        synccode_3rd_1dly                  <= i_synccode_3rd    ;
        synccode_4th_sof_1dly              <= i_synccode_4th_sof;
        synccode_4th_sol_1dly              <= i_synccode_4th_sol;
        synccode_1st_2dly                  <= synccode_1st_1dly    ;
        synccode_2nd_2dly                  <= synccode_2nd_1dly    ;
        synccode_3rd_2dly                  <= synccode_3rd_1dly    ;
        synccode_4th_sof_2dly              <= synccode_4th_sof_1dly;
        synccode_4th_sol_2dly              <= synccode_4th_sol_1dly;
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

endmodule
