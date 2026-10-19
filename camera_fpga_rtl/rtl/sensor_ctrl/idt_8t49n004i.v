`timescale 1ns / 1ps
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Module    : idt_8t49n004i
// --File      : idt_8t49n004i.v
// --Author    : guangcai.ding@gpixel.com
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --Function  : idt_8t49n004i module.
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2025.03.23   Guangcai Ding   Modify
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

module idt_8t49n004i
(
    input           clk           ,
    input           rst           ,
    input           i_cmd_set_pll ,
    output          o_CLK_SEL     ,
    output          o_FSEL0       ,
    output          o_FSEL1       ,
    output          o_IDT_8T_SCL  ,
    output          o_IDT_8T_SDA
);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameter
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

localparam    IDLE          = 6'd0,
              START         = 6'd1,
              SLAVE_ADDR    = 6'd2,
              ACK_addr      = 6'd3,
              Write_byte    = 6'd4,
              ACK_byte      = 6'd5,
              STOP          = 6'd6;

`define       SCL_POS (cnt == 3'd0)
`define       SCL_HIG (cnt == 3'd1)
`define       SCL_NEG (cnt == 3'd2)
`define       SCL_LOW (cnt == 3'd3)

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Reg & Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

reg               scl                     ;
reg               sda                     ;
reg     [2:0]     cnt                     ;
reg     [9:0]     cnt_delay               ;
reg     [8:0]     cnt_ACK                 ;
reg     [4:0]     cnt_byte                ;
wire    [7:0]     WRITE_BYTE[24:0]        ;
reg               sda_link                ;
reg               scl_link                ;
reg     [20:0]    cnt_20ms                ;
reg               cmd_set_IDT_8T49N004I_en;
reg               cmd_set_IDT_8T49N004I_q ;
reg               cmd_set_IDT_8T49N004I_qq;
reg     [3:0]     sda_num                 ;
reg     [5:0]     fsm_wr_IDT              ;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Assignment
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

assign    o_IDT_8T_SDA      = sda_link ? sda : 1'bz;    // FPGA收数，所以置位
assign    o_IDT_8T_SCL      = scl_link ? scl : 1'b1;
assign    o_CLK_SEL         = 1'b1;
assign    o_FSEL0           = 1'b0;
assign    o_FSEL1           = 1'b0;
assign    WRITE_BYTE[0]     = 8'b0000_0000;    // 起始寄存器地址
assign    WRITE_BYTE[1]     = 8'b0001_1110;    // M0 = 60
assign    WRITE_BYTE[2]     = 8'b0000_0000;    // M1
assign    WRITE_BYTE[3]     = 8'b0000_0000;    // M2
assign    WRITE_BYTE[4]     = 8'b0000_0000;    // M3
assign    WRITE_BYTE[5]     = 8'b0000_0010;    // N0 = 2
assign    WRITE_BYTE[6]     = 8'b0000_0000;    // N1
assign    WRITE_BYTE[7]     = 8'b0000_0000;    // N2
assign    WRITE_BYTE[8]     = 8'b0000_0000;    // N3
assign    WRITE_BYTE[9]     = 8'b0000_0001;    // BYPASS0 = 0    PS0 = 00    P0 = 00     CP0 = 01
assign    WRITE_BYTE[10]    = 8'b0000_0000;
assign    WRITE_BYTE[11]    = 8'b0000_0000;
assign    WRITE_BYTE[12]    = 8'b0000_0000;
assign    WRITE_BYTE[13]    = 8'b0110_0110;    // LVDS_SEL0 = 4'b1111
assign    WRITE_BYTE[14]    = 8'b0110_0110;
assign    WRITE_BYTE[15]    = 8'b0110_0110;
assign    WRITE_BYTE[16]    = 8'b0110_0110;
assign    WRITE_BYTE[17]    = 8'b0000_0010;    // OE0 = 4'b0001
assign    WRITE_BYTE[18]    = 8'b0000_0000;
assign    WRITE_BYTE[19]    = 8'b0000_0000;
assign    WRITE_BYTE[20]    = 8'b0000_0000;
assign    WRITE_BYTE[21]    = 8'b0000_0000;
assign    WRITE_BYTE[22]    = 8'b0000_0000;
assign    WRITE_BYTE[23]    = 8'b0000_0000;
assign    WRITE_BYTE[24]    = 8'b0000_0000;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Code
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        cmd_set_IDT_8T49N004I_en    <= 1'b0;
        cnt_20ms                    <= 'd0;
    end
    else begin
        if(i_cmd_set_pll == 1'b1) begin
            if(cnt_20ms >= 'd2000000) begin
                cmd_set_IDT_8T49N004I_en    <= 1'b1;
            end
            else begin
                cmd_set_IDT_8T49N004I_en    <= 1'b0;
                cnt_20ms                    <= cnt_20ms + 1'b1;
            end
        end
        else begin
            cmd_set_IDT_8T49N004I_en    <= 1'b0;
            cnt_20ms                    <= 'd0;
        end
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        cnt_delay    <= 10'd0;
    end
    else if(cnt_delay == 10'd999) begin    // 199
        cnt_delay    <= 10'd0;
    end
    else begin
        cnt_delay    <= cnt_delay + 1'b1;
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        cnt    <= 3'd5;
    end
    else begin
        case(cnt_delay)
            10'd200: cnt    <= 3'd1;
            10'd500: cnt    <= 3'd2;
            10'd700: cnt    <= 3'd3;
            10'd0:   cnt    <= 3'd0;
            default: cnt    <= 3'd5;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        scl    <= 1'b0;
    end
    else if(cnt == 3'd0) begin
        scl    <= 1'b1;
    end
    else if(cnt == 3'd2) begin
        scl    <= 1'b0;
    end
    else begin
        scl    <= scl;
    end
end    
    
always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        fsm_wr_IDT                  <= IDLE;
        sda                         <= 1'b1;
        sda_link                    <= 1'b0;
        scl_link                    <= 1'b0;
        sda_num                     <= 4'd0;
        cmd_set_IDT_8T49N004I_q     <= 1'b0;
        cmd_set_IDT_8T49N004I_qq    <= 1'b0;
        cnt_ACK                     <= 9'd0;
        cnt_byte                    <= 5'd0;
    end
    else begin
        cmd_set_IDT_8T49N004I_q     <= cmd_set_IDT_8T49N004I_en;
        cmd_set_IDT_8T49N004I_qq    <= cmd_set_IDT_8T49N004I_q ;
        case (fsm_wr_IDT)
            IDLE:       begin
                            sda_link    <= 1'b1;
                            sda         <= 1'b1;
                            scl_link    <= 1'b0;
                            if((cmd_set_IDT_8T49N004I_qq == 1'b0) && (cmd_set_IDT_8T49N004I_q == 1'b1)) begin
                                scl_link      <= 1'b1;
                                fsm_wr_IDT    <= START;
                            end
                        end
            START:      begin
                            if(`SCL_HIG) begin
                                sda_link      <= 1'b1;
                                sda           <= 1'b0;
                                fsm_wr_IDT    <= SLAVE_ADDR;
                                sda_num       <= 4'd0;
                            end
                            else begin
                                fsm_wr_IDT    <= START;
                            end
                        end
            SLAVE_ADDR: begin
                            if(`SCL_LOW) begin
                                if(sda_num == 4'd8) begin
                                    sda_num        <= 4'd0;
                                    sda            <= 1'b1;
                                    sda_link       <= 1'b0;
                                    fsm_wr_IDT     <= ACK_addr;
                                    cnt_ACK        <= 9'd0;
                                end
                                else begin
                                    fsm_wr_IDT     <= SLAVE_ADDR;
                                    sda_num        <= sda_num + 4'b1;
                                    case (sda_num)
                                        4'd0: sda     <= 1;
                                        4'd1: sda     <= 1;
                                        4'd2: sda     <= 0;
                                        4'd3: sda     <= 1;
                                        4'd4: sda     <= 1;
                                        4'd5: sda     <= 1;
                                        4'd6: sda     <= 0;
                                        4'd7: sda     <= 0;//写标志
                                        default: ;
                                    endcase
                                end
                            end
                            else begin
                                fsm_wr_IDT    <= SLAVE_ADDR;
                            end
                        end
            ACK_addr:   begin
                            if(!o_IDT_8T_SDA) begin
                                sda_link      <= 1'b1;
                                fsm_wr_IDT    <= Write_byte;
                                cnt_byte      <= 5'd0;
                            end
                            else begin
                                fsm_wr_IDT    <= ACK_addr;
                            end
                            
                            if(cnt_ACK == 9'd300) begin
                                cnt_ACK       <= 9'd0;
                                fsm_wr_IDT    <= IDLE;
                            end
                            else begin 
                                cnt_ACK       <= cnt_ACK + 9'd1;
                            end
                        end
            Write_byte: begin
                            if(`SCL_LOW) begin
                                if(sda_num == 4'd8) begin
                                    sda_num       <= 4'd0;
                                    sda           <= 1'b1;
                                    sda_link      <= 1'b0;
                                    fsm_wr_IDT    <= ACK_byte;
                                    cnt_ACK       <= 9'd0;
                                end
                                else begin
                                    fsm_wr_IDT    <= Write_byte;
                                    sda_num       <= sda_num + 4'b1;
                                    case (sda_num)
                                        4'd0: sda <= WRITE_BYTE[cnt_byte][7];
                                        4'd1: sda <= WRITE_BYTE[cnt_byte][6];
                                        4'd2: sda <= WRITE_BYTE[cnt_byte][5];
                                        4'd3: sda <= WRITE_BYTE[cnt_byte][4];
                                        4'd4: sda <= WRITE_BYTE[cnt_byte][3];
                                        4'd5: sda <= WRITE_BYTE[cnt_byte][2];
                                        4'd6: sda <= WRITE_BYTE[cnt_byte][1];
                                        4'd7: sda <= WRITE_BYTE[cnt_byte][0];
                                        default: ;
                                    endcase
                                end
                            end
                            else begin
                                fsm_wr_IDT    <= Write_byte;
                            end
                        end
            ACK_byte:   begin
                            if(!o_IDT_8T_SDA) begin
                                sda_link      <= 1'b1;
                                fsm_wr_IDT    <= Write_byte;
                                cnt_byte      <= cnt_byte + 5'd1;
                                if(cnt_byte == 5'd24)begin
                                    fsm_wr_IDT    <= STOP;
                                    cnt_byte      <= 5'd0;
                                end
                            end
                            else begin
                                fsm_wr_IDT <= ACK_byte;
                            end
                            if(cnt_ACK == 9'd300) begin
                                cnt_ACK       <= 9'd0;
                                fsm_wr_IDT    <= IDLE;
                            end
                            else begin
                                cnt_ACK       <= cnt_ACK + 9'd1;
                            end
                        end
            STOP:   begin
                        if(`SCL_LOW) begin
                            sda_link      <= 1'b1;
                            sda           <= 1'b0;
                            fsm_wr_IDT    <= STOP;
                        end
                        else if(`SCL_HIG) begin
                            sda_link      <= 1'b1;
                            sda           <= 1'b1;
                            fsm_wr_IDT    <= IDLE;
                        end
                        else begin
                            fsm_wr_IDT    <= STOP;
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

endmodule
