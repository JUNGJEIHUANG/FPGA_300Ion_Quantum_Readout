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
module des_8bit_to_adcdepth#(
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
