`timescale 1ns / 1ps
//--------------------------------------------------------------------------------------------------------------
//             -- Copyright (c) 2012 Gpixel, Technology Co. Ltd. --
//             --         All Rights Reserved.                   --
//---------------------------------------------------------------------------------------------------------------
// --Module    : rx_pkg_decoder
// --File      : rx_pkg_decoder.v
// --Author    : 
//---------------------------------------------------------------------------------------------------------------
// --Function  : decoder PC protocols. remove package and output FPGA and Sensor protocols
// --------------------------------------------------------------------------------------------------------------
// --History   :
// --Ver       Date         By              Description
// --1.0       2022.07.24   Tianjiao Hou      create
//---------------------------------------------------------------------------------------------------------------
module rx_pkg_decoder(
    input                        clk                     , // same to 'clk' of usb30_interface.v/cl_serial.v,tx_pkg_encoder.v
    input                        rst                     , // same to 'rst' of usb30_interface.v/cl_serial.v,tx_pkg_encoder.v
    input             [31:0]     i_rx_fifo_rd_dat        , // Read latency of usb/cl receive FIFO  : 1 
    output    reg                o_rx_fifo_rd_en         , // rd_en to usb/cl receive FIFO 
    input                        i_rx_fifo_rd_empty      , // empty flag of usb/cl receive FIFO 
    output    reg     [7:0]      o_ack_content           , // 0:without ACK operation,1:ACK will based on ACK Content. //detail refer to <FPGA-PC Comunication Protocol>
    output    reg     [7:0]      o_ack_mode              , // 0:ACK without result,1:ACK with result. //detail refer to <FPGA-PC Comunication Protocol>
    output    reg                o_prot_sensor_vld       , // Sensor protocol, valid of address and data
    output    reg     [15:0]     o_prot_sensor_addr      , // Sensor protocol, address
    output    reg     [15:0]     o_prot_sensor_dat       , // Sensor protocol, data
    output    reg                o_prot_fpga_vld         , // FPGA protocol, valid of address and data
    output    reg     [15:0]     o_prot_fpga_addr        , // FPGA protocol, address
    output    reg     [15:0]     o_prot_fpga_dat           // FPGA protocol, data
);

//------------------------------------------------------------------------------------------------------------------
//parameter
//------------------------------------------------------------------------------------------------------------------

localparam    s_idle            = 3'd0,
              s_pkg_header      = 3'd1,
              s_rd_infor        = 3'd2,
              s_pkg_infor       = 3'd3,
              s_rd_dat          = 3'd4,
              s_data            = 3'd5,
              s_sum_check       = 3'd6,
              s_dat_rst         = 3'd7;
localparam    s_f_idle          = 1'b0,
              s_f_rd            = 1'b1;
localparam    s_s_idle          = 1'b0,
              s_s_rd            = 1'b1; 
localparam    wait_time         = 32'd10000000;//100ms

//------------------------------------------------------------------------------------------------------------------
//Reg & wire Define
//------------------------------------------------------------------------------------------------------------------
reg     [31:0]    rx_fifo_rd_dat_dly   ;
reg     [2:0]     cs_decoder           ;
reg     [2:0]     ns_decoder           ;
reg               state_next           ;
reg               state_back           ;
reg               state_jump           ;
reg               tmp_sensor_dat_valid ;
reg               tmp_fpga_dat_valid   ;
reg               dat_rst              ;
reg     [3:0]     cnt_dat_rst          ;
reg     [31:0]    sum_check            ;
reg               fpga_fifo_wren       ;
reg               sensor_fifo_wren     ; 
reg     [31:0]    cnt_wait_rec_data    ;
reg     [15:0]    dat_length           ;
reg     [15:0]    cnt_dat_length       ;
reg               fpga_load_valid      ;
reg               sensor_load_valid    ;
//-----------fpga decoder---------------
reg               cs_f_rd              ;
reg               ns_f_rd              ;
reg               state_f_next         ; 
reg               fpga_dat_rden        ;
wire    [31:0]    fpga_fifo_datout     ;
wire              fpga_dat_empty       ;
wire    [13:0]    fpga_fifo_dat_count  ;
//-----------sensor decoder-------------
reg              cs_s_rd               ;
reg              ns_s_rd               ;
reg              state_s_next          ;
reg              sensor_dat_rden       ;
wire    [31:0]   sensor_fifo_datout    ;
wire             sensor_dat_empty      ;
wire    [13:0]   sensor_fifo_dat_count ;

//------------------------------------------------------------------------------------------------------------------
//Main code
//------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------Frame decoder ---------------------------------------------------
always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin 
        cs_decoder <= s_idle;
    end 
    else begin
        cs_decoder <= ns_decoder;
    end
end


always @(*) begin
    case(cs_decoder)
        s_idle:         begin
                            if(state_next==1'b1) begin
                               ns_decoder           = s_pkg_header;
                            end
                            else begin
                                ns_decoder          = s_idle;
                            end
                        end
        s_pkg_header:   begin
                            if(state_next==1'b1) begin
                               ns_decoder           = s_rd_infor;
                            end
                            else if(state_back==1'b1) begin
                               ns_decoder           = s_dat_rst;
                            end
                            else begin
                                ns_decoder          = s_pkg_header;
                            end
                        end 
        s_rd_infor:     begin
                            if(state_next==1'b1) begin
                               ns_decoder           = s_pkg_infor;
                            end
                            else if(state_back==1'b1) begin
                               ns_decoder           = s_dat_rst;
                            end
                            else begin
                                ns_decoder          = s_rd_infor;
                            end
                        end 
        s_pkg_infor:    begin
                            ns_decoder           = s_rd_dat;
                        end 
        s_rd_dat:       begin
                            if(state_next==1'b1) begin
                               ns_decoder           = s_data;
                            end
                            else if(state_jump==1'b1) begin
                               ns_decoder           = s_sum_check;
                            end
                            else if(state_back==1'b1) begin
                               ns_decoder           = s_dat_rst;
                            end
                            else begin
                                ns_decoder          = s_rd_dat;
                            end
                        end 
        s_data:         begin
                            ns_decoder           = s_rd_dat;
                        end 
        s_sum_check:    begin
                            if(state_next==1'b1) begin
                               ns_decoder           = s_idle;
                            end
                            else if(state_back==1'b1) begin
                               ns_decoder           = s_dat_rst;
                            end
                            else begin
                                ns_decoder          = s_sum_check;
                            end
                        end 
        s_dat_rst:      begin
                            if(state_next==1'b1) begin
                               ns_decoder           = s_idle;
                            end
                            else begin
                                ns_decoder          = s_dat_rst;
                            end
                        end
    endcase
end

always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        o_rx_fifo_rd_en       <= 1'b0;
        o_ack_content         <= 'd0;
        o_ack_mode            <= 'd0;
        state_next            <= 1'b0;
        state_back            <= 1'b0;
        state_jump            <= 1'b0;
        dat_rst               <= 1'b0;
        cnt_dat_rst           <= 'd0;
        fpga_fifo_wren        <= 1'b0;
        sensor_fifo_wren      <= 1'b0;
        tmp_sensor_dat_valid  <= 1'b0;
        tmp_fpga_dat_valid    <= 1'b0;
        sum_check             <= 'd0;
        cnt_dat_length        <= 'd0;
        fpga_load_valid       <= 1'b0;
        sensor_load_valid     <= 1'b0;
        dat_length            <= 'd0;
        cnt_wait_rec_data     <= 'd0;
    end
    else begin
        case(cs_decoder)
            s_idle:         begin
                                if(state_next==1'b0) begin
                                    if(i_rx_fifo_rd_empty==1'b0) begin
                                        o_rx_fifo_rd_en          <= 1'b1;
                                        state_next               <= 1'b1;
                                    end
                                    else begin
                                        o_rx_fifo_rd_en      <= 1'b0;
                                    end 
                                    tmp_sensor_dat_valid     <= 1'b0;
                                    tmp_fpga_dat_valid       <= 1'b0;
                                    sum_check                <= 'd0;
                                    fpga_load_valid          <= 1'b0;
                                    sensor_load_valid        <= 1'b0;
                                    cnt_dat_length           <= 'd0;
                                    fpga_fifo_wren           <= 1'b0;
                                    sensor_fifo_wren         <= 1'b0;
                                    cnt_wait_rec_data        <= 'd0;
                                    dat_rst                  <= 1'b0;
                                    cnt_dat_rst              <= 'd0;
                                end
                                else begin
                                    state_next               <= 1'b0;
                                    o_rx_fifo_rd_en          <= 1'b0;
                                end
                            end
            s_pkg_header:    begin
                                if((state_next==1'b0)&&(state_back==1'b0)) begin 
                                    if(i_rx_fifo_rd_dat[15:0] == 16'hAA55) begin
                                        state_next               <= 1'b1;
                                        if(i_rx_fifo_rd_dat[23:16] == 8'd0) begin
                                            tmp_sensor_dat_valid <= 1'b0;
                                            tmp_fpga_dat_valid   <= 1'b1;
                                        end
                                        else begin
                                            tmp_sensor_dat_valid <= 1'b1;
                                            tmp_fpga_dat_valid   <= 1'b0;
                                        end
                                        sum_check                <= sum_check + i_rx_fifo_rd_dat;
                                    end
                                    else begin
                                        sum_check            <= 'd0;
                                        state_back           <= 1'b1;
                                    end
                                end
                                else begin
                                    state_next               <= 1'b0;
                                    state_back               <= 1'b0; 
                                end
                            end
            s_rd_infor:     begin
                                if((state_next==1'b0)&&(state_back==1'b0)) begin 
                                    if(i_rx_fifo_rd_empty==1'b0) begin
                                        state_next             <= 1'b1;
                                        o_rx_fifo_rd_en        <= 1'b1;
                                        cnt_wait_rec_data      <= 'd0;
                                    end
                                    else begin
                                        if( cnt_wait_rec_data >= wait_time)begin
                                            state_back             <= 1'b1;
                                            dat_rst                <= 1'b1;
                                            cnt_wait_rec_data      <= 'd0;
                                        end
                                        else begin
                                            cnt_wait_rec_data      <= cnt_wait_rec_data + 1'b1;
                                        end
                                    end
                                end
                                else begin
                                    state_next               <= 1'b0;
                                    state_back               <= 1'b0;
                                    o_rx_fifo_rd_en          <= 1'b0;
                                end
                            end
            s_pkg_infor:    begin
                                o_ack_content            <= i_rx_fifo_rd_dat[31:24];
                                o_ack_mode               <= i_rx_fifo_rd_dat[23:16];
                                dat_length               <= i_rx_fifo_rd_dat[15:0];
                                sum_check                <= sum_check + i_rx_fifo_rd_dat;
                                cnt_dat_length           <= 'd0;
                            end
            s_rd_dat:       begin
                                if((state_next==1'b0)&&(state_back==1'b0)&&(state_jump==1'b0)) begin 
                                    fpga_fifo_wren       <= 1'b0;
                                    sensor_fifo_wren     <= 1'b0;
                                    if(i_rx_fifo_rd_empty==1'b0) begin
                                        o_rx_fifo_rd_en        <= 1'b1;
                                        cnt_wait_rec_data      <= 'd0;
                                        if( cnt_dat_length >= dat_length)begin
                                            state_jump             <= 1'b1;
                                            cnt_dat_length         <= 'd0;
                                        end
                                        else begin
                                            state_next             <= 1'b1;
                                            cnt_dat_length         <= cnt_dat_length + 1'b1;
                                        end
                                    end
                                    else begin
                                        if( cnt_wait_rec_data >= wait_time)begin
                                            state_back             <= 1'b1;
                                            dat_rst                <= 1'b1;
                                            cnt_wait_rec_data      <= 'd0;
                                        end
                                        else begin
                                            cnt_wait_rec_data      <= cnt_wait_rec_data + 1'b1;
                                        end
                                    end
                                end
                                else begin
                                    state_next               <= 1'b0;
                                    state_jump               <= 1'b0;
                                    state_back               <= 1'b0;
                                    o_rx_fifo_rd_en          <= 1'b0;
                                end
                            end
            s_data:         begin
                                fpga_fifo_wren       <= tmp_fpga_dat_valid;
                                sensor_fifo_wren     <= tmp_sensor_dat_valid;
                                sum_check            <= sum_check + i_rx_fifo_rd_dat;
                            end
            s_sum_check:    begin
                                if((state_next==1'b0)&&(state_back==1'b0)) begin 
                                    if(sum_check == i_rx_fifo_rd_dat) begin
                                        state_next           <= 1'b1;
                                        fpga_load_valid      <= tmp_fpga_dat_valid ;
                                        sensor_load_valid    <= tmp_sensor_dat_valid;
                                    end
                                    else begin
                                        dat_rst              <= 1'b1;
                                        state_back           <= 1'b1;
                                    end
                                end
                                else begin
                                    state_next               <= 1'b0;
                                    state_back               <= 1'b0;
                                    fpga_load_valid          <= 1'b0;
                                    sensor_load_valid        <= 1'b0;
                                end
                            end
            s_dat_rst:      begin
                                if(state_next==1'b0) begin
                                    if( cnt_dat_rst >= 'd13)begin
                                        state_next             <= 1'b1; 
                                        dat_rst                <= 1'b0; 
                                    end
                                    else begin
                                        cnt_dat_rst            <= cnt_dat_rst + 1'b1;
                                    end
                                end
                                else begin
                                    state_next               <= 1'b0; 
                                end
                            end
        endcase
    end

end

//-----------------------------------Read valid FPGA data to fpga decoder---------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin 
        cs_f_rd <= s_f_idle;
    end 
    else begin
        cs_f_rd <= ns_f_rd;
    end
end

always @(*) begin
    case(cs_f_rd)
        s_f_idle:       begin
                            if(state_f_next==1'b1) begin 
                                ns_f_rd    = s_f_rd;
                            end
                            else begin
                                ns_f_rd    = s_f_idle;
                            end
                        end
        s_f_rd:         begin
                            if(state_f_next==1'b1) begin 
                                ns_f_rd    = s_f_idle;
                            end
                            else begin
                                ns_f_rd    = s_f_rd;
                            end
                        end  
    endcase
end

always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        o_prot_fpga_vld     <= 1'b0;
        o_prot_fpga_addr    <= 'd0;
        o_prot_fpga_dat     <= 'd0;
        fpga_dat_rden       <= 1'b0;
        state_f_next        <= 1'b0;
    end
    else begin
        case(cs_f_rd)
            s_f_idle:   begin
                            if(state_f_next==1'b0) begin 
                                if(fpga_load_valid == 1'b1 && fpga_dat_empty == 1'b0) begin
                                    state_f_next        <= 1'b1;
                                    fpga_dat_rden       <= 1'b1;
                                end 
                                else begin
                                    o_prot_fpga_vld     <= 1'b0; 
                                    fpga_dat_rden       <= 1'b0;
                                end 
                            end
                            else begin
                                if(fpga_fifo_dat_count <= 'd1) begin
                                    fpga_dat_rden          <= 1'b0;
                                end
                                else begin
                                    state_f_next           <= 1'b0;
                                end
                            end
                        end
            s_f_rd:     begin
                            o_prot_fpga_addr        <= fpga_fifo_datout[31:16];
                            o_prot_fpga_dat         <= fpga_fifo_datout[15:0];
                            o_prot_fpga_vld         <= 1'b1;
                            if(state_f_next==1'b0) begin 
                                if(fpga_fifo_dat_count <= 'd1)begin
                                    state_f_next            <= 1'b1;
                                    fpga_dat_rden           <= 1'b0;
                                end
                                else begin
                                    fpga_dat_rden           <= 1'b1;
                                end
                            end
                            else begin
                                state_f_next             <= 1'b0;
                            end
                        end
        endcase
    end

end

//-----------------------------------Read valid sensor data to sensor decoder---------------------------------

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin 
        cs_s_rd <= s_s_idle;
    end 
    else begin
        cs_s_rd <= ns_s_rd;
    end
end

always @(*) begin
    case(cs_s_rd)
        s_s_idle:       begin
                            if(state_s_next==1'b1) begin 
                                ns_s_rd    = s_s_rd;
                            end
                            else begin
                                ns_s_rd    = s_s_idle;
                            end
                        end
        s_s_rd:         begin
                            if(state_s_next==1'b1) begin 
                                ns_s_rd    = s_s_idle;
                            end
                            else begin
                                ns_s_rd    = s_s_rd;
                            end
                        end  
    endcase
end


always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        o_prot_sensor_vld     <= 1'b0;
        o_prot_sensor_addr    <= 'd0;
        o_prot_sensor_dat     <= 'd0;
        sensor_dat_rden       <= 1'b0;
        state_s_next          <= 1'b0;
    end
    else begin
        case(cs_s_rd)
            s_s_idle:   begin
                            if(state_s_next==1'b0) begin 
                                if(sensor_load_valid == 1'b1 && sensor_dat_empty == 1'b0) begin
                                    state_s_next          <= 1'b1;
                                    sensor_dat_rden       <= 1'b1;
                                end
                                else begin
                                    o_prot_sensor_vld     <= 1'b0;
                                    sensor_dat_rden       <= 1'b0;
                                end
                            end
                            else begin
                                if(sensor_fifo_dat_count <= 'd1) begin
                                    sensor_dat_rden          <= 1'b0;
                                end
                                else begin
                                    state_s_next             <= 1'b0;
                                end
                            end
                        end
            s_f_rd:     begin
                            o_prot_sensor_addr    <= sensor_fifo_datout[31:16];
                            o_prot_sensor_dat     <= sensor_fifo_datout[15:0];
                            o_prot_sensor_vld     <= 1'b1;
                            if(state_s_next==1'b0) begin 
                                if(sensor_fifo_dat_count <= 'd1)begin
                                    state_s_next           <= 1'b1;
                                    sensor_dat_rden        <= 1'b0;
                                end
                                else begin
                                    sensor_dat_rden        <= 1'b1;
                                end
                            end
                            else begin
                                state_s_next           <= 1'b0;
                            end
                        end
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin 
        rx_fifo_rd_dat_dly <= 'd0;
    end 
    else begin
        rx_fifo_rd_dat_dly <= i_rx_fifo_rd_dat;
    end
end
//--------------------------------------------------------------------------------------------------------------------
//Instance
//--------------------------------------------------------------------------------------------------------------------
//Basic:        Native 
//              Native Common Clock Block RAM
//Native Ports: Standard FIFO
//              Write/read width 32 bit
//              Depth 16384
//              Reset Pin : select
//              Reset Type: Asynchronous Resetselect 
//              Enable Safety Circuit: not select
//Data Counts:Data Count
 
fifo_w32d16384_ccbr_st fpga_fifo_w32d16384_ccbr_st (
    .clk            (clk                  ),
    .rst            (rst|dat_rst          ),
    .din            (rx_fifo_rd_dat_dly   ),
    .wr_en          (fpga_fifo_wren       ),
    .rd_en          (fpga_dat_rden        ),
    .dout           (fpga_fifo_datout     ),
    .full           (                     ),
    .empty          (fpga_dat_empty       ),
    .data_count     (fpga_fifo_dat_count  )
);

fifo_w32d16384_ccbr_st sensor_fifo_w32d16384_ccbr_st (
    .clk            (clk                  ),
    .rst            (rst|dat_rst          ),
    .din            (rx_fifo_rd_dat_dly   ),
    .wr_en          (sensor_fifo_wren     ),
    .rd_en          (sensor_dat_rden      ),
    .dout           (sensor_fifo_datout   ),
    .full           (                     ),
    .empty          (sensor_dat_empty     ),
    .data_count     (sensor_fifo_dat_count)
);

endmodule
