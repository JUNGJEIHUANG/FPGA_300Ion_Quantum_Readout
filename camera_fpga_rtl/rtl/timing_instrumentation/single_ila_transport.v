`timescale 1ns/1ps
// Observation-only trace transport. Source packets are atomic FIFO words.
// Never connect FIFO full/ready back to the functional sensor/DDR/FX3 path.
module trace_source #(
    parameter integer WIDTH=86, BURST=256,
    parameter [9:0] EVENT_MASK=10'h303
)(
    input wire src_clk, src_rst, start,
    input wire [9:0] events,
    input wire [WIDTH-1:0] packet,
    input wire sys_clk, pop,
    output wire [WIDTH-1:0] head,
    output wire available,
    output reg done, fault,
    output wire fifo_full
);
    reg armed;
    reg [9:0] seen;
    reg [9:0] remaining;
    reg [5:0] reset_hold=6'h3f;
    wire [9:0] new_events=events & EVENT_MASK & ~(start ? 10'b0 : seen);
    wire [9:0] next_seen=(start ? 10'b0 : seen) | new_events;
    wire capture=(armed || start) && ((|new_events) || (!start && remaining!=0));
    wire wr_busy,rd_busy,empty;
    wire fifo_write=capture && !src_rst && !wr_busy && !fifo_full;
    always @(posedge src_clk) begin
        if(src_rst) reset_hold<=6'h3f;
        else reset_hold<={reset_hold[4:0],1'b0};
        if(src_rst) begin armed<=0; seen<=0; remaining<=0; done<=0; fault<=0; end
        else begin
            if(start) begin
                if(armed && !done) fault<=1; // overlapping frame is not a valid trace
                armed<=1; done<=0; seen<=new_events;
                remaining<=(|new_events) ? BURST-1 : 0;
            end else if(armed) begin
                seen<=next_seen;
                if(|new_events) remaining<=BURST-1;
                else if(remaining!=0) remaining<=remaining-1'b1;
                if(next_seen==EVENT_MASK && !(|new_events) && remaining==1) begin
                    done<=1; armed<=0;
                end
            end
            if(capture && (wr_busy || fifo_full)) fault<=1;
        end
    end
    assign available=!empty && !rd_busy;
    // XPM owns the Gray-pointer synchronizers and associated embedded constraints.
    // Every sampled bus, control, native event, and timestamp crosses together.
    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("block"), .ECC_MODE("no_ecc"),
        .FIFO_WRITE_DEPTH(1024), .WRITE_DATA_WIDTH(WIDTH), .READ_DATA_WIDTH(WIDTH),
        .READ_MODE("fwft"), .FIFO_READ_LATENCY(0), .CDC_SYNC_STAGES(3),
        .RELATED_CLOCKS(0), .DOUT_RESET_VALUE("0"), .FULL_RESET_VALUE(0),
        .WR_DATA_COUNT_WIDTH(11), .RD_DATA_COUNT_WIDTH(11),
        .PROG_FULL_THRESH(1000), .PROG_EMPTY_THRESH(10),
        .USE_ADV_FEATURES("0000"), .SIM_ASSERT_CHK(0)
    ) u_fifo (
        .rst(reset_hold[5]), .wr_clk(src_clk), .rd_clk(sys_clk),
        .din(packet), .wr_en(fifo_write), .rd_en(pop && available),
        .dout(head), .full(fifo_full), .empty(empty),
        .wr_rst_busy(wr_busy), .rd_rst_busy(rd_busy),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0),
        .almost_empty(), .almost_full(), .data_valid(), .overflow(),
        .prog_empty(), .prog_full(), .rd_data_count(), .sbiterr(),
        .dbiterr(), .underflow(), .wr_ack(), .wr_data_count()
    );
endmodule

module single_ila_transport #(
    parameter integer SYS_BURST=256, RX_BURST=256, UI_BURST=512,
    parameter integer CAPTURE_DEPTH=4096
)(
    input wire clk,rst,rx_clk,rx_rst,ui_clk,ui_rst,
    input wire sys_start,rx_start,ui_start,
    input wire [9:0] sys_events,rx_events,ui_events,
    input wire [85:0] sys_packet,
    input wire [682:0] rx_packet,
    input wire [460:0] ui_packet,
    output reg [85:0] sys_sample,
    output reg [682:0] rx_sample,
    output reg [460:0] ui_sample,
    output reg [2:0] record_valid,
    output reg capture_enable,frame_trigger,padding,frame_end,
    output reg [11:0] sample_index,
    output reg [9:0] record_events,
    output wire [2:0] source_done,source_fault,
    output wire [7:0] status_error
);
    wire [85:0] sys_head;
    wire [682:0] rx_head;
    wire [460:0] ui_head;
    wire [2:0] avail,done_raw,fault_raw,unused_full;
    trace_source #(86,SYS_BURST,10'h303) u_sys (
        clk,rst,sys_start,sys_events,sys_packet,clk,avail[0],
        sys_head,avail[0],done_raw[0],fault_raw[0],unused_full[0]);
    trace_source #(683,RX_BURST,10'h03c) u_rx (
        rx_clk,rx_rst,rx_start,rx_events,rx_packet,clk,avail[1],
        rx_head,avail[1],done_raw[1],fault_raw[1],unused_full[1]);
    trace_source #(461,UI_BURST,10'h0c0) u_ui (
        ui_clk,ui_rst,ui_start,ui_events,ui_packet,clk,avail[2],
        ui_head,avail[2],done_raw[2],fault_raw[2],unused_full[2]);
    (* ASYNC_REG="TRUE",SHREG_EXTRACT="NO" *) reg [2:0] done_meta,done_sync;
    (* ASYNC_REG="TRUE",SHREG_EXTRACT="NO" *) reg [2:0] fault_meta,fault_sync;
    always @(posedge clk) begin
        if(rst) begin done_meta<=0;done_sync<=0;fault_meta<=0;fault_sync<=0; end
        else begin done_meta<=done_raw;done_sync<=done_meta;
                   fault_meta<=fault_raw;fault_sync<=fault_meta; end
    end
    assign source_done=done_sync;
    assign source_fault=fault_sync;
    wire [9:0] arrival_events=(avail[0] ? sys_head[53:44] : 10'b0) |
                              (avail[1] ? rx_head[618:609] : 10'b0) |
                              (avail[2] ? ui_head[396:387] : 10'b0);
    wire p0_arrives=avail[0] && sys_head[44];
    reg active,pad_mode;
    reg [11:0] emitted;
    reg [9:0] seen_events;
    reg [4:0] quiet;
    reg overlap_fault,capacity_fault,orphan_fault;
    assign status_error={2'b0,orphan_fault,capacity_fault,overlap_fault,fault_sync};
    always @(posedge clk) begin
        if(rst) begin
            sys_sample<=0;rx_sample<=0;ui_sample<=0;record_valid<=0;
            capture_enable<=0;frame_trigger<=0;padding<=0;frame_end<=0;
            sample_index<=0;record_events<=0;active<=0;pad_mode<=0;
            emitted<=0;seen_events<=0;quiet<=0;
            overlap_fault<=0;capacity_fault<=0;orphan_fault<=0;
        end else begin
            record_valid<=0;capture_enable<=0;frame_trigger<=0;
            padding<=0;frame_end<=0;record_events<=0;
            if(|avail) quiet<=0;
            else if(quiet!=31) quiet<=quiet+1'b1;
            // Data remain held when record_valid[n]==0; held values are NOT
            // simultaneous observations of the five interfaces.
            if(avail[0]) sys_sample<=sys_head;
            if(avail[1]) rx_sample<=rx_head;
            if(avail[2]) ui_sample<=ui_head;
            if(p0_arrives && !active) begin
                active<=1;pad_mode<=0;emitted<=1;sample_index<=0;
                record_valid<=avail;capture_enable<=1;frame_trigger<=1;
                record_events<=arrival_events;seen_events<=arrival_events;
            end else if(active) begin
                if(p0_arrives) overlap_fault<=1;
                if(pad_mode) begin
                    if(|avail) overlap_fault<=1;
                    capture_enable<=1;padding<=1;sample_index<=emitted;
                    if(emitted==CAPTURE_DEPTH-1) begin
                        frame_end<=1;active<=0;pad_mode<=0;emitted<=0;
                    end else emitted<=emitted+1'b1;
                end else if(|avail) begin
                    record_valid<=avail;capture_enable<=1;sample_index<=emitted;
                    record_events<=arrival_events;seen_events<=seen_events|arrival_events;
                    if(emitted>=CAPTURE_DEPTH-2) begin
                        capacity_fault<=1;pad_mode<=1;emitted<=CAPTURE_DEPTH-1;
                    end else emitted<=emitted+1'b1;
                end else if((&done_sync) && seen_events==10'h3ff && quiet>=16)
                    pad_mode<=1;
            end else if(|avail) orphan_fault<=1;
        end
    end
endmodule
