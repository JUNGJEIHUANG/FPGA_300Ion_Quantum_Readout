`timescale 1ns/1ps
// Observation-only count-boundary instrumentation. It does not control the image path.
// Lane input now means exact sync-filter-to-lane-FIFO input pins.
module rough_word_counter (
    input wire clk, rst, start, valid,
    input wire [31:0] expected,
    output reg first_toggle, last_toggle,
    output wire first_pulse, last_pulse
);
    reg active;
    reg [31:0] count, target;
    wire [31:0] next_count = count + 32'd1;
    assign first_pulse = !rst && !start && active && valid && (count == 0);
    assign last_pulse = !rst && !start && active && valid && (target != 0) && (next_count == target);
    always @(posedge clk) begin
        if (rst) begin
            active <= 0; count <= 0; target <= 0;
            first_toggle <= 0; last_toggle <= 0;
        end else if (start) begin
            active <= 1; count <= 0; target <= expected;
        end else if (active && valid) begin
            count <= next_count;
            if (count == 0) first_toggle <= ~first_toggle;
            if ((target != 0) && (next_count == target)) last_toggle <= ~last_toggle;
        end
    end
endmodule

// Counts each physical lane independently: skewed writes are not merged into
// fictitious aligned beats. P2 is the first write on any lane; P3 is the edge
// completing the expected per-lane count on ALL lanes. A missing lane cannot
// produce P3. Counts saturate after completion. No effect on FIFO controls.
module lane_write_events(
    input wire clk, rst, start,
    input wire [15:0] wr_en, fifo_rst,
    input wire [31:0] expected_per_lane,
    output reg first_toggle, last_toggle,
    output wire first_pulse, last_pulse
);
    reg active, first_seen;
    reg [31:0] target_last;
    reg [15:0] done;
    wire [15:0] hit;
    wire [15:0] write_now = wr_en & ~fifo_rst;
    assign first_pulse = !rst && !start && active && !first_seen && (|write_now);
    assign last_pulse = !rst && !start && active && !(&done) && (&(done | hit));
    genvar lane;
    generate for(lane=0;lane<16;lane=lane+1) begin: g_lane
        reg [31:0] count;
        assign hit[lane] = active && !done[lane] && write_now[lane] && (count == target_last);
        always @(posedge clk) begin
            if(rst || start) count <= 0;
            else if(active && !done[lane] && write_now[lane]) count <= count + 1'b1;
        end
    end endgenerate
    always @(posedge clk) begin
        if(rst) begin
            active<=0; first_seen<=0; target_last<=0; done<=0;
            first_toggle<=0; last_toggle<=0;
        end else if(start) begin
            active <= (expected_per_lane != 0); first_seen<=0;
            target_last<=expected_per_lane-1'b1; done<=0;
        end else if(active) begin
            done <= done | hit;
            if(first_pulse) begin first_seen<=1; first_toggle<=~first_toggle; end
            if(last_pulse) begin active<=0; last_toggle<=~last_toggle; end
        end
    end
endmodule

// Registered Gray source, two destination synchronizer stages. This is only
// an alignment tag, NEVER the clock or enable for actual bus acquisition.
// Destination ILA sees a source tick from approximately two local cycles ago.
// Common tick = 10 ns; host must disclose CDC / phase uncertainty.
module ila_timebase(
    input wire clk, rst, rx_clk, rx_rst, ui_clk, ui_rst,
    output reg [31:0] common_tick_sys,
    output wire [31:0] common_tick_rx, common_tick_ui,
    output reg [31:0] local_cycle_rx, local_cycle_ui
);
    reg [31:0] common_gray;
    wire [31:0] next_tick = common_tick_sys + 1'b1;
    (* ASYNC_REG="TRUE", SHREG_EXTRACT="NO" *) reg [31:0] rx_gray_meta, rx_gray_sync;
    (* ASYNC_REG="TRUE", SHREG_EXTRACT="NO" *) reg [31:0] ui_gray_meta, ui_gray_sync;
    function [31:0] gray_decode;
        input [31:0] gray;
        integer k;
        begin
            gray_decode[31]=gray[31];
            for(k=30;k>=0;k=k-1) gray_decode[k]=gray_decode[k+1]^gray[k];
        end
    endfunction
    assign common_tick_rx = gray_decode(rx_gray_sync);
    assign common_tick_ui = gray_decode(ui_gray_sync);
    always @(posedge clk) begin
        if(rst) begin common_tick_sys<=0; common_gray<=0; end
        else begin common_tick_sys<=next_tick; common_gray<=next_tick^(next_tick>>1); end
    end
    always @(posedge rx_clk) begin
        if(rx_rst) begin rx_gray_meta<=0; rx_gray_sync<=0; local_cycle_rx<=0; end
        else begin rx_gray_meta<=common_gray; rx_gray_sync<=rx_gray_meta; local_cycle_rx<=local_cycle_rx+1'b1; end
    end
    always @(posedge ui_clk) begin
        if(ui_rst) begin ui_gray_meta<=0; ui_gray_sync<=0; local_cycle_ui<=0; end
        else begin ui_gray_meta<=common_gray; ui_gray_sync<=ui_gray_meta; local_cycle_ui<=local_cycle_ui+1'b1; end
    end
endmodule

// One 100 MHz system-clock ILA. Native-domain atomic trace packets are
// transferred through independent XPM asynchronous FIFOs. Not direct async sampling.
// Internal word counts and AA55 parsing are required to identify last beats.
module g6504_rough_ila #(parameter integer SAMPLE_DIV = 100000)(
    input wire clk, rst, vsync,
    input wire rx_clk, rx_rst, rx_start, ddr_in_valid,
    input wire [15:0] lane_fifo_wen, lane_fifo_rst,
    input wire ui_clk, ui_rst, request_toggle, ddr_out_valid,
    input wire [31:0] expected_fx3_words,
    input wire fx3_strobe,
    input wire [31:0] fx3_data,
    input wire [191:0] lane_fifo_din,
    input wire [383:0] ddr_in_data, ddr_out_data,
    input wire ddr_read_request, ddr_calibrated,
    input wire ddr_read_request_sys, fx3_flagb, fx3_slcs_n, fx3_slwr_n,
    input wire fx3_slrd_n, fx3_sloe_n, fx3_pktend_n,
    input wire [1:0] fx3_addr
);
    wire [31:0] expected_lvds = expected_fx3_words >> 3;
    wire [31:0] expected_ddr = expected_fx3_words >> 4;
    wire lf_t, ll_t, di_f_t, di_l_t, do_f_t, do_l_t;
    wire p2_lvds_first_rx, p3_lvds_last_rx, p4_ddr_in_first_rx, p5_ddr_in_last_rx;
    wire p6_ddr_out_first_ui, p7_ddr_out_last_ui;

    (* ASYNC_REG = "TRUE" *) reg [2:0] req_sync;
    wire ui_start = req_sync[2] ^ req_sync[1];
    always @(posedge ui_clk)
        if (ui_rst) req_sync <= 0;
        else req_sync <= {req_sync[1:0], request_toggle};

    lane_write_events lane_counter(rx_clk, rx_rst, rx_start, lane_fifo_wen, lane_fifo_rst,
        expected_lvds, lf_t, ll_t, p2_lvds_first_rx, p3_lvds_last_rx);
    rough_word_counter ddr_in_counter(rx_clk, rx_rst, rx_start, ddr_in_valid,
        expected_ddr, di_f_t, di_l_t, p4_ddr_in_first_rx, p5_ddr_in_last_rx);
    rough_word_counter ddr_out_counter(ui_clk, ui_rst, ui_start, ddr_out_valid,
        expected_ddr, do_f_t, do_l_t, p6_ddr_out_first_ui, p7_ddr_out_last_ui);

    (* ASYNC_REG = "TRUE" *) reg [5:0] evt_meta, evt_sync;
    reg [5:0] evt_prev;
    always @(posedge clk) begin
        if (rst) begin
            evt_meta <= 0; evt_sync <= 0; evt_prev <= 0;
        end else begin
            evt_meta <= {do_f_t, di_f_t, do_l_t, di_l_t, ll_t, lf_t};
            evt_sync <= evt_meta;
            evt_prev <= evt_sync;
        end
    end


    reg vsync_d, started;
    reg [31:0] expected_latched, fx3_count;
    reg [31:0] header_geometry, header_frames;
    reg [2:0] header_state;

    wire [5:0] cross_events = evt_sync ^ evt_prev;
    wire p0_vsync_rise = vsync && !vsync_d;
    wire p1_vsync_fall = started && !vsync && vsync_d;
    wire p2_lvds_first = started && cross_events[0];
    wire p3_lvds_last = started && cross_events[1];
    wire p4_ddr_in_first = started && cross_events[4];
    wire p5_ddr_in_last = started && cross_events[2];
    wire p6_ddr_out_first = started && cross_events[5];
    wire p7_ddr_out_last = started && cross_events[3];
    wire payload_write = started && fx3_strobe && (header_state == 5);
    wire p8_fx3_first = payload_write && (fx3_count == 0);
    wire p9_fx3_last = payload_write && (expected_latched != 0) &&
                         (fx3_count == expected_latched - 1);
    wire [9:0] events = {p9_fx3_last, p8_fx3_first, p7_ddr_out_last,
                         p6_ddr_out_first, p5_ddr_in_last, p4_ddr_in_first,
                         p3_lvds_last, p2_lvds_first, p1_vsync_fall, p0_vsync_rise};

    reg event_d;
    reg [31:0] sample_divider;
    wire sample_tick = started && (sample_divider == SAMPLE_DIV-1);
    wire event_capture_enable = (|events) || event_d || sample_tick;

    always @(posedge clk) begin
        if (rst) begin
            vsync_d <= 0; event_d <= 0;
        end else begin
            vsync_d <= vsync; event_d <= |events;
        end
        if (rst || p0_vsync_rise) begin
            started <= !rst;
            expected_latched <= rst ? 0 : expected_fx3_words;
            fx3_count <= 0;
            header_geometry <= 0; header_frames <= 0;
            header_state <= 0;
            sample_divider <= 0;
        end else if (started) begin
            sample_divider <= sample_tick ? 0 : sample_divider + 1;
            if (fx3_strobe) begin
                case (header_state)
                    0: if (fx3_data == 32'h0000AA55) header_state <= 1;
                    1: if (fx3_data == 32'h86000000) header_state <= 2;
                       else header_state <= (fx3_data == 32'h0000AA55) ? 1 : 0;
                    2: begin header_geometry <= fx3_data; header_state <= 3; end
                    3: begin header_frames <= fx3_data; header_state <= 4; end
                    4: begin
                        if (fx3_data == 12 && header_geometry[15:0] != 0 &&
                            header_geometry[31:16] != 0 && header_frames[15:0] != 0 &&
                            expected_latched != 0)
                            header_state <= 5;
                        else header_state <= 0;
                    end
                    5: begin
                        fx3_count <= fx3_count + 1;
                        if (p9_fx3_last) header_state <= 6;
                    end
                    6: begin end // Ignore writes after the expected payload.
                    default: header_state <= 0;
                endcase
            end
        end
    end

    wire [31:0] common_tick_sys, common_tick_rx, common_tick_ui;
    wire [31:0] local_cycle_rx, local_cycle_ui;
    ila_timebase u_timebase(clk, rst, rx_clk, rx_rst, ui_clk, ui_rst,
        common_tick_sys, common_tick_rx, common_tick_ui, local_cycle_rx, local_cycle_ui);

    wire [9:0] sys_events=events & 10'h303;
    wire [9:0] rx_events={4'b0,p5_ddr_in_last_rx,p4_ddr_in_first_rx,
                          p3_lvds_last_rx,p2_lvds_first_rx,2'b0};
    wire [9:0] ui_events={2'b0,p7_ddr_out_last_ui,p6_ddr_out_first_ui,6'b0};
    wire [10:0] fx3_controls={ddr_read_request_sys,payload_write,fx3_addr,
        fx3_pktend_n,fx3_sloe_n,fx3_slrd_n,fx3_slwr_n,fx3_slcs_n,fx3_flagb,fx3_strobe};
    wire [85:0] sys_packet={common_tick_sys,sys_events,vsync,fx3_data,fx3_controls};
    wire [682:0] rx_packet={common_tick_rx,local_cycle_rx,rx_events,
        lane_fifo_din,lane_fifo_wen,lane_fifo_rst,ddr_in_data,ddr_in_valid};
    wire [460:0] ui_packet={common_tick_ui,local_cycle_ui,ui_events,
        ddr_out_data,ddr_out_valid,ddr_read_request,ddr_calibrated};
    wire [85:0] sys_sample;
    wire [682:0] rx_sample;
    wire [460:0] ui_sample;
    wire [2:0] record_valid,source_done,source_fault;
    wire capture_enable,frame_trigger,padding,frame_end;
    wire [11:0] sample_index;
    wire [9:0] record_events;
    wire [7:0] status_error;
    single_ila_transport u_transport(
        .clk(clk),.rst(rst),.rx_clk(rx_clk),.rx_rst(rx_rst),.ui_clk(ui_clk),.ui_rst(ui_rst),
        .sys_start(p0_vsync_rise),.rx_start(rx_start),.ui_start(ui_start),
        .sys_events(sys_events),.rx_events(rx_events),.ui_events(ui_events),
        .sys_packet(sys_packet),.rx_packet(rx_packet),.ui_packet(ui_packet),
        .sys_sample(sys_sample),.rx_sample(rx_sample),.ui_sample(ui_sample),
        .record_valid(record_valid),.capture_enable(capture_enable),.frame_trigger(frame_trigger),
        .padding(padding),.frame_end(frame_end),.sample_index(sample_index),
        .record_events(record_events),.source_done(source_done),.source_fault(source_fault),
        .status_error(status_error));
    // Each group's data/time changes only with its corresponding record_valid bit.
    // record_events are one-cycle native events transported with the same data.
    wire [0:0] trace_VSYNC=sys_sample[43];
    wire [31:0] trace_FX3_data=sys_sample[42:11];
    wire [10:0] trace_FX3_controls=sys_sample[10:0];
    wire [31:0] trace_SYS_source_tick=sys_sample[85:54];
    wire [191:0] trace_lane_fifo_din=rx_sample[608:417];
    wire [15:0] trace_lane_fifo_wen=rx_sample[416:401];
    wire [15:0] trace_lane_fifo_rst=rx_sample[400:385];
    wire [383:0] trace_DDR_in_data=rx_sample[384:1];
    wire [0:0] trace_DDR_in_valid=rx_sample[0];
    wire [31:0] trace_RX_source_tick=rx_sample[682:651];
    wire [31:0] trace_RX_source_cycle=rx_sample[650:619];
    wire [383:0] trace_DDR_out_data=ui_sample[386:3];
    wire [0:0] trace_DDR_out_valid=ui_sample[2];
    wire [0:0] trace_DDR_read_request=ui_sample[1];
    wire [0:0] trace_DDR_calibrated=ui_sample[0];
    wire [31:0] trace_UI_source_tick=ui_sample[460:429];
    wire [31:0] trace_UI_source_cycle=ui_sample[428:397];
    ila_five_sys u_ila_single(.clk(clk),
        .probe0(record_valid),
        .probe1(capture_enable),
        .probe2(frame_trigger),
        .probe3(padding),
        .probe4(status_error),
        .probe5(sample_index),
        .probe6(record_events[0]),
        .probe7(record_events[1]),
        .probe8(record_events[2]),
        .probe9(record_events[3]),
        .probe10(record_events[4]),
        .probe11(record_events[5]),
        .probe12(record_events[6]),
        .probe13(record_events[7]),
        .probe14(record_events[8]),
        .probe15(record_events[9]),
        .probe16(trace_VSYNC),
        .probe17(trace_FX3_data),
        .probe18(trace_FX3_controls),
        .probe19(trace_SYS_source_tick),
        .probe20(trace_lane_fifo_din),
        .probe21(trace_lane_fifo_wen),
        .probe22(trace_lane_fifo_rst),
        .probe23(trace_DDR_in_data),
        .probe24(trace_DDR_in_valid),
        .probe25(trace_RX_source_tick),
        .probe26(trace_RX_source_cycle),
        .probe27(trace_DDR_out_data),
        .probe28(trace_DDR_out_valid),
        .probe29(trace_DDR_read_request),
        .probe30(trace_DDR_calibrated),
        .probe31(trace_UI_source_tick),
        .probe32(trace_UI_source_cycle),
        .probe33(source_done),
        .probe34(source_fault),
        .probe35(frame_end)
    );
endmodule
