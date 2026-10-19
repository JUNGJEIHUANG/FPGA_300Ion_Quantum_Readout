`timescale 1ns/1ps

module power_rst_seq #(
    parameter integer POWER_STEP = 6,
    parameter integer POWERON_SYS2CLK = 1000,
    parameter integer POWERON_CLK2DONE = 1000,
    parameter integer POWEROFF_SYS2CLK = 200,
    parameter integer POWEROFF_CLK2PWR = 200,
    parameter integer POWEROFF_STEP_TIME = 100000
)(
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         i_cmd_power_on,
    input  wire [POWER_STEP*32-1:0]     i_time_pwr_on_clk,
    input  wire [POWER_STEP*32-1:0]     i_time_pwr_off_clk,
    output reg  [POWER_STEP-1:0]        o_POWER_BUS,
    output reg                          o_SYS_RST_N,
    output reg                          o_clk_ref_en,
    output reg                          o_power_done
);

    localparam integer STEP_W = (POWER_STEP <= 2) ? 2 : $clog2(POWER_STEP + 3);
    localparam [2:0] ST_IDLE    = 3'd0,
                     ST_ON_RUN  = 3'd1,
                     ST_ON_DONE = 3'd2,
                     ST_OFF_RUN = 3'd3;

    reg [2:0] state_c;
    reg [2:0] state_n;
    reg [STEP_W-1:0] step;
    reg [31:0] cnt;
    reg power_on_d;

    wire power_on_rise = i_cmd_power_on & ~power_on_d;
    wire power_on_fall = ~i_cmd_power_on & power_on_d;
    wire [31:0] power_on_time [0:POWER_STEP-1];
    wire [31:0] power_off_time [0:POWER_STEP-1];

    genvar gi;
    generate
        for (gi = 0; gi < POWER_STEP; gi = gi + 1) begin : gen_power_on_time
            assign power_on_time[gi] = i_time_pwr_on_clk[(gi+1)*32-1:gi*32];
            assign power_off_time[gi] = i_time_pwr_off_clk[(gi+1)*32-1:gi*32];
        end
    endgenerate

    // FSM part 1: state register
    always @(posedge clk) begin
        if (rst) begin
            state_c <= ST_IDLE;
        end else begin
            state_c <= state_n;
        end
    end

    // FSM part 2: next-state combinational logic
    always @(*) begin


        case (state_c)
            ST_IDLE: begin
                if (power_on_rise) begin
                    state_n = ST_ON_RUN;
                end else begin
                    state_n = state_c;
                end
            end

            ST_ON_RUN: begin
                if (power_on_fall) begin
                    state_n = ST_OFF_RUN;
                end else if ((step > POWER_STEP) && (cnt >= POWERON_CLK2DONE-1)) begin
                    state_n = ST_ON_DONE;
                end else begin
                    state_n = state_c;
                end
            end

            ST_ON_DONE: begin
                if (power_on_fall) begin
                    state_n = ST_OFF_RUN;
                end else begin
                    state_n = state_c;
                end
            end

            ST_OFF_RUN: begin
                if (step >= POWER_STEP + 2) begin
                    state_n = ST_IDLE;
                end else begin
                    state_n = state_c;
                end
            end

            default: begin
                state_n = ST_IDLE;
            end
        endcase
    end

    // FSM part 3: output and datapath registers
    always @(posedge clk) begin
        if (rst) begin
            step         <= {STEP_W{1'b0}};
            cnt          <= 32'd0;
            power_on_d   <= 1'b0;
            o_POWER_BUS  <= {POWER_STEP{1'b0}};
            o_SYS_RST_N  <= 1'b0;
            o_clk_ref_en <= 1'b0;
            o_power_done <= 1'b0;
        end else begin
            power_on_d   <= i_cmd_power_on;

            case (state_c)
                ST_IDLE: begin
                    if (power_on_rise) begin
                        step         <= {STEP_W{1'b0}};
                        cnt          <= 32'd0;
                        o_POWER_BUS  <= {POWER_STEP{1'b0}};
                        o_SYS_RST_N  <= 1'b0;
                        o_clk_ref_en <= 1'b0;
                        o_power_done <= 1'b0;
                    end
                end

                ST_ON_RUN: begin
                    o_power_done <= 1'b0;
                    if (power_on_fall) begin
                        step         <= {STEP_W{1'b0}};
                        cnt          <= 32'd0;
                        o_SYS_RST_N  <= 1'b0;
                        o_clk_ref_en <= 1'b0;
                    end else if (step < POWER_STEP) begin
                        o_POWER_BUS[step] <= 1'b1;
                        if (cnt >= power_on_time[step]) begin
                            cnt  <= 32'd0;
                            step <= step + 1'b1;
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end else if (step == POWER_STEP) begin
                        o_SYS_RST_N <= 1'b1;
                        if (cnt >= POWERON_SYS2CLK-1) begin
                            cnt  <= 32'd0;
                            step <= step + 1'b1;
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end else begin
                        o_clk_ref_en <= 1'b1;
                        if (cnt >= POWERON_CLK2DONE-1) begin
                            cnt <= 32'd0;
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end
                end

                ST_ON_DONE: begin
                    o_power_done <= 1'b1;
                    if (power_on_fall) begin
                        step         <= {STEP_W{1'b0}};
                        cnt          <= 32'd0;
                        o_SYS_RST_N  <= 1'b0; // power-off step 0: assert sensor reset first
                        o_power_done <= 1'b0;
                    end
                end

                ST_OFF_RUN: begin
                    o_power_done <= 1'b0;
                    if (step >= POWER_STEP + 2) begin
                        cnt <= 32'd0;
                    end else if (step == 0) begin
                        o_SYS_RST_N <= 1'b0;
                        if (cnt >= POWEROFF_SYS2CLK-1) begin
                            cnt  <= 32'd0;
                            step <= step + 1'b1;
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end else if (step == 1) begin
                        o_clk_ref_en <= 1'b0; // power-off step 1: disable CLK_REF
                        if (cnt >= POWEROFF_CLK2PWR-1) begin
                            cnt  <= 32'd0;
                            step <= step + 1'b1;
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end else begin
                        // power-off step 2..: disable POWER[5] down to POWER[0]
                        o_POWER_BUS[POWER_STEP + 1 - step] <= 1'b0;
                        if (cnt >= ((power_off_time[POWER_STEP + 1 - step] != 32'd0) ?
                                    power_off_time[POWER_STEP + 1 - step] - 1 :
                                    POWEROFF_STEP_TIME-1)) begin
                            cnt <= 32'd0;
                            if (step < POWER_STEP + 2) begin
                                step <= step + 1'b1;
                            end
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end
                end
            endcase
        end
    end

endmodule
