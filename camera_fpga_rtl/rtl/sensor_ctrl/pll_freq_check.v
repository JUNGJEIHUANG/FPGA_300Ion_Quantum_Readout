`timescale 1ns/1ps
//-----------------------------------------------------------------------------
// pll_freq_check : 轻量测频 / PLL-OK 检测
//   在 clk_ref(已知频率)域开固定长度窗口, 数 clk_meas 的边沿数,
//   与预期区间[CNT_MIN,CNT_MAX]比较, 给出 pll_ok。
//   clk_meas 建议接 clk_rx(~150M, 时序安全)；clk_check_pll(~600M)直接数计数器很难收时序。
//-----------------------------------------------------------------------------
module pll_freq_check #(
    parameter integer GATE_CYCLES = 100_000,  // 测量窗口长度(clk_ref 周期数). 100k@100M = 1ms
    parameter integer CNT_MIN     = 147_000,  // 预期 clk_meas 边沿数下限(先估, 实测后收紧)
    parameter integer CNT_MAX     = 153_000,  // 预期 clk_meas 边沿数上限
    parameter integer CNT_W       = 20        // 计数位宽, 需 >= 预期最大计数
)(
    input  wire             clk_ref,   // 已知参考时钟, 接 clk_sys (100MHz)
    input  wire             rst,       // 高有效, clk_ref 域 (接 rst_sys)
    input  wire             clk_meas,  // 被测时钟, 接 clk_rx[0] (或 clk_check_pll)
    output reg              valid,     // 每完成一次测量拉高一拍
    output reg  [CNT_W-1:0] freq_cnt,  // 最近一次测得的边沿数 (送 VIO/ILA 观察)
    output reg              pll_ok     // 1: 落在区间内; 0: 越界或无时钟
);

    // ---- clk_ref 域: 产生测量窗口 gate (高 GATE_CYCLES, 低 GATE_CYCLES, 循环) ----
    reg [31:0] gate_cnt;
    reg        gate;
    always @(posedge clk_ref or posedge rst) begin
        if (rst) begin
            gate_cnt <= 32'd0;
            gate     <= 1'b0;
        end else if (gate_cnt >= GATE_CYCLES-1) begin
            gate_cnt <= 32'd0;
            gate     <= ~gate;
        end else begin
            gate_cnt <= gate_cnt + 32'd1;
        end
    end

    // ---- clk_meas 域: 窗口内计数, 窗口结束锁存并翻转握手位 ----
    (* ASYNC_REG="TRUE" *) reg gate_m1, gate_m2;
    reg                        gate_m2_d;
    reg [CNT_W-1:0]            cnt_meas;
    reg [CNT_W-1:0]            cnt_latched;
    reg                        done_tgl;
    always @(posedge clk_meas or posedge rst) begin
        if (rst) begin
            gate_m1     <= 1'b0; gate_m2 <= 1'b0; gate_m2_d <= 1'b0;
            cnt_meas    <= {CNT_W{1'b0}};
            cnt_latched <= {CNT_W{1'b0}};
            done_tgl    <= 1'b0;
        end else begin
            {gate_m2, gate_m1} <= {gate_m1, gate};       // 同步 gate 到 clk_meas 域
            gate_m2_d <= gate_m2;
            if (gate_m2 & ~gate_m2_d)
                cnt_meas <= {{(CNT_W-1){1'b0}}, 1'b1};    // 窗口起: 计 1
            else if (gate_m2)
                cnt_meas <= cnt_meas + 1'b1;              // 窗口内: 累加
            if (~gate_m2 & gate_m2_d) begin               // 窗口落: 锁存结果
                cnt_latched <= cnt_meas;
                done_tgl    <= ~done_tgl;
            end
        end
    end

    // ---- clk_ref 域: 同步 done 翻转, 采样(此时已稳定的) cnt_latched, 比较 ----
    (* ASYNC_REG="TRUE" *) reg done_r1, done_r2, done_r3;
    always @(posedge clk_ref or posedge rst) begin
        if (rst) {done_r3, done_r2, done_r1} <= 3'b0;
        else     {done_r3, done_r2, done_r1} <= {done_r2, done_r1, done_tgl};
    end
    wire done_pulse = done_r3 ^ done_r2;

    always @(posedge clk_ref or posedge rst) begin
        if (rst) begin
            freq_cnt <= {CNT_W{1'b0}};
            pll_ok   <= 1'b0;
            valid    <= 1'b0;
        end else begin
            valid <= 1'b0;
            if (done_pulse) begin
                freq_cnt <= cnt_latched;  // 窗口空闲期 cnt_latched 不变, 跨域采样安全
                pll_ok   <= (cnt_latched >= CNT_MIN) && (cnt_latched <= CNT_MAX);
                valid    <= 1'b1;
            end
        end
    end

endmodule

