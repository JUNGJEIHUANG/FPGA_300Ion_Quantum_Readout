import os
import numpy as np
import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.gridspec as gridspec
import matplotlib.cm as cm
from matplotlib.ticker import ScalarFormatter, NullFormatter

# Project-Wide Figure Defaults (Times New Roman, 300 DPI)
matplotlib.rcParams["font.family"] = "serif"
matplotlib.rcParams["font.serif"] = ["Times New Roman", "DejaVu Serif"]
matplotlib.rcParams["mathtext.fontset"] = "stix"
matplotlib.rcParams["axes.edgecolor"] = "#333333"
matplotlib.rcParams["axes.linewidth"] = 0.9

def build_figure():
    fig = Figure(figsize=(14.5, 10.4), dpi=300)
    FigureCanvasAgg(fig)

    gs = gridspec.GridSpec(
        2, 2, figure=fig,
        height_ratios=[1.05, 1.15],
        width_ratios=[1.08, 1.0],
        hspace=0.36, wspace=0.24
    )

    # Viridis palette sampling (perceptually uniform, colorblind-safe)
    cmap = cm.get_cmap("viridis")
    c_dark   = cmap(0.12)  # Deep purple-blue
    c_mid    = cmap(0.46)  # Teal-green
    c_bright = cmap(0.78)  # Warm viridis green-gold

    # =========================================================================
    # PANEL (a): Area-Latency Pareto Trajectory on Kintex-7 XC7K325T
    # =========================================================================
    ax_a = fig.add_subplot(gs[0, 0])

    # Synthesis data across Run #1 (Baseline), Run #2 (DSP-Unrolled), Run #3 (Pareto-Optimal)
    # Clock periods (ns): Run #1 = 5.521, Run #2 = 4.850, Run #3 = 4.176
    # HLS Clock Uncertainty = 1.35 ns (+/- 0.675 ns half-window)
    interval_cycles = np.array([101042306, 17404268, 17404268], dtype=float)
    clk_est_ns      = np.array([5.521, 4.850, 4.176], dtype=float)
    interval_ms     = interval_cycles * clk_est_ns * 1e-6
    interval_err_ms = interval_cycles * 0.675 * 1e-6  # +/- 0.675 ns (1.35 ns HLS clock uncertainty window)

    bram_util = np.array([369.4, 108.4, 98.5])   # 3288, 965, 877 / 890
    lut_util  = np.array([40.3,  109.9, 84.9])   # 82032, 224023, 172945 / 203800
    dsp_util  = np.array([38.0,  55.1,  47.6])   # 319, 463, 400 / 840

    # Shaded feasibility zones
    ax_a.axhspan(100.0, 425.0, color="#FDE8E8", alpha=0.45, zorder=0)
    ax_a.axhspan(0.0, 100.0, color="#E8F6F3", alpha=0.45, zorder=0)
    ax_a.axhline(100.0, color="#B03A2E", linestyle="--", linewidth=1.4, zorder=2,
                 label="XC7K325T Resource Limit (100% Capacity)")

    # Plot trajectories with horizontal clock-uncertainty error bars
    ax_a.errorbar(
        interval_ms, bram_util, xerr=interval_err_ms,
        color=c_dark, linestyle="-", linewidth=2.2,
        marker="o", markersize=8.5, markerfacecolor=c_dark, markeredgecolor="white", markeredgewidth=0.9,
        capsize=3.5, elinewidth=1.1, zorder=4,
        label="BRAM_18K Utilization (% of 890)"
    )
    ax_a.errorbar(
        interval_ms, lut_util, xerr=interval_err_ms,
        color=c_mid, linestyle="--", linewidth=2.0,
        marker="s", markersize=8.0, markerfacecolor=c_mid, markeredgecolor="white", markeredgewidth=0.9,
        capsize=3.5, elinewidth=1.1, zorder=4,
        label="LUT Utilization (% of 203.8K)"
    )
    ax_a.errorbar(
        interval_ms, dsp_util, xerr=interval_err_ms,
        color=c_bright, linestyle="-.", linewidth=2.0,
        marker="^", markersize=8.5, markerfacecolor=c_bright, markeredgecolor="#222222", markeredgewidth=0.8,
        capsize=3.5, elinewidth=1.1, zorder=4,
        label="DSP48E1 Utilization (% of 840)"
    )

    # Annotations positioned in open space with zero overlap with legend or lines
    ax_a.annotate(
        "Run #1 (Baseline)\n3,288 BRAM (369.4%)\n557.9 ms (181.1 MHz)",
        xy=(interval_ms[0], bram_util[0]),
        xytext=(280.0, 185.0),
        fontsize=8.6, fontweight="bold", color="#222222",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor=c_dark, alpha=0.94),
        arrowprops=dict(arrowstyle="->", color=c_dark, lw=1.2, connectionstyle="arc3,rad=-0.12"),
        zorder=6
    )
    ax_a.annotate(
        "Run #2 (DSP-Unrolled)\n965 BRAM (108.4%)\n109.9% LUT, 84.4 ms",
        xy=(interval_ms[1], lut_util[1]),
        xytext=(120.0, 122.0),
        fontsize=8.5, color="#222222",
        bbox=dict(boxstyle="round,pad=0.28", facecolor="white", edgecolor=c_mid, alpha=0.94),
        arrowprops=dict(arrowstyle="->", color=c_mid, lw=1.2, connectionstyle="arc3,rad=0.10"),
        zorder=6
    )
    ax_a.annotate(
        "Run #3 (Pareto-Optimal)\n877 BRAM (98.5%), 84.9% LUT\n72.7 ms @ 239.4 MHz (7.67x)",
        xy=(interval_ms[2], bram_util[2]),
        xytext=(57.0, 195.0),
        fontsize=8.6, fontweight="bold", color="#0B5345",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="#EAFAF1", edgecolor="#117864", lw=1.2, alpha=0.96),
        arrowprops=dict(arrowstyle="->", color="#117864", lw=1.4, connectionstyle="arc3,rad=-0.12"),
        zorder=6
    )

    ax_a.text(650.0, 108.0, "Overflow Zone (>100%)", fontsize=8.5, color="#922B21",
              ha="right", va="bottom", style="italic")
    ax_a.text(650.0, 10.0, "Feasible On-Chip Zone (<=100%)", fontsize=8.5, color="#0E6251",
              ha="right", va="bottom", style="italic")

    ax_a.set_xscale("log")
    ax_a.set_xlim(52.0, 720.0)
    ax_a.set_ylim(0.0, 420.0)
    ax_a.set_xticks([60, 80, 100, 150, 250, 400, 600])
    ax_a.get_xaxis().set_major_formatter(ScalarFormatter())
    ax_a.get_xaxis().set_minor_formatter(NullFormatter())

    ax_a.set_title("(a) Area-Latency Pareto Trajectory on Kintex-7 XC7K325T", fontsize=11.5, fontweight="bold", pad=8)
    ax_a.set_xlabel("Streaming Frame Interval at Estimated Fmax (ms, Log Scale)", fontsize=10.5)
    ax_a.set_ylabel("On-Chip Resource Utilization (% of XC7K325T)", fontsize=10.5)
    ax_a.grid(True, which="both", linestyle=":", alpha=0.35, color="#666666")
    # Place legend in the open upper-left region where no data points exist
    ax_a.legend(loc="upper left", fontsize=8.3, framealpha=0.95, edgecolor="#CCCCCC")

    # =========================================================================
    # PANEL (b): Subsystem BRAM_18K Reduction Across Optimization Runs
    # =========================================================================
    ax_b = fig.add_subplot(gs[0, 1])

    stages_x = np.array([0, 1, 2])
    stage_labels = [
        "Run #1: Baseline\n(Total: 3,288 BRAM)",
        "Run #2: 18-bit Packed\n(Total: 965 BRAM)",
        "Run #3: 14/16-bit Hybrid\n(Total: 877 BRAM)"
    ]

    # Exact synthesized BRAM_18K per hardware subsystem
    bram_dwconv   = np.array([1549, 216, 192])
    bram_skipfifo = np.array([937,  279, 217])
    bram_pwconv   = np.array([338,  296, 296])
    bram_dconv    = np.array([234,  160, 160])
    bram_resize   = np.array([230,   14,  12])  # BilinearResize (216->0->0) + Control/Misc (14->14->12)

    subsystems = [
        (bram_dwconv,   "DWConv Line Buffers (1,549 -> 192, -87.6%)", cmap(0.10), "o", "-"),
        (bram_skipfifo, "U-Net Skip FIFOs (937 -> 217, -76.8%)",      cmap(0.35), "s", "--"),
        (bram_pwconv,   "PWConv Weight ROMs (338 -> 296, -12.4%)",    cmap(0.58), "^", "-."),
        (bram_dconv,    "TransposeConv ROMs (234 -> 160, -31.6%)",    cmap(0.76), "D", ":"),
        (bram_resize,   "Bilinear Resize & Misc (230 -> 12, -94.8%)", cmap(0.90), "v", "-"),
    ]

    for y_vals, lbl, col, mk, ls in subsystems:
        edge_c = "#222222" if col == cmap(0.90) else "white"
        ax_b.plot(
            stages_x, y_vals, label=lbl, color=col, linestyle=ls, linewidth=2.2,
            marker=mk, markersize=8.0, markerfacecolor=col, markeredgecolor=edge_c, markeredgewidth=0.9,
            zorder=4
        )

    # Carefully placed direct labels at Run #1 (x=0), Run #2 (x=1), and Run #3 (x=2)
    r1_pos = [( -0.06, 1650, "right"), ( -0.06, 937, "right"), ( -0.06, 365, "right"), ( -0.06, 238, "right"), ( -0.06, 165, "right")]
    r2_pos = [(  1.12,  180, "left"),  (  1.12, 236, "left"),  (  1.00, 365, "center"), (  1.00, 118, "center"), (  1.06,  16, "left")]
    r3_pos = [(  2.06,  182, "left"),  (  2.06, 232, "left"),  (  2.06, 315, "left"),   (  2.06, 138, "left"),   (  2.06,  12, "left")]

    for idx, (y_vals, _, col, _, _) in enumerate(subsystems):
        txt_col = "#556B2F" if idx == 4 else col
        ax_b.text(r1_pos[idx][0], r1_pos[idx][1], f"{y_vals[0]}",
                  fontsize=8.5, fontweight="bold", color=txt_col, ha=r1_pos[idx][2], va="center")
        ax_b.text(r2_pos[idx][0], r2_pos[idx][1], f"{y_vals[1]}",
                  fontsize=8.5, fontweight="bold", color=txt_col, ha=r2_pos[idx][2], va="center")
        ax_b.text(r3_pos[idx][0], r3_pos[idx][1], f"{y_vals[2]}",
                  fontsize=8.5, fontweight="bold", color=txt_col, ha=r3_pos[idx][2], va="center")

    ax_b.set_yscale("log")
    ax_b.set_xlim(-0.36, 2.40)
    ax_b.set_ylim(7.5, 5800.0)
    ax_b.set_xticks(stages_x)
    ax_b.set_xticklabels(stage_labels, fontsize=9.2)
    ax_b.set_yticks([10, 25, 50, 100, 250, 500, 1000, 2000])
    ax_b.get_yaxis().set_major_formatter(ScalarFormatter())
    ax_b.get_yaxis().set_minor_formatter(NullFormatter())

    ax_b.set_title("(b) Subsystem BRAM_18K Reduction Across Optimization Runs", fontsize=11.5, fontweight="bold", pad=8)
    ax_b.set_xlabel("HLS Synthesis Optimization Stage", fontsize=10.5)
    ax_b.set_ylabel("Allocated Block RAM (BRAM_18K Blocks, Log Scale)", fontsize=10.5)
    ax_b.grid(True, which="both", linestyle=":", alpha=0.35, color="#666666")
    ax_b.legend(loc="upper right", fontsize=8.1, framealpha=0.95, edgecolor="#CCCCCC")

    # =========================================================================
    # PANEL (c): Layer-Wise Latency Ablation Across High-Resolution Bottlenecks
    # =========================================================================
    ax_c = fig.add_subplot(gs[1, :])

    layer_names = [
        "invres4_pw1\n(88x456, 16->96)\n[UF=16]",
        "invres4_pw2\n(88x456, 96->8)\n[UF=32]",
        "invres3_pw2\n(44x228, 144->12)\n[UF=16]",
        "invres3_pw1\n(44x228, 24->144)\n[UF=24]",
        "dconv4\n(44x228->88x456)\n[UF=12]",
        "feat2_pw1\n(88x456, 8->48)\n[UF=8]",
        "invres2_pw1\n(22x114, 32->192)\n[UF=8]",
        "feat3_pw2\n(44x228, 72->12)\n[UF=6]",
        "feat1_pw\n(88x456, 16->8)\n[UF=16]",
        "Full-Net Interval\n(Top Dataflow)\n[Streaming]"
    ]

    # Exact synthesized cycle counts from Run #1, Run #2, and Run #3 reports
    cyc_r1 = np.array([
        101042305, 30903385, 17587537, 17577505, 15914681,
        15730225,  15431809,  8798785,  5176561, 101042306
    ], dtype=float)

    cyc_r2 = np.array([
        5256769, 5938817, 2798929, 1926145, 4276361,
        2808961, 2069097, 2347489, 1645249, 17404268
    ], dtype=float)

    cyc_r3 = np.array([
        5016001, 5577793, 2708641, 1835857, 4264877,
        2688577, 2051545, 2327425, 1404481, 17404268
    ], dtype=float)

    # Convert cycles to wall-clock latency (ms) at each run's synthesized Fmax:
    # Run #1: 5.521 ns (181.12 MHz), Run #2: 4.850 ns (206.20 MHz), Run #3: 4.176 ns (239.43 MHz)
    lat_r1 = cyc_r1 * 5.521 * 1e-6
    lat_r2 = cyc_r2 * 4.850 * 1e-6
    lat_r3 = cyc_r3 * 4.176 * 1e-6

    # Real hardware timing uncertainty error bars from Vitis HLS Clock Uncertainty (+/- 0.675 ns = 1.35 ns window)
    err_r1 = cyc_r1 * 0.675 * 1e-6
    err_r2 = cyc_r2 * 0.675 * 1e-6
    err_r3 = cyc_r3 * 0.675 * 1e-6

    x_pos = np.arange(len(layer_names))
    width = 0.26

    ax_c.bar(
        x_pos - width, lat_r1, width, yerr=err_r1, capsize=3.0,
        color=c_dark, edgecolor="#1A1A1A", linewidth=0.8, hatch="//", alpha=0.92,
        error_kw=dict(ecolor="#111111", lw=1.0, capthick=1.0),
        label="Run #1: Baseline UF=1 (Fmax = 181.12 MHz, 3,288 BRAM [369.4%])"
    )
    ax_c.bar(
        x_pos, lat_r2, width, yerr=err_r2, capsize=3.0,
        color=c_mid, edgecolor="#1A1A1A", linewidth=0.8, hatch="..", alpha=0.92,
        error_kw=dict(ecolor="#111111", lw=1.0, capthick=1.0),
        label="Run #2: DSP-Unrolled + 18-bit Packed (Fmax = 206.20 MHz, 965 BRAM [108.4%])"
    )
    ax_c.bar(
        x_pos + width, lat_r3, width, yerr=err_r3, capsize=3.0,
        color=c_bright, edgecolor="#1A1A1A", linewidth=0.8, hatch="xx", alpha=0.95,
        error_kw=dict(ecolor="#111111", lw=1.0, capthick=1.0),
        label="Run #3: Proposed Pareto-Optimal (Fmax = 239.43 MHz, 877 BRAM [98.5%])"
    )

    # Annotate speedup ratios (Run #1 vs Run #3) above each group
    speedups = lat_r1 / lat_r3
    for i in range(len(x_pos)):
        top_y = (lat_r1[i] + err_r1[i]) * 1.10
        ax_c.text(
            x_pos[i], top_y, f"{speedups[i]:.1f}x\n({lat_r1[i]:.0f}->{lat_r3[i]:.1f} ms)",
            ha="center", va="bottom", fontsize=8.0, fontweight="bold", color="#1B4F72",
            bbox=dict(boxstyle="round,pad=0.18", facecolor="#F8F9F9", edgecolor="#AED6F1", lw=0.7, alpha=0.94)
        )

    ax_c.set_yscale("log")
    ax_c.set_ylim(2.5, 1600.0)
    ax_c.set_xlim(-0.65, len(layer_names) - 0.35)
    ax_c.set_xticks(x_pos)
    ax_c.set_xticklabels(layer_names, fontsize=8.7)
    ax_c.set_yticks([5, 10, 25, 50, 100, 250, 500, 1000])
    ax_c.get_yaxis().set_major_formatter(ScalarFormatter())
    ax_c.get_yaxis().set_minor_formatter(NullFormatter())

    ax_c.set_title(
        "(c) Layer-Wise Latency Ablation Across High-Resolution Bottleneck Stages (Error Bars: HLS Clock Uncertainty ±0.675 ns)",
        fontsize=11.5, fontweight="bold", pad=8
    )
    ax_c.set_xlabel("Bottleneck Network Layer, Feature Map Resolution, and Parallel Unroll Factor [UF]", fontsize=10.5)
    ax_c.set_ylabel("Execution Latency at Fmax (ms, Log)", fontsize=10.5)
    ax_c.grid(True, which="both", axis="y", linestyle=":", alpha=0.35, color="#666666")
    # Place legend in the open upper-center valley (between x=1.5 and x=8.2, y=280..1450 ms)
    ax_c.legend(loc="upper center", bbox_to_anchor=(0.51, 0.98), fontsize=8.5, ncol=1,
                framealpha=0.95, edgecolor="#CCCCCC")

    fig.suptitle(
        "Hardware Area-Latency Pareto Optimization and Layer-Wise Ablation for Real-Time DWNetV2-UNet on Kintex-7 FPGA",
        fontsize=13.5, fontweight="bold", y=0.985
    )

    output_path = os.environ.get("CHART_OUTPUT", "G_ISCAS_Area_Latency_Pareto_Ablation.png")
    fig.savefig(output_path, dpi=300, bbox_inches="tight", facecolor="white")

    # Also save publication PNG, PDF, and SVG with G_ prefix next to the script
    base_dir = os.path.dirname(os.path.abspath(__file__))
    fig.savefig(os.path.join(base_dir, "G_ISCAS_Area_Latency_Pareto_Ablation.png"), dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(os.path.join(base_dir, "G_ISCAS_Area_Latency_Pareto_Ablation.pdf"), dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(os.path.join(base_dir, "G_ISCAS_Area_Latency_Pareto_Ablation.svg"), dpi=300, bbox_inches="tight", facecolor="white")

if __name__ == "__main__":
    build_figure()
