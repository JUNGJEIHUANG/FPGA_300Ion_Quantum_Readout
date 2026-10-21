> Historical DWNetV2 documentation. Claims and measurements below belong to the legacy work and are not DW-ViT results or current camera signoff.

# FPGA_300Ion_Quantum_Readout

**On-Camera FPGA Acceleration of Depthwise Separable U-Net and Streaming Centroid Extraction for 300-Ion Quantum Readout Compression**

This repository contains the GSENSE6504BSI sCMOS camera control & readout Verilog RTL (`camera_fpga_rtl/`), Vitis HLS hardware accelerator source code (`fpga_hls/`), PyTorch segmentation models (`nets/`), trained 8-bit/FP32 checkpoints (`weights/`), unified `INT8` & `ap_fixed<8,I>` evaluation scripts (`scripts/`), and synthesis/benchmark artifacts (`results/`, `figures/`) for on-camera real-time readout compression of 300-ion $^{171}\text{Yb}^+$ quantum registers on a Xilinx Kintex-7 (`xc7k325tffg900-2`) FPGA.

---

## Architecture Overview

![Hardware Architecture](figures/G_ISCAS_Hardware_Architecture.png)

Instead of streaming full $88 \times 456$ 18-bit sCMOS camera frames ($90,288\text{ B}$ / $722,304\text{ bits}$) to a host workstation or GPU, the on-camera FPGA executes a 19-stage streaming MobileNetV2-based depthwise separable U-Net (`DWNetV2-UNet`) followed by a zero-BRAM, zero-DSP single-pass connected-component labeling (`streaming_ccl`) centroid extractor:

1. **On-Camera Semantic Compression ($25.1\times$–$50.7\times$ Bit Reduction):** Each $88 \times 456$ sCMOS frame is compressed on the fly into compact 96-bit (`12 B`) `IonCenter = {x_hat, y_hat, Area}` packets per detected bright ion ($|1\rangle$), achieving a **$25.1\times$–$26.0\times$** payload reduction on full-bright registers ($289$–$300$ ions) and a **$50.0\times$–$50.7\times$** dynamic compression on mixed $\pi/2$ quantum states ($\sim 149$ bright ions).
2. **Stage-Adaptive Dual-Packed $2^k$-Split Skip FIFOs & Dual-Row `UQ3.5` Line Buffers:** Packing pairs of 7-bit stage-adaptive activations (`Q3.4` for shallow `x1/x2`, `Q4.3` for deep `x3/x4`) into 14-bit words ($\le 16\text{ bits}$, avoiding Xilinx 7-Series `RAMB18E1` parity-bit loss for depth $\ge 4,096$) with compile-time $2^k$ depth splitting reduces skip-FIFO BRAM by **76.8%** ($937 \to 217$ `BRAM_18K`). Dual-row 16-bit `UQ3.5` packing for `ReLU6`-bounded $3\times 3$ depthwise line buffers and a 2-row rolling LUTRAM bilinear resize unit reduce depthwise and resize BRAM by **87.6%** ($1,549 \to 192$) and **100%** ($216 \to 0$), respectively.
3. **Whole-Word Burst-Load (`II = UF`) Reshaped Pointwise Engine:** Eliminates dynamic 576-bit read-modify-write (RMW) barrel shifters during reshaped input loading, saving **51,078 LUTs (-22.8%)**, boosting $F_{\max}$ to **239.43 MHz (+32.2%)**, accelerating the dominant `invres4_pw1` layer by **26.63$\times$** ($557.85\text{ ms} \to 20.95\text{ ms}$), and improving pipeline frame throughput by **7.67$\times$**.

---

## Hardware Synthesis Progression (Xilinx Kintex-7 `xc7k325tffg900-2`)

![Area-Latency Pareto and Layer Ablation](figures/G_ISCAS_Area_Latency_Pareto_Ablation.png)

| Architecture & Synthesis Metric | Kintex-7 Budget | Run #1: Baseline | Run #2: DSP-Unrolled | Run #3: Proposed Optimal |
| :--- | :---: | :---: | :---: | :---: |
| **Optimization Strategy** | | | | |
| Skip FIFO & DWConv Line Buffer | — | 18-bit Native FIFO | 16-bit Dual-8b + 2ᵏ Split | **14b Stage-Adaptive + 16b Dual-UQ3.5** |
| Pointwise Schedule & Reshaper | — | Scalar (`UF=2, II=1`) | Element RMW (`UF≤32, II=1`) | **Whole-Word Burst (`UF≤32, II=UF`)** |
| **FPGA Resource Utilization** | | | | |
| Total BRAM_18K Blocks | 890 (100%) | 3,288 *(369.4% · Overflow ❌)* | 965 *(108.4% · Overflow ❌)* | **877 (98.5% · Fits Device ✅)** |
| Total DSP48E1 / FF | 840 / 407,600 | 319 (38.0%) / 76.1k | 463 (55.1%) / 124.1k | **400 (47.6%) / 102,604 (25.2%)** |
| Total LUT Usage | 203,800 (100%) | 82,032 (40.3%) | 224,023 *(109.9% · Overflow ❌)* | **172,945 (84.9% · Fits Device ✅)** |
| **Timing & Throughput** | | | | |
| Estimated Clock Period / F_max | 5.000 ns (200 MHz) | 5.521 ns (181.12 MHz) | 4.850 ns (206.20 MHz) | **4.176 ns (239.43 MHz, +32.2%)** |
| Dominant Layer (`invres4_pw1`) | — | 557.85 ms (101.04M cyc) | 25.49 ms (5.26M cyc) | **20.95 ms (5.02M cyc, 26.63× ⚡)** |
| Dataflow Frame Interval | — | 557.85 ms (1.79 FPS) | 84.40 ms (11.85 FPS) | **72.69 ms (13.76 FPS, 7.67× ⚡)** |
| **Fidelity Verification** | | | | |
| C-Sim Detected Ion Count | 289 / 289 GT | 289 / 289 (100%) | 289 / 289 (100%) | **289 / 289 (100%, Zero Loss ✅)** |

---

## Unified 8-Bit (INT8 & HLS ap_fixed<8,I>) Readout Benchmark

<table>
  <thead>
    <tr>
      <th rowspan="2" align="left">Method / Model</th>
      <th rowspan="2" align="center">Precision</th>
      <th colspan="2" align="center">Complexity</th>
      <th colspan="4" align="center">Training Subset (Diagnostic)</th>
      <th colspan="5" align="center">Real1600 Benchmark (Testset)</th>
    </tr>
    <tr>
      <th align="center">Weights</th>
      <th align="center">MACs</th>
      <th align="center">Full (%)</th>
      <th align="center"><i>F</i><sub>assign</sub></th>
      <th align="center">Ions (Full/π/2)</th>
      <th align="center">MAE</th>
      <th align="center">Full (%)</th>
      <th align="center">π/2 (%)</th>
      <th align="center"><i>F</i><sub>assign</sub> (ΔFP32)</th>
      <th align="center">MAE (px)</th>
      <th align="center">Bit-CR (Full / π/2)</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="left">OpenCV Otsu + 8-Conn CCL</td>
      <td align="center">8-bit Pixel</td>
      <td align="center">0 KB</td>
      <td align="center">—</td>
      <td align="center">97.85%</td>
      <td align="center">97.34%</td>
      <td align="center">145.3 / 149.0</td>
      <td align="center">0.781 px</td>
      <td align="center">97.95%</td>
      <td align="center">95.28%</td>
      <td align="center">96.62% (—)</td>
      <td align="center">0.893 px</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td align="left">Standard U-Net</td>
      <td align="center">INT8 Per-Ch</td>
      <td align="center">31.04 MB</td>
      <td align="center">23.81G</td>
      <td align="center">99.87%</td>
      <td align="center"><b>96.93%</b></td>
      <td align="center">300.1 / 152.9</td>
      <td align="center">0.563 px</td>
      <td align="center"><b>99.94%</b></td>
      <td align="center"><b>94.68%</b></td>
      <td align="center"><b>97.31%</b> (-0.00%)</td>
      <td align="center">0.480 px</td>
      <td align="center">25.1× / 51.3×</td>
    </tr>
    <tr>
      <td align="left">ViTSeg</td>
      <td align="center">INT8 Per-Ch</td>
      <td align="center">5.29 MB</td>
      <td align="center">4.15G</td>
      <td align="center">99.87%</td>
      <td align="center">96.71%</td>
      <td align="center">300.0 / 150.9</td>
      <td align="center">0.561 px</td>
      <td align="center"><b>99.94%</b></td>
      <td align="center">93.89%</td>
      <td align="center">96.91% (-0.02%)</td>
      <td align="center">0.480 px</td>
      <td align="center">25.1× / 50.5×</td>
    </tr>
    <tr>
      <td align="left">PSFANet</td>
      <td align="center">INT8 Per-Ch</td>
      <td align="center">4.68 MB</td>
      <td align="center">3.82G</td>
      <td align="center">99.86%</td>
      <td align="center">96.75%</td>
      <td align="center">300.0 / 153.5</td>
      <td align="center">0.563 px</td>
      <td align="center">99.91%</td>
      <td align="center">93.65%</td>
      <td align="center">96.78% (-0.24%)</td>
      <td align="center">0.482 px</td>
      <td align="center">25.1× / 51.6×</td>
    </tr>
    <tr>
      <td align="left">DWNetV2_pretrained</td>
      <td align="center">INT8 Per-Ch</td>
      <td align="center">4.68 MB</td>
      <td align="center">3.82G</td>
      <td align="center">99.87%</td>
      <td align="center">93.19%</td>
      <td align="center">300.0 / 152.3</td>
      <td align="center">0.582 px</td>
      <td align="center">99.89%</td>
      <td align="center">91.71%</td>
      <td align="center">95.80% (-1.23%)</td>
      <td align="center">0.504 px</td>
      <td align="center">25.1× / 52.9×</td>
    </tr>
    <tr>
      <td align="left">DWNetV2_pretrained</td>
      <td align="center">HLS <code>ap_fixed</code></td>
      <td align="center">4.68 MB</td>
      <td align="center">3.82G</td>
      <td align="center">95.54%</td>
      <td align="center">93.66%</td>
      <td align="center">286.4 / 144.8</td>
      <td align="center">0.616 px</td>
      <td align="center">95.39%</td>
      <td align="center">91.22%</td>
      <td align="center">93.31% (-3.72%)</td>
      <td align="center">0.567 px</td>
      <td align="center">26.4× / 55.8×</td>
    </tr>
    <tr>
      <td align="left">DWNetV2_scratch</td>
      <td align="center">INT8 Per-Ch</td>
      <td align="center">4.68 MB</td>
      <td align="center">3.82G</td>
      <td align="center">99.85%</td>
      <td align="center">96.69%</td>
      <td align="center">300.0 / 150.9</td>
      <td align="center">0.562 px</td>
      <td align="center"><b>99.94%</b></td>
      <td align="center">93.83%</td>
      <td align="center">96.88% (-0.00%)</td>
      <td align="center"><b>0.479 px</b></td>
      <td align="center">25.1× / 50.7×</td>
    </tr>
    <tr>
      <td align="left"><b>Ours: DWNetV2_scratch</b></td>
      <td align="center"><b>HLS + CCL</b></td>
      <td align="center"><b>0.76–4.68 MB</b></td>
      <td align="center"><b>1.08–3.82G</b></td>
      <td align="center">99.85%</td>
      <td align="center">96.66%</td>
      <td align="center"><b>300.0 / 150.5</b></td>
      <td align="center"><b>0.562 px</b></td>
      <td align="center"><b>99.94%</b></td>
      <td align="center">93.76%</td>
      <td align="center">96.85% (<b>-0.03%</b>)</td>
      <td align="center"><b>0.479 px</b></td>
      <td align="center"><b>25.1× / 50.7×</b></td>
    </tr>
  </tbody>
</table>

---

## Repository Structure

```text
FPGA_300Ion_Quantum_Readout/
├── camera_fpga_rtl/           # GSENSE6504BSI sCMOS camera control & readout Verilog RTL (Kintex-7)
│   ├── rtl/top/               # Top-level camera bring-up & readout module (g6504_bringup_top.v)
│   ├── rtl/sensor_ctrl/       # 6-step power sequencer, IDT clock init, SPI master, AA55 bridge & UART
│   ├── rtl/lvds_rx_training/  # 16-lane ISERDESE2/IDELAYE2 LVDS RX, eye training & sync-code parser
│   ├── rtl/image_ddr_path/    # Top/bottom channel reorder, DDR3 MIG frame buffer & pixel packer
│   ├── rtl/usb3_fx3/          # Cypress FX3 USB 3.0 32-bit Slave FIFO RX/TX & protocol decoders
│   ├── rtl/timing_instrumentation/ # Cross-clock exposure-to-USB latency & word-count monitors
│   ├── constraints/           # Pinout, LVDS I/O, DDR3 MIG & timing constraints (g6504_k7_cc2405.xdc)
│   ├── ip_xci/                # Xilinx Vivado IP core configurations (.xci & mig_a.prj)
│   ├── host_control/          # Python USB 3.0 / AA55 camera configuration, GUI & auto-exposure tools
│   └── create_vivado_project.tcl # Automated Vivado project reconstruction script
├── fpga_hls/                  # Vitis HLS 2022.2 C++ accelerator & streaming CCL sources
│   ├── dwnet_config.h         # Image dimensions (88x456) and hardware configuration
│   ├── dwnet_types.h          # 18-bit activation & layer-adaptive 8-bit weight types
│   ├── dwnet_layers.h/.cpp    # Dual-packed Skip FIFO, UQ3.5 DWConv, II=UF Burst PWConv
│   ├── dwnet_unet.h/.cpp      # Top-level 19-stage DATAFLOW streaming architecture
│   ├── streaming_ccl.h/.cpp   # Single-pass 0-BRAM/0-DSP union-find centroid extractor
│   ├── dwnet_weights_all.cpp  # BN-fused 8-bit fixed-point ROM weights
│   ├── tb_dwnet_unet.cpp      # Bit-accurate C-simulation testbench
│   ├── run_hls.tcl            # Automated Vitis HLS C-sim & synthesis Tcl script
│   └── reports/               # Archived Vitis HLS csynth reports (Run #2 & Run #3)
├── nets/                      # PyTorch model definitions (DWNetV2-UNet, ViTSeg, U-Net, PSFANet)
├── weights/                   # Trained PyTorch checkpoints (DWNetV2_scratch, pretrained, ViTSeg)
├── scripts/                   # Unified INT8 evaluation & Pareto plotting scripts
├── results/                   # Structured JSON and CSV experimental benchmark tables
└── figures/                   # High-resolution architecture & Pareto ablation figures
```

---

## Quick Start

### 1. Reconstruct the Camera Control FPGA Vivado Project (Verilog RTL)
```bash
cd camera_fpga_rtl
vivado -mode batch -source create_vivado_project.tcl
```

### 2. Run Vitis HLS C-Simulation and Synthesis
```bash
cd fpga_hls
vitis_hls -f run_hls.tcl
```

### 3. Run Unified INT8 & Hardware Fixed-Point Evaluation
```bash
pip install -r requirements.txt
python scripts/test_int8_all.py
```

### 4. Regenerate Area-Latency Pareto & Layer Ablation Figures
```bash
python scripts/plot_iscas_pareto.py
```

---

## License

Released under the [MIT License](LICENSE).
