# GSENSE6504BSI On-Camera FPGA Readout & Control RTL (`camera_fpga_rtl/`)

This directory contains the complete Verilog RTL hardware design, Xilinx Kintex-7 (`xc7k325tffg900-2`) physical/timing constraints, IP core configurations (`.xci`), and PC host control utilities developed for the **GSENSE6504BSI sCMOS scientific camera platform (CC2405 EVK)**.

---

## Hardware Architecture & Dataflow Pipeline

1. **6-Stage Power Sequencing & Clock Synthesis (`rtl/sensor_ctrl/`)**:
   - `power_rst_seq.v`: Controls the 6-step deterministic power-rail bring-up (`VDDA`, `VDDIO`, `VDDD`, `VDDAD`, `VDDPLL`, `VDDSF`, `EN_H/L`, `EN_BIAS`, `EN_VDDQ`), `SYS_RST_N` release, and `PLL_EN` gating.
   - `idt_8t49n004i.v`: Configures the on-board IDT `8T49N004I` clock synthesizer to generate the 40 MHz reference and LVDS sensor clocks.
   - `g6504_sensor_ctrl.v` & `g6504_spi_master.v`: Manages sensor register initialization, external `VSYNC` exposure pulse generation, and real-time SPI register read/write access.
   - `g6504_official_command_bridge.v` & `g6504_aa55_sensor_parser.v`: Bridges host `0xAA55` USB 3.0 / differential RS-422 UART command packets to FPGA control registers and the sensor SPI bus.

2. **16-Lane High-Speed LVDS Receiver & Bit/Word Alignment (`rtl/lvds_rx_training/`)**:
   - `clock_rx.v` & `training_top.v`: Receives 16 differential LVDS sensor data lanes using Xilinx 7-Series `IDELAYE2` + `ISERDESE2` primitives.
   - `training/` & `training_dbg/`: Performs dynamic eye-centering tap delay scan (`align_ctrl.v`), bit-slip word alignment (`train_word_align_7serial.v`), and 8-bit to 12-bit ADC depth gearbox deserialization (`des_8bit_to_adcdepth.v`).
   - `lval_synccode_top.v` & `lval_synccode.v`: Decodes embedded sCMOS sync codes (`SOF`, `EOF`, `SOL`, `EOL`) and generates line-valid (`LVAL`) strobes across all 16 sub-channels.

3. **Top/Bottom Channel Reordering & DDR3 MIG Frame Buffer (`rtl/image_ddr_path/`)**:
   - `g6504_img_reorder_top.v`, `chan_mult.v`, & `chan_top_bot_synrd_out.v`: Reorders interleaved multi-tap top/bottom sensor channels into raster-scan pixel streams and packs 12-bit pixels into 16-bit containers (`pixel_12to16.v`) or 384-bit wide DDR beats (`g6504_beat_gather.v`).
   - `g6504_k7_ddr_image_path.v`, `image_ddr_driver.v`, & `mig_driver.v`: Interfaces with the 64-bit DDR3 SDRAM (`mig_k7_mem_bar_512b`, 512-bit UI bus) to buffer full $2176 \times 2048$ frames or cropped ion-chain ROIs ($88 \times 456$).

4. **USB 3.0 FX3 (`CYUSB3014`) 32-Bit Slave FIFO Interface (`rtl/usb3_fx3/`)**:
   - `usb30_interface.v`, `usb30_rx.v`, `usb30_tx.v`, & `protocol_ctrl/`: Drives the 100 MHz 32-bit synchronous Slave FIFO interface (`FLAGA`/`FLAGB`, `SLCS_N`, `SLWR_N`, `SLRD_N`, `SLOE_N`, `PKTEND_N`) for high-throughput frame readout and bidirectional command/ACK packets.

5. **Cross-Clock Exposure-to-FX3 Timing Instrumentation (`rtl/timing_instrumentation/`)**:
   - `g6504_rough_ila.v` & `single_ila_transport.v`: Non-intrusive Gray-coded cross-domain timebase (`sys_clk` 100 MHz, `rx_clk` 220 MHz, `ddr_ui_clk` 200 MHz) and event-boundary counters (`VSYNC` -> LVDS lane write -> DDR3 ingress/egress -> FX3 burst completion).

---

## Directory Structure

```text
camera_fpga_rtl/
├── create_vivado_project.tcl          # Automated Vivado project reconstruction script
├── constraints/
│   └── g6504_k7_cc2405.xdc            # Pin assignments, LVDS I/O, MIG DDR3 & timing constraints
├── rtl/
│   ├── top/
│   │   └── g6504_bringup_top.v        # Top-level camera control & readout integration module
│   ├── sensor_ctrl/                   # Power sequencer, IDT clock init, SPI master, AA55 bridge & UART
│   ├── lvds_rx_training/              # 16-lane ISERDESE2/IDELAYE2 LVDS RX, training & sync-code parser
│   ├── image_ddr_path/                # Channel reordering, DDR3 MIG arbiter & frame buffer pipeline
│   ├── usb3_fx3/                      # Cypress FX3 USB 3.0 32-bit Slave FIFO RX/TX & protocol decoders
│   └── timing_instrumentation/        # Cross-clock exposure-to-USB latency & word-count monitors
├── ip_xci/                            # Xilinx Vivado IP core configurations (.xci & mig_a.prj)
└── host_control/                      # Python USB 3.0 / AA55 camera configuration, GUI & auto-exposure tools
    ├── cc2405_official_control.py     # CLI utility for sensor SPI R/W, mode setup & frame capture
    ├── cc2405_official_usb.py         # Low-level FX3 USB 3.0 bulk/command transaction driver
    ├── cc2405_control_gui.py          # Interactive GUI for camera bring-up and live readout
    ├── auto_exposure_std.py           # Closed-loop auto-exposure controller
    ├── ae_capture_std.py              # Automated exposure sweep & frame acquisition script
    ├── sensor_aa55_packet.py          # AA55 protocol packet builder and parser
    ├── sensor_aa55_usb_config.py      # USB FX3 session wrapper
    └── Gpixel_reg_GT_config/          # Golden FPGA & GSENSE6504BSI register tables (STD / HDR / High-Sens)
```

---

## Building the Vivado Project

To reconstruct the Vivado project from source:

```bash
cd camera_fpga_rtl
vivado -mode batch -source create_vivado_project.tcl
```

To configure the camera and capture frames from a host PC over USB 3.0:

```bash
cd camera_fpga_rtl/host_control
python cc2405_official_control.py --help
```
