# -----------------------------------------------------------------------------
# G6504 constraints aligned to K7 Demo CC2405 XDC names/pins.
#
# Source reference:
#   E:/G6504_060/K7_Demo/CC2405.srcs/constrs_1/new/io_evk.xdc
#   E:/G6504_060/K7_Demo/CC2405.srcs/constrs_1/new/io_dut.xdc
#   E:/G6504_060/K7_Demo/CC2405.srcs/constrs_1/new/timing.xdc
#
# Only ports present in g6504_bringup_top are constrained here. K7 Demo ports
# not implemented in this top, such as ADC/DP/FLASH, are not added. DDR3 is constrained by the MIG IP XDC.
# -----------------------------------------------------------------------------

#--------------------------------------------------------------------
#                EVK - EVM 2.0 PIN MAPPING - USB 3.0
#--------------------------------------------------------------------
# PACKAGE PINS
#--------------------------------------------------------------------
set_property IOSTANDARD DIFF_SSTL15 [get_ports i_CLK_OSC_IN_P]
set_property PACKAGE_PIN AD12 [get_ports i_CLK_OSC_IN_P]

set_property IOSTANDARD LVCMOS33 [get_ports i_FPGA_RST_N]
set_property PACKAGE_PIN R19 [get_ports i_FPGA_RST_N]
set_property PULLUP TRUE [get_ports i_FPGA_RST_N]

# Primary clocks: same names/waveforms as K7 Demo timing.xdc
create_clock -period 5.000 -name i_CLK_OSC_IN_P -waveform {0.000 2.500} [get_ports i_CLK_OSC_IN_P]
create_clock -period 1.667 -name i_CLK_DDR_IN_P -waveform {0.000 0.833} [get_ports i_CLK_DDR_IN_P]

# POWER
set_property PACKAGE_PIN C14 [get_ports o_EN_VCC4V5]
set_property PACKAGE_PIN A12 [get_ports o_EN_VCC2V5]
set_property PACKAGE_PIN C15 [get_ports o_EN_VCC3V3]
set_property PACKAGE_PIN B15 [get_ports o_EN_VCC_LT]
set_property PACKAGE_PIN H17 [get_ports o_EN_VCC   ]
set_property PACKAGE_PIN D14 [get_ports o_EN_VCC5V5]
set_property PACKAGE_PIN D11 [get_ports o_EN_VDDA  ]
set_property PACKAGE_PIN J17 [get_ports o_EN_VDDIO ]
set_property PACKAGE_PIN B14 [get_ports o_EN_VDDD  ]
set_property PACKAGE_PIN A15 [get_ports o_EN_VDDAD ]
set_property PACKAGE_PIN K20 [get_ports o_EN_VDDPLL]
set_property PACKAGE_PIN C11 [get_ports o_EN_VDDSF ]
set_property PACKAGE_PIN L17 [get_ports o_EN_H     ]
set_property PACKAGE_PIN L18 [get_ports o_EN_L     ]
set_property PACKAGE_PIN K19 [get_ports o_EN_BIAS  ]
set_property PACKAGE_PIN F17 [get_ports o_EN_VDDQ  ]

set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VCC4V5]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VCC2V5]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VCC3V3]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VCC_LT]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VCC   ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VCC5V5]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VDDA  ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VDDIO ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VDDD  ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VDDAD ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VDDPLL]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VDDSF ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_H     ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_L     ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_BIAS  ]
set_property IOSTANDARD LVCMOS25 [get_ports o_EN_VDDQ  ]

# o_BUS_TRANS_OE_N
set_property PACKAGE_PIN N30 [get_ports o_BUS_TRANS_OE_N]
set_property IOSTANDARD LVCMOS33 [get_ports o_BUS_TRANS_OE_N]

# OTP
set_property PACKAGE_PIN M30 [get_ports o_OTP_CTRL]
set_property IOSTANDARD LVCMOS33 [get_ports o_OTP_CTRL]

# sensor
set_property PACKAGE_PIN K21 [get_ports o_CLK_REF]
set_property IOSTANDARD LVCMOS33 [get_ports o_CLK_REF]

set_property PACKAGE_PIN J23 [get_ports o_VSYNC]
set_property PACKAGE_PIN J24 [get_ports o_HSYNC]
set_property IOSTANDARD LVCMOS33 [get_ports o_VSYNC]
set_property IOSTANDARD LVCMOS33 [get_ports o_HSYNC]

set_property PACKAGE_PIN N20 [get_ports o_SYS_RST_N]
set_property IOSTANDARD LVCMOS33 [get_ports o_SYS_RST_N]

set_property PACKAGE_PIN L21 [get_ports o_SPI_SCK]
set_property PACKAGE_PIN L20 [get_ports o_SPI_XCE]
set_property PACKAGE_PIN K26 [get_ports i_SPI_SDO]
set_property PACKAGE_PIN N19 [get_ports o_SPI_SDI]
set_property IOSTANDARD LVCMOS33 [get_ports o_SPI_SCK]
set_property IOSTANDARD LVCMOS33 [get_ports o_SPI_XCE]
set_property IOSTANDARD LVCMOS33 [get_ports i_SPI_SDO]
set_property IOSTANDARD LVCMOS33 [get_ports o_SPI_SDI]

# CLK DDR
set_property PACKAGE_PIN D12 [get_ports i_CLK_DDR_IN_P]
set_property PACKAGE_PIN D13 [get_ports i_CLK_DDR_IN_N]
set_property IOSTANDARD LVDS_25 [get_ports i_CLK_DDR_IN_P]

# TOP
set_property PACKAGE_PIN L15 [get_ports {i_SENSOR_DATA_P[0]}]
set_property PACKAGE_PIN K15 [get_ports {i_SENSOR_DATA_N[0]}]
set_property PACKAGE_PIN L12 [get_ports {i_SENSOR_DATA_P[1]}]
set_property PACKAGE_PIN L13 [get_ports {i_SENSOR_DATA_N[1]}]
set_property PACKAGE_PIN G13 [get_ports {i_SENSOR_DATA_P[2]}]
set_property PACKAGE_PIN F13 [get_ports {i_SENSOR_DATA_N[2]}]
set_property PACKAGE_PIN J11 [get_ports {i_SENSOR_DATA_P[3]}]
set_property PACKAGE_PIN J12 [get_ports {i_SENSOR_DATA_N[3]}]
set_property PACKAGE_PIN K13 [get_ports {i_SENSOR_DATA_P[4]}]
set_property PACKAGE_PIN J13 [get_ports {i_SENSOR_DATA_N[4]}]
set_property PACKAGE_PIN L16 [get_ports {i_SENSOR_DATA_P[5]}]
set_property PACKAGE_PIN K16 [get_ports {i_SENSOR_DATA_N[5]}]
set_property PACKAGE_PIN F15 [get_ports {i_SENSOR_DATA_P[6]}]
set_property PACKAGE_PIN E16 [get_ports {i_SENSOR_DATA_N[6]}]
set_property PACKAGE_PIN H15 [get_ports {i_SENSOR_DATA_P[7]}]
set_property PACKAGE_PIN G15 [get_ports {i_SENSOR_DATA_N[7]}]

# BOT
set_property PACKAGE_PIN F12 [get_ports {i_SENSOR_DATA_P[8]}]
set_property PACKAGE_PIN E13 [get_ports {i_SENSOR_DATA_N[8]}]
set_property PACKAGE_PIN K14 [get_ports {i_SENSOR_DATA_P[9]}]
set_property PACKAGE_PIN J14 [get_ports {i_SENSOR_DATA_N[9]}]
set_property PACKAGE_PIN H11 [get_ports {i_SENSOR_DATA_P[10]}]
set_property PACKAGE_PIN H12 [get_ports {i_SENSOR_DATA_N[10]}]
set_property PACKAGE_PIN L11 [get_ports {i_SENSOR_DATA_P[11]}]
set_property PACKAGE_PIN K11 [get_ports {i_SENSOR_DATA_N[11]}]
set_property PACKAGE_PIN E14 [get_ports {i_SENSOR_DATA_P[12]}]
set_property PACKAGE_PIN E15 [get_ports {i_SENSOR_DATA_N[12]}]
set_property PACKAGE_PIN J16 [get_ports {i_SENSOR_DATA_P[13]}]
set_property PACKAGE_PIN H16 [get_ports {i_SENSOR_DATA_N[13]}]
set_property PACKAGE_PIN H14 [get_ports {i_SENSOR_DATA_P[14]}]
set_property PACKAGE_PIN G14 [get_ports {i_SENSOR_DATA_N[14]}]
set_property PACKAGE_PIN C12 [get_ports {i_SENSOR_DATA_P[15]}]
set_property PACKAGE_PIN B12 [get_ports {i_SENSOR_DATA_N[15]}]

set_property IOSTANDARD LVDS_25 [get_ports {i_SENSOR_DATA_P[*]}]

# PLL CTRL
set_property PACKAGE_PIN J21     [get_ports o_PLL_EN ]
set_property IOSTANDARD LVCMOS33 [get_ports o_PLL_EN ]
set_property PACKAGE_PIN T25     [get_ports o_SEL    ]
set_property PACKAGE_PIN T23     [get_ports o_SEL3   ]
set_property PACKAGE_PIN U23     [get_ports o_SEL2   ]
set_property PACKAGE_PIN P26     [get_ports o_SEL1   ]
set_property PACKAGE_PIN R26     [get_ports o_SEL0   ]
set_property PACKAGE_PIN M22     [get_ports o_CLK_SEL]
set_property PACKAGE_PIN L22     [get_ports o_SDATA  ]
set_property PACKAGE_PIN N21     [get_ports o_SCLK   ]
set_property PACKAGE_PIN N22     [get_ports o_FSEL1  ]
set_property PACKAGE_PIN L23     [get_ports o_FSEL0  ]
set_property IOSTANDARD LVCMOS33 [get_ports o_SEL    ]
set_property IOSTANDARD LVCMOS33 [get_ports o_SEL3   ]
set_property IOSTANDARD LVCMOS33 [get_ports o_SEL2   ]
set_property IOSTANDARD LVCMOS33 [get_ports o_SEL1   ]
set_property IOSTANDARD LVCMOS33 [get_ports o_SEL0   ]
set_property IOSTANDARD LVCMOS33 [get_ports o_CLK_SEL]
set_property IOSTANDARD LVCMOS33 [get_ports o_SDATA  ]
set_property IOSTANDARD LVCMOS33 [get_ports o_SCLK   ]
set_property IOSTANDARD LVCMOS33 [get_ports o_FSEL1  ]
set_property IOSTANDARD LVCMOS33 [get_ports o_FSEL0  ]

# PLL input clock to sensor data-interface PLL (40 MHz LVDS), same as K7 Demo io_dut.xdc (F11)
set_property PACKAGE_PIN F11 [get_ports o_PLL_IN_P]
set_property IOSTANDARD LVDS_25 [get_ports o_PLL_IN_P]

# OS
set_property PACKAGE_PIN B20 [get_ports o_SLAVE_CTRL]
set_property PACKAGE_PIN C20 [get_ports o_CTRL_RELAY]
set_property IOSTANDARD LVCMOS25 [get_ports o_SLAVE_CTRL]
set_property IOSTANDARD LVCMOS25 [get_ports o_CTRL_RELAY]

# Differential UART/SENSOR-register control, copied from K7_Demo's OS serial
# interface. The P pin is constrained; the paired N pin is inferred by the
# differential port. The external PC adapter must be LVDS/RS-422 compatible.
set_property PACKAGE_PIN B18 [get_ports i_OS_SERTC_P]
set_property PACKAGE_PIN C19 [get_ports o_OS_SERTFG_P]
set_property IOSTANDARD LVDS_25 [get_ports i_OS_SERTC_P]
set_property IOSTANDARD LVDS_25 [get_ports o_OS_SERTFG_P]

# Clock groups: same intent/names as K7 Demo, adjusted to current valid clock names.
# Use -quiet because clk_wiz generated clocks may not exist while this XDC is
# parsed during synthesis; they are applied when available in implementation.
set_clock_groups -quiet -name rx_sys  -asynchronous -group [get_clocks -quiet clk_rx]             -group [get_clocks -quiet clk_100M_clk_wiz_0]
set_clock_groups -quiet -name glb_sys -asynchronous -group [get_clocks -quiet clk_220M_clk_wiz_0] -group [get_clocks -quiet clk_100M_clk_wiz_0]
set_clock_groups -quiet -name ddr_sys -asynchronous -group [get_clocks -quiet i_CLK_DDR_IN_P]     -group [get_clocks -quiet clk_100M_clk_wiz_0]

# Current-top CDC into CLK_REF enable synchronizer; keep valid, narrow false path.
set_false_path -quiet -from [get_clocks -quiet clk_100M_clk_wiz_0] -to [get_pins -quiet clk_ref_en_sync1_reg/D]


# ---- K7_Demo-equivalent DDR/mig_driver timing exceptions ----
# K7 timing.xdc false-paths these configuration delay registers inside
# u_imageprocess_top/u_image_ddr_driver/inst_mig_driver.  In this port the
# equivalent hierarchy is u_g6504_k7_ddr_image_path/u_image_ddr_driver/inst_mig_driver.
set_false_path -quiet -from [get_pins -hier -filter {NAME =~ *u_g6504_k7_ddr_image_path/u_image_ddr_driver/inst_mig_driver/frame_num_3dly_reg[*]/C}]
set_false_path -quiet -from [get_pins -hier -filter {NAME =~ *u_g6504_k7_ddr_image_path/u_image_ddr_driver/inst_mig_driver/win_row_len_3dly_reg[*]/C}]
set_false_path -quiet -from [get_pins -hier -filter {NAME =~ *u_g6504_k7_ddr_image_path/u_image_ddr_driver/inst_mig_driver/lval_len_3dly_reg[*]/C}]

# K7 timing.xdc false-paths the DDR output-buffer frame reset crossing.
set_false_path -quiet -from [get_pins -hier -filter {NAME =~ *u_g6504_k7_ddr_image_path/u_ddr_dout_buf_0/inst/frame_rst_wr_reg/C}]

# Self-developed debug-only CDC: DDR/UI-domain counters and status are sampled by
# the 220 MHz ILA for observability only.  Do not let these debug probes dominate
# functional DDR timing closure.
set_false_path -quiet -from [get_clocks -quiet clk_pll_i] -to [get_clocks -quiet clk_220M_clk_wiz_0]
set_false_path -quiet -from [get_clocks -quiet clk_220M_clk_wiz_0] -to [get_clocks -quiet clk_pll_i]
# load
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 26 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 1 [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE Yes [current_design]

set_false_path -to [get_pins -hier -filter {NAME =~ *u_pll_freq_check/gate_m1_reg*/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *u_pll_freq_check/done_r1_reg*/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *u_pll_freq_check/freq_cnt_reg*/D}]

# ---- USB 3.0 / FX3 (from K7_Demo io_evk.xdc) ----
set_property IOSTANDARD LVCMOS25 [get_ports {io_FX3_DB[*]}]
set_property PACKAGE_PIN AJ24 [get_ports {io_FX3_DB[0]}]
set_property PACKAGE_PIN AF25 [get_ports {io_FX3_DB[1]}]
set_property PACKAGE_PIN Y20  [get_ports {io_FX3_DB[2]}]
set_property PACKAGE_PIN AG24 [get_ports {io_FX3_DB[3]}]
set_property PACKAGE_PIN AK21 [get_ports {io_FX3_DB[4]}]
set_property PACKAGE_PIN AH25 [get_ports {io_FX3_DB[5]}]
set_property PACKAGE_PIN AD24 [get_ports {io_FX3_DB[6]}]
set_property PACKAGE_PIN AD21 [get_ports {io_FX3_DB[7]}]
set_property PACKAGE_PIN AH22 [get_ports {io_FX3_DB[8]}]
set_property PACKAGE_PIN AC21 [get_ports {io_FX3_DB[9]}]
set_property PACKAGE_PIN AF20 [get_ports {io_FX3_DB[10]}]
set_property PACKAGE_PIN AC20 [get_ports {io_FX3_DB[11]}]
set_property PACKAGE_PIN AH21 [get_ports {io_FX3_DB[12]}]
set_property PACKAGE_PIN AH20 [get_ports {io_FX3_DB[13]}]
set_property PACKAGE_PIN AK20 [get_ports {io_FX3_DB[14]}]
set_property PACKAGE_PIN AC22 [get_ports {io_FX3_DB[15]}]
set_property PACKAGE_PIN AK23 [get_ports {io_FX3_DB[16]}]
set_property PACKAGE_PIN AJ22 [get_ports {io_FX3_DB[17]}]
set_property PACKAGE_PIN AF21 [get_ports {io_FX3_DB[18]}]
set_property PACKAGE_PIN AK24 [get_ports {io_FX3_DB[19]}]
set_property PACKAGE_PIN AJ23 [get_ports {io_FX3_DB[20]}]
set_property PACKAGE_PIN AF22 [get_ports {io_FX3_DB[21]}]
set_property PACKAGE_PIN AG23 [get_ports {io_FX3_DB[22]}]
set_property PACKAGE_PIN AE21 [get_ports {io_FX3_DB[23]}]
set_property PACKAGE_PIN AE23 [get_ports {io_FX3_DB[24]}]
set_property PACKAGE_PIN AD23 [get_ports {io_FX3_DB[25]}]
set_property PACKAGE_PIN AD22 [get_ports {io_FX3_DB[26]}]
set_property PACKAGE_PIN AE24 [get_ports {io_FX3_DB[27]}]
set_property PACKAGE_PIN AF23 [get_ports {io_FX3_DB[28]}]
set_property PACKAGE_PIN Y21  [get_ports {io_FX3_DB[29]}]
set_property PACKAGE_PIN AB24 [get_ports {io_FX3_DB[30]}]
set_property PACKAGE_PIN AA22 [get_ports {io_FX3_DB[31]}]

set_property IOSTANDARD LVCMOS25 [get_ports {o_FX3_A[*]}]
set_property PACKAGE_PIN AG20 [get_ports {o_FX3_A[0]}]
set_property PACKAGE_PIN AK25 [get_ports {o_FX3_A[1]}]

set_property IOSTANDARD LVCMOS25 [get_ports o_FX3_SLWR_N]
set_property PACKAGE_PIN AB20 [get_ports o_FX3_SLWR_N]
set_property IOSTANDARD LVCMOS25 [get_ports o_FX3_SLRD_N]
set_property PACKAGE_PIN AA20 [get_ports o_FX3_SLRD_N]
set_property IOSTANDARD LVCMOS25 [get_ports o_FX3_SLOE_N]
set_property PACKAGE_PIN AC24 [get_ports o_FX3_SLOE_N]
set_property IOSTANDARD LVCMOS25 [get_ports o_FX3_SLCS_N]
set_property PACKAGE_PIN AG22 [get_ports o_FX3_SLCS_N]
set_property IOSTANDARD LVCMOS25 [get_ports o_FX3_PKTEND_N]
set_property PACKAGE_PIN AE20 [get_ports o_FX3_PKTEND_N]
set_property IOSTANDARD LVCMOS25 [get_ports o_FX3_PCLK]
set_property PACKAGE_PIN AB23 [get_ports o_FX3_PCLK]
set_property IOSTANDARD LVCMOS25 [get_ports i_FX3_FLAGB]
set_property PACKAGE_PIN AB22 [get_ports i_FX3_FLAGB]
set_property IOSTANDARD LVCMOS25 [get_ports i_FX3_FLAGA]
set_property PACKAGE_PIN AH24 [get_ports i_FX3_FLAGA]
set_property IOSTANDARD LVCMOS33 [get_ports o_USB_RESET]
set_property PACKAGE_PIN AE26 [get_ports o_USB_RESET]

# Async clock group: clk_220M and clk_100M are unrelated for timing purposes
set_clock_groups -quiet -name img_cdc -asynchronous \
    -group [get_clocks -quiet clk_220M_clk_wiz_0] \
    -group [get_clocks -quiet clk_100M_clk_wiz_0]

# DDR3 extra address pins for K7_Demo-compatible MIG RowAddress=16
set_property PACKAGE_PIN AK10 [get_ports {o_DDR3_ADDR[14]}]
set_property IOSTANDARD SSTL15 [get_ports {o_DDR3_ADDR[14]}]
set_property SLEW FAST [get_ports {o_DDR3_ADDR[14]}]
set_property VCCAUX_IO HIGH [get_ports {o_DDR3_ADDR[14]}]
set_property PACKAGE_PIN AK11 [get_ports {o_DDR3_ADDR[15]}]
set_property IOSTANDARD SSTL15 [get_ports {o_DDR3_ADDR[15]}]
set_property SLEW FAST [get_ports {o_DDR3_ADDR[15]}]
set_property VCCAUX_IO HIGH [get_ports {o_DDR3_ADDR[15]}]
