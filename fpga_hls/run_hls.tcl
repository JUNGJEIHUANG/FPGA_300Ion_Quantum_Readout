# Vitis HLS 2022.2 Synthesis Script for xc7k325tffg900-2
open_project -reset proj_dwnet_unet

# Add design files
add_files dwnet_unet.cpp
add_files streaming_ccl.cpp
add_files dwnet_weights_all.cpp

# Add testbench
add_files -tb tb_dwnet_unet.cpp

# Set top-level function name
set_top dwnet_unet

# Create solution
open_solution -reset "solution1" -flow_target vivado

# Set target FPGA part and speed grade (Kintex-7 325T, -2 speed grade)
set_part {xc7k325tffg900-2}

# Target clock period: 5.0 ns (200 MHz)
create_clock -period 5.0 -name default

# Disable automatic array partitioning on large unpartitioned Dual-Port ROMs
config_array_partition -throughput_driven off
set_param hls.enable_auto_array_partition false
set_param hls.enable_auto_parallel_performance_driven false
set_param hls.enable_reflow_partition_for_automation false

# Run HLS C-Simulation (behavioral test)
# csim_design

# Run HLS C-Synthesis (translate to Verilog RTL)
csynth_design

# Exit Vitis HLS
exit
