# ==============================================================================
# Vivado Project Reconstruction Script for GSENSE6504BSI Camera Control FPGA
# Target Device: Xilinx Kintex-7 xc7k325tffg900-2 (CC2405 Evaluation Platform)
# Usage: vivado -mode batch -source create_vivado_project.tcl
# ==============================================================================

set proj_name "G6504_Camera_Control"
set proj_dir  "./vivado_project"
set part_name "xc7k325tffg900-2"

create_project -force $proj_name $proj_dir -part $part_name

# 1. Add Verilog RTL Sources & Sensor Initialization Memory Files
add_files -fileset sources_1 [glob -nocomplain ./rtl/top/*.v]
add_files -fileset sources_1 [glob -nocomplain ./rtl/sensor_ctrl/*.v]
add_files -fileset sources_1 [glob -nocomplain ./rtl/sensor_ctrl/*.mem]
add_files -fileset sources_1 [glob -nocomplain ./rtl/lvds_rx_training/*.v]
add_files -fileset sources_1 [glob -nocomplain ./rtl/lvds_rx_training/training/*.v]
add_files -fileset sources_1 [glob -nocomplain ./rtl/lvds_rx_training/training_dbg/*.v]
add_files -fileset sources_1 [glob -nocomplain ./rtl/image_ddr_path/*.v]
add_files -fileset sources_1 [glob -nocomplain ./rtl/usb3_fx3/*.v]
add_files -fileset sources_1 [glob -nocomplain ./rtl/usb3_fx3/protocol_ctrl/*.v]
add_files -fileset sources_1 [glob -nocomplain ./rtl/timing_instrumentation/*.v]

# 2. Add Xilinx IP Core Configurations (.xci)
foreach xci_file [glob -nocomplain ./ip_xci/*/*.xci] {
    import_ip -files $xci_file
}

# 3. Add Physical & Timing Constraints (.xdc)
add_files -fileset constrs_1 [glob -nocomplain ./constraints/*.xdc]

# 4. Set Top-Level Module
set_property top g6504_bringup_top [current_fileset]
update_compile_order -fileset sources_1

puts "Vivado project '$proj_name' created successfully in $proj_dir ($part_name)."
