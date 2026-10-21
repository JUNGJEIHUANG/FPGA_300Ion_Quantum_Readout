# Isolated decoder-variant HLS flow, no camera project mutation or programming.
set root [file normalize [file join [file dirname [info script]] ..]]
set variant $env(T_VIT_VARIANT)
set mode $env(T_VIT_HLS_MODE)
if {$variant ni {dw dw_dilated dw_dilated_fast} || $mode ni {csim synth}} {error "Expected decoder variant and mode csim/synth"}
set source [file join $root 01_hls src_$variant]
if {![file exists [file join $source vit_parameters.h]]} {error "Missing frozen parameter header"}
cd [file join $root 01_hls]
open_project vitseg_$variant
set_top vitseg_core
set flags "-std=c++14 -ffp-contract=off"
if {$mode eq "csim"} {append flags " -O3"}
add_files [file join $source vitseg_core.cpp] -cflags $flags
add_files -tb [file join $root 01_hls tb_$variant T_tb_vitseg.cpp] -cflags "-std=c++14 -O3 -ffp-contract=off"
open_solution solution1 -flow_target vivado
set_part {xc7k325tffg900-2}
create_clock -period 10
config_compile -pipeline_loops 0
if {$mode eq "csim"} {
    set old {E:/ISCAS_3/vit0392_fpga_deploy_20261020/testdata/validation_150D_150B_6_input_q18.bin}
    csim_design -clean -argv "$old [file join $root reports T_csim_${variant}_mixed]"
} else {
    if {![file exists [file join $root reports T_${variant}_csim_pass.json]]} {error "Csim gate missing"}
    puts [exec {D:/anaconda3/python.exe} -I [file join $root scripts T_verify_csim_gate.py] --variant $variant]
    csynth_design
}
puts "T_VARIANT_HLS_COMPLETE $variant $mode"
exit
