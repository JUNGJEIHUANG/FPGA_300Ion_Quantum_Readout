# Isolated out-of-context synthesis; this is not camera-fit or timing signoff.
set root [file normalize [file join [file dirname [info script]] ..]]
set variant $env(T_VIT_VARIANT)
if {$variant ni {dw dw_dilated dw_dilated_fast}} {error "Expected decoder variant"}
set dir [file join $root 02_vivado $variant]
set rtl [file join $dir T_rtl_snapshot]
if {![file exists [file join $rtl vitseg_core.v]]} {error "Missing packed HLS RTL"}
if {[file exists [file join $dir project]]} {error "Refusing to overwrite OOC project"}
set work [file join $dir T_work]
file mkdir $work
cd $work
set_param general.maxThreads 1
create_project T_vit_${variant} [file join $dir project] -part xc7k325tffg900-2
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]
add_files [glob [file join $rtl *.v]]
add_files [glob [file join $rtl *.dat]]
foreach script [glob [file join $rtl *_ip.tcl]] {source $script}
set_property top vitseg_core [current_fileset]
update_compile_order -fileset sources_1
set clock_file [file join $dir T_ooc_clock.xdc]
set f [open $clock_file w]
puts $f {create_clock -name ap_clk -period 10.000 [get_ports ap_clk]}
close $f
add_files -fileset constrs_1 $clock_file
synth_design -top vitseg_core -part xc7k325tffg900-2 -mode out_of_context -flatten_hierarchy none
report_utilization -hierarchical -file [file join $root reports T_${variant}_ooc_utilization.rpt]
report_utilization -file [file join $root reports T_${variant}_ooc_summary.rpt]
report_timing_summary -report_unconstrained -file [file join $root reports T_${variant}_ooc_timing.rpt]
report_drc -file [file join $root reports T_${variant}_ooc_drc.rpt]
write_checkpoint [file join $dir T_vit_${variant}_synth.dcp]
set bb [get_cells -hier -filter {IS_BLACKBOX == 1}]
if {[llength $bb]} {error "Unresolved black boxes: $bb"}
puts "T_VIT_VARIANT_OOC_SYNTH_COMPLETE $variant"
exit
