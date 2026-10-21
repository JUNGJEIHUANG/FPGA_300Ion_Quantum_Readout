# Camera-preserving replay build only. Never programs hardware or writes flash.
set root [file normalize [file join [file dirname [info script]] ..]]
set variant $env(T_VIT_VARIANT)
if {$variant ni {dw dw_dilated}} {error "Missing selected decoder variant"}
set mode full
if {[llength $argv]} {set mode [lindex $argv 0]}
if {$mode ni {synth impl full}} {error "Expected synth, impl, or full"}
set cam [file join $root camera]
set linked [file join $root 02_vivado T_preload_camera_linked.dcp]
set prelink [file join $root 02_vivado T_preload_camera_prelink.dcp]
set reports [file join $root reports]
set core [file join $root 02_vivado $variant T_vit_${variant}_synth.dcp]
set cdc [file join $root scripts T_compact_observer_constraints.tcl]
set_param general.maxThreads 1
file mkdir [file join $cam T_work]
cd [file join $cam T_work]
if {$mode ne "impl"} {
    if {[file exists $linked]} {error "Refusing to overwrite linked checkpoint"}
    if {![file exists $core]} {error "Missing packed algorithm OOC checkpoint"}
    puts [exec {D:/anaconda3/python.exe} -I [file join $root scripts T_verify_roms.py] --variant $variant]
    if {[file exists $prelink]} {
        open_checkpoint $prelink
    } else {
    open_project [file join $cam T_depthwise_camera.xpr]
    # save_project_as can inherit an automatically scoped core DCP. Fresh builds
    # remove it here and link explicitly AFTER exporting native camera timing.
    foreach f [get_files -quiet *T_vit_ooc_packed_hier_synth.dcp] {remove_files $f}
    if {[get_property PART [current_project]] ne {xc7k325tffg900-2}} {error "Camera part mismatch"}
    foreach name {T_vit_replay_bridge.sv T_vit_replay_sidecar.sv T_vitseg_core_stub.v T_usb_axi_master.sv} {
        add_files -norecurse [file join $root rtl $name]
    }
    # Keep all camera IPs, remove duplicated copied JTAG-only sidecar sources.
    foreach f [get_files -quiet {*T_vit_replay_bridge.sv *T_vit_replay_sidecar.sv *T_vitseg_core_stub.v}] {
        if {![string match -nocase "$root/rtl/*" [file normalize [get_property NAME $f]]]} {remove_files $f}
    }
    if {![llength [get_ips -quiet T_ila_camera_compact]]} {
        create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name T_ila_camera_compact
    }
    set_property -dict [list CONFIG.C_NUM_OF_PROBES 6 CONFIG.C_DATA_DEPTH 1024 CONFIG.C_EN_STRG_QUAL 1 \
        CONFIG.C_PROBE0_WIDTH 10 CONFIG.C_PROBE1_WIDTH 32 CONFIG.C_PROBE2_WIDTH 32 \
        CONFIG.C_PROBE3_WIDTH 32 CONFIG.C_PROBE4_WIDTH 11 CONFIG.C_PROBE5_WIDTH 1] [get_ips T_ila_camera_compact]
    # Local, source-based IP synthesis avoids concurrent worker memory spikes.
    foreach file [get_files -all -quiet *.xci] {
        set path [file normalize [get_property NAME $file]]
        if {![string match -nocase "$cam/*" $path]} {error "External IP would be modified: $path"}
        set ip [get_ips -quiet [file rootname [file tail $path]]]
        if {![llength $ip] || [get_property IS_LOCKED $ip]} {
            # The baseline has legacy catalog placeholders with active handwritten
            # RTL replacements. Do not mutate/regenerate their unavailable definitions.
            puts "T_SKIP_UNAVAILABLE_OR_CHILD_IP $path"
            continue
        }
        set_property generate_synth_checkpoint false $file
    }
    # Replace only run hooks explicitly tied to the removed observer Gray timebase.
    foreach run [get_runs] {
        foreach prop [list_property $run] {
            if {![string match *TCL.* $prop]} {continue}
            set value [get_property $prop $run]
            if {$value eq "" || ![file exists $value]} {continue}
            set f [open $value r];set text [read $f];close $f
            if {[string first {u_timebase/common_gray_reg} $text]>=0} {set_property $prop $cdc $run}
        }
    }
    set regenerable {}
    foreach ip [get_ips] {if {![get_property IS_LOCKED $ip]} {lappend regenerable $ip}}
    generate_target {synthesis} $regenerable
    report_ip_status -file [file join $reports T_preload_camera_ip_status.rpt]
    set_property top g6504_bringup_top [current_fileset]
    update_compile_order -fileset sources_1
    synth_design -top g6504_bringup_top -part xc7k325tffg900-2 -flatten_hierarchy rebuilt
    write_checkpoint $prelink
    }
    # Vivado's debug hub is an expected placeholder until opt_design implements it.
    # No other unknown black box is permitted; the algorithm is explicitly linked below.
    set bb [get_cells -quiet -hier -filter {IS_BLACKBOX == 1 && NAME != dbg_hub}]
    if {[llength $bb]!=1 || [get_property NAME [lindex $bb 0]] ne {u_vit_replay/u_algorithm}} {error "Unexpected pre-link black boxes: $bb"}
    # Vivado 2022.2 has no delete_clocks command. Export the complete camera
    # constraints before importing OOC timing, then restore that exact set.
    set camera_constraints [file join $reports T_camera_prelink_constraints.xdc]
    set native_clock_periods [dict create]
    foreach clock [get_clocks] {dict set native_clock_periods [get_property NAME $clock] [get_property PERIOD $clock]}
    write_xdc -force $camera_constraints
    read_checkpoint -cell u_vit_replay/u_algorithm $core
    reset_timing
    read_xdc $camera_constraints
    foreach name [dict keys $native_clock_periods] {
        set restored [get_clocks -quiet $name]
        if {[llength $restored]!=1 || abs([get_property PERIOD $restored]-[dict get $native_clock_periods $name])>0.00001} {error "Native camera clock was not restored: $name"}
    }
    if {[llength [get_clocks -quiet *ap_clk*]]} {error "Standalone OOC clock still present after native-constraint restoration"}
    set algorithm_clock [get_clocks -of_objects [get_pins u_vit_replay/u_algorithm/ap_clk]]
    if {[llength $algorithm_clock]!=1 || abs([get_property PERIOD $algorithm_clock]-10.0)>0.01} {error "Algorithm is not on the expected camera 100 MHz clock"}
    if {[llength [get_cells -quiet -hier -filter {IS_BLACKBOX == 1 && NAME != dbg_hub}]]} {error "Unresolved non-debug black boxes after algorithm linking"}
    source $cdc
    set n36 [llength [get_cells -hier -filter {REF_NAME =~ RAMB36* || REF_NAME =~ FIFO36*}]]
    set n18 [llength [get_cells -hier -filter {REF_NAME =~ RAMB18* || REF_NAME =~ FIFO18*}]]
    report_utilization -hierarchical -file [file join $reports T_preload_camera_linked_utilization.rpt]
    report_utilization -file [file join $reports T_preload_camera_linked_summary.rpt]
    report_clocks -file [file join $reports T_preload_camera_linked_clocks.rpt]
    if {2*$n36+$n18>890} {error "Combined design exceeds BRAM: RAMB36=$n36 RAMB18=$n18"}
    write_checkpoint $linked
    puts "T_CAMERA_REPLAY_LINKED bram18_equivalent=[expr {2*$n36+$n18}] NOT_IMPLEMENTED_OR_PROGRAMMED"
    if {$mode eq "synth"} {close_project;exit}
} else {
    if {![file exists $linked]} {error "Missing linked camera checkpoint"}
    open_checkpoint $linked
}
opt_design
if {[llength [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]]} {error "Unresolved black boxes after debug implementation"}
set n36 [llength [get_cells -hier -filter {REF_NAME =~ RAMB36* || REF_NAME =~ FIFO36*}]]
set n18 [llength [get_cells -hier -filter {REF_NAME =~ RAMB18* || REF_NAME =~ FIFO18*}]]
if {2*$n36+$n18>890} {error "Optimized full design exceeds BRAM capacity"}
write_checkpoint [file join $root 02_vivado T_preload_camera_opt.dcp]
place_design -directive Explore
phys_opt_design -directive Explore
route_design -directive Explore
phys_opt_design -directive Explore
report_route_status -file [file join $reports T_preload_camera_route_status.rpt]
report_utilization -hierarchical -file [file join $reports T_preload_camera_routed_utilization.rpt]
report_timing_summary -report_unconstrained -file [file join $reports T_preload_camera_routed_timing.rpt]
report_drc -file [file join $reports T_preload_camera_routed_drc.rpt]
report_methodology -file [file join $reports T_preload_camera_methodology.rpt]
write_checkpoint [file join $root 02_vivado T_preload_camera_routed.dcp]
set setup [get_timing_paths -max_paths 1 -nworst 1 -setup]
set hold [get_timing_paths -max_paths 1 -nworst 1 -hold]
if {![llength $setup] || [get_property SLACK $setup]<0} {error "Setup timing missing/failing"}
if {![llength $hold] || [get_property SLACK $hold]<0} {error "Hold timing missing/failing"}
set bad [get_drc_violations -quiet -filter {SEVERITY == Error || SEVERITY == {Critical Warning}}]
if {[llength $bad]} {error "Blocking DRC: $bad"}
if {[get_property CFGBVS [current_design]] ne {VCCO} || [get_property CONFIG_VOLTAGE [current_design]]!=3.3} {error "Camera configuration voltage constraints missing/changed"}
set out [file join $root output]
file mkdir $out
write_bitstream [file join $out T_camera_vit_depthwise.bit]
write_debug_probes [file join $out T_camera_vit_depthwise.ltx]
set f [open [file join $reports T_preload_implementation_gate.txt] w]
puts $f "CAMERA_PRESERVING_IMPLEMENTATION_TIMING_DRC_PASS"
puts $f "SETUP_SLACK_NS [get_property SLACK $setup]"
puts $f "HOLD_SLACK_NS [get_property SLACK $hold]"
puts $f "PROGRAMMING NOT_PERFORMED"
close $f
puts "T_PRELOAD_CAMERA_BITSTREAM_READY_NOT_PROGRAMMED"
exit
