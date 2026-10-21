# Only the compact observer's first-stage, single-bit event synchronizers.
# The old Gray-bus hook is not applicable: its raw trace transport was removed.
# No sensor, image, DDR, USB, or algorithm datapath is excepted here.
set_param general.maxThreads 1
set event_meta [get_pins -quiet -hier -filter {NAME =~ *u_rough_timing/evt_meta_reg*/D}]
if {[llength $event_meta]!=6} {error "Expected six compact-observer event synchronizer inputs"}
set_false_path -to $event_meta
set request_meta [get_pins -quiet -hier -regexp {.*u_rough_timing/req_sync_reg\[0\]/D}]
if {[llength $request_meta]!=1} {error "Expected one compact-observer request synchronizer input"}
set_false_path -to $request_meta
puts "T_COMPACT_OBSERVER_CDC first_stage_events=6 first_stage_request=1"
