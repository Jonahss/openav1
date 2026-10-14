# Vivado (batch mode) implementation of an out-of-context checkpoint from synth.tcl: opt, place, physical
# optimisation, route, reports. Driven by impl.sh (IMPL_OUT = the synth output directory).
set out $::env(IMPL_OUT)
open_checkpoint $out/post_synth.dcp
set_param general.maxThreads 8
# the out-of-context clock: tell the tools where it comes from so the timer stops warning
set_property HD.CLK_SRC BUFGCTRL_X0Y0 [get_ports clk]
opt_design
place_design
phys_opt_design
route_design
report_route_status -file $out/impl_route_status.rpt
report_timing_summary -max_paths 10 -file $out/impl_timing.rpt
report_utilization -file $out/impl_util.rpt
report_utilization -hierarchical -hierarchical_depth 2 -file $out/impl_util_hier.rpt
write_checkpoint -force $out/post_route.dcp
puts "IMPL_DONE"
exit
