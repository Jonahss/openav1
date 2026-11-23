# Post-route timing repair of an image that missed by a little (fpga/u200/postroute.sh): the routed checkpoint of a
# finished build is opened, phys_opt_design works on the failing paths in place (it keeps the design routed), and the
# reports and the bitstream are written as build.tcl writes them. Nothing but Vivado's own steps touches the design.
# Jonah 2026-11-23: "go ahead and fix the routed checkpoint in vivado" (two images of the wide read port had missed
# inside the PCIe / DDR4 IP, the decoder's clocks met).
set root $::env(U200_ROOT)
set out  $::env(U200_OUT)
source $root/fpga/u200/msg_limit.tcl
set_param general.maxThreads 8
open_checkpoint $::env(U200_SRC_DCP)
proc wns {} { return [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] }
puts "POSTROUTE start: WNS [wns]"
foreach step {{phys_opt_design -directive AggressiveExplore} {route_design -directive Explore} {phys_opt_design -directive AggressiveExplore}} {
    if {[wns] >= 0} break
    puts "POSTROUTE step: $step"
    {*}$step
    puts "POSTROUTE after: WNS [wns]"
}
write_checkpoint -force $out/postroute.dcp
source $root/fpga/u200/reports.tcl
file mkdir $out/proj/u200.runs/impl_1
write_bitstream -force $out/proj/u200.runs/impl_1/sys_wrapper.bit
puts "BUILD_DONE"
exit
