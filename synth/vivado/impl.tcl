# Vivado (batch mode) implementation of an out-of-context checkpoint from synth.tcl: opt, place, physical
# optimisation, route, reports. Driven by impl.sh (IMPL_OUT = the synth output directory).
set out $::env(IMPL_OUT)
open_checkpoint $out/post_synth.dcp
set_param general.maxThreads 8
# the out-of-context clock: tell the tools where it comes from so the timer stops warning
set_property HD.CLK_SRC BUFGCTRL_X0Y0 [get_ports clk]
if {[llength [get_ports -quiet clk2x]]} { set_property HD.CLK_SRC BUFGCTRL_X0Y1 [get_ports clk2x] }
# The double-pumped UltraRAM frame memory (rtl/frame_mem_uram.sv) changes its read data only at the clk-aligned
# clk2x edge and everything that consumes it samples at a clk-aligned edge: the decoder at clk (covered by the
# clk2x -> clk multicycle in clk.xdc) and the memory's own write-request capture at the next clk-aligned clk2x
# edge. So a path from a URAM output into the clk2x domain (read data -> recon add -> write request) has a full
# clk cycle, not a clk2x one. The launching cells are the UltraRAMs and the XPM's column-select registers (both
# loaded by a read at the clk-aligned edge); they only exist after synthesis, hence the constraint lives here.
# (the launching cells: the UltraRAMs and the output-mux select registers of every frame-memory ring, all loaded at E_b)
set fmregs [get_cells -quiet -hier -filter {(NAME =~ *u_fb*/g_uram[*].u || NAME =~ *u_fb*/*_sel_r_reg*) && IS_SEQUENTIAL}]
if {[llength $fmregs]} {
    set_multicycle_path -setup 2 -from $fmregs -to [get_clocks clk2x]
    set_multicycle_path -hold 1 -from $fmregs -to [get_clocks clk2x]
}
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
