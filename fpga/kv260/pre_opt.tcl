# Implementation-time constraints (STEPS.OPT_DESIGN.TCL.PRE of impl_1, set by build.tcl): the ones whose cells only
# exist after synthesis. See synth/vivado/impl.tcl for the out-of-context equivalent.
set c2 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out2}]]
# Two-cycle frame-memory read (2026-10-31, rtl/frame_mem_uram.sv): the UltraRAMs' output registers (OREG, clock-enabled at
# the clk-aligned clk2x edge E_b only) and the output-mux select registers (*_sel_q, loaded at E_b only) change once per
# clk cycle, at E_b, and every consumer samples at E_b: URAM / select -> clk paths are a whole clk cycle (two clk2x
# cycles), launched at the E_b before the capture (-start). The write-request registers (wq_*, loaded at E_b) feed the
# RAMs at the next E_b: a whole clk cycle too. The read-request registers (rq_*, loaded at E_b) feed the RAMs at E_a,
# a true half cycle: no exception.
set c1 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out1}]]
set fmout [get_cells -quiet -hier -filter {(NAME =~ *u_fb*/g_uram[*].u || NAME =~ *u_fb*/*_sel_q_reg*) && IS_SEQUENTIAL}]
if {[llength $fmout]} {
    set_multicycle_path -setup 2 -start -from $fmout -to $c1
    set_multicycle_path -hold 1 -start -from $fmout -to $c1
}
set fmwq [get_cells -quiet -hier -filter {NAME =~ *u_fb*/wq_*_reg* && IS_SEQUENTIAL}]
set fmram [get_cells -quiet -hier -filter {NAME =~ *u_fb*/g_uram[*].u}]
if {[llength $fmwq] && [llength $fmram]} {
    set_multicycle_path -setup 2 -from $fmwq -to $fmram
    set_multicycle_path -hold 1 -from $fmwq -to $fmram
}
# (the phase generator samples clk on the falling clk2x edge since 2026-10-27: no alignment path to except)
