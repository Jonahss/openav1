# Implementation-time constraints (STEPS.OPT_DESIGN.TCL.PRE of impl_1, set by build.tcl): the ones whose cells only
# exist after synthesis. See synth/vivado/impl.tcl for the out-of-context equivalent.
set c2 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out2}]]
# the double-pumped UltraRAM frame memory changes its read data only at the clk-aligned clk2x edge and every
# consumer samples at a clk-aligned edge, so URAM -> clk2x paths (read data -> recon -> write request) are a clk
# cycle long
# (the launching cells: the UltraRAMs and the XPM's column-select registers, all loaded by the read)
# (the launching cells: the UltraRAMs and the output-mux select registers of every frame-memory ring, all loaded at E_b)
set fmregs [get_cells -quiet -hier -filter {(NAME =~ *u_fb*/g_uram[*].u || NAME =~ *u_fb*/*_sel_r_reg*) && IS_SEQUENTIAL}]
if {0 && [llength $fmregs]} {   # read data registered at E_a since 2026-10-27: single-cycle paths, no exception
    set_multicycle_path -setup 2 -from $fmregs -to $c2
    set_multicycle_path -hold 1 -from $fmregs -to $c2
}
# the phase generator's alignment path (rtl/phase_gen.sv): a clk register sampled by a clk2x register at the very
# next clk2x edge, i.e. one clk2x period, unlike every other clk -> clk2x path (sys.xdc)
set rq [get_cells -quiet -hier -filter {NAME =~ *pg/*rst_q_reg*}]
set ph [get_cells -quiet -hier -filter {NAME =~ *pg/*phase_reg*}]
if {[llength $rq] && [llength $ph]} {
    set_multicycle_path -setup 1 -from $rq -to $ph
    set_multicycle_path -hold 0 -from $rq -to $ph
} else {
    puts "CRITICAL WARNING: phase_gen registers not found; the clk -> clk2x multicycle applies to the alignment path"
}
