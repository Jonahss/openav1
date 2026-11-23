# The reports of a routed design and the hold-anchor gate (sourced by build.tcl and postroute.tcl, which set $out and
# have the design open).
report_timing_summary -max_paths 10 -file $out/impl_timing.rpt
# Hard gate (2026-11-16): the clk <-> clk2x exceptions must leave every hold check at the launch edge. A hold exception
# anchored at the wrong clock (sys.xdc before this date) shows as a non-zero hold requirement and as TIMING-29 in the
# methodology report; either one means the timing summary above is not to be believed, so no BUILD_DONE.
set hc1 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out1}]]
set hc2 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out2}]]
set hbad 0; set hn 0; set hworst 99
foreach {hf ht} [list $hc1 $hc2 $hc2 $hc1] {
    foreach p [get_timing_paths -from $hf -to $ht -max_paths 100000 -nworst 1 -hold] {
        incr hn
        if {[regexp {clk_s_reg} [get_property ENDPOINT_PIN $p]]} continue   ;# the phase generator samples the clock itself
        if {abs([get_property REQUIREMENT $p]) > 0.001} { incr hbad }
        if {[get_property SLACK $p] < $hworst} { set hworst [get_property SLACK $p] }
    }
}
report_methodology -file $out/impl_methodology.rpt
set fh [open $out/impl_methodology.rpt r]; set t29 [regexp -all {TIMING-29#} [read $fh]]; close $fh
puts "HOLD_ANCHOR clk<->clk2x: $hn hold endpoints, $hbad with a requirement other than 0, worst hold slack $hworst; TIMING-29: $t29"
if {$hbad > 0 || $t29 > 0} { puts "HOLD_ANCHOR_FAILED: a clk <-> clk2x hold check is not at the launch edge"; exit 1 }
report_utilization -file $out/impl_util.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file $out/impl_util_hier.rpt
report_route_status -file $out/impl_route_status.rpt
# the board pins the IP constraints chose, for checking against the board file / UG1289
foreach p [get_ports -filter {NAME =~ *refclk* || NAME =~ *perstn* || NAME =~ *300mhz* || NAME =~ *ddr4*_act_n || NAME =~ *ddr4*_reset_n}] {
    puts "PIN $p [get_property PACKAGE_PIN $p] [get_property IOSTANDARD $p]"
}
puts "PCIE block: [get_property LOC [get_cells -hier -filter {REF_NAME =~ PCIE40E4* || REF_NAME =~ PCIE4CE4*}]]"
