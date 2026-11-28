# trace.tcl <example project .xpr> <pattern dir> <pattern> <start> <clocks> (K60-110): the read-address handshakes at
# every stage of the interconnect's S02 path, decoder clock by decoder clock, from simulated time <start> (e.g. 100us)
# for <clocks> clocks of 12.5 ns. Uses the simulation as last compiled by run.tcl. One line per clock: for each stage
# output, v = valid, r = ready as two digits.
set xpr [lindex $argv 0]; set dir [file normalize [lindex $argv 1]]; set pat [lindex $argv 2]; set t0 [lindex $argv 3]; set n [lindex $argv 4]
open_project $xpr
set_property -name {xsim.simulate.runtime} -value {0ns} -objects [get_filesets sim_1]
set_property -name {xsim.simulate.xsim.more_options} -value "-testplusarg READS=$dir/$pat.reads -testplusarg WRITES=$dir/$pat.writes" -objects [get_filesets sim_1]
launch_simulation
set top [lindex [get_scopes /sim_tb_top/u_example_top/u_sc/*/sc/inst] 0]
puts "SCOPE $top"
set sig {}
foreach c [get_scopes -quiet $top/s02_entry_pipeline/*] { puts "ENTRY $c" }
foreach st {s02_entry_pipeline/s02_mmu s02_entry_pipeline/s02_transaction_regulator s02_entry_pipeline/s02_si_converter s02_axi2sc} {
    set objs [get_objects -quiet -filter {TYPE == in_port || TYPE == out_port} $top/$st/*]
    set names {}
    foreach o $objs { set b [file tail $o]; if {[regexp -nocase {^(s_axi_arvalid|s_axi_arready|m_axi_arvalid|m_axi_arready|s_axi_rvalid|s_axi_rlast|m_axi_rvalid)$} $b]} { lappend names $b } }
    puts "STAGE $st: $names"
    foreach b $names { lappend sig $top/$st/$b }
}
run $t0
puts "TRACE_START [current_time]"
for {set i 0} {$i < $n} {incr i} {
    set line ""
    foreach s $sig { append line [get_value -radix hex $s] " " }
    puts "T $i $line"
    run 12500ps
}
puts "TRACE_END"
close_sim
