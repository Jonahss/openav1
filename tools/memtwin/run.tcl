# run.tcl <example project .xpr> <src dir> <pattern dir> <sc netlist or -> <pattern> [<pattern> ...] (K60-110): the
# memory twin in xsim. Compiles, then runs each pattern (files <dir>/<pattern>.reads and, if present,
# <dir>/<pattern>.writes) from time 0 (calibration included, about 44 us of simulated time) and prints the player's
# RESULT lines. <sc netlist>: sys_sc_mem_0_sim_netlist.v of the image build when example_top was patched with `sc`.
# Environment: TWIN_OUT (reads outstanding at the player, default 16), TWIN_DEC_PS (decoder clock period in ps),
# TWIN_ARGS (further plusargs, e.g. "CHECK=1 CHECKB=1").
set xpr [lindex $argv 0]; set src [lindex $argv 1]; set dir [file normalize [lindex $argv 2]]; set net [lindex $argv 3]
open_project $xpr
# the player and the bridge are taken from <src dir> on every run (a mutant copy of the directory can be named)
# (the bridge is fpga/u200/rf_ddr_bridge.sv of the repo <src dir> belongs to, unless <src dir> holds a copy of its own)
foreach f {twin_player.sv rf_ddr_bridge.sv} {
    if {[llength [get_files -quiet $f]] > 0} { remove_files -fileset sim_1 [get_files $f] }
    foreach cand [list $src/$f $src/../../fpga/u200/$f] {
        if {[file exists $cand]} { add_files -fileset sim_1 -norecurse [file normalize $cand]; puts "SOURCE [file normalize $cand]"; break }
    }
}
if {[string match *.f $net]} {
    # a regenerated interconnect (mk_sc.tcl): its list of simulation sources
    set fh [open $net]; set fl [split [string trim [read $fh]] "\n"]; close $fh
    foreach x $fl { if {[llength [get_files -quiet $x]] == 0} { add_files -fileset sim_1 -norecurse $x } }
} elseif {$net ne "-" && [llength [get_files -quiet [file tail $net]]] == 0} { add_files -fileset sim_1 -norecurse $net }
set_property XPM_LIBRARIES {XPM_CDC XPM_FIFO XPM_MEMORY} [current_project]
update_compile_order -fileset sim_1
set_property -name {xsim.simulate.runtime} -value {0ns} -objects [get_filesets sim_1]
foreach pat [lrange $argv 4 end] {
    set args "-testplusarg READS=$dir/$pat.reads -testplusarg WRITES=$dir/$pat.writes"
    if {[info exists ::env(TWIN_OUT)]} { append args " -testplusarg OUT=$::env(TWIN_OUT)" }
    if {[info exists ::env(TWIN_DEC_PS)]} { append args " -testplusarg DEC_PS=$::env(TWIN_DEC_PS)" }
    # the second player of the bridge testbench takes <pattern>.readsB / .writesB; TWIN_ARGS: further plusargs (CHECK=1 CHECKB=1)
    append args " -testplusarg READSB=$dir/$pat.readsB -testplusarg WRITESB=$dir/$pat.writesB"
    if {[info exists ::env(TWIN_ARGS)]} { foreach x $::env(TWIN_ARGS) { append args " -testplusarg $x" } }
    set_property -name {xsim.simulate.xsim.more_options} -value $args -objects [get_filesets sim_1]
    puts "PATTERN $pat"
    launch_simulation
    run 400ms
    puts "PATTERN_END $pat at [current_time]"
    if {[info exists ::env(TWIN_PROBE)]} {
        foreach g [split $::env(TWIN_PROBE) ,] {
            foreach o [lrange [get_objects -quiet $g] 0 40] { puts "PROBE $o = [get_value -radix hex $o]" }
        }
    }
    close_sim
}
puts "RUN_DONE"
