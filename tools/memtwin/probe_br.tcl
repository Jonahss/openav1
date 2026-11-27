# probe_br.tcl <xpr> <pattern dir> <pattern> (K60-108): the bridge testbench as last compiled, run in steps, printing
# the second player's state at each stop (to see where a run goes silent)
set xpr [lindex $argv 0]; set dir [file normalize [lindex $argv 1]]; set pat [lindex $argv 2]
open_project $xpr
set_property -name {xsim.simulate.runtime} -value {0ns} -objects [get_filesets sim_1]
set_property -name {xsim.simulate.xsim.more_options} -value "-testplusarg READS=$dir/$pat.reads -testplusarg WRITES=$dir/$pat.writes -testplusarg READSB=$dir/$pat.readsB -testplusarg WRITESB=$dir/$pat.writesB -testplusarg CHECK=1 -testplusarg CHECKB=1" -objects [get_filesets sim_1]
launch_simulation
set T /sim_tb_top/u_example_top
set B $T/u_player_b
foreach step {100us 100us 60us 20us 20us 20us 20us 20us 40us} {
    run $step
    set line "AT [current_time]:"
    foreach s {cyc n_wr n_b r_cnt r_gap outstanding idle_cnt w_cnt w_left} { append line " $s [get_value -radix unsigned $B/$s]" }
    foreach s {r_have r_eof w_have w_eof w_data_phase done} { append line " $s [get_value $B/$s]" }
    foreach s {tb_arvalid tb_arready tb_rvalid tb_awvalid tb_awready tb_wvalid tb_wready tb_bvalid sc_arvalid sc_arready sc_rvalid sc_rready sc_awvalid sc_awready sc_wvalid sc_wready sc_bvalid a_done b_done} { append line " $s [get_value $T/$s]" }
    puts $line
}
close_sim
