# smoke.tcl (K60-109): the example design stock simulation in xsim; stops at the testbench own $finish or after 2 ms
open_project [lindex $argv 0]
set_property -name {xsim.simulate.runtime} -value {0ns} -objects [get_filesets sim_1]
launch_simulation
run 2ms
puts "SMOKE_TIME [current_time]"
puts "SMOKE_CALIB [get_value /sim_tb_top/c0_init_calib_complete]"
close_sim
