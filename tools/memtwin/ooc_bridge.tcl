# ooc_bridge.tcl <rf_ddr_bridge.sv> <outdir> (K60-108): out-of-context synthesis of the bridge for the U200 part, with
# a 300 MHz controller clock and an 80 MHz decoder clock; utilisation and the timing summary of the synthesised netlist
set src [lindex $argv 0]; set out [lindex $argv 1]; file mkdir $out
read_verilog -sv $src
synth_design -top rf_ddr_bridge -part xcu200-fsgd2104-2-e -mode out_of_context
create_clock -name uclk -period 3.332 [get_ports uclk]
create_clock -name dclk -period 12.5 [get_ports dclk]
set_clock_groups -asynchronous -group uclk -group dclk
report_utilization -file $out/util.rpt
report_timing_summary -file $out/timing.rpt
report_cdc -file $out/cdc.rpt
puts "OOC_DONE"
