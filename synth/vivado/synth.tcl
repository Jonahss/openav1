# Vivado (batch mode) out-of-context synthesis of SYNTH_TOP (dec_top or one block) for the Kria K26 (xck26).
# Driven by run.sh, which sets SYNTH_ROOT (repo), SYNTH_OUT (report dir), SYNTH_GENERICS (-generic flags) and
# SYNTH_SOURCES (the rtl file list, same order as tb/runner.py's dec_top entry, frame_mem.sv replaced by the
# black-box stub next to this script).
set_param general.maxThreads 8
set root $::env(SYNTH_ROOT)
set out $::env(SYNTH_OUT)
file mkdir $out
foreach f $::env(SYNTH_SOURCES) { read_verilog -sv $f }
set fp [open $out/clk.xdc w]; puts $fp "create_clock -period 10.000 -name clk \[get_ports clk\]"; close $fp
# a double-pumped memory (rtl/frame_mem_uram.sv) has a 2x clock port: run.sh sets the env var when the top has one
if {[info exists ::env(SYNTH_CLK2X)]} {
    set fp [open $out/clk.xdc a]
    puts $fp "create_clock -period 5.000 -name clk2x \[get_ports clk2x\]"
    # the double-pumped memory samples clk-domain requests only on the clk-aligned clk2x edge and changes its
    # read data only there (rtl/frame_mem_uram.sv), so every clk <-> clk2x path is a full clk cycle
    puts $fp "set_multicycle_path -setup 2 -from \[get_clocks clk\] -to \[get_clocks clk2x\]"
    puts $fp "set_multicycle_path -hold 1 -from \[get_clocks clk\] -to \[get_clocks clk2x\]"
    # fast -> slow: the launch is the clk-aligned clk2x edge, one clk2x period before the default (-start)
    puts $fp "set_multicycle_path -setup 2 -start -from \[get_clocks clk2x\] -to \[get_clocks clk\]"
    puts $fp "set_multicycle_path -hold 1 -start -from \[get_clocks clk2x\] -to \[get_clocks clk\]"
    close $fp
}
read_xdc -mode out_of_context $out/clk.xdc
eval synth_design -top $::env(SYNTH_TOP) -part xck26-sfvc784-2LV-c -mode out_of_context -flatten_hierarchy none $::env(SYNTH_GENERICS)
report_utilization -file $out/util.rpt
report_utilization -hierarchical -hierarchical_depth 2 -file $out/util_hier.rpt
report_ram_utilization -file $out/ram.rpt
report_timing_summary -max_paths 5 -file $out/timing.rpt
write_checkpoint -force $out/post_synth.dcp
puts "SYNTH_DONE"
exit
