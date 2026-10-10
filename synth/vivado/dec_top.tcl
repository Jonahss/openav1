# Vivado (batch mode) out-of-context synthesis of dec_top for the Kria K26 (xck26). Driven by run.sh, which
# sets SYNTH_ROOT (repo), SYNTH_OUT (report dir), SYNTH_FBX / SYNTH_FBY (frame-buffer log2 size) and
# SYNTH_SOURCES (the rtl file list, same order as tb/runner.py's dec_top entry, frame_mem.sv replaced by the
# black-box stub next to this script).
set_param general.maxThreads 8
set root $::env(SYNTH_ROOT)
set out $::env(SYNTH_OUT)
file mkdir $out
foreach f $::env(SYNTH_SOURCES) { read_verilog -sv $f }
set fp [open $out/clk.xdc w]; puts $fp "create_clock -period 10.000 -name clk \[get_ports clk\]"; close $fp
read_xdc -mode out_of_context $out/clk.xdc
synth_design -top dec_top -part xck26-sfvc784-2LV-c -mode out_of_context -flatten_hierarchy none \
    -generic FBX=$::env(SYNTH_FBX) -generic FBY=$::env(SYNTH_FBY)
report_utilization -file $out/util.rpt
report_utilization -hierarchical -hierarchical_depth 2 -file $out/util_hier.rpt
report_ram_utilization -file $out/ram.rpt
report_timing_summary -max_paths 5 -file $out/timing.rpt
write_checkpoint -force $out/post_synth.dcp
puts "SYNTH_DONE"
exit
