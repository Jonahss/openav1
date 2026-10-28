# Clock relationship of the double-pumped frame memory (rtl/frame_mem_uram.sv; the same as synth/vivado/synth.tcl
# writes for the out-of-context runs): clk and clk2x come from one MMCM (clkgen), aligned; the memory samples
# clk-domain requests only at the clk-aligned clk2x edge and changes its read data only there, so every
# clk <-> clk2x path is a full clk cycle. The clk2x -> clk direction launches at the clk-aligned edge (-start).
# The phase generator's one-period alignment path is excepted in pre_opt.tcl.
set c1 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out1}]]
set c2 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out2}]]
set_multicycle_path -setup 2 -from $c1 -to $c2
set_multicycle_path -hold 1 -from $c1 -to $c2
