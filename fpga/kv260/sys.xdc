# Clock relationship of the double-pumped frame memory (rtl/frame_mem_uram.sv; the same as synth/vivado/synth.tcl
# writes for the out-of-context runs): clk and clk2x come from one MMCM (clkgen), aligned; the memory samples
# clk-domain requests only at the clk-aligned clk2x edge and changes its read data only there, so every
# clk <-> clk2x path is a full clk cycle. The clk2x -> clk direction launches at the clk-aligned edge (-start).
# The phase generator's one-period alignment path is excepted in pre_opt.tcl.
set c1 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out1}]]
set c2 [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *clkgen/clk_out2}]]
set_multicycle_path -setup 2 -from $c1 -to $c2
# Hold (2026-11-16): -end. clk -> clk2x is slow to fast, and a hold exception without -end is anchored at the launch
# clock: Vivado then checked hold half a clk period BEFORE the launch edge (requirement -4.998 ns on every path), so
# the router never repaired hold on the frame memory's request registers. The true check is at the launch edge itself
# (requirement 0): a request register must not capture, at the clk-aligned clk2x edge, what clk launches at that same
# edge. Two timing-clean chain images (c86eb7f, 100 and 90 MHz) decoded wrongly with about -0.8 ns of real hold slack
# on fc_out -> u_fb2/wq_addr; Vivado had flagged the pair all along as TIMING-29. build.tcl now refuses an image
# whose clk -> clk2x hold requirement is not 0.
set_multicycle_path -hold 1 -end -from $c1 -to $c2
