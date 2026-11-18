# Alveo U200: bitstream / configuration bank settings (UG1289; the config bank is 1.8 V) and the clock
# relationship of the double-pumped frame memory, as it was on the KV260 (clk and clk2x from one MMCM, aligned;
# clk -> clk2x paths (requests captured at E_b) are a full clk cycle; the read data comes back registered at E_a
# (frame_mem_uram), so clk2x -> clk paths are single cycles; the phase generator's alignment path is excepted in
# fpga/u200/pre_opt.tcl). The pin locations come from the board interfaces (AMD's
# au200 board files through the XDMA, util_ds_buf and DDR4 IP constraints), not from here; build.tcl prints the
# chosen pins at the end for checking.
set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLUP [current_design]
set_property BITSTREAM.CONFIG.OVERTEMPSHUTDOWN Enable [current_design]
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
