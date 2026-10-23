# Alveo U200: bitstream / configuration bank settings (UG1289; the config bank is 1.8 V) and the clock
# relationship of the double-pumped frame memory, as fpga/kv260/sys.xdc (clk and clk2x from one MMCM, aligned;
# every clk <-> clk2x path is a full clk cycle; the phase generator's alignment path is excepted in
# fpga/kv260/pre_opt.tcl, which this build also runs). The pin locations come from the board interfaces (AMD's
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
set_multicycle_path -hold 1 -from $c1 -to $c2
set_multicycle_path -setup 2 -start -from $c2 -to $c1
set_multicycle_path -hold 1 -start -from $c2 -to $c1
