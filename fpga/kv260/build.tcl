# Vivado (batch) build of the KV260 bitstream: a block design with the Zynq MPSoC PS, an MMCM making the decoder
# clock and its 2x from pl_clk0, the phase generator, and dec_host (the AXI-Lite register file around the core
# decoder) on the PS's HPM0 master at 0xA000_0000. Driven by build.sh, which sets KV260_ROOT (repo), KV260_OUT
# (work dir, with the stubs and the rtl symlinks), KV260_MHZ (decoder clock) and KV260_SOURCES (the rtl list).
set root $::env(KV260_ROOT)
set out  $::env(KV260_OUT)
set mhz  $::env(KV260_MHZ)
set part xck26-sfvc784-2LV-c
set_param general.maxThreads 8
create_project -force kv260 $out/proj -part $part
set_property target_language Verilog [current_project]
add_files -norecurse $::env(KV260_SOURCES)
set_property file_type SystemVerilog [get_files -filter {NAME =~ *.sv}]
set_property file_type Verilog [get_files -filter {NAME =~ *_v.v}]
add_files -fileset constrs_1 -norecurse $root/fpga/kv260/sys.xdc
update_compile_order -fileset sources_1

# ---------------------------------------------------------------- the block design
create_bd_design sys
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 ps]
# only what the PL needs: HPM0 FPD as the register master, pl_clk0 at 100 MHz, pl_resetn0. The rest of the PS
# configuration is the boot firmware's business (the bitstream is loaded under a running Linux) - including the
# port's fabric width: FPD_SLCR.AFI_FS is set by the K26 firmware's psu_init to 128 bits for HPM0/HPM1, and a PL
# built for 32 bits sees only lane 0 (2026-10-14: registers at offsets 4/8/12 mod 16 unreachable). So 128 here;
# the smartconnect narrows to the AXI-Lite slave.
set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__MAXIGP0__DATA_WIDTH {128} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} CONFIG.PSU__USE__M_AXI_GP2 {0} \
    CONFIG.PSU__USE__S_AXI_GP0 {0} CONFIG.PSU__USE__S_AXI_GP1 {0} CONFIG.PSU__USE__S_AXI_GP2 {0} \
    CONFIG.PSU__USE__S_AXI_GP3 {0} CONFIG.PSU__USE__S_AXI_GP4 {0} CONFIG.PSU__USE__S_AXI_GP5 {0} CONFIG.PSU__USE__S_AXI_GP6 {0} \
    CONFIG.PSU__FPGA_PL0_ENABLE {1} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
    CONFIG.PSU__FPGA_PL1_ENABLE {0} CONFIG.PSU__FPGA_PL2_ENABLE {0} CONFIG.PSU__FPGA_PL3_ENABLE {0} \
    CONFIG.PSU__USE__IRQ0 {0}] $ps
set clkgen [create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:6.0 clkgen]
# the input frequency is propagated from pl_clk0 at validation (96.97 MHz as the PS IP computes it without the
# board preset; a user-set PRIM_IN_FREQ must match it to the hertz, so it is left alone). The overlay asks the
# running Linux for 100 MHz, so the real decoder clock is ~3% above the analysed one: KV260_MHZ keeps margin.
puts "pl_clk0 = [get_property CONFIG.FREQ_HZ [get_bd_pins ps/pl_clk0]] Hz"
set_property -dict [list \
    CONFIG.PRIMITIVE {MMCM} CONFIG.USE_LOCKED {true} CONFIG.USE_RESET {false} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ $mhz CONFIG.CLKOUT2_USED {true} CONFIG.CLKOUT2_REQUESTED_OUT_FREQ [expr {2 * $mhz}] \
    CONFIG.CLKOUT1_REQUESTED_PHASE {0.000} CONFIG.CLKOUT2_REQUESTED_PHASE {0.000}] $clkgen
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst]
set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 sc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $sc
set host [create_bd_cell -type module -reference dec_host_v host]
set_property -dict [list CONFIG.FBX {9} CONFIG.FBY {9} CONFIG.ITX_LANES {2} CONFIG.FRAME_URAM {1}] $host
set pg [create_bd_cell -type module -reference phase_gen_v pg]

connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins clkgen/clk_in1]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins rst/ext_reset_in]
connect_bd_net [get_bd_pins clkgen/locked] [get_bd_pins rst/dcm_locked]
connect_bd_net [get_bd_pins clkgen/clk_out1] [get_bd_pins ps/maxihpm0_fpd_aclk] [get_bd_pins sc/aclk] \
    [get_bd_pins host/clk] [get_bd_pins rst/slowest_sync_clk] [get_bd_pins pg/clk]
connect_bd_net [get_bd_pins clkgen/clk_out2] [get_bd_pins host/clk2x] [get_bd_pins pg/clk2x]
connect_bd_net [get_bd_pins rst/peripheral_aresetn] [get_bd_pins sc/aresetn] [get_bd_pins host/aresetn]
connect_bd_net [get_bd_pins rst/peripheral_reset] [get_bd_pins pg/rst_in]
connect_bd_net [get_bd_pins pg/phase] [get_bd_pins host/phase]
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins sc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins sc/M00_AXI] [get_bd_intf_pins host/s_axi]
assign_bd_address -offset 0xA0000000 -range 4K -target_address_space [get_bd_addr_spaces ps/Data] \
    [get_bd_addr_segs host/s_axi/reg0]
validate_bd_design
puts "clk = [get_property CONFIG.FREQ_HZ [get_bd_pins clkgen/clk_out1]] Hz, clk2x = [get_property CONFIG.FREQ_HZ [get_bd_pins clkgen/clk_out2]] Hz"
save_bd_design
set bd [get_files sys.bd]
generate_target all $bd
make_wrapper -files $bd -top
add_files -norecurse [file join $out proj kv260.gen sources_1 bd sys hdl sys_wrapper.v]
set_property top sys_wrapper [current_fileset]
update_compile_order -fileset sources_1

# ---------------------------------------------------------------- runs
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.OPT_DESIGN.TCL.PRE $root/fpga/kv260/pre_opt.tcl [get_runs impl_1]
launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { puts "SYNTH_FAILED"; exit 1 }
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { puts "IMPL_FAILED"; exit 1 }
open_run impl_1
report_timing_summary -max_paths 10 -file $out/impl_timing.rpt
report_utilization -file $out/impl_util.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file $out/impl_util_hier.rpt
report_route_status -file $out/impl_route_status.rpt
puts "BUILD_DONE"
exit
