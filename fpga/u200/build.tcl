# Vivado (batch) build of the Alveo U200 bitstream: a block design with the PCIe endpoint + DMA (XDMA), one of
# the card's DDR4 banks, an MMCM making the decoder clock and its 2x, the phase generator, and dec_host (the
# AXI-Lite register file around the decoder) reached from the host through XDMA's AXI-Lite master (BAR0) and from
# the Vivado hardware manager through a JTAG-to-AXI master. The decoder's AXI write/read master (picture flush,
# reference fetch) and XDMA's AXI memory-mapped master (the host reading pictures, loading streams) share the
# DDR4 bank. Driven by build.sh, which sets U200_ROOT (repo), U200_OUT (work dir), U200_MHZ, U200_FBX/FBY/WIN/
# INTER, U200_DDR (bank), U200_BOARD_REPO (AMD's au200 board files) and U200_SOURCES (the rtl list).
#
# Address maps (docs/u200-bringup.md):
#   host (BAR0, /dev/xdma0_user) and JTAG master:  0x0000_0000 .. 0x0000_0FFF  dec_host registers (as on the KV260)
#                                                  0x0001_0000 .. 0x0001_FFFF  DDR4 controller ECC status (PG150)
#   host DMA (/dev/xdma0_h2c_0, _c2h_0) and the decoder's m_axi:  0x0_0000_0000 .. 0x3_FFFF_FFFF  the 16 GB DDR4 bank
set root $::env(U200_ROOT)
set out  $::env(U200_OUT)
set mhz  $::env(U200_MHZ)
set part xcu200-fsgd2104-2-e
set_param general.maxThreads 8
set_param board.repoPaths [list $::env(U200_BOARD_REPO)]
create_project -force u200 $out/proj -part $part
set bp [lindex [lsort -dictionary [get_board_parts xilinx.com:au200:part0:*]] end]
if {$bp eq ""} { error "no au200 board part found under $::env(U200_BOARD_REPO)" }
puts "board part: $bp"
set_property board_part $bp [current_project]
set_property target_language Verilog [current_project]
add_files -norecurse $::env(U200_SOURCES)
set_property file_type SystemVerilog [get_files -filter {NAME =~ *.sv}]
set_property file_type Verilog [get_files -filter {NAME =~ *_v.v}]
add_files -fileset constrs_1 -norecurse $root/fpga/u200/sys.xdc
update_compile_order -fileset sources_1

# ---------------------------------------------------------------- the block design
create_bd_design sys

# PCIe: XDMA, Gen3 x16, 512-bit AXI at 250 MHz, one H2C and one C2H channel, AXI-Lite master for BAR0 (the
# register file), no interrupts. The GT quad / PCIe block location, the reference clock and PERST pins come from
# the board interfaces (pci_express_x16, pcie_refclk, pcie_perstn: the IP's own constraints carry the LOCs).
set xdma [create_bd_cell -type ip -vlnv xilinx.com:ip:xdma:4.2 xdma]
set_property -dict [list \
    CONFIG.PCIE_BOARD_INTERFACE {pci_express_x16} CONFIG.SYS_RST_N_BOARD_INTERFACE {pcie_perstn} \
    CONFIG.pl_link_cap_max_link_width {X16} CONFIG.pl_link_cap_max_link_speed {8.0_GT/s} \
    CONFIG.axi_data_width {512_bit} CONFIG.axisten_freq {250} \
    CONFIG.xdma_axi_intf_mm {AXI_Memory_Mapped} CONFIG.xdma_rnum_chnl {1} CONFIG.xdma_wnum_chnl {1} \
    CONFIG.axilite_master_en {true} CONFIG.axilite_master_size {1} CONFIG.axilite_master_scale {Megabytes} \
    CONFIG.pf0_msix_enabled {false} CONFIG.xdma_num_usr_irq {1} \
    CONFIG.pf0_device_id {A710} CONFIG.pf0_subsystem_id {0001} CONFIG.pf0_class_code_base {04} CONFIG.pf0_class_code_sub {00}] $xdma
set pcie_mgt [create_bd_intf_port -mode Master -vlnv xilinx.com:interface:pcie_7x_mgt_rtl:1.0 pci_express_x16]
set pcie_refclk [create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 pcie_refclk]
set_property CONFIG.FREQ_HZ 100000000 $pcie_refclk
set pcie_perstn [create_bd_port -dir I -type rst pcie_perstn]
set_property CONFIG.POLARITY ACTIVE_LOW $pcie_perstn
set refbuf [create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.2 refbuf]
set_property -dict [list CONFIG.C_BUF_TYPE {IBUFDSGTE} CONFIG.DIFF_CLK_IN_BOARD_INTERFACE {pcie_refclk} CONFIG.USE_BOARD_FLOW {true}] $refbuf
connect_bd_intf_net $pcie_mgt [get_bd_intf_pins xdma/pcie_mgt]
connect_bd_intf_net $pcie_refclk [get_bd_intf_pins refbuf/CLK_IN_D]
connect_bd_net [get_bd_pins refbuf/IBUF_DS_ODIV2] [get_bd_pins xdma/sys_clk]
connect_bd_net [get_bd_pins refbuf/IBUF_OUT] [get_bd_pins xdma/sys_clk_gt]
connect_bd_net [get_bd_ports pcie_perstn] [get_bd_pins xdma/sys_rst_n]
set irq0 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 irq0]
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {1}] $irq0
connect_bd_net [get_bd_pins irq0/dout] [get_bd_pins xdma/usr_irq_req]

# DDR4: bank U200_DDR (1 = the bank whose pins and 300 MHz clock sit in the PCIe SLR), the board preset
# (MTA18ASF2G72PZ-2G3 RDIMM, 72-bit with ECC, DDR4-2400, 512-bit AXI at 300 MHz), plus a 100 MHz user clock
# (addn_ui_clkout1) that feeds the decoder's MMCM exactly as pl_clk0 does on the Kria.
set bank $::env(U200_DDR)
set ddr [create_bd_cell -type ip -vlnv xilinx.com:ip:ddr4:2.2 ddr]
set_property -dict [list CONFIG.C0_DDR4_BOARD_INTERFACE ddr4_sdram_c$bank CONFIG.C0_CLOCK_BOARD_INTERFACE default_300mhz_clk$bank] $ddr
set_property -dict [list CONFIG.ADDN_UI_CLKOUT1_FREQ_HZ {100} CONFIG.C0.DDR4_AxiSelection {true} CONFIG.C0.DDR4_AxiDataWidth {512} CONFIG.C0.DDR4_AxiAddressWidth {34}] $ddr
set ddr_if [create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddr4_rtl:1.0 ddr4_sdram_c$bank]
connect_bd_intf_net $ddr_if [get_bd_intf_pins ddr/C0_DDR4]
set sysclk [create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 default_300mhz_clk$bank]
set_property CONFIG.FREQ_HZ 300000000 $sysclk
connect_bd_intf_net $sysclk [get_bd_intf_pins ddr/C0_SYS_CLK]
# the controller's reset: PERST (through a reset block; sys_rst is active high, asynchronous)
set rst_pcie [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_pcie]
connect_bd_net [get_bd_pins xdma/axi_aclk] [get_bd_pins rst_pcie/slowest_sync_clk]
connect_bd_net [get_bd_ports pcie_perstn] [get_bd_pins rst_pcie/ext_reset_in]
connect_bd_net [get_bd_pins rst_pcie/peripheral_reset] [get_bd_pins ddr/sys_rst]
# the AXI side of the controller in its own clock domain (c0_ddr4_ui_clk), reset from its sync reset
set rst_ddr [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_ddr]
connect_bd_net [get_bd_pins ddr/c0_ddr4_ui_clk] [get_bd_pins rst_ddr/slowest_sync_clk]
connect_bd_net [get_bd_pins ddr/c0_ddr4_ui_clk_sync_rst] [get_bd_pins rst_ddr/ext_reset_in]
connect_bd_net [get_bd_pins rst_ddr/peripheral_aresetn] [get_bd_pins ddr/c0_ddr4_aresetn]

# The decoder clocks: the same hand-instantiated MMCM as fpga/kv260/build.tcl (clk and clk2x exactly 2:1 from one
# VCO; the clock wizard does not honour the ratio), from the DDR4 controller's 100 MHz user clock.
set m 0
for {set cand 16} {$cand >= 8} {incr cand -1} {
    set vco [expr {100.0 * $cand}]
    set q [expr {$vco / (2.0 * $mhz)}]
    if {$q == int($q) && $vco >= 800 && $vco <= 1550} { set m $cand; break }
}
# (VCO capped at 1550: the DDR4 controller's "100 MHz" is 300.12 / 3 = 100.04 MHz, DDR4-2400 at 3332 ps, and the
# -2 speed grade's VCO limit is 1600; a x16 MMCM failed the DRC at 1600.64 MHz on the first build, 2026-10-23)
if {$m == 0} { error "no integer MMCM dividers for $mhz MHz from a 100 MHz input" }
set d2 [expr {int(100 * $m / (2 * $mhz))}]
set d1 [expr {2 * $d2}]
set f1 [expr {100000000 * $m / $d1}]
set f2 [expr {100000000 * $m / $d2}]
puts "MMCM: VCO [expr {100 * $m}] MHz, clk = VCO/$d1 = $f1 Hz, clk2x = VCO/$d2 = $f2 Hz"
set fin [get_property CONFIG.FREQ_HZ [get_bd_pins ddr/addn_ui_clkout1]]
puts "MMCM input: addn_ui_clkout1 at $fin Hz"
set fh [open $out/work/clkgen_v.v w]
puts $fh "// Generated by fpga/u200/build.tcl: the decoder clocks from one MMCM, exactly 2:1 (clk = VCO/$d1, clk2x = VCO/$d2, VCO = 100 MHz x $m)."
puts $fh "module clkgen_v ("
puts $fh "    (* X_INTERFACE_INFO = \"xilinx.com:signal:clock:1.0 clk_in1 CLK\" *) (* X_INTERFACE_PARAMETER = \"FREQ_HZ $fin\" *) input  wire clk_in1,"
puts $fh "    (* X_INTERFACE_INFO = \"xilinx.com:signal:clock:1.0 clk_out1 CLK\" *) (* X_INTERFACE_PARAMETER = \"FREQ_HZ $f1\" *) output wire clk_out1,"
puts $fh "    (* X_INTERFACE_INFO = \"xilinx.com:signal:clock:1.0 clk_out2 CLK\" *) (* X_INTERFACE_PARAMETER = \"FREQ_HZ $f2\" *) output wire clk_out2,"
puts $fh "    output wire locked);"
puts $fh "    wire fb, fbbuf, o0, o1;"
puts $fh "    MMCME4_ADV #(.CLKFBOUT_MULT_F([format %.3f $m]), .DIVCLK_DIVIDE(1), .CLKIN1_PERIOD(10.000), .CLKOUT0_DIVIDE_F([format %.3f $d1]), .CLKOUT1_DIVIDE($d2),"
puts $fh "                 .CLKOUT0_PHASE(0.000), .CLKOUT1_PHASE(0.000), .CLKOUT0_DUTY_CYCLE(0.500), .CLKOUT1_DUTY_CYCLE(0.500), .COMPENSATION(\"AUTO\"), .STARTUP_WAIT(\"FALSE\"))"
puts $fh "        mmcm (.CLKIN1(clk_in1), .CLKIN2(1'b0), .CLKINSEL(1'b1), .CLKFBIN(fbbuf), .CLKFBOUT(fb), .CLKFBOUTB(), .CLKOUT0(o0), .CLKOUT0B(), .CLKOUT1(o1), .CLKOUT1B(),"
puts $fh "              .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(), .LOCKED(locked), .RST(1'b0), .PWRDWN(1'b0),"
puts $fh "              .DADDR(7'd0), .DCLK(1'b0), .DEN(1'b0), .DI(16'd0), .DWE(1'b0), .DO(), .DRDY(), .PSCLK(1'b0), .PSEN(1'b0), .PSINCDEC(1'b0), .PSDONE(),"
puts $fh "              .CDDCREQ(1'b0), .CDDCDONE(), .CLKINSTOPPED(), .CLKFBSTOPPED());"
puts $fh "    BUFG b_fb (.I(fb), .O(fbbuf));"
puts $fh "    BUFG b_0 (.I(o0), .O(clk_out1));"
puts $fh "    BUFG b_1 (.I(o1), .O(clk_out2));"
puts $fh "endmodule"
close $fh
add_files -norecurse $out/work/clkgen_v.v
set_property file_type Verilog [get_files $out/work/clkgen_v.v]
update_compile_order -fileset sources_1
set clkgen [create_bd_cell -type module -reference clkgen_v clkgen]
connect_bd_net [get_bd_pins ddr/addn_ui_clkout1] [get_bd_pins clkgen/clk_in1]

# The decoder: dec_host + phase generator in the clk domain, reset while the PCIe side is in reset or the MMCM
# is not locked (so a host-side reset of the card resets the decoder too)
set rst_dec [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_dec]
connect_bd_net [get_bd_pins clkgen/clk_out1] [get_bd_pins rst_dec/slowest_sync_clk]
connect_bd_net [get_bd_pins xdma/axi_aresetn] [get_bd_pins rst_dec/ext_reset_in]
connect_bd_net [get_bd_pins clkgen/locked] [get_bd_pins rst_dec/dcm_locked]
set host [create_bd_cell -type module -reference dec_host_v host]
set fbx $::env(U200_FBX); set fby $::env(U200_FBY); set win $::env(U200_WIN); set inter $::env(U200_INTER)
set_property -dict [list CONFIG.FBX $fbx CONFIG.FBY $fby CONFIG.ITX_LANES {2} CONFIG.FRAME_URAM {1} CONFIG.FRAME_WIN $win CONFIG.INTER $inter] $host
puts "decoder: FBX $fbx FBY $fby FRAME_WIN $win INTER $inter"
set pg [create_bd_cell -type module -reference phase_gen_v pg]
connect_bd_net [get_bd_pins clkgen/clk_out1] [get_bd_pins host/clk] [get_bd_pins pg/clk]
connect_bd_net [get_bd_pins clkgen/clk_out2] [get_bd_pins host/clk2x] [get_bd_pins pg/clk2x]
connect_bd_net [get_bd_pins rst_dec/peripheral_aresetn] [get_bd_pins host/aresetn]
connect_bd_net [get_bd_pins rst_dec/peripheral_reset] [get_bd_pins pg/rst_in]
connect_bd_net [get_bd_pins pg/phase] [get_bd_pins host/phase]

# JTAG-to-AXI master: the register file from the Vivado hardware manager (bring-up without the PCIe driver)
set jtag [create_bd_cell -type ip -vlnv xilinx.com:ip:jtag_axi:1.2 jtag]
set_property -dict [list CONFIG.PROTOCOL {2} CONFIG.M_AXI_ADDR_WIDTH {32}] $jtag
connect_bd_net [get_bd_pins xdma/axi_aclk] [get_bd_pins jtag/aclk]
connect_bd_net [get_bd_pins xdma/axi_aresetn] [get_bd_pins jtag/aresetn]

# Interconnect. Registers: XDMA's AXI-Lite master and the JTAG master (both at axi_aclk) -> dec_host (clk).
# Memory: XDMA's 512-bit master (axi_aclk) and the decoder's 64-bit master (clk) -> the DDR4 bank (ui_clk).
# The smartconnects do the width and clock conversions (every clock they see is listed on their aclk pins).
# (the ECC controller's AXI-Lite control port, which the IP insists is connected, is the second register target:
# ECC status / injection registers at 0x1_0000, PG150 "AXI4-Lite Slave Control/Status Register Interface")
set sc_regs [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 sc_regs]
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {2} CONFIG.NUM_CLKS {3}] $sc_regs
connect_bd_net [get_bd_pins xdma/axi_aclk] [get_bd_pins sc_regs/aclk]
connect_bd_net [get_bd_pins clkgen/clk_out1] [get_bd_pins sc_regs/aclk1]
connect_bd_net [get_bd_pins ddr/c0_ddr4_ui_clk] [get_bd_pins sc_regs/aclk2]
connect_bd_net [get_bd_pins xdma/axi_aresetn] [get_bd_pins sc_regs/aresetn]
connect_bd_intf_net [get_bd_intf_pins xdma/M_AXI_LITE] [get_bd_intf_pins sc_regs/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins jtag/M_AXI] [get_bd_intf_pins sc_regs/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_regs/M00_AXI] [get_bd_intf_pins host/s_axi]
connect_bd_intf_net [get_bd_intf_pins sc_regs/M01_AXI] [get_bd_intf_pins ddr/C0_DDR4_S_AXI_CTRL]
set sc_mem [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 sc_mem]
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {3}] $sc_mem
connect_bd_net [get_bd_pins xdma/axi_aclk] [get_bd_pins sc_mem/aclk]
connect_bd_net [get_bd_pins clkgen/clk_out1] [get_bd_pins sc_mem/aclk1]
connect_bd_net [get_bd_pins ddr/c0_ddr4_ui_clk] [get_bd_pins sc_mem/aclk2]
connect_bd_net [get_bd_pins xdma/axi_aresetn] [get_bd_pins sc_mem/aresetn]
connect_bd_intf_net [get_bd_intf_pins xdma/M_AXI] [get_bd_intf_pins sc_mem/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins host/m_axi] [get_bd_intf_pins sc_mem/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_mem/M00_AXI] [get_bd_intf_pins ddr/C0_DDR4_S_AXI]

# Address maps
assign_bd_address -offset 0x00000000 -range 4K -target_address_space [get_bd_addr_spaces xdma/M_AXI_LITE] [get_bd_addr_segs host/s_axi/reg0]
assign_bd_address -offset 0x00000000 -range 4K -target_address_space [get_bd_addr_spaces jtag/Data] [get_bd_addr_segs host/s_axi/reg0]
assign_bd_address -offset 0x00010000 -range 64K -target_address_space [get_bd_addr_spaces xdma/M_AXI_LITE] [get_bd_addr_segs ddr/C0_DDR4_MEMORY_MAP_CTRL/C0_REG]
assign_bd_address -offset 0x00010000 -range 64K -target_address_space [get_bd_addr_spaces jtag/Data] [get_bd_addr_segs ddr/C0_DDR4_MEMORY_MAP_CTRL/C0_REG]
assign_bd_address -offset 0x00000000 -range 16G -target_address_space [get_bd_addr_spaces xdma/M_AXI] [get_bd_addr_segs ddr/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK]
assign_bd_address -offset 0x00000000 -range 16G -target_address_space [get_bd_addr_spaces host/m_axi] [get_bd_addr_segs ddr/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK]
validate_bd_design
puts "clk = [get_property CONFIG.FREQ_HZ [get_bd_pins clkgen/clk_out1]] Hz, clk2x = [get_property CONFIG.FREQ_HZ [get_bd_pins clkgen/clk_out2]] Hz, ui_clk = [get_property CONFIG.FREQ_HZ [get_bd_pins ddr/c0_ddr4_ui_clk]] Hz, axi_aclk = [get_property CONFIG.FREQ_HZ [get_bd_pins xdma/axi_aclk]] Hz"
save_bd_design
set bd [get_files sys.bd]
generate_target all $bd
make_wrapper -files $bd -top
add_files -norecurse [file join $out proj u200.gen sources_1 bd sys hdl sys_wrapper.v]
set_property top sys_wrapper [current_fileset]
update_compile_order -fileset sources_1

# ---------------------------------------------------------------- runs
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
if {[info exists ::env(U200_STRATEGY)] && $::env(U200_STRATEGY) ne ""} {
    set_property strategy $::env(U200_STRATEGY) [get_runs impl_1]
    puts "impl strategy: $::env(U200_STRATEGY)"
}
# the implementation-time constraints are the KV260's (the cells have the same names in this block design)
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
# the board pins the IP constraints chose, for checking against the board file / UG1289
foreach p [get_ports -filter {NAME =~ *refclk* || NAME =~ *perstn* || NAME =~ *300mhz* || NAME =~ *ddr4*_act_n || NAME =~ *ddr4*_reset_n}] {
    puts "PIN $p [get_property PACKAGE_PIN $p] [get_property IOSTANDARD $p]"
}
puts "PCIE block: [get_property LOC [get_cells -hier -filter {REF_NAME =~ PCIE40E4* || REF_NAME =~ PCIE4CE4*}]]"
puts "BUILD_DONE"
exit
