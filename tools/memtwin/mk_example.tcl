# mk_example.tcl <outdir> <sys_ddr_0.xci> <board repo> (K60-109): the DDR4 controller of a U200 image build, taken as configured
# there (its .xci), with the example design Vivado makes for it: controller + Xilinx ddr4_model + an AXI traffic
# generator, for xsim
set out [file normalize [lindex $argv 0]]; set xci [lindex $argv 1]
set_param board.repoPaths [list [lindex $argv 2]]
create_project -force twin $out/proj -part xcu200-fsgd2104-2-e
set bp [lindex [lsort -dictionary [get_board_parts xilinx.com:au200:part0:*]] end]
set_property board_part $bp [current_project]
import_ip $xci
set ddr [get_ips sys_ddr_0]
foreach p {C0.DDR4_MemoryPart C0.DDR4_TimePeriod C0.DDR4_DataWidth C0.DDR4_Ecc C0.DDR4_CasLatency C0.DDR4_CasWriteLatency C0.DDR4_Mem_Add_Map C0.DDR4_AxiIDWidth C0.DDR4_Ordering C0.DDR4_AxiArbitrationScheme C0.DDR4_AxiDataWidth C0_DDR4_BOARD_INTERFACE} {
  puts "CFG $p = [get_property CONFIG.$p $ddr]" }
generate_target all $ddr
open_example_project -force -dir $out/ex $ddr
puts "EXAMPLE_DONE"
