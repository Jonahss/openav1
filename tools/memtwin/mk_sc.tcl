# mk_sc.tcl <outdir> <name> <sys_sc_mem_0_stub.v> [<interface>.<PARAM>=<value> ...] (K60-110): the image's memory
# interconnect made again as a block design of its own, for the twin: a smartconnect with the same three slave ports,
# one master port and three clocks, every interface carrying the parameters the image build's stub records
# (X_INTERFACE_PARAMETER), except those overridden on the command line, e.g. S02_AXI.NUM_READ_OUTSTANDING=32.
# Output: <outdir>/<name>/ with the generated simulation sources and <outdir>/<name>/files.f (what xsim must compile);
# the top module is <name>_wrapper with the stub's port names.
set out [file normalize [lindex $argv 0]]; set name [lindex $argv 1]; set stub [lindex $argv 2]
array set over {}
foreach o [lrange $argv 3 end] { regexp {^([^.]+)\.([^=]+)=(.*)$} $o -> i p v; set over($i,$p) $v; puts "OVERRIDE $i $p = $v" }
set fh [open $stub]; set txt [read $fh]; close $fh
# the parameter string of each AXI interface and each clock (the first port of an interface carries it)
array set par {}
foreach {m i s} [regexp -all -inline {XIL_INTERFACENAME ([A-Za-z0-9_.]+), ([^"]*)"} $txt] { if {![info exists par($i)]} { set par($i) $s } }
create_project -force $name $out/$name/proj -part xcu200-fsgd2104-2-e
create_bd_design $name
set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 sc]
set_property -dict [list CONFIG.NUM_SI {3} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {3}] $sc
proc clkport {n pin freq} {
    set p [create_bd_port -dir I -type clk -freq_hz $freq $n]
    connect_bd_net $p [get_bd_pins sc/$pin]
    return $p
}
set c0 [clkport aclk aclk 250000000]; set c1 [clkport aclk1 aclk1 80000000]; set c2 [clkport aclk2 aclk2 300000000]
set r [create_bd_port -dir I -type rst aresetn]; set_property CONFIG.POLARITY ACTIVE_LOW $r
connect_bd_net $r [get_bd_pins sc/aresetn]
set keep {DATA_WIDTH PROTOCOL ID_WIDTH ADDR_WIDTH READ_WRITE_MODE HAS_BURST HAS_LOCK HAS_PROT HAS_CACHE HAS_QOS HAS_REGION HAS_WSTRB HAS_BRESP HAS_RRESP SUPPORTS_NARROW_BURST NUM_READ_OUTSTANDING NUM_WRITE_OUTSTANDING MAX_BURST_LENGTH NUM_READ_THREADS NUM_WRITE_THREADS}
foreach {i mode clk} {S00_AXI Slave aclk S01_AXI Slave aclk1 S02_AXI Slave aclk1 M00_AXI Master aclk2} {
    set p [create_bd_intf_port -mode $mode -vlnv xilinx.com:interface:aximm_rtl:1.0 $i]
    set d [list]
    foreach kv [split $par($i) ,] {
        set kv [string trim $kv]; set k [lindex $kv 0]; set v [lindex $kv 1]
        if {[lsearch $keep $k] < 0} continue
        if {[info exists over($i,$k)]} { set v $over($i,$k) }
        lappend d CONFIG.$k $v
    }
    set_property -dict $d $p
    puts "INTERFACE $i: $d"
    connect_bd_intf_net $p [get_bd_intf_pins sc/$i]
}
set_property CONFIG.ASSOCIATED_BUSIF {S00_AXI} $c0
set_property CONFIG.ASSOCIATED_BUSIF {S01_AXI:S02_AXI} $c1
set_property CONFIG.ASSOCIATED_BUSIF {M00_AXI} $c2
assign_bd_address
foreach s [get_bd_addr_segs -of_objects [get_bd_addr_spaces]] { puts "SEG $s [get_property OFFSET $s] [get_property RANGE $s]" }
foreach s [get_bd_addr_segs -quiet */SEG_M00_AXI_Reg] { set_property OFFSET 0x0 $s; set_property RANGE 16G $s }
validate_bd_design
save_bd_design
set bd [get_files $name.bd]
generate_target simulation $bd
make_wrapper -files $bd -top
add_files -norecurse [glob $out/$name/proj/$name.gen/sources_1/bd/$name/hdl/${name}_wrapper.v]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
set_property top ${name}_wrapper [get_filesets sim_1]
# the list of simulation sources, in compile order
set f [open $out/$name/files.f w]
foreach x [get_files -compile_order sources -used_in simulation] { puts $f $x }
close $f
puts "MK_SC_DONE [llength [get_files -compile_order sources -used_in simulation]] files"
