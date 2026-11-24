# where.tcl: where the DDR4 controller, XDMA and the decoder sit in a routed checkpoint (clock-region histograms), the
# failing setup paths with their regions, and the pblocks. Read it after the first image that misses timing inside
# vendor IP (2026-11-23, K60-85): DCP=<routed.dcp> vivado -mode batch -nojournal -log where.log -source fpga/u200/where.tcl
open_checkpoint $::env(DCP)
proc hist {name cells} {
    array set h {}
    foreach c $cells {
        set s [get_sites -quiet -of_objects $c]
        if {$s eq ""} continue
        set r [get_property CLOCK_REGION [lindex $s 0]]
        if {![info exists h($r)]} { set h($r) 0 }
        incr h($r)
    }
    set out {}
    foreach r [lsort [array names h]] { lappend out "$r:$h($r)" }
    puts "WHERE $name ([llength $cells] cells): [join $out { }]"
}
set ddr  [get_cells -hier -filter {PRIMITIVE_LEVEL == LEAF && NAME =~ sys_i/ddr/*}]
set xdma [get_cells -hier -filter {PRIMITIVE_LEVEL == LEAF && NAME =~ sys_i/xdma/*}]
set host [get_cells -hier -filter {PRIMITIVE_LEVEL == LEAF && NAME =~ sys_i/host/*}]
hist ddr $ddr
hist xdma $xdma
hist host $host
set others [get_cells -hier -filter {PRIMITIVE_LEVEL == LEAF && NAME !~ sys_i/ddr/* && NAME !~ sys_i/xdma/* && NAME !~ sys_i/host/*}]
hist others $others
foreach p [get_timing_paths -setup -max_paths 40 -nworst 1 -slack_lesser_than 0] {
    set sp [get_property STARTPOINT_PIN $p]; set ep [get_property ENDPOINT_PIN $p]
    set sc [get_cells -of_objects $sp]; set ec [get_cells -of_objects $ep]
    puts "FAIL [get_property SLACK $p] [get_property CLOCK_REGION [get_sites -of_objects $sc]] -> [get_property CLOCK_REGION [get_sites -of_objects $ec]] lvls [get_property LOGIC_LEVELS $p] [string range $sp 0 90] -> [string range $ep 0 90]"
}
puts "PBLOCKS [get_pblocks]"
foreach pb [get_pblocks] { puts "PBLOCK $pb [get_property GRID_RANGES $pb] exclude [get_property EXCLUDE_PLACEMENT $pb] contain_routing [get_property CONTAIN_ROUTING $pb]" }
puts "WHERE_DONE"
exit
