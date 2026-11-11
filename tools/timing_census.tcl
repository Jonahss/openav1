# timing_census.tcl (2026-11-10): every failing setup path of a routed checkpoint, one CSV line per endpoint.
# vivado -mode batch -source timing_census.tcl -tclargs <routed.dcp> <out.csv> [slack_below_ns, default 0] [max paths, default 30000]
set dcp [lindex $argv 0]; set out [lindex $argv 1]
set lim [expr {[llength $argv] > 2 ? [lindex $argv 2] : 0}]
set maxp [expr {[llength $argv] > 3 ? [lindex $argv 3] : 30000}]
open_checkpoint $dcp
set paths [get_timing_paths -setup -max_paths $maxp -nworst 1 -slack_lesser_than $lim]
set f [open $out w]
puts $f "slack,levels,datapath,logic,route,startclk,endclk,start,end"
foreach p $paths {
    puts $f [join [list [get_property SLACK $p] [get_property LOGIC_LEVELS $p] [get_property DATAPATH_DELAY $p] \
        [get_property DATAPATH_LOGIC_DELAY $p] [get_property DATAPATH_NET_DELAY $p] \
        [get_property STARTPOINT_CLOCK $p] [get_property ENDPOINT_CLOCK $p] \
        [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p]] ","]
}
close $f
puts "TIMING_CENSUS [llength $paths] paths below $lim ns -> $out"
