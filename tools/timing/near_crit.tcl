# near-critical census of a routed image: every endpoint of the decoder clock with setup slack under 0.30 ns
# usage: vivado -mode batch -nojournal -nolog -source near_crit.tcl -tclargs <routed.dcp> <out.txt>
set dcp [lindex $argv 0]
set out [lindex $argv 1]
open_checkpoint $dcp
set fh [open $out w]
foreach g {o0} {
    set paths [get_timing_paths -group $g -setup -max_paths 20000 -nworst 1 -slack_lesser_than 0.30]
    puts $fh "# group $g: [llength $paths] endpoints under 0.30 ns"
    foreach p $paths {
        puts $fh [format "%.3f %s %s %s %.3f %.3f" [get_property SLACK $p] [get_property LOGIC_LEVELS $p] \
            [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p] \
            [get_property DATAPATH_LOGIC_DELAY $p] [get_property DATAPATH_NET_DELAY $p]]
    }
}
close $fh
puts "NEAR_CRIT_DONE"
exit
