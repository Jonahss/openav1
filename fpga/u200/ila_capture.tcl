# ILA capture on the U200 (debug builds of fpga/u200/build.sh with U200_ILA=1). Driven by fpga/u200/ila_capture.sh:
#   ILA_LTX      probes file (sys_wrapper.ltx next to the bitstream)
#   ILA_CSV      where the capture goes (write_hw_ila_data -csv_file)
#   ILA_URL      hw_server (default localhost:3121)
#   ILA_TRIG     glob of the trigger probe (default *start*: dec_host's start pulses) and ILA_TRIG_VALUE its compare value
#                (default eq6'bxxxx1x = frame_start); ILA_WINDOWS windows, each armed on the trigger (default 2: the
#                second window starts with the second frame_start of the run, i.e. frame 1)
#   ILA_CAPTURE  space-separated probe globs OR-ed as the storage qualifier (default: coefficient writes, record pops,
#                residual adds, transform writes and start pulses), empty = store every cycle
#   ILA_TIMEOUT  seconds to wait for the trigger (default 600)
# Prints PROBES (the probe names), ARMED when run_hw_ila returned (start the decode then), CAPTURED <csv> at the end.
set url [expr {[info exists ::env(ILA_URL)] ? $::env(ILA_URL) : "localhost:3121"}]
set ltx $::env(ILA_LTX)
set csv $::env(ILA_CSV)
set trig [expr {[info exists ::env(ILA_TRIG)] ? $::env(ILA_TRIG) : "*start*"}]
set trigv [expr {[info exists ::env(ILA_TRIG_VALUE)] ? $::env(ILA_TRIG_VALUE) : "eq6'bxxxx1x"}]
set nwin [expr {[info exists ::env(ILA_WINDOWS)] ? $::env(ILA_WINDOWS) : 2}]
set caps [expr {[info exists ::env(ILA_CAPTURE)] ? $::env(ILA_CAPTURE) : "*nz_we* *a_valid* *itx_we* *ev_pop* *start*"}]
set tmo [expr {[info exists ::env(ILA_TIMEOUT)] ? $::env(ILA_TIMEOUT) : 600}]
open_hw_manager
connect_hw_server -url $url
after 1000
open_hw_target
set dev [lindex [get_hw_devices xcu200*] 0]
if {$dev eq ""} { puts "no xcu200 on the JTAG chain: [get_hw_devices]"; exit 1 }
current_hw_device $dev
set_property PROBES.FILE $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev
set ila [lindex [get_hw_ilas -of_objects $dev] 0]
if {$ila eq ""} { puts "no ILA in the programmed design"; exit 1 }
set probes [get_hw_probes -of_objects $ila]
puts "PROBES [llength $probes]"
proc pw {p} { return [get_property WIDTH $p] }
foreach p $probes { puts "  $p \[[pw $p]\]" }
set depth [get_property STATIC.MAX_DATA_DEPTH $ila]
set_property CONTROL.WINDOW_COUNT $nwin $ila
set wdepth [expr {[info exists ::env(ILA_WINDOW_DEPTH)] ? $::env(ILA_WINDOW_DEPTH) : $depth / $nwin}]
set_property CONTROL.DATA_DEPTH $wdepth $ila
set_property CONTROL.TRIGGER_POSITION 0 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
# ILA_TRIG: either one glob (compared with ILA_TRIG_VALUE) or space-separated glob=value pairs AND-ed together
set tdesc {}
if {[string first "=" $trig] >= 0} {
    foreach pair $trig {
        lassign [split $pair "="] g v
        set tp [get_hw_probes $g -of_objects $ila]
        if {[llength $tp] != 1} { puts "trigger glob $g matched [llength $tp] probes: $tp"; exit 1 }
        set_property TRIGGER_COMPARE_VALUE $v $tp
        lappend tdesc "$tp=$v"
    }
} else {
    set tp [get_hw_probes $trig -of_objects $ila]
    if {[llength $tp] != 1} { puts "trigger glob $trig matched [llength $tp] probes: $tp"; exit 1 }
    set_property TRIGGER_COMPARE_VALUE $trigv $tp
    lappend tdesc "$tp=$trigv"
}
puts "TRIGGER $tdesc, $nwin window(s) of $wdepth"
if {$caps ne ""} {
    set_property CONTROL.CAPTURE_MODE BASIC $ila
    set_property CONTROL.CAPTURE_CONDITION OR $ila
    foreach g $caps {
        foreach p [get_hw_probes $g -of_objects $ila] {
            set w [pw $p]
            if {$w == 1} { set_property CAPTURE_COMPARE_VALUE eq1'b1 $p } else { set_property CAPTURE_COMPARE_VALUE "neq${w}'b[string repeat 0 $w]" $p }
            puts "CAPTURE $p"
        }
    }
} else {
    set_property CONTROL.CAPTURE_MODE ALWAYS $ila
}
run_hw_ila $ila
puts "ARMED"
flush stdout
catch { report_property -regexp $ila {STATUS.*} }
set t0 [clock seconds]; set last ""
while {1} {
    after 1000
    set st ""
    catch { set st "[get_property STATUS.CORE_STATUS $ila] samples=[get_property STATUS.SAMPLE_COUNT $ila]" }
    if {$st ne $last} { puts "STATUS $st"; flush stdout; set last $st }
    if {[string match "*POST_TRIGGER*" $st] || [string match "*FULL*" $st] || [string match "*IDLE*" $st]} { break }
    if {[clock seconds] - $t0 > $tmo} { puts "TIMEOUT waiting for the trigger ($st)"; exit 2 }
}
upload_hw_ila_data $ila
write_hw_ila_data -force -csv_file $csv [current_hw_ila_data]
puts "CAPTURED $csv"
exit
