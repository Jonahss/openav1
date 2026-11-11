# dump_place.tcl (2026-10-30, remake of the 10-19 script): vivado -mode batch -source dump_place.tcl -tclargs <routed.dcp> <outdir>
# Writes <outdir>/cells.csv (cell name, site, ref name, tile) for every placed leaf cell and <outdir>/sites.csv
# (site, type, RPM_X, RPM_Y, clock region, SLR) for every site of the device, for render_place.py on the Pi.
set dcp [lindex $argv 0]; set out [lindex $argv 1]; file mkdir $out
open_checkpoint $dcp
set f [open $out/cells.csv w]
foreach c [get_cells -hierarchical -filter {IS_PRIMITIVE && LOC != ""}] {
  puts $f "[get_property NAME $c]\t[get_property LOC $c]\t[get_property REF_NAME $c]"
}
close $f
set f [open $out/sites.csv w]
foreach s [get_sites -filter {IS_USED}] {
  set t [get_tiles -of $s]
  puts $f "[get_property NAME $s]\t[get_property SITE_TYPE $s]\t[get_property RPM_X $s]\t[get_property RPM_Y $s]\t[get_property CLOCK_REGION $s]\t[get_property SLR_INDEX [get_slrs -of $s]]"
}
close $f
set f [open $out/device.csv w]
foreach s [get_sites -filter {SITE_TYPE =~ "SLICE*" || SITE_TYPE =~ "RAMB36*" || SITE_TYPE =~ "URAM*" || SITE_TYPE =~ "DSP48*"}] {
  puts $f "[get_property NAME $s]\t[get_property SITE_TYPE $s]\t[get_property RPM_X $s]\t[get_property RPM_Y $s]\t[get_property CLOCK_REGION $s]\t[get_property SLR_INDEX [get_slrs -of $s]]"
}
close $f
puts "DUMP_DONE [llength [get_cells -hierarchical -filter {IS_PRIMITIVE && LOC != \"\"}]] cells"
