#!/usr/bin/env bash
# Program the Alveo U200 over its JTAG (the micro-USB on the bracket) from the host it sits in, then make the host
# see the new PCIe function without a reboot:
#   fpga/u200/program.sh <sys_wrapper.bit> [hw_server url, default localhost:3121]
# Needs Vivado (or Vivado Lab) on PATH and a running hw_server (`hw_server -s tcp::3121 &`). After programming,
# the PCIe device that was enumerated at boot (if any) is removed and the bus rescanned (root); if the card was not
# enumerated at boot (slot empty of config at power-up: no bitstream in flash), a rescan alone finds it only if the
# root port trained the link, which needs the bitstream present at power-up or a warm reboot: `fpga/u200/program.sh`
# then `sudo reboot` is the dependable day-one path (UG1289: "the host must be rebooted after programming").
set -euo pipefail
bit=${1:?bitstream}; url=${2:-localhost:3121}
tcl=$(mktemp)
cat > "$tcl" <<TCL
open_hw_manager
connect_hw_server -url $url
after 2000
refresh_hw_server
open_hw_target
set dev [lindex [get_hw_devices xcu200*] 0]
if {\$dev eq ""} { puts "no xcu200 on the JTAG chain: [get_hw_devices]"; exit 1 }
current_hw_device \$dev
set_property PROGRAM.FILE {$bit} \$dev
program_hw_devices \$dev
puts "PROGRAMMED [get_property PROGRAM.FILE \$dev]"
exit
TCL
vivado -mode batch -nojournal -nolog -source "$tcl" | grep -E "PROGRAMMED|ERROR|no xcu200" || true
rm -f "$tcl"
# PCIe: drop the stale function (vendor 10ee = Xilinx) and rescan
for d in /sys/bus/pci/devices/*; do
    if [ "$(cat "$d/vendor" 2>/dev/null)" = "0x10ee" ]; then
        echo "removing $(basename "$d") ($(cat "$d/device"))"; echo 1 | sudo tee "$d/remove" >/dev/null
    fi
done
sleep 1
echo 1 | sudo tee /sys/bus/pci/rescan >/dev/null
sleep 1
lspci -d 10ee: -vv 2>/dev/null | grep -E "^[0-9a-f]|LnkSta:|Region 0" || echo "no Xilinx PCIe function visible (reboot the host with the card programmed)"
