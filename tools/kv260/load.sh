#!/usr/bin/env bash
# On the KV260: install a firmware directory (bit.bin + dtbo + shell.json from fpga/kv260/build.sh) under
# /lib/firmware/xilinx/<name> and load it with xmutil; then show what the FPGA manager and the clock tree say.
#   sudo tools/kv260/load.sh <fw dir> [name]        (name defaults to the directory's basename, e.g. openav1)
set -euo pipefail
src=${1:?firmware dir}
name=${2:-$(basename "$src")}
dst=/lib/firmware/xilinx/$name
mkdir -p "$dst"
cp "$src"/*.bit.bin "$src"/*.dtbo "$src"/shell.json "$dst"/
xmutil unloadapp >/dev/null 2>&1 || true
xmutil loadapp "$name"
sleep 1
echo "fpga state: $(cat /sys/class/fpga_manager/fpga0/state)"
dmesg | grep -iE "fpga|clocking|fclk" | tail -4
echo "pl clocks:"; grep -E "fclk|pl0" /sys/kernel/debug/clk/clk_summary 2>/dev/null | head -3 || true
