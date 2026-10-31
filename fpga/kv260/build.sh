#!/usr/bin/env bash
# Build the KV260 bitstream of the core decoder (no CDEF / LR / SR, 512 x 512 UltraRAM frame memory, 2 transform
# lanes) behind the AXI-Lite host interface, and package it for the Kria firmware loader:
#   fpga/kv260/build.sh [tag]          (KV260_MHZ=50 by default: the decoder clock; the memory runs at 2x;
#                                       KV260_FBX / KV260_FBY = 9 and KV260_WIN = 0 by default: 512 x 512 whole-picture
#                                       memory; KV260_FBX=11 KV260_FBY=11 KV260_WIN=3 for the 1080p window build)
# Output in ${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/kv260-<tag>/: the Vivado project (proj/), impl_*.rpt, and
# fw/openav1/{openav1.bit.bin, openav1.dtbo, shell.json} for `xmutil loadapp openav1` (docs/kv260-bringup.md).
# Needs Vivado (with bootgen) on PATH and dtc for the overlay.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
tag=${1:-$(git -C "$root" rev-parse --short HEAD)}
out=${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/kv260-$tag
export KV260_ROOT=$root KV260_OUT=$out KV260_MHZ=${KV260_MHZ:-50}
mkdir -p "$out/stubs" "$out/work" "$out/fw/openav1"
# the core build: CDEF and LR replaced by null stubs (synth/vivado/run.sh -n), the simulation frame memory by the
# place-and-route stand-in (never elaborated with FRAME_URAM = 1, but it must not be a black box)
# (file list = synth/vivado/run.sh's, plus dec_host and phase_gen; the inter blocks are left out by INTER = 0)
files="cdf_map_pkg tx_tables_pkg blk_tables_pkg q_tables_pkg syn_pkg scan_rom cdf_store msac sym_seq sym_path coef_rd blk_ctx
       pal_syntax mv_mem mf_band seg_band mv_stack blk_syntax lr_syntax tile_syntax itx_pair itx_lvl itx_row itx2d ipred cfl qm_rom
       mc_tables_pkg mc_filt fb_fetch ref_fetch warp_setup frame_mem_w frame_mem_uram recon_top cdef_pkg lr_pkg mi_store frame_state_w fs_load rec_fifo tile_engines lf_top cdef_top lr_top sr_top dec_top
       dec_host phase_gen"
sources="$root/synth/vivado/frame_mem_pr.sv"
for f in $files; do
    case " cdef_top lr_top " in
        *" $f "*) python3 "$root/synth/vivado/stub.py" --null "$root/rtl/$f.sv" > "$out/stubs/$f.sv"; sources="$sources $out/stubs/$f.sv" ;;
        *) sources="$sources $root/rtl/$f.sv" ;;
    esac
done
sources="$sources $here/dec_host_v.v $here/phase_gen_v.v"       # Verilog shells: the module-reference flow wants a Verilog top
export KV260_SOURCES=$sources
# qm_rom loads rtl/qm_rom.hex by a path relative to a simulation build dir (../../rtl/): the runs execute in
# proj/kv260.runs/<run>/, so proj/rtl must be the rtl directory
mkdir -p "$out/proj"; ln -sfn "$root/rtl" "$out/proj/rtl"
echo "kv260 build $tag at $KV260_MHZ MHz -> $out"
cd "$out/work"
vivado -mode batch -nojournal -log "$out/vivado.log" -source "$here/build.tcl"
grep -q BUILD_DONE "$out/vivado.log" || { echo "build failed, see $out/vivado.log" >&2; exit 1; }
# package: bit -> bit.bin (bootgen), dts -> dtbo, shell.json
bit=$out/proj/kv260.runs/impl_1/sys_wrapper.bit
cd "$out/fw"
cat > openav1.bif <<EOF
all: { [destination_device = pl] $bit }
EOF
bootgen -arch zynqmp -image openav1.bif -process_bitstream bin -w > bootgen.log 2>&1
cp "$bit.bin" openav1/openav1.bit.bin
cp "$here/shell.json" openav1/
if command -v dtc >/dev/null; then dtc -@ -O dtb -o openav1/openav1.dtbo "$here/openav1.dts"; else echo "no dtc: compile fpga/kv260/openav1.dts on the board" >&2; fi
ls -la openav1
grep -A2 "WNS" "$out/impl_timing.rpt" | head -8 || true
echo "fw dir: $out/fw/openav1 (copy to /lib/firmware/xilinx/openav1/ on the KV260)"
