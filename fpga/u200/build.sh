#!/usr/bin/env bash
# Build the Alveo U200 bitstream of the decoder behind the AXI-Lite host interface, with the host side over PCIe
# (XDMA) instead of the Kria's PS: same dec_host, same register map, same host protocol (tb/host_regs.py).
#   fpga/u200/build.sh [tag]
# Environment (defaults = the 1080p window build that runs on the KV260, at 100 MHz):
#   U200_MHZ=100          decoder clock (the frame memory runs at 2x); the MMCM dividers are computed from the
#                         DDR4 controller's 100 MHz user clock, so 50 / 80 / 100 / 125 / 160 / 200 work directly
#   U200_FBX=11 U200_FBY=11 U200_WIN=3   frame memory geometry (window of 3 superblock rows at full 1080p width;
#                         U200_WIN=0 with FBX=FBY=9 is the 512 x 512 whole-picture memory)
#   U200_SB128=0          1: the window takes 128x128 superblocks too (128-line window rows: twice the frame-memory
#                         UltraRAM; libaom's default superblock size from 720p up)
#   U200_TE=1             tile engines: the tile columns of a tile row decoded at once (rtl/tile_engines.sv)
#   U200_INTER=0          1: the inter decoder (ref_fetch / mc_filt / warp_setup); needs a frame memory that is not
#                         the window (docs/frame-memory.md), so 0 until that exists
#   U200_CORE=1           1: CDEF and LR as null stubs (the core build, as on the KV260); 0: the real blocks, with
#                         fb1 / fb2 as UltraRAM rings of U200_WIN1 / U200_WIN2 superblock rows (default 3 / 3 when
#                         U200_CORE=0, docs/picture-memory.md step A; -1 leaves them out)
#   U200_DDR=1            which of the card's four DDR4 banks holds the pictures (1 = the bank in the PCIe SLR)
#   U200_BOARD_REPO=~/vivado-dl/boards   directory holding au200/<ver>/ (AMD's au200_board_files zip, unpacked)
#   U200_STRATEGY=        an implementation strategy for a timing-tight build (Performance_ExtraTimingOpt closed the
#                         decoder at 100 MHz on 2026-10-23; 80 MHz closes under the default strategy)
#   U200_PBLOCK=0         1: keep XDMA in the PCIe clock regions (fpga/u200/pcie_pblock.xdc), for the 100 MHz builds
#                         whose only misses were the PCIe core's 500 MHz paths into the hard block
# Output in ${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/u200-<tag>/: the Vivado project (proj/), impl_*.rpt, and
# sys_wrapper.bit for JTAG programming (fpga/u200/program.sh) or sys_wrapper.mcs for the configuration flash.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
tag=${1:-$(git -C "$root" rev-parse --short HEAD)}
out=${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/u200-$tag
export U200_ROOT=$root U200_OUT=$out U200_MHZ=${U200_MHZ:-100}
export U200_FBX=${U200_FBX:-11} U200_FBY=${U200_FBY:-11} U200_WIN=${U200_WIN:-3} U200_INTER=${U200_INTER:-0}
export U200_CORE=${U200_CORE:-1} U200_DDR=${U200_DDR:-1} U200_BOARD_REPO=${U200_BOARD_REPO:-$HOME/vivado-dl/boards} U200_SB128=${U200_SB128:-0} U200_TE=${U200_TE:-1}
if [ "$U200_CORE" = 1 ]; then export U200_WIN1=${U200_WIN1:--1} U200_WIN2=${U200_WIN2:--1}; else export U200_WIN1=${U200_WIN1:-3} U200_WIN2=${U200_WIN2:-3}; fi
[ -d "$U200_BOARD_REPO/au200" ] || { echo "no board files at $U200_BOARD_REPO/au200 (unpack au200_board_files_*.zip there)" >&2; exit 1; }
mkdir -p "$out/stubs" "$out/work"
# the rtl list = synth/vivado/run.sh's plus dec_host and phase_gen (keep in step with fpga/kv260/build.sh)
files="cdf_map_pkg tx_tables_pkg blk_tables_pkg q_tables_pkg syn_pkg scan_rom cdf_store msac sym_seq sym_path coef_rd blk_ctx
       pal_syntax mv_mem mf_band seg_band mv_stack blk_syntax lr_syntax tile_syntax itx_pair itx_lvl itx_row itx2d ipred cfl qm_rom
       mc_tables_pkg mc_filt ref_fetch warp_setup frame_mem_w frame_mem_uram recon_top cdef_pkg lr_pkg mi_store frame_state_w rec_fifo tile_engines lf_top cdef_top lr_top sr_top dec_top
       dec_host phase_gen"
sources="$root/synth/vivado/frame_mem_pr.sv"
for f in $files; do
    if [ "$U200_CORE" = 1 ] && case " cdef_top lr_top " in *" $f "*) true ;; *) false ;; esac; then
        python3 "$root/synth/vivado/stub.py" --null "$root/rtl/$f.sv" > "$out/stubs/$f.sv"; sources="$sources $out/stubs/$f.sv"
    else
        sources="$sources $root/rtl/$f.sv"
    fi
done
# the Verilog shells of the KV260 flow (the module-reference flow wants a Verilog top): shared, not copied
sources="$sources $root/fpga/kv260/dec_host_v.v $root/fpga/kv260/phase_gen_v.v"
export U200_SOURCES=$sources
# qm_rom loads rtl/qm_rom.hex by a path relative to a simulation build dir (../../rtl/): the runs execute in
# proj/u200.runs/<run>/, so proj/rtl must be the rtl directory
mkdir -p "$out/proj"; ln -sfn "$root/rtl" "$out/proj/rtl"
echo "u200 build $tag at $U200_MHZ MHz (FBX $U200_FBX FBY $U200_FBY WIN $U200_WIN/$U200_WIN1/$U200_WIN2 INTER $U200_INTER SB128 $U200_SB128 TE $U200_TE CORE $U200_CORE DDR bank $U200_DDR${U200_STRATEGY:+ strategy $U200_STRATEGY}${U200_PBLOCK:+ pblock $U200_PBLOCK}) -> $out"
cd "$out/work"
vivado -mode batch -nojournal -log "$out/vivado.log" -source "$here/build.tcl"
grep -q BUILD_DONE "$out/vivado.log" || { echo "build failed, see $out/vivado.log" >&2; exit 1; }
cp "$out/proj/u200.runs/impl_1/sys_wrapper.bit" "$out/"
grep -A2 "WNS" "$out/impl_timing.rpt" | head -8 || true
echo "bitstream: $out/sys_wrapper.bit (program over JTAG with fpga/u200/program.sh, then rescan PCIe or reboot the host)"
