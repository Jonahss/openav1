#!/usr/bin/env bash
# Build the Alveo U200 bitstream of the decoder behind the AXI-Lite host interface, with the host side over PCIe
# (XDMA): dec_host, its register map and host protocol (tb/host_regs.py).
#   fpga/u200/build.sh <tag> [--size 1080p|4k] [--te N] [--mhz N] [--ring N] [--strategy NAME] [--threads N] [--board-repo DIR]
#   fpga/u200/build.sh --print [options]        the configuration line and nothing else
# There is one decoder configuration but for its picture size: the frame memory a window of 5 superblock rows, the
# final picture a ring of 3, the inter decoder, 64x64 and 128x128 superblocks, XDMA kept in the PCIe clock regions
# (pcie_pblock.xdc), pictures in DDR4 bank 1. Those are written in build.tcl. What a build may choose is here, and
# only on the command line (nothing is read from the caller's environment: on 2026-11-17 two images were built with
# a setting left in a shell):
#   --size 1080p    the largest picture: 1080p (frame buffers 2048 x 2048: FBX = FBY = 11) or 4k (4096 x 4096: 12)
#   --te 2          tile engines: the tile columns of a tile row decoded at once (rtl/tile_engines.sv)
#   --mhz 100       decoder clock (the frame memory runs at 2x); the MMCM dividers are computed from the DDR4
#                   controller's 100 MHz user clock, so 50 / 80 / 100 / 125 / 160 / 200 work directly
#   --ring 16       ref_fetch line buffers (reference rows in flight)
#   --strategy Performance_ExtraTimingOpt   implementation strategy ("default" = Vivado's own)
#   --threads 8     general.maxThreads (16 helps synthesis; place/route stay at 8)
#   --board-repo ~/vivado-dl/boards         directory holding au200/<ver>/ (AMD's au200_board_files zip, unpacked)
#   --bd-only       stop after the block design is made and validated (minutes): the smoke of a change to the shell's
#                   wiring or address map before an image is launched
# Output in ~/openav1-synth/u200-<tag>/: the Vivado project (proj/), impl_*.rpt, and sys_wrapper.bit for JTAG
# programming (fpga/u200/program.sh).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
size=1080p; te=2; mhz=100; ring=16; strategy=Performance_ExtraTimingOpt; threads=8; board_repo=$HOME/vivado-dl/boards; tag=; print=0; bdonly=0
while [ $# -gt 0 ]; do
  case "$1" in
    --size) size=$2; shift 2;; --te) te=$2; shift 2;; --mhz) mhz=$2; shift 2;; --ring) ring=$2; shift 2;; --strategy) strategy=$2; shift 2;;
    --threads) threads=$2; shift 2;; --board-repo) board_repo=$2; shift 2;; --print) print=1; shift;; --bd-only) bdonly=1; shift;;
    -*) echo "build.sh: unknown option $1" >&2; exit 2;;
    *) [ -z "$tag" ] || { echo "build.sh: one tag, got '$tag' and '$1'" >&2; exit 2; }; tag=$1; shift;;
  esac
done
case "$size" in 1080p) fb=11;; 4k) fb=12;; *) echo "build.sh: --size is 1080p or 4k, not $size" >&2; exit 2;; esac
# a strategy Vivado does not know fails two minutes into the build (2026-11-25: "Explore" for Performance_Explore, both images of a pair)
case "$strategy" in default|Performance_*|Area_*|Congestion_*|Flow_*|Power_*) ;; *) echo "build.sh: unknown strategy $strategy (Performance_Explore, Performance_ExtraTimingOpt, ..., or default)" >&2; exit 2;; esac
config="$size TE=$te RING=$ring $mhz MHz $strategy"
if [ $print = 1 ]; then echo "$config"; exit 0; fi
[ -n "$tag" ] || { echo "usage: build.sh <tag> [--size 1080p|4k] [--te N] [--mhz N] [--ring N] [--strategy NAME] [--threads N] [--board-repo DIR] [--bd-only] | --print [options]" >&2; exit 2; }
out=$HOME/openav1-synth/u200-$tag
# build.tcl's inputs, every one set here
unset U200_TWIN U200_BD_ONLY
[ $bdonly = 1 ] && export U200_BD_ONLY=1
export U200_ROOT=$root U200_OUT=$out U200_MHZ=$mhz U200_FB=$fb U200_TE=$te U200_REF_RING=$ring U200_THREADS=$threads U200_BOARD_REPO=$board_repo
[ "$strategy" = default ] && export U200_STRATEGY= || export U200_STRATEGY=$strategy
[ -d "$U200_BOARD_REPO/au200" ] || { echo "no board files at $U200_BOARD_REPO/au200 (unpack au200_board_files_*.zip there)" >&2; exit 1; }
mkdir -p "$out/work"
# the rtl list = synth/vivado/run.sh's plus dec_host and phase_gen
files="cdf_map_pkg tx_tables_pkg blk_tables_pkg q_tables_pkg syn_pkg scan_rom cdf_store msac sym_seq sym_path coef_rd blk_ctx
       pal_syntax mv_mem mf_band seg_band mv_stack blk_syntax lr_syntax tile_syntax itx_pair itx_lvl itx_row itx2d ipred cfl qm_rom
       mc_tables_pkg mc_filt fb_fetch ref_fetch rf_axi_mux mc_lane warp_setup frame_mem_w edge_line frame_mem_uram frame_mem_uram_pc recon_top cdef_pkg lr_pkg mi_store frame_state_w fs_load mf_project rec_fifo tile_engines lf_cbuf lf_filt lf_edge lf_feed lf_stream cdef_row cdef_dir cdef_str cdef_stream lr_wiener_pkg lr_wiener_h lr_wiener_v lr_sgr_ab lr_sgr_f lr_stream lr_bconv lr_bstore sr_stream fc_lfifo fc_plane fc_top fc_rstore fc_out dec_top
       dec_host phase_gen"
sources="$root/synth/vivado/frame_mem_pr.sv"
for f in $files; do sources="$sources $root/rtl/$f.sv"; done
# the Verilog shells (the module-reference flow wants a Verilog top)
sources="$sources $here/dec_host_v.v $here/phase_gen_v.v"
# reference fetch's own way to the DDR4 controller (Vivado only: XPM FIFOs), with its shell
sources="$sources $here/rf_ddr_bridge.sv $here/rf_ddr_bridge_v.v"
export U200_SOURCES=$sources
# qm_rom loads rtl/qm_rom.hex by a path relative to a simulation build dir (../../rtl/): the runs execute in
# proj/u200.runs/<run>/, so proj/rtl must be the rtl directory
mkdir -p "$out/proj"; ln -sfn "$root/rtl" "$out/proj/rtl"
echo "u200 build $tag: $config"
cd "$out/work"
vivado -mode batch -nojournal -log "$out/vivado.log" -source "$here/build.tcl"
if [ $bdonly = 1 ]; then
  grep -q BD_ONLY_DONE "$out/vivado.log" || { echo "block design failed, see $out/vivado.log" >&2; exit 1; }
  grep -E "^memory map:|^decoder:|^clk = |CRITICAL WARNING|^ERROR" "$out/vivado.log" || true
  echo "block design made and validated: $out/proj (no image)"; exit 0
fi
grep -q BUILD_DONE "$out/vivado.log" || { echo "build failed, see $out/vivado.log" >&2; exit 1; }
cp "$out/proj/u200.runs/impl_1/sys_wrapper.bit" "$out/"
grep -A2 "WNS" "$out/impl_timing.rpt" | head -8 || true
echo "bitstream: $out/sys_wrapper.bit (program over JTAG with fpga/u200/program.sh, then rescan PCIe or reboot the host)"
