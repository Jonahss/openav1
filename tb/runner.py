"""cocotb 2.x runner: python tb/runner.py [top] [test_module]  (defaults: smoke / test_smoke)
Set WAVES=1 to record an FST trace in the build dir (slow and large for long replays); WAVE_FMT=vcd for a VCD; WAVE_DEPTH=1 traces only the top's ports (small, fast).
TS_BUILD_ONLY=1 builds and exits; TS_TEST_DIR=<dir> runs the test there instead of the build dir, so several
runs can share one build (keep it two levels below the repo, like build/<x>/, for the relative paths)."""
import os, sys
from pathlib import Path
from cocotb_tools.runner import get_runner

ROOT = Path(__file__).resolve().parent.parent
top = sys.argv[1] if len(sys.argv) > 1 else "smoke"
mod = sys.argv[2] if len(sys.argv) > 2 else f"test_{top}"
waves = os.environ.get("WAVES", "0") == "1"
waves_fmt = os.environ.get("WAVE_FMT", "fst")      # fst, or vcd (needs neither lz4 nor a working FST writer)
sys.path.insert(0, str(ROOT / "tb"))
r = get_runner("verilator")
EXTRA = {"itx1d": ["itx_ucode.sv", "cos128_lut.sv"], "itx2d": ["itx_ucode.sv", "cos128_lut.sv", "itx1d.sv"],
         "coef_top": ["cdf_map_pkg.sv", "tx_tables_pkg.sv", "scan_rom.sv", "cdf_store.sv", "msac.sv", "sym_seq.sv", "coef_rd.sv"],
         "tile_syntax": ["cdf_map_pkg.sv", "tx_tables_pkg.sv", "blk_tables_pkg.sv", "syn_pkg.sv", "scan_rom.sv", "cdf_store.sv", "msac.sv",
                         "sym_seq.sv", "coef_rd.sv", "blk_ctx.sv", "pal_syntax.sv", "mv_mem.sv", "mv_stack.sv", "blk_syntax.sv", "lr_syntax.sv"],
         "dec_top": ["cdf_map_pkg.sv", "tx_tables_pkg.sv", "blk_tables_pkg.sv", "q_tables_pkg.sv", "syn_pkg.sv", "scan_rom.sv", "cdf_store.sv", "msac.sv",
                     "sym_seq.sv", "coef_rd.sv", "blk_ctx.sv", "pal_syntax.sv", "mv_mem.sv", "mv_stack.sv", "blk_syntax.sv", "lr_syntax.sv", "tile_syntax.sv",
                     "itx_ucode.sv", "cos128_lut.sv", "itx1d.sv", "itx2d.sv", "ipred.sv", "cfl.sv", "qm_rom.sv", "frame_mem.sv", "frame_mem_w.sv", "recon_top.sv",
                     "cdef_pkg.sv", "lr_pkg.sv", "mi_store.sv", "rec_fifo.sv", "lf_top.sv", "cdef_top.sv", "lr_top.sv", "sr_top.sv"],
         "lf_tb_top": ["blk_tables_pkg.sv", "tx_tables_pkg.sv", "cdf_map_pkg.sv", "syn_pkg.sv", "frame_mem.sv", "frame_mem_w.sv", "mi_store.sv", "lf_top.sv"],
         "lr_tb_top": ["blk_tables_pkg.sv", "tx_tables_pkg.sv", "cdf_map_pkg.sv", "syn_pkg.sv", "lr_pkg.sv", "frame_mem.sv", "frame_mem_w.sv", "mi_store.sv", "lr_top.sv"],
         "cdef_tb_top": ["blk_tables_pkg.sv", "tx_tables_pkg.sv", "cdf_map_pkg.sv", "syn_pkg.sv", "cdef_pkg.sv", "frame_mem.sv", "frame_mem_w.sv", "mi_store.sv", "cdef_top.sv"]}
sources = [ROOT / "rtl" / f for f in EXTRA.get(top, [])] + [ROOT / "rtl" / f"{top}.sv"]
# frame-buffer / per-4x4 state capacity for the frame-level tops (dec_top, *_tb_top): FBX/FBY = log2 pixels,
# ML2 = log2 4x4 units (FBX-2 / FBY-2 covers the same picture). Defaults fit 1024x512; Argon needs more.
params = {}
if top in ("dec_top", "lf_tb_top", "cdef_tb_top", "lr_tb_top") and os.environ.get("FBX"):
    params = {"FBX": int(os.environ["FBX"]), "FBY": int(os.environ.get("FBY", os.environ["FBX"]))}
# ITX_LANES=2 builds the K26 configuration (two transform engines); vectors for a 2-lane netlist replay must be
# recorded from this build, since the lane count changes the cycle cadence (2026-10-12: a 4-lane recording
# replayed against the 2-lane netlist mismatched from the first transform block on).
if top in ("dec_top", "recon_top") and os.environ.get("ITX_LANES"):
    params["ITX_LANES"] = int(os.environ["ITX_LANES"])
# SIM_FAST=1: optimise for simulation speed (Verilator -O3, compiler -O2, N threads); default is Verilator's -Os,
# which compiles fastest. SIM_THREADS overrides the thread count (default: all cores).
build_args = ["-Wall", "-Wno-UNUSEDPARAM", "-Wno-UNUSEDSIGNAL", "-Wno-PINCONNECTEMPTY"] + (["--trace-fst"] if waves and waves_fmt == "fst" else []) \
             + (["--trace-depth", os.environ["WAVE_DEPTH"]] if waves and os.environ.get("WAVE_DEPTH") else [])
if os.environ.get("SIM_FAST", "0") == "1":
    threads = int(os.environ.get("SIM_THREADS", str(os.cpu_count() or 1)))
    build_args += ["-O3", "--x-assign", "fast", "--x-initial", "fast", "-CFLAGS", "-O2", "--threads", str(threads), "-Wno-UNOPTTHREADS"]
# One build dir per configuration (build/<top> or build/<top>_fb<X>x<Y>), so Argon classes with different
# frame-buffer sizes build and run side by side instead of rebuilding one dir in turn.
build_dir = ROOT / "build" / (top + (f"_fb{params['FBX']}x{params['FBY']}" if "FBX" in params else "")
                              + (f"_l{params['ITX_LANES']}" if "ITX_LANES" in params else ""))
r.build(sources=sources, hdl_toplevel=top, parameters=params,
        build_dir=build_dir, build_args=build_args, waves=waves)
if os.environ.get("TS_BUILD_ONLY") == "1":
    sys.exit(0)
test_dir = Path(os.environ["TS_TEST_DIR"]).resolve() if os.environ.get("TS_TEST_DIR") else build_dir
plusargs = [f"+vis={os.environ['TS_VIS']}"] if os.environ.get("TS_VIS") else []   # decode-progress trace (docs/vis-trace.md)
r.test(hdl_toplevel=top, test_module=mod, build_dir=build_dir, test_dir=test_dir, waves=waves, plusargs=plusargs)
