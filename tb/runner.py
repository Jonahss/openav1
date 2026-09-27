"""cocotb 2.x runner: python tb/runner.py [top] [test_module]  (defaults: smoke / test_smoke)
Set WAVES=1 to record an FST trace in build/<top>/ (slow and large for long replays)."""
import os, sys
from pathlib import Path
from cocotb_tools.runner import get_runner

ROOT = Path(__file__).resolve().parent.parent
top = sys.argv[1] if len(sys.argv) > 1 else "smoke"
mod = sys.argv[2] if len(sys.argv) > 2 else f"test_{top}"
waves = os.environ.get("WAVES", "0") == "1"
sys.path.insert(0, str(ROOT / "tb"))
r = get_runner("verilator")
EXTRA = {"itx1d": ["itx_ucode.sv", "cos128_lut.sv"], "itx2d": ["itx_ucode.sv", "cos128_lut.sv", "itx1d.sv"],
         "coef_top": ["cdf_map_pkg.sv", "tx_tables_pkg.sv", "scan_rom.sv", "cdf_store.sv", "msac.sv", "sym_seq.sv", "coef_rd.sv"]}
sources = [ROOT / "rtl" / f for f in EXTRA.get(top, [])] + [ROOT / "rtl" / f"{top}.sv"]
r.build(sources=sources, hdl_toplevel=top,
        build_dir=ROOT / "build" / top, build_args=["-Wall", "-Wno-UNUSEDPARAM", "-Wno-UNUSEDSIGNAL"] + (["--trace-fst"] if waves else []),
        waves=waves)
r.test(hdl_toplevel=top, test_module=mod, build_dir=ROOT / "build" / top, waves=waves)
