#!/usr/bin/env python3
"""tools/check_perf_names.py: the state names tb/host_regs.py gives reconstruction's counters (RECON_RS, RECON_BST)
must be rtl/recon_top.sv's enums rst_t and bst_t, in order: the host reads the counters by index."""
import re
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tb"))
import host_regs as HR                                       # noqa: E402
src = (ROOT / "rtl" / "recon_top.sv").read_text()
bad = 0
for typ, names in (("rst_t", HR.RECON_RS), ("bst_t", HR.RECON_BST)):
    m = re.search(r"typedef enum logic \[4:0\] \{([^}]*)\}\s*" + typ + ";", src)
    rtl = [x.strip() for x in re.sub(r"//[^\n]*", "", m.group(1)).replace("\n", " ").split(",") if x.strip()]
    if rtl != list(names):
        bad = 1
        print(f"{typ}: rtl {rtl}\n      host_regs {list(names)}")
print("perf names: " + ("MISMATCH" if bad else f"match ({len(HR.RECON_RS)} + {len(HR.RECON_BST)} states)"))
sys.exit(bad)
