#!/usr/bin/env python3
"""timing_census.py <paths.csv> [depth]: group the failing endpoints of tools/timing_census.tcl by (start block -> end block).
A block is the instance path under u_dec cut to `depth` levels (default 2), generate indices and register names dropped.
Prints per group: endpoints, worst slack, median levels, median share of routing in the data path."""
import csv, re, sys, statistics as st
from collections import defaultdict
depth = int(sys.argv[2]) if len(sys.argv) > 2 else 2
def blk(pin):
    p = pin.rsplit("/", 2)[0] if pin.count("/") > 1 else pin        # drop cell/pin
    p = re.sub(r"^.*?/u/", "", p)
    p = re.sub(r"\[\d+\]", "", p)
    parts = [x for x in p.split("/") if not re.search(r"_reg(_|$)", x)]     # memory / register cells are not blocks
    if not parts: return "(top)"                                    # an endpoint with no block path (a top-level register)
    if parts[0] != "u_dec": return "/".join(parts[:2])
    return "/".join(parts[1:1 + depth]) or "u_dec"
g = defaultdict(list)
rows = list(csv.DictReader(open(sys.argv[1])))
for r in rows:
    g[(blk(r["start"]), blk(r["end"]))].append(r)
if not rows:
    print("0 endpoints below the limit"); sys.exit(0)
print(f"{len(rows)} endpoints below the limit, worst {min(float(r['slack']) for r in rows):.3f} ns, {len(g)} groups")
print(f"{'n':>6} {'worst':>7} {'lvl':>4} {'route%':>6}  start -> end")
for k, v in sorted(g.items(), key=lambda kv: -len(kv[1])):
    sl = min(float(r["slack"]) for r in v); lv = st.median(int(r["levels"]) for r in v)
    rt = st.median(100 * float(r["route"]) / max(float(r["datapath"]), 1e-9) for r in v)
    print(f"{len(v):>6} {sl:>7.3f} {lv:>4.0f} {rt:>6.0f}  {k[0]} -> {k[1]}")
