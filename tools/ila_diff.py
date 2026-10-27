#!/usr/bin/env python3
"""Diff an ILA capture off the card against the twin's TS_ILA_LOG (tb/test_dec_host_inter.py ila_log), cycle by cycle.

    tools/ila_diff.py <ila_export.csv> <twin_log.csv> [--cycles N] [--skip col,col]

Both start at the cycle of the first ev_pop (the ILA trigger). The ILA's column names carry the hierarchy prefix and the
bus width ("sys_i/host/inst/u/u_dec/ev_tx[eob][10:0]"); the twin's are the names under dec_host ("u_dec/ev_tx[eob]").
Prints, per column, the first cycle it differs and how many cycles differ, then the first differing cycles in full.
"""
import csv, re, sys, argparse

ap = argparse.ArgumentParser()
ap.add_argument("ila"); ap.add_argument("twin")
ap.add_argument("--cycles", type=int, default=0, help="compare only the first N cycles")
ap.add_argument("--skip", default="start", help="columns to ignore (comma list; default: the host's start pulses)")
ap.add_argument("--show", type=int, default=12, help="how many differing cycles to print")
a = ap.parse_args()

def norm(name):
    name = name.split("/u/", 1)[1] if "/u/" in name else name
    return re.sub(r"\[\d+:\d+\]$", "", name)

ila_rows = list(csv.reader(open(a.ila)))
ila_cols = [norm(c) for c in ila_rows[0]]
ila = [r for r in ila_rows[1:] if r and r[0] != "Radix - UNSIGNED"]
twin_rows = list(csv.reader(open(a.twin)))
twin_cols = twin_rows[0]; twin = twin_rows[1:]
skip = set(a.skip.split(",")) if a.skip else set()
common = [c for c in twin_cols if c in ila_cols and c not in skip]
missing = [c for c in twin_cols if c not in ila_cols]
if missing:
    print("twin-only columns:", missing)
ii = {c: ila_cols.index(c) for c in common}; ti = {c: twin_cols.index(c) for c in common}
n = min(len(ila), len(twin))
if a.cycles:
    n = min(n, a.cycles)
print(f"ila {len(ila)} cycles, twin {len(twin)} cycles, comparing {n} over {len(common)} columns")

def val(s):
    s = s.strip()
    if s in ("", "NA"):
        return None
    return int(s, 16)

first = {}; count = {}; diff_cycles = []
for k in range(n):
    bad = []
    for c in common:
        v1 = val(ila[k][ii[c]]); v2 = val(twin[k][ti[c]])
        if v1 is None or v2 is None:
            continue
        if v1 != v2:
            bad.append((c, v1, v2))
            count[c] = count.get(c, 0) + 1
            first.setdefault(c, k)
    if bad:
        diff_cycles.append((k, bad))
print(f"{len(diff_cycles)} of {n} cycles differ")
for c in sorted(first, key=lambda c: first[c]):
    print(f"  {c:40s} first differs at cycle {first[c]:6d}, {count[c]} cycles")
for k, bad in diff_cycles[: a.show]:
    print(f"cycle {k}: " + "  ".join(f"{c}: card {v1:x} twin {v2:x}" for c, v1, v2 in bad))
