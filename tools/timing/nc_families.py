#!/usr/bin/env python3
"""Group a near_crit.tcl census into families: (start register, end register) with indices and replicas stripped.
usage: nc_families.py <census.txt> [top N]"""
import re, sys, collections

def norm(pin):
    p = pin.replace("sys_i/host/inst/", "")
    p = re.sub(r"/[A-Z0-9_\[\]]+$", "", p)            # the pin
    p = re.sub(r"_replica(_\d+)?", "", p)
    p = re.sub(r"\[\d+\]", "[]", p)
    p = re.sub(r"_reg(_\d+)*(_bram_\d+)?(_i_\d+)?$", "_reg", p)
    p = re.sub(r"mem_reg_\d+_bram_\d+", "mem_reg", p)
    p = re.sub(r"/DSP_[A-Z_]+_INST$", "(dsp)", p)
    return p

fam = collections.defaultdict(list)
for line in open(sys.argv[1]):
    if line.startswith("#") or not line.strip():
        continue
    f = line.split()
    fam[(norm(f[2]), norm(f[3]))].append((float(f[0]), int(f[1]), float(f[4]), float(f[5])))
top = int(sys.argv[2]) if len(sys.argv) > 2 else 25
rows = sorted(fam.items(), key=lambda kv: min(x[0] for x in kv[1]))
print(f"{sum(len(v) for v in fam.values())} endpoints, {len(fam)} families; failing endpoints: {sum(1 for v in fam.values() for x in v if x[0] < 0)}")
print("worst  n  n<0  levels  route%  start -> end")
for (s, e), v in rows[:top]:
    w = min(v)
    print(f"{w[0]:6.3f} {len(v):4d} {sum(1 for x in v if x[0] < 0):4d}  {w[1]:3d}   {100 * w[3] / (w[2] + w[3]):3.0f}   {s} -> {e}")
