#!/usr/bin/env python3
"""Memories by module from Vivado hierarchical utilisation reports (report_utilization -hierarchical):
    tools/mem_by_module.py <a/util_hier.rpt> [<b/util_hier.rpt>]
One row per instance that holds LUT RAM, block RAM or UltraRAM: LUTs, LUTRAMs, RAMB36, RAMB18, URAM; with two reports,
both side by side (the same decoder at two frame-buffer sizes: what a wider picture costs, and where)."""
import sys

MEMS = ("lutram", "b36", "b18", "uram")


def read(path):
    rows = {}
    for line in open(path):
        cells = line.split("|")
        f = [x.strip() for x in cells]
        if len(f) < 12 or not f[4].isdigit():
            continue
        depth = (len(cells[1]) - len(cells[1].lstrip())) // 2
        rows[(depth, f[1])] = dict(lut=int(f[3]), lutram=int(f[5]), b36=int(f[8]), b18=int(f[9]), uram=int(f[10]))
    return rows


reps = [read(p) for p in sys.argv[1:3]]
keys = []
for r in reps:
    keys += [k for k in r if k not in keys and any(r[k][m] for m in MEMS)]
head = "instance".ljust(34)
for _ in reps:
    head += " | " + "LUT".rjust(7) + " " + "LUTRAM".rjust(7) + " " + "B36".rjust(5) + " " + "B18".rjust(4) + " " + "URAM".rjust(5)
print(head)
for depth, name in keys:
    row = ("  " * depth + name).ljust(34)[:34]
    for r in reps:
        v = r.get((depth, name))
        row += " | " + ("%7d %7d %5d %4d %5d" % (v["lut"], v["lutram"], v["b36"], v["b18"], v["uram"]) if v else " " * 32)
    print(row)
