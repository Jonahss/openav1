#!/usr/bin/env python3
"""Block RAM and UltraRAM by table, from Vivado's report_ram_utilization (synth/vivado/run.sh writes ram.rpt):
    tools/ram_by_table.py <ram.rpt> [instance-prefix] [--min N]
One row per inferred memory under the prefix (default: everything): its array (depth x width), RAMB36, RAMB18, URAM288
and the share of the primitives' bits it fills, largest first (rows under --min block-RAM tiles are summed in one
line; a RAMB18 is half a tile). tools/mem_by_module.py gives the totals by module; this says which table inside."""
import re
import sys

args = [a for a in sys.argv[1:] if not a.startswith("--")]
minimum = float(sys.argv[sys.argv.index("--min") + 1]) if "--min" in sys.argv else 0.0
if "--min" in sys.argv:
    args = [a for a in args if a != sys.argv[sys.argv.index("--min") + 1]]
path, prefix = args[0], (args[1] if len(args) > 1 else "")
lines = open(path).read().split("\n")
start = [i for i, l in enumerate(lines) if l.startswith("3. Memory Utilization - Inferred")][-1]    # [0] is the contents list
end = [i for i, l in enumerate(lines) if l.startswith("4. Memory Performance - Inferred")][-1]
dims = {}
for l in lines[:start]:                                  # section 2: the array size of each memory
    m = re.match(r"\| (\S+)\s+\|\s+(\d+) \|[^|]*\|[^|]*\|\s+(\d+x\d+)", l)
    if m:
        dims[m.group(1)] = m.group(3)
tables, cur = {}, None
for l in lines[start:end]:
    c = [x.strip() for x in l.split("|")]
    if len(c) < 6 or c[1] in ("", "Memory Name"):
        continue
    if not l.startswith("|  "):                          # a memory; its primitives follow, indented
        cur = tables.setdefault(c[1], {"RAMB36": 0, "RAMB18": 0, "URAM288": 0, "avail": 0, "used": 0})
    elif cur is not None:
        kind = next((k for k in ("RAMB36", "RAMB18", "URAM288") if c[2].startswith(k)), None)
        if kind:
            cur[kind] += 1; cur["avail"] += int(c[4]); cur["used"] += int(c[5])
rows = [(n, t) for n, t in tables.items() if n.startswith(prefix) and (t["RAMB36"] or t["RAMB18"] or t["URAM288"])]
tiles = lambda t: t["RAMB36"] + t["RAMB18"] / 2
rows.sort(key=lambda r: (-tiles(r[1]), -r[1]["URAM288"], r[0]))
print(f"{'memory':58s} {'array':>14s} {'B36':>5s} {'B18':>4s} {'URAM':>5s} {'fill':>5s}")
tot = {"RAMB36": 0, "RAMB18": 0, "URAM288": 0}; small = {"RAMB36": 0, "RAMB18": 0, "URAM288": 0, "n": 0}
for n, t in rows:
    for k in tot:
        tot[k] += t[k]
    if tiles(t) < minimum and not t["URAM288"]:
        small["n"] += 1
        for k in tot:
            small[k] += t[k]
        continue
    print(f"{n[len(prefix):][-58:]:58s} {dims.get(n, ''):>14s} {t['RAMB36']:5d} {t['RAMB18']:4d} {t['URAM288']:5d} {100 * t['used'] / max(t['avail'], 1):4.0f}%")
if small["n"]:
    print(f"{'(' + str(small['n']) + ' memories under ' + str(minimum) + ' tiles)':58s} {'':>14s} {small['RAMB36']:5d} {small['RAMB18']:4d} {small['URAM288']:5d}")
print(f"{'total under ' + (prefix or 'the top'):58s} {'':>14s} {tot['RAMB36']:5d} {tot['RAMB18']:4d} {tot['URAM288']:5d}   = {tot['RAMB36'] + tot['RAMB18'] / 2:.1f} block RAM tiles")
