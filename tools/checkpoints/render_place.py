#!/usr/bin/env python3
"""render_place.py <dumpdir> <title> <out.png> — a placement picture of a routed U200 build, coloured by decoder block.
Input: <dumpdir>/cells.csv (cell\tsite\tref), sites.csv (used sites), device.csv (every SLICE/RAMB36/URAM/DSP site:
name\ttype\tRPM_X\tRPM_Y\tclock region\tSLR), written by atium ~/openav1-synth/layout/dump_place.tcl.
Each site becomes one pixel-block on a grid of the device's RPM rows/columns; the colour is the block that owns most of
the cells in that site; unused sites are the die background. Palette: the dataviz reference instance, fixed order."""
import sys, csv, collections
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

dumpdir, title, out = sys.argv[1], sys.argv[2], sys.argv[3]
GROUPS = [  # (label, hierarchical prefix test, colour)  — first match wins
    ("tile engines: entropy decoder, syntax, MV stack", lambda n: "/u_dec/u_ts" in n, "#2a78d6"),
    ("reconstruction: prediction, transforms",          lambda n: "/u_dec/u_rc" in n, "#eb6834"),
    ("loop restoration",                                lambda n: "/u_dec/u_lr" in n, "#1baf7a"),
    ("CDEF",                                            lambda n: "/u_dec/u_cdef" in n, "#eda100"),
    ("deblocking, super-resolution, mode info",         lambda n: any(k in n for k in ("/u_dec/u_lf", "/u_dec/u_sr", "/u_dec/u_mi")), "#e87ba4"),
    ("frame memory (UltraRAM rings)",                   lambda n: "/u_dec/g_fb" in n or "u_fbm" in n or "/u_dec/u_fb" in n, "#008300"),
    ("host interface, sequencer",                       lambda n: "/host/" in n, "#6250d6"),
    ("PCIe XDMA + DDR4 controller (shell)",             lambda n: "/xdma/" in n or "/ddr/" in n, "#e34948"),
]
OTHER = ("other shell logic", None, "#9b9a94")
def group_of(name):
    for i, (_, test, _) in enumerate(GROUPS):
        if test(name): return i
    return len(GROUPS)

# device grid
dev = {}
xs, ys = set(), set()
with open(f"{dumpdir}/device.csv") as f:
    for row in csv.reader(f, delimiter="\t"):
        name, typ, rx, ry, region, slr = row[:6]
        rx, ry = int(rx), int(ry); dev[name] = (typ, rx, ry, slr); xs.add(rx); ys.add(ry)
xs, ys = sorted(xs), sorted(ys); xi = {v: i for i, v in enumerate(xs)}; yi = {v: i for i, v in enumerate(ys)}
W, H = len(xs), len(ys)
# cells -> site -> majority group; LUT/FF/RAM/DSP counts per group
site_votes = collections.defaultdict(collections.Counter)
counts = collections.defaultdict(collections.Counter)
site_kinds = collections.defaultdict(set)
with open(f"{dumpdir}/cells.csv") as f:
    for row in csv.reader(f, delimiter="\t"):
        if len(row) < 3: continue
        name, site, ref = row[:3]
        g = group_of(name); site_votes[site][g] += 1
        if ref.startswith("LUT"): counts[g]["LUT"] += 1
        elif ref.startswith("FD"): counts[g]["FF"] += 1
        else:   # DSP / BRAM / URAM primitives appear as several leaf cells per site: count sites, once each
            kind = "DSP" if site.startswith("DSP48E2_") else "BRAM" if site.startswith("RAMB36_") else "BRAM18" if site.startswith("RAMB18_") else "URAM" if site.startswith("URAM288_") else None
            if kind: site_kinds[kind].add((g, site))
for kind, pairs in site_kinds.items():
    for g, site in pairs: counts[g]["BRAM" if kind == "BRAM18" else kind] += 0.5 if kind == "BRAM18" else 1
img = np.ones((H, W, 3)) * np.array([0.945, 0.943, 0.93])   # die background
alpha_mask = np.zeros((H, W), bool)
def hex2rgb(h): return np.array([int(h[i:i+2], 16)/255 for i in (1, 3, 5)])
cols = [hex2rgb(c) for _, _, c in GROUPS] + [hex2rgb(OTHER[2])]
for name, (typ, rx, ry, slr) in dev.items():
    if name in site_votes:
        g = site_votes[name].most_common(1)[0][0]
        img[yi[ry], xi[rx]] = cols[g]; alpha_mask[yi[ry], xi[rx]] = True
# SLR boundaries from the device sites
slr_rows = collections.defaultdict(set)
for name, (typ, rx, ry, slr) in dev.items(): slr_rows[slr].add(yi[ry])
fig_h = 16; fig_w = max(9, fig_h * W / H * 1.0)
fig, ax = plt.subplots(figsize=(fig_w + 6.5, fig_h), dpi=150)
ax.imshow(img, origin="lower", interpolation="nearest", aspect="auto")
for slr, rows in sorted(slr_rows.items()):
    lo, hi = min(rows), max(rows)
    ax.axhline(hi + 0.5, color="#52514e", lw=0.8, ls=(0, (4, 3)))
    ax.text(W * 0.015, (lo + hi) / 2, f"SLR {slr}", color="#52514e", fontsize=11, va="center", ha="left", bbox=dict(facecolor="#fcfcfb", edgecolor="none", alpha=0.85, pad=2))
ax.set_xticks([]); ax.set_yticks([])
for s in ax.spines.values(): s.set_edgecolor("#c3c2b7")
used = sum(1 for n in dev if n in site_votes)
ax.set_title(title.replace(" | ", "\n"), loc="left", fontsize=14, fontweight="bold", color="#0b0b0b", pad=14)
handles = []
for i, (label, _, c) in enumerate(list(GROUPS) + [OTHER]):
    k = counts[i]
    parts = [f"{k['LUT']:,} LUT"] + ([f"{int(k['DSP'])} DSP"] if k["DSP"] else []) + ([f"{int(k['URAM'])} URAM"] if k["URAM"] else []) + ([f"{k['BRAM']:g} BRAM"] if k["BRAM"] else [])
    handles.append(Patch(facecolor=c, edgecolor="none", label=f"{label}\n   {', '.join(parts)}"))
leg = ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=10.5, labelspacing=1.1, handlelength=1.6, handleheight=1.6, borderaxespad=0)
fig.text(0.5 * fig_w / (fig_w + 6.5), 0.012, f"{used:,} of {len(dev):,} logic/RAM/DSP sites used; one cell per pixel-block, coloured by the block that owns most of its cells", fontsize=10, color="#52514e", ha="center")
fig.patch.set_facecolor("#fcfcfb"); ax.set_facecolor("#fcfcfb")
plt.subplots_adjust(left=0.02, right=0.98 - 6.0 / (fig_w + 6.5), top=0.95, bottom=0.04)
fig.savefig(out, dpi=150, facecolor=fig.get_facecolor()); print("wrote", out, f"grid {W}x{H}, {used} used sites")
