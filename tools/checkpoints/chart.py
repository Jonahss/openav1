#!/usr/bin/env python3
"""chart.py <docs/checkpoints dir>: the fps-over-time chart of the checkpoint table (checkpoints.csv -> fps.svg, and
fps.png when a headless Chromium is on the PATH). A step line per clip: a speed holds until the next image. The 1080p
clip is checkpoints.csv; the 4K clip (checkpoints_4k.csv, since 2026-11-22: the goal is 4K at 60 and the 1080p line
cannot show progress there) is a second line on the same fps axis, until it catches up.
The speed is the unchecked run (nothing compared) where the row has one, C10 on, and the checked run before that;
the first unchecked point carries a dagger and the footnote says so. Standard library only. Colours: the dataviz reference palette, slots 1 and 2 on the light surface (validated together: scripts/validate_palette.js)."""
import csv, shutil, subprocess, sys
from datetime import datetime as D, timedelta
from pathlib import Path

d = Path(sys.argv[1])
rows = list(csv.DictReader(open(d / "checkpoints.csv")))
T = lambda s: D.strptime(s, "%Y-%m-%d %H:%M")
pts = [(T(r["date_pt"]), float(r["fps_unchecked"] or r["fps_checked"]), r) for r in rows if r["fps_unchecked"] or r["fps_checked"]]
f4 = d / "checkpoints_4k.csv"
rows4 = list(csv.DictReader(open(f4))) if f4.exists() else []
pts4 = [(T(r["date_pt"]), float(r["fps_unchecked"] or r["fps_checked"]), r) for r in rows4 if r["fps_unchecked"] or r["fps_checked"]]
first_un = next((r["id"] for r in rows if r["fps_unchecked"]), None)     # the measurement changes here
W, H = 1200, 675
L, R, TOP, B = 70, 250, 96, 70
t0 = D(pts[0][0].year, pts[0][0].month, pts[0][0].day)
t1 = max(p[0] for p in pts + pts4) + timedelta(hours=15)
top = max(v for _, v, _ in pts + pts4)
ymax = 5 * (int(top * 1.08) // 5 + 1)
X = lambda t: L + (W - L - R) * (t - t0).total_seconds() / (t1 - t0).total_seconds()
Y = lambda v: H - B - (H - TOP - B) * v / ymax
ink, ink2, grid, blue, orange, surf = "#0b0b0b", "#52514e", "#e4e3df", "#2a78d6", "#eb6834", "#fcfcfb"
series = [(pts, blue, "1080p")] + ([(pts4, orange, "4K")] if pts4 else [])
o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="DejaVu Sans, Liberation Sans, Arial, sans-serif">',
     f'<rect width="{W}" height="{H}" fill="{surf}"/>',
     f'<text x="{L}" y="40" font-size="24" font-weight="700" fill="{ink}">openav1 on the Alveo U200: decode speed</text>',
     f'<text x="{L}" y="66" font-size="15" fill="{ink2}">Frames per second, card and host end to end: one 1080p clip (548 frames)' + (" and one 4K clip (248 frames)" if pts4 else "") + '</text>']
for i, (_, col, name) in enumerate(series if pts4 else []):          # legend: two series are never told apart by colour alone
    lx = W - R - 150 + 80 * i
    o.append(f'<line x1="{lx}" x2="{lx + 22}" y1="{TOP - 20}" y2="{TOP - 20}" stroke="{col}" stroke-width="2"/><circle cx="{lx + 11}" cy="{TOP - 20}" r="4" fill="{col}"/>')
    o.append(f'<text x="{lx + 28}" y="{TOP - 15}" font-size="13" fill="{ink2}">{name}</text>')
for v in range(0, ymax + 1, 5):
    o.append(f'<line x1="{L}" x2="{W - R}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" stroke="{grid}" stroke-width="1"/>')
    o.append(f'<text x="{L - 10}" y="{Y(v) + 5:.1f}" font-size="13" fill="{ink2}" text-anchor="end">{v}</text>')
o.append(f'<text x="{L - 10}" y="{TOP - 14}" font-size="13" fill="{ink2}" text-anchor="end">fps</text>')
days = (t1 - t0).days + 1
step = 2 if days <= 16 else 7 if days <= 70 else 14
for k in range(0, days, step):
    dt = t0 + timedelta(days=k)
    o.append(f'<line x1="{X(dt):.1f}" x2="{X(dt):.1f}" y1="{H - B}" y2="{H - B + 5}" stroke="{ink2}" stroke-width="1"/>')
    o.append(f'<text x="{X(dt):.1f}" y="{H - B + 22}" font-size="13" fill="{ink2}" text-anchor="middle">{dt.strftime("%b")} {dt.day}</text>')
for sp, col, name in series:
    path = f"M{X(sp[0][0]):.1f},{Y(sp[0][1]):.1f}"
    for (ta, va, _), (tb, vb, _) in zip(sp, sp[1:]):
        path += f" L{X(tb):.1f},{Y(va):.1f} L{X(tb):.1f},{Y(vb):.1f}"
    if pts4:                                                 # both lines run to the right edge, where their names are
        path += f" L{X(t1):.1f},{Y(sp[-1][1]):.1f}"
        o.append(f'<text x="{X(t1) + 8:.1f}" y="{Y(sp[-1][1]) + 5:.1f}" font-size="13" font-weight="700" fill="{ink}">{name}</text>')
    o.append(f'<path d="{path}" fill="none" stroke="{col}" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>')
    for t, v, r in sp:
        o.append(f'<circle cx="{X(t):.1f}" cy="{Y(v):.1f}" r="5" fill="{col}" stroke="{surf}" stroke-width="2"><title>{r["id"]} {r["date_pt"]}: {v} fps{" unchecked (" + r["fps_checked"] + " checked)" if r["fps_unchecked"] else " checked"}. {r["what_changed"]}</title></circle>')
for t, v, r in pts + pts4:
    if not r["label"]:
        continue
    dx, dy, anchor, text = r["label"].split(";", 3)
    x, y, lines = X(t) + float(dx), Y(v) + float(dy), text.split("|")
    v = f"{v}\u2020" if r["id"] == first_un else v            # dagger: see the footnote
    if len(lines) == 1 and abs(float(dy)) < 12:              # number and note on one line, beside the dot
        o.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="15" font-weight="700" fill="{ink}" text-anchor="{anchor}">{v}</text>')
        o.append(f'<text x="{x + 12 + 9.5 * len(str(v)):.1f}" y="{y:.1f}" font-size="12.5" fill="{ink2}">{lines[0]}</text>')
    else:
        o.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="15" font-weight="700" fill="{ink}" text-anchor="{anchor}">{v}</text>')
        for i, l in enumerate(lines):
            o.append(f'<text x="{x:.1f}" y="{y + 17 * (i + 1):.1f}" font-size="12.5" fill="{ink2}" text-anchor="{anchor}">{l}</text>')
o.append(f'<text x="{L}" y="{H - 30}" font-size="12" fill="{ink2}">Each dot is a card image; the line holds a speed until the next one. \u2020 From here the speed is measured without the picture-by-picture check</text>')
o.append(f'<text x="{L}" y="{H - 14}" font-size="12" fill="{ink2}">(that image: {next((r["fps_checked"] for r in rows if r["id"] == first_un), "")} with it); every image is still verified against the reference decoder. Tables: docs/checkpoints/*.csv</text>')
o.append("</svg>")
(d / "fps.svg").write_text("\n".join(o))
chrome = shutil.which("chromium") or shutil.which("chromium-browser") or shutil.which("google-chrome")
if chrome:
    html = d / "_fps.html"
    html.write_text(f'<!doctype html><html><body style="margin:0;background:{surf}">' + "\n".join(o) + "</body></html>")
    raw = d / "_raw.png"
    subprocess.run([chrome, "--headless", "--no-sandbox", "--disable-gpu", "--hide-scrollbars", "--force-device-scale-factor=2",
                    f"--window-size={W},{H + 150}", f"--screenshot={raw}", f"file://{html.resolve()}"], capture_output=True)
    try:
        from PIL import Image
        Image.open(raw).crop((0, 0, 2 * W, 2 * H)).save(d / "fps.png")
    except ImportError:
        raw.rename(d / "fps.png")
    for f in (html, raw):
        f.unlink(missing_ok=True)
print(f"{len(pts)} + {len(pts4)} checkpoints -> {d / 'fps.svg'}" + (f", {d / 'fps.png'}" if chrome else ""))
