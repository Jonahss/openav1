#!/usr/bin/env python3
"""chart.py <docs/checkpoints dir>: the fps-over-time chart of the checkpoint table (checkpoints.csv -> fps.svg, and
fps.png when a headless Chromium is on the PATH). One series, a step line: a speed holds until the next image.
Solid dots are checked runs (every picture compared), hollow dots the unchecked run of the same image. Standard
library only. Colours: the dataviz reference palette, slot 1 on the light surface."""
import csv, shutil, subprocess, sys
from datetime import datetime as D, timedelta
from pathlib import Path

d = Path(sys.argv[1])
rows = list(csv.DictReader(open(d / "checkpoints.csv")))
T = lambda s: D.strptime(s, "%Y-%m-%d %H:%M")
pts = [(T(r["date_pt"]), float(r["fps_checked"]), r) for r in rows if r["fps_checked"]]
W, H = 1200, 675
L, R, TOP, B = 70, 250, 96, 70
t0 = D(pts[0][0].year, pts[0][0].month, pts[0][0].day)
t1 = pts[-1][0] + timedelta(hours=15)
top = max(max(v for _, v, _ in pts), max((float(r["fps_unchecked"]) for r in rows if r["fps_unchecked"]), default=0))
ymax = 5 * (int(top * 1.08) // 5 + 1)
X = lambda t: L + (W - L - R) * (t - t0).total_seconds() / (t1 - t0).total_seconds()
Y = lambda v: H - B - (H - TOP - B) * v / ymax
ink, ink2, grid, blue, surf = "#0b0b0b", "#52514e", "#e4e3df", "#2a78d6", "#fcfcfb"
o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="DejaVu Sans, Liberation Sans, Arial, sans-serif">',
     f'<rect width="{W}" height="{H}" fill="{surf}"/>',
     f'<text x="{L}" y="40" font-size="24" font-weight="700" fill="{ink}">openav1 on the Alveo U200: 1080p decode speed</text>',
     f'<text x="{L}" y="66" font-size="15" fill="{ink2}">Frames per second on the same 548-frame clip, every picture checked against the reference decoder</text>']
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
path = f"M{X(pts[0][0]):.1f},{Y(pts[0][1]):.1f}"
for (ta, va, _), (tb, vb, _) in zip(pts, pts[1:]):
    path += f" L{X(tb):.1f},{Y(va):.1f} L{X(tb):.1f},{Y(vb):.1f}"
o.append(f'<path d="{path}" fill="none" stroke="{blue}" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>')
for t, v, r in pts:
    o.append(f'<circle cx="{X(t):.1f}" cy="{Y(v):.1f}" r="5" fill="{blue}" stroke="{surf}" stroke-width="2"><title>{r["id"]} {r["date_pt"]}: {v} fps checked. {r["what_changed"]}</title></circle>')
for t, v, r in pts:
    if not r["label"]:
        continue
    dx, dy, anchor, text = r["label"].split(";", 3)
    x, y, lines = X(t) + float(dx), Y(v) + float(dy), text.split("|")
    if len(lines) == 1 and abs(float(dy)) < 12:              # number and note on one line, beside the dot
        o.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="15" font-weight="700" fill="{ink}" text-anchor="{anchor}">{v}</text>')
        o.append(f'<text x="{x + 12 + 9.5 * len(str(v)):.1f}" y="{y:.1f}" font-size="12.5" fill="{ink2}">{lines[0]}</text>')
    else:
        o.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="15" font-weight="700" fill="{ink}" text-anchor="{anchor}">{v}</text>')
        for i, l in enumerate(lines):
            o.append(f'<text x="{x:.1f}" y="{y + 17 * (i + 1):.1f}" font-size="12.5" fill="{ink2}" text-anchor="{anchor}">{l}</text>')
unchecked = [(T(r["date_pt"]), float(r["fps_unchecked"]), float(r["fps_checked"]), r) for r in rows if r["fps_unchecked"]]
for k, (t, u, c, r) in enumerate(unchecked):
    ux, uy = X(t) + 3, Y(u)
    o.append(f'<circle cx="{ux:.1f}" cy="{uy:.1f}" r="5" fill="{surf}" stroke="{blue}" stroke-width="2"><title>{r["id"]}: {u} fps unchecked</title></circle>')
    if k == len(unchecked) - 1:                               # only the newest one is labelled
        o.append(f'<text x="{ux + 16:.1f}" y="{uy - 8:.1f}" font-size="15" font-weight="700" fill="{ink}">{u}</text>')
        o.append(f'<text x="{ux + 16:.1f}" y="{uy + 9:.1f}" font-size="12.5" fill="{ink2}">same image, checking off</text>')
o.append(f'<text x="{L}" y="{H - 14}" font-size="12" fill="{ink2}">Each dot is a card image measured end to end, host included. The line holds a speed until the next image. Table: docs/checkpoints/checkpoints.csv</text>')
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
print(f"{len(pts)} checkpoints -> {d / 'fps.svg'}" + (f", {d / 'fps.png'}" if chrome else ""))
