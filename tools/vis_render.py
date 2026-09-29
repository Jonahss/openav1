#!/usr/bin/env python3
"""Render a decode-progress animation from an RTL event trace (dec_top +vis=<file>, see docs/vis-trace.md).

    tools/vis_render.py <trace.txt> <picture.yuv> WxH <out.mp4> [--seconds S] [--fps N] [--bd 8]

The picture (planar 4:2:0, the RTL's dumped output) appears block by block as the tile decoder writes each
transform block, and the in-loop filter fronts sweep down behind it, each stage tinted until the next stage
has passed. With --seconds equal to the measured wall time the animation runs at the pace the simulation ran;
--seconds 30 makes the short cut. Every event time comes from the simulation's cycle counter, scaled linearly.
Uses numpy + Pillow + ffmpeg (system python3).
"""
import argparse
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

STAGES = {"L": 1, "C": 2, "S": 3, "R": 4}           # deblock, CDEF, super-resolution, loop restoration
TINT = {0: ((90, 140, 255), 0.30),                  # reconstructed, unfiltered: cool blue
        1: ((255, 190, 60), 0.22),                  # deblocked: amber
        2: ((120, 230, 140), 0.16),                 # CDEF: green
        3: ((230, 120, 230), 0.16),                 # super-resolution: magenta
        4: (None, 0.0)}                             # restored: the final picture, no tint
LABEL = {0: "tile decoder", 1: "deblocking", 2: "CDEF", 3: "super-resolution", 4: "loop restoration"}


def load_yuv(path, w, h, bd):
    cw, ch = (w + 1) // 2, (h + 1) // 2
    if bd == 8:
        raw = np.fromfile(path, dtype=np.uint8)
        y = raw[:w * h].reshape(h, w).astype(np.float32)
        u = raw[w * h:w * h + cw * ch].reshape(ch, cw).astype(np.float32)
        v = raw[w * h + cw * ch:w * h + 2 * cw * ch].reshape(ch, cw).astype(np.float32)
    else:
        raw = np.fromfile(path, dtype="<u2").astype(np.float32) * (255.0 / ((1 << bd) - 1))
        y = raw[:w * h].reshape(h, w)
        u = raw[w * h:w * h + cw * ch].reshape(ch, cw)
        v = raw[w * h + cw * ch:w * h + 2 * cw * ch].reshape(ch, cw)
    u = np.repeat(np.repeat(u, 2, 0), 2, 1)[:h, :w]
    v = np.repeat(np.repeat(v, 2, 0), 2, 1)[:h, :w]
    # BT.601 limited range, what ffmpeg assumes for yuv420p by default
    yy = (y - 16) * (255 / 219)
    r = yy + 1.596 * (v - 128)
    g = yy - 0.392 * (u - 128) - 0.813 * (v - 128)
    b = yy + 2.017 * (u - 128)
    return np.clip(np.stack([r, g, b], -1), 0, 255).astype(np.uint8)


def read_trace(path):
    ev = []
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] == "T":
            ev.append((int(p[1]), "T", int(p[2]), int(p[3]), int(p[4]), int(p[5]), int(p[6])))
        elif p[0] == "F":
            ev.append((int(p[1]), "F", p[2], int(p[3]), int(p[4])))
        elif p[0] == "E":
            ev.append((int(p[1]), "E"))
    ev.sort(key=lambda e: e[0])
    return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("yuv"); ap.add_argument("size"); ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=30.0, help="playback length for the whole decode")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--bd", type=int, default=8)
    ap.add_argument("--scale", type=int, default=2, help="integer upscale of the picture in the video")
    ap.add_argument("--title", default="openav1: AV1 decoder RTL, one frame")
    a = ap.parse_args()
    w, h = map(int, a.size.lower().split("x"))
    pic = load_yuv(a.yuv, w, h, a.bd)
    ev = read_trace(a.trace)
    if not ev:
        sys.exit("empty trace")
    total = max(e[0] for e in ev)
    n_frames = int(a.seconds * a.fps) + a.fps          # plus one second of the finished picture
    sc = a.scale
    hud = 40 * sc + 12                             # three HUD lines: title, counters, legend
    W, H = w * sc, h * sc + hud
    H += H % 2; W += W % 2
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 9 * sc)
    font_b = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 9 * sc)

    decoded = np.zeros((h, w), dtype=bool)
    level = np.zeros(h, dtype=np.uint8)               # per luma row: highest filter stage that has passed
    bg = np.full((h, w, 3), 22, dtype=np.uint8)
    bg[::64, :, :] = 34; bg[:, ::64, :] = 34            # faint superblock grid
    tints = np.zeros((5, 3), dtype=np.float32); amt = np.zeros(5, dtype=np.float32)
    for k, (c, f) in TINT.items():
        if c is not None:
            tints[k] = c; amt[k] = f

    ff = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                           "-r", str(a.fps), "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
                           "-pix_fmt", "yuv420p", "-movflags", "faststart", a.out], stdin=subprocess.PIPE)
    i = 0
    counts = {k: 0 for k in "TF"}
    fronts = {s: 0 for s in "LCSR"}
    blocks = 0
    for fi in range(n_frames):
        cyc = min(total, int(total * fi / (a.seconds * a.fps)))
        while i < len(ev) and ev[i][0] <= cyc:
            e = ev[i]; i += 1
            if e[1] == "T":
                _, _, plane, x, y, bw, bh = e
                if plane == 0:
                    decoded[y:y + bh, x:x + bw] = True; blocks += 1
            elif e[1] == "F":
                _, _, st, y0, y1 = e
                lv = STAGES[st]
                level[y0:y1 + 1] = np.maximum(level[y0:y1 + 1], lv)
                fronts[st] = y1 + 1
        # composite
        lv2 = level[:, None]                                   # (h,1)
        t = tints[level][:, None, :]                            # (h,1,3)
        f = amt[level][:, None, None]                           # (h,1,1)
        out = pic.astype(np.float32) * (1 - f) + t * f
        out = np.where(decoded[:, :, None], out, bg.astype(np.float32)).astype(np.uint8)
        frame = np.repeat(np.repeat(out, sc, 0), sc, 1) if sc > 1 else out
        img = Image.new("RGB", (W, H), (16, 16, 18))
        img.paste(Image.fromarray(frame), (0, 0))
        d = ImageDraw.Draw(img)
        # front markers at the right edge
        for st, col in (("L", TINT[1][0]), ("C", TINT[2][0]), ("R", (255, 255, 255))):
            yy = min(h, fronts[st]) * sc
            if fronts[st]:
                d.line([(W - 6 * sc, yy), (W, yy)], fill=col, width=sc)
        # HUD
        y0 = h * sc + 6
        d.text((8, y0), a.title, font=font_b, fill=(235, 235, 235))
        pct = 100.0 * cyc / total
        d.text((8, y0 + 12 * sc), f"cycle {cyc:>9,d} / {total:,d}  ({pct:5.1f} %)   blocks {blocks:,d}", font=font, fill=(200, 200, 200))
        x = 8
        yl = y0 + 25 * sc
        for k in (0, 1, 2, 4):
            lab = LABEL[k]
            col = TINT[k][0] or (255, 255, 255)
            d.rectangle([x, yl + 2, x + 4 * sc, yl + 2 + 8 * sc], fill=col)
            x += 6 * sc
            d.text((x, yl), lab, font=font, fill=(200, 200, 200))
            x += d.textlength(lab, font=font) + 10 * sc
        ff.stdin.write(np.asarray(img).tobytes())
    ff.stdin.close(); ff.wait()
    print(f"{a.out}: {n_frames} frames, {blocks} luma blocks, {total:,d} cycles over {a.seconds}s")


if __name__ == "__main__":
    main()
