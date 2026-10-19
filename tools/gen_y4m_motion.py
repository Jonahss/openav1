#!/usr/bin/env python3
"""A multi-frame y4m from a single-frame one: each frame is the source panned by a slow drift (sub-sample motion for the
encoder to find, so MVs, compound references and interpolation filters appear) with light noise.
Usage: tools/gen_y4m_motion.py in.y4m out.y4m [frames] [seed]"""
import random
import sys


def main():
    src, dst = sys.argv[1], sys.argv[2]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    rng = random.Random(int(sys.argv[4]) if len(sys.argv) > 4 else 1)
    with open(src, "rb") as fh:
        data = fh.read()
    hdr_end = data.index(b"\n")
    hdr = data[:hdr_end].decode()
    fields = dict(f[0] for f in [(t[0], t[1:]) for t in hdr.split()[1:]] if False) or {}
    for t in hdr.split()[1:]:
        fields[t[0]] = t[1:]
    W, H = int(fields["W"]), int(fields["H"])
    assert fields.get("C", "420jpeg").startswith("420"), "4:2:0 sources only"
    frame0 = data.index(b"FRAME\n", hdr_end) + 6
    ysz, csz = W * H, (W // 2) * (H // 2)
    y = data[frame0:frame0 + ysz]
    u = data[frame0 + ysz:frame0 + ysz + csz]
    v = data[frame0 + ysz + csz:frame0 + ysz + 2 * csz]

    def pan(plane, w, h, dx, dy):
        out = bytearray(w * h)
        for yy in range(h):
            sy = min(max(yy + dy, 0), h - 1)
            row = plane[sy * w:(sy + 1) * w]
            for xx in range(w):
                sx = min(max(xx + dx, 0), w - 1)
                out[yy * w + xx] = row[sx]
        return out

    with open(dst, "wb") as out:
        out.write(data[:hdr_end + 1])
        ox = oy = 0.0
        vx, vy = rng.uniform(-1.5, 1.5), rng.uniform(-1.0, 1.0)
        for k in range(n):
            dx, dy = int(round(ox)), int(round(oy))
            fy = pan(y, W, H, dx, dy)
            fu = pan(u, W // 2, H // 2, dx // 2, dy // 2)
            fv = pan(v, W // 2, H // 2, dx // 2, dy // 2)
            for p in (fy, fu, fv):
                for i in range(0, len(p), 7):
                    p[i] = min(255, max(0, p[i] + rng.randint(-2, 2)))
            out.write(b"FRAME\n" + bytes(fy) + bytes(fu) + bytes(fv))
            ox += vx; oy += vy
            vx += rng.uniform(-0.3, 0.3); vy += rng.uniform(-0.3, 0.3)
    print(f"{dst}: {n} frames {W}x{H}")


if __name__ == "__main__":
    main()
