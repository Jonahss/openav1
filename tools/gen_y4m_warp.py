#!/usr/bin/env python3
"""A multi-frame y4m whose frames are the source under a slowly changing affine transform (rotation + zoom + drift,
bilinear resampling) plus a small counter-moving object: content for which an encoder picks warped motion (LOCALWARP
blocks and non-translational global motion). Usage: tools/gen_y4m_warp.py in.y4m out.y4m [frames] [seed]"""
import math
import random
import sys

import numpy as np


def affine_sample(plane, M):
    """plane under the inverse-mapped affine M (dst -> src), bilinear, edge-clamped."""
    H, W = plane.shape
    ys, xs = np.mgrid[0:H, 0:W].astype(np.float64)
    sx = M[0, 0] * xs + M[0, 1] * ys + M[0, 2]
    sy = M[1, 0] * xs + M[1, 1] * ys + M[1, 2]
    x0 = np.floor(sx); y0 = np.floor(sy)
    fx = sx - x0; fy = sy - y0
    x0 = x0.astype(int); y0 = y0.astype(int)
    x1 = np.clip(x0 + 1, 0, W - 1); y1 = np.clip(y0 + 1, 0, H - 1)
    x0 = np.clip(x0, 0, W - 1); y0 = np.clip(y0, 0, H - 1)
    p = plane.astype(np.float64)
    v = (p[y0, x0] * (1 - fx) * (1 - fy) + p[y0, x1] * fx * (1 - fy) + p[y1, x0] * (1 - fx) * fy + p[y1, x1] * fx * fy)
    return np.clip(np.rint(v), 0, 255).astype(np.uint8)


def main():
    src, dst = sys.argv[1], sys.argv[2]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    rng = random.Random(int(sys.argv[4]) if len(sys.argv) > 4 else 1)
    data = open(src, "rb").read()
    hdr_end = data.index(b"\n")
    hdr = data[:hdr_end].decode()
    fields = {t[0]: t[1:] for t in hdr.split()[1:]}
    W, H = int(fields["W"]), int(fields["H"])
    assert fields.get("C", "420jpeg").startswith("420"), "4:2:0 sources only"
    frame0 = data.index(b"FRAME\n", hdr_end) + 6
    ysz, csz = W * H, (W // 2) * (H // 2)
    y = np.frombuffer(data[frame0:frame0 + ysz], np.uint8).reshape(H, W)
    u = np.frombuffer(data[frame0 + ysz:frame0 + ysz + csz], np.uint8).reshape(H // 2, W // 2)
    v = np.frombuffer(data[frame0 + ysz + csz:frame0 + ysz + 2 * csz], np.uint8).reshape(H // 2, W // 2)
    with open(dst, "wb") as out:
        out.write(data[:hdr_end + 1])
        ang, zoom, tx, ty = 0.0, 1.0, 0.0, 0.0
        for k in range(n):
            # dst -> src mapping about the centre: rotate by -ang, scale by 1/zoom, shift back by the drift
            c, s = math.cos(ang), math.sin(ang)
            planes = []
            for pl, (pw, ph, sc) in zip((y, u, v), ((W, H, 1.0), (W // 2, H // 2, 0.5), (W // 2, H // 2, 0.5))):
                cx, cy = pw / 2.0, ph / 2.0
                M = np.array([[c / zoom, s / zoom, cx - (c * cx + s * cy) / zoom - tx * sc],
                              [-s / zoom, c / zoom, cy - (-s * cx + c * cy) / zoom - ty * sc]])
                planes.append(affine_sample(pl, M))
            fy, fu, fv = planes
            # noise so the encoder cannot predict everything perfectly
            noise = np.zeros(fy.shape, dtype=int); noise[::7, ::5] = 1
            fy = np.clip(fy.astype(int) + noise * rng.randint(-2, 2), 0, 255).astype(np.uint8)
            out.write(b"FRAME\n" + fy.tobytes() + fu.tobytes() + fv.tobytes())
            ang += math.radians(rng.uniform(0.6, 1.4)) * (1 if k % 5 != 4 else -1)
            zoom *= rng.uniform(1.008, 1.02)
            tx += rng.uniform(-1.5, 1.5); ty += rng.uniform(-1.0, 1.0)
    print(f"{dst}: {n} frames {W}x{H}")


if __name__ == "__main__":
    main()
