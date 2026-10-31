#!/usr/bin/env python3
"""peek_state.py <mv_base_hex> <mi_cols> <mi_rows> [--xdma N] — read a frame's saved motion field (the MV sideband the
card wrote, docs/frame-state.md) back out of the U200's DDR4 and draw it: one character per 8x8 unit, the reference the
unit's motion vector points at (. = none, 1 LAST .. 7 ALTREF), then a few of the vectors. The addresses are the ones
decode.py --card-state prints after each frame."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tb"))
from xdma import XdmaStage                                   # noqa: E402

base = int(sys.argv[1], 0); mi_cols = int(sys.argv[2]); mi_rows = int(sys.argv[3])
card = int(sys.argv[sys.argv.index("--xdma") + 1]) if "--xdma" in sys.argv else 0
w8, h8 = mi_cols >> 1, mi_rows >> 1
rowb = (w8 * 4 + 7) & ~7
st = XdmaStage(card, 0x1000_0000, 256 << 20)
data = st.read(base - 0x1000_0000, rowb * h8)
print(f"saved motion field at 0x{base:x}: {w8} x {h8} units of 8x8 pixels, {rowb} bytes a line, read back over PCIe")
names = ".1234567"
sample = []
for y8 in range(h8):
    row = []
    for x8 in range(w8):
        w = int.from_bytes(data[y8 * rowb + 4 * x8: y8 * rowb + 4 * x8 + 4], "little")
        ref = w & 7
        row.append(names[ref])
        if ref and len(sample) < 6 and (y8 * w8 + x8) % 97 == 0:
            col = (w >> 3) & 0x1FFF; r_ = (w >> 16) & 0x1FFF
            col -= 0x2000 if col & 0x1000 else 0; r_ -= 0x2000 if r_ & 0x1000 else 0
            sample.append((y8, x8, ref, r_, col))
    print("".join(row))
print("a few of the vectors (y8, x8, ref, mv row, mv col in 1/8 samples):", sample)
