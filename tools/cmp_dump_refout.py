#!/usr/bin/env python3
"""Compare RTL frame dumps (tb/test_dec_top.py with TS_DUMP) directly against dav1d's decoded pictures
(refout/<stream>.yuv from tools/regress.sh: planar, frames concatenated, 8-bit bytes or 16-bit LE).

Usage: tools/cmp_dump_refout.py <dump-dir> [refout-dir]
For every <stream>.ivf_frame_<n>.rtl.yuv in the dump dir, the matching frame slice of refout/<stream>.yuv is
compared byte for byte. Exit 0 when every frame matches.
"""
import os
import re
import sys
from pathlib import Path

dump = Path(sys.argv[1])
refdir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parent.parent / "refout"
rc = 0
n = 0
for f in sorted(dump.glob("*.rtl.yuv")):
    m = re.match(r"(.+)\.ivf_frame_(\d+)\.rtl\.yuv$", f.name)
    if not m:
        continue
    stream, fi = m.group(1), int(m.group(2))
    ref = refdir / f"{stream}.yuv"
    if not ref.exists():
        print(f"SKIP {f.name}: no {ref}")
        continue
    data = f.read_bytes()
    size = len(data)
    with open(ref, "rb") as fh:
        fh.seek(size * fi)
        want = fh.read(size)
    n += 1
    if data == want:
        print(f"MATCH {stream} frame {fi} ({size} bytes == dav1d)")
    else:
        bad = sum(1 for a, b in zip(data, want) if a != b) + abs(len(data) - len(want))
        print(f"DIFF  {stream} frame {fi}: {bad} differing bytes vs dav1d")
        rc = 1
print(f"{n} frames compared, {'all identical to dav1d' if rc == 0 else 'MISMATCHES'}")
sys.exit(rc)
