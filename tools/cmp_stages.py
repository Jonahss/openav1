#!/usr/bin/env python3
"""Compare RTL frame-buffer dumps (tb/test_dec_top.py TS_DUMP: <stem>_frame<n>_fb{0,1,2}.yuv, upscaled visible
size) with the model's stage pictures (tools/model_stages.py: frame<n>_{deblocked_up|cdef_up|lr}.yuv or the
non-upscaled names when superres is off). Usage: tools/cmp_stages.py <rtl-dump-dir> <stem> <model-stage-dir>"""
import struct
import sys
from pathlib import Path

rtl_dir, stem, mdir = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
for txt in sorted(mdir.glob("frame*.txt")):
    fi = int(txt.stem[5:])
    W, U, H, bd, sx, sy, np_ = [int(v) for v in txt.read_text().split()]
    bps = 1 if bd == 8 else 2
    sizes = [(U, H)] + ([((U + sx) >> sx, (H + sy) >> sy)] * 2 if np_ == 3 else [])
    for fb, names in ((0, ("deblocked_up", "deblocked")), (1, ("cdef_up", "cdef")), (2, ("lr",))):
        rp = rtl_dir / f"{stem}_frame{fi}_fb{fb}.yuv"
        mp = next((mdir / f"frame{fi}_{n}.yuv" for n in names if (mdir / f"frame{fi}_{n}.yuv").exists()), None)
        if not rp.exists() or mp is None:
            print(f"frame {fi} fb{fb}: {'no RTL dump' if not rp.exists() else 'no model stage'}")
            continue
        r, m = rp.read_bytes(), mp.read_bytes()
        off = 0
        for p, (w, h) in enumerate(sizes):
            n = w * h * bps
            rv = r[off:off + n] if bps == 1 else struct.unpack("<%dH" % (w * h), r[off:off + n])
            mv = m[off:off + n] if bps == 1 else struct.unpack("<%dH" % (w * h), m[off:off + n])
            bad = [(i % w, i // w, rv[i], mv[i]) for i in range(min(len(rv), len(mv))) if rv[i] != mv[i]]
            print(f"frame {fi} fb{fb} ({mp.name}) plane {p} {w}x{h}: {'OK' if not bad else f'{len(bad)} differ, first {bad[:6]}'}")
            off += n
