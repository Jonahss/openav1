#!/usr/bin/env python3
"""Header-only scan for stage 2c.1 candidates: streams with a scaled reference (7.11.3.3 is_scaled on any inter
frame) and no global warp beyond TRANSLATION (allow_warped_motion is set on most Argon frames whether or not a block uses
LOCALWARP: join with tools/inter_scan.py's block-level counts for that).
Usage: tools/scaled_scan.py stream.obu [...]   -> one line per stream: scaled-frame count, total inter frames, verdict"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import obu_parser as op   # noqa: E402

for path in sys.argv[1:]:
    try:
        d = op.parse_file(path)
    except Exception as e:   # noqa: BLE001
        print(f"{Path(path).name}: ERROR {e}")
        continue
    scaled = inter = 0
    warp = False
    for h in d.frame_headers:
        if h.show_existing_frame or h.FrameIsIntra:
            continue
        inter += 1
        gm = getattr(h, "GmType", None) or getattr(h, "gm_type", None)
        if gm and any(t > 1 for t in gm):
            warp = True
        for i in range(1, 8):
            if h.RefUpscaledWidths[i] and (h.RefUpscaledWidths[i] != h.UpscaledWidth or h.RefFrameHeights[i] != h.FrameHeight):
                scaled += 1
                break
    verdict = "2c1-clean" if scaled and not warp else ("scaled+warp" if scaled else "unscaled")
    print(f"{Path(path).name}: scaled_frames {scaled} inter_frames {inter} {verdict}")
