#!/usr/bin/env python3
"""show_stream.py <stream.ivf|.obu> [n] — what the AV1 bitstream says before any pixel is decoded: the sequence
header's picture size and profile, then one line per coded frame (type, size, order hint, which reference slots it
reads and which it refreshes, its CDF/segment-map source, tiles). This is the host's parser (tb/obu_parser.py), written
from the specification; the card gets these headers as registers (tb/host_regs.py)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tb"))
import obu_parser as op                                      # noqa: E402

path = sys.argv[1]; n = int(sys.argv[2]) if len(sys.argv) > 2 else 8
d = op.Decoder()
raw = open(path, "rb").read()
d.feed_annexb(raw) if path.endswith(".obu") else d.feed_ivf(raw)
hs = [h for h in d.frame_headers if not h.show_existing_frame]
th0 = d.tiles[d.frames[0][0]][0]
print(f"{os.path.basename(path)}: {len(d.frames)} coded frames, {len(d.tiles)} tiles; {th0.FrameWidth}x{th0.FrameHeight}, "
      f"{th0.BitDepth}-bit, {'4:2:0' if th0.subsampling_x else '4:4:4'}, superblocks {'128' if getattr(th0, 'use_128x128_superblock', 0) else '64'}")
names = {0: "KEY", 1: "INTER", 2: "INTRA_ONLY", 3: "SWITCH"}
for fi in range(min(n, len(d.frames))):
    h = hs[fi]; th = d.tiles[d.frames[fi][0]][0]
    refs = list(h.ref_frame_idx) if not h.FrameIsIntra else []
    prim = "defaults" if h.primary_ref_frame == 7 else f"slot {h.ref_frame_idx[h.primary_ref_frame]}"
    print(f"frame {fi:3d}: {names.get(h.frame_type, h.frame_type):10s} {th.FrameWidth}x{th.FrameHeight}  order_hint {h.order_hint:3d}  "
          f"reads slots {refs}  refreshes 0x{h.refresh_frame_flags:02x}  CDFs/segment map from {prim}  "
          f"{len(d.frames[fi])} tile(s){'  uses the motion field' if getattr(h, 'use_ref_frame_mvs', 0) else ''}")
