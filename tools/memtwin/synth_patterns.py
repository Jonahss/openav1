#!/usr/bin/env python3
"""synth_patterns.py <outdir> (K60-110): synthetic traffic for the memory twin, to see what the memory path does with
the kinds of reads reference fetch makes. Each pattern is <name>.reads (and maybe <name>.writes) for twin_player.
A read is a picture row: one burst of 2 beats (64 bytes each), the next row one stride further.

  one        one stream, 2,000 rows at stride 3840 (the 4K luma stride: every row in the same DDR4 bank group)
  one1       the same with single beats
  st<n>      one stream at stride n = 3904, 3968, 4032, 4160 (3904 and 4160 rotate through the four bank groups)
  strip256, strip128   rows 256 / 128 bytes apart (a picture stored as vertical strips)
  patch3840, patch256  the same 71-row patch walked 28 times
  two        two streams taking turns burst by burst, 64 MB apart (the luma and chroma lanes), strides 3840 / 1920
  two3904    the same at strides 3904 / 1984
  twoblk     the two streams in turns of 71 rows
  seq16      one sequential read of the same 4,000 beats in 16-beat bursts
  one_w, two_w   `one` / `two` beside a stream of 16-beat writes
  short      200 rows of `one` (a quick run)
"""
import os, sys
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
ROWS = 2000
A, B, W = 0x0400_0000, 0x0800_0000, 0x0C00_0000

def reads(name, seq):
    with open(os.path.join(out, name + ".reads"), "w") as f:
        for stream, addr, ln in seq:
            f.write("%d 0 %x %d\n" % (stream, addr & ~63, ln))

def writes(name, n):
    with open(os.path.join(out, name + ".writes"), "w") as f:
        for i in range(n):
            f.write("0 %x 15\n" % (W + i * 1024))

one = [(0, A + r * 3840 + 100, 1) for r in range(ROWS)]
two = [x for r in range(ROWS // 2) for x in ((0, A + r * 3840 + 100, 1), (1, B + r * 1920 + 36, 1))]
blk = []
for p in range(ROWS // 142):
    blk += [(0, A + (p * 71 + r) * 3840 + 100, 1) for r in range(71)]
    blk += [(1, B + (p * 71 + r) * 1920 + 36, 1) for r in range(71)]
reads("one", one); reads("short", one[:200]); reads("two", two); reads("twoblk", blk)
reads("one1", [(0, A + r * 3840, 0) for r in range(ROWS)])
for st in (3904, 3968, 4032, 4160):
    reads("st%d" % st, [(0, A + r * st + 100, 1) for r in range(ROWS)])
reads("two3904", [x for r in range(ROWS // 2) for x in ((0, A + r * 3904 + 100, 1), (1, B + r * 1984 + 36, 1))])
reads("strip256", [(0, A + r * 256 + 100, 1) for r in range(ROWS)])
reads("strip128", [(0, A + r * 128 + 36, 1) for r in range(ROWS)])
reads("patch3840", [(0, A + r * 3840 + 100 + 64 * (p % 4), 1) for p in range(28) for r in range(71)])
reads("patch256", [(0, A + r * 256 + 100, 1) for p in range(28) for r in range(71)])
reads("seq16", [(0, A + i * 1024, 15) for i in range(250)])
reads("one_w", one); writes("one_w", 600)
reads("two_w", two); writes("two_w", 600)
print("patterns in", out)
