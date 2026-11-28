#!/usr/bin/env python3
"""bridge_patterns.py <outdir> (K60-108): the data check of rf_ddr_bridge on the twin. Player B (XDMA's place) writes two
regions, each beat carrying its own address, then reads one of them back while player A (reference fetch's place)
reads rows of the other through the bridge; both check every beat (+CHECK=1 +CHECKB=1). Pattern name: chk."""
import os, sys
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
A, C, N = 0x0400_0000, 0x0600_0000, 256                      # two regions of N x 1 KB
with open(os.path.join(out, "chk.writesB"), "w") as f:
    for base in (A, C):
        for i in range(N):
            f.write("0 %x 15\n" % (base + i * 1024))
with open(os.path.join(out, "chk.readsB"), "w") as f:        # after the writes: 16-beat and 3-beat reads of region C
    for i in range(N):
        f.write("0 %d %x 15\n" % (25000 if i == 0 else 3, C + i * 1024))
    for i in range(400):
        f.write("0 0 %x 2\n" % (C + (i * 448) % (N * 1024 - 256)))
with open(os.path.join(out, "chk.reads"), "w") as f:         # reference fetch: rows of 1, 2 and 4 beats of region A, two streams
    for i in range(1500):
        f.write("%d %d %x %d\n" % (i & 1, 8100 if i == 0 else (0 if i % 37 else 9), A + (i * 3840 + (i & 1) * 77 * 64) % (N * 1024 - 512) & ~63, (0, 1, 1, 3)[i % 4]))
print("chk written")
