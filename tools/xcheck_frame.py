#!/usr/bin/env python3
"""End-to-end frame check: decode every frame of a trace with the Python models (tile syntax ->
reconstruction -> loop filter [-> CDEF -> LR when modelled]) and compare the visible picture with
dav1d's raw output (refout/<name>.yuv, planar 4:2:0/4:2:2/4:4:4, 8-bit bytes or 16-bit LE).

Usage: tools/xcheck_frame.py <trace.txt> <ref.yuv> [--stage recon|lf|cdef|lr] [-v]
The stage selects how far post-processing is applied before comparing (default: lf).
"""
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm     # noqa: E402
import recon_model as rm    # noqa: E402
import lf_model as lfm      # noqa: E402
import cdef_model as cdm    # noqa: E402
import lr_model as lrm      # noqa: E402
import av1_tables as T      # noqa: E402


class FrameState:
    """Per-4x4 arrays of a whole frame, merged from its tile decoders (each only fills its own region)."""

    def __init__(self, hdr, decs):
        self.h = hdr
        R, C = hdr.MiRows, hdr.MiCols
        self.Skips = [[0] * C for _ in range(R)]
        self.MiSizes = [[0] * C for _ in range(R)]
        self.YModes = [[0] * C for _ in range(R)]
        self.SegmentIds = [[0] * C for _ in range(R)]
        self.DeltaLFs = [[None] * C for _ in range(R)]
        self.LoopfilterTxSizes = [[row[:] for row in d] for d in decs[0].LoopfilterTxSizes]
        self.cdef_idx = {}
        self.LrType, self.LrWiener, self.LrSgrSet, self.LrSgrXqd = {}, {}, {}, {}
        for d in decs:
            self.LrType.update(d.LrType); self.LrWiener.update(d.LrWiener)
            self.LrSgrSet.update(d.LrSgrSet); self.LrSgrXqd.update(d.LrSgrXqd)
            th = d.h
            for r in range(th.MiRowStart, min(th.MiRowEnd, R)):
                for c in range(th.MiColStart, min(th.MiColEnd, C)):
                    self.Skips[r][c] = d.Skips[r][c]
                    self.MiSizes[r][c] = d.MiSizes[r][c]
                    self.YModes[r][c] = d.YModes[r][c]
                    self.SegmentIds[r][c] = d.SegmentIds[r][c]
                    self.DeltaLFs[r][c] = d.DeltaLFs[r][c]
            for plane in range(hdr.NumPlanes):
                subX = hdr.subsampling_x if plane > 0 else 0
                subY = hdr.subsampling_y if plane > 0 else 0
                src, dst = d.LoopfilterTxSizes[plane], self.LoopfilterTxSizes[plane]
                for r in range(th.MiRowStart >> subY, min((th.MiRowEnd + subY) >> subY, len(dst))):
                    for c in range(th.MiColStart >> subX, min((th.MiColEnd + subX) >> subX, len(dst[0]))):
                        dst[r][c] = src[r][c]
            self.cdef_idx.update(d.cdef_idx)

    def get_plane_residual_size(self, subsize, plane):
        subx = self.h.subsampling_x if plane > 0 else 0
        suby = self.h.subsampling_y if plane > 0 else 0
        return T.Subsampled_Size[subsize][subx][suby]


def read_ref_frames(path, hdr):
    bd = hdr.BitDepth
    W, H = hdr.UpscaledWidth, hdr.FrameHeight
    sx, sy = hdr.subsampling_x, hdr.subsampling_y
    cw, ch = (W + sx) >> sx, (H + sy) >> sy
    bps = 1 if bd == 8 else 2
    sizes = [(W, H)] + ([(cw, ch), (cw, ch)] if hdr.NumPlanes == 3 else [])
    frame_bytes = sum(w * h for w, h in sizes) * bps
    data = open(path, "rb").read()
    frames = []
    off = 0
    while off + frame_bytes <= len(data):
        planes = []
        for w, h in sizes:
            n = w * h
            if bps == 1:
                vals = list(data[off:off + n])
            else:
                vals = list(struct.unpack("<%dH" % n, data[off:off + 2 * n]))
            planes.append([vals[r * w:(r + 1) * w] for r in range(h)])
            off += n * bps
        frames.append(planes)
    return frames, sizes


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    stage = sys.argv[sys.argv.index("--stage") + 1] if "--stage" in sys.argv else "lf"
    trace, ref = args[0], args[1]
    tiles = []
    cur = None
    with open(trace) as fh:
        for line in fh:
            if not line or line[0] in "#SUPQRC":
                continue
            f = line.split()
            if f[0] == "T":
                cur = dict(hdr=None, data=bytes.fromhex(f[3]) if len(f) > 3 else b"")
                tiles.append(cur)
            elif f[0] == "H":
                cur["hdr"] = [int(x) for x in f[1:]]
    # group tiles into frames
    frames = []
    for t in tiles:
        hdr = tm.FrameHeader(t["hdr"])
        if hdr.tile_row == 0 and hdr.tile_col == 0:
            frames.append([])
        frames[-1].append((hdr, t["data"]))
    hdr0 = frames[0][0][0]
    refs, sizes = read_ref_frames(ref, hdr0)
    if len(refs) < len(frames):
        print(f"reference has {len(refs)} frames, trace {len(frames)}")
    total_bad = 0
    for fi, tl in enumerate(frames):
        frame = {}
        decs = []
        for hdr, data in tl:
            dec = rm.FrameRecon(hdr, data, frame)
            dec.decode_tile()
            decs.append(dec)
        hdr = tl[0][0]
        planes = frame["planes"]
        state = FrameState(hdr, decs)
        if stage in ("lf", "cdef", "lr"):
            lfm.LoopFilter(hdr, state, planes).apply()
        deblocked = planes
        if stage in ("cdef", "lr") and hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:
            planes = cdm.Cdef(hdr, state, planes).apply()
        if stage == "lr":
            planes = lrm.LoopRestoration(hdr, state, deblocked, planes).apply()
        if fi >= len(refs):
            break
        for p, (w, h) in enumerate(sizes):
            got = planes[p]
            exp = refs[fi][p]
            bad = 0
            first = None
            for y in range(h):
                gr, er = got[y], exp[y]
                for x in range(w):
                    if gr[x] != er[x]:
                        bad += 1
                        if first is None:
                            first = (x, y, gr[x], er[x])
            total_bad += bad
            tag = "OK " if bad == 0 else "BAD"
            print(f"frame {fi} plane {p} {w}x{h}: {tag} {bad} differing samples" + (f", first at x={first[0]} y={first[1]} model={first[2]} dav1d={first[3]}" if first else ""))
    print("OK" if total_bad == 0 else f"{total_bad} differing samples")
    sys.exit(1 if total_bad else 0)


if __name__ == "__main__":
    main()
