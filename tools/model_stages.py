#!/usr/bin/env python3
"""Dump the Python model's intermediate pictures of every frame of a trace: reconstruction, deblocked, CDEF,
the two super-resolution inputs of loop restoration (upscaled deblocked / upscaled CDEF) and the restored
frame, as raw planar yuv (visible size, 8-bit bytes or 16-bit LE) plus a .txt with the geometry.
Usage: tools/model_stages.py <trace.txt> <out-dir>
"""
import copy
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import tile_model as tm         # noqa: E402
import recon_model as rm        # noqa: E402
import lf_model as lfm          # noqa: E402
import cdef_model as cdm        # noqa: E402
import lr_model as lrm          # noqa: E402
import superres_model as srm    # noqa: E402
import xcheck_frame as xf       # noqa: E402


def dump(path, hdr, planes, width):
    bps = 1 if hdr.BitDepth == 8 else 2
    with open(path, "wb") as fh:
        for p in range(hdr.NumPlanes):
            sx = hdr.subsampling_x if p else 0
            sy = hdr.subsampling_y if p else 0
            w = (width + sx) >> sx
            h = (hdr.FrameHeight + sy) >> sy
            for y in range(h):
                row = planes[p][y][:w]
                fh.write(bytes(row) if bps == 1 else struct.pack("<%dH" % w, *row))


def main():
    trace, out = sys.argv[1], Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
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
    frames = []
    for t in tiles:
        hdr = tm.FrameHeader(t["hdr"])
        if hdr.tile_row == 0 and hdr.tile_col == 0:
            frames.append([])
        frames[-1].append((hdr, t["data"]))
    for fi, tl in enumerate(frames):
        frame = {}
        decs = []
        for hdr, data in tl:
            dec = rm.FrameRecon(hdr, data, frame)
            dec.decode_tile()
            decs.append(dec)
        hdr = tl[0][0]
        planes = frame["planes"]
        W, U = hdr.FrameWidth, hdr.UpscaledWidth
        tag = out / f"frame{fi}"
        dump(f"{tag}_recon.yuv", hdr, planes, W)
        state = xf.FrameState(hdr, decs)
        lfm.LoopFilter(hdr, state, planes).apply()
        dump(f"{tag}_deblocked.yuv", hdr, planes, W)
        deblocked = planes
        cdef = planes
        if hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:
            cdef = cdm.Cdef(hdr, state, planes).apply()
            dump(f"{tag}_cdef.yuv", hdr, cdef, W)
        if hdr.use_superres:
            deblocked = srm.upscale(hdr, deblocked)
            cdef = srm.upscale(hdr, cdef) if cdef is not planes else deblocked
            dump(f"{tag}_deblocked_up.yuv", hdr, deblocked, U)
            dump(f"{tag}_cdef_up.yuv", hdr, cdef, U)
        if any(t != 0 for t in hdr.FrameRestorationType[:hdr.NumPlanes]):
            lr = lrm.LoopRestoration(hdr, state, deblocked, cdef).apply()
            dump(f"{tag}_lr.yuv", hdr, lr, U)
        (out / f"frame{fi}.txt").write_text(f"{W} {U} {hdr.FrameHeight} {hdr.BitDepth} {hdr.subsampling_x} {hdr.subsampling_y} {hdr.NumPlanes}\n")
        print(f"frame {fi}: {W}->{U} x {hdr.FrameHeight} bd{hdr.BitDepth} cdef={hdr.enable_cdef} lr={hdr.FrameRestorationType[:hdr.NumPlanes]} superres={hdr.use_superres}")


if __name__ == "__main__":
    main()
