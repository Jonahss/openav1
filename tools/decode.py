#!/usr/bin/env python3
"""Standalone Python AV1 intra decoder: stream in, raw YUV out. No dav1d involved.

  tools/decode.py <stream.ivf|.obu> -o out.yuv [--frames N]

Pipeline: obu_parser (headers, tiles) -> tile_model + recon_model (tile syntax, prediction, dequant,
inverse transforms, reconstruction) -> lf_model -> cdef_model -> lr_model -> cropped planes.
Output is planar, 8-bit as bytes, >8-bit as 16-bit little-endian (same as `dav1d -o x.yuv`).
"""
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import obu_parser as op     # noqa: E402
import recon_model as rm    # noqa: E402
import lf_model as lfm      # noqa: E402
import cdef_model as cdm    # noqa: E402
import lr_model as lrm      # noqa: E402
from xcheck_frame import FrameState  # noqa: E402


def decode_frame(tiles):
    """tiles: list of (hdr, bytes) for one frame. Returns (hdr, planes) after all in-loop filters."""
    frame = {}
    decs = []
    for hdr, data in tiles:
        dec = rm.FrameRecon(hdr, data, frame)
        dec.decode_tile()
        decs.append(dec)
    hdr = tiles[0][0]
    planes = frame["planes"]
    state = FrameState(hdr, decs)
    lfm.LoopFilter(hdr, state, planes).apply()
    deblocked = planes
    if hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:
        planes = cdm.Cdef(hdr, state, planes).apply()
    planes = lrm.LoopRestoration(hdr, state, deblocked, planes).apply()
    return hdr, planes


def write_frame(fh, hdr, planes):
    W, H = hdr.UpscaledWidth, hdr.FrameHeight
    sx, sy = hdr.subsampling_x, hdr.subsampling_y
    sizes = [(W, H)] + ([((W + sx) >> sx, (H + sy) >> sy)] * 2 if hdr.NumPlanes == 3 else [])
    for p, (w, h) in enumerate(sizes):
        for y in range(h):
            row = planes[p][y][:w]
            fh.write(bytes(row) if hdr.BitDepth == 8 else struct.pack("<%dH" % w, *row))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    out = sys.argv[sys.argv.index("-o") + 1] if "-o" in sys.argv else None
    max_frames = int(sys.argv[sys.argv.index("--frames") + 1]) if "--frames" in sys.argv else None
    d = op.parse_file(args[0])
    n = 0
    with (open(out, "wb") if out else open("/dev/null", "wb")) as fh:
        for fidx in d.frames:
            tiles = [d.tiles[i] for i in fidx]
            hdr = tiles[0][0]
            if not hdr.show_frame:
                continue            # hidden frames are not output (no show_existing_frame support yet)
            hdr, planes = decode_frame(tiles)
            write_frame(fh, hdr, planes)
            n += 1
            print(f"frame {n}: {hdr.UpscaledWidth}x{hdr.FrameHeight} {hdr.BitDepth}-bit, {len(tiles)} tile(s)", file=sys.stderr)
            if max_frames and n >= max_frames:
                break
    print(f"decoded {n} frames", file=sys.stderr)


if __name__ == "__main__":
    main()
