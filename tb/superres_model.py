"""Super-resolution upscaling (spec 7.16): horizontal 8-tap resampling of a plane array from FrameWidth to
UpscaledWidth, applied to both the deblocked picture and the CDEF picture before loop restoration.
Spec-literal (Upscale_Filter, SUPERRES_* constants from the spec tables)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import av1_tables as T      # noqa: E402

SUPERRES_SCALE_BITS = T.CONST["SUPERRES_SCALE_BITS"]        # 14
SUPERRES_EXTRA_BITS = T.CONST["SUPERRES_EXTRA_BITS"]        # 8
SUPERRES_SCALE_MASK = T.CONST["SUPERRES_SCALE_MASK"]        # (1 << 14) - 1
SUPERRES_FILTER_TAPS = T.CONST["SUPERRES_FILTER_TAPS"]      # 8
SUPERRES_FILTER_OFFSET = T.CONST["SUPERRES_FILTER_OFFSET"]  # 3
FILTER_BITS = T.CONST["FILTER_BITS"]                        # 7
MI_SIZE = 4


def round2(x, n):
    return x if n == 0 else (x + (1 << (n - 1))) >> n


def _plane_params(hdr, plane):
    subX = hdr.subsampling_x if plane > 0 else 0
    subY = hdr.subsampling_y if plane > 0 else 0
    downscaledPlaneW = round2(hdr.FrameWidth, subX)
    upscaledPlaneW = round2(hdr.UpscaledWidth, subX)
    planeH = round2(hdr.FrameHeight, subY)
    stepX = ((downscaledPlaneW << SUPERRES_SCALE_BITS) + (upscaledPlaneW // 2)) // upscaledPlaneW
    err = (upscaledPlaneW * stepX) - (downscaledPlaneW << SUPERRES_SCALE_BITS)
    # the spec's divisions truncate towards zero (C semantics); both numerators can be negative
    num = -((upscaledPlaneW - downscaledPlaneW) << (SUPERRES_SCALE_BITS - 1)) + upscaledPlaneW // 2
    q = abs(num) // upscaledPlaneW
    q = -q if num < 0 else q
    initialSubpelX = q + (1 << (SUPERRES_EXTRA_BITS - 1)) - err // 2 if err >= 0 else q + (1 << (SUPERRES_EXTRA_BITS - 1)) + (-err) // 2
    initialSubpelX &= SUPERRES_SCALE_MASK
    maxX = (hdr.MiCols >> subX) * MI_SIZE - 1
    return upscaledPlaneW, planeH, stepX, initialSubpelX, maxX


def upscale_row(hdr, plane, y, srow):
    """One line of a plane through 7.16 (the upscale works line by line): the line as upscale() would leave it,
    UpscaledWidth wide or wider; a line below the picture is only padded. For the streaming filter chain."""
    upscaledPlaneW, planeH, stepX, initialSubpelX, maxX = _plane_params(hdr, plane)
    W_out = max(upscaledPlaneW, len(srow))
    drow = list(srow[:W_out]) + [0] * (W_out - len(srow[:W_out]))
    if y >= planeH:
        return drow
    mx = (1 << hdr.BitDepth) - 1
    for x in range(upscaledPlaneW):
        srcX = -(1 << SUPERRES_SCALE_BITS) + initialSubpelX + x * stepX
        srcXPx = srcX >> SUPERRES_SCALE_BITS
        srcXSubpel = (srcX & SUPERRES_SCALE_MASK) >> SUPERRES_EXTRA_BITS
        taps = T.Upscale_Filter[srcXSubpel]
        s = 0
        for k in range(SUPERRES_FILTER_TAPS):
            sampleX = srcXPx + (k - SUPERRES_FILTER_OFFSET)
            sampleX = 0 if sampleX < 0 else maxX if sampleX > maxX else sampleX
            s += srow[sampleX] * taps[k]
        v = round2(s, FILTER_BITS)
        drow[x] = 0 if v < 0 else mx if v > mx else v
    return drow


def upscale(hdr, planes):
    """7.16: returns new plane arrays of width Round2(UpscaledWidth, subX) (rows beyond the picture are kept as
    they are so callers can keep addressing the same margins). No-op copy when use_superres is 0."""
    if not hdr.use_superres:
        return planes
    return [[upscale_row(hdr, plane, y, srow) for y, srow in enumerate(planes[plane])] for plane in range(hdr.NumPlanes)]
