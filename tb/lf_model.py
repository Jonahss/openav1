"""Spec-literal model of the AV1 deblocking loop filter (spec 7.14), applied in place to the planes of a
reconstructed frame (recon_model.FrameRecon holds the per-4x4 state it needs).

Intra frames only for now (RefFrames are all INTRA_FRAME, modeType is 0).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import av1_tables as T  # noqa: E402

MI_SIZE = 4
MAX_LOOP_FILTER = 63
INTRA_FRAME = 0
SEG_LVL_ALT_LF_Y_V = 1


def clip3(lo, hi, x):
    return lo if x < lo else hi if x > hi else x


def round2(x, n):
    return x if n == 0 else (x + (1 << (n - 1))) >> n


class LoopFilter:
    def __init__(self, hdr, dec, planes):
        """hdr: tile_model.FrameHeader (with loop filter fields); dec: a TileDecoder holding the frame's
        per-4x4 arrays (MiSizes, Skips, YModes, SegmentIds, DeltaLFs, LoopfilterTxSizes); planes: pixel arrays."""
        self.h = hdr
        self.d = dec
        self.planes = planes

    def apply(self):
        h = self.h
        for plane in range(h.NumPlanes):
            if plane == 0 or h.loop_filter_level[1 + plane]:
                for pss in range(2):
                    rowStep = 1 if plane == 0 else (1 << h.subsampling_y)
                    colStep = 1 if plane == 0 else (1 << h.subsampling_x)
                    for row in range(0, h.MiRows, rowStep):
                        for col in range(0, h.MiCols, colStep):
                            self.edge(plane, pss, row, col)

    # ---- 7.14.2 edge loop filter --------------------------------------------------------------------
    def edge(self, plane, pss, row, col):
        h = self.h
        d = self.d
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        dx, dy = (1, 0) if pss == 0 else (0, 1)
        x = col * MI_SIZE
        y = row * MI_SIZE
        row |= subY
        col |= subX
        if x >= h.FrameWidth or y >= h.FrameHeight:
            return
        if pss == 0 and x == 0:
            return
        if pss == 1 and y == 0:
            return
        xP = x >> subX
        yP = y >> subY
        prevRow = row - (dy << subY)
        prevCol = col - (dx << subX)
        MiSize = d.MiSizes[row][col]
        txSz = d.LoopfilterTxSizes[plane][row >> subY][col >> subX]
        planeSize = d.get_plane_residual_size(MiSize, plane)
        skip = d.Skips[row][col]
        isIntra = True
        prevTxSz = d.LoopfilterTxSizes[plane][prevRow >> subY][prevCol >> subX]
        if pss == 0:
            isBlockEdge = (xP % T.Block_Width[planeSize]) == 0
            isTxEdge = (xP % T.Tx_Width[txSz]) == 0
        else:
            isBlockEdge = (yP % T.Block_Height[planeSize]) == 0
            isTxEdge = (yP % T.Tx_Height[txSz]) == 0
        if not isTxEdge:
            applyFilter = 0
        elif isBlockEdge or not skip or isIntra:
            applyFilter = 1
        else:
            applyFilter = 0
        # 7.14.3 filter size
        if pss == 0:
            baseSize = min(T.Tx_Width[prevTxSz], T.Tx_Width[txSz])
        else:
            baseSize = min(T.Tx_Height[prevTxSz], T.Tx_Height[txSz])
        filterSize = min(16, baseSize) if plane == 0 else min(8, baseSize)
        lvl, limit, blimit, thresh = self.strength(row, col, plane, pss)
        if lvl == 0:
            lvl, limit, blimit, thresh = self.strength(prevRow, prevCol, plane, pss)
        for i in range(MI_SIZE):
            if applyFilter and lvl > 0:
                self.sample_filter(xP + dy * i, yP + dx * i, plane, limit, blimit, thresh, dx, dy, filterSize)

    # ---- 7.14.4 / 7.14.5 strength ------------------------------------------------------------------
    def strength(self, row, col, plane, pss):
        h = self.h
        d = self.d
        segment = d.SegmentIds[row][col]
        ref = INTRA_FRAME
        modeType = 0
        dlf = d.DeltaLFs[row][col] or [0, 0, 0, 0]
        if not h.delta_lf_multi:
            deltaLF = dlf[0]
        else:
            deltaLF = dlf[pss if plane == 0 else plane + 1]
        # 7.14.5
        i = pss if plane == 0 else plane + 1
        baseFilterLevel = clip3(0, MAX_LOOP_FILTER, deltaLF + h.loop_filter_level[i])
        lvlSeg = baseFilterLevel
        feature = SEG_LVL_ALT_LF_Y_V + i
        if h.segmentation_enabled and h.FeatureEnabled[segment][feature]:
            lvlSeg = clip3(0, MAX_LOOP_FILTER, h.FeatureData[segment][feature] + lvlSeg)
        if h.loop_filter_delta_enabled:
            nShift = lvlSeg >> 5
            if ref == INTRA_FRAME:
                lvlSeg = lvlSeg + (h.loop_filter_ref_deltas[INTRA_FRAME] << nShift)
            else:
                lvlSeg = lvlSeg + (h.loop_filter_ref_deltas[ref] << nShift) + (h.loop_filter_mode_deltas[modeType] << nShift)
            lvlSeg = clip3(0, MAX_LOOP_FILTER, lvlSeg)
        lvl = lvlSeg
        sharp = h.loop_filter_sharpness
        shift = 2 if sharp > 4 else 1 if sharp > 0 else 0
        if sharp > 0:
            limit = clip3(1, 9 - sharp, lvl >> shift)
        else:
            limit = max(1, lvl >> shift)
        blimit = 2 * (lvl + 2) + limit
        thresh = lvl >> 4
        return lvl, limit, blimit, thresh

    # ---- 7.14.6 sample filtering -------------------------------------------------------------------
    def sample_filter(self, x, y, plane, limit, blimit, thresh, dx, dy, filterSize):
        buf = self.planes[plane]
        bd = self.h.BitDepth

        def px(k):  # k >= 0: q_k ; k < 0: p_{-k-1}
            return buf[y + dy * k][x + dx * k]

        q0, q1, q2, q3 = px(0), px(1), px(2), px(3)
        p0, p1, p2, p3 = px(-1), px(-2), px(-3), px(-4)
        hevMask = 0
        threshBd = thresh << (bd - 8)
        hevMask |= abs(p1 - p0) > threshBd
        hevMask |= abs(q1 - q0) > threshBd
        if filterSize == 4:
            filterLen = 4
        elif plane != 0:
            filterLen = 6
        elif filterSize == 8:
            filterLen = 8
        else:
            filterLen = 16
        limitBd = limit << (bd - 8)
        blimitBd = blimit << (bd - 8)
        mask = 0
        mask |= abs(p1 - p0) > limitBd
        mask |= abs(q1 - q0) > limitBd
        mask |= abs(p0 - q0) * 2 + abs(p1 - q1) // 2 > blimitBd
        if filterLen >= 6:
            mask |= abs(p2 - p1) > limitBd
            mask |= abs(q2 - q1) > limitBd
        if filterLen >= 8:
            mask |= abs(p3 - p2) > limitBd
            mask |= abs(q3 - q2) > limitBd
        filterMask = (mask == 0)
        thresholdBd = 1 << (bd - 8)
        flatMask = flatMask2 = 0
        if filterSize >= 8:
            mask = 0
            mask |= abs(p1 - p0) > thresholdBd
            mask |= abs(q1 - q0) > thresholdBd
            mask |= abs(p2 - p0) > thresholdBd
            mask |= abs(q2 - q0) > thresholdBd
            if filterLen >= 8:
                mask |= abs(p3 - p0) > thresholdBd
                mask |= abs(q3 - q0) > thresholdBd
            flatMask = (mask == 0)
        if filterSize >= 16:
            q4, q5, q6 = px(4), px(5), px(6)
            p4, p5, p6 = px(-5), px(-6), px(-7)
            mask = 0
            mask |= abs(p6 - p0) > thresholdBd
            mask |= abs(q6 - q0) > thresholdBd
            mask |= abs(p5 - p0) > thresholdBd
            mask |= abs(q5 - q0) > thresholdBd
            mask |= abs(p4 - p0) > thresholdBd
            mask |= abs(q4 - q0) > thresholdBd
            flatMask2 = (mask == 0)
        if not filterMask:
            return
        if filterSize == 4 or not flatMask:
            self.narrow(hevMask, x, y, plane, dx, dy)
        elif filterSize == 8 or not flatMask2:
            self.wide(x, y, plane, dx, dy, 3)
        else:
            self.wide(x, y, plane, dx, dy, 4)

    def narrow(self, hevMask, x, y, plane, dx, dy):
        buf = self.planes[plane]
        bd = self.h.BitDepth
        lo, hi = -(1 << (bd - 1)), (1 << (bd - 1)) - 1

        def c4(v):
            return clip3(lo, hi, v)

        off = 0x80 << (bd - 8)
        q0 = buf[y][x]
        q1 = buf[y + dy][x + dx]
        p0 = buf[y - dy][x - dx]
        p1 = buf[y - dy * 2][x - dx * 2]
        ps1, ps0, qs0, qs1 = p1 - off, p0 - off, q0 - off, q1 - off
        filt = c4(ps1 - qs1) if hevMask else 0
        filt = c4(filt + 3 * (qs0 - ps0))
        filter1 = c4(filt + 4) >> 3
        filter2 = c4(filt + 3) >> 3
        buf[y][x] = c4(qs0 - filter1) + off
        buf[y - dy][x - dx] = c4(ps0 + filter2) + off
        if not hevMask:
            filt = round2(filter1, 1)
            buf[y + dy][x + dx] = c4(qs1 - filt) + off
            buf[y - dy * 2][x - dx * 2] = c4(ps1 + filt) + off

    def wide(self, x, y, plane, dx, dy, log2Size):
        buf = self.planes[plane]
        if log2Size == 4:
            n = 6
        elif plane == 0:
            n = 3
        else:
            n = 2
        n2 = 0 if (log2Size == 3 and plane == 0) else 1
        F = {}
        for i in range(-n, n):
            t = 0
            for j in range(-n, n + 1):
                p = clip3(-(n + 1), n, i + j)
                tap = 2 if abs(j) <= n2 else 1
                t += buf[y + p * dy][x + p * dx] * tap
            F[i] = round2(t, log2Size)
        for i in range(-n, n):
            buf[y + i * dy][x + i * dx] = F[i]
