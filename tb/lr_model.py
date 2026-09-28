"""Spec-literal model of loop restoration (spec 7.17): Wiener and self-guided filters, applied per 4x4
block with the 64-row stripe rule (samples outside the current stripe come from the deblocked but
not CDEF-filtered picture), both already upscaled by superres_model when use_superres is set (7.16).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import av1_tables as T  # noqa: E402

MI_SIZE = 4
RESTORE_NONE, RESTORE_WIENER, RESTORE_SGRPROJ = 0, 1, 2
FILTER_BITS = 7
SGRPROJ_RST_BITS = 4
SGRPROJ_PRJ_BITS = 7
SGRPROJ_MTABLE_BITS = 20
SGRPROJ_SGR_BITS = 8
SGRPROJ_RECIP_BITS = 12


def round2(x, n):
    return x if n == 0 else (x + (1 << (n - 1))) >> n


def clip3(lo, hi, x):
    return lo if x < lo else hi if x > hi else x


def count_units_in_frame(unitSize, frameSize):
    return max((frameSize + (unitSize >> 1)) // unitSize, 1)


class LoopRestoration:
    def __init__(self, hdr, state, cur_planes, cdef_planes):
        """cur_planes: deblocked (UpscaledCurrFrame); cdef_planes: after CDEF (UpscaledCdefFrame)."""
        self.h = hdr
        self.s = state
        self.cur = cur_planes
        self.cdef = cdef_planes

    def apply(self):
        h = self.h
        self.lr = [[row[:] for row in p] for p in self.cdef]
        if all(t == RESTORE_NONE for t in h.FrameRestorationType[:h.NumPlanes]):
            return self.lr
        bd = h.BitDepth
        self.InterRound0 = 3 + (2 if bd == 12 else 0)
        self.InterRound1 = 11 - (2 if bd == 12 else 0)
        for y in range(0, h.FrameHeight, MI_SIZE):
            for x in range(0, h.UpscaledWidth, MI_SIZE):
                for plane in range(h.NumPlanes):
                    if h.FrameRestorationType[plane] != RESTORE_NONE:
                        self.block(plane, y >> 2, x >> 2)
        return self.lr

    def block(self, plane, row, col):
        h = self.h
        lumaY = row * MI_SIZE
        stripeNum = (lumaY + 8) // 64
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        self.StripeStartY = (-8 + stripeNum * 64) >> subY
        self.StripeEndY = self.StripeStartY + (64 >> subY) - 1
        unitSize = h.LoopRestorationSize[plane]
        unitRows = count_units_in_frame(unitSize, round2(h.FrameHeight, subY))
        unitCols = count_units_in_frame(unitSize, round2(h.UpscaledWidth, subX))
        unitRow = min(unitRows - 1, ((row * MI_SIZE + 8) >> subY) // unitSize)
        unitCol = min(unitCols - 1, ((col * MI_SIZE) >> subX) // unitSize)
        self.PlaneEndX = round2(h.UpscaledWidth, subX) - 1
        self.PlaneEndY = round2(h.FrameHeight, subY) - 1
        x = (col * MI_SIZE) >> subX
        y = (row * MI_SIZE) >> subY
        w = min(MI_SIZE >> subX, self.PlaneEndX - x + 1)
        hh = min(MI_SIZE >> subY, self.PlaneEndY - y + 1)
        if w <= 0 or hh <= 0:
            return
        key = (plane, unitRow, unitCol)
        rType = self.s.LrType.get(key, RESTORE_NONE)
        if rType == RESTORE_WIENER:
            self.wiener(plane, key, x, y, w, hh)
        elif rType == RESTORE_SGRPROJ:
            self.self_guided(plane, key, x, y, w, hh)

    # 7.17.6
    def src(self, plane, x, y):
        x = max(0, min(self.PlaneEndX, x))
        y = max(0, min(self.PlaneEndY, y))
        if y < self.StripeStartY:
            y = max(self.StripeStartY - 2, y)
            return self.cur[plane][y][x]
        if y > self.StripeEndY:
            y = min(self.StripeEndY + 2, y)
            return self.cur[plane][y][x]
        return self.cdef[plane][y][x]

    # 7.17.4 / 7.17.5
    @staticmethod
    def wiener_coeffs(coeff):
        f = [0] * 7
        f[3] = 128
        for i in range(3):
            c = coeff[i]
            f[i] = c
            f[6 - i] = c
            f[3] -= 2 * c
        return f

    def wiener(self, plane, key, x, y, w, hh):
        bd = self.h.BitDepth
        coefs = self.s.LrWiener[key]
        vfilter = self.wiener_coeffs(coefs[0])
        hfilter = self.wiener_coeffs(coefs[1])
        r0, r1 = self.InterRound0, self.InterRound1
        offset = 1 << (bd + FILTER_BITS - r0 - 1)
        limit = (1 << (bd + 1 + FILTER_BITS - r0)) - 1
        inter = [[0] * w for _ in range(hh + 6)]
        for r in range(hh + 6):
            for c in range(w):
                s = 0
                for t in range(7):
                    s += hfilter[t] * self.src(plane, x + c + t - 3, y + r - 3)
                v = round2(s, r0)
                inter[r][c] = clip3(-offset, limit - offset, v)
        mx = (1 << bd) - 1
        out = self.lr[plane]
        for r in range(hh):
            for c in range(w):
                s = 0
                for t in range(7):
                    s += vfilter[t] * inter[r + t][c]
                v = round2(s, r1)
                out[y + r][x + c] = clip3(0, mx, v)

    # 7.17.2 / 7.17.3
    def self_guided(self, plane, key, x, y, w, hh):
        h = self.h
        bd = h.BitDepth
        st = self.s.LrSgrSet[key]
        flt0 = self.box_filter(plane, x, y, w, hh, st, 0)
        flt1 = self.box_filter(plane, x, y, w, hh, st, 1)
        w0, w1 = self.s.LrSgrXqd[key]
        w2 = (1 << SGRPROJ_PRJ_BITS) - w0 - w1
        r0 = T.Sgr_Params[st][0]
        r1 = T.Sgr_Params[st][2]
        mx = (1 << bd) - 1
        out = self.lr[plane]
        cd = self.cdef[plane]
        for i in range(hh):
            for j in range(w):
                u = cd[y + i][x + j] << SGRPROJ_RST_BITS
                v = w1 * u
                v += w0 * (flt0[i][j] if r0 else u)
                v += w2 * (flt1[i][j] if r1 else u)
                s = round2(v, SGRPROJ_RST_BITS + SGRPROJ_PRJ_BITS)
                out[y + i][x + j] = clip3(0, mx, s)

    def box_filter(self, plane, x, y, w, hh, st, pss):
        bd = self.h.BitDepth
        r = T.Sgr_Params[st][pss * 2 + 0]
        if r == 0:
            return None
        eps = T.Sgr_Params[st][pss * 2 + 1]
        n = (2 * r + 1) * (2 * r + 1)
        n2e = n * n * eps
        s = ((1 << SGRPROJ_MTABLE_BITS) + n2e // 2) // n2e
        A = {}
        B = {}
        oneOverN = ((1 << SGRPROJ_RECIP_BITS) + (n // 2)) // n
        for i in range(-1, hh + 1):
            for j in range(-1, w + 1):
                a = 0
                b = 0
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        c = self.src(plane, x + j + dx, y + i + dy)
                        a += c * c
                        b += c
                a = round2(a, 2 * (bd - 8))
                d = round2(b, bd - 8)
                p = max(0, a * n - d * d)
                z = round2(p * s, SGRPROJ_MTABLE_BITS)
                if z >= 255:
                    a2 = 256
                elif z == 0:
                    a2 = 1
                else:
                    a2 = ((z << SGRPROJ_SGR_BITS) + (z // 2)) // (z + 1)
                b2 = ((1 << SGRPROJ_SGR_BITS) - a2) * b * oneOverN
                A[(i, j)] = a2
                B[(i, j)] = round2(b2, SGRPROJ_RECIP_BITS)
        F = [[0] * w for _ in range(hh)]
        cd = self.cdef[plane]
        for i in range(hh):
            shift = 5
            if pss == 0 and (i & 1):
                shift = 4
            for j in range(w):
                a = 0
                b = 0
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if pss == 0:
                            weight = (6 if dx == 0 else 5) if ((i + dy) & 1) else 0
                        else:
                            weight = 4 if (dx == 0 or dy == 0) else 3
                        a += weight * A[(i + dy, j + dx)]
                        b += weight * B[(i + dy, j + dx)]
                v = a * cd[y + i][x + j] + b
                F[i][j] = round2(v, SGRPROJ_SGR_BITS + shift - SGRPROJ_RST_BITS)
        return F
