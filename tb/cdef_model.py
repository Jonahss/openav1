"""Spec-literal model of CDEF (spec 7.15): direction search, variance-adaptive primary strength and the
constrained directional filter. Reads the deblocked planes, returns new planes (CdefFrame).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import av1_tables as T  # noqa: E402

MI_SIZE = 4
CDEF_DIRECTIONS = T.Cdef_Directions
CDEF_UV_DIR = T.Cdef_Uv_Dir
DIV_TABLE = T.Div_Table
PRI_TAPS = T.Cdef_Pri_Taps
SEC_TAPS = T.Cdef_Sec_Taps


def floor_log2(x):
    return x.bit_length() - 1


def clip3(lo, hi, x):
    return lo if x < lo else hi if x > hi else x


def constrain(diff, threshold, damping):
    if not threshold:
        return 0
    dampingAdj = max(0, damping - floor_log2(threshold))
    sign = -1 if diff < 0 else 1
    return sign * clip3(0, abs(diff), threshold - (abs(diff) >> dampingAdj))


class Cdef:
    def __init__(self, hdr, state, planes):
        """hdr: FrameHeader; state: object with Skips[r][c] and cdef_idx[(r,c)] (4x4 luma units); planes: deblocked."""
        self.h = hdr
        self.s = state
        self.cur = planes
        # strengths as coded: y/uv_strength[i] = (pri << 2) | sec, with sec 3 meaning 4 (dav1d packs them the same way)
        self.y_pri = [v >> 2 for v in hdr.cdef_y_strengths]
        self.y_sec = [(v & 3) + ((v & 3) == 3) for v in hdr.cdef_y_strengths]
        self.uv_pri = [v >> 2 for v in hdr.cdef_uv_strengths]
        self.uv_sec = [(v & 3) + ((v & 3) == 3) for v in hdr.cdef_uv_strengths]

    def apply(self):
        h = self.h
        self.out = [[row[:] for row in p] for p in self.cur]     # CdefFrame starts as a copy
        step4 = T.Num_4x4_Blocks_Wide[T.ENUM["BLOCK_8X8"]]
        cdefSize4 = T.Num_4x4_Blocks_Wide[T.ENUM["BLOCK_64X64"]]
        cdefMask4 = ~(cdefSize4 - 1)
        for r in range(0, h.MiRows, step4):
            for c in range(0, h.MiCols, step4):
                idx = self.s.cdef_idx.get((r & cdefMask4, c & cdefMask4), -1)
                self.block(r, c, idx)
        return self.out

    def skips(self, r, c):
        h = self.h
        return self.s.Skips[min(r, h.MiRows - 1)][min(c, h.MiCols - 1)]

    def block(self, r, c, idx):
        h = self.h
        if idx == -1:
            return
        coeffShift = h.BitDepth - 8
        skip = self.skips(r, c) and self.skips(r + 1, c) and self.skips(r, c + 1) and self.skips(r + 1, c + 1)
        if skip:
            return
        yDir, var = self.direction(r, c)
        priStr = self.y_pri[idx] << coeffShift
        secStr = self.y_sec[idx] << coeffShift
        dr = 0 if priStr == 0 else yDir
        varStr = min(floor_log2(var >> 6), 12) if (var >> 6) else 0
        priStr = ((priStr * (4 + varStr) + 8) >> 4) if var else 0
        damping = h.cdef_damping + coeffShift
        self.filter(0, r, c, priStr, secStr, damping, dr)
        if h.NumPlanes == 1:
            return
        priStr = self.uv_pri[idx] << coeffShift
        secStr = self.uv_sec[idx] << coeffShift
        dr = 0 if priStr == 0 else CDEF_UV_DIR[h.subsampling_x][h.subsampling_y][yDir]
        damping = h.cdef_damping + coeffShift - 1
        self.filter(1, r, c, priStr, secStr, damping, dr)
        self.filter(2, r, c, priStr, secStr, damping, dr)

    def direction(self, r, c):
        h = self.h
        buf = self.cur[0]
        cost = [0] * 8
        partial = [[0] * 15 for _ in range(8)]
        x0 = c << 2
        y0 = r << 2
        for i in range(8):
            row = buf[y0 + i]
            for j in range(8):
                x = (row[x0 + j] >> (h.BitDepth - 8)) - 128
                partial[0][i + j] += x
                partial[1][i + j // 2] += x
                partial[2][i] += x
                partial[3][3 + i - j // 2] += x
                partial[4][7 + i - j] += x
                partial[5][3 - i // 2 + j] += x
                partial[6][j] += x
                partial[7][i // 2 + j] += x
        for i in range(8):
            cost[2] += partial[2][i] * partial[2][i]
            cost[6] += partial[6][i] * partial[6][i]
        cost[2] *= DIV_TABLE[8]
        cost[6] *= DIV_TABLE[8]
        for i in range(7):
            cost[0] += (partial[0][i] * partial[0][i] + partial[0][14 - i] * partial[0][14 - i]) * DIV_TABLE[i + 1]
            cost[4] += (partial[4][i] * partial[4][i] + partial[4][14 - i] * partial[4][14 - i]) * DIV_TABLE[i + 1]
        cost[0] += partial[0][7] * partial[0][7] * DIV_TABLE[8]
        cost[4] += partial[4][7] * partial[4][7] * DIV_TABLE[8]
        for i in range(1, 8, 2):
            for j in range(5):
                cost[i] += partial[i][3 + j] * partial[i][3 + j]
            cost[i] *= DIV_TABLE[8]
            for j in range(3):
                cost[i] += (partial[i][j] * partial[i][j] + partial[i][10 - j] * partial[i][10 - j]) * DIV_TABLE[2 * j + 2]
        bestCost = 0
        yDir = 0
        for i in range(8):
            if cost[i] > bestCost:
                bestCost = cost[i]
                yDir = i
        var = (bestCost - cost[(yDir + 4) & 7]) >> 10
        return yDir, var

    def filter(self, plane, r, c, priStr, secStr, damping, dr):
        h = self.h
        coeffShift = h.BitDepth - 8
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        x0 = (c * MI_SIZE) >> subX
        y0 = (r * MI_SIZE) >> subY
        w = 8 >> subX
        hh = 8 >> subY
        src = self.cur[plane]
        dst = self.out[plane]
        H = len(src)
        W = len(src[0])
        pt = PRI_TAPS[(priStr >> coeffShift) & 1]
        st = SEC_TAPS[(priStr >> coeffShift) & 1]
        for i in range(hh):
            for j in range(w):
                x = src[y0 + i][x0 + j]
                total = 0
                mx = mn = x
                for k in range(2):
                    for sign in (-1, 1):
                        yy = y0 + i + sign * CDEF_DIRECTIONS[dr][k][0]
                        xx = x0 + j + sign * CDEF_DIRECTIONS[dr][k][1]
                        if 0 <= yy < H and 0 <= xx < W:          # is_inside_filter_region: whole frame
                            p = src[yy][xx]
                            total += pt[k] * constrain(p - x, priStr, damping)
                            mx = max(p, mx); mn = min(p, mn)
                        for dirOff in (-2, 2):
                            d2 = (dr + dirOff) & 7
                            yy = y0 + i + sign * CDEF_DIRECTIONS[d2][k][0]
                            xx = x0 + j + sign * CDEF_DIRECTIONS[d2][k][1]
                            if 0 <= yy < H and 0 <= xx < W:
                                s = src[yy][xx]
                                total += st[k] * constrain(s - x, secStr, damping)
                                mx = max(s, mx); mn = min(s, mn)
                dst[y0 + i][x0 + j] = clip3(mn, mx, x + ((8 + total - (1 if total < 0 else 0)) >> 4))
