"""Spec-literal Python model of AV1 intra prediction (spec section 7.11.2) plus CfL (7.11.5).

Unit-level: the caller supplies the already-built edge arrays AboveRow / LeftCol (length w+h,
built with the availability/replication rules of 7.11.2.1, which are position dependent and
belong to the caller) and the top-left sample. This is exactly what dav1d's ipred functions
receive, so a trace of dav1d's edges + parameters + output pixels checks this model directly.

All functions return pred as a list of h rows of w samples.
"""

# ---- constants and tables -------------------------------------------------------------------
DC_PRED, V_PRED, H_PRED, D45_PRED, D135_PRED, D113_PRED, D157_PRED, D203_PRED, D67_PRED, \
    SMOOTH_PRED, SMOOTH_V_PRED, SMOOTH_H_PRED, PAETH_PRED = range(13)
UV_CFL_PRED = 13

ANGLE_STEP = 3
INTRA_FILTER_SCALE_BITS = 4
MODE_TO_ANGLE = [0, 90, 180, 45, 135, 113, 157, 203, 67, 0, 0, 0, 0]

DR_INTRA_DERIVATIVE = [
    0, 0, 0, 1023, 0, 0, 547, 0, 0, 372, 0, 0, 0, 0,
    273, 0, 0, 215, 0, 0, 178, 0, 0, 151, 0, 0, 132, 0, 0,
    116, 0, 0, 102, 0, 0, 0, 90, 0, 0, 80, 0, 0, 71, 0, 0,
    64, 0, 0, 57, 0, 0, 51, 0, 0, 45, 0, 0, 0, 40, 0, 0,
    35, 0, 0, 31, 0, 0, 27, 0, 0, 23, 0, 0, 19, 0, 0,
    15, 0, 0, 0, 0, 11, 0, 0, 7, 0, 0, 3, 0, 0,
]

SM_WEIGHTS = {
    2: [255, 149, 85, 64],
    3: [255, 197, 146, 105, 73, 50, 37, 32],
    4: [255, 225, 196, 170, 145, 123, 102, 84, 68, 54, 43, 33, 26, 20, 17, 16],
    5: [255, 240, 225, 210, 196, 182, 169, 157, 145, 133, 122, 111, 101, 92, 83, 74,
        66, 59, 52, 45, 39, 34, 29, 25, 21, 17, 14, 12, 10, 9, 8, 8],
    6: [255, 248, 240, 233, 225, 218, 210, 203, 196, 189, 182, 176, 169, 163, 156,
        150, 144, 138, 133, 127, 121, 116, 111, 106, 101, 96, 91, 86, 82, 77, 73, 69,
        65, 61, 57, 54, 50, 47, 44, 41, 38, 35, 32, 29, 27, 25, 22, 20, 18, 16, 15,
        13, 12, 10, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4],
}

INTRA_FILTER_TAPS = [
    [[-6, 10, 0, 0, 0, 12, 0], [-5, 2, 10, 0, 0, 9, 0], [-3, 1, 1, 10, 0, 7, 0], [-3, 1, 1, 2, 10, 5, 0],
     [-4, 6, 0, 0, 0, 2, 12], [-3, 2, 6, 0, 0, 2, 9], [-3, 2, 2, 6, 0, 2, 7], [-3, 1, 2, 2, 6, 3, 5]],
    [[-10, 16, 0, 0, 0, 10, 0], [-6, 0, 16, 0, 0, 6, 0], [-4, 0, 0, 16, 0, 4, 0], [-2, 0, 0, 0, 16, 2, 0],
     [-10, 16, 0, 0, 0, 0, 10], [-6, 0, 16, 0, 0, 0, 6], [-4, 0, 0, 16, 0, 0, 4], [-2, 0, 0, 0, 16, 0, 2]],
    [[-8, 8, 0, 0, 0, 16, 0], [-8, 0, 8, 0, 0, 16, 0], [-8, 0, 0, 8, 0, 16, 0], [-8, 0, 0, 0, 8, 16, 0],
     [-4, 4, 0, 0, 0, 0, 16], [-4, 0, 4, 0, 0, 0, 16], [-4, 0, 0, 4, 0, 0, 16], [-4, 0, 0, 0, 4, 0, 16]],
    [[-2, 8, 0, 0, 0, 10, 0], [-1, 3, 8, 0, 0, 6, 0], [-1, 2, 3, 8, 0, 4, 0], [0, 1, 2, 3, 8, 2, 0],
     [-1, 4, 0, 0, 0, 3, 10], [-1, 3, 4, 0, 0, 4, 6], [-1, 2, 3, 4, 0, 4, 4], [-1, 2, 2, 3, 4, 3, 3]],
    [[-12, 14, 0, 0, 0, 14, 0], [-10, 0, 14, 0, 0, 12, 0], [-9, 0, 0, 14, 0, 11, 0], [-8, 0, 0, 0, 14, 10, 0],
     [-10, 12, 0, 0, 0, 0, 14], [-9, 1, 12, 0, 0, 0, 12], [-8, 0, 0, 12, 0, 1, 11], [-7, 0, 0, 1, 12, 1, 9]],
]

INTRA_EDGE_KERNEL = [[0, 4, 8, 4, 0], [0, 5, 6, 5, 0], [2, 4, 4, 4, 2]]


def round2(x, n):
    return x if n == 0 else (x + (1 << (n - 1))) >> n


def round2signed(x, n):
    return round2(x, n) if x >= 0 else -round2(-x, n)


def clip3(lo, hi, x):
    return lo if x < lo else hi if x > hi else x


def clip1(x, bd):
    return clip3(0, (1 << bd) - 1, x)


def is_directional(mode):
    return V_PRED <= mode <= D67_PRED


class Edge:
    """An edge array with negative indices (AboveRow[-1], and upsampling writes down to -2)."""
    OFF = 16

    def __init__(self, values, topleft):
        self.a = [0] * (self.OFF + 2 * len(values) + 8)
        for i, v in enumerate(values):
            self.a[self.OFF + i] = v
        self.a[self.OFF - 1] = topleft

    def __getitem__(self, i):
        return self.a[self.OFF + i]

    def __setitem__(self, i, v):
        self.a[self.OFF + i] = v


# ---- 7.11.2.2 basic (Paeth) --------------------------------------------------------------------
def pred_paeth(above, left, w, h):
    pred = [[0] * w for _ in range(h)]
    tl = above[-1]
    for i in range(h):
        for j in range(w):
            base = above[j] + left[i] - tl
            p_left = abs(base - left[i])
            p_top = abs(base - above[j])
            p_tl = abs(base - tl)
            if p_left <= p_top and p_left <= p_tl:
                pred[i][j] = left[i]
            elif p_top <= p_tl:
                pred[i][j] = above[j]
            else:
                pred[i][j] = tl
    return pred


# ---- 7.11.2.3 recursive (filter intra) ---------------------------------------------------------
def pred_filter_intra(above, left, w, h, filter_intra_mode, bd):
    pred = [[0] * w for _ in range(h)]
    w4, h2 = w >> 2, h >> 1
    for i2 in range(h2):
        for j4 in range(w4):
            p = [0] * 7
            for i in range(7):
                if i < 5:
                    if i2 == 0:
                        p[i] = above[(j4 << 2) + i - 1]
                    elif j4 == 0 and i == 0:
                        p[i] = left[(i2 << 1) - 1]
                    else:
                        p[i] = pred[(i2 << 1) - 1][(j4 << 2) + i - 1]
                else:
                    if j4 == 0:
                        p[i] = left[(i2 << 1) + i - 5]
                    else:
                        p[i] = pred[(i2 << 1) + i - 5][(j4 << 2) - 1]
            for i1 in range(2):
                for j1 in range(4):
                    pr = sum(INTRA_FILTER_TAPS[filter_intra_mode][(i1 << 2) + j1][i] * p[i] for i in range(7))
                    pred[(i2 << 1) + i1][(j4 << 2) + j1] = clip1(round2signed(pr, INTRA_FILTER_SCALE_BITS), bd)
    return pred


# ---- 7.11.2.7 .. 7.11.2.12 edge preparation -----------------------------------------------------
def filter_corner(above, left):
    s = left[0] * 5 + above[-1] * 6 + above[0] * 5
    return round2(s, 4)


def edge_filter_strength(w, h, filter_type, delta):
    d = abs(delta)
    blk = w + h
    strength = 0
    if filter_type == 0:
        if blk <= 8:
            if d >= 56: strength = 1
        elif blk <= 12:
            if d >= 40: strength = 1
        elif blk <= 16:
            if d >= 40: strength = 1
        elif blk <= 24:
            if d >= 8: strength = 1
            if d >= 16: strength = 2
            if d >= 32: strength = 3
        elif blk <= 32:
            strength = 1
            if d >= 4: strength = 2
            if d >= 32: strength = 3
        else:
            strength = 3
    else:
        if blk <= 8:
            if d >= 40: strength = 1
            if d >= 64: strength = 2
        elif blk <= 16:
            if d >= 20: strength = 1
            if d >= 48: strength = 2
        elif blk <= 24:
            if d >= 4: strength = 3
        else:
            strength = 3
    return strength


def use_upsample(w, h, filter_type, delta):
    d = abs(delta)
    blk = w + h
    if d <= 0 or d >= 40:
        return 0
    if filter_type == 0:
        return 1 if blk <= 16 else 0
    return 1 if blk <= 8 else 0


def edge_upsample(buf, num_px, bd):
    dup = [0] * (num_px + 3)
    dup[0] = buf[-1]
    for i in range(-1, num_px):
        dup[i + 2] = buf[i]
    dup[num_px + 2] = buf[num_px - 1]
    buf[-2] = dup[0]
    for i in range(num_px):
        s = -dup[i] + 9 * dup[i + 1] + 9 * dup[i + 2] - dup[i + 3]
        s = clip1(round2(s, 4), bd)
        buf[2 * i - 1] = s
        buf[2 * i] = dup[i + 2]


def edge_filter(buf, sz, strength):
    if strength == 0:
        return
    edge = [buf[i - 1] for i in range(sz)]
    for i in range(1, sz):
        s = 0
        for j in range(5):
            k = clip3(0, sz - 1, i - 2 + j)
            s += INTRA_EDGE_KERNEL[strength - 1][j] * edge[k]
        buf[i - 1] = (s + 8) >> 4


# ---- 7.11.2.4 directional ----------------------------------------------------------------------
def pred_directional(above, left, w, h, mode, angle_delta, bd, have_left, have_above,
                     enable_intra_edge_filter, filter_type, above_px=None, left_px=None):
    """above_px / left_px: Min(w, maxX - x + 1) and Min(h, maxY - y + 1) from the caller (default w, h).
    above/left are Edge objects and are MODIFIED (filtered / upsampled) like the spec's arrays."""
    if above_px is None: above_px = w
    if left_px is None: left_px = h
    p_angle = MODE_TO_ANGLE[mode] + angle_delta * ANGLE_STEP
    upsample_above = upsample_left = 0
    if enable_intra_edge_filter:
        if p_angle != 90 and p_angle != 180:
            if 90 < p_angle < 180 and (w + h) >= 24:
                c = filter_corner(above, left)
                left[-1] = c
                above[-1] = c
            if have_above:
                strength = edge_filter_strength(w, h, filter_type, p_angle - 90)
                num_px = above_px + (h if p_angle < 90 else 0) + 1
                edge_filter(above, num_px, strength)
            if have_left:
                strength = edge_filter_strength(w, h, filter_type, p_angle - 180)
                num_px = left_px + (w if p_angle > 180 else 0) + 1
                edge_filter(left, num_px, strength)
        upsample_above = use_upsample(w, h, filter_type, p_angle - 90)
        num_px = w + (h if p_angle < 90 else 0)
        if upsample_above:
            edge_upsample(above, num_px, bd)
        upsample_left = use_upsample(w, h, filter_type, p_angle - 180)
        num_px = h + (w if p_angle > 180 else 0)
        if upsample_left:
            edge_upsample(left, num_px, bd)

    dx = dy = None
    if p_angle < 90:
        dx = DR_INTRA_DERIVATIVE[p_angle]
    elif 90 < p_angle < 180:
        dx = DR_INTRA_DERIVATIVE[180 - p_angle]
    if 90 < p_angle < 180:
        dy = DR_INTRA_DERIVATIVE[p_angle - 90]
    elif p_angle > 180:
        dy = DR_INTRA_DERIVATIVE[270 - p_angle]

    pred = [[0] * w for _ in range(h)]
    if p_angle < 90:
        max_base_x = (w + h - 1) << upsample_above
        for i in range(h):
            for j in range(w):
                idx = (i + 1) * dx
                base = (idx >> (6 - upsample_above)) + (j << upsample_above)
                shift = ((idx << upsample_above) >> 1) & 0x1F
                if base < max_base_x:
                    pred[i][j] = round2(above[base] * (32 - shift) + above[base + 1] * shift, 5)
                else:
                    pred[i][j] = above[max_base_x]
    elif 90 < p_angle < 180:
        for i in range(h):
            for j in range(w):
                idx = (j << 6) - (i + 1) * dx
                base = idx >> (6 - upsample_above)
                if base >= -(1 << upsample_above):
                    shift = ((idx << upsample_above) >> 1) & 0x1F
                    pred[i][j] = round2(above[base] * (32 - shift) + above[base + 1] * shift, 5)
                else:
                    idx = (i << 6) - (j + 1) * dy
                    base = idx >> (6 - upsample_left)
                    shift = ((idx << upsample_left) >> 1) & 0x1F
                    pred[i][j] = round2(left[base] * (32 - shift) + left[base + 1] * shift, 5)
    elif p_angle > 180:
        for i in range(h):
            for j in range(w):
                idx = (j + 1) * dy
                base = (idx >> (6 - upsample_left)) + (i << upsample_left)
                shift = ((idx << upsample_left) >> 1) & 0x1F
                pred[i][j] = round2(left[base] * (32 - shift) + left[base + 1] * shift, 5)
    elif p_angle == 90:
        for i in range(h):
            for j in range(w):
                pred[i][j] = above[j]
    else:  # 180
        for i in range(h):
            for j in range(w):
                pred[i][j] = left[i]
    return pred


# ---- 7.11.2.5 DC -------------------------------------------------------------------------------
def pred_dc(above, left, w, h, log2w, log2h, have_left, have_above, bd):
    if have_left and have_above:
        s = sum(left[k] for k in range(h)) + sum(above[k] for k in range(w))
        s += (w + h) >> 1
        v = s // (w + h)
    elif have_left:
        s = sum(left[k] for k in range(h))
        v = clip1((s + (h >> 1)) >> log2h, bd)
    elif have_above:
        s = sum(above[k] for k in range(w))
        v = clip1((s + (w >> 1)) >> log2w, bd)
    else:
        v = 1 << (bd - 1)
    return [[v] * w for _ in range(h)]


# ---- 7.11.2.6 smooth ---------------------------------------------------------------------------
def pred_smooth(above, left, w, h, log2w, log2h, mode):
    pred = [[0] * w for _ in range(h)]
    wx, wy = SM_WEIGHTS[log2w], SM_WEIGHTS[log2h]
    for i in range(h):
        for j in range(w):
            if mode == SMOOTH_PRED:
                s = wy[i] * above[j] + (256 - wy[i]) * left[h - 1] + wx[j] * left[i] + (256 - wx[j]) * above[w - 1]
                pred[i][j] = round2(s, 9)
            elif mode == SMOOTH_V_PRED:
                s = wy[i] * above[j] + (256 - wy[i]) * left[h - 1]
                pred[i][j] = round2(s, 8)
            else:
                s = wx[j] * left[i] + (256 - wx[j]) * above[w - 1]
                pred[i][j] = round2(s, 8)
    return pred


# ---- 7.11.2.1 dispatcher -------------------------------------------------------------------------
def predict_intra(above_vals, left_vals, topleft, mode, log2w, log2h, bd, *, have_left=1, have_above=1,
                  angle_delta=0, enable_intra_edge_filter=1, filter_type=0, use_filter_intra=False,
                  filter_intra_mode=0, above_px=None, left_px=None):
    """above_vals / left_vals: the w+h edge samples (AboveRow[0..w+h-1], LeftCol[0..w+h-1])."""
    w, h = 1 << log2w, 1 << log2h
    above = Edge(above_vals, topleft)
    left = Edge(left_vals, topleft)
    if use_filter_intra:
        return pred_filter_intra(above, left, w, h, filter_intra_mode, bd)
    if is_directional(mode):
        return pred_directional(above, left, w, h, mode, angle_delta, bd, have_left, have_above,
                                enable_intra_edge_filter, filter_type, above_px, left_px)
    if mode in (SMOOTH_PRED, SMOOTH_V_PRED, SMOOTH_H_PRED):
        return pred_smooth(above, left, w, h, log2w, log2h, mode)
    if mode == DC_PRED:
        return pred_dc(above, left, w, h, log2w, log2h, have_left, have_above, bd)
    return pred_paeth(above, left, w, h)


# ---- 7.11.5 chroma from luma -----------------------------------------------------------------------
def predict_cfl(dc_pred, luma_sub, alpha, log2w, log2h, bd):
    """dc_pred: h x w DC-predicted chroma; luma_sub: h x w subsampled luma L values (already << (3-subX-subY)).
    alpha: signed CflAlpha. Returns the final chroma prediction."""
    w, h = 1 << log2w, 1 << log2h
    avg = round2(sum(sum(r) for r in luma_sub), log2w + log2h)
    out = [[0] * w for _ in range(h)]
    for i in range(h):
        for j in range(w):
            scaled = round2signed(alpha * (luma_sub[i][j] - avg), 6)
            out[i][j] = clip1(dc_pred[i][j] + scaled, bd)
    return out


def cfl_subsample_luma(luma, start_x, start_y, w, h, sub_x, sub_y, max_luma_w, max_luma_h):
    """Build L per the spec from the reconstructed luma plane (2D list)."""
    L = [[0] * w for _ in range(h)]
    for i in range(h):
        ly = min((start_y + i) << sub_y, max_luma_h - (1 << sub_y))
        for j in range(w):
            lx = min((start_x + j) << sub_x, max_luma_w - (1 << sub_x))
            t = 0
            for dy in range(sub_y + 1):
                for dx in range(sub_x + 1):
                    t += luma[ly + dy][lx + dx]
            L[i][j] = t << (3 - sub_x - sub_y)
    return L


# ---- self-test ----------------------------------------------------------------------------------
if __name__ == "__main__":
    import random
    rng = random.Random(5)
    bad = 0
    # 1. constant edges -> constant prediction in every mode / size / angle delta
    for bd in (8, 10):
        for log2w in range(2, 7):
            for log2h in range(2, 7):
                if abs(log2w - log2h) > 2: continue
                w, h = 1 << log2w, 1 << log2h
                c = rng.randint(0, (1 << bd) - 1)
                for mode in range(13):
                    for ad in ([-3, -1, 0, 2, 3] if is_directional(mode) else [0]):
                        for ft in (0, 1):
                            p = predict_intra([c] * (w + h), [c] * (w + h), c, mode, log2w, log2h, bd,
                                              angle_delta=ad, filter_type=ft)
                            if any(v != c for row in p for v in row):
                                bad += 1
                                if bad < 5: print("non-constant:", mode, ad, log2w, log2h, p[0][:4])
                if log2w <= 5 and log2h <= 5:
                    for fm in range(5):
                        p = predict_intra([c] * (w + h), [c] * (w + h), c, 0, log2w, log2h, bd,
                                          use_filter_intra=True, filter_intra_mode=fm)
                        if any(v != c for row in p for v in row):
                            bad += 1; print("filter-intra non-constant", fm, log2w, log2h)
    # 2. V / H copy the edges exactly
    for _ in range(50):
        log2w, log2h = rng.randint(2, 6), rng.randint(2, 6)
        if abs(log2w - log2h) > 2: continue
        w, h = 1 << log2w, 1 << log2h
        a = [rng.randint(0, 255) for _ in range(w + h)]
        l = [rng.randint(0, 255) for _ in range(w + h)]
        pv = predict_intra(a, l, 128, V_PRED, log2w, log2h, 8)
        ph = predict_intra(a, l, 128, H_PRED, log2w, log2h, 8)
        if any(pv[i] != a[:w] for i in range(h)) or any(ph[i] != [l[i]] * w for i in range(h)):
            bad += 1; print("V/H copy failed")
    # 3. DC of a known set
    p = predict_intra([10, 20, 30, 40, 0, 0, 0, 0], [50, 60, 70, 80, 0, 0, 0, 0], 0, DC_PRED, 2, 2, 8)
    assert p[0][0] == (10 + 20 + 30 + 40 + 50 + 60 + 70 + 80 + 4) // 8, p[0][0]
    # 4. D45 with a ramp: pred[0][0] should interpolate above[1..2]
    a = list(range(0, 160, 10)); l = [0] * 16
    p = predict_intra(a[:8], l[:8], 0, D45_PRED, 2, 2, 8, enable_intra_edge_filter=0)
    print("D45 ramp row0:", p[0], "row3:", p[3])
    print("self-test", "OK" if bad == 0 else f"{bad} problems")
