"""Spec-literal Python model of the AV1 inverse transforms (spec section 7.13.2 and 7.13.3).

Transcribed from the AV1 Bitstream & Decoding Process Specification. This is the oracle the
inverse-transform RTL is fuzzed against; dav1d traces (C events -> R minus P) confirm it later.

Intermediate-range checks are the spec's "requirement of bitstream conformance" clauses; they raise
Nonconformant so a fuzzer can discard inputs no real encoder would produce.
"""

# ---- constants -------------------------------------------------------------------------
TX_4X4, TX_8X8, TX_16X16, TX_32X32, TX_64X64, TX_4X8, TX_8X4, TX_8X16, TX_16X8, TX_16X32, \
    TX_32X16, TX_32X64, TX_64X32, TX_4X16, TX_16X4, TX_8X32, TX_32X8, TX_16X64, TX_64X16 = range(19)
TX_WIDTH_LOG2 = [2, 3, 4, 5, 6, 2, 3, 3, 4, 4, 5, 5, 6, 2, 4, 3, 5, 4, 6]
TX_HEIGHT_LOG2 = [2, 3, 4, 5, 6, 3, 2, 4, 3, 5, 4, 6, 5, 4, 2, 5, 3, 6, 4]
TRANSFORM_ROW_SHIFT = [0, 1, 2, 2, 2, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2]

DCT_DCT, ADST_DCT, DCT_ADST, ADST_ADST, FLIPADST_DCT, DCT_FLIPADST, FLIPADST_FLIPADST, \
    ADST_FLIPADST, FLIPADST_ADST, IDTX, V_DCT, H_DCT, V_ADST, H_ADST, V_FLIPADST, H_FLIPADST = range(16)

# 1D transform class used for rows / columns, per the 2D process text.
ROW_DCT = {DCT_DCT, ADST_DCT, FLIPADST_DCT, H_DCT}
ROW_ADST = {DCT_ADST, ADST_ADST, DCT_FLIPADST, FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST, H_ADST, H_FLIPADST}
COL_DCT = {DCT_DCT, DCT_ADST, DCT_FLIPADST, V_DCT}
COL_ADST = {ADST_DCT, ADST_ADST, FLIPADST_DCT, FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST, V_ADST, V_FLIPADST}

COS128_LOOKUP = [
    4096, 4095, 4091, 4085, 4076, 4065, 4052, 4036,
    4017, 3996, 3973, 3948, 3920, 3889, 3857, 3822,
    3784, 3745, 3703, 3659, 3612, 3564, 3513, 3461,
    3406, 3349, 3290, 3229, 3166, 3102, 3035, 2967,
    2896, 2824, 2751, 2675, 2598, 2520, 2440, 2359,
    2276, 2191, 2106, 2019, 1931, 1842, 1751, 1660,
    1567, 1474, 1380, 1285, 1189, 1092, 995, 897,
    799, 700, 601, 501, 401, 301, 201, 101, 0,
]
SINPI_1_9, SINPI_2_9, SINPI_3_9, SINPI_4_9 = 1321, 2482, 3344, 3803


class Nonconformant(Exception):
    pass


def round2(x, n):
    if n == 0:
        return x
    return (x + (1 << (n - 1))) >> n   # arithmetic shift: matches the spec for negative x


def clip3(lo, hi, x):
    return lo if x < lo else hi if x > hi else x


def fits(x, bits):
    return -(1 << (bits - 1)) <= x <= (1 << (bits - 1)) - 1


def brev(num_bits, x):
    t = 0
    for i in range(num_bits):
        t += ((x >> i) & 1) << (num_bits - 1 - i)
    return t


def cos128(angle):
    a = angle & 255
    if a <= 64:
        return COS128_LOOKUP[a]
    if a <= 128:
        return -COS128_LOOKUP[128 - a]
    if a <= 192:
        return -COS128_LOOKUP[a - 128]
    return COS128_LOOKUP[256 - a]


def sin128(angle):
    return cos128(angle - 64)


# ---- butterflies (operate on list T in place) ------------------------------------------
def B(T, a, b, angle, flip, r):
    x = T[a] * cos128(angle) - T[b] * sin128(angle)
    y = T[a] * sin128(angle) + T[b] * cos128(angle)
    T[a] = round2(x, 12)
    T[b] = round2(y, 12)
    if not (fits(T[a], r) and fits(T[b], r)):
        raise Nonconformant(f"B({a},{b},{angle}) overflow r={r}")
    if flip:
        T[a], T[b] = T[b], T[a]


def H(T, a, b, flip, r):
    if flip:
        a, b = b, a
    x, y = T[a], T[b]
    lo, hi = -(1 << (r - 1)), (1 << (r - 1)) - 1
    T[a] = clip3(lo, hi, x + y)
    T[b] = clip3(lo, hi, x - y)


# ---- inverse DCT -------------------------------------------------------------------------
def inv_dct_permute(T, n):
    c = list(T)
    for i in range(1 << n):
        T[i] = c[brev(n, i)]


def inv_dct(T, n, r):
    assert 2 <= n <= 6 and len(T) == (1 << n)
    inv_dct_permute(T, n)
    if n == 6:
        for i in range(16): B(T, 32 + i, 63 - i, 63 - 4 * brev(4, i), 0, r)
    if n >= 5:
        for i in range(8): B(T, 16 + i, 31 - i, 6 + (brev(3, 7 - i) << 3), 0, r)
    if n == 6:
        for i in range(16): H(T, 32 + i * 2, 33 + i * 2, i & 1, r)
    if n >= 4:
        for i in range(4): B(T, 8 + i, 15 - i, 12 + (brev(2, 3 - i) << 4), 0, r)
    if n >= 5:
        for i in range(8): H(T, 16 + 2 * i, 17 + 2 * i, i & 1, r)
    if n == 6:
        for i in range(4):
            for j in range(2): B(T, 62 - i * 4 - j, 33 + i * 4 + j, 60 - 16 * brev(2, i) + 64 * j, 1, r)
    if n >= 3:
        for i in range(2): B(T, 4 + i, 7 - i, 56 - 32 * i, 0, r)
    if n >= 4:
        for i in range(4): H(T, 8 + 2 * i, 9 + 2 * i, i & 1, r)
    if n >= 5:
        for i in range(2):
            for j in range(2): B(T, 30 - 4 * i - j, 17 + 4 * i + j, 24 + (j << 6) + ((1 - i) << 5), 1, r)
    if n == 6:
        for i in range(8):
            for j in range(2): H(T, 32 + i * 4 + j, 35 + i * 4 - j, i & 1, r)
    for i in range(2): B(T, 2 * i, 2 * i + 1, 32 + 16 * i, 1 - i, r)
    if n >= 3:
        for i in range(2): H(T, 4 + 2 * i, 5 + 2 * i, i, r)
    if n >= 4:
        for i in range(2): B(T, 14 - i, 9 + i, 48 + 64 * i, 1, r)
    if n >= 5:
        for i in range(4):
            for j in range(2): H(T, 16 + 4 * i + j, 19 + 4 * i - j, i & 1, r)
    if n == 6:
        for i in range(2):
            for j in range(4): B(T, 61 - i * 8 - j, 34 + i * 8 + j, 56 - i * 32 + (j >> 1) * 64, 1, r)
    for i in range(2): H(T, i, 3 - i, 0, r)
    if n >= 3:
        B(T, 6, 5, 32, 1, r)
    if n >= 4:
        for i in range(2):
            for j in range(2): H(T, 8 + 4 * i + j, 11 + 4 * i - j, i, r)
    if n >= 5:
        for i in range(4): B(T, 29 - i, 18 + i, 48 + (i >> 1) * 64, 1, r)
    if n == 6:
        for i in range(4):
            for j in range(4): H(T, 32 + 8 * i + j, 39 + 8 * i - j, i & 1, r)
    if n >= 3:
        for i in range(4): H(T, i, 7 - i, 0, r)
    if n >= 4:
        for i in range(2): B(T, 13 - i, 10 + i, 32, 1, r)
    if n >= 5:
        for i in range(2):
            for j in range(4): H(T, 16 + i * 8 + j, 23 + i * 8 - j, i, r)
    if n == 6:
        for i in range(8): B(T, 59 - i, 36 + i, 48 if i < 4 else 112, 1, r)
    if n >= 4:
        for i in range(8): H(T, i, 15 - i, 0, r)
    if n >= 5:
        for i in range(4): B(T, 27 - i, 20 + i, 32, 1, r)
    if n == 6:
        for i in range(8):
            H(T, 32 + i, 47 - i, 0, r)
            H(T, 48 + i, 63 - i, 1, r)
    if n >= 5:
        for i in range(16): H(T, i, 31 - i, 0, r)
    if n == 6:
        for i in range(8): B(T, 55 - i, 40 + i, 32, 1, r)
    if n == 6:
        for i in range(32): H(T, i, 63 - i, 0, r)


# ---- inverse ADST ------------------------------------------------------------------------
def inv_adst_in_permute(T, n):
    n0 = 1 << n
    c = list(T)
    for i in range(n0):
        idx = (i - 1) if (i & 1) else (n0 - i - 1)
        T[i] = c[idx]


def inv_adst_out_permute(T, n):
    n0 = 1 << n
    c = list(T)
    for i in range(n0):
        a = (i >> 3) & 1
        b = ((i >> 2) & 1) ^ ((i >> 3) & 1)
        cc = ((i >> 1) & 1) ^ ((i >> 2) & 1)
        d = (i & 1) ^ ((i >> 1) & 1)
        idx = ((d << 3) | (cc << 2) | (b << 1) | a) >> (4 - n)
        T[i] = -c[idx] if (i & 1) else c[idx]


def inv_adst4(T, r):
    s = [0] * 7
    s[0] = SINPI_1_9 * T[0]
    s[1] = SINPI_2_9 * T[0]
    s[2] = SINPI_3_9 * T[1]
    s[3] = SINPI_4_9 * T[2]
    s[4] = SINPI_1_9 * T[2]
    s[5] = SINPI_2_9 * T[3]
    s[6] = SINPI_4_9 * T[3]
    a7 = T[0] - T[2]
    b7 = a7 + T[3]
    if not fits(a7, r + 1) or not fits(b7, r):
        raise Nonconformant("adst4 a7/b7")
    s[0] = s[0] + s[3]
    s[1] = s[1] - s[4]
    s[3] = s[2]
    s[2] = SINPI_3_9 * b7
    s[0] = s[0] + s[5]
    s[1] = s[1] - s[6]
    x = [0] * 4
    x[0] = s[0] + s[3]
    x[1] = s[1] + s[3]
    x[2] = s[2]
    x[3] = s[0] + s[1]
    x[3] = x[3] - s[3]
    for v in s + x:
        if not fits(v, r + 12):
            raise Nonconformant("adst4 s/x")
    for i in range(4):
        T[i] = round2(x[i], 12)


def inv_adst8(T, r):
    inv_adst_in_permute(T, 3)
    for i in range(4): B(T, 2 * i, 2 * i + 1, 60 - 16 * i, 1, r)
    for i in range(4): H(T, i, 4 + i, 0, r)
    for i in range(2): B(T, 4 + 3 * i, 5 + i, 48 - 32 * i, 1, r)
    for i in range(2):
        for j in range(2): H(T, 4 * j + i, 2 + 4 * j + i, 0, r)
    for i in range(2): B(T, 2 + 4 * i, 3 + 4 * i, 32, 1, r)
    inv_adst_out_permute(T, 3)


def inv_adst16(T, r):
    inv_adst_in_permute(T, 4)
    for i in range(8): B(T, 2 * i, 2 * i + 1, 62 - 8 * i, 1, r)
    for i in range(8): H(T, i, 8 + i, 0, r)
    for i in range(2):
        B(T, 8 + 2 * i, 9 + 2 * i, 56 - 32 * i, 1, r)
        B(T, 13 + 2 * i, 12 + 2 * i, 8 + 32 * i, 1, r)
    for i in range(4):
        for j in range(2): H(T, 8 * j + i, 4 + 8 * j + i, 0, r)
    for i in range(2):
        for j in range(2): B(T, 4 + 8 * j + 3 * i, 5 + 8 * j + i, 48 - 32 * i, 1, r)
    for i in range(2):
        for j in range(4): H(T, 4 * j + i, 2 + 4 * j + i, 0, r)
    for i in range(4): B(T, 2 + 4 * i, 3 + 4 * i, 32, 1, r)
    inv_adst_out_permute(T, 4)


def inv_adst(T, n, r):
    assert 2 <= n <= 4 and len(T) == (1 << n)
    if n == 2:
        inv_adst4(T, r)
    elif n == 3:
        inv_adst8(T, r)
    else:
        inv_adst16(T, r)


# ---- WHT and identity --------------------------------------------------------------------
def inv_wht(T, shift):
    a = T[0] >> shift
    c = T[1] >> shift
    d = T[2] >> shift
    b = T[3] >> shift
    a += c
    d -= b
    e = (a - d) >> 1
    b = e - b
    c = e - c
    a -= b
    d += c
    T[0], T[1], T[2], T[3] = a, b, c, d


def inv_identity(T, n):
    assert 2 <= n <= 5
    if n == 2:
        for i in range(4): T[i] = round2(T[i] * 5793, 12)
    elif n == 3:
        for i in range(8): T[i] = T[i] * 2
    elif n == 4:
        for i in range(16): T[i] = round2(T[i] * 11586, 12)
    else:
        for i in range(32): T[i] = T[i] * 4


# ---- 2D inverse transform (7.13.3) -------------------------------------------------------
def inverse_transform_2d(dequant, tx_sz, tx_type, bit_depth, lossless=False):
    """dequant: h x w list of lists (already clipped per 7.12.3). Returns Residual h x w."""
    log2w, log2h = TX_WIDTH_LOG2[tx_sz], TX_HEIGHT_LOG2[tx_sz]
    w, h = 1 << log2w, 1 << log2h
    row_shift = 0 if lossless else TRANSFORM_ROW_SHIFT[tx_sz]
    col_shift = 0 if lossless else 4
    row_clamp = bit_depth + 8
    col_clamp = max(bit_depth + 6, 16)
    residual = [[0] * w for _ in range(h)]

    for i in range(h):
        T = [dequant[i][j] if (i < 32 and j < 32) else 0 for j in range(w)]
        if abs(log2w - log2h) == 1:
            T = [round2(t * 2896, 12) for t in T]
        if lossless:
            inv_wht(T, 2)
        elif tx_type in ROW_DCT:
            inv_dct(T, log2w, row_clamp)
        elif tx_type in ROW_ADST:
            inv_adst(T, log2w, row_clamp)
        else:
            inv_identity(T, log2w)
        for j in range(w):
            residual[i][j] = round2(T[j], row_shift)

    lo, hi = -(1 << (col_clamp - 1)), (1 << (col_clamp - 1)) - 1
    for i in range(h):
        for j in range(w):
            residual[i][j] = clip3(lo, hi, residual[i][j])

    for j in range(w):
        T = [residual[i][j] for i in range(h)]
        if lossless:
            inv_wht(T, 0)
        elif tx_type in COL_DCT:
            inv_dct(T, log2h, col_clamp)
        elif tx_type in COL_ADST:
            inv_adst(T, log2h, col_clamp)
        else:
            inv_identity(T, log2h)
        for i in range(h):
            residual[i][j] = round2(T[i], col_shift)
    return residual


# ---- self-test: integer DCT vs floating-point reference ----------------------------------
if __name__ == "__main__":
    import math, random
    rng = random.Random(7)
    worst = 0.0
    for n in range(2, 7):
        N = 1 << n
        for _ in range(200):
            coefs = [rng.randint(-2000, 2000) for _ in range(N)]
            T = list(coefs)
            inv_dct(T, n, 24)
            # AV1's integer DCT computes x[k] = sum_j c_j * cos(pi*(2k+1)*j/(2N)) with c_0 scaled by 1/sqrt2
            ref = [sum((c / math.sqrt(2) if j == 0 else c) * math.cos(math.pi * (2 * k + 1) * j / (2 * N))
                       for j, c in enumerate(coefs)) for k in range(N)]
            err = max(abs(a - b) for a, b in zip(T, ref))
            worst = max(worst, err)
    print(f"inv_dct vs float: worst abs error {worst:.2f} (expect a few units of rounding)")
    T = [1000, 0, 0, 0]; inv_dct(T, 2, 20); print("DC-only 4-point:", T)
    T = [1000, -300, 200, 50]; inv_identity(T, 2); print("identity4:", T)
    T = [100, 20, -30, 40]; inv_wht(T, 2); print("wht:", T)
    d = [[0] * 8 for _ in range(8)]; d[0][0] = 4096
    print("2D 8x8 DC 4096 ->", inverse_transform_2d(d, TX_8X8, DCT_DCT, 8)[0][:4])
    d = [[0] * 4 for _ in range(8)]; d[0][0] = 1000; d[1][2] = -500
    print("2D 4x8 ADST_ADST ->", inverse_transform_2d(d, TX_4X8, ADST_ADST, 10)[0])
    print("self-test done")
