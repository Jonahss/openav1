"""Spec-literal Python model of the AV1 tile syntax for INTRA frames (spec 5.11 + 8.2 + 8.3.2 + 7.12.3).

Given the frame-header fields (from the trace's H event) and a tile's bytes, this decodes every
symbol of the tile exactly as the spec's syntax tables say: partitions, intra mode info, palette,
filter-intra, CfL, delta q/lf, cdef, loop-restoration units, transform sizes and types, and the
coefficients including dequantisation. Prediction and reconstruction are NOT done here (they are
separate models); the outputs are the decoded syntax and the Dequant blocks.

Every symbol read goes through SymbolDecoder, which has a hook so a harness can compare each read
with dav1d's S/U events. Not modelled: inter frames, intrabc, superres-dependent LR unit maths is
modelled but untested.
"""
import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import av1_tables as T  # noqa: E402

# ---- constants (from the spec via av1_tables, with the few we rely on spelled out) --------------
C = T.CONST
E = dict(T.ENUM)
for _k, _v in T.CONST.items():        # names like BLOCK_INVALID live in the constants list
    E.setdefault(_k, _v)
MI_SIZE = 4
MAX_ANGLE_DELTA = 3
PALETTE_COLORS = 8
PALETTE_NUM_NEIGHBORS = 3
DELTA_Q_SMALL = 3
DELTA_LF_SMALL = 3
MAX_LOOP_FILTER = 63
NUM_BASE_LEVELS = 2
COEFF_BASE_RANGE = 12
BR_CDF_SIZE = 4
SIG_COEF_CONTEXTS = C["SIG_COEF_CONTEXTS"]
SIG_COEF_CONTEXTS_EOB = C["SIG_COEF_CONTEXTS_EOB"]
SIG_COEF_CONTEXTS_2D = C["SIG_COEF_CONTEXTS_2D"]
TX_CLASS_2D, TX_CLASS_HORIZ, TX_CLASS_VERT = 0, 1, 2
CFL_SIGN_ZERO, CFL_SIGN_NEG, CFL_SIGN_POS = 0, 1, 2
MAX_VARTX_DEPTH = 2
MAX_SEGMENTS = 8
SEG_LVL_ALT_Q, SEG_LVL_REF_FRAME, SEG_LVL_SKIP, SEG_LVL_GLOBALMV = 0, 5, 6, 7
FRAME_LF_COUNT = 4
SGRPROJ_PARAMS_BITS = 4
SGRPROJ_PRJ_SUBEXP_K = 4
SGRPROJ_PRJ_BITS = 7
SUPERRES_NUM = 8
TX_SET_DCTONLY, TX_SET_INTRA_1, TX_SET_INTRA_2 = 0, 1, 2
INTRA_FRAME, NONE = 0, -1

BLOCK_4X4, BLOCK_8X8, BLOCK_64X64, BLOCK_128X128 = E["BLOCK_4X4"], E["BLOCK_8X8"], E["BLOCK_64X64"], E["BLOCK_128X128"]
BLOCK_INVALID = E["BLOCK_INVALID"]
PARTITION_NONE, PARTITION_HORZ, PARTITION_VERT, PARTITION_SPLIT = 0, 1, 2, 3
PARTITION_HORZ_A, PARTITION_HORZ_B, PARTITION_VERT_A, PARTITION_VERT_B, PARTITION_HORZ_4, PARTITION_VERT_4 = 4, 5, 6, 7, 8, 9
TX_4X4, TX_8X8, TX_16X16, TX_32X32, TX_64X64 = 0, 1, 2, 3, 4
TX_16X64, TX_64X16, TX_16X32, TX_32X16 = E["TX_16X64"], E["TX_64X16"], E["TX_16X32"], E["TX_32X16"]
TX_SIZES = 5
TX_MODE_SELECT = 2
DC_PRED, V_PRED, D67_PRED, UV_CFL_PRED = 0, 1, 8, 13
DCT_DCT, IDTX, V_DCT, H_DCT, V_ADST, H_ADST, V_FLIPADST, H_FLIPADST = 0, 9, 10, 11, 12, 13, 14, 15
RESTORE_NONE, RESTORE_WIENER, RESTORE_SGRPROJ, RESTORE_SWITCHABLE = 0, 1, 2, 3


def is_directional_mode(mode):
    return V_PRED <= mode <= D67_PRED


def clip3(lo, hi, x):
    return lo if x < lo else hi if x > hi else x


def round2(x, n):
    return x if n == 0 else (x + (1 << (n - 1))) >> n


def floor_log2(x):
    return x.bit_length() - 1


def ceil_log2(x):
    if x < 2:
        return 0
    return (x - 1).bit_length()


# =================================================================================================
# 8.2 symbol decoder (spec form: cdf[N-1] == 32768, cdf[N] = adaptation counter)
# =================================================================================================
class SymbolDecoder:
    def __init__(self, data, disable_cdf_update, hook=None):
        self.data = data
        self.bitpos = 0
        self.disable_cdf_update = disable_cdf_update
        self.hook = hook                      # hook(kind, n, value, rng, cdf_before, cdf_after)
        sz = len(data)
        num_bits = min(sz * 8, 15)
        buf = self._f(num_bits)
        padded = buf << (15 - num_bits)
        self.value = ((1 << 15) - 1) ^ padded
        self.range = 1 << 15
        self.max_bits = 8 * sz - 15

    def _f(self, n):
        x = 0
        for _ in range(n):
            byte = self.data[self.bitpos >> 3] if (self.bitpos >> 3) < len(self.data) else 0
            bit = (byte >> (7 - (self.bitpos & 7))) & 1
            x = (x << 1) | bit
            self.bitpos += 1
        return x

    def read_symbol(self, cdf, name=""):
        N = len(cdf) - 1
        before = list(cdf) if self.hook else None
        cur = self.range
        symbol = -1
        while True:
            symbol += 1
            prev = cur
            fval = (1 << 15) - cdf[symbol]
            cur = ((self.range >> 8) * (fval >> 6)) >> 1
            cur += 4 * (N - symbol - 1)
            if not (self.value < cur):
                break
        self.range = prev - cur
        self.value -= cur
        bits = 15 - floor_log2(self.range)
        self.range <<= bits
        num_bits = min(bits, max(0, self.max_bits))
        new_data = self._f(num_bits)
        padded = new_data << (bits - num_bits)
        self.value = padded ^ (((self.value + 1) << bits) - 1)
        self.max_bits -= bits
        if not self.disable_cdf_update:
            rate = 3 + (cdf[N] > 15) + (cdf[N] > 31) + min(floor_log2(N), 2)
            tmp = 0
            for i in range(N - 1):
                tmp = (1 << 15) if i == symbol else tmp
                if tmp < cdf[i]:
                    cdf[i] -= (cdf[i] - tmp) >> rate
                else:
                    cdf[i] += (tmp - cdf[i]) >> rate
            cdf[N] += 1 if cdf[N] < 32 else 0
        if self.hook:
            self.hook("S", N, symbol, self.range, before, list(cdf), name)
        return symbol

    def read_bool(self, name=""):
        # a fresh cdf each time: adaptation is discarded
        return self.read_symbol([1 << 14, 1 << 15, 0], name)

    def read_literal(self, n, name=""):
        x = 0
        for _ in range(n):
            x = 2 * x + self.read_bool(name)
        return x

    def read_ns(self, n, name=""):
        w = floor_log2(n) + 1
        m = (1 << w) - n
        v = self.read_literal(w - 1, name)
        if v < m:
            return v
        extra_bit = self.read_literal(1, name)
        return (v << 1) - m + extra_bit


# =================================================================================================
# frame header (from the trace's H event) -> the spec variables the tile syntax needs
# =================================================================================================
class FrameHeader:
    """Fields in the order written by the setup_tile hook in tools/dav1d_trace_patch.py."""

    def __init__(self, f):
        it = iter(f)
        nxt = lambda: next(it)  # noqa: E731
        self.tile_row, self.tile_col = nxt(), nxt()
        self.MiColStart, self.MiColEnd, self.MiRowStart, self.MiRowEnd = nxt(), nxt(), nxt(), nxt()
        self.MiCols, self.MiRows, self.BitDepth, self.layout = nxt(), nxt(), nxt(), nxt()
        self.use_128x128_superblock, self.frame_type, self.primary_ref_frame = nxt(), nxt(), nxt()
        self.TxMode, self.reduced_tx_set, self.disable_cdf_update = nxt(), nxt(), nxt()
        self.base_q_idx, self.DeltaQYDc, self.DeltaQUDc, self.DeltaQUAc, self.DeltaQVDc, self.DeltaQVAc = \
            nxt(), nxt(), nxt(), nxt(), nxt(), nxt()
        self.using_qmatrix, self.qm_y, self.qm_u, self.qm_v = nxt(), nxt(), nxt(), nxt()
        self.delta_q_present, self.delta_q_res, self.delta_lf_present, self.delta_lf_res, self.delta_lf_multi = \
            nxt(), nxt(), nxt(), nxt(), nxt()
        self.allow_screen_content_tools, self.allow_intrabc = nxt(), nxt()
        self.enable_filter_intra, self.enable_intra_edge_filter, self.enable_cdef = nxt(), nxt(), nxt()
        self.cdef_damping, self.cdef_bits, self.CodedLossless = nxt(), nxt(), nxt()
        # dav1d's enum is in coded (lr_type) order: NONE, SWITCHABLE, WIENER, SGRPROJ -> spec FrameRestorationType
        _remap = [RESTORE_NONE, RESTORE_SWITCHABLE, RESTORE_WIENER, RESTORE_SGRPROJ]
        self.FrameRestorationType = [_remap[nxt()], _remap[nxt()], _remap[nxt()]]
        us = [nxt(), nxt()]
        self.LoopRestorationSize = [1 << us[0], 1 << us[1], 1 << us[1]]
        self.FrameWidth, self.UpscaledWidth, self.FrameHeight = nxt(), nxt(), nxt()
        self.use_superres, self.SuperresDenom = nxt(), nxt()
        self.subsampling_x, self.subsampling_y = nxt(), nxt()
        self.segmentation_enabled, self.segmentation_update_map, self.segmentation_temporal_update = nxt(), nxt(), nxt()
        self.SegIdPreSkip, self.LastActiveSegId = nxt(), nxt()
        self.LosslessArray = [0] * 8
        self.seg_qidx = [0] * 8
        self.FeatureEnabled = [[0] * 8 for _ in range(8)]
        self.FeatureData = [[0] * 8 for _ in range(8)]
        for s in range(8):
            lossless, qidx, dq, lfyv, lfyh, lfu, lfv, ref, skip, gmv = [nxt() for _ in range(10)]
            self.LosslessArray[s] = lossless
            self.seg_qidx[s] = qidx
            vals = [dq, lfyv, lfyh, lfu, lfv, ref, skip, gmv]
            for j, v in enumerate(vals):
                if j == 5:
                    en = v >= 0
                else:
                    en = v != 0
                self.FeatureEnabled[s][j] = 1 if en else 0
                self.FeatureData[s][j] = v if en else 0
        rest = list(it)
        if len(rest) >= 32:
            self.loop_filter_level = [rest[0], rest[1], rest[2], rest[3]]
            self.loop_filter_sharpness = rest[4]
            self.loop_filter_delta_enabled = rest[5]
            self.loop_filter_ref_deltas = rest[6:14]
            self.loop_filter_mode_deltas = rest[14:16]
            self.cdef_y_strengths = rest[16:24]
            self.cdef_uv_strengths = rest[24:32]
        else:
            self.loop_filter_level = [0, 0, 0, 0]
            self.loop_filter_sharpness = 0
            self.loop_filter_delta_enabled = 0
            self.loop_filter_ref_deltas = [1, 0, 0, 0, -1, 0, -1, -1]
            self.loop_filter_mode_deltas = [0, 0]
            self.cdef_y_strengths = [0] * 8
            self.cdef_uv_strengths = [0] * 8
        self.NumPlanes = 1 if self.layout == 0 else 3
        self.FrameIsIntra = self.frame_type in (0, 2)
        assert self.FrameIsIntra, "tile_model handles intra frames only"
        assert self.primary_ref_frame == 7, "CDF inheritance from a reference frame is not modelled"
        self.SegQMLevel = [[0] * 8 for _ in range(3)]
        for s in range(8):
            if self.using_qmatrix and not self.LosslessArray[s]:
                self.SegQMLevel[0][s], self.SegQMLevel[1][s], self.SegQMLevel[2][s] = self.qm_y, self.qm_u, self.qm_v
            else:
                self.SegQMLevel[0][s] = self.SegQMLevel[1][s] = self.SegQMLevel[2][s] = 15


# =================================================================================================
# CDF set (tile copies of the defaults)
# =================================================================================================
def make_cdfs(base_q_idx):
    idx = 0 if base_q_idx <= 20 else 1 if base_q_idx <= 60 else 2 if base_q_idx <= 120 else 3
    d = {}
    for name in dir(T):
        if name.startswith("Default_") and name.endswith("_Cdf"):
            key = name[len("Default_"):-len("_Cdf")]
            val = getattr(T, name)
            if key in ("Txb_Skip", "Eob_Pt_16", "Eob_Pt_32", "Eob_Pt_64", "Eob_Pt_128", "Eob_Pt_256", "Eob_Pt_512",
                       "Eob_Pt_1024", "Eob_Extra", "Dc_Sign", "Coeff_Base_Eob", "Coeff_Base", "Coeff_Br"):
                val = val[idx]
            d[key] = copy.deepcopy(val)
    d["Delta_Lf_Multi"] = [copy.deepcopy(T.Default_Delta_Lf_Cdf) for _ in range(FRAME_LF_COUNT)]
    return d


# =================================================================================================
# the tile decoder
# =================================================================================================
class TileDecoder:
    def __init__(self, hdr, data, hook=None):
        self.h = hdr
        self.dec = SymbolDecoder(data, hdr.disable_cdf_update, hook)
        self.cdf = make_cdfs(hdr.base_q_idx)
        h = hdr
        R, Cc = h.MiRows, h.MiCols
        self.YModes = [[DC_PRED] * Cc for _ in range(R)]
        self.UVModes = [[DC_PRED] * Cc for _ in range(R)]
        self.Skips = [[0] * Cc for _ in range(R)]
        self.TxSizes = [[TX_4X4] * Cc for _ in range(R)]
        self.InterTxSizes = [[TX_4X4] * Cc for _ in range(R)]
        self.MiSizes = [[BLOCK_4X4] * Cc for _ in range(R)]
        self.SegmentIds = [[0] * Cc for _ in range(R)]
        self.PaletteSizes = [[[0] * Cc for _ in range(R)] for _ in range(2)]
        self.PaletteColors = [[[None] * Cc for _ in range(R)] for _ in range(2)]
        self.TxTypes = [[DCT_DCT] * Cc for _ in range(R)]
        self.DeltaLFs = [[None] * Cc for _ in range(R)]
        self.AboveLevelContext = [[0] * Cc for _ in range(3)]
        self.AboveDcContext = [[0] * Cc for _ in range(3)]
        self.LeftLevelContext = [[0] * R for _ in range(3)]
        self.LeftDcContext = [[0] * R for _ in range(3)]
        self.AboveSegPredContext = [0] * Cc
        self.LeftSegPredContext = [0] * R
        self.cdef_idx = {}
        self.DeltaLF = [0] * FRAME_LF_COUNT
        self.CurrentQIndex = h.base_q_idx
        self.ReadDeltas = 0
        self.RefLrWiener = [[[3, -7, 15] for _ in range(2)] for _ in range(3)]
        self.RefSgrXqd = [[-32, 31] for _ in range(3)]
        self.LrType = {}       # (plane, unitRow, unitCol) -> restoration type
        self.LrWiener = {}     # (plane, unitRow, unitCol) -> [[3 coefs pass 0], [3 coefs pass 1]]
        self.LrSgrSet = {}     # (plane, unitRow, unitCol) -> set
        self.LrSgrXqd = {}     # (plane, unitRow, unitCol) -> [w0, w1]
        self.events = []          # ("C", plane, x4, y4, txSz, txType, eob, dequant rows) etc.
        self.blocks = []          # decoded block infos (for inspection)
        self.Quant = [0] * 1024
        self.Dequant = None
        self.LoopfilterTxSizes = []
        for plane in range(3):
            subX = hdr.subsampling_x if plane > 0 else 0
            subY = hdr.subsampling_y if plane > 0 else 0
            self.LoopfilterTxSizes.append([[TX_4X4] * ((Cc + subX) >> subX) for _ in range((R + subY) >> subY)])

    # ---- helpers ------------------------------------------------------------------------------
    def sym(self, cdf, name):
        return self.dec.read_symbol(cdf, name)

    def L(self, n, name):
        return self.dec.read_literal(n, name)

    def is_inside(self, r, c):
        h = self.h
        return h.MiColStart <= c < h.MiColEnd and h.MiRowStart <= r < h.MiRowEnd

    def seg_feature_active_idx(self, idx, feature):
        return self.h.segmentation_enabled and self.h.FeatureEnabled[idx][feature]

    def seg_feature_active(self, feature):
        return self.seg_feature_active_idx(self.segment_id, feature)

    def get_qindex(self, ignore_delta_q, segment_id):
        h = self.h
        if self.seg_feature_active_idx(segment_id, SEG_LVL_ALT_Q):
            data = h.FeatureData[segment_id][SEG_LVL_ALT_Q]
            qindex = h.base_q_idx + data
            if ignore_delta_q == 0 and h.delta_q_present == 1:
                qindex = self.CurrentQIndex + data
            return clip3(0, 255, qindex)
        if ignore_delta_q == 0 and h.delta_q_present == 1:
            return self.CurrentQIndex
        return h.base_q_idx

    # ---- 5.11.1 decode_tile ----------------------------------------------------------------------
    def decode_tile(self):
        h = self.h
        # clear_above_context
        for p in range(3):
            for i in range(h.MiCols):
                self.AboveLevelContext[p][i] = 0
                self.AboveDcContext[p][i] = 0
        for i in range(h.MiCols):
            self.AboveSegPredContext[i] = 0
        self.DeltaLF = [0] * FRAME_LF_COUNT
        for plane in range(h.NumPlanes):
            for pss in range(2):
                self.RefSgrXqd[plane][pss] = T.Sgrproj_Xqd_Mid[pss]
                for i in range(3):
                    self.RefLrWiener[plane][pss][i] = T.Wiener_Taps_Mid[i]
        sbSize = BLOCK_128X128 if h.use_128x128_superblock else BLOCK_64X64
        sbSize4 = T.Num_4x4_Blocks_Wide[sbSize]
        r = h.MiRowStart
        while r < h.MiRowEnd:
            # clear_left_context
            for p in range(3):
                for i in range(h.MiRows):
                    self.LeftLevelContext[p][i] = 0
                    self.LeftDcContext[p][i] = 0
            for i in range(h.MiRows):
                self.LeftSegPredContext[i] = 0
            c = h.MiColStart
            while c < h.MiColEnd:
                self.ReadDeltas = h.delta_q_present
                self.clear_cdef(r, c)
                self.clear_block_decoded_flags(r, c, sbSize4)
                self.read_lr(r, c, sbSize)
                self.decode_partition(r, c, sbSize)
                c += sbSize4
            r += sbSize4

    def clear_cdef(self, r, c):
        self.cdef_idx[(r, c)] = -1
        if self.h.use_128x128_superblock:
            cdefSize4 = T.Num_4x4_Blocks_Wide[BLOCK_64X64]
            self.cdef_idx[(r, c + cdefSize4)] = -1
            self.cdef_idx[(r + cdefSize4, c)] = -1
            self.cdef_idx[(r + cdefSize4, c + cdefSize4)] = -1

    # ---- 5.11.4 decode_partition -----------------------------------------------------------------
    def decode_partition(self, r, c, bSize):
        h = self.h
        if r >= h.MiRows or c >= h.MiCols:
            return
        self.AvailU = self.is_inside(r - 1, c)
        self.AvailL = self.is_inside(r, c - 1)
        num4x4 = T.Num_4x4_Blocks_Wide[bSize]
        halfBlock4x4 = num4x4 >> 1
        quarterBlock4x4 = halfBlock4x4 >> 1
        hasRows = (r + halfBlock4x4) < h.MiRows
        hasCols = (c + halfBlock4x4) < h.MiCols
        self.part_bsize = bSize           # (read by tools/gen_stream.py to keep generated partitions conformant)
        if bSize < BLOCK_8X8:
            partition = PARTITION_NONE
        elif hasRows and hasCols:
            partition = self.sym(self.partition_cdf(r, c, bSize), "partition")
        elif hasCols:
            pcdf = self.partition_cdf(r, c, bSize)
            psum = (pcdf[PARTITION_VERT] - pcdf[PARTITION_VERT - 1] + pcdf[PARTITION_SPLIT] - pcdf[PARTITION_SPLIT - 1]
                    + pcdf[PARTITION_HORZ_A] - pcdf[PARTITION_HORZ_A - 1] + pcdf[PARTITION_VERT_A] - pcdf[PARTITION_VERT_A - 1]
                    + pcdf[PARTITION_VERT_B] - pcdf[PARTITION_VERT_B - 1])
            if bSize != BLOCK_128X128:
                psum += pcdf[PARTITION_VERT_4] - pcdf[PARTITION_VERT_4 - 1]
            split_or_horz = self.sym([(1 << 15) - psum, 1 << 15, 0], "split_or_horz")
            partition = PARTITION_SPLIT if split_or_horz else PARTITION_HORZ
        elif hasRows:
            pcdf = self.partition_cdf(r, c, bSize)
            psum = (pcdf[PARTITION_HORZ] - pcdf[PARTITION_HORZ - 1] + pcdf[PARTITION_SPLIT] - pcdf[PARTITION_SPLIT - 1]
                    + pcdf[PARTITION_HORZ_A] - pcdf[PARTITION_HORZ_A - 1] + pcdf[PARTITION_HORZ_B] - pcdf[PARTITION_HORZ_B - 1]
                    + pcdf[PARTITION_VERT_A] - pcdf[PARTITION_VERT_A - 1])
            if bSize != BLOCK_128X128:
                psum += pcdf[PARTITION_HORZ_4] - pcdf[PARTITION_HORZ_4 - 1]
            split_or_vert = self.sym([(1 << 15) - psum, 1 << 15, 0], "split_or_vert")
            partition = PARTITION_SPLIT if split_or_vert else PARTITION_VERT
        else:
            partition = PARTITION_SPLIT
        subSize = T.Partition_Subsize[partition][bSize]
        splitSize = T.Partition_Subsize[PARTITION_SPLIT][bSize]
        db, dp = self.decode_block, self.decode_partition
        if partition == PARTITION_NONE:
            db(r, c, subSize)
        elif partition == PARTITION_HORZ:
            db(r, c, subSize)
            if hasRows: db(r + halfBlock4x4, c, subSize)
        elif partition == PARTITION_VERT:
            db(r, c, subSize)
            if hasCols: db(r, c + halfBlock4x4, subSize)
        elif partition == PARTITION_SPLIT:
            dp(r, c, subSize); dp(r, c + halfBlock4x4, subSize)
            dp(r + halfBlock4x4, c, subSize); dp(r + halfBlock4x4, c + halfBlock4x4, subSize)
        elif partition == PARTITION_HORZ_A:
            db(r, c, splitSize); db(r, c + halfBlock4x4, splitSize); db(r + halfBlock4x4, c, subSize)
        elif partition == PARTITION_HORZ_B:
            db(r, c, subSize); db(r + halfBlock4x4, c, splitSize); db(r + halfBlock4x4, c + halfBlock4x4, splitSize)
        elif partition == PARTITION_VERT_A:
            db(r, c, splitSize); db(r + halfBlock4x4, c, splitSize); db(r, c + halfBlock4x4, subSize)
        elif partition == PARTITION_VERT_B:
            db(r, c, subSize); db(r, c + halfBlock4x4, splitSize); db(r + halfBlock4x4, c + halfBlock4x4, splitSize)
        elif partition == PARTITION_HORZ_4:
            db(r + quarterBlock4x4 * 0, c, subSize); db(r + quarterBlock4x4 * 1, c, subSize)
            db(r + quarterBlock4x4 * 2, c, subSize)
            if r + quarterBlock4x4 * 3 < h.MiRows: db(r + quarterBlock4x4 * 3, c, subSize)
        else:
            db(r, c + quarterBlock4x4 * 0, subSize); db(r, c + quarterBlock4x4 * 1, subSize)
            db(r, c + quarterBlock4x4 * 2, subSize)
            if c + quarterBlock4x4 * 3 < h.MiCols: db(r, c + quarterBlock4x4 * 3, subSize)

    def partition_cdf(self, r, c, bSize):
        bsl = T.Mi_Width_Log2[bSize]
        above = self.AvailU and (T.Mi_Width_Log2[self.MiSizes[r - 1][c]] < bsl)
        left = self.AvailL and (T.Mi_Height_Log2[self.MiSizes[r][c - 1]] < bsl)
        ctx = int(left) * 2 + int(above)
        return {1: self.cdf["Partition_W8"], 2: self.cdf["Partition_W16"], 3: self.cdf["Partition_W32"],
                4: self.cdf["Partition_W64"], 5: self.cdf["Partition_W128"]}[bsl][ctx]

    # ---- 5.11.5 decode_block -------------------------------------------------------------------
    def decode_block(self, r, c, subSize):
        h = self.h
        self.MiRow, self.MiCol, self.MiSize = r, c, subSize
        bw4 = T.Num_4x4_Blocks_Wide[subSize]
        bh4 = T.Num_4x4_Blocks_High[subSize]
        if bh4 == 1 and h.subsampling_y and (r & 1) == 0:
            self.HasChroma = 0
        elif bw4 == 1 and h.subsampling_x and (c & 1) == 0:
            self.HasChroma = 0
        else:
            self.HasChroma = 1 if h.NumPlanes > 1 else 0
        self.AvailU = self.is_inside(r - 1, c)
        self.AvailL = self.is_inside(r, c - 1)
        self.AvailUChroma = self.AvailU
        self.AvailLChroma = self.AvailL
        if self.HasChroma:
            if h.subsampling_y and bh4 == 1:
                self.AvailUChroma = self.is_inside(r - 2, c)
            if h.subsampling_x and bw4 == 1:
                self.AvailLChroma = self.is_inside(r, c - 2)
        else:
            self.AvailUChroma = self.AvailLChroma = 0
        self.intra_frame_mode_info()
        self.palette_tokens()
        self.read_block_tx_size()
        if self.skip:
            self.reset_block_context(bw4, bh4)
        for y in range(bh4):
            for x in range(bw4):
                if r + y < h.MiRows and c + x < h.MiCols:
                    self.YModes[r + y][c + x] = self.YMode
                    if self.HasChroma:
                        self.UVModes[r + y][c + x] = self.UVMode
        self.residual()
        for y in range(bh4):
            for x in range(bw4):
                if r + y < h.MiRows and c + x < h.MiCols:
                    self.Skips[r + y][c + x] = self.skip
                    self.TxSizes[r + y][c + x] = self.TxSize
                    self.MiSizes[r + y][c + x] = self.MiSize
                    self.SegmentIds[r + y][c + x] = self.segment_id
                    self.PaletteSizes[0][r + y][c + x] = self.PaletteSizeY
                    self.PaletteSizes[1][r + y][c + x] = self.PaletteSizeUV
                    self.PaletteColors[0][r + y][c + x] = list(self.palette_colors_y[:self.PaletteSizeY])
                    self.PaletteColors[1][r + y][c + x] = list(self.palette_colors_u[:self.PaletteSizeUV])
                    self.DeltaLFs[r + y][c + x] = list(self.DeltaLF)
        self.blocks.append(dict(r=r, c=c, size=subSize, skip=self.skip, ymode=self.YMode, uvmode=self.UVMode,
                                tx=self.TxSize, seg=self.segment_id, pal=(self.PaletteSizeY, self.PaletteSizeUV),
                                fi=self.use_filter_intra, angle=(self.AngleDeltaY, self.AngleDeltaUV),
                                cfl=(self.CflAlphaU, self.CflAlphaV)))

    def reset_block_context(self, bw4, bh4):
        h = self.h
        for plane in range(1 + 2 * self.HasChroma):
            subX = h.subsampling_x if plane > 0 else 0
            subY = h.subsampling_y if plane > 0 else 0
            for i in range(self.MiCol >> subX, (self.MiCol + bw4) >> subX):
                if i < h.MiCols:
                    self.AboveLevelContext[plane][i] = 0
                    self.AboveDcContext[plane][i] = 0
            for i in range(self.MiRow >> subY, (self.MiRow + bh4) >> subY):
                if i < h.MiRows:
                    self.LeftLevelContext[plane][i] = 0
                    self.LeftDcContext[plane][i] = 0

    # ---- 5.11.7 intra_frame_mode_info -----------------------------------------------------------
    def intra_frame_mode_info(self):
        h = self.h
        self.skip = 0
        if h.SegIdPreSkip:
            self.intra_segment_id()
        self.skip_mode = 0
        self.read_skip()
        if not h.SegIdPreSkip:
            self.intra_segment_id()
        self.read_cdef()
        self.read_delta_qindex()
        self.read_delta_lf()
        self.ReadDeltas = 0
        self.RefFrame = [INTRA_FRAME, NONE]
        self.use_intrabc = self.sym(self.cdf["Intrabc"], "use_intrabc") if h.allow_intrabc else 0
        assert not self.use_intrabc, "intrabc is not modelled"
        self.is_inter = 0
        self.PaletteSizeY = self.PaletteSizeUV = 0
        self.palette_colors_y = [0] * 8
        self.palette_colors_u = [0] * 8
        self.palette_colors_v = [0] * 8
        self.use_filter_intra = 0
        self.filter_intra_mode = 0
        self.AngleDeltaY = self.AngleDeltaUV = 0
        self.CflAlphaU = self.CflAlphaV = 0
        self.UVMode = DC_PRED
        abovemode = T.Intra_Mode_Context[self.YModes[self.MiRow - 1][self.MiCol] if self.AvailU else DC_PRED]
        leftmode = T.Intra_Mode_Context[self.YModes[self.MiRow][self.MiCol - 1] if self.AvailL else DC_PRED]
        self.YMode = self.sym(self.cdf["Intra_Frame_Y_Mode"][abovemode][leftmode], "intra_frame_y_mode")
        self.intra_angle_info_y()
        if self.HasChroma:
            self.UVMode = self.sym(self.uv_mode_cdf(), "uv_mode")
            if self.UVMode == UV_CFL_PRED:
                self.read_cfl_alphas()
            self.intra_angle_info_uv()
        if (self.MiSize >= BLOCK_8X8 and T.Block_Width[self.MiSize] <= 64 and T.Block_Height[self.MiSize] <= 64
                and h.allow_screen_content_tools):
            self.palette_mode_info()
        self.filter_intra_mode_info()

    def uv_mode_cdf(self):
        if self.Lossless and self.get_plane_residual_size(self.MiSize, 1) == BLOCK_4X4:
            return self.cdf["Uv_Mode_Cfl_Allowed"][self.YMode]
        if not self.Lossless and max(T.Block_Width[self.MiSize], T.Block_Height[self.MiSize]) <= 32:
            return self.cdf["Uv_Mode_Cfl_Allowed"][self.YMode]
        return self.cdf["Uv_Mode_Cfl_Not_Allowed"][self.YMode]

    def intra_segment_id(self):
        if self.h.segmentation_enabled:
            self.read_segment_id()
        else:
            self.segment_id = 0
        self.Lossless = self.h.LosslessArray[self.segment_id]

    def read_segment_id(self):
        r, c = self.MiRow, self.MiCol
        prevUL = self.SegmentIds[r - 1][c - 1] if (self.AvailU and self.AvailL) else -1
        prevU = self.SegmentIds[r - 1][c] if self.AvailU else -1
        prevL = self.SegmentIds[r][c - 1] if self.AvailL else -1
        if prevU == -1:
            pred = 0 if prevL == -1 else prevL
        elif prevL == -1:
            pred = prevU
        else:
            pred = prevU if prevUL == prevU else prevL
        if self.skip:
            self.segment_id = pred
        else:
            if prevUL < 0:
                ctx = 0
            elif prevUL == prevU and prevUL == prevL:
                ctx = 2
            elif prevUL == prevU or prevUL == prevL or prevU == prevL:
                ctx = 1
            else:
                ctx = 0
            self.seg_pred = pred          # (read by tools/gen_stream.py to keep generated ids conformant)
            sid = self.sym(self.cdf["Segment_Id"][ctx], "segment_id")
            self.segment_id = self.neg_deinterleave(sid, pred, self.h.LastActiveSegId + 1)

    @staticmethod
    def neg_deinterleave(diff, ref, mx):
        if not ref:
            return diff
        if ref >= mx - 1:
            return mx - diff - 1
        if 2 * ref < mx:
            if diff <= 2 * ref:
                return ref + ((diff + 1) >> 1) if (diff & 1) else ref - (diff >> 1)
            return diff
        if diff <= 2 * (mx - ref - 1):
            return ref + ((diff + 1) >> 1) if (diff & 1) else ref - (diff >> 1)
        return mx - (diff + 1)

    def read_skip(self):
        if self.h.SegIdPreSkip and self.seg_feature_active(SEG_LVL_SKIP):
            self.skip = 1
        else:
            ctx = 0
            if self.AvailU: ctx += self.Skips[self.MiRow - 1][self.MiCol]
            if self.AvailL: ctx += self.Skips[self.MiRow][self.MiCol - 1]
            self.skip = self.sym(self.cdf["Skip"][ctx], "skip")

    def read_cdef(self):
        h = self.h
        if self.skip or h.CodedLossless or not h.enable_cdef or h.allow_intrabc:
            return
        cdefSize4 = T.Num_4x4_Blocks_Wide[BLOCK_64X64]
        cdefMask4 = ~(cdefSize4 - 1)
        r = self.MiRow & cdefMask4
        c = self.MiCol & cdefMask4
        if self.cdef_idx.get((r, c), -1) == -1:
            v = self.L(h.cdef_bits, "cdef_idx")
            self.cdef_idx[(r, c)] = v
            w4 = T.Num_4x4_Blocks_Wide[self.MiSize]
            h4 = T.Num_4x4_Blocks_High[self.MiSize]
            for i in range(r, r + h4, cdefSize4):
                for j in range(c, c + w4, cdefSize4):
                    self.cdef_idx[(i, j)] = v

    def read_delta_qindex(self):
        h = self.h
        sbSize = BLOCK_128X128 if h.use_128x128_superblock else BLOCK_64X64
        if self.MiSize == sbSize and self.skip:
            return
        if self.ReadDeltas:
            delta_q_abs = self.sym(self.cdf["Delta_Q"], "delta_q_abs")
            if delta_q_abs == DELTA_Q_SMALL:
                delta_q_rem_bits = self.L(3, "delta_q_rem_bits") + 1
                delta_q_abs_bits = self.L(delta_q_rem_bits, "delta_q_abs_bits")
                delta_q_abs = delta_q_abs_bits + (1 << delta_q_rem_bits) + 1
            if delta_q_abs:
                sign = self.L(1, "delta_q_sign_bit")
                reduced = -delta_q_abs if sign else delta_q_abs
                self.CurrentQIndex = clip3(1, 255, self.CurrentQIndex + (reduced << h.delta_q_res))

    def read_delta_lf(self):
        h = self.h
        sbSize = BLOCK_128X128 if h.use_128x128_superblock else BLOCK_64X64
        if self.MiSize == sbSize and self.skip:
            return
        if self.ReadDeltas and h.delta_lf_present:
            frameLfCount = 1
            if h.delta_lf_multi:
                frameLfCount = FRAME_LF_COUNT if h.NumPlanes > 1 else FRAME_LF_COUNT - 2
            for i in range(frameLfCount):
                cdf = self.cdf["Delta_Lf_Multi"][i] if h.delta_lf_multi else self.cdf["Delta_Lf"]
                delta_lf_abs = self.sym(cdf, "delta_lf_abs")
                if delta_lf_abs == DELTA_LF_SMALL:
                    n = self.L(3, "delta_lf_rem_bits") + 1
                    bits = self.L(n, "delta_lf_abs_bits")
                    deltaLfAbs = bits + (1 << n) + 1
                else:
                    deltaLfAbs = delta_lf_abs
                if deltaLfAbs:
                    sign = self.L(1, "delta_lf_sign_bit")
                    reduced = -deltaLfAbs if sign else deltaLfAbs
                    self.DeltaLF[i] = clip3(-MAX_LOOP_FILTER, MAX_LOOP_FILTER, self.DeltaLF[i] + (reduced << h.delta_lf_res))

    def intra_angle_info_y(self):
        self.AngleDeltaY = 0
        if self.MiSize >= BLOCK_8X8 and is_directional_mode(self.YMode):
            v = self.sym(self.cdf["Angle_Delta"][self.YMode - V_PRED], "angle_delta_y")
            self.AngleDeltaY = v - MAX_ANGLE_DELTA

    def intra_angle_info_uv(self):
        self.AngleDeltaUV = 0
        if self.MiSize >= BLOCK_8X8 and is_directional_mode(self.UVMode):
            v = self.sym(self.cdf["Angle_Delta"][self.UVMode - V_PRED], "angle_delta_uv")
            self.AngleDeltaUV = v - MAX_ANGLE_DELTA

    def read_cfl_alphas(self):
        signs = self.sym(self.cdf["Cfl_Sign"], "cfl_alpha_signs")
        signU = (signs + 1) // 3
        signV = (signs + 1) % 3
        if signU != CFL_SIGN_ZERO:
            a = self.sym(self.cdf["Cfl_Alpha"][(signU - 1) * 3 + signV], "cfl_alpha_u")
            self.CflAlphaU = -(1 + a) if signU == CFL_SIGN_NEG else 1 + a
        else:
            self.CflAlphaU = 0
        if signV != CFL_SIGN_ZERO:
            a = self.sym(self.cdf["Cfl_Alpha"][(signV - 1) * 3 + signU], "cfl_alpha_v")
            self.CflAlphaV = -(1 + a) if signV == CFL_SIGN_NEG else 1 + a
        else:
            self.CflAlphaV = 0

    # ---- palette ----------------------------------------------------------------------------------
    def palette_mode_info(self):
        h = self.h
        bd = h.BitDepth
        bsizeCtx = T.Mi_Width_Log2[self.MiSize] + T.Mi_Height_Log2[self.MiSize] - 2
        if self.YMode == DC_PRED:
            ctx = 0
            if self.AvailU and self.PaletteSizes[0][self.MiRow - 1][self.MiCol] > 0: ctx += 1
            if self.AvailL and self.PaletteSizes[0][self.MiRow][self.MiCol - 1] > 0: ctx += 1
            has_palette_y = self.sym(self.cdf["Palette_Y_Mode"][bsizeCtx][ctx], "has_palette_y")
            if has_palette_y:
                self.PaletteSizeY = self.sym(self.cdf["Palette_Y_Size"][bsizeCtx], "palette_size_y_minus_2") + 2
                cache = self.get_palette_cache(0)
                idx = 0
                i = 0
                while i < len(cache) and idx < self.PaletteSizeY:
                    if self.L(1, "use_palette_color_cache_y"):
                        self.palette_colors_y[idx] = cache[i]
                        idx += 1
                    i += 1
                if idx < self.PaletteSizeY:
                    self.palette_colors_y[idx] = self.L(bd, "palette_colors_y")
                    idx += 1
                if idx < self.PaletteSizeY:
                    minBits = bd - 3
                    paletteBits = minBits + self.L(2, "palette_num_extra_bits_y")
                while idx < self.PaletteSizeY:
                    delta = self.L(paletteBits, "palette_delta_y") + 1
                    self.palette_colors_y[idx] = clip3(0, (1 << bd) - 1, self.palette_colors_y[idx - 1] + delta)
                    rng = (1 << bd) - self.palette_colors_y[idx] - 1
                    paletteBits = min(paletteBits, ceil_log2(rng))
                    idx += 1
                self.palette_colors_y[:self.PaletteSizeY] = sorted(self.palette_colors_y[:self.PaletteSizeY])
        if self.HasChroma and self.UVMode == DC_PRED:
            ctx = 1 if self.PaletteSizeY > 0 else 0
            has_palette_uv = self.sym(self.cdf["Palette_Uv_Mode"][ctx], "has_palette_uv")
            if has_palette_uv:
                self.PaletteSizeUV = self.sym(self.cdf["Palette_Uv_Size"][bsizeCtx], "palette_size_uv_minus_2") + 2
                cache = self.get_palette_cache(1)
                idx = 0
                i = 0
                while i < len(cache) and idx < self.PaletteSizeUV:
                    if self.L(1, "use_palette_color_cache_u"):
                        self.palette_colors_u[idx] = cache[i]
                        idx += 1
                    i += 1
                if idx < self.PaletteSizeUV:
                    self.palette_colors_u[idx] = self.L(bd, "palette_colors_u")
                    idx += 1
                if idx < self.PaletteSizeUV:
                    minBits = bd - 3
                    paletteBits = minBits + self.L(2, "palette_num_extra_bits_u")
                while idx < self.PaletteSizeUV:
                    delta = self.L(paletteBits, "palette_delta_u")
                    self.palette_colors_u[idx] = clip3(0, (1 << bd) - 1, self.palette_colors_u[idx - 1] + delta)
                    rng = (1 << bd) - self.palette_colors_u[idx]
                    paletteBits = min(paletteBits, ceil_log2(rng))
                    idx += 1
                self.palette_colors_u[:self.PaletteSizeUV] = sorted(self.palette_colors_u[:self.PaletteSizeUV])
                if self.L(1, "delta_encode_palette_colors_v"):
                    minBits = bd - 4
                    maxVal = 1 << bd
                    paletteBits = minBits + self.L(2, "palette_num_extra_bits_v")
                    self.palette_colors_v[0] = self.L(bd, "palette_colors_v")
                    for idx in range(1, self.PaletteSizeUV):
                        delta = self.L(paletteBits, "palette_delta_v")
                        if delta:
                            if self.L(1, "palette_delta_sign_bit_v"):
                                delta = -delta
                        val = self.palette_colors_v[idx - 1] + delta
                        if val < 0: val += maxVal
                        if val >= maxVal: val -= maxVal
                        self.palette_colors_v[idx] = clip3(0, (1 << bd) - 1, val)
                else:
                    for idx in range(self.PaletteSizeUV):
                        self.palette_colors_v[idx] = self.L(bd, "palette_colors_v")

    def get_palette_cache(self, plane):
        aboveN = 0
        if (self.MiRow * MI_SIZE) % 64:
            aboveN = self.PaletteSizes[plane][self.MiRow - 1][self.MiCol]
        leftN = self.PaletteSizes[plane][self.MiRow][self.MiCol - 1] if self.AvailL else 0
        above = self.PaletteColors[plane][self.MiRow - 1][self.MiCol] if aboveN else []
        left = self.PaletteColors[plane][self.MiRow][self.MiCol - 1] if leftN else []
        aboveIdx = leftIdx = 0
        cache = []
        while aboveIdx < aboveN and leftIdx < leftN:
            aboveC = above[aboveIdx]
            leftC = left[leftIdx]
            if leftC < aboveC:
                if not cache or leftC != cache[-1]:
                    cache.append(leftC)
                leftIdx += 1
            else:
                if not cache or aboveC != cache[-1]:
                    cache.append(aboveC)
                aboveIdx += 1
                if leftC == aboveC:
                    leftIdx += 1
        while aboveIdx < aboveN:
            val = above[aboveIdx]; aboveIdx += 1
            if not cache or val != cache[-1]:
                cache.append(val)
        while leftIdx < leftN:
            val = left[leftIdx]; leftIdx += 1
            if not cache or val != cache[-1]:
                cache.append(val)
        return cache

    def get_palette_color_context(self, colorMap, r, c, n):
        scores = [0] * PALETTE_COLORS
        order = list(range(PALETTE_COLORS))
        if c > 0:
            scores[colorMap[r][c - 1]] += 2
        if r > 0 and c > 0:
            scores[colorMap[r - 1][c - 1]] += 1
        if r > 0:
            scores[colorMap[r - 1][c]] += 2
        for i in range(PALETTE_NUM_NEIGHBORS):
            maxScore = scores[i]
            maxIdx = i
            for j in range(i + 1, n):
                if scores[j] > maxScore:
                    maxScore = scores[j]
                    maxIdx = j
            if maxIdx != i:
                maxScore = scores[maxIdx]
                maxColorOrder = order[maxIdx]
                for k in range(maxIdx, i, -1):
                    scores[k] = scores[k - 1]
                    order[k] = order[k - 1]
                scores[i] = maxScore
                order[i] = maxColorOrder
        hsh = 0
        for i in range(PALETTE_NUM_NEIGHBORS):
            hsh += scores[i] * T.Palette_Color_Hash_Multipliers[i]
        return T.Palette_Color_Context[hsh], order

    def palette_tokens(self):
        h = self.h
        blockHeight = T.Block_Height[self.MiSize]
        blockWidth = T.Block_Width[self.MiSize]
        onscreenHeight = min(blockHeight, (h.MiRows - self.MiRow) * MI_SIZE)
        onscreenWidth = min(blockWidth, (h.MiCols - self.MiCol) * MI_SIZE)
        self.ColorMapY = None
        self.ColorMapUV = None
        if self.PaletteSizeY:
            self.ColorMapY = self._palette_map(blockWidth, blockHeight, onscreenWidth, onscreenHeight,
                                               self.PaletteSizeY, "y")
        if self.PaletteSizeUV:
            blockHeight >>= h.subsampling_y
            blockWidth >>= h.subsampling_x
            onscreenHeight >>= h.subsampling_y
            onscreenWidth >>= h.subsampling_x
            if blockWidth < 4:
                blockWidth += 2
                onscreenWidth += 2
            if blockHeight < 4:
                blockHeight += 2
                onscreenHeight += 2
            self.ColorMapUV = self._palette_map(blockWidth, blockHeight, onscreenWidth, onscreenHeight,
                                                self.PaletteSizeUV, "uv")

    def _palette_map(self, blockWidth, blockHeight, onscreenWidth, onscreenHeight, n, tag):
        cmap = [[0] * blockWidth for _ in range(blockHeight)]
        cmap[0][0] = self.dec.read_ns(n, "color_index_map_" + tag)
        cdfs = self.cdf["Palette_Size_%d_%s_Color" % (n, "Y" if tag == "y" else "Uv")]
        for i in range(1, onscreenHeight + onscreenWidth - 1):
            j = min(i, onscreenWidth - 1)
            while j >= max(0, i - onscreenHeight + 1):
                ctx, order = self.get_palette_color_context(cmap, i - j, j, n)
                idx = self.sym(cdfs[ctx], "palette_color_idx_" + tag)
                cmap[i - j][j] = order[idx]
                j -= 1
        for i in range(onscreenHeight):
            for j in range(onscreenWidth, blockWidth):
                cmap[i][j] = cmap[i][onscreenWidth - 1]
        for i in range(onscreenHeight, blockHeight):
            for j in range(blockWidth):
                cmap[i][j] = cmap[onscreenHeight - 1][j]
        return cmap

    def filter_intra_mode_info(self):
        self.use_filter_intra = 0
        if (self.h.enable_filter_intra and self.YMode == DC_PRED and self.PaletteSizeY == 0
                and max(T.Block_Width[self.MiSize], T.Block_Height[self.MiSize]) <= 32):
            self.use_filter_intra = self.sym(self.cdf["Filter_Intra"][self.MiSize], "use_filter_intra")
            if self.use_filter_intra:
                self.filter_intra_mode = self.sym(self.cdf["Filter_Intra_Mode"], "filter_intra_mode")

    # ---- tx size -----------------------------------------------------------------------------------
    def read_block_tx_size(self):
        h = self.h
        bw4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        bh4 = T.Num_4x4_Blocks_High[self.MiSize]
        # intra blocks: never the var-tx path
        self.read_tx_size(allowSelect=(not self.skip) or (not self.is_inter))
        for row in range(self.MiRow, self.MiRow + bh4):
            for col in range(self.MiCol, self.MiCol + bw4):
                if row < h.MiRows and col < h.MiCols:
                    self.InterTxSizes[row][col] = self.TxSize

    def read_tx_size(self, allowSelect):
        if self.Lossless:
            self.TxSize = TX_4X4
            return
        maxRectTxSize = T.Max_Tx_Size_Rect[self.MiSize]
        maxTxDepth = T.Max_Tx_Depth[self.MiSize]
        self.TxSize = maxRectTxSize
        if self.MiSize > BLOCK_4X4 and allowSelect and self.h.TxMode == TX_MODE_SELECT:
            tx_depth = self.sym(self.tx_depth_cdf(maxRectTxSize, maxTxDepth), "tx_depth")
            for _ in range(tx_depth):
                self.TxSize = T.Split_Tx_Size[self.TxSize]

    def tx_depth_cdf(self, maxRectTxSize, maxTxDepth):
        maxTxWidth = T.Tx_Width[maxRectTxSize]
        maxTxHeight = T.Tx_Height[maxRectTxSize]
        r, c = self.MiRow, self.MiCol
        # neighbours are intra (IsInters == 0) in an intra frame
        aboveW = self.get_above_tx_width(r, c) if self.AvailU else 0
        leftH = self.get_left_tx_height(r, c) if self.AvailL else 0
        ctx = int(aboveW >= maxTxWidth) + int(leftH >= maxTxHeight)
        if maxTxDepth == 4: return self.cdf["Tx_64x64"][ctx]
        if maxTxDepth == 3: return self.cdf["Tx_32x32"][ctx]
        if maxTxDepth == 2: return self.cdf["Tx_16x16"][ctx]
        return self.cdf["Tx_8x8"][ctx]

    def get_above_tx_width(self, row, col):
        if row == self.MiRow:
            if not self.AvailU:
                return 64
            # Skips && IsInters never both true in an intra frame
        return T.Tx_Width[self.InterTxSizes[row - 1][col]]

    def get_left_tx_height(self, row, col):
        if col == self.MiCol:
            if not self.AvailL:
                return 64
        return T.Tx_Height[self.InterTxSizes[row][col - 1]]

    # ---- residual ---------------------------------------------------------------------------------
    def get_plane_residual_size(self, subsize, plane):
        subx = self.h.subsampling_x if plane > 0 else 0
        suby = self.h.subsampling_y if plane > 0 else 0
        return T.Subsampled_Size[subsize][subx][suby]

    def get_tx_size(self, plane, txSz):
        if plane == 0:
            return txSz
        uvTx = T.Max_Tx_Size_Rect[self.get_plane_residual_size(self.MiSize, plane)]
        if T.Tx_Width[uvTx] == 64 or T.Tx_Height[uvTx] == 64:
            if T.Tx_Width[uvTx] == 16:
                return TX_16X32
            if T.Tx_Height[uvTx] == 16:
                return TX_32X16
            return TX_32X32
        return uvTx

    def residual(self):
        h = self.h
        sbMask = 31 if h.use_128x128_superblock else 15
        widthChunks = max(1, T.Block_Width[self.MiSize] >> 6)
        heightChunks = max(1, T.Block_Height[self.MiSize] >> 6)
        miSizeChunk = BLOCK_64X64 if (widthChunks > 1 or heightChunks > 1) else self.MiSize
        for chunkY in range(heightChunks):
            for chunkX in range(widthChunks):
                miRowChunk = self.MiRow + (chunkY << 4)
                miColChunk = self.MiCol + (chunkX << 4)
                for plane in range(1 + self.HasChroma * 2):
                    txSz = TX_4X4 if self.Lossless else self.get_tx_size(plane, self.TxSize)
                    stepX = T.Tx_Width[txSz] >> 2
                    stepY = T.Tx_Height[txSz] >> 2
                    planeSz = self.get_plane_residual_size(miSizeChunk, plane)
                    num4x4W = T.Num_4x4_Blocks_Wide[planeSz]
                    num4x4H = T.Num_4x4_Blocks_High[planeSz]
                    subX = h.subsampling_x if plane > 0 else 0
                    subY = h.subsampling_y if plane > 0 else 0
                    baseXBlock = (self.MiCol >> subX) * MI_SIZE
                    baseYBlock = (self.MiRow >> subY) * MI_SIZE
                    y = 0
                    while y < num4x4H:
                        x = 0
                        while x < num4x4W:
                            self.transform_block(plane, baseXBlock, baseYBlock, txSz,
                                                 x + ((chunkX << 4) >> subX), y + ((chunkY << 4) >> subY))
                            x += stepX
                        y += stepY

    def transform_block(self, plane, baseX, baseY, txSz, x, y):
        h = self.h
        startX = baseX + 4 * x
        startY = baseY + 4 * y
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        row = (startY << subY) >> 2
        col = (startX << subX) >> 2
        sbMask = 31 if h.use_128x128_superblock else 15
        subBlockMiRow = row & sbMask
        subBlockMiCol = col & sbMask
        stepX = T.Tx_Width[txSz] >> 2
        stepY = T.Tx_Height[txSz] >> 2
        maxX = (h.MiCols * MI_SIZE) >> subX
        maxY = (h.MiRows * MI_SIZE) >> subY
        if startX >= maxX or startY >= maxY:
            return
        # prediction hook (reconstruction models override; the syntax model itself predicts nothing)
        self.predict_block(plane, startX, startY, txSz, x, y, subX, subY, subBlockMiRow, subBlockMiCol, stepX, stepY)
        if not self.skip:
            eob = self.coeffs(plane, startX, startY, txSz)
            if eob > 0:
                rows = self.reconstruct_dequant(plane, startX, startY, txSz, eob)
                self.reconstruct_block(plane, startX, startY, txSz, rows)
        lts = self.LoopfilterTxSizes[plane]
        for i in range(stepY):
            for j in range(stepX):
                rr, cc = (row >> subY) + i, (col >> subX) + j
                if rr < len(lts) and cc < len(lts[0]):
                    lts[rr][cc] = txSz
        self.after_transform_block(plane, row, col, subX, subY, stepX, stepY, subBlockMiRow, subBlockMiCol)

    # hooks for reconstruction models
    def clear_block_decoded_flags(self, r, c, sbSize4):
        pass

    def predict_block(self, plane, startX, startY, txSz, x, y, subX, subY, sbMiRow, sbMiCol, stepX, stepY):
        pass

    def reconstruct_block(self, plane, startX, startY, txSz, dequant_rows):
        pass

    def after_transform_block(self, plane, row, col, subX, subY, stepX, stepY, sbMiRow, sbMiCol):
        pass

    # ---- coefficients ---------------------------------------------------------------------------------
    def get_tx_set(self, txSz):
        txSzSqr = T.Tx_Size_Sqr[txSz]
        txSzSqrUp = T.Tx_Size_Sqr_Up[txSz]
        if txSzSqrUp > TX_32X32:
            return TX_SET_DCTONLY
        # intra
        if txSzSqrUp == TX_32X32: return TX_SET_DCTONLY
        if self.h.reduced_tx_set: return TX_SET_INTRA_2
        if txSzSqr == TX_16X16: return TX_SET_INTRA_2
        return TX_SET_INTRA_1

    def transform_type(self, x4, y4, txSz):
        h = self.h
        s = self.get_tx_set(txSz)
        qidx = self.get_qindex(1, self.segment_id) if h.segmentation_enabled else h.base_q_idx
        if s > 0 and qidx > 0:
            intraDir = T.Filter_Intra_Mode_To_Intra_Dir[self.filter_intra_mode] if self.use_filter_intra else self.YMode
            if s == TX_SET_INTRA_1:
                v = self.sym(self.cdf["Intra_Tx_Type_Set1"][T.Tx_Size_Sqr[txSz]][intraDir], "intra_tx_type")
                TxType = T.Tx_Type_Intra_Inv_Set1[v]
            else:
                v = self.sym(self.cdf["Intra_Tx_Type_Set2"][T.Tx_Size_Sqr[txSz]][intraDir], "intra_tx_type")
                TxType = T.Tx_Type_Intra_Inv_Set2[v]
        else:
            TxType = DCT_DCT
        for i in range(T.Tx_Width[txSz] >> 2):
            for j in range(T.Tx_Height[txSz] >> 2):
                if y4 + j < h.MiRows and x4 + i < h.MiCols:
                    self.TxTypes[y4 + j][x4 + i] = TxType

    def compute_tx_type(self, plane, txSz, blockX, blockY):
        txSzSqrUp = T.Tx_Size_Sqr_Up[txSz]
        if self.Lossless or txSzSqrUp > TX_32X32:
            return DCT_DCT
        txSet = self.get_tx_set(txSz)
        if plane == 0:
            return self.TxTypes[blockY][blockX]
        txType = T.Mode_To_Txfm[self.UVMode]
        if not T.Tx_Type_In_Set_Intra[txSet][txType]:
            return DCT_DCT
        return txType

    @staticmethod
    def get_tx_class(txType):
        if txType in (V_DCT, V_ADST, V_FLIPADST): return TX_CLASS_VERT
        if txType in (H_DCT, H_ADST, H_FLIPADST): return TX_CLASS_HORIZ
        return TX_CLASS_2D

    def get_scan(self, txSz, PlaneTxType):
        if txSz == TX_16X64: return T.Default_Scan_16x32
        if txSz == TX_64X16: return T.Default_Scan_32x16
        if T.Tx_Size_Sqr_Up[txSz] == TX_64X64: return T.Default_Scan_32x32
        w, hh = T.Tx_Width[txSz], T.Tx_Height[txSz]
        if PlaneTxType == IDTX:
            return getattr(T, "Default_Scan_%dx%d" % (w, hh))
        preferRow = PlaneTxType in (V_DCT, V_ADST, V_FLIPADST)
        preferCol = PlaneTxType in (H_DCT, H_ADST, H_FLIPADST)
        if preferRow:
            return getattr(T, "Mrow_Scan_%dx%d" % (w, hh))
        if preferCol:
            return getattr(T, "Mcol_Scan_%dx%d" % (w, hh))
        return getattr(T, "Default_Scan_%dx%d" % (w, hh))

    def coeffs(self, plane, startX, startY, txSz):
        h = self.h
        x4 = startX >> 2
        y4 = startY >> 2
        w4 = T.Tx_Width[txSz] >> 2
        h4 = T.Tx_Height[txSz] >> 2
        txSzCtx = (T.Tx_Size_Sqr[txSz] + T.Tx_Size_Sqr_Up[txSz] + 1) >> 1
        ptype = 1 if plane > 0 else 0
        segEob = 512 if txSz in (TX_16X64, TX_64X16) else min(1024, T.Tx_Width[txSz] * T.Tx_Height[txSz])
        Quant = self.Quant
        for c in range(segEob):
            Quant[c] = 0
        eob = 0
        culLevel = 0
        dcCategory = 0
        all_zero = self.sym(self.cdf["Txb_Skip"][txSzCtx][self.all_zero_ctx(plane, txSz, x4, y4, w4, h4)], "all_zero")
        if all_zero:
            if plane == 0:
                for i in range(w4):
                    for j in range(h4):
                        if y4 + j < h.MiRows and x4 + i < h.MiCols:
                            self.TxTypes[y4 + j][x4 + i] = DCT_DCT
            self.PlaneTxType = DCT_DCT
        else:
            if plane == 0:
                self.transform_type(x4, y4, txSz)
            PlaneTxType = self.compute_tx_type(plane, txSz, x4, y4)
            self.PlaneTxType = PlaneTxType
            scan = self.get_scan(txSz, PlaneTxType)
            txClass = self.get_tx_class(PlaneTxType)
            eobMultisize = min(T.Tx_Width_Log2[txSz], 5) + min(T.Tx_Height_Log2[txSz], 5) - 4
            ctx2d = 0 if txClass == TX_CLASS_2D else 1
            names = ["Eob_Pt_16", "Eob_Pt_32", "Eob_Pt_64", "Eob_Pt_128", "Eob_Pt_256", "Eob_Pt_512", "Eob_Pt_1024"]
            if eobMultisize <= 4:
                eobPt = self.sym(self.cdf[names[eobMultisize]][ptype][ctx2d], names[eobMultisize].lower()) + 1
            else:
                eobPt = self.sym(self.cdf[names[eobMultisize]][ptype], names[eobMultisize].lower()) + 1
            eob = eobPt if eobPt < 2 else (1 << (eobPt - 2)) + 1
            eobShift = max(-1, eobPt - 3)
            if eobShift >= 0:
                eob_extra = self.sym(self.cdf["Eob_Extra"][txSzCtx][ptype][eobPt - 3], "eob_extra")
                if eob_extra:
                    eob += 1 << eobShift
                for i in range(1, max(0, eobPt - 2)):
                    eobShift = max(0, eobPt - 2) - 1 - i
                    if self.L(1, "eob_extra_bit"):
                        eob += 1 << eobShift
            adjTxSz = T.Adjusted_Tx_Size[txSz]
            bwl = T.Tx_Width_Log2[adjTxSz]
            width = 1 << bwl
            height = T.Tx_Height[adjTxSz]
            for c in range(eob - 1, -1, -1):
                pos = scan[c]
                if c == eob - 1:
                    ctx = self.coeff_base_ctx(txSz, pos, c, True, bwl, width, height, txClass) - SIG_COEF_CONTEXTS + SIG_COEF_CONTEXTS_EOB
                    level = self.sym(self.cdf["Coeff_Base_Eob"][txSzCtx][ptype][ctx], "coeff_base_eob") + 1
                else:
                    ctx = self.coeff_base_ctx(txSz, pos, c, False, bwl, width, height, txClass)
                    level = self.sym(self.cdf["Coeff_Base"][txSzCtx][ptype][ctx], "coeff_base")
                if level > NUM_BASE_LEVELS:
                    brctx = self.coeff_br_ctx(txSz, pos, bwl, width, height, txClass)
                    for idx in range(COEFF_BASE_RANGE // (BR_CDF_SIZE - 1)):
                        coeff_br = self.sym(self.cdf["Coeff_Br"][min(txSzCtx, TX_32X32)][ptype][brctx], "coeff_br")
                        level += coeff_br
                        if coeff_br < BR_CDF_SIZE - 1:
                            break
                Quant[pos] = level
            for c in range(eob):
                pos = scan[c]
                if Quant[pos] != 0:
                    if c == 0:
                        sign = self.sym(self.cdf["Dc_Sign"][ptype][self.dc_sign_ctx(plane, x4, y4, w4, h4)], "dc_sign")
                    else:
                        sign = self.L(1, "sign_bit")
                else:
                    sign = 0
                if Quant[pos] > NUM_BASE_LEVELS + COEFF_BASE_RANGE:
                    length = 0
                    while True:
                        length += 1
                        if self.L(1, "golomb_length_bit"):
                            break
                        assert length < 32
                    x = 1
                    for i in range(length - 2, -1, -1):
                        x = (x << 1) | self.L(1, "golomb_data_bit")
                    Quant[pos] = x + COEFF_BASE_RANGE + NUM_BASE_LEVELS
                if pos == 0 and Quant[pos] > 0:
                    dcCategory = 1 if sign else 2
                Quant[pos] = Quant[pos] & 0xFFFFF
                culLevel += Quant[pos]
                if sign:
                    Quant[pos] = -Quant[pos]
            culLevel = min(63, culLevel)
        for i in range(w4):
            if x4 + i < len(self.AboveLevelContext[plane]):
                self.AboveLevelContext[plane][x4 + i] = culLevel
                self.AboveDcContext[plane][x4 + i] = dcCategory
        for i in range(h4):
            if y4 + i < len(self.LeftLevelContext[plane]):
                self.LeftLevelContext[plane][y4 + i] = culLevel
                self.LeftDcContext[plane][y4 + i] = dcCategory
        return eob

    def all_zero_ctx(self, plane, txSz, x4, y4, w4, h4):
        h = self.h
        maxX4, maxY4 = h.MiCols, h.MiRows
        if plane > 0:
            maxX4 >>= h.subsampling_x
            maxY4 >>= h.subsampling_y
        w = T.Tx_Width[txSz]
        hh = T.Tx_Height[txSz]
        bsize = self.get_plane_residual_size(self.MiSize, plane)
        bw = T.Block_Width[bsize]
        bh = T.Block_Height[bsize]
        if plane == 0:
            top = left = 0
            for k in range(w4):
                if x4 + k < maxX4:
                    top = max(top, self.AboveLevelContext[plane][x4 + k])
            for k in range(h4):
                if y4 + k < maxY4:
                    left = max(left, self.LeftLevelContext[plane][y4 + k])
            top = min(top, 255)
            left = min(left, 255)
            if bw == w and bh == hh: return 0
            if top == 0 and left == 0: return 1
            if top == 0 or left == 0: return 2 + int(max(top, left) > 3)
            if max(top, left) <= 3: return 4
            if min(top, left) <= 3: return 5
            return 6
        above = left = 0
        for i in range(w4):
            if x4 + i < maxX4:
                above |= self.AboveLevelContext[plane][x4 + i]
                above |= self.AboveDcContext[plane][x4 + i]
        for i in range(h4):
            if y4 + i < maxY4:
                left |= self.LeftLevelContext[plane][y4 + i]
                left |= self.LeftDcContext[plane][y4 + i]
        ctx = int(above != 0) + int(left != 0)
        ctx += 7
        if bw * bh > w * hh:
            ctx += 3
        return ctx

    def coeff_base_ctx(self, txSz, pos, c, isEob, bwl, width, height, txClass):
        if isEob:
            if c == 0: return SIG_COEF_CONTEXTS - 4
            if c <= (height << bwl) // 8: return SIG_COEF_CONTEXTS - 3
            if c <= (height << bwl) // 4: return SIG_COEF_CONTEXTS - 2
            return SIG_COEF_CONTEXTS - 1
        row = pos >> bwl
        col = pos - (row << bwl)
        mag = 0
        Quant = self.Quant
        for idx in range(5):
            refRow = row + T.Sig_Ref_Diff_Offset[txClass][idx][0]
            refCol = col + T.Sig_Ref_Diff_Offset[txClass][idx][1]
            if 0 <= refRow < height and 0 <= refCol < width:
                mag += min(abs(Quant[(refRow << bwl) + refCol]), 3)
        ctx = min((mag + 1) >> 1, 4)
        if txClass == TX_CLASS_2D:
            if row == 0 and col == 0:
                return 0
            return ctx + T.Coeff_Base_Ctx_Offset[txSz][min(row, 4)][min(col, 4)]
        idx = row if txClass == TX_CLASS_VERT else col
        return ctx + T.Coeff_Base_Pos_Ctx_Offset[min(idx, 2)]

    def coeff_br_ctx(self, txSz, pos, bwl, width, height, txClass):
        txw = width
        row = pos >> bwl
        col = pos - (row << bwl)
        mag = 0
        Quant = self.Quant
        for idx in range(3):
            refRow = row + T.Mag_Ref_Offset_With_Tx_Class[txClass][idx][0]
            refCol = col + T.Mag_Ref_Offset_With_Tx_Class[txClass][idx][1]
            if 0 <= refRow < height and 0 <= refCol < (1 << bwl):
                mag += min(Quant[refRow * txw + refCol], COEFF_BASE_RANGE + NUM_BASE_LEVELS + 1)
        mag = min((mag + 1) >> 1, 6)
        if pos == 0:
            return mag
        if txClass == 0:
            return mag + 7 if (row < 2 and col < 2) else mag + 14
        if txClass == 1:
            return mag + 7 if col == 0 else mag + 14
        return mag + 7 if row == 0 else mag + 14

    def dc_sign_ctx(self, plane, x4, y4, w4, h4):
        h = self.h
        maxX4, maxY4 = h.MiCols, h.MiRows
        if plane > 0:
            maxX4 >>= h.subsampling_x
            maxY4 >>= h.subsampling_y
        dcSign = 0
        for k in range(w4):
            if x4 + k < maxX4:
                s = self.AboveDcContext[plane][x4 + k]
                if s == 1: dcSign -= 1
                elif s == 2: dcSign += 1
        for k in range(h4):
            if y4 + k < maxY4:
                s = self.LeftDcContext[plane][y4 + k]
                if s == 1: dcSign -= 1
                elif s == 2: dcSign += 1
        return 1 if dcSign < 0 else 2 if dcSign > 0 else 0

    # ---- 7.12.3 dequantisation (the part of "reconstruct" that produces Dequant) ----------------------
    def get_dc_quant(self, plane):
        h = self.h
        q = self.get_qindex(0, self.segment_id)
        delta = [h.DeltaQYDc, h.DeltaQUDc, h.DeltaQVDc][plane]
        return T.Dc_Qlookup[(h.BitDepth - 8) >> 1][clip3(0, 255, q + delta)]

    def get_ac_quant(self, plane):
        h = self.h
        q = self.get_qindex(0, self.segment_id)
        delta = [0, h.DeltaQUAc, h.DeltaQVAc][plane]
        return T.Ac_Qlookup[(h.BitDepth - 8) >> 1][clip3(0, 255, q + delta)]

    def reconstruct_dequant(self, plane, x, y, txSz, eob):
        h = self.h
        if txSz in (TX_32X32, TX_16X32, TX_32X16, TX_16X64, TX_64X16):
            dqDenom = 2
        elif txSz in (TX_64X64, E["TX_32X64"], E["TX_64X32"]):
            dqDenom = 4
        else:
            dqDenom = 1
        w = T.Tx_Width[txSz]
        hh = T.Tx_Height[txSz]
        tw = min(32, w)
        th = min(32, hh)
        rows = []
        lim = 1 << (7 + h.BitDepth)
        for i in range(th):
            row = []
            for j in range(tw):
                q = self.get_dc_quant(plane) if (i == 0 and j == 0) else self.get_ac_quant(plane)
                if h.using_qmatrix and self.PlaneTxType < IDTX and h.SegQMLevel[plane][self.segment_id] < 15:
                    qm = T.Quantizer_Matrix[h.SegQMLevel[plane][self.segment_id]][1 if plane > 0 else 0][T.Qm_Offset[txSz] + i * tw + j]
                    q2 = round2(q * qm, 5)
                else:
                    q2 = q
                dq = self.Quant[i * tw + j] * q2
                sign = -1 if dq < 0 else 1
                dq2 = sign * ((abs(dq) & 0xFFFFFF) // dqDenom)
                row.append(clip3(-lim, lim - 1, dq2))
            rows.append(row)
        # dav1d labels lossless blocks WHT_WHT (16); the spec keeps PlaneTxType and uses the Lossless flag
        self.events.append(("C", plane, x >> 2, y >> 2, txSz, 16 if self.Lossless else self.PlaneTxType, eob, rows))
        return rows

    # ---- loop restoration units -----------------------------------------------------------------------
    def read_lr(self, r, c, bSize):
        h = self.h
        if h.allow_intrabc:
            return
        w = T.Num_4x4_Blocks_Wide[bSize]
        hh = T.Num_4x4_Blocks_High[bSize]
        for plane in range(h.NumPlanes):
            if h.FrameRestorationType[plane] != RESTORE_NONE:
                subX = 0 if plane == 0 else h.subsampling_x
                subY = 0 if plane == 0 else h.subsampling_y
                unitSize = h.LoopRestorationSize[plane]
                unitRows = self.count_units_in_frame(unitSize, round2(h.FrameHeight, subY))
                unitCols = self.count_units_in_frame(unitSize, round2(h.UpscaledWidth, subX))
                unitRowStart = (r * (MI_SIZE >> subY) + unitSize - 1) // unitSize
                unitRowEnd = min(unitRows, ((r + hh) * (MI_SIZE >> subY) + unitSize - 1) // unitSize)
                if h.use_superres:
                    numerator = (MI_SIZE >> subX) * h.SuperresDenom
                    denominator = unitSize * SUPERRES_NUM
                else:
                    numerator = MI_SIZE >> subX
                    denominator = unitSize
                unitColStart = (c * numerator + denominator - 1) // denominator
                unitColEnd = min(unitCols, ((c + w) * numerator + denominator - 1) // denominator)
                for unitRow in range(unitRowStart, unitRowEnd):
                    for unitCol in range(unitColStart, unitColEnd):
                        self.read_lr_unit(plane, unitRow, unitCol)

    @staticmethod
    def count_units_in_frame(unitSize, frameSize):
        return max((frameSize + (unitSize >> 1)) // unitSize, 1)

    def read_lr_unit(self, plane, unitRow, unitCol):
        h = self.h
        frt = h.FrameRestorationType[plane]
        if frt == RESTORE_WIENER:
            restoration_type = RESTORE_WIENER if self.sym(self.cdf["Use_Wiener"], "use_wiener") else RESTORE_NONE
        elif frt == RESTORE_SGRPROJ:
            restoration_type = RESTORE_SGRPROJ if self.sym(self.cdf["Use_Sgrproj"], "use_sgrproj") else RESTORE_NONE
        else:
            restoration_type = self.sym(self.cdf["Restoration_Type"], "restoration_type")
        key = (plane, unitRow, unitCol)
        self.LrType[key] = restoration_type
        if restoration_type == RESTORE_WIENER:
            coefs = [[0, 0, 0], [0, 0, 0]]
            for pss in range(2):
                firstCoeff = 1 if plane else 0
                for j in range(firstCoeff, 3):
                    mn = T.Wiener_Taps_Min[j]
                    mx = T.Wiener_Taps_Max[j]
                    k = T.Wiener_Taps_K[j]
                    v = self.decode_signed_subexp_with_ref_bool(mn, mx + 1, k, self.RefLrWiener[plane][pss][j])
                    coefs[pss][j] = v
                    self.RefLrWiener[plane][pss][j] = v
            self.LrWiener[key] = coefs
        elif restoration_type == RESTORE_SGRPROJ:
            lr_sgr_set = self.L(SGRPROJ_PARAMS_BITS, "lr_sgr_set")
            self.LrSgrSet[key] = lr_sgr_set
            xqd = [0, 0]
            for i in range(2):
                radius = T.Sgr_Params[lr_sgr_set][i * 2]
                mn = T.Sgrproj_Xqd_Min[i]
                mx = T.Sgrproj_Xqd_Max[i]
                if radius:
                    v = self.decode_signed_subexp_with_ref_bool(mn, mx + 1, SGRPROJ_PRJ_SUBEXP_K, self.RefSgrXqd[plane][i])
                else:
                    v = 0
                    if i == 1:
                        v = clip3(mn, mx, (1 << SGRPROJ_PRJ_BITS) - self.RefSgrXqd[plane][0])
                xqd[i] = v
                self.RefSgrXqd[plane][i] = v
            self.LrSgrXqd[key] = xqd

    def decode_signed_subexp_with_ref_bool(self, low, high, k, r):
        x = self.decode_unsigned_subexp_with_ref_bool(high - low, k, r - low)
        return x + low

    def decode_unsigned_subexp_with_ref_bool(self, mx, k, r):
        v = self.decode_subexp_bool(mx, k)
        if (r << 1) <= mx:
            return self.inverse_recenter(r, v)
        return mx - 1 - self.inverse_recenter(mx - 1 - r, v)

    def decode_subexp_bool(self, numSyms, k):
        i = 0
        mk = 0
        while True:
            b2 = k + i - 1 if i else k
            a = 1 << b2
            if numSyms <= mk + 3 * a:
                return self.dec.read_ns(numSyms - mk, "subexp_unif_bools") + mk
            if self.L(1, "subexp_more_bools"):
                i += 1
                mk += a
            else:
                return self.L(b2, "subexp_bools") + mk

    @staticmethod
    def inverse_recenter(r, v):
        if v > 2 * r:
            return v
        if v & 1:
            return r - ((v + 1) >> 1)
        return r + (v >> 1)
