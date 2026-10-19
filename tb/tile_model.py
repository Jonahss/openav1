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
MI_SIZE_LOG2 = 2
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
TX_SET_INTER_1, TX_SET_INTER_2, TX_SET_INTER_3 = 1, 2, 3
INTRA_FRAME, NONE = 0, -1
# inter / intra block copy (5.11.26 assign_mv, 5.11.32 read_mv, 7.10.2 find_mv_stack)
MV_JOINT_ZERO, MV_JOINT_HNZVZ, MV_JOINT_HZVNZ, MV_JOINT_HNZVNZ = 0, 1, 2, 3
MV_CLASS_0 = 0
CLASS0_SIZE = 2
MV_CONTEXTS = 2
MV_INTRABC_CONTEXT = 1
MV_BORDER = 128
INTRABC_DELAY_PIXELS = 256
REF_CAT_LEVEL = 640
MAX_REF_MV_STACK_SIZE = 8
NEARESTMV, NEARMV, GLOBALMV, NEWMV = 13, 14, 15, 16
NEAREST_NEARESTMV, NEAR_NEARMV, NEAREST_NEWMV, NEW_NEARESTMV, NEAR_NEWMV, NEW_NEARMV, GLOBAL_GLOBALMV, NEW_NEWMV = \
    17, 18, 19, 20, 21, 22, 23, 24
SIMPLE, OBMC, LOCALWARP = 0, 1, 2
COMPOUND_WEDGE, COMPOUND_DIFFWTD, COMPOUND_AVERAGE, COMPOUND_INTRA, COMPOUND_DISTANCE = 0, 1, 2, 3, 4
EIGHTTAP, EIGHTTAP_SMOOTH, EIGHTTAP_SHARP, BILINEAR, SWITCHABLE = 0, 1, 2, 3, 4
# inter frames (5.11.7 .. 5.11.33, 7.9, 7.10)
LAST_FRAME, LAST2_FRAME, LAST3_FRAME, GOLDEN_FRAME, BWDREF_FRAME, ALTREF2_FRAME, ALTREF_FRAME = 1, 2, 3, 4, 5, 6, 7
SINGLE_REFERENCE, COMPOUND_REFERENCE = 0, 1
UNIDIR_COMP_REFERENCE, BIDIR_COMP_REFERENCE = 0, 1
IDENTITY, TRANSLATION, ROTZOOM, AFFINE = 0, 1, 2, 3
WARPEDMODEL_PREC_BITS = 16
REF_SCALE_SHIFT = 14
COMP_NEWMV_CTXS = 5
LEAST_SQUARES_SAMPLES_MAX = 8
MFMV_STACK_SIZE = 3
MAX_FRAME_DISTANCE = 31
MAX_OFFSET_WIDTH, MAX_OFFSET_HEIGHT = 8, 0
REFMVS_LIMIT = (1 << 12) - 1
MV_INVALID = -(1 << 15)                                    # MotionFieldMvs hole


def round2signed(x, n):
    if n == 0:
        return x
    return (x + (1 << (n - 1))) >> n if x >= 0 else -((-x + (1 << (n - 1))) >> n)

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


def find_tx_size(w, h):
    for txSz in range(T.TX_SIZES_ALL):
        if T.Tx_Width[txSz] == w and T.Tx_Height[txSz] == h:
            return txSz
    return T.TX_SIZES_ALL


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
        # not in the trace: fixed by the spec for intra frames (uncompressed_header: FrameIsIntra forces
        # force_integer_mv = 1; allow_high_precision_mv / use_ref_frame_mvs are only read for inter frames)
        self.force_integer_mv = 1
        self.allow_high_precision_mv = 0
        self.use_ref_frame_mvs = 0
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
        # inter fields (79 values, traces since 2026-10-18); names as in tb/obu_parser.py
        self.ref_frame_idx = [0] * 7
        self.OrderHints = [0] * 8
        self.GmType = [0] * 8
        self.gm_params = [[0, 0, 1 << 16, 0, 0, 1 << 16] for _ in range(8)]
        self.reference_select = self.skip_mode_present = self.allow_warped_motion = 0
        self.SkipModeFrame = [0, 0]
        self.interpolation_filter = self.is_motion_mode_switchable = 0
        # the references' upscaled width / height (by RefFrame) and the sequence flags the tile syntax reads: set only
        # when the trace carries them (traces since 2026-10-19), so that tools/xcheck_hdr.py skips them on older traces
        if len(rest) >= 32 + 79 + 21:
            self.RefUpscaledWidths = [0] * 8
            self.RefFrameHeights = [0] * 8
            r = rest[32 + 79:32 + 79 + 21]
            if self.frame_type in (1, 3):
                for i in range(7):
                    self.RefUpscaledWidths[1 + i], self.RefFrameHeights[1 + i] = r[2 * i], r[2 * i + 1]
            (self.enable_interintra_compound, self.enable_masked_compound, self.enable_jnt_comp, self.enable_dual_filter,
             self.enable_order_hint, self.OrderHintBits, self.enable_warped_motion) = r[14:21]
        if len(rest) >= 32 + 79:
            r = rest[32:32 + 79]
            (self.order_hint, self.refresh_frame_flags, self.error_resilient_mode, self.force_integer_mv,
             self.show_frame, self.showable_frame) = r[0:6]
            (self.allow_high_precision_mv, self.interpolation_filter, self.is_motion_mode_switchable,
             self.use_ref_frame_mvs) = r[6:10]
            self.reference_select, skip_allowed, self.skip_mode_present, sm0, sm1, self.allow_warped_motion = r[10:16]
            self.SkipModeFrame = [1 + sm0, 1 + sm1] if skip_allowed else [0, 0]     # set whenever skip mode is allowed (5.9.22)
            is_inter = self.frame_type in (1, 3)
            if is_inter:
                self.ref_frame_idx = list(r[16:23])
                self.OrderHints = [0] + list(r[23:30])
            for i in range(7):                            # dav1d's gmv[i]: type, matrix[6] (spec order)
                g = r[30 + 7 * i: 37 + 7 * i]
                self.GmType[1 + i] = g[0]
                self.gm_params[1 + i] = list(g[1:7])
        self.NumPlanes = 1 if self.layout == 0 else 3
        self.FrameIsIntra = self.frame_type in (0, 2)
        # the tile decode (TileDecoder) still handles intra frames without CDF inheritance only; the header object
        # itself is built for every frame so that tools/xcheck_hdr.py can check inter headers
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
    # MvJointCdf[ctx], MvClassCdf[ctx][comp], MvClass0FrCdf[ctx][comp][bit], MvFrCdf[ctx][comp] (the comp
    # dimension is part of the default table); MvClass0BitCdf / MvClass0HpCdf / MvSignCdf / MvHpCdf /
    # MvBitCdf are [ctx][comp] copies of a single default row (MvBit: [ctx][comp][i])
    for k in ("Mv_Joint", "Mv_Class", "Mv_Class0_Fr", "Mv_Fr"):
        d[k] = [copy.deepcopy(d[k]) for _ in range(MV_CONTEXTS)]
    for k in ("Mv_Class0_Bit", "Mv_Class0_Hp", "Mv_Sign", "Mv_Hp", "Mv_Bit"):
        d[k] = [[copy.deepcopy(d[k]) for _ in range(2)] for _ in range(MV_CONTEXTS)]
    return d


# =================================================================================================
# the tile decoder
# =================================================================================================
class FrameCtx:
    """What a tile of an inter frame needs from the frames before it (tb/seq_model.py builds one per frame):
    the frame's CDFs (None: the defaults for base_q_idx), PrevSegmentIds (None: zeros) and the motion field
    MotionFieldMvs[ref][y8][x8] (None: use_ref_frame_mvs off)."""
    def __init__(self, cdfs=None, prev_segment_ids=None, motion_field=None):
        self.cdfs, self.prev_segment_ids, self.motion_field = cdfs, prev_segment_ids, motion_field


class TileDecoder:
    def __init__(self, hdr, data, hook=None, frame=None):
        if not hdr.FrameIsIntra or hdr.primary_ref_frame != 7:
            assert frame is not None, "an inter frame (or CDF inheritance) needs the sequence state (FrameCtx)"
        self.h = hdr
        self.frame = frame if frame is not None else FrameCtx()
        self.dec = SymbolDecoder(data, hdr.disable_cdf_update, hook)
        self.cdf = copy.deepcopy(self.frame.cdfs) if self.frame.cdfs is not None else make_cdfs(hdr.base_q_idx)
        h = hdr
        R, Cc = h.MiRows, h.MiCols
        # 7.20 / 5.9.2: the sign bias of each reference (the trace header carries the order hints only)
        self.RefFrameSignBias = [0] * 8
        if not h.FrameIsIntra and getattr(h, "enable_order_hint", 0):
            for ref in range(LAST_FRAME, ALTREF_FRAME + 1):
                self.RefFrameSignBias[ref] = int(self.get_relative_dist(h.OrderHints[ref], h.order_hint) > 0)
        # The per-4x4 frame arrays are allocated once per frame and shared by the frame's tiles (FrameCtx.arena):
        # every tile reads and writes only its own region (the spec's is_inside / availability checks stop at tile
        # edges, above contexts are per tile below), so sharing is exact, and a 2,000-tile Argon frame no longer
        # allocates 25 frame-sized arrays per tile (hours and 20 GB on 2026-10-19).
        ar = getattr(self.frame, "arena", None)
        if ar is None:
            ar = self.frame.arena = {}

        def arr(name, make):
            a = ar.get(name)
            if a is None:
                a = ar[name] = make()
            return a
        self.SkipModes = arr("SkipModes", lambda: [[0] * Cc for _ in range(R)])
        self.CompGroupIdxs = arr("CompGroupIdxs", lambda: [[0] * Cc for _ in range(R)])
        self.CompoundIdxs = arr("CompoundIdxs", lambda: [[0] * Cc for _ in range(R)])
        self.PrevSegmentIds = self.frame.prev_segment_ids
        self.MotionFieldMvs = self.frame.motion_field
        self.YModes = arr("YModes", lambda: [[DC_PRED] * Cc for _ in range(R)])
        self.UVModes = arr("UVModes", lambda: [[DC_PRED] * Cc for _ in range(R)])
        self.Skips = arr("Skips", lambda: [[0] * Cc for _ in range(R)])
        self.TxSizes = arr("TxSizes", lambda: [[TX_4X4] * Cc for _ in range(R)])
        self.InterTxSizes = arr("InterTxSizes", lambda: [[TX_4X4] * Cc for _ in range(R)])
        self.MiSizes = arr("MiSizes", lambda: [[BLOCK_4X4] * Cc for _ in range(R)])
        self.SegmentIds = arr("SegmentIds", lambda: [[0] * Cc for _ in range(R)])
        self.PaletteSizes = arr("PaletteSizes", lambda: [[[0] * Cc for _ in range(R)] for _ in range(2)])
        self.PaletteColors = arr("PaletteColors", lambda: [[[None] * Cc for _ in range(R)] for _ in range(2)])
        self.TxTypes = arr("TxTypes", lambda: [[DCT_DCT] * Cc for _ in range(R)])
        self.DeltaLFs = arr("DeltaLFs", lambda: [[None] * Cc for _ in range(R)])
        self.RefFrames = arr("RefFrames", lambda: [[None] * Cc for _ in range(R)])      # None = not written yet this frame (7.10.2.4)
        self.IsInters = arr("IsInters", lambda: [[0] * Cc for _ in range(R)])
        self.Mvs = arr("Mvs", lambda: [[None] * Cc for _ in range(R)])
        self.InterpFilters = arr("InterpFilters", lambda: [[None] * Cc for _ in range(R)])
        self.use_intrabc = 0
        self.is_inter = 0
        self.AboveLevelContext = [[0] * Cc for _ in range(3)]
        self.AboveDcContext = [[0] * Cc for _ in range(3)]
        self.LeftLevelContext = [[0] * R for _ in range(3)]
        self.LeftDcContext = [[0] * R for _ in range(3)]
        self.AboveSegPredContext = [0] * (Cc + 32)             # written for the whole block, past the frame edge
        self.LeftSegPredContext = [0] * (R + 32)
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
        def lf_sizes():
            out = []
            for plane in range(3):
                subX = hdr.subsampling_x if plane > 0 else 0
                subY = hdr.subsampling_y if plane > 0 else 0
                out.append([[TX_4X4] * ((Cc + subX) >> subX) for _ in range((R + subY) >> subY)])
            return out
        self.LoopfilterTxSizes = arr("LoopfilterTxSizes", lf_sizes)

    # ---- helpers ------------------------------------------------------------------------------
    def sym(self, cdf, name):
        return self.dec.read_symbol(cdf, name)

    def L(self, n, name):
        return self.dec.read_literal(n, name)

    def NS(self, n, name):
        return self.dec.read_ns(n, name)

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
        if h.FrameIsIntra:
            self.intra_frame_mode_info()
        else:
            self.inter_frame_mode_info()
        self.palette_tokens()
        self.read_block_tx_size()
        if self.skip:
            self.reset_block_context(bw4, bh4)
        isCompound = self.RefFrame[1] > INTRA_FRAME
        for y in range(bh4):
            for x in range(bw4):
                if r + y < h.MiRows and c + x < h.MiCols:
                    self.YModes[r + y][c + x] = self.YMode
                    if self.RefFrame[0] == INTRA_FRAME and self.HasChroma:
                        self.UVModes[r + y][c + x] = self.UVMode
                    self.RefFrames[r + y][c + x] = list(self.RefFrame)
                    if self.is_inter:
                        if not self.use_intrabc:
                            self.CompGroupIdxs[r + y][c + x] = self.comp_group_idx
                            self.CompoundIdxs[r + y][c + x] = self.compound_idx
                        self.InterpFilters[r + y][c + x] = list(self.interp_filter)
                        self.Mvs[r + y][c + x] = [list(self.Mv[i]) for i in range(1 + isCompound)]
        self.compute_prediction()
        self.residual()
        for y in range(bh4):
            for x in range(bw4):
                if r + y < h.MiRows and c + x < h.MiCols:
                    self.IsInters[r + y][c + x] = self.is_inter
                    self.SkipModes[r + y][c + x] = self.skip_mode
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
                                cfl=(self.CflAlphaU, self.CflAlphaV), intrabc=self.use_intrabc,
                                mv=(tuple(self.Mv[0]) if self.use_intrabc else None),
                                inter=self.is_inter, ref=tuple(self.RefFrame), mvs=[tuple(m) for m in self.Mv[:1 + isCompound]],
                                motion_mode=self.motion_mode, compound_type=self.compound_type,
                                interintra=self.interintra, interp=tuple(self.interp_filter), skip_mode=self.skip_mode,
                                # the optional inter fields, zero where the syntax did not read them (as the RTL record has them)
                                interintra_mode=self.interintra_mode if self.interintra else 0,
                                wedge_interintra=self.wedge_interintra if self.interintra else 0,
                                wedge_index=self.wedge_index if ((self.interintra and self.wedge_interintra) or (self.is_inter and self.compound_type == COMPOUND_WEDGE)) else 0,
                                wedge_sign=self.wedge_sign if (self.is_inter and self.compound_type == COMPOUND_WEDGE and not self.interintra) else 0,
                                mask_type=self.mask_type if (self.is_inter and self.compound_type == COMPOUND_DIFFWTD) else 0,
                                comp_group_idx=getattr(self, "comp_group_idx", 0), compound_idx=getattr(self, "compound_idx", 1)))

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
        self.PaletteSizeY = self.PaletteSizeUV = 0
        self.palette_colors_y = [0] * 8
        self.palette_colors_u = [0] * 8
        self.palette_colors_v = [0] * 8
        self.use_filter_intra = 0
        self.filter_intra_mode = 0
        self.AngleDeltaY = self.AngleDeltaUV = 0
        self.CflAlphaU = self.CflAlphaV = 0
        self.UVMode = DC_PRED
        self.Mv = [[0, 0], [0, 0]]
        self.motion_mode = SIMPLE
        self.compound_type = COMPOUND_AVERAGE
        self.interp_filter = [EIGHTTAP, EIGHTTAP]
        self.interintra = 0
        self.comp_group_idx, self.compound_idx = 0, 1
        if self.use_intrabc:
            self.is_inter = 1
            self.YMode = DC_PRED
            self.UVMode = DC_PRED
            self.interp_filter = [BILINEAR, BILINEAR]
            self.find_mv_stack(0)
            self.assign_mv(0)
            return
        self.is_inter = 0
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

    # ---- 5.11.7 inter_frame_mode_info --------------------------------------------------------------------
    def inter_frame_mode_info(self):
        h = self.h
        r, c = self.MiRow, self.MiCol
        self.use_intrabc = 0
        self.LeftRefFrame = [self.RefFrames[r][c - 1][0] if self.AvailL else INTRA_FRAME,
                             self.RefFrames[r][c - 1][1] if self.AvailL else NONE]
        self.AboveRefFrame = [self.RefFrames[r - 1][c][0] if self.AvailU else INTRA_FRAME,
                              self.RefFrames[r - 1][c][1] if self.AvailU else NONE]
        self.LeftIntra = self.LeftRefFrame[0] <= INTRA_FRAME
        self.AboveIntra = self.AboveRefFrame[0] <= INTRA_FRAME
        self.LeftSingle = self.LeftRefFrame[1] <= INTRA_FRAME
        self.AboveSingle = self.AboveRefFrame[1] <= INTRA_FRAME
        self.skip = 0
        self.inter_segment_id(1)
        self.read_skip_mode()
        if self.skip_mode:
            self.skip = 1
        else:
            self.read_skip()
        if not h.SegIdPreSkip:
            self.inter_segment_id(0)
        self.Lossless = h.LosslessArray[self.segment_id]
        self.read_cdef()
        self.read_delta_qindex()
        self.read_delta_lf()
        self.ReadDeltas = 0
        # the block fields every block stores (decode_block), at their defaults
        self.PaletteSizeY = self.PaletteSizeUV = 0
        self.palette_colors_y = [0] * 8
        self.palette_colors_u = [0] * 8
        self.palette_colors_v = [0] * 8
        self.use_filter_intra = 0
        self.filter_intra_mode = 0
        self.AngleDeltaY = self.AngleDeltaUV = 0
        self.CflAlphaU = self.CflAlphaV = 0
        self.UVMode = DC_PRED
        self.Mv = [[0, 0], [0, 0]]
        self.motion_mode = SIMPLE
        self.compound_type = COMPOUND_AVERAGE
        self.interp_filter = [EIGHTTAP, EIGHTTAP]
        self.interintra = 0
        self.comp_group_idx, self.compound_idx = 0, 1
        self.read_is_inter()
        if self.is_inter:
            self.inter_block_mode_info()
        else:
            self.intra_block_mode_info()

    def inter_segment_id(self, preSkip):
        h = self.h
        if not h.segmentation_enabled:
            self.segment_id = 0
            return
        predictedSegmentId = self.get_segment_id()
        if not h.segmentation_update_map:
            self.segment_id = predictedSegmentId
            return
        if preSkip and not h.SegIdPreSkip:
            self.segment_id = 0
            return
        bw4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        bh4 = T.Num_4x4_Blocks_High[self.MiSize]
        if not preSkip and self.skip:
            for i in range(bw4):
                self.AboveSegPredContext[self.MiCol + i] = 0
            for i in range(bh4):
                self.LeftSegPredContext[self.MiRow + i] = 0
            self.read_segment_id()
            return
        if h.segmentation_temporal_update == 1:
            ctx = self.LeftSegPredContext[self.MiRow] + self.AboveSegPredContext[self.MiCol]
            seg_id_predicted = self.sym(self.cdf["Segment_Id_Predicted"][ctx], "seg_id_predicted")
            if seg_id_predicted:
                self.segment_id = predictedSegmentId
            else:
                self.read_segment_id()
            for i in range(bw4):
                self.AboveSegPredContext[self.MiCol + i] = seg_id_predicted
            for i in range(bh4):
                self.LeftSegPredContext[self.MiRow + i] = seg_id_predicted
        else:
            self.read_segment_id()

    def get_segment_id(self):
        h = self.h
        bw4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        bh4 = T.Num_4x4_Blocks_High[self.MiSize]
        xMis = min(h.MiCols - self.MiCol, bw4)
        yMis = min(h.MiRows - self.MiRow, bh4)
        seg = 7
        prev = self.PrevSegmentIds
        for y in range(yMis):
            for x in range(xMis):
                seg = min(seg, prev[self.MiRow + y][self.MiCol + x] if prev is not None else 0)
        return seg

    def read_skip_mode(self):
        h = self.h
        if (self.seg_feature_active(SEG_LVL_SKIP) or self.seg_feature_active(SEG_LVL_REF_FRAME)
                or self.seg_feature_active(SEG_LVL_GLOBALMV) or not h.skip_mode_present
                or T.Block_Width[self.MiSize] < 8 or T.Block_Height[self.MiSize] < 8):
            self.skip_mode = 0
            return
        ctx = 0
        if self.AvailU:
            ctx += self.SkipModes[self.MiRow - 1][self.MiCol]
        if self.AvailL:
            ctx += self.SkipModes[self.MiRow][self.MiCol - 1]
        self.skip_mode = self.sym(self.cdf["Skip_Mode"][ctx], "skip_mode")

    def read_is_inter(self):
        h = self.h
        if self.skip_mode:
            self.is_inter = 1
        elif self.seg_feature_active(SEG_LVL_REF_FRAME):
            self.is_inter = int(h.FeatureData[self.segment_id][SEG_LVL_REF_FRAME] != INTRA_FRAME)
        elif self.seg_feature_active(SEG_LVL_GLOBALMV):
            self.is_inter = 1
        else:
            if self.AvailU and self.AvailL:
                ctx = 3 if (self.LeftIntra and self.AboveIntra) else int(self.LeftIntra or self.AboveIntra)
            elif self.AvailU or self.AvailL:
                ctx = 2 * int(self.AboveIntra if self.AvailU else self.LeftIntra)
            else:
                ctx = 0
            self.is_inter = self.sym(self.cdf["Is_Inter"][ctx], "is_inter")

    # ---- 5.11.22 intra_block_mode_info (intra blocks of inter frames)
    def intra_block_mode_info(self):
        h = self.h
        self.RefFrame = [INTRA_FRAME, NONE]
        self.YMode = self.sym(self.cdf["Y_Mode"][T.Size_Group[self.MiSize]], "y_mode")
        self.intra_angle_info_y()
        if self.HasChroma:
            self.UVMode = self.sym(self.uv_mode_cdf(), "uv_mode")
            if self.UVMode == UV_CFL_PRED:
                self.read_cfl_alphas()
            self.intra_angle_info_uv()
        self.PaletteSizeY = self.PaletteSizeUV = 0
        if (self.MiSize >= BLOCK_8X8 and T.Block_Width[self.MiSize] <= 64 and T.Block_Height[self.MiSize] <= 64
                and h.allow_screen_content_tools):
            self.palette_mode_info()
        self.filter_intra_mode_info()

    # ---- 5.11.23 inter_block_mode_info
    def inter_block_mode_info(self):
        h = self.h
        self.PaletteSizeY = self.PaletteSizeUV = 0
        self.read_ref_frames()
        isCompound = int(self.RefFrame[1] > INTRA_FRAME)
        self.find_mv_stack(isCompound)
        if self.skip_mode:
            self.YMode = NEAREST_NEARESTMV
        elif self.seg_feature_active(SEG_LVL_SKIP) or self.seg_feature_active(SEG_LVL_GLOBALMV):
            self.YMode = GLOBALMV
        elif isCompound:
            ctx = T.Compound_Mode_Ctx_Map[self.RefMvContext >> 1][min(self.NewMvContext, COMP_NEWMV_CTXS - 1)]
            compound_mode = self.sym(self.cdf["Compound_Mode"][ctx], "compound_mode")
            self.YMode = NEAREST_NEARESTMV + compound_mode
        else:
            new_mv = self.sym(self.cdf["New_Mv"][self.NewMvContext], "new_mv")
            if new_mv == 0:
                self.YMode = NEWMV
            else:
                zero_mv = self.sym(self.cdf["Zero_Mv"][self.ZeroMvContext], "zero_mv")
                if zero_mv == 0:
                    self.YMode = GLOBALMV
                else:
                    ref_mv = self.sym(self.cdf["Ref_Mv"][self.RefMvContext], "ref_mv")
                    self.YMode = NEARESTMV if ref_mv == 0 else NEARMV
        self.RefMvIdx = 0
        if self.YMode in (NEWMV, NEW_NEWMV):
            for idx in range(2):
                if self.NumMvFound > idx + 1:
                    drl_mode = self.sym(self.cdf["Drl_Mode"][self.DrlCtxStack[idx]], "drl_mode")
                    if drl_mode == 0:
                        self.RefMvIdx = idx
                        break
                    self.RefMvIdx = idx + 1
        elif self.has_nearmv():
            self.RefMvIdx = 1
            for idx in range(1, 3):
                if self.NumMvFound > idx + 1:
                    drl_mode = self.sym(self.cdf["Drl_Mode"][self.DrlCtxStack[idx]], "drl_mode")
                    if drl_mode == 0:
                        self.RefMvIdx = idx
                        break
                    self.RefMvIdx = idx + 1
        self.assign_mv(isCompound)
        self.read_interintra_mode(isCompound)
        self.read_motion_mode(isCompound)
        self.read_compound_type(isCompound)
        if h.interpolation_filter == SWITCHABLE:
            for d in range(2 if h.enable_dual_filter else 1):
                if self.needs_interp_filter():
                    self.interp_filter[d] = self.sym(self.cdf["Interp_Filter"][self.interp_filter_ctx(d)], "interp_filter")
                else:
                    self.interp_filter[d] = EIGHTTAP
            if not h.enable_dual_filter:
                self.interp_filter[1] = self.interp_filter[0]
        else:
            self.interp_filter = [h.interpolation_filter, h.interpolation_filter]

    def has_nearmv(self):
        return self.YMode in (NEARMV, NEAR_NEARMV, NEAR_NEWMV, NEW_NEARMV)

    def needs_interp_filter(self):
        h = self.h
        large = min(T.Block_Width[self.MiSize], T.Block_Height[self.MiSize]) >= 8
        if self.skip_mode or self.motion_mode == LOCALWARP:
            return 0
        if large and self.YMode == GLOBALMV:
            return int(h.GmType[self.RefFrame[0]] == TRANSLATION)
        if large and self.YMode == GLOBAL_GLOBALMV:
            return int(h.GmType[self.RefFrame[0]] == TRANSLATION or h.GmType[self.RefFrame[1]] == TRANSLATION)
        return 1

    def interp_filter_ctx(self, d):
        ctx = ((d & 1) * 2 + int(self.RefFrame[1] > INTRA_FRAME)) * 4
        leftType = aboveType = 3
        r, c = self.MiRow, self.MiCol
        if self.AvailL:
            rf = self.RefFrames[r][c - 1]
            if rf[0] == self.RefFrame[0] or rf[1] == self.RefFrame[0]:
                leftType = self.InterpFilters[r][c - 1][d]
        if self.AvailU:
            rf = self.RefFrames[r - 1][c]
            if rf[0] == self.RefFrame[0] or rf[1] == self.RefFrame[0]:
                aboveType = self.InterpFilters[r - 1][c][d]
        if leftType == aboveType:
            ctx += leftType
        elif leftType == 3:
            ctx += aboveType
        elif aboveType == 3:
            ctx += leftType
        else:
            ctx += 3
        return ctx

    # ---- 5.11.25 read_ref_frames and the reference contexts (8.3.2)
    def count_refs(self, frameType):
        c = 0
        if self.AvailU:
            c += int(self.AboveRefFrame[0] == frameType) + int(self.AboveRefFrame[1] == frameType)
        if self.AvailL:
            c += int(self.LeftRefFrame[0] == frameType) + int(self.LeftRefFrame[1] == frameType)
        return c

    @staticmethod
    def ref_count_ctx(c0, c1):
        return 0 if c0 < c1 else 1 if c0 == c1 else 2

    def ctx_comp_ref(self):
        return self.ref_count_ctx(self.count_refs(LAST_FRAME) + self.count_refs(LAST2_FRAME),
                                  self.count_refs(LAST3_FRAME) + self.count_refs(GOLDEN_FRAME))

    def ctx_comp_ref_p1(self):
        return self.ref_count_ctx(self.count_refs(LAST_FRAME), self.count_refs(LAST2_FRAME))

    def ctx_comp_ref_p2(self):
        return self.ref_count_ctx(self.count_refs(LAST3_FRAME), self.count_refs(GOLDEN_FRAME))

    def ctx_comp_bwdref(self):
        return self.ref_count_ctx(self.count_refs(BWDREF_FRAME) + self.count_refs(ALTREF2_FRAME),
                                  self.count_refs(ALTREF_FRAME))

    def ctx_comp_bwdref_p1(self):
        return self.ref_count_ctx(self.count_refs(BWDREF_FRAME), self.count_refs(ALTREF2_FRAME))

    def ctx_single_ref_p1(self):
        fwd = sum(self.count_refs(f) for f in (LAST_FRAME, LAST2_FRAME, LAST3_FRAME, GOLDEN_FRAME))
        bwd = sum(self.count_refs(f) for f in (BWDREF_FRAME, ALTREF2_FRAME, ALTREF_FRAME))
        return self.ref_count_ctx(fwd, bwd)

    def ctx_uni_comp_ref_p1(self):
        return self.ref_count_ctx(self.count_refs(LAST2_FRAME),
                                  self.count_refs(LAST3_FRAME) + self.count_refs(GOLDEN_FRAME))

    @staticmethod
    def check_backward(refFrame):
        return int(BWDREF_FRAME <= refFrame <= ALTREF_FRAME)

    def ctx_comp_mode(self):
        if self.AvailU and self.AvailL:
            if self.AboveSingle and self.LeftSingle:
                return self.check_backward(self.AboveRefFrame[0]) ^ self.check_backward(self.LeftRefFrame[0])
            if self.AboveSingle:
                return 2 + int(self.check_backward(self.AboveRefFrame[0]) or self.AboveIntra)
            if self.LeftSingle:
                return 2 + int(self.check_backward(self.LeftRefFrame[0]) or self.LeftIntra)
            return 4
        if self.AvailU:
            return self.check_backward(self.AboveRefFrame[0]) if self.AboveSingle else 3
        if self.AvailL:
            return self.check_backward(self.LeftRefFrame[0]) if self.LeftSingle else 3
        return 1

    @staticmethod
    def is_samedir_ref_pair(ref0, ref1):
        return int((ref0 >= BWDREF_FRAME) == (ref1 >= BWDREF_FRAME))

    def ctx_comp_ref_type(self):
        above0, above1 = self.AboveRefFrame
        left0, left1 = self.LeftRefFrame
        aboveCompInter = self.AvailU and not self.AboveIntra and not self.AboveSingle
        leftCompInter = self.AvailL and not self.LeftIntra and not self.LeftSingle
        aboveUniComp = aboveCompInter and self.is_samedir_ref_pair(above0, above1)
        leftUniComp = leftCompInter and self.is_samedir_ref_pair(left0, left1)
        if self.AvailU and not self.AboveIntra and self.AvailL and not self.LeftIntra:
            samedir = self.is_samedir_ref_pair(above0, left0)
            if not aboveCompInter and not leftCompInter:
                return 1 + 2 * samedir
            if not aboveCompInter:
                return 1 if not leftUniComp else 3 + samedir
            if not leftCompInter:
                return 1 if not aboveUniComp else 3 + samedir
            if not aboveUniComp and not leftUniComp:
                return 0
            if not aboveUniComp or not leftUniComp:
                return 2
            return 3 + int((above0 == BWDREF_FRAME) == (left0 == BWDREF_FRAME))
        if self.AvailU and self.AvailL:
            if aboveCompInter:
                return 1 + 2 * int(aboveUniComp)
            if leftCompInter:
                return 1 + 2 * int(leftUniComp)
            return 2
        if aboveCompInter:
            return 4 * int(aboveUniComp)
        if leftCompInter:
            return 4 * int(leftUniComp)
        return 2

    def read_ref_frames(self):
        h = self.h
        cdf = self.cdf
        if self.skip_mode:
            self.RefFrame = list(h.SkipModeFrame)
            return
        if self.seg_feature_active(SEG_LVL_REF_FRAME):
            self.RefFrame = [h.FeatureData[self.segment_id][SEG_LVL_REF_FRAME], NONE]
            return
        if self.seg_feature_active(SEG_LVL_SKIP) or self.seg_feature_active(SEG_LVL_GLOBALMV):
            self.RefFrame = [LAST_FRAME, NONE]
            return
        bw4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        bh4 = T.Num_4x4_Blocks_High[self.MiSize]
        if h.reference_select and min(bw4, bh4) >= 2:
            comp_mode = self.sym(cdf["Comp_Mode"][self.ctx_comp_mode()], "comp_mode")
        else:
            comp_mode = SINGLE_REFERENCE
        if comp_mode == COMPOUND_REFERENCE:
            comp_ref_type = self.sym(cdf["Comp_Ref_Type"][self.ctx_comp_ref_type()], "comp_ref_type")
            if comp_ref_type == UNIDIR_COMP_REFERENCE:
                uni_comp_ref = self.sym(cdf["Uni_Comp_Ref"][self.ctx_single_ref_p1()][0], "uni_comp_ref")
                if uni_comp_ref:
                    self.RefFrame = [BWDREF_FRAME, ALTREF_FRAME]
                else:
                    uni_comp_ref_p1 = self.sym(cdf["Uni_Comp_Ref"][self.ctx_uni_comp_ref_p1()][1], "uni_comp_ref_p1")
                    if uni_comp_ref_p1:
                        uni_comp_ref_p2 = self.sym(cdf["Uni_Comp_Ref"][self.ctx_comp_ref_p2()][2], "uni_comp_ref_p2")
                        self.RefFrame = [LAST_FRAME, GOLDEN_FRAME] if uni_comp_ref_p2 else [LAST_FRAME, LAST3_FRAME]
                    else:
                        self.RefFrame = [LAST_FRAME, LAST2_FRAME]
            else:
                comp_ref = self.sym(cdf["Comp_Ref"][self.ctx_comp_ref()][0], "comp_ref")
                if comp_ref == 0:
                    comp_ref_p1 = self.sym(cdf["Comp_Ref"][self.ctx_comp_ref_p1()][1], "comp_ref_p1")
                    ref0 = LAST2_FRAME if comp_ref_p1 else LAST_FRAME
                else:
                    comp_ref_p2 = self.sym(cdf["Comp_Ref"][self.ctx_comp_ref_p2()][2], "comp_ref_p2")
                    ref0 = GOLDEN_FRAME if comp_ref_p2 else LAST3_FRAME
                comp_bwdref = self.sym(cdf["Comp_Bwd_Ref"][self.ctx_comp_bwdref()][0], "comp_bwdref")
                if comp_bwdref == 0:
                    comp_bwdref_p1 = self.sym(cdf["Comp_Bwd_Ref"][self.ctx_comp_bwdref_p1()][1], "comp_bwdref_p1")
                    ref1 = ALTREF2_FRAME if comp_bwdref_p1 else BWDREF_FRAME
                else:
                    ref1 = ALTREF_FRAME
                self.RefFrame = [ref0, ref1]
        else:
            single_ref_p1 = self.sym(cdf["Single_Ref"][self.ctx_single_ref_p1()][0], "single_ref_p1")
            if single_ref_p1:
                single_ref_p2 = self.sym(cdf["Single_Ref"][self.ctx_comp_bwdref()][1], "single_ref_p2")
                if single_ref_p2 == 0:
                    single_ref_p6 = self.sym(cdf["Single_Ref"][self.ctx_comp_bwdref_p1()][5], "single_ref_p6")
                    ref0 = ALTREF2_FRAME if single_ref_p6 else BWDREF_FRAME
                else:
                    ref0 = ALTREF_FRAME
            else:
                single_ref_p3 = self.sym(cdf["Single_Ref"][self.ctx_comp_ref()][2], "single_ref_p3")
                if single_ref_p3:
                    single_ref_p5 = self.sym(cdf["Single_Ref"][self.ctx_comp_ref_p2()][4], "single_ref_p5")
                    ref0 = GOLDEN_FRAME if single_ref_p5 else LAST3_FRAME
                else:
                    single_ref_p4 = self.sym(cdf["Single_Ref"][self.ctx_comp_ref_p1()][3], "single_ref_p4")
                    ref0 = LAST2_FRAME if single_ref_p4 else LAST_FRAME
            self.RefFrame = [ref0, NONE]

    # ---- 5.11.28 .. 5.11.30 inter-intra, motion mode, compound type
    def read_interintra_mode(self, isCompound):
        h = self.h
        self.interintra = 0
        self.wedge_interintra = 0
        if (not self.skip_mode and h.enable_interintra_compound and not isCompound
                and BLOCK_8X8 <= self.MiSize <= E["BLOCK_32X32"]):
            ctx = T.Size_Group[self.MiSize] - 1
            self.interintra = self.sym(self.cdf["Inter_Intra"][ctx], "interintra")
            if self.interintra:
                self.interintra_mode = self.sym(self.cdf["Inter_Intra_Mode"][ctx], "interintra_mode")
                self.RefFrame[1] = INTRA_FRAME
                self.AngleDeltaY = self.AngleDeltaUV = 0
                self.use_filter_intra = 0
                self.wedge_interintra = self.sym(self.cdf["Wedge_Inter_Intra"][self.MiSize], "wedge_interintra")
                if self.wedge_interintra:
                    self.wedge_index = self.sym(self.cdf["Wedge_Index"][self.MiSize], "wedge_index")
                    self.wedge_sign = 0

    def is_scaled(self, refFrame):
        h = self.h
        xScale = ((h.RefUpscaledWidths[refFrame] << REF_SCALE_SHIFT) + (h.FrameWidth // 2)) // h.FrameWidth
        yScale = ((h.RefFrameHeights[refFrame] << REF_SCALE_SHIFT) + (h.FrameHeight // 2)) // h.FrameHeight
        noScale = 1 << REF_SCALE_SHIFT
        return xScale != noScale or yScale != noScale

    def read_motion_mode(self, isCompound):
        h = self.h
        self.motion_mode = SIMPLE
        if self.skip_mode or not h.is_motion_mode_switchable:
            return
        if min(T.Block_Width[self.MiSize], T.Block_Height[self.MiSize]) < 8:
            return
        if not h.force_integer_mv and self.YMode in (GLOBALMV, GLOBAL_GLOBALMV):
            if h.GmType[self.RefFrame[0]] > TRANSLATION:
                return
        if isCompound or self.RefFrame[1] == INTRA_FRAME or not self.has_overlappable_candidates():
            return
        self.find_warp_samples()
        if h.force_integer_mv or self.NumSamples == 0 or not h.allow_warped_motion or self.is_scaled(self.RefFrame[0]):
            use_obmc = self.sym(self.cdf["Use_Obmc"][self.MiSize], "use_obmc")
            self.motion_mode = OBMC if use_obmc else SIMPLE
        else:
            self.motion_mode = self.sym(self.cdf["Motion_Mode"][self.MiSize], "motion_mode")

    def read_compound_type(self, isCompound):
        h = self.h
        self.comp_group_idx = 0
        self.compound_idx = 1
        if self.skip_mode:
            self.compound_type = COMPOUND_AVERAGE
            return
        if isCompound:
            n = T.Wedge_Bits[self.MiSize]
            if h.enable_masked_compound:
                self.comp_group_idx = self.sym(self.cdf["Comp_Group_Idx"][self.ctx_comp_group_idx()], "comp_group_idx")
            if self.comp_group_idx == 0:
                if h.enable_jnt_comp:
                    self.compound_idx = self.sym(self.cdf["Compound_Idx"][self.ctx_compound_idx()], "compound_idx")
                    self.compound_type = COMPOUND_AVERAGE if self.compound_idx else COMPOUND_DISTANCE
                else:
                    self.compound_type = COMPOUND_AVERAGE
            else:
                if n == 0:
                    self.compound_type = COMPOUND_DIFFWTD
                else:
                    self.compound_type = self.sym(self.cdf["Compound_Type"][self.MiSize], "compound_type")
            if self.compound_type == COMPOUND_WEDGE:
                self.wedge_index = self.sym(self.cdf["Wedge_Index"][self.MiSize], "wedge_index")
                self.wedge_sign = self.L(1, "wedge_sign")
            elif self.compound_type == COMPOUND_DIFFWTD:
                self.mask_type = self.L(1, "mask_type")
        else:
            if self.interintra:
                self.compound_type = COMPOUND_WEDGE if self.wedge_interintra else COMPOUND_INTRA
            else:
                self.compound_type = COMPOUND_AVERAGE

    def ctx_comp_group_idx(self):
        ctx = 0
        r, c = self.MiRow, self.MiCol
        if self.AvailU:
            if not self.AboveSingle:
                ctx += self.CompGroupIdxs[r - 1][c]
            elif self.AboveRefFrame[0] == ALTREF_FRAME:
                ctx += 3
        if self.AvailL:
            if not self.LeftSingle:
                ctx += self.CompGroupIdxs[r][c - 1]
            elif self.LeftRefFrame[0] == ALTREF_FRAME:
                ctx += 3
        return min(5, ctx)

    def ctx_compound_idx(self):
        h = self.h
        fwd = abs(self.get_relative_dist(h.OrderHints[self.RefFrame[0]], h.order_hint))
        bck = abs(self.get_relative_dist(h.OrderHints[self.RefFrame[1]], h.order_hint))
        ctx = 3 if fwd == bck else 0
        r, c = self.MiRow, self.MiCol
        if self.AvailU:
            if not self.AboveSingle:
                ctx += self.CompoundIdxs[r - 1][c]
            elif self.AboveRefFrame[0] == ALTREF_FRAME:
                ctx += 1
        if self.AvailL:
            if not self.LeftSingle:
                ctx += self.CompoundIdxs[r][c - 1]
            elif self.LeftRefFrame[0] == ALTREF_FRAME:
                ctx += 1
        return ctx

    def get_relative_dist(self, a, b):
        h = self.h
        if not getattr(h, "enable_order_hint", 0):
            return 0
        diff = a - b
        m = 1 << (h.OrderHintBits - 1)
        return (diff & (m - 1)) - (diff & m)

    # ---- 7.10.3 / 7.10.4 overlappable candidates, warp samples
    def has_overlappable_candidates(self):
        h = self.h
        if self.AvailU:
            w4 = T.Num_4x4_Blocks_Wide[self.MiSize]
            for x4 in range(self.MiCol, min(h.MiCols, self.MiCol + w4), 2):
                if self.RefFrames[self.MiRow - 1][x4 | 1][0] > INTRA_FRAME:
                    return 1
        if self.AvailL:
            h4 = T.Num_4x4_Blocks_High[self.MiSize]
            for y4 in range(self.MiRow, min(h.MiRows, self.MiRow + h4), 2):
                if self.RefFrames[y4 | 1][self.MiCol - 1][0] > INTRA_FRAME:
                    return 1
        return 0

    def find_warp_samples(self):
        h = self.h
        self.NumSamples = 0
        self.NumSamplesScanned = 0
        self.CandList = []
        w4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        h4 = T.Num_4x4_Blocks_High[self.MiSize]
        doTopLeft = doTopRight = 1
        if self.AvailU:
            srcSize = self.MiSizes[self.MiRow - 1][self.MiCol]
            srcW = T.Num_4x4_Blocks_Wide[srcSize]
            if w4 <= srcW:
                colOffset = -(self.MiCol & (srcW - 1))
                if colOffset < 0:
                    doTopLeft = 0
                if colOffset + srcW > w4:
                    doTopRight = 0
                self.add_sample(-1, 0)
            else:
                i = 0
                while i < min(w4, h.MiCols - self.MiCol):
                    srcSize = self.MiSizes[self.MiRow - 1][self.MiCol + i]
                    srcW = T.Num_4x4_Blocks_Wide[srcSize]
                    miStep = min(w4, srcW)
                    self.add_sample(-1, i)
                    i += miStep
        if self.AvailL:
            srcSize = self.MiSizes[self.MiRow][self.MiCol - 1]
            srcH = T.Num_4x4_Blocks_High[srcSize]
            if h4 <= srcH:
                rowOffset = -(self.MiRow & (srcH - 1))
                if rowOffset < 0:
                    doTopLeft = 0
                self.add_sample(0, -1)
            else:
                i = 0
                while i < min(h4, h.MiRows - self.MiRow):
                    srcSize = self.MiSizes[self.MiRow + i][self.MiCol - 1]
                    srcH = T.Num_4x4_Blocks_High[srcSize]
                    miStep = min(h4, srcH)
                    self.add_sample(i, -1)
                    i += miStep
        if doTopLeft:
            self.add_sample(-1, -1)
        if doTopRight and max(w4, h4) <= 16:
            self.add_sample(-1, w4)
        if self.NumSamples == 0 and self.NumSamplesScanned > 0:
            self.NumSamples = 1

    def add_sample(self, deltaRow, deltaCol):
        if self.NumSamplesScanned >= LEAST_SQUARES_SAMPLES_MAX:
            return
        mvRow = self.MiRow + deltaRow
        mvCol = self.MiCol + deltaCol
        if not self.is_inside(mvRow, mvCol):
            return
        if self.RefFrames[mvRow][mvCol] is None:
            return
        if self.RefFrames[mvRow][mvCol][0] != self.RefFrame[0]:
            return
        if self.RefFrames[mvRow][mvCol][1] != NONE:
            return
        candSz = self.MiSizes[mvRow][mvCol]
        candW4 = T.Num_4x4_Blocks_Wide[candSz]
        candH4 = T.Num_4x4_Blocks_High[candSz]
        candRow = mvRow & ~(candH4 - 1)
        candCol = mvCol & ~(candW4 - 1)
        midY = candRow * 4 + candH4 * 2 - 1
        midX = candCol * 4 + candW4 * 2 - 1
        threshold = clip3(16, 112, max(T.Block_Width[self.MiSize], T.Block_Height[self.MiSize]))
        cmv = self.Mvs[candRow][candCol][0]
        mvDiffRow = abs(cmv[0] - self.Mv[0][0])
        mvDiffCol = abs(cmv[1] - self.Mv[0][1])
        valid = (mvDiffRow + mvDiffCol) <= threshold
        cand = [midY * 8, midX * 8, midY * 8 + cmv[0], midX * 8 + cmv[1]]
        self.NumSamplesScanned += 1
        if not valid and self.NumSamplesScanned > 1:
            return
        if len(self.CandList) <= self.NumSamples:
            self.CandList.append(cand)
        else:
            self.CandList[self.NumSamples] = cand
        if valid:
            self.NumSamples += 1

    # ---- 7.10.2 motion vector prediction ------------------------------------------------------------------
    def find_mv_stack(self, isCompound):
        h = self.h
        bw4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        bh4 = T.Num_4x4_Blocks_High[self.MiSize]
        self.NumMvFound = 0
        self.NewMvCount = 0
        self.RefStackMv = [[[0, 0], [0, 0]] for _ in range(MAX_REF_MV_STACK_SIZE)]
        self.WeightStack = [0] * MAX_REF_MV_STACK_SIZE
        self.GlobalMvs = [self.setup_global_mv(0), self.setup_global_mv(1) if isCompound else [0, 0]]
        self.FoundMatch = 0
        self.scan_row(-1, isCompound)
        foundAboveMatch = self.FoundMatch
        self.FoundMatch = 0
        self.scan_col(-1, isCompound)
        foundLeftMatch = self.FoundMatch
        self.FoundMatch = 0
        if max(bw4, bh4) <= 16:
            self.scan_point(-1, bw4, isCompound)
        if self.FoundMatch:
            foundAboveMatch = 1
        self.CloseMatches = foundAboveMatch + foundLeftMatch
        numNearest = self.NumMvFound
        numNew = self.NewMvCount
        for idx in range(numNearest):
            self.WeightStack[idx] += REF_CAT_LEVEL
        self.ZeroMvContext = 0
        if h.use_ref_frame_mvs == 1:
            self.temporal_scan(isCompound)
        self.scan_point(-1, -1, isCompound)
        if self.FoundMatch:
            foundAboveMatch = 1
        self.FoundMatch = 0
        self.scan_row(-3, isCompound)
        if self.FoundMatch:
            foundAboveMatch = 1
        self.FoundMatch = 0
        self.scan_col(-3, isCompound)
        if self.FoundMatch:
            foundLeftMatch = 1
        self.FoundMatch = 0
        if bh4 > 1:
            self.scan_row(-5, isCompound)
        if self.FoundMatch:
            foundAboveMatch = 1
        self.FoundMatch = 0
        if bw4 > 1:
            self.scan_col(-5, isCompound)
        if self.FoundMatch:
            foundLeftMatch = 1
        self.TotalMatches = foundAboveMatch + foundLeftMatch
        self.sort_stack(0, numNearest)
        self.sort_stack(numNearest, self.NumMvFound)
        if self.NumMvFound < 2:
            self.extra_search(isCompound)
        self.context_and_clamping(isCompound, numNew)

    def setup_global_mv(self, refList):
        h = self.h
        ref = self.RefFrame[refList]
        typ = h.GmType[ref] if ref != INTRA_FRAME else IDENTITY
        if ref == INTRA_FRAME or typ == IDENTITY:
            mv = [0, 0]
        elif typ == TRANSLATION:
            gm = h.gm_params[ref]
            mv = [gm[0] >> (WARPEDMODEL_PREC_BITS - 3), gm[1] >> (WARPEDMODEL_PREC_BITS - 3)]
        else:
            gm = h.gm_params[ref]
            bw = T.Block_Width[self.MiSize]
            bh = T.Block_Height[self.MiSize]
            x = self.MiCol * MI_SIZE + bw // 2 - 1
            y = self.MiRow * MI_SIZE + bh // 2 - 1
            xc = (gm[2] - (1 << WARPEDMODEL_PREC_BITS)) * x + gm[3] * y + gm[0]
            yc = gm[4] * x + (gm[5] - (1 << WARPEDMODEL_PREC_BITS)) * y + gm[1]
            if h.allow_high_precision_mv:
                mv = [round2signed(yc, WARPEDMODEL_PREC_BITS - 3), round2signed(xc, WARPEDMODEL_PREC_BITS - 3)]
            else:
                mv = [round2signed(yc, WARPEDMODEL_PREC_BITS - 2) * 2, round2signed(xc, WARPEDMODEL_PREC_BITS - 2) * 2]
        self.lower_mv_precision(mv)
        return mv

    # 7.10.2.5 / 7.10.2.6 temporal candidates from the motion field
    def temporal_scan(self, isCompound):
        bw4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        bh4 = T.Num_4x4_Blocks_High[self.MiSize]
        stepW4 = 4 if bw4 >= 16 else 2
        stepH4 = 4 if bh4 >= 16 else 2
        for deltaRow in range(0, min(bh4, 16), stepH4):
            for deltaCol in range(0, min(bw4, 16), stepW4):
                self.add_tpl_ref_mv(deltaRow, deltaCol, isCompound)
        allowExtension = (T.Num_4x4_Blocks_High[BLOCK_8X8] <= bh4 < T.Num_4x4_Blocks_High[BLOCK_64X64]
                          and T.Num_4x4_Blocks_Wide[BLOCK_8X8] <= bw4 < T.Num_4x4_Blocks_Wide[BLOCK_64X64])
        if allowExtension:
            for deltaRow, deltaCol in ((bh4, -2), (bh4, bw4), (bh4 - 2, bw4)):
                row = (self.MiRow & 15) + deltaRow
                col = (self.MiCol & 15) + deltaCol
                if 0 <= row < 16 and 0 <= col < 16:
                    self.add_tpl_ref_mv(deltaRow, deltaCol, isCompound)

    def add_tpl_ref_mv(self, deltaRow, deltaCol, isCompound):
        mvRow = (self.MiRow + deltaRow) | 1
        mvCol = (self.MiCol + deltaCol) | 1
        if not self.is_inside(mvRow, mvCol):
            return
        x8, y8 = mvCol >> 1, mvRow >> 1
        if deltaRow == 0 and deltaCol == 0:
            self.ZeroMvContext = 1
        mf = self.MotionFieldMvs
        if not isCompound:
            candMv = list(mf[self.RefFrame[0]][y8][x8])
            if candMv[0] == MV_INVALID:
                return
            self.lower_mv_precision(candMv)
            if deltaRow == 0 and deltaCol == 0:
                g = self.GlobalMvs[0]
                self.ZeroMvContext = 1 if (abs(candMv[0] - g[0]) >= 16 or abs(candMv[1] - g[1]) >= 16) else 0
            for idx in range(self.NumMvFound):
                if candMv == self.RefStackMv[idx][0]:
                    self.WeightStack[idx] += 2
                    return
            if self.NumMvFound < MAX_REF_MV_STACK_SIZE:
                self.RefStackMv[self.NumMvFound][0] = candMv
                self.WeightStack[self.NumMvFound] = 2
                self.NumMvFound += 1
        else:
            candMv0 = list(mf[self.RefFrame[0]][y8][x8])
            if candMv0[0] == MV_INVALID:
                return
            candMv1 = list(mf[self.RefFrame[1]][y8][x8])
            if candMv1[0] == MV_INVALID:
                return
            self.lower_mv_precision(candMv0)
            self.lower_mv_precision(candMv1)
            if deltaRow == 0 and deltaCol == 0:
                g0, g1 = self.GlobalMvs
                self.ZeroMvContext = 1 if (abs(candMv0[0] - g0[0]) >= 16 or abs(candMv0[1] - g0[1]) >= 16
                                           or abs(candMv1[0] - g1[0]) >= 16 or abs(candMv1[1] - g1[1]) >= 16) else 0
            for idx in range(self.NumMvFound):
                if candMv0 == self.RefStackMv[idx][0] and candMv1 == self.RefStackMv[idx][1]:
                    self.WeightStack[idx] += 2
                    return
            if self.NumMvFound < MAX_REF_MV_STACK_SIZE:
                self.RefStackMv[self.NumMvFound][0] = candMv0
                self.RefStackMv[self.NumMvFound][1] = candMv1
                self.WeightStack[self.NumMvFound] = 2
                self.NumMvFound += 1

    def scan_row(self, deltaRow, isCompound):
        h = self.h
        bw4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        end4 = min(min(bw4, h.MiCols - self.MiCol), 16)
        deltaCol = 0
        useStep16 = bw4 >= 16
        if abs(deltaRow) > 1:
            deltaRow += self.MiRow & 1
            deltaCol = 1 - (self.MiCol & 1)
        i = 0
        while i < end4:
            mvRow = self.MiRow + deltaRow
            mvCol = self.MiCol + deltaCol + i
            if not self.is_inside(mvRow, mvCol):
                break
            length = min(bw4, T.Num_4x4_Blocks_Wide[self.MiSizes[mvRow][mvCol]])
            if abs(deltaRow) > 1:
                length = max(2, length)
            if useStep16:
                length = max(4, length)
            self.add_ref_mv_candidate(mvRow, mvCol, isCompound, length * 2)
            i += length

    def scan_col(self, deltaCol, isCompound):
        h = self.h
        bh4 = T.Num_4x4_Blocks_High[self.MiSize]
        end4 = min(min(bh4, h.MiRows - self.MiRow), 16)
        deltaRow = 0
        useStep16 = bh4 >= 16
        if abs(deltaCol) > 1:
            deltaRow = 1 - (self.MiRow & 1)
            deltaCol += self.MiCol & 1
        i = 0
        while i < end4:
            mvRow = self.MiRow + deltaRow + i
            mvCol = self.MiCol + deltaCol
            if not self.is_inside(mvRow, mvCol):
                break
            length = min(bh4, T.Num_4x4_Blocks_High[self.MiSizes[mvRow][mvCol]])
            if abs(deltaCol) > 1:
                length = max(2, length)
            if useStep16:
                length = max(4, length)
            self.add_ref_mv_candidate(mvRow, mvCol, isCompound, length * 2)
            i += length

    def scan_point(self, deltaRow, deltaCol, isCompound):
        mvRow = self.MiRow + deltaRow
        mvCol = self.MiCol + deltaCol
        if self.is_inside(mvRow, mvCol) and self.RefFrames[mvRow][mvCol] is not None:
            self.add_ref_mv_candidate(mvRow, mvCol, isCompound, 4)

    def add_ref_mv_candidate(self, mvRow, mvCol, isCompound, weight):
        if not self.IsInters[mvRow][mvCol]:
            return
        rf = self.RefFrames[mvRow][mvCol]
        if not isCompound:
            for candList in range(2):
                if rf[candList] == self.RefFrame[0]:
                    self.search_stack(mvRow, mvCol, candList, weight)
        else:
            if rf[0] == self.RefFrame[0] and rf[1] == self.RefFrame[1]:
                self.compound_search_stack(mvRow, mvCol, weight)

    @staticmethod
    def has_newmv(mode):
        return mode in (NEWMV, NEW_NEWMV, NEAR_NEWMV, NEW_NEARMV, NEAREST_NEWMV, NEW_NEARESTMV)

    def search_stack(self, mvRow, mvCol, candList, weight):
        h = self.h
        candMode = self.YModes[mvRow][mvCol]
        candSize = self.MiSizes[mvRow][mvCol]
        large = min(T.Block_Width[candSize], T.Block_Height[candSize]) >= 8
        if candMode in (GLOBALMV, GLOBAL_GLOBALMV) and h.GmType[self.RefFrame[0]] > TRANSLATION and large:
            candMv = list(self.GlobalMvs[0])
        else:
            candMv = list(self.Mvs[mvRow][mvCol][candList])
        self.lower_mv_precision(candMv)
        if self.has_newmv(candMode):
            self.NewMvCount += 1
        self.FoundMatch = 1
        for idx in range(self.NumMvFound):
            if candMv == self.RefStackMv[idx][0]:
                self.WeightStack[idx] += weight
                return
        if self.NumMvFound < MAX_REF_MV_STACK_SIZE:
            self.RefStackMv[self.NumMvFound][0] = candMv
            self.WeightStack[self.NumMvFound] = weight
            self.NumMvFound += 1

    def compound_search_stack(self, mvRow, mvCol, weight):
        h = self.h
        candMvs = [list(m) for m in self.Mvs[mvRow][mvCol]]
        candMode = self.YModes[mvRow][mvCol]
        if candMode == GLOBAL_GLOBALMV:
            for refList in range(2):
                if h.GmType[self.RefFrame[refList]] > TRANSLATION:
                    candMvs[refList] = list(self.GlobalMvs[refList])
        for i in range(2):
            self.lower_mv_precision(candMvs[i])
        self.FoundMatch = 1
        found = False
        for idx in range(self.NumMvFound):
            if candMvs[0] == self.RefStackMv[idx][0] and candMvs[1] == self.RefStackMv[idx][1]:
                self.WeightStack[idx] += weight
                found = True
                break
        if not found and self.NumMvFound < MAX_REF_MV_STACK_SIZE:
            self.RefStackMv[self.NumMvFound][0] = candMvs[0]
            self.RefStackMv[self.NumMvFound][1] = candMvs[1]
            self.WeightStack[self.NumMvFound] = weight
            self.NumMvFound += 1
        if self.has_newmv(candMode):
            self.NewMvCount += 1

    def lower_mv_precision(self, candMv):
        if self.h.allow_high_precision_mv:
            return
        for i in range(2):
            if self.h.force_integer_mv:
                a = abs(candMv[i])
                aInt = (a + 3) >> 3
                candMv[i] = (aInt << 3) if candMv[i] > 0 else -(aInt << 3)
            elif candMv[i] & 1:
                candMv[i] += -1 if candMv[i] > 0 else 1

    def sort_stack(self, start, end):
        while end > start:
            newEnd = start
            for idx in range(start + 1, end):
                if self.WeightStack[idx - 1] < self.WeightStack[idx]:
                    self.WeightStack[idx - 1], self.WeightStack[idx] = self.WeightStack[idx], self.WeightStack[idx - 1]
                    self.RefStackMv[idx - 1], self.RefStackMv[idx] = self.RefStackMv[idx], self.RefStackMv[idx - 1]
                    newEnd = idx
            end = newEnd

    def extra_search(self, isCompound):
        h = self.h
        self.RefIdCount = [0, 0]
        self.RefDiffCount = [0, 0]
        self.RefIdMvs = [[None, None], [None, None]]
        self.RefDiffMvs = [[None, None], [None, None]]
        w4 = min(16, T.Num_4x4_Blocks_Wide[self.MiSize])
        h4 = min(16, T.Num_4x4_Blocks_High[self.MiSize])
        w4 = min(w4, h.MiCols - self.MiCol)
        h4 = min(h4, h.MiRows - self.MiRow)
        num4x4 = min(w4, h4)
        for pass_ in range(2):
            idx = 0
            while idx < num4x4 and self.NumMvFound < 2:
                if pass_ == 0:
                    mvRow, mvCol = self.MiRow - 1, self.MiCol + idx
                else:
                    mvRow, mvCol = self.MiRow + idx, self.MiCol - 1
                if not self.is_inside(mvRow, mvCol):
                    break
                self.add_extra_mv_candidate(mvRow, mvCol, isCompound)
                if pass_ == 0:
                    idx += T.Num_4x4_Blocks_Wide[self.MiSizes[mvRow][mvCol]]
                else:
                    idx += T.Num_4x4_Blocks_High[self.MiSizes[mvRow][mvCol]]
        if isCompound:
            combinedMvs = [[None, None], [None, None]]
            for lst in range(2):
                compCount = 0
                for idx in range(self.RefIdCount[lst]):
                    combinedMvs[compCount][lst] = list(self.RefIdMvs[lst][idx])
                    compCount += 1
                idx = 0
                while idx < self.RefDiffCount[lst] and compCount < 2:
                    combinedMvs[compCount][lst] = list(self.RefDiffMvs[lst][idx])
                    compCount += 1
                    idx += 1
                while compCount < 2:
                    combinedMvs[compCount][lst] = list(self.GlobalMvs[lst])
                    compCount += 1
            if self.NumMvFound == 1:
                if combinedMvs[0][0] == self.RefStackMv[0][0] and combinedMvs[0][1] == self.RefStackMv[0][1]:
                    self.RefStackMv[self.NumMvFound][0] = combinedMvs[1][0]
                    self.RefStackMv[self.NumMvFound][1] = combinedMvs[1][1]
                else:
                    self.RefStackMv[self.NumMvFound][0] = combinedMvs[0][0]
                    self.RefStackMv[self.NumMvFound][1] = combinedMvs[0][1]
                self.WeightStack[self.NumMvFound] = 2
                self.NumMvFound += 1
            else:
                for idx in range(2):
                    self.RefStackMv[self.NumMvFound][0] = combinedMvs[idx][0]
                    self.RefStackMv[self.NumMvFound][1] = combinedMvs[idx][1]
                    self.WeightStack[self.NumMvFound] = 2
                    self.NumMvFound += 1
        else:
            for idx in range(self.NumMvFound, 2):
                self.RefStackMv[idx][0] = list(self.GlobalMvs[0])

    def add_extra_mv_candidate(self, mvRow, mvCol, isCompound):
        rf = self.RefFrames[mvRow][mvCol]
        if isCompound:
            for candList in range(2):
                candRef = rf[candList]
                if candRef > INTRA_FRAME:
                    for lst in range(2):
                        candMv = list(self.Mvs[mvRow][mvCol][candList])
                        if candRef == self.RefFrame[lst] and self.RefIdCount[lst] < 2:
                            self.RefIdMvs[lst][self.RefIdCount[lst]] = candMv
                            self.RefIdCount[lst] += 1
                        elif self.RefDiffCount[lst] < 2:
                            if self.RefFrameSignBias[candRef] != self.RefFrameSignBias[self.RefFrame[lst]]:
                                candMv = [-candMv[0], -candMv[1]]
                            self.RefDiffMvs[lst][self.RefDiffCount[lst]] = candMv
                            self.RefDiffCount[lst] += 1
        else:
            for candList in range(2):
                candRef = rf[candList]
                if candRef > INTRA_FRAME:
                    candMv = list(self.Mvs[mvRow][mvCol][candList])
                    if self.RefFrameSignBias[candRef] != self.RefFrameSignBias[self.RefFrame[0]]:
                        candMv = [-candMv[0], -candMv[1]]
                    idx = 0
                    while idx < self.NumMvFound:
                        if candMv == self.RefStackMv[idx][0]:
                            break
                        idx += 1
                    if idx == self.NumMvFound:
                        self.RefStackMv[idx][0] = candMv
                        self.WeightStack[idx] = 2
                        self.NumMvFound += 1

    def clamp_mv_row(self, mvec, border):
        bh4 = T.Num_4x4_Blocks_High[self.MiSize]
        mbToTopEdge = -((self.MiRow * MI_SIZE) * 8)
        mbToBottomEdge = ((self.h.MiRows - bh4 - self.MiRow) * MI_SIZE) * 8
        return clip3(mbToTopEdge - border, mbToBottomEdge + border, mvec)

    def clamp_mv_col(self, mvec, border):
        bw4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        mbToLeftEdge = -((self.MiCol * MI_SIZE) * 8)
        mbToRightEdge = ((self.h.MiCols - bw4 - self.MiCol) * MI_SIZE) * 8
        return clip3(mbToLeftEdge - border, mbToRightEdge + border, mvec)

    def context_and_clamping(self, isCompound, numNew):
        bw = T.Block_Width[self.MiSize]
        bh = T.Block_Height[self.MiSize]
        self.DrlCtxStack = []
        for idx in range(self.NumMvFound):
            z = 0
            if idx + 1 < self.NumMvFound:
                w0 = self.WeightStack[idx]
                w1 = self.WeightStack[idx + 1]
                if w0 >= REF_CAT_LEVEL:
                    if w1 < REF_CAT_LEVEL:
                        z = 1
                else:
                    z = 2
            self.DrlCtxStack.append(z)
        for lst in range(2 if isCompound else 1):
            for idx in range(self.NumMvFound):
                refMv = self.RefStackMv[idx][lst]
                refMv[0] = self.clamp_mv_row(refMv[0], MV_BORDER + bh * 8)
                refMv[1] = self.clamp_mv_col(refMv[1], MV_BORDER + bw * 8)
        if self.CloseMatches == 0:
            self.NewMvContext = min(self.TotalMatches, 1)
            self.RefMvContext = self.TotalMatches
        elif self.CloseMatches == 1:
            self.NewMvContext = 3 - min(numNew, 1)
            self.RefMvContext = 2 + self.TotalMatches
        else:
            self.NewMvContext = 5 - min(numNew, 1)
            self.RefMvContext = 5

    # ---- 5.11.26 assign_mv / 5.11.32 read_mv ----------------------------------------------------------------
    def get_mode(self, refList):
        y = self.YMode
        if refList == 0:
            if y < NEAREST_NEARESTMV:
                return y
            if y in (NEW_NEWMV, NEW_NEARESTMV, NEW_NEARMV):
                return NEWMV
            if y in (NEAREST_NEARESTMV, NEAREST_NEWMV):
                return NEARESTMV
            if y in (NEAR_NEARMV, NEAR_NEWMV):
                return NEARMV
            return GLOBALMV
        if y in (NEW_NEWMV, NEAREST_NEWMV, NEAR_NEWMV):
            return NEWMV
        if y in (NEAREST_NEARESTMV, NEW_NEARESTMV):
            return NEARESTMV
        if y in (NEAR_NEARMV, NEW_NEARMV):
            return NEARMV
        return GLOBALMV

    def assign_mv(self, isCompound):
        h = self.h
        self.PredMv = [[0, 0], [0, 0]]
        for i in range(1 + isCompound):
            compMode = NEWMV if self.use_intrabc else self.get_mode(i)
            if self.use_intrabc:
                self.PredMv[0] = list(self.RefStackMv[0][0])
                if self.PredMv[0] == [0, 0]:
                    self.PredMv[0] = list(self.RefStackMv[1][0])
                if self.PredMv[0] == [0, 0]:
                    sbSize = BLOCK_128X128 if h.use_128x128_superblock else BLOCK_64X64
                    sbSize4 = T.Num_4x4_Blocks_High[sbSize]
                    if self.MiRow - sbSize4 < h.MiRowStart:
                        self.PredMv[0] = [0, -(sbSize4 * MI_SIZE + INTRABC_DELAY_PIXELS) * 8]
                    else:
                        self.PredMv[0] = [-(sbSize4 * MI_SIZE * 8), 0]
            elif compMode == GLOBALMV:
                self.PredMv[i] = list(self.GlobalMvs[i])
            else:
                pos = 0 if compMode == NEARESTMV else self.RefMvIdx
                if compMode == NEWMV and self.NumMvFound <= 1:
                    pos = 0
                self.PredMv[i] = list(self.RefStackMv[pos][i])
            if compMode == NEWMV:
                self.read_mv(i)
            else:
                self.Mv[i] = list(self.PredMv[i])

    def read_mv(self, ref):
        diffMv = [0, 0]
        MvCtx = MV_INTRABC_CONTEXT if self.use_intrabc else 0
        mv_joint = self.sym(self.cdf["Mv_Joint"][MvCtx], "mv_joint")
        if mv_joint in (MV_JOINT_HZVNZ, MV_JOINT_HNZVNZ):
            diffMv[0] = self.read_mv_component(MvCtx, 0)
        if mv_joint in (MV_JOINT_HNZVZ, MV_JOINT_HNZVNZ):
            diffMv[1] = self.read_mv_component(MvCtx, 1)
        self.Mv[ref] = [self.PredMv[ref][0] + diffMv[0], self.PredMv[ref][1] + diffMv[1]]

    def read_mv_component(self, MvCtx, comp):
        h = self.h
        cdf = self.cdf
        mv_sign = self.sym(cdf["Mv_Sign"][MvCtx][comp], "mv_sign")
        mv_class = self.sym(cdf["Mv_Class"][MvCtx][comp], "mv_class")
        if mv_class == MV_CLASS_0:
            mv_class0_bit = self.sym(cdf["Mv_Class0_Bit"][MvCtx][comp], "mv_class0_bit")
            if h.force_integer_mv:
                mv_class0_fr = 3
            else:
                mv_class0_fr = self.sym(cdf["Mv_Class0_Fr"][MvCtx][comp][mv_class0_bit], "mv_class0_fr")
            if h.allow_high_precision_mv:
                mv_class0_hp = self.sym(cdf["Mv_Class0_Hp"][MvCtx][comp], "mv_class0_hp")
            else:
                mv_class0_hp = 1
            mag = ((mv_class0_bit << 3) | (mv_class0_fr << 1) | mv_class0_hp) + 1
        else:
            d = 0
            for i in range(mv_class):
                mv_bit = self.sym(cdf["Mv_Bit"][MvCtx][comp][i], "mv_bit")
                d |= mv_bit << i
            mag = CLASS0_SIZE << (mv_class + 2)
            if h.force_integer_mv:
                mv_fr = 3
            else:
                mv_fr = self.sym(cdf["Mv_Fr"][MvCtx][comp], "mv_fr")
            if h.allow_high_precision_mv:
                mv_hp = self.sym(cdf["Mv_Hp"][MvCtx][comp], "mv_hp")
            else:
                mv_hp = 1
            mag += ((d << 3) | (mv_fr << 1) | mv_hp) + 1
        return -mag if mv_sign else mag

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
        cmap[0][0] = self.NS(n, "color_index_map_" + tag)
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
        if (h.TxMode == TX_MODE_SELECT and self.MiSize > BLOCK_4X4 and self.is_inter
                and not self.skip and not self.Lossless):
            maxTxSz = T.Max_Tx_Size_Rect[self.MiSize]
            txW4 = T.Tx_Width[maxTxSz] // MI_SIZE
            txH4 = T.Tx_Height[maxTxSz] // MI_SIZE
            for row in range(self.MiRow, self.MiRow + bh4, txH4):
                for col in range(self.MiCol, self.MiCol + bw4, txW4):
                    self.read_var_tx_size(row, col, maxTxSz, 0)
        else:
            self.read_tx_size(allowSelect=(not self.skip) or (not self.is_inter))
            for row in range(self.MiRow, self.MiRow + bh4):
                for col in range(self.MiCol, self.MiCol + bw4):
                    if row < h.MiRows and col < h.MiCols:
                        self.InterTxSizes[row][col] = self.TxSize

    def read_var_tx_size(self, row, col, txSz, depth):
        h = self.h
        if row >= h.MiRows or col >= h.MiCols:
            return
        if txSz == TX_4X4 or depth == MAX_VARTX_DEPTH:
            txfm_split = 0
        else:
            txfm_split = self.sym(self.cdf["Txfm_Split"][self.txfm_split_ctx(row, col, txSz)], "txfm_split")
        w4 = T.Tx_Width[txSz] // MI_SIZE
        h4 = T.Tx_Height[txSz] // MI_SIZE
        if txfm_split:
            subTxSz = T.Split_Tx_Size[txSz]
            stepW = T.Tx_Width[subTxSz] // MI_SIZE
            stepH = T.Tx_Height[subTxSz] // MI_SIZE
            for i in range(0, h4, stepH):
                for j in range(0, w4, stepW):
                    self.read_var_tx_size(row + i, col + j, subTxSz, depth + 1)
        else:
            for i in range(h4):
                for j in range(w4):
                    if row + i < h.MiRows and col + j < h.MiCols:
                        self.InterTxSizes[row + i][col + j] = txSz
            self.TxSize = txSz

    def txfm_split_ctx(self, row, col, txSz):
        above = int(self.get_above_tx_width(row, col) < T.Tx_Width[txSz])
        left = int(self.get_left_tx_height(row, col) < T.Tx_Height[txSz])
        size = min(64, max(T.Block_Width[self.MiSize], T.Block_Height[self.MiSize]))
        maxTxSz = find_tx_size(size, size)
        txSzSqrUp = T.Tx_Size_Sqr_Up[txSz]
        return int(txSzSqrUp != maxTxSz) * 3 + (TX_SIZES - 1 - maxTxSz) * 6 + above + left

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
        # unavailable neighbours count as width/height 0; inter neighbours count with their block size
        if self.AvailU and self.IsInters[r - 1][c]:
            aboveW = T.Block_Width[self.MiSizes[r - 1][c]]
        else:
            aboveW = self.get_above_tx_width(r, c) if self.AvailU else 0
        if self.AvailL and self.IsInters[r][c - 1]:
            leftH = T.Block_Height[self.MiSizes[r][c - 1]]
        else:
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
            if self.Skips[row - 1][col] and self.IsInters[row - 1][col]:
                return T.Block_Width[self.MiSizes[row - 1][col]]
        return T.Tx_Width[self.InterTxSizes[row - 1][col]]

    def get_left_tx_height(self, row, col):
        if col == self.MiCol:
            if not self.AvailL:
                return 64
            if self.Skips[row][col - 1] and self.IsInters[row][col - 1]:
                return T.Block_Height[self.MiSizes[row][col - 1]]
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
                    if self.is_inter and not self.Lossless and plane == 0:
                        baseX = (miColChunk >> subX) * MI_SIZE
                        baseY = (miRowChunk >> subY) * MI_SIZE
                        self.transform_tree(baseX, baseY, num4x4W * 4, num4x4H * 4)
                        continue
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

    def transform_tree(self, startX, startY, w, hh):
        h = self.h
        maxX = h.MiCols * MI_SIZE
        maxY = h.MiRows * MI_SIZE
        if startX >= maxX or startY >= maxY:
            return
        row = startY >> MI_SIZE_LOG2
        col = startX >> MI_SIZE_LOG2
        lumaTxSz = self.InterTxSizes[row][col]
        lumaW = T.Tx_Width[lumaTxSz]
        lumaH = T.Tx_Height[lumaTxSz]
        if w <= lumaW and hh <= lumaH:
            txSz = find_tx_size(w, hh)
            self.transform_block(0, startX, startY, txSz, 0, 0)
        elif w > hh:
            self.transform_tree(startX, startY, w // 2, hh)
            self.transform_tree(startX + w // 2, startY, w // 2, hh)
        elif w < hh:
            self.transform_tree(startX, startY, w, hh // 2)
            self.transform_tree(startX, startY + hh // 2, w, hh // 2)
        else:
            self.transform_tree(startX, startY, w // 2, hh // 2)
            self.transform_tree(startX + w // 2, startY, w // 2, hh // 2)
            self.transform_tree(startX, startY + hh // 2, w // 2, hh // 2)
            self.transform_tree(startX + w // 2, startY + hh // 2, w // 2, hh // 2)

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

    def compute_prediction(self):
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
        if self.is_inter:
            if self.h.reduced_tx_set or txSzSqrUp == TX_32X32: return TX_SET_INTER_3
            if txSzSqr == TX_16X16: return TX_SET_INTER_2
            return TX_SET_INTER_1
        if txSzSqrUp == TX_32X32: return TX_SET_DCTONLY
        if self.h.reduced_tx_set: return TX_SET_INTRA_2
        if txSzSqr == TX_16X16: return TX_SET_INTRA_2
        return TX_SET_INTRA_1

    def transform_type(self, x4, y4, txSz):
        h = self.h
        s = self.get_tx_set(txSz)
        qidx = self.get_qindex(1, self.segment_id) if h.segmentation_enabled else h.base_q_idx
        if s > 0 and qidx > 0 and self.is_inter:
            if s == TX_SET_INTER_1:
                v = self.sym(self.cdf["Inter_Tx_Type_Set1"][T.Tx_Size_Sqr[txSz]], "inter_tx_type")
                TxType = T.Tx_Type_Inter_Inv_Set1[v]
            elif s == TX_SET_INTER_2:
                v = self.sym(self.cdf["Inter_Tx_Type_Set2"], "inter_tx_type")
                TxType = T.Tx_Type_Inter_Inv_Set2[v]
            else:
                v = self.sym(self.cdf["Inter_Tx_Type_Set3"][T.Tx_Size_Sqr[txSz]], "inter_tx_type")
                TxType = T.Tx_Type_Inter_Inv_Set3[v]
        elif s > 0 and qidx > 0:
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
        if self.is_inter:
            x4 = max(self.MiCol, blockX << self.h.subsampling_x)
            y4 = max(self.MiRow, blockY << self.h.subsampling_y)
            txType = self.TxTypes[y4][x4]
            if not T.Tx_Type_In_Set_Inter[txSet][txType]:
                return DCT_DCT
            return txType
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
                return self.NS(numSyms - mk, "subexp_unif_bools") + mk
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
