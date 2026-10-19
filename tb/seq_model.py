"""Frame-sequence state for the tile model (inter frames): what one frame leaves for the next.

The spec's reference frame update process (7.20) saves, per slot, the CDFs after the frame-end update, the motion
field (7.19 MfRefFrames / MfMvs), the segmentation map and the frame-level values the header parser already keeps
(tb/obu_parser.py). A frame then loads its CDFs from ref_frame_idx[primary_ref_frame] (counts zeroed), its
PrevSegmentIds, and estimates the motion field (7.9) from the saved motion vectors for temporal candidates.

Usage (tools/xcheck_tile.py): seq = SequenceState(); for each decoded frame in order:
    ctx = seq.start_frame(h)                 # h: the parser's ParsedFrameHeader (ref_frame_idx, OrderHints, ...)
    decoders = [TileDecoder(hdr, data, hook, frame=ctx) ... decode_tile()] in TileNum order
    seq.end_frame(h, decoders)
and seq.show_existing(h) for a show_existing_frame header (a shown key frame refreshes every slot).
"""
import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import av1_tables as T      # noqa: E402
import tile_model as tm     # noqa: E402

NUM_REF_FRAMES, REFS_PER_FRAME = 8, 7
INTRA_FRAME, NONE = tm.INTRA_FRAME, tm.NONE
LAST_FRAME, LAST2_FRAME, BWDREF_FRAME, ALTREF2_FRAME, ALTREF_FRAME, GOLDEN_FRAME = 1, 2, 5, 6, 7, 4
KEY_FRAME, INTER_FRAME, INTRA_ONLY_FRAME, SWITCH_FRAME = 0, 1, 2, 3
PRIMARY_REF_NONE = 7
MV_INVALID = tm.MV_INVALID


def zero_counts(cdfs):
    """load_cdfs: the last entry of every CDF array (the adaptation count) is set to 0."""
    def walk(v):
        if isinstance(v, list):
            if v and not isinstance(v[0], list):
                v[-1] = 0
            else:
                for x in v:
                    walk(x)
    for v in cdfs.values():
        walk(v)
    return cdfs


class Slot:
    def __init__(self):
        self.cdfs = None
        self.mvs = None             # MfMvs [row][col] -> [row, col] (7.19)
        self.ref_frames = None      # MfRefFrames [row][col]
        self.segment_ids = None
        self.mi_rows = self.mi_cols = 0
        self.frame_type = KEY_FRAME
        self.order_hint = 0
        self.saved_order_hints = [0] * 8


class SequenceState:
    def __init__(self):
        self.slots = [Slot() for _ in range(NUM_REF_FRAMES)]

    # ---- helpers
    @staticmethod
    def relative_dist(h, a, b):
        if not h.enable_order_hint:
            return 0
        diff = a - b
        m = 1 << (h.OrderHintBits - 1)
        return (diff & (m - 1)) - (diff & m)

    # ---- the frame start: load_cdfs / load_previous_segment_ids / motion_field_estimation
    def start_frame(self, h):
        cdfs = prev_seg = None
        if h.primary_ref_frame != PRIMARY_REF_NONE:
            slot = self.slots[h.ref_frame_idx[h.primary_ref_frame]]
            assert slot.cdfs is not None, "primary_ref_frame points at a slot no frame has filled"
            cdfs = zero_counts(copy.deepcopy(slot.cdfs))
            if h.segmentation_enabled and slot.mi_rows == h.MiRows and slot.mi_cols == h.MiCols and slot.segment_ids is not None:
                prev_seg = slot.segment_ids
        motion_field = self.motion_field_estimation(h) if (not h.FrameIsIntra and h.use_ref_frame_mvs) else None
        return tm.FrameCtx(cdfs=cdfs, prev_segment_ids=prev_seg, motion_field=motion_field)

    # ---- 7.9 motion field estimation
    def motion_field_estimation(self, h):
        w8, h8 = h.MiCols >> 1, h.MiRows >> 1
        mf = [[[[MV_INVALID, MV_INVALID] for _ in range(w8)] for _ in range(h8)] for _ in range(8)]
        lastIdx = h.ref_frame_idx[0]
        curGoldOrderHint = h.OrderHints[GOLDEN_FRAME]
        lastAltOrderHint = self.slots[lastIdx].saved_order_hints[ALTREF_FRAME]
        useLast = lastAltOrderHint != curGoldOrderHint
        if useLast:
            self.project(h, mf, LAST_FRAME, -1)
        refStamp = tm.MFMV_STACK_SIZE - 2
        if self.relative_dist(h, h.OrderHints[BWDREF_FRAME], h.order_hint) > 0:
            if self.project(h, mf, BWDREF_FRAME, 1):
                refStamp -= 1
        if self.relative_dist(h, h.OrderHints[ALTREF2_FRAME], h.order_hint) > 0:
            if self.project(h, mf, ALTREF2_FRAME, 1):
                refStamp -= 1
        if self.relative_dist(h, h.OrderHints[ALTREF_FRAME], h.order_hint) > 0 and refStamp >= 0:
            if self.project(h, mf, ALTREF_FRAME, 1):
                refStamp -= 1
        if refStamp >= 0:
            self.project(h, mf, LAST2_FRAME, -1)
        return mf

    def project(self, h, mf, src, dstSign):
        srcIdx = h.ref_frame_idx[src - LAST_FRAME]
        slot = self.slots[srcIdx]
        w8, h8 = h.MiCols >> 1, h.MiRows >> 1
        if slot.mi_rows != h.MiRows or slot.mi_cols != h.MiCols or slot.frame_type == INTRA_ONLY_FRAME:
            return 0
        if slot.ref_frames is None:
            return 0
        for y8 in range(h8):
            for x8 in range(w8):
                row, col = 2 * y8 + 1, 2 * x8 + 1
                srcRef = slot.ref_frames[row][col]
                if srcRef > INTRA_FRAME:
                    refToCur = self.relative_dist(h, h.OrderHints[src], h.order_hint)
                    refOffset = self.relative_dist(h, h.OrderHints[src], slot.saved_order_hints[srcRef])
                    posValid = (abs(refToCur) <= tm.MAX_FRAME_DISTANCE and abs(refOffset) <= tm.MAX_FRAME_DISTANCE
                                and refOffset > 0)
                    if posValid:
                        mv = slot.mvs[row][col]
                        projMv = self.mv_projection(mv, refToCur * dstSign, refOffset)
                        posValid, PosY8, PosX8 = self.block_position(h, x8, y8, dstSign, projMv)
                        if posValid:
                            for dst in range(LAST_FRAME, ALTREF_FRAME + 1):
                                refToDst = self.relative_dist(h, h.order_hint, h.OrderHints[dst])
                                mf[dst][PosY8][PosX8] = self.mv_projection(mv, refToDst, refOffset)
        return 1

    @staticmethod
    def mv_projection(mv, numerator, denominator):
        clippedDenominator = min(tm.MAX_FRAME_DISTANCE, denominator)
        clippedNumerator = tm.clip3(-tm.MAX_FRAME_DISTANCE, tm.MAX_FRAME_DISTANCE, numerator)
        out = [0, 0]
        for i in range(2):
            scaled = tm.round2signed(mv[i] * clippedNumerator * T.Div_Mult[clippedDenominator], 14)
            out[i] = tm.clip3(-(1 << 14) + 1, (1 << 14) - 1, scaled)
        return out

    @staticmethod
    def block_position(h, x8, y8, dstSign, projMv):
        valid = [1]

        def project(v8, delta, max8, maxOff8):
            base8 = (v8 >> 3) << 3
            if delta >= 0:
                offset8 = delta >> (3 + 1 + tm.MI_SIZE_LOG2)
            else:
                offset8 = -((-delta) >> (3 + 1 + tm.MI_SIZE_LOG2))
            v8 += dstSign * offset8
            if v8 < 0 or v8 >= max8 or v8 < base8 - maxOff8 or v8 >= base8 + 8 + maxOff8:
                valid[0] = 0
            return v8
        PosY8 = project(y8, projMv[0], h.MiRows >> 1, tm.MAX_OFFSET_HEIGHT)
        PosX8 = project(x8, projMv[1], h.MiCols >> 1, tm.MAX_OFFSET_WIDTH)
        return valid[0], PosY8, PosX8

    # ---- the frame end: 7.19 motion field storage, 7.20 reference frame update
    def end_frame(self, h, decoders, frame_cdfs=None):
        """decoders: the frame's TileDecoders in TileNum order (their per-4x4 arrays cover their own tiles);
        frame_cdfs: the CDFs the frame started with (None: the defaults for base_q_idx)."""
        R, Cc = h.MiRows, h.MiCols
        ref_frames = [[NONE] * Cc for _ in range(R)]
        mvs = [[[0, 0] for _ in range(Cc)] for _ in range(R)]
        seg = [[0] * Cc for _ in range(R)]
        for d in decoders:
            th = d.h
            for row in range(th.MiRowStart, min(th.MiRowEnd, R)):
                for col in range(th.MiColStart, min(th.MiColEnd, Cc)):
                    seg[row][col] = d.SegmentIds[row][col]
                    rf = d.RefFrames[row][col]
                    if rf is None:
                        continue
                    for lst in range(2):
                        r = rf[lst]
                        if r > INTRA_FRAME:
                            dist = self.relative_dist(h, h.OrderHints[r], h.order_hint)
                            if dist < 0:
                                mv = d.Mvs[row][col][lst]
                                if abs(mv[0]) <= tm.REFMVS_LIMIT and abs(mv[1]) <= tm.REFMVS_LIMIT:
                                    ref_frames[row][col] = r
                                    mvs[row][col] = list(mv)
        # frame end update: the CDFs of tile context_update_tile_id (final, with counts) unless disabled
        if h.disable_frame_end_update_cdf:
            cdfs = frame_cdfs if frame_cdfs is not None else tm.make_cdfs(h.base_q_idx)
        else:
            cdfs = decoders[h.context_update_tile_id].cdf
        for i in range(NUM_REF_FRAMES):
            if (h.refresh_frame_flags >> i) & 1:
                s = Slot()
                s.cdfs = copy.deepcopy(cdfs)
                s.mvs, s.ref_frames, s.segment_ids = mvs, ref_frames, seg
                s.mi_rows, s.mi_cols = R, Cc
                s.frame_type = h.frame_type
                s.order_hint = h.order_hint
                s.saved_order_hints = list(h.OrderHints)
                self.slots[i] = s

    def show_existing(self, h):
        """show_existing_frame: a shown key frame refreshes every slot with the shown one (7.20 after 7.21)."""
        if h.frame_type == KEY_FRAME:
            src = self.slots[h.frame_to_show_map_idx]
            for i in range(NUM_REF_FRAMES):
                self.slots[i] = copy.deepcopy(src)
