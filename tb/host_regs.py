"""The decoder's host-side protocol, with no simulator dependency: the register map of rtl/dec_host.sv, the
header structs packed the way syn_pkg declares them (hdr_t, rec_hdr_t, lf_hdr_t), and HostDecoder, the sequence
that drives a frame through the register file (headers, CDF default rows, tile bytes, stage starts, done polls,
picture read-back). tb/test_dec_host.py uses the packing from here under cocotb; tools/kv260/decode.py runs
HostDecoder on the KV260 over /dev/mem. One definition of the protocol, so the board does what the test does."""
import array
import sys
import os
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import cdf_map as M     # noqa: E402

# coordinate widths: must match syn_pkg::MIW / SBW / PXW (4x4-unit, 64x64-superblock and pixel positions / counts)
MIW, SBW, PXW = 13, 9, 15

HDR_FIELDS = [  # (name, width) in syn_pkg::hdr_t order (MSB first)
    ("mi_rows", MIW), ("mi_cols", MIW), ("mi_row_start", MIW), ("mi_row_end", MIW), ("mi_col_start", MIW), ("mi_col_end", MIW),
    ("ssx", 1), ("ssy", 1), ("mono", 1), ("sb128", 1), ("bit_depth", 4), ("seg_enabled", 1), ("seg_preskip", 1),
    ("last_active_segid", 3), ("seg_skip_en", 8), ("lossless", 8), ("seg_qidx", 64), ("base_q_idx", 8), ("tx_mode", 2),
    ("reduced_tx_set", 1), ("allow_sct", 1), ("allow_intrabc", 1), ("enable_filter_intra", 1), ("enable_cdef", 1), ("cdef_bits", 2),
    ("coded_lossless", 1), ("delta_q_present", 1), ("delta_q_res", 2), ("delta_lf_present", 1), ("delta_lf_res", 2),
    ("delta_lf_multi", 1), ("disable_cdf_update", 1), ("lr_type", 6), ("lr_size", 6), ("frame_height", PXW), ("upscaled_width", PXW),
    ("use_superres", 1), ("superres_denom", 5)]
REC_FIELDS = [  # syn_pkg::rec_hdr_t order (MSB first)
    ("enable_intra_edge_filter", 1), ("dq_ydc", 7), ("dq_udc", 7), ("dq_uac", 7), ("dq_vdc", 7), ("dq_vac", 7),
    ("seg_altq_en", 8), ("seg_altq", 72), ("using_qmatrix", 1), ("qm_level", 96)]
ENT_FIELDS = [  # syn_pkg::mv_ent_t order (MSB first): one 4x4 unit's motion info in mv_mem
    ("is_inter", 1), ("is_intrabc", 1), ("ref0", 4), ("ref1", 4), ("mv_row", 16), ("mv_col", 16), ("mv1_row", 16), ("mv1_col", 16),
    ("newmv_fam", 1), ("globalmv_fam", 1), ("bsize", 5), ("skip_mode", 1), ("comp_group_idx", 1), ("compound_idx", 1),
    ("interp0", 2), ("interp1", 2), ("seg_pred", 1)]
IHDR_FIELDS = [  # syn_pkg::ihdr_t order (MSB first)
    ("is_inter", 1), ("reference_select", 1), ("skip_mode_present", 1), ("skip_mode_frame0", 3), ("skip_mode_frame1", 3),
    ("allow_hp_mv", 1), ("force_integer_mv", 1), ("interp_filter", 3), ("motion_mode_switchable", 1), ("allow_warped_motion", 1),
    ("use_ref_frame_mvs", 1), ("enable_dual_filter", 1), ("enable_interintra_compound", 1), ("enable_masked_compound", 1),
    ("enable_jnt_comp", 1), ("enable_order_hint", 1), ("order_hint_bits", 4), ("order_hint", 8), ("order_hints", 56),
    ("ref_sign_bias", 7), ("ref_scaled", 7), ("gm_type", 14), ("gm_params", 1008), ("seg_ref_en", 8), ("seg_ref_data", 24),
    ("seg_globalmv_en", 8), ("seg_update_map", 1), ("seg_temporal_update", 1)]
LF_FIELDS = [  # syn_pkg::lf_hdr_t order (MSB first)
    ("frame_width", PXW), ("frame_height", PXW), ("level", 24), ("sharpness", 3), ("delta_enabled", 1),
    ("ref_deltas", 56), ("mode_deltas", 14), ("delta_lf_multi", 1), ("seg_lf_en", 32), ("seg_lf_data", 224)]


def pack(fields, vals):
    v = 0
    for name, w in fields:
        x = vals[name]
        # a value outside the field is a layout bug, not something to truncate quietly (OrderHintBits 8 in a 3-bit field
        # reached the RTL as 0 and broke every relative-distance computation in the reconstruction; test13842 frame 11)
        assert -(1 << (w - 1)) <= x < (1 << w), f"pack: {name} = {x} does not fit {w} bits"
        x &= (1 << w) - 1
        v = (v << w) | x
    return v


def unpack(fields, v):
    out = {}
    total = sum(w for _, w in fields)
    pos = total
    for name, w in fields:
        pos -= w
        out[name] = (v >> pos) & ((1 << w) - 1)
    return out


def ihdr_vals(th):
    """ihdr_t fields for frame header th (all zero for an intra frame, whose parser leaves the inter fields unset)."""
    z = dict((n, 0) for n, _ in IHDR_FIELDS)
    z["force_integer_mv"] = 1                 # every intra frame (the syntax reads whole-sample intrabc MVs)
    z["seg_update_map"] = 1
    if getattr(th, "FrameIsIntra", 1):
        return z
    g = lambda n, d=0: getattr(th, n, d)      # noqa: E731
    v = dict(is_inter=1, reference_select=g("reference_select"), skip_mode_present=g("skip_mode_present"),
             skip_mode_frame0=g("SkipModeFrame", [0, 0])[0], skip_mode_frame1=g("SkipModeFrame", [0, 0])[1],
             allow_hp_mv=g("allow_high_precision_mv"), force_integer_mv=g("force_integer_mv"),
             interp_filter=g("interpolation_filter"), motion_mode_switchable=g("is_motion_mode_switchable"),
             allow_warped_motion=g("allow_warped_motion"), use_ref_frame_mvs=g("use_ref_frame_mvs"),
             enable_dual_filter=g("enable_dual_filter"), enable_interintra_compound=g("enable_interintra_compound"),
             enable_masked_compound=g("enable_masked_compound"), enable_jnt_comp=g("enable_jnt_comp"),
             enable_order_hint=g("enable_order_hint"), order_hint_bits=g("OrderHintBits"), order_hint=g("order_hint"))
    oh, sb, sc, gt, gp = 0, 0, 0, 0, 0
    for i in range(7):
        ref = 1 + i
        oh |= (th.OrderHints[ref] & 0xFF) << (8 * i)
        sb |= (th.RefFrameSignBias[ref] & 1) << i
        xs = ((th.RefUpscaledWidths[ref] << 14) + th.FrameWidth // 2) // th.FrameWidth
        ys = ((th.RefFrameHeights[ref] << 14) + th.FrameHeight // 2) // th.FrameHeight
        sc |= int(xs != (1 << 14) or ys != (1 << 14)) << i
        gt |= (th.GmType[ref] & 3) << (2 * i)
        for j in range(6):
            p = th.gm_params[ref][j]
            assert -(1 << 23) <= p < (1 << 23), ("gm_params out of 24 bits", ref, j, p)
            gp |= (p & 0xFFFFFF) << (24 * (6 * i + j))
    v.update(order_hints=oh, ref_sign_bias=sb, ref_scaled=sc, gm_type=gt, gm_params=gp)
    sr_en = sr_d = sg = 0
    for s in range(8):
        sr_en |= int(bool(th.FeatureEnabled[s][5])) << s
        sr_d |= (th.FeatureData[s][5] & 7) << (3 * s)
        sg |= int(bool(th.FeatureEnabled[s][7])) << s
    v.update(seg_ref_en=sr_en, seg_ref_data=sr_d, seg_globalmv_en=sg,
             seg_update_map=int(getattr(th, "segmentation_update_map", 1)), seg_temporal_update=int(getattr(th, "segmentation_temporal_update", 0)))
    z.update(v)
    return z


def seg_qindex(th, s):
    """get_qindex(ignoreDeltaQ = 1, segment s) from the frame header alone (spec 7.12.2): the base index plus
    the segment's alt-q feature when it is active."""
    if th.segmentation_enabled and th.FeatureEnabled[s][0]:
        return max(0, min(255, th.base_q_idx + th.FeatureData[s][0]))
    return th.base_q_idx


def hdr_vals(th, dec=None, frame_parity=0):
    """hdr_t fields for tile header th. dec (a tile_model.TileDecoder) supplies get_qindex when given; without
    it the header rule above is used, which is the same function."""
    qidx = (lambda s: dec.get_qindex(1, s)) if dec is not None else (lambda s: seg_qindex(th, s))
    v = dict(mi_rows=th.MiRows, mi_cols=th.MiCols, mi_row_start=th.MiRowStart, mi_row_end=th.MiRowEnd,
             mi_col_start=th.MiColStart, mi_col_end=th.MiColEnd, ssx=th.subsampling_x, ssy=th.subsampling_y,
             mono=1 if th.NumPlanes == 1 else 0, sb128=th.use_128x128_superblock, bit_depth=th.BitDepth,
             seg_enabled=th.segmentation_enabled, seg_preskip=th.SegIdPreSkip, last_active_segid=th.LastActiveSegId,
             seg_skip_en=sum((th.FeatureEnabled[s][6] & 1) << s for s in range(8)),
             lossless=sum((th.LosslessArray[s] & 1) << s for s in range(8)),
             seg_qidx=sum((qidx(s) & 0xFF) << (8 * s) for s in range(8)),
             base_q_idx=th.base_q_idx, tx_mode=th.TxMode, reduced_tx_set=th.reduced_tx_set,
             allow_sct=th.allow_screen_content_tools, allow_intrabc=th.allow_intrabc, enable_filter_intra=th.enable_filter_intra, enable_cdef=th.enable_cdef,
             cdef_bits=th.cdef_bits, coded_lossless=th.CodedLossless, delta_q_present=th.delta_q_present,
             delta_q_res=th.delta_q_res, delta_lf_present=th.delta_lf_present, delta_lf_res=th.delta_lf_res,
             delta_lf_multi=th.delta_lf_multi, disable_cdf_update=th.disable_cdf_update,
             lr_type=sum(th.FrameRestorationType[p] << (2 * p) for p in range(3)),
             lr_size=sum(((th.LoopRestorationSize[p].bit_length() - 1 - 5) & 3) << (2 * p) for p in range(3)),   # log2(size) - 5: 32 .. 256
             frame_height=th.FrameHeight, upscaled_width=th.UpscaledWidth,
             use_superres=th.use_superres, superres_denom=th.SuperresDenom if th.use_superres else 8)
    return v


def rec_vals(th):
    return dict(enable_intra_edge_filter=th.enable_intra_edge_filter,
                dq_ydc=th.DeltaQYDc, dq_udc=th.DeltaQUDc, dq_uac=th.DeltaQUAc, dq_vdc=th.DeltaQVDc, dq_vac=th.DeltaQVac if hasattr(th, "DeltaQVac") else th.DeltaQVAc,
                seg_altq_en=sum((th.FeatureEnabled[s][0] & 1) << s for s in range(8)),
                seg_altq=sum((th.FeatureData[s][0] & 0x1FF) << (9 * s) for s in range(8)),
                using_qmatrix=th.using_qmatrix,
                qm_level=sum((th.SegQMLevel[p][s] & 0xF) << ((p * 8 + s) * 4) for p in range(3) for s in range(8)))


CDEF_FIELDS = [("damping", 3), ("bits", 2), ("y_str", 48), ("uv_str", 48)]   # syn_pkg::cdef_hdr_t order (MSB first)
CH_WORDS = 4


def cdef_vals(th):
    return dict(damping=th.cdef_damping, bits=th.cdef_bits,
                y_str=sum((th.cdef_y_strengths[i] & 63) << (6 * i) for i in range(8)),
                uv_str=sum((th.cdef_uv_strengths[i] & 63) << (6 * i) for i in range(8)))


def cdef_on(hdr):
    return bool(hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc)


def lf_vals(th):
    return dict(frame_width=th.FrameWidth, frame_height=th.FrameHeight,
                level=sum((th.loop_filter_level[i] & 63) << (6 * i) for i in range(4)),
                sharpness=th.loop_filter_sharpness, delta_enabled=th.loop_filter_delta_enabled,
                ref_deltas=sum((th.loop_filter_ref_deltas[i] & 0x7F) << (7 * i) for i in range(8)),
                mode_deltas=sum((th.loop_filter_mode_deltas[i] & 0x7F) << (7 * i) for i in range(2)),
                delta_lf_multi=th.delta_lf_multi,
                seg_lf_en=sum((th.FeatureEnabled[s][1 + i] & 1) << (8 * i + s) for i in range(4) for s in range(8)),
                seg_lf_data=sum((th.FeatureData[s][1 + i] & 0x7F) << (7 * (8 * i + s)) for i in range(4) for s in range(8)))


# ---------------------------------------------------------------- register map (rtl/dec_host.sv)
R_ID, R_CFG, R_CTRL, R_START, R_STATUS, R_PERF_SYN, R_PERF_REC = 0x000, 0x004, 0x008, 0x00C, 0x010, 0x014, 0x018
R_TILE_ENG, R_TILE_COL, R_TILE_FIFO = 0x038, 0x060, 0x070      # tile engines: mask, columns per engine (+4e), FIFO status per engine (+4e)
CTRL_ENG_SHIFT, CTRL_ENG_MASK = 24, 3 << 24                     # CTRL[25:24]: the engine DATA1 / DATA4 push to
R_DATA1, R_DATA4, R_HADDR, R_HDATA, R_DEF_ADDR = 0x01C, 0x020, 0x024, 0x028, 0x02C
R_HDATA2, R_HDATA4 = 0x030, 0x034
R_DMA_CFG, R_DMA_START, R_DMA_STATUS, R_DMA_BEATS, R_DMA_PLANE = 0x180, 0x184, 0x188, 0x18C, 0x190   # + 0x10 per plane
DMA_BUSY, DMA_DONE, DMA_ERR, DMA_FLUSH_BUSY = 1, 2, 4, 8
DMA_CFG_AUTO = 1 << 8                      # window builds: released superblock rows flushed to the plane bases as they finish
R_DEF, R_HDR, R_RH, R_LH, R_CH = 0x040, 0x080, 0x0C0, 0x100, 0x140
R_IH = 0x200                                 # ihdr_t, 64 words
R_MF_ADDR, R_MF_DATA, R_SP_ADDR, R_SP_DATA0, R_SP_DATA1 = 0x300, 0x304, 0x308, 0x30C, 0x310   # inter frame bands
R_SLOT, R_REF_SLOT, R_REF_CFG = 0x400, 0x600, 0x604        # reference slot table (0x40 per slot, 0x10 per plane), ref_frame_idx, fmt16
ID_VALUE = 0xA71D0001
ST_TILE_BUSY, ST_TILE_DONE, ST_UNSUP, ST_LF_DONE, ST_FIFO_BAD, ST_FIFO_EMPTY = 1 << 0, 1 << 1, 1 << 2, 1 << 6, 1 << 14, 1 << 15
START_TILE, START_FRAME, START_LF = 1, 2, 4
ST_FRAME_DONE = 1 << 4
CTRL_RST, CTRL_PIPE, CTRL_EOS = 1, 1 << 1, 1 << 8
HDR_WORDS, RH_WORDS, LH_WORDS = 8, 7, 13     # words of each header window actually written
IH_WORDS = (sum(w for _, w in IHDR_FIELDS) + 31) // 32


def words(v, n):
    return [(v >> (32 * i)) & 0xFFFFFFFF for i in range(n)]


def dma_layout(hdr, base, visible=True):
    """Where the DMA engine puts the picture: per plane (base, stride, W, H) with the dav1d sample format
    (bytes for 8-bit, 16-bit little-endian otherwise) and the stride rounded up to 8 bytes; planes follow each
    other. Returns (planes, total bytes)."""
    bps = 1 if hdr.BitDepth == 8 else 2
    planes, off = [], 0
    for p in range(hdr.NumPlanes):
        W, H = plane_size(hdr, p, visible)
        stride = (W * bps + 7) & ~7
        planes.append((base + off, stride, W, H))
        off += stride * H
    return planes, off


def motion_field_seq(mf, mi_rows, mi_cols):
    """MotionFieldMvs[ref][y8][x8] (seq_model; None = none) as the band's register writes: every 8x8 position of the
    frame, holes as invalid entries."""
    seq = []
    for ref in range(1, 8):
        for y8 in range(mi_rows >> 1):             # (MiRows >> 1) x (MiCols >> 1) entries: one more row / column aliases onto 0
            for x8 in range(mi_cols >> 1):
                mv = mf[ref][y8][x8] if (mf is not None and y8 < len(mf[ref]) and x8 < len(mf[ref][y8])) else [M.tm.MV_INVALID] * 2
                valid = mv[0] != M.tm.MV_INVALID
                seq.append((R_MF_ADDR, (int(valid) << 31) | ((ref - 1) << 24) | (y8 << 12) | x8))
                seq.append((R_MF_DATA, (((mv[0] & 0xFFFF) << 16) | (mv[1] & 0xFFFF)) if valid else 0))
    return seq


def seg_map_seq(prev, mi_rows, mi_cols):
    """PrevSegmentIds (None = zeros) as the band's register writes, 16 ids per word."""
    seq = []
    for r in range(mi_rows):
        for wc in range((mi_cols + 15) // 16):
            w = 0
            if prev is not None:
                for k in range(16):
                    c = wc * 16 + k
                    if c < mi_cols:
                        w |= (prev[r][c] & 7) << (3 * k)
            seq += [(R_SP_ADDR, (r << 16) | wc), (R_SP_DATA0, w & 0xFFFFFFFF), (R_SP_DATA1, w >> 32)]
    return seq


def ref_seq(ph, store, fmt16):
    """The reference slot table (0x400: per slot and plane the picture's base / stride in DDR, per slot its size and
    the 7.11.3.3 scale factors for this frame) and REF_SLOT / REF_CFG for the inter frame ph. store[slot] =
    dict(planes=[(base, stride) per plane], upscaled_width, frame_height), as the flush left the pictures."""
    seq = []
    ref_slot = 0
    for i in range(7):
        slot = ph.ref_frame_idx[i]
        ref_slot |= slot << (3 * i)
        st = store[slot]
        for p, (base, stride) in enumerate(st["planes"]):
            a = R_SLOT + 0x40 * slot + 0x10 * p
            seq += [(a, base & 0xFFFFFFFF), (a + 4, base >> 32), (a + 8, stride)]
        xsc = ((st["upscaled_width"] << 14) + ph.FrameWidth // 2) // ph.FrameWidth
        ysc = ((st["frame_height"] << 14) + ph.FrameHeight // 2) // ph.FrameHeight
        seq += [(R_SLOT + 0x40 * slot + 0x30, st["upscaled_width"] | (st["frame_height"] << 16)),
                (R_SLOT + 0x40 * slot + 0x34, xsc | (ysc << 16))]
    seq += [(R_REF_SLOT, ref_slot), (R_REF_CFG, 1 if fmt16 else 0)]
    return seq


def dma_regs(hdr, base, visible=True):
    """The (address, value) register writes that program the DMA engine for the picture at base."""
    planes, total = dma_layout(hdr, base, visible)
    seq = [(R_DMA_CFG, (0 if hdr.BitDepth == 8 else 1) | ((hdr.NumPlanes - 1) << 4))]
    for p, (b, stride, W, H) in enumerate(planes):
        r = R_DMA_PLANE + 0x10 * p
        seq += [(r, b & 0xFFFFFFFF), (r + 4, b >> 32), (r + 8, stride), (r + 12, (H << 16) | W)]
    return seq, planes, total


def unpack_dma(hdr, data, planes):
    """The DMA image back into plane row lists (data: the bytes from the first plane's base)."""
    bps = 1 if hdr.BitDepth == 8 else 2
    base0 = planes[0][0]
    out = []
    for b, stride, W, H in planes:
        rows = []
        for y in range(H):
            off = b - base0 + y * stride
            row = data[off:off + W * bps]
            if bps == 1:
                rows.append(list(row))
            else:
                a = array.array("H"); a.frombytes(row)            # 16-bit little-endian samples (the host is little-endian)
                rows.append(a.tolist())
        out.append(rows)
    return out


def plane_size(hdr, plane, visible=True):
    """(width, height) of a plane: the visible picture (dav1d's output) or the whole decoded MI area."""
    sx = hdr.subsampling_x if plane else 0
    sy = hdr.subsampling_y if plane else 0
    if visible:
        return (hdr.UpscaledWidth + sx) >> sx, (hdr.FrameHeight + sy) >> sy
    return (hdr.MiCols * 4) >> sx, (hdr.MiRows * 4) >> sy


class HostError(Exception):
    pass


class HostDecoder:
    """Drives one register file (regs.rd(addr) / regs.wr(addr, value), each one 32-bit bus transfer) through a
    frame the way tb/test_dec_host.py does in simulation. Synchronous: meant for software on the PS."""
    def __init__(self, regs, poll_timeout=30.0, log=None, fast=None):
        """fast: an optional bulk backend (tools/kv260/hostlib.py over the C helper) with write_seq(addrs, vals),
        push(data, timeout) -> bytes pushed or -1, poll(addr, mask, value, timeout) -> value or None, and
        read_plane(plane, W, H, bd8) -> rows; the protocol stays here, only the word moving goes there."""
        self.r = regs
        self.fast = fast
        self.poll_timeout = poll_timeout
        self.log = log or (lambda s: None)
        self.stats = dict(frames=0, tiles=0, bytes=0, pixels=0, writes=0, reads=0)
        self._def_seq = {}                      # base_q_idx -> the CDF default rows as a write sequence

    # -- primitives (TS_REG_LOG=<file>: every register write and read as "w addr value" / "r addr value" lines, so a
    # simulation twin and a board run of the same stream can be diffed; the bulk backend's traffic is not logged)
    _reg_log = open(os.environ["TS_REG_LOG"], "w") if os.environ.get("TS_REG_LOG") else None

    def wr(self, a, v):
        self.r.wr(a, v)
        self.stats["writes"] += 1
        if self._reg_log: self._reg_log.write(f"w {a:03x} {v & 0xFFFFFFFF:08x}\n")

    def rd(self, a):
        self.stats["reads"] += 1
        v = self.r.rd(a)
        if self._reg_log: self._reg_log.write(f"r {a:03x} {v & 0xFFFFFFFF:08x}\n")
        return v

    def write_wide(self, base, v, nwords):
        for i, w in enumerate(words(v, nwords)):
            self.wr(base + 4 * i, w)

    def write_motion_field(self, mf, mi_rows, mi_cols):
        """MotionFieldMvs[ref][y8][x8] (seq_model; None = none) into the band: every 8x8 position of the frame, holes as
        invalid entries."""
        self.write_seq(motion_field_seq(mf, mi_rows, mi_cols))

    def write_seg_map(self, prev, mi_rows, mi_cols):
        """PrevSegmentIds (None = zeros) into the band, 16 ids per word."""
        self.write_seq(seg_map_seq(prev, mi_rows, mi_cols))

    def write_refs(self, ph, store, fmt16):
        """The reference slot table and ref_frame_idx for an inter frame: store[slot] = dict(planes=[(base, stride), ...],
        upscaled_width, frame_height) describes the picture each slot holds (where the flush put it, docs/picture-memory.md)."""
        self.write_seq(ref_seq(ph, store, fmt16))

    def read_cdf_rows(self, rows=None):
        """The tile's working CDF store (the adapted rows after a tile; only between tiles): DEF_ADDR bit 31 reads a
        row back into DEF_DATA. rows: an iterable of row addresses (default all of them)."""
        out = {}
        for i in (range(M.ROWS) if rows is None else rows):
            self.wr(R_DEF_ADDR, i | (1 << 31))
            v = 0
            for k in range(8):
                v |= self.rd(R_DEF + 4 * k) << (32 * k)
            out[i] = v & ((1 << M.ROW_BITS) - 1)
        return out

    def write_seq(self, pairs):
        """(address, value) pairs in order, or a prebuilt (addrs, vals) pair of arrays for the fast backend."""
        if self.fast is not None:
            if isinstance(pairs, tuple):
                self.fast.write_seq(*pairs)
                self.stats["writes"] += len(pairs[0])
            else:
                self.fast.write_seq([a for a, _ in pairs], [v for _, v in pairs])
                self.stats["writes"] += len(pairs)
        elif isinstance(pairs, tuple):
            for a, v in zip(*pairs):
                self.wr(a, v)
        else:
            for a, v in pairs:
                self.wr(a, v)

    def default_rows_seq(self, base_q_idx):
        """The CDF default rows of a base_q_idx as a write sequence (8 data words + the address write per row),
        computed once: the rows take 60 ms to generate and 11 k words to marshal."""
        seq = self._def_seq.get(base_q_idx)
        if seq is None:
            pairs = []
            for i, row in enumerate(M.default_rows(base_q_idx)):
                pairs += [(R_DEF + 4 * k, w) for k, w in enumerate(words(row, 8))]
                pairs.append((R_DEF_ADDR, i))
            addrs, vals = [a for a, _ in pairs], [v for _, v in pairs]
            seq = self._def_seq[base_q_idx] = self.fast.seq_arrays(addrs, vals) if self.fast is not None else (addrs, vals)
        return seq

    def poll(self, a, mask, value, what):
        if self.fast is not None:
            v = self.fast.poll(a, mask, value, self.poll_timeout)
            self.stats["reads"] += 1
            if v is None:
                raise HostError(f"{what}: timeout after {self.poll_timeout}s (0x{a:03x} reads 0x{self.rd(a):08x})")
            return v
        t0 = time.monotonic()
        while True:
            v = self.rd(a)
            if (v & mask) == value:
                return v
            if time.monotonic() - t0 > self.poll_timeout:
                raise HostError(f"{what}: timeout after {self.poll_timeout}s (0x{a:03x} reads 0x{v:08x})")

    def identify(self):
        ident = self.rd(R_ID)
        if ident != ID_VALUE:
            raise HostError(f"ID register reads 0x{ident:08x}, expected 0x{ID_VALUE:08x}: no decoder at this address?")
        cfg = self.rd(R_CFG)
        self.geom = dict(fbx=cfg >> 16 & 0xFF, fby=cfg >> 8 & 0xFF, lanes=cfg >> 4 & 0xF, uram=cfg & 1, sb128=bool(cfg >> 1 & 1),
                         te=max(1, cfg >> 24 & 0xF))
        return self.geom

    def reset(self):
        self.wr(R_CTRL, CTRL_RST)
        self.rd(R_CTRL)                 # a read behind the posted write: the reset is held a few cycles at least
        self.wr(R_CTRL, 0)

    # -- stages
    def run_tile(self, th, data, tag, dec=None, rows=None):
        """Headers, CDF rows (the defaults of base_q_idx, or rows: the inherited frame CDFs of an inter stream), start,
        the tile bytes, end of stream; returns when the tile is decoded."""
        seq = [(R_HDR + 4 * i, w) for i, w in enumerate(words(pack(HDR_FIELDS, hdr_vals(th, dec, 0)), HDR_WORDS))]
        seq += [(R_RH + 4 * i, w) for i, w in enumerate(words(pack(REC_FIELDS, rec_vals(th)), RH_WORDS))]
        seq += [(R_IH + 4 * i, w) for i, w in enumerate(words(pack(IHDR_FIELDS, ihdr_vals(th)), IH_WORDS))]
        self.write_seq(seq)
        if rows is None:
            self.write_seq(self.default_rows_seq(th.base_q_idx))
        else:
            pairs = []
            for i, row in enumerate(rows):
                pairs += [(R_DEF + 4 * k, w) for k, w in enumerate(words(row, 8))]
                pairs.append((R_DEF_ADDR, i))
            self.write_seq(pairs)
        self.wr(R_START, START_TILE)
        pos = 0
        n_words = (len(data) + 3) // 4
        t0 = time.monotonic()
        if self.fast is not None:
            n = self.fast.push(data, self.poll_timeout)
            if n != len(data):
                raise HostError(f"{tag}: the tile FIFO never freed a word ({n} of {len(data)} bytes pushed)")
            self.stats["writes"] += n_words
            pos = len(data)
        while pos < len(data):
            free = self.rd(R_STATUS) >> 16
            if free == 0:
                if time.monotonic() - t0 > self.poll_timeout:
                    raise HostError(f"{tag}: the tile FIFO never freed a word")
                continue
            for _ in range(free):                        # every push is one FIFO word, tail bytes included
                if pos >= len(data):
                    break
                chunk = data[pos:pos + 4]
                if len(chunk) == 4:
                    self.wr(R_DATA4, int.from_bytes(chunk, "little"))
                    pos += 4
                else:
                    self.wr(R_DATA1, chunk[0])
                    pos += 1
        self.wr(R_CTRL, self.rd(R_CTRL) | CTRL_EOS)
        st = self.poll(R_STATUS, ST_TILE_DONE | ST_UNSUP, ST_TILE_DONE, f"{tag}: tile done")
        if st & ST_UNSUP:
            raise HostError(f"{tag}: the decoder flagged unsupported syntax")
        if st & ST_FIFO_BAD:
            raise HostError(f"{tag}: FIFO overflow or a split register write (STATUS bit 14)")
        self.stats["tiles"] += 1
        self.stats["bytes"] += len(data)

    def run_tile_row(self, tiles, tag, dec=None, rows=None, chunk=1024):
        """The tiles of one tile row ([(th, data), ...] in column order) through TE engines at once (rtl/tile_engines.sv):
        the headers once (the row's rows), the engine mask and each engine's columns, the CDF rows (broadcast), one
        start, then the tiles' bytes round-robin in chunks so no engine starves (reconstruction waits for every
        column's superblock row before moving on), end of stream, tile done. One tile = run_tile."""
        if len(tiles) == 1:
            th, data = tiles[0]
            return self.run_tile(th, data, tag, dec=dec, rows=rows)
        te = getattr(self, "geom", {}).get("te", 1)
        if len(tiles) > te:
            raise HostError(f"{tag}: {len(tiles)} tile columns on a {te}-engine build")
        th0 = tiles[0][0]
        seq = [(R_HDR + 4 * i, w) for i, w in enumerate(words(pack(HDR_FIELDS, hdr_vals(th0, dec, 0)), HDR_WORDS))]
        seq += [(R_RH + 4 * i, w) for i, w in enumerate(words(pack(REC_FIELDS, rec_vals(th0)), RH_WORDS))]
        seq += [(R_IH + 4 * i, w) for i, w in enumerate(words(pack(IHDR_FIELDS, ihdr_vals(th0)), IH_WORDS))]
        for e, (th, _) in enumerate(tiles):
            seq.append((R_TILE_COL + 4 * e, (th.MiColEnd << 16) | th.MiColStart))
        seq.append((R_TILE_ENG, (1 << len(tiles)) - 1))
        self.write_seq(seq)
        if rows is None:
            self.write_seq(self.default_rows_seq(th0.base_q_idx))
        else:
            pairs = []
            for i, row in enumerate(rows):
                pairs += [(R_DEF + 4 * k, w) for k, w in enumerate(words(row, 8))]
                pairs.append((R_DEF_ADDR, i))
            self.write_seq(pairs)
        ctrl0 = self.rd(R_CTRL) & ~(CTRL_ENG_MASK | CTRL_EOS)
        self.wr(R_CTRL, ctrl0)
        self.wr(R_START, START_TILE)
        pos = [0] * len(tiles)
        t0 = time.monotonic()
        while any(pos[e] < len(d) for e, (_, d) in enumerate(tiles)):
            progressed = False
            for e, (_, data) in enumerate(tiles):
                if pos[e] >= len(data):
                    continue
                free = self.rd(R_TILE_FIFO + 4 * e) >> 16
                if free == 0:
                    continue
                self.wr(R_CTRL, ctrl0 | (e << CTRL_ENG_SHIFT))
                # every push is one FIFO word, a tail byte included: the budget counts pushes, not bytes (the single-
                # engine loop's 2026-10-15 lesson; this loop budgeted free * 4 bytes, and test5413 frame 0's 3-byte
                # tail into one free word overflowed engine 0's FIFO, 2026-10-28)
                end = pos[e]
                for _ in range(min(free, chunk // 4)):
                    if end >= len(data):
                        break
                    end += 4 if len(data) - end >= 4 else 1
                piece = data[pos[e]:end]
                if self.fast is not None:
                    n = self.fast.push(piece, self.poll_timeout)
                    if n != len(piece):
                        raise HostError(f"{tag}: engine {e}'s FIFO never freed a word")
                    self.stats["writes"] += (len(piece) + 3) // 4
                else:
                    q = 0
                    while q < len(piece):
                        c = piece[q:q + 4]
                        if len(c) == 4:
                            self.wr(R_DATA4, int.from_bytes(c, "little")); q += 4
                        else:
                            self.wr(R_DATA1, c[0]); q += 1
                pos[e] = end
                progressed = True
            if not progressed and time.monotonic() - t0 > self.poll_timeout:
                raise HostError(f"{tag}: no engine FIFO freed a word")
        self.wr(R_CTRL, ctrl0 | CTRL_EOS)
        st = self.poll(R_STATUS, ST_TILE_DONE | ST_UNSUP, ST_TILE_DONE, f"{tag}: tile row done")
        self.wr(R_TILE_ENG, 0)
        if st & ST_UNSUP:
            raise HostError(f"{tag}: the decoder flagged unsupported syntax")
        if st & ST_FIFO_BAD:
            raise HostError(f"{tag}: FIFO overflow or a split register write (STATUS bit 14)")
        self.stats["tiles"] += len(tiles)
        self.stats["bytes"] += sum(len(d) for _, d in tiles)

    def loop_filter(self, hdr, tag):
        self.write_wide(R_LH, pack(LF_FIELDS, lf_vals(hdr)), LH_WORDS)
        self.wr(R_START, START_LF)
        self.poll(R_STATUS, ST_LF_DONE, ST_LF_DONE, f"{tag}: loop filter done")

    def read_sample(self, plane, x, y):
        a = (plane << 28) | (y << 14) | x
        self.wr(R_HADDR, a)
        # the AXI write and the following read travel on independent channels and the read can overtake the
        # posted write (first board run: every pixel came back as its predecessor): wait for the address to
        # read back before the data read is issued
        for _ in range(1000):
            if self.rd(R_HADDR) == a:
                break
        else:
            raise HostError(f"HADDR never read back 0x{a:08x}")
        return self.rd(R_HDATA)

    def set_haddr(self, plane, x, y):
        a = (plane << 28) | (y << 14) | x
        self.wr(R_HADDR, a)
        for _ in range(1000):
            if self.rd(R_HADDR) == a:
                return
        raise HostError(f"HADDR never read back 0x{a:08x}")

    def read_row(self, hdr, plane, y, W, packed=True):
        """One row of W samples: HDATA4 (four 8-bit samples a word) or HDATA2 (two 16-bit lanes) with the
        address advancing by itself, the odd tail sample by sample; packed=False reads every sample singly."""
        if not packed:
            return [self.read_sample(plane, x, y) for x in range(W)]
        n = 4 if hdr.BitDepth == 8 else 2
        reg = R_HDATA4 if n == 4 else R_HDATA2
        row = []
        self.set_haddr(plane, 0, y)
        for _ in range(W // n):
            v = self.rd(reg)
            if n == 4:
                row += [v & 0xFF, v >> 8 & 0xFF, v >> 16 & 0xFF, v >> 24]
            else:
                row += [v & 0xFFF, v >> 16 & 0xFFF]
        for x in range(len(row), W):
            row.append(self.read_sample(plane, x, y))
        return row

    def read_plane(self, hdr, plane, visible=True, packed=True):
        W, H = plane_size(hdr, plane, visible)
        if self.fast is not None and packed:
            rows = self.fast.read_plane(plane, W, H, hdr.BitDepth == 8)
            self.stats["reads"] += H * (W // (4 if hdr.BitDepth == 8 else 2) + 2)
            self.stats["pixels"] += W * H
            return rows
        rows = [self.read_row(hdr, plane, y, W, packed) for y in range(H)]
        self.stats["pixels"] += W * H
        return rows

    def dma_picture(self, hdr, base, tag, visible=True):
        """The decoded picture into memory at base through the DMA engine; returns the plane layout."""
        seq, planes, total = dma_regs(hdr, base, visible)
        self.write_seq(seq)
        self.wr(R_DMA_START, 1)
        st = self.poll(R_DMA_STATUS, DMA_DONE | DMA_BUSY, DMA_DONE, f"{tag}: picture DMA")
        if st & DMA_ERR:
            raise HostError(f"{tag}: the picture DMA got an AXI error response")
        return planes, total

    # -- a whole stream
    def check_core_build(self, hdr, tag, window=False, n_tile_cols=1):
        if os.environ.get("HOST_SKIP_CORE_CHECK"):           # debugging on a full build: run the tiles / deblocking only, no CDEF / LR
            return
        if hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:
            raise HostError(f"{tag}: CDEF is not in the core build (encode with --enable-cdef=0)")
        if any(t != 0 for t in hdr.FrameRestorationType[:hdr.NumPlanes]):
            raise HostError(f"{tag}: loop restoration is not in the core build (encode with --enable-restoration=0)")
        if window:
            if hdr.use_128x128_superblock:
                raise HostError(f"{tag}: the window build takes 64-pixel superblocks (encode with --sb-size=64)")
            if hdr.NumPlanes > 1 and not (hdr.subsampling_x and hdr.subsampling_y):
                raise HostError(f"{tag}: the window build takes 4:2:0 or monochrome")
            if n_tile_cols != 1:
                raise HostError(f"{tag}: the window build takes one tile column (encode with --tile-columns=0)")
            if hdr.allow_intrabc:
                raise HostError(f"{tag}: intra block copy is not in the window build")

    @staticmethod
    def window_ok(hdr, n_tile_cols=1, fbx=None, fby=None, sb128=False, te=1):
        """Whether a window build takes the frame: sb64, 4:2:0 or mono, one tile column, no intra block copy, and (when
        the image's geometry is known: identify()) a picture no wider than 1 << FBX and no taller than 1 << FBY; sb128 builds (CFG bit 1) take 128x128 superblocks."""
        return (sb128 or not hdr.use_128x128_superblock) and (hdr.NumPlanes == 1 or (hdr.subsampling_x and hdr.subsampling_y)) \
            and n_tile_cols <= max(1, te) and not hdr.allow_intrabc \
            and (fbx is None or hdr.UpscaledWidth <= (1 << fbx)) and (fby is None or hdr.FrameHeight <= (1 << fby))

    def frame_pipe_start(self, hdr, n_tile_cols):
        """Pipelined frame (pipe_en): the in-loop filters trail the tile decoder by superblock rows under the decoder's
        frame sequencer, which needs the filter headers and the tile-column count before the tiles."""
        self.write_wide(R_LH, pack(LF_FIELDS, lf_vals(hdr)), LH_WORDS)
        if cdef_on(hdr):
            self.write_wide(R_CH, pack(CDEF_FIELDS, cdef_vals(hdr)), CH_WORDS)
        self.wr(R_CTRL, CTRL_PIPE | (n_tile_cols << 16))
        self.wr(R_START, START_FRAME)

    def frame_pipe_finish(self, tag):
        self.poll(R_STATUS, ST_FRAME_DONE, ST_FRAME_DONE, f"{tag}: pipelined frame done")
        self.wr(R_CTRL, 0)

    def flush_wait(self, tag):
        """Window builds: after frame_done, until every released row has been written to DDR."""
        self.poll(R_DMA_STATUS, DMA_FLUSH_BUSY, 0, f"{tag}: row flush")

    def decode_frames(self, dec, stage="lf", visible=True, name="stream", packed=True, dma=None, pipe=False, want_planes=True, window=False):
        """dec: an obu_parser.Decoder that has been fed the stream. Yields (frame index, header, planes, timings)
        per coded frame, planes as lists of rows of samples. dma: (base address, read(offset, n) -> bytes) to take
        the picture through the DMA engine instead of the registers. pipe: the deblocking filter runs under the
        frame sequencer, overlapped with the tiles (stage lf). want_planes=False with dma leaves the picture in
        memory and yields planes=None (the unpack into Python lists costs ~8 ms a CIF frame). window: a window
        build (FRAME_WIN > 0): the planes are programmed before the frame and auto-flushed row by row; needs dma
        and pipe."""
        for fi, tile_idx in enumerate(dec.frames):
            tag = f"{name} frame {fi}"
            hdr0 = dec.tiles[tile_idx[0]][0]
            self.check_core_build(hdr0, tag, window, len({dec.tiles[ti][0].MiColStart for ti in tile_idx}))
            t = {}
            t0 = time.monotonic()
            self.reset()
            if window:
                assert dma is not None and pipe and stage == "lf", "a window build decodes pipelined frames into DDR"
                seq, layout, total = dma_regs(hdr0, dma[0], visible)
                seq = [(a, (v | DMA_CFG_AUTO) if a == R_DMA_CFG else v) for a, v in seq]
                self.write_seq(seq)
            if pipe and stage == "lf":
                self.frame_pipe_start(hdr0, len({dec.tiles[ti][0].MiColStart for ti in tile_idx}))
            for ti in tile_idx:
                th, data = dec.tiles[ti]
                self.run_tile(th, data, f"{tag} tile ({th.MiColStart},{th.MiRowStart})")
            t["tiles"] = time.monotonic() - t0
            t0 = time.monotonic()
            if pipe and stage == "lf":
                self.frame_pipe_finish(tag)
            elif stage == "lf":
                self.loop_filter(hdr0, tag)
            t["lf"] = time.monotonic() - t0
            t0 = time.monotonic()
            if dma is not None and window:
                base, read = dma
                self.flush_wait(tag)
                layout, total = dma_layout(hdr0, base, visible)
                planes = unpack_dma(hdr0, read(0, total), layout) if want_planes else None
                self.stats["pixels"] += sum(W * H for _, _, W, H in layout)
            elif dma is not None:
                base, read = dma
                layout, total = self.dma_picture(hdr0, base, tag, visible)
                planes = unpack_dma(hdr0, read(0, total), layout) if want_planes else None
                self.stats["pixels"] += sum(W * H for _, _, W, H in layout)
            else:
                planes = [self.read_plane(hdr0, p, visible, packed) for p in range(hdr0.NumPlanes)]
            t["read"] = time.monotonic() - t0
            self.stats["frames"] += 1
            yield fi, hdr0, planes, t

    @staticmethod
    def upload_picture(write, off, base, hdr, planes, budget, tag):
        """A model-decoded picture into the pool buffer at base (write(off, bytes) = the DMA region's writer, off = the
        buffer's offset in it), in the flush layout the reference reader expects: dav1d's sample format (bytes for
        8-bit, 16-bit little-endian otherwise), rows on 8-byte strides, planes one after the other. Returns the
        slot-table geometry [(plane base, stride)]."""
        bps = 1 if hdr.BitDepth == 8 else 2
        dims = [(len(planes[p][0]), len(planes[p])) for p in range(hdr.NumPlanes)]
        if sum(((w * bps + 7) & ~7) * h for w, h in dims) > budget:
            # too big for a pool buffer: the picture stays on the host (the caller marks the slot mem=None)
            sys.stderr.write(f"{tag}: a model-decoded picture (planes {dims}, bd{hdr.BitDepth}) does not fit a {budget}-byte frame buffer; kept on the host\n")
            return None
        mem, pos = [], 0
        for p in range(hdr.NumPlanes):
            w, h = dims[p]
            stride = (w * bps + 7) & ~7
            buf = bytearray(stride * h)
            for y, row in enumerate(planes[p]):
                if bps == 1:
                    buf[y * stride:y * stride + w] = bytes(row)
                else:
                    a = array.array("H", row)
                    buf[y * stride:y * stride + 2 * w] = a.tobytes()
            write(off + pos, bytes(buf))
            mem.append((base + pos, stride))
            pos += stride * h
        return mem

    def decode_frames_inter(self, dec, dma, name="stream", pool=8, want_planes=True, model_planes=False):
        """Inter streams on a RING build (docs/picture-memory.md step A; tb/test_dec_host_inter.py is the simulation
        twin): every frame pipelined under the sequencer with CDEF / super-res / LR, flushed into one of `pool`
        frame buffers carved from dma = (base, size, read(offset, n)); the references of later frames are those
        flushed pictures (the slot table points at them). The frame sequence (inherited CDFs, segmentation map,
        motion field) comes from the model, which decodes every frame alongside the RTL (tools/xcheck_frame.py's
        pieces: seconds a frame; the hardware save / project engines are the throughput phase's). Yields
        (fi, hdr, planes, model planes or None, timings); planes is the flushed picture (None with want_planes=False)."""
        import recon_model as rm
        import seq_model as sm
        import lf_model as lfm
        import cdef_model as cdm
        import lr_model as lrm
        import superres_model as srm
        import xcheck_frame as xf
        base, size, read = dma[:3]
        write = dma[3] if len(dma) > 3 else None            # (offset, bytes): needed to hand the card a model-decoded reference
        stride = (size // pool) & ~0xFFF
        seq = sm.SequenceState()
        store = {}                                       # slot -> dict(planes (model), upscaled_width, frame_height, mem=[(base, stride)...])
        hdr_iter = iter(dec.frame_headers)
        for fi, tile_idx in enumerate(dec.frames):
            tag = f"{name} frame {fi}"
            ph = None
            for ph in hdr_iter:
                if ph.show_existing_frame:
                    seq.show_existing(ph)
                    if ph.frame_type == 0 and ph.frame_to_show_map_idx in store:
                        for i in range(8):
                            store[i] = store[ph.frame_to_show_map_idx]
                    continue
                break
            if ph is None or ph.show_existing_frame:
                raise HostError(f"{tag}: no parsed frame header")
            ctx = seq.start_frame(ph)
            hdr0 = dec.tiles[tile_idx[0]][0]
            n_tile_cols = len({dec.tiles[ti][0].MiColStart for ti in tile_idx})
            # A frame the image cannot take (sb128 / non-4:2:0 / several tile columns / intrabc / too large) is decoded by
            # the model alone and its picture uploaded into a pool buffer, so the frames after it still run on the card
            # with the right references (the twin, tb/test_dec_host_inter.py, does the same: model_only).
            model_only = not self.window_ok(hdr0, n_tile_cols, **{k: v for k, v in getattr(self, "geom", {}).items() if k in ("fbx", "fby", "sb128", "te")})
            why = "outside the window build's limits" if model_only else None
            if not model_only and not ph.FrameIsIntra:
                # a reference the card never got (a picture too big for a pool buffer, below) keeps the frame off the card too
                missing = [ph.ref_frame_idx[i] for i in range(7) if ph.ref_frame_idx[i] not in store or not store[ph.ref_frame_idx[i]].get("mem")]
                if missing:
                    model_only, why = True, f"reference slot {missing[0]} holds no card picture"
            if model_only and write is None:
                raise HostError(f"{tag}: {why} (sb128 / non-4:2:0 / tile columns / intrabc / wider than 1 << FBX or taller than 1 << FBY)")
            t = {}
            t0 = time.monotonic()
            # the flush target: a frame buffer no live reference slot holds
            busy = {e["mem"][0][0] for e in store.values() if e.get("mem")}
            target = next((base + k * stride for k in range(pool) if base + k * stride not in busy), None)
            if target is None:
                raise HostError(f"{tag}: every one of the {pool} frame buffers holds a live reference")
            dseq, layout, total = dma_regs(hdr0, target, True)
            self.last_target = target                    # where this frame's picture lands (tools/kv260/decode.py --show)
            # A model-decoded picture bigger than a pool buffer (an 8K-class frame) stays on the host: no card frame can
            # take a picture that size as a reference anyway, and a frame that still names it goes to the model as well.
            host_only = model_only and total > stride
            if total > stride and not model_only:
                raise HostError(f"{tag}: a {total}-byte picture does not fit a {stride}-byte frame buffer (pool {pool} in {size} bytes)")
            if not model_only:
              self.reset()
            if not model_only and not ph.FrameIsIntra:
                self.write_seq(seg_map_seq(ctx.prev_segment_ids, hdr0.MiRows, hdr0.MiCols))
                self.write_seq(motion_field_seq(ctx.motion_field, hdr0.MiRows, hdr0.MiCols))
                self.write_seq(ref_seq(ph, {s_: dict(planes=e["mem"], upscaled_width=e["upscaled_width"], frame_height=e["frame_height"])
                                           for s_, e in store.items() if e.get("mem")}, hdr0.BitDepth > 8))
            if not model_only:
                self.write_seq([(a, (v | DMA_CFG_AUTO) if a == R_DMA_CFG else v) for a, v in dseq])
                self.frame_pipe_start(hdr0, n_tile_cols)
            t["setup"] = time.monotonic() - t0
            t0 = time.monotonic()
            # the model decodes each tile first (the RTL needs the same CDF rows; the sequence state needs the tile's summary)
            frame = {"refs": store}
            rows = M.rows_of(ctx.cdfs) if ctx.cdfs is not None else None
            summaries = []
            state = xf.FrameState(hdr0)
            # the tiles of a tile row go to the engines together (run_tile_row; one tile = run_tile)
            pending, pending_dec = [], None
            for k, ti in enumerate(tile_idx):
                th, data = dec.tiles[ti]
                mdec = rm.FrameRecon(th, data, frame, ctx=ctx)
                mdec.decode_tile()
                state.add(mdec)
                summaries.append(sm.TileSummary(mdec, keep_cdf=(k == ph.context_update_tile_id)))
                t["model"] = t.get("model", 0) + time.monotonic() - t0
                t0 = time.monotonic()
                if not model_only:
                    if pending and th.MiRowStart != pending[0][0].MiRowStart:
                        self.run_tile_row(pending, f"{tag} tile row {pending[0][0].MiRowStart}", dec=pending_dec, rows=rows)
                        pending, pending_dec = [], None
                    pending.append((th, data))
                    if pending_dec is None:
                        pending_dec = mdec
                    if k == len(tile_idx) - 1:
                        self.run_tile_row(pending, f"{tag} tile row {pending[0][0].MiRowStart}", dec=pending_dec, rows=rows)
                        pending, pending_dec = [], None
                t["tiles"] = t.get("tiles", 0) + time.monotonic() - t0
                t0 = time.monotonic()
                del mdec
            if not model_only:
                self.frame_pipe_finish(tag)
                self.flush_wait(tag)
            t["filters"] = time.monotonic() - t0
            t0 = time.monotonic()
            planes = unpack_dma(hdr0, read(target - base, total), layout) if (want_planes and not model_only) else None
            if not model_only:
                self.stats["pixels"] += sum(W * H for _, _, W, H in layout)
            t["read"] = time.monotonic() - t0
            t0 = time.monotonic()
            # the model's final picture: the reference store (and, if asked, the comparison)
            mp = frame["planes"]
            lfm.LoopFilter(hdr0, state, mp).apply()
            deblocked = mp
            if cdef_on(hdr0):
                mp = cdm.Cdef(hdr0, state, mp).apply()
            if hdr0.use_superres:
                deblocked = srm.upscale(hdr0, deblocked)
                mp = srm.upscale(hdr0, mp) if mp is not deblocked else deblocked
            if any(tt != 0 for tt in hdr0.FrameRestorationType[:hdr0.NumPlanes]):
                mp = lrm.LoopRestoration(hdr0, state, deblocked, mp if cdef_on(hdr0) else [[r[:] for r in p_] for p_ in mp]).apply()
            seq.end_frame(ph, summaries, frame_cdfs=ctx.cdfs)
            if model_only:
                mem_at = None if host_only else self.upload_picture(write, target - base, target, hdr0, mp, stride, tag)
                t["upload"] = time.monotonic() - t0
                t["model_only"] = why + (" (picture bigger than a pool buffer, kept on the host)" if mem_at is None else "")
                self.stats["model_only"] = self.stats.get("model_only", 0) + 1
            else:
                mem_at = [(b, st) for b, st, _, _ in layout]
            entry = dict(planes=mp, upscaled_width=ph.UpscaledWidth, frame_height=ph.FrameHeight, mem=mem_at)
            for i in range(8):
                if (ph.refresh_frame_flags >> i) & 1:
                    store[i] = entry
            t["model_filters"] = time.monotonic() - t0
            if not model_only:
                self.stats["frames"] += 1
            yield fi, hdr0, planes, (mp if (model_planes or model_only) else None), t
