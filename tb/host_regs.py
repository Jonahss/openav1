"""The decoder's host-side protocol, with no simulator dependency: the register map of rtl/dec_host.sv, the
header structs packed the way syn_pkg declares them (hdr_t, rec_hdr_t, lf_hdr_t), and HostDecoder, the sequence
that drives a frame through the register file (headers, CDF default rows, tile bytes, stage starts, done polls,
picture read-back). tb/test_dec_host.py uses the packing from here under cocotb; tools/kv260/decode.py runs
HostDecoder on the KV260 over /dev/mem. One definition of the protocol, so the board does what the test does."""
import sys
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
LF_FIELDS = [  # syn_pkg::lf_hdr_t order (MSB first)
    ("frame_width", PXW), ("frame_height", PXW), ("level", 24), ("sharpness", 3), ("delta_enabled", 1),
    ("ref_delta_intra", 7), ("delta_lf_multi", 1), ("seg_lf_en", 32), ("seg_lf_data", 224)]


def pack(fields, vals):
    v = 0
    for name, w in fields:
        x = vals[name] & ((1 << w) - 1)
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
             lr_size=sum((th.LoopRestorationSize[p].bit_length() - 1 - 6) << (2 * p) for p in range(3)),
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


def lf_vals(th):
    return dict(frame_width=th.FrameWidth, frame_height=th.FrameHeight,
                level=sum((th.loop_filter_level[i] & 63) << (6 * i) for i in range(4)),
                sharpness=th.loop_filter_sharpness, delta_enabled=th.loop_filter_delta_enabled,
                ref_delta_intra=th.loop_filter_ref_deltas[0], delta_lf_multi=th.delta_lf_multi,
                seg_lf_en=sum((th.FeatureEnabled[s][1 + i] & 1) << (8 * i + s) for i in range(4) for s in range(8)),
                seg_lf_data=sum((th.FeatureData[s][1 + i] & 0x7F) << (7 * (8 * i + s)) for i in range(4) for s in range(8)))


# ---------------------------------------------------------------- register map (rtl/dec_host.sv)
R_ID, R_CFG, R_CTRL, R_START, R_STATUS, R_PERF_SYN, R_PERF_REC = 0x000, 0x004, 0x008, 0x00C, 0x010, 0x014, 0x018
R_DATA1, R_DATA4, R_HADDR, R_HDATA, R_DEF_ADDR = 0x01C, 0x020, 0x024, 0x028, 0x02C
R_HDATA2, R_HDATA4 = 0x030, 0x034
R_DEF, R_HDR, R_RH, R_LH, R_CH = 0x040, 0x080, 0x0C0, 0x100, 0x140
ID_VALUE = 0xA71D0001
ST_TILE_BUSY, ST_TILE_DONE, ST_UNSUP, ST_LF_DONE, ST_FIFO_BAD, ST_FIFO_EMPTY = 1 << 0, 1 << 1, 1 << 2, 1 << 6, 1 << 14, 1 << 15
START_TILE, START_LF = 1, 4
CTRL_RST, CTRL_EOS = 1, 1 << 8
HDR_WORDS, RH_WORDS, LH_WORDS = 8, 7, 11     # words of each header window actually written


def words(v, n):
    return [(v >> (32 * i)) & 0xFFFFFFFF for i in range(n)]


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
    def __init__(self, regs, poll_timeout=30.0, log=None):
        self.r = regs
        self.poll_timeout = poll_timeout
        self.log = log or (lambda s: None)
        self.stats = dict(frames=0, tiles=0, bytes=0, pixels=0, writes=0, reads=0)

    # -- primitives
    def wr(self, a, v):
        self.r.wr(a, v)
        self.stats["writes"] += 1

    def rd(self, a):
        self.stats["reads"] += 1
        return self.r.rd(a)

    def write_wide(self, base, v, nwords):
        for i, w in enumerate(words(v, nwords)):
            self.wr(base + 4 * i, w)

    def poll(self, a, mask, value, what):
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
        return dict(fbx=cfg >> 16 & 0xFF, fby=cfg >> 8 & 0xFF, lanes=cfg >> 4 & 0xF, uram=cfg & 1)

    def reset(self):
        self.wr(R_CTRL, CTRL_RST)
        self.rd(R_CTRL)                 # a read behind the posted write: the reset is held a few cycles at least
        self.wr(R_CTRL, 0)

    # -- stages
    def run_tile(self, th, data, tag, dec=None):
        """Headers, CDF defaults, start, the tile bytes, end of stream; returns when the tile is decoded."""
        self.write_wide(R_HDR, pack(HDR_FIELDS, hdr_vals(th, dec, 0)), HDR_WORDS)
        self.write_wide(R_RH, pack(REC_FIELDS, rec_vals(th)), RH_WORDS)
        for i, row in enumerate(M.default_rows(th.base_q_idx)):
            self.write_wide(R_DEF, row, 8)
            self.wr(R_DEF_ADDR, i)
        self.wr(R_START, START_TILE)
        pos = 0
        n_words = (len(data) + 3) // 4
        t0 = time.monotonic()
        while pos < len(data):
            free = self.rd(R_STATUS) >> 16
            if free == 0:
                if time.monotonic() - t0 > self.poll_timeout:
                    raise HostError(f"{tag}: the tile FIFO never freed a word")
                continue
            for _ in range(min(free, n_words)):
                chunk = data[pos:pos + 4]
                if len(chunk) == 4:
                    self.wr(R_DATA4, int.from_bytes(chunk, "little"))
                else:
                    for b in chunk:
                        self.wr(R_DATA1, b)
                pos += 4
                n_words -= 1
                if pos >= len(data):
                    break
        self.wr(R_CTRL, self.rd(R_CTRL) | CTRL_EOS)
        st = self.poll(R_STATUS, ST_TILE_DONE | ST_UNSUP, ST_TILE_DONE, f"{tag}: tile done")
        if st & ST_UNSUP:
            raise HostError(f"{tag}: the decoder flagged unsupported syntax")
        if st & ST_FIFO_BAD:
            raise HostError(f"{tag}: FIFO overflow or a split register write (STATUS bit 14)")
        self.stats["tiles"] += 1
        self.stats["bytes"] += len(data)

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
        rows = [self.read_row(hdr, plane, y, W, packed) for y in range(H)]
        self.stats["pixels"] += W * H
        return rows

    # -- a whole stream
    def check_core_build(self, hdr, tag):
        if hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:
            raise HostError(f"{tag}: CDEF is not in the core build (encode with --enable-cdef=0)")
        if any(t != 0 for t in hdr.FrameRestorationType[:hdr.NumPlanes]):
            raise HostError(f"{tag}: loop restoration is not in the core build (encode with --enable-restoration=0)")

    def decode_frames(self, dec, stage="lf", visible=True, name="stream", packed=True):
        """dec: an obu_parser.Decoder that has been fed the stream. Yields (frame index, header, planes, timings)
        per coded frame, planes as lists of rows of samples."""
        for fi, tile_idx in enumerate(dec.frames):
            tag = f"{name} frame {fi}"
            hdr0 = dec.tiles[tile_idx[0]][0]
            self.check_core_build(hdr0, tag)
            t = {}
            t0 = time.monotonic()
            self.reset()
            for ti in tile_idx:
                th, data = dec.tiles[ti]
                self.run_tile(th, data, f"{tag} tile ({th.MiColStart},{th.MiRowStart})")
            t["tiles"] = time.monotonic() - t0
            t0 = time.monotonic()
            if stage == "lf":
                self.loop_filter(hdr0, tag)
            t["lf"] = time.monotonic() - t0
            t0 = time.monotonic()
            planes = [self.read_plane(hdr0, p, visible, packed) for p in range(hdr0.NumPlanes)]
            t["read"] = time.monotonic() - t0
            self.stats["frames"] += 1
            yield fi, hdr0, planes, t
