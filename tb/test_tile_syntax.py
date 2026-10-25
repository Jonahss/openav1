"""tile_syntax (the whole intra tile syntax decoder in RTL) vs tb/tile_model.py.

Streams come from tools/gen_stream.py (random syntax, no palette / no loop restoration yet), are parsed by
tb/obu_parser.py, decoded by the model (recording every block and transform block), and replayed through
the RTL tile by tile. Compared: every block record (position, size, skip, segment, modes, angles, CfL
alphas, filter-intra, tx size, CurrentQIndex, DeltaLF) and every transform block (plane, position, tx
size, tx type, eob, all Quant coefficients).

Env: TS_SEEDS (comma list or count, default "1,2,3"), TS_W/TS_H (default 128x96), TS_FMT (default random),
TS_IVF=a.ivf,b.obu (real streams), TS_FRAMES=0,3 (with TS_IVF: only these decoded-frame indices), TS_DEBUG=1.
"""
import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import av1_tables as T          # noqa: E402
import cdf_map as M             # noqa: E402
import gen_stream as g          # noqa: E402
import obu_parser as op         # noqa: E402
import seq_model as sm          # noqa: E402
import tile_model as tm         # noqa: E402

ACK_DELAY = int(os.environ.get("TS_ACK_DELAY", "0"))
DBG_TX = int(os.environ["TS_DBG_TX"]) if os.environ.get("TS_DBG_TX") else None
# coordinate widths: must match syn_pkg::MIW / SBW / PXW (4x4-unit, 64x64-superblock and pixel positions / counts)
from host_regs import MIW, SBW, PXW, HDR_FIELDS, IHDR_FIELDS, pack, unpack, hdr_vals, ihdr_vals   # noqa: E402,F401  (cocotb-free: the board driver shares them)
BLK_FIELDS = [
    ("r", MIW), ("c", MIW), ("bsize", 5), ("skip", 1), ("seg", 3), ("lossless", 1), ("has_chroma", 1), ("ymode", 5), ("uvmode", 4),
    ("angle_y", 3), ("angle_uv", 3), ("cfl_u", 6), ("cfl_v", 6), ("use_fi", 1), ("fi_mode", 3), ("txsz", 5), ("qidx", 8),
    ("delta_lf", 28), ("cdef_valid", 1), ("cdef_idx", 3), ("cdef_units", 4),
    ("pal_y", 4), ("pal_uv", 4), ("col_y", 96), ("col_u", 96), ("col_v", 96),
    ("is_inter", 1), ("intrabc", 1), ("mv_row", 18), ("mv_col", 18),
    ("skip_mode", 1), ("ref0", 4), ("ref1", 4), ("mv1_row", 18), ("mv1_col", 18), ("motion_mode", 2), ("compound_type", 3),
    ("interintra", 1), ("interintra_mode", 2), ("wedge_interintra", 1), ("wedge_index", 4), ("wedge_sign", 1), ("mask_type", 1),
    ("comp_group_idx", 1), ("compound_idx", 1), ("interp0", 2), ("interp1", 2), ("cmv_intra", 1), ("cmv", 304), ("obmc", 408), ("ls", 224)]
TX_FIELDS = [("plane", 2), ("x", PXW), ("y", PXW), ("txsz", 5), ("txtype", 4), ("eob", 11), ("skip", 1), ("lossless", 1), ("slot", 3)]
LR_FIELDS = [("plane", 2), ("unit_row", SBW), ("unit_col", SBW), ("lr_type", 2), ("wiener", 42), ("sgr_set", 4), ("xqd", 16)]



def pack_cmv(cmv):
    """blk_rec_t.cmv from the model's chroma_mv_units: unit k at [76k +: 76]."""
    v = 0
    if cmv and not cmv["intra"]:
        for k, u in enumerate(cmv["units"]):
            rf0, rf1 = u["ref"][0], (u["ref"][1] if u["ref"][1] >= 0 else 15)
            mv0 = u["mvs"][0]
            mv1 = u["mvs"][1] if len(u["mvs"]) > 1 else (0, 0)
            w = rf0 | (rf1 << 4) | ((mv0[0] & 0xFFFF) << 8) | ((mv0[1] & 0xFFFF) << 24) | ((mv1[0] & 0xFFFF) << 40) \
                | ((mv1[1] & 0xFFFF) << 56) | (u["interp"][0] << 72) | (u["interp"][1] << 74)
            v |= w << (76 * k)
    return v


def pack_ls(ls):
    """blk_rec_t.ls: the seven least-squares sums, 32-bit two's complement, A00 lowest."""
    if not ls:
        return 0
    v = 0
    for i, a in enumerate(ls):
        assert -(1 << 31) <= a < (1 << 31), f"ls sum {i} = {a} outside 32 bits"
        v |= (a & 0xFFFFFFFF) << (32 * i)
    return v


def pack_obmc(ob):
    """blk_rec_t.obmc from the model's obmc_neighbours: above entries 0..3, left 4..7, entry k at [51k +: 51]."""
    v = 0
    if ob:
        for base, lst in ((0, ob["above"]), (4, ob["left"])):
            for k, e in enumerate(lst):
                w = 1 | (e["off"] << 1) | (e["step4"] << 6) | (e["ref"] << 11) | ((e["mv"][0] & 0xFFFF) << 15) \
                    | ((e["mv"][1] & 0xFFFF) << 31) | (e["interp"][0] << 47) | (e["interp"][1] << 49)
                v |= w << (51 * (base + k))
    return v


def to_signed(v, bits):
    return v - (1 << bits) if v >> (bits - 1) else v


class RecDecoder(tm.TileDecoder):
    """Records what the RTL emits: per block and per transform block."""

    def __init__(self, hdr, data, frame=None):
        super().__init__(hdr, data, None, frame=frame)
        self.tx_recs = []
        self.lr_recs = []
        self.symlog = []
        self.idmap = {}
        for name in M.TABLES:
            e = M.MAP[name]

            def walk(v, depth, idx):
                if depth == len(e["dims"]):
                    self.idmap[id(v)] = (M.addr(name, *idx), e["N"])
                else:
                    for i, sub in enumerate(v):
                        walk(sub, depth + 1, idx + [i])
            walk(self.cdf[name], 0, [])

    def sym(self, cdf, name):
        pre = list(cdf)
        v = super().sym(cdf, name)
        self.symlog.append((name, self.idmap.get(id(cdf), ("?", len(cdf) - 1)), v, pre, self.dec.range))
        return v

    def L(self, n, name):
        v = super().L(n, name)
        self.symlog.append((name, "L%d" % n, v))
        return v

    def NS(self, n, name):
        # same bit consumption as SymbolDecoder.read_ns, through L() so every bool lands in the symbol log
        w = tm.floor_log2(n) + 1
        m = (1 << w) - n
        v = self.L(w - 1, name)
        if v < m:
            return v
        return (v << 1) - m + self.L(1, name)

    def NS(self, n, name):
        # read_ns through L so the literal bits land in the symbol log (same bit sequence as the spec)
        w = n.bit_length()
        m = (1 << w) - n
        v = self.L(w - 1, name)
        if v < m:
            return v
        return (v << 1) - m + self.L(1, name)

    def predict_block(self, plane, startX, startY, txSz, x, y, subX, subY, sbMiRow, sbMiCol, stepX, stepY):
        self.tx_recs.append(dict(plane=plane, x=startX, y=startY, txsz=txSz, skip=self.skip, txtype=0, eob=0, quant=[],
                                 lossless=self.Lossless))

    def coeffs(self, plane, startX, startY, txSz):
        eob = super().coeffs(plane, startX, startY, txSz)
        segEob = 512 if txSz in (tm.TX_16X64, tm.TX_64X16) else min(1024, T.Tx_Width[txSz] * T.Tx_Height[txSz])
        rec = self.tx_recs[-1]
        rec["txtype"] = self.PlaneTxType if eob > 0 else 0
        rec["eob"] = eob
        rec["quant"] = list(self.Quant[:segEob]) if eob > 0 else [0] * segEob
        return eob

    def read_lr_unit(self, plane, unitRow, unitCol):
        super().read_lr_unit(plane, unitRow, unitCol)
        key = (plane, unitRow, unitCol)
        t = self.LrType[key]
        self.lr_recs.append(dict(plane=plane, unit_row=unitRow, unit_col=unitCol, lr_type=t,
                                 wiener=self.LrWiener.get(key, [[0, 0, 0], [0, 0, 0]]) if t == tm.RESTORE_WIENER else [[0, 0, 0], [0, 0, 0]],
                                 sgr_set=self.LrSgrSet.get(key, 0) if t == tm.RESTORE_SGRPROJ else 0,
                                 xqd=self.LrSgrXqd.get(key, [0, 0]) if t == tm.RESTORE_SGRPROJ else [0, 0]))

    def decode_block(self, r, c, subSize):
        super().decode_block(r, c, subSize)
        self.blocks[-1]["qidx"] = self.CurrentQIndex
        self.blocks[-1]["dlf"] = list(self.DeltaLF)
        self.blocks[-1]["lossless"] = self.Lossless
        self.blocks[-1]["has_chroma"] = self.HasChroma
        self.blocks[-1]["fim"] = self.filter_intra_mode
        b = self.blocks[-1]
        b["col_y"] = list(self.palette_colors_y[:self.PaletteSizeY])
        b["col_u"] = list(self.palette_colors_u[:self.PaletteSizeUV])
        b["col_v"] = list(self.palette_colors_v[:self.PaletteSizeUV])
        b["cmap_y"], b["cmap_uv"] = self.ColorMapY, self.ColorMapUV
        h = self.h
        b["os"] = (min(T.Block_Width[subSize], (h.MiCols - c) * 4), min(T.Block_Height[subSize], (h.MiRows - r) * 4))




_feed_task = [None]


def start_feed(dut, coro):
    """Start a byte feeder for a tile, cancelling the previous tile's feeder first: a tile may carry trailing
    bytes the decoder never fetches (Argon streams do), which would leave the old feeder blocked and its bytes
    interleaving with the next tile's."""
    if _feed_task[0] is not None and not _feed_task[0].done():
        _feed_task[0].cancel()
    dut.in_valid.value = 0
    _feed_task[0] = cocotb.start_soon(coro)
    return _feed_task[0]

async def monitor_syms(dut, log):
    """Every accepted msac request: (row addr of the active sequencer, n, kind) and its symbol."""
    # requests may be accepted in the same cycle the previous response arrives: keep a FIFO, respond first
    pend = []
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.resp_valid.value) and pend:
            log.append(pend.pop(0) + (int(dut.resp_sym.value), int(dut.resp_rng.value)))
        if int(dut.req_valid.value) and int(dut.req_ready.value):
            k = int(dut.sel_k.value)
            addr = int(dut.u_sq_k.l_addr.value) if k else int(dut.u_sq_c.l_addr.value)
            pend.append(("k" if k else "c", addr, int(dut.req_n.value), int(dut.req_kind.value)))


async def monitor_nb(dut, log):
    """Every answered neighbour query: position/size and what blk_ctx returned."""
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.nb_valid.value):
            c = dut.u_ctx
            log.append(dict(r=int(c.q_r.value), c=int(c.q_c.value), bs=int(c.q_bs.value), au=int(dut.avail_u.value), al=int(dut.avail_l.value),
                            a_ymode=int(dut.a_ymode.value), l_ymode=int(dut.l_ymode.value), a_skip=int(dut.a_skip.value), l_skip=int(dut.l_skip.value),
                            a_misize=int(dut.a_misize.value), l_misize=int(dut.l_misize.value), a_txsz=int(dut.a_txsz.value), l_txsz=int(dut.l_txsz.value),
                            seg=(int(dut.seg_ul.value), int(dut.seg_u.value), int(dut.seg_l.value)),
                            lword=hex(int(c.l_word.value)), aword=hex(int(c.a_word.value))))


FEED_GAP = [int(x) for x in os.environ.get("TS_FEED_GAP", "0,0").split(",")]   # every N bytes, no bytes for M cycles (starvation)


async def feed_bytes(dut, data):
    dut.in_eos.value = 0
    for i, b in enumerate(data):
        if FEED_GAP[0] and i and i % FEED_GAP[0] == 0:
            dut.in_valid.value = 0
            await ClockCycles(dut.clk, FEED_GAP[1])
            await Timer(1, "ns")
        dut.in_data.value = b
        dut.in_valid.value = 1
        while True:
            await ReadOnly()
            ok = int(dut.in_ready.value) == 1
            await RisingEdge(dut.clk)
            if ok:
                break
        await Timer(1, "ns")
    dut.in_valid.value = 0
    dut.in_eos.value = 1


def gen(seed, w, h, fmt):
    rng = random.Random(seed)
    screen_env = os.environ.get("TS_SCREEN", "rand")      # 1 / 0 / rand
    args = dict(fmt=fmt, bd=rng.choice([8, 10, 12]), w=w, h=h, sb128=rng.random() < 0.3, tiles=rng.random() < 0.3,
                screen=(screen_env == "1") or (screen_env == "rand" and rng.random() < 0.5),
                lossless=rng.random() < 0.15, nolr=bool(os.environ.get("TS_NOLR")), frames=1)
    if fmt == "422":
        args["bd"] = rng.choice([10, 12])
    data, info = g.generate(seed, args)
    d = op.Decoder()
    d.feed_ivf(data)
    return d, args


@cocotb.test()
async def tile_vs_model(dut):
    seeds_env = os.environ.get("TS_SEEDS", "1,2,3")
    seeds = [int(x) for x in seeds_env.split(",") if x] if "," in seeds_env else list(range(1, int(seeds_env) + 1))
    W = int(os.environ.get("TS_W", "128"))
    H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    if hasattr(dut, "clk2x"):                                # dec_top with the double-pumped frame memory (FRAME_URAM)
        cocotb.start_soon(Clock(dut.clk2x, 5, unit="ns").start())
        async def phase_gen():                               # 1 at the mid-cycle clk2x edge, 0 at the clk-aligned one
            while True:
                await RisingEdge(dut.clk)
                await Timer(2.5, "ns")
                dut.phase.value = 1
                await Timer(5, "ns")
                dut.phase.value = 0
        dut.phase.value = 0
        cocotb.start_soon(phase_gen())
    for s in ("in_valid", "in_eos", "def_we", "rb_en", "rb_addr", "tile_start", "tx_ack", "q_addr", "q_slot_w", "q_slot_r", "hdr", "ihdr", "blk_ack", "pm_plane", "pm_x", "pm_y",
              "mf_we", "mf_wref", "mf_wy8", "mf_wx8", "mf_wvalid", "mf_wrow", "mf_wcol", "sp_we", "sp_wrow", "sp_wwc", "sp_wdata"):
        if hasattr(dut, s):
            getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(tiles=0, blocks=0, txblocks=0, coefs=0, lrunits=0, palblocks=0, palpix=0)
    symlog = []
    debug = bool(os.environ.get("TS_DEBUG"))
    nblog = []
    if debug:
        cocotb.start_soon(monitor_syms(dut, symlog))
        cocotb.start_soon(monitor_nb(dut, nblog))

    def dump(dec):
        if not debug:
            return
        # normalise both logs to (kind, addr, n, value) and show the first divergence with context
        mnorm = []
        mcdf = {}
        for i_, entry in enumerate(dec.symlog):
            pass
        for name, where, v, *rest in dec.symlog:
            if isinstance(where, str) and where.startswith("L"):
                n = int(where[1:])
                for i in range(n - 1, -1, -1):
                    mnorm.append(("B", 0, 1, (v >> i) & 1, name))
            elif where[0] == "?":
                mnorm.append(("F", 0, 1, v, name))
            else:
                mnorm.append(("S", where[0], where[1] - 1, v, name))
                if rest and where[0] not in mcdf:
                    mcdf[where[0]] = rest[0]          # the model's CDF row content at its first use
        rnorm = []
        rrng = []
        for master, addr, n, kind, sym, rng in symlog:
            rnorm.append(("B" if kind == 2 else "F" if kind == 1 else "S", addr if kind == 0 else 0, n if kind == 0 else 1, sym, master))
            rrng.append(rng)
        mrng = []
        for name, where, v, *rest in dec.symlog:
            if isinstance(where, str) and where.startswith("L"):
                mrng.extend([None] * int(where[1:]))
            else:
                mrng.append(rest[1] if len(rest) > 1 else None)
        first = None
        for i in range(min(len(mnorm), len(rnorm))):
            if mnorm[i][:4] != rnorm[i][:4]:
                first = i
                break
        if first is None:
            first = min(len(mnorm), len(rnorm))
        dut._log.info(f"first divergence at symbol {first} (model {len(mnorm)} symbols, rtl {len(rnorm)})")
        for x in nblog[-3:]:
            dut._log.info(f"  NB {x}")
            r, c = x["r"], x["c"]
            S = dec.SegmentIds
            nb = dict(ul=S[r - 1][c - 1] if r > 0 and c > 0 else None, u=S[r - 1][c] if r > 0 else None, l=S[r][c - 1] if c > 0 else None)
            dut._log.info(f"     model SegmentIds at ({r},{c}): {nb}; MiSizes above/left: "
                          f"{dec.MiSizes[r - 1][c] if r > 0 else None}/{dec.MiSizes[r][c - 1] if c > 0 else None}")
        dut._log.info(f"  model blocks so far: {[(b['r'], b['c'], b['size'], b['seg'], b['skip']) for b in dec.blocks[:24]]}")
        for b in dec.blocks[:8]:
            dut._log.info(f"  MODEL BLK r={b['r']} c={b['c']} size={b['size']} skip={b['skip']} ymode={b['ymode']} tx={b['tx']}")
        for i in range(max(0, first - 20), min(first + 8, max(len(mnorm), len(rnorm)))):
            m = mnorm[i] if i < len(mnorm) else None
            r = rnorm[i] if i < len(rnorm) else None
            mr = mrng[i] if i < len(mrng) else None
            rr = rrng[i] if i < len(rrng) else None
            dut._log.info(f"  {i:5d} {'!!' if (m is None or r is None or m[:4] != r[:4]) else '  '} model {m}  rtl {r}  range model/rtl {mr}/{rr}{' <-- RANGE DIFFERS' if (mr is not None and rr is not None and mr != rr) else ''}")
        # CDF rows used up to and including the divergence: the model's row as first used vs the RTL's defaults
        seen = set()
        for i in range(0, min(first + 1, len(mnorm))):
            if mnorm[i][0] != "S" or mnorm[i][1] in seen:
                continue
            seen.add(mnorm[i][1])
            addr = mnorm[i][1]
            n = mnorm[i][2] + 1
            rtl_def = M.unpack_row(int(dut.u_cdf.def_mem[addr].value), n)
            same = "same" if list(mcdf.get(addr, [])) == list(rtl_def) else "DIFFERENT"
            dut._log.info(f"  CDF row {addr} ({mnorm[i][4]}): {same}; model first-use {mcdf.get(addr)}; RTL def_mem {rtl_def}")
    # real streams (TS_IVF=a.ivf,b.obu): every tile of every frame, records compared like the generated ones
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    jobs = []
    if ivfs:
        for path in ivfs:
            d = op.Decoder()
            with open(path, "rb") as fh:
                raw = fh.read()
            if path.endswith(".obu"):
                d.feed_annexb(raw)
            else:
                if path.endswith(".obu"):
                    d.feed_annexb(raw)
                else:
                    d.feed_ivf(raw)
            jobs.append((os.path.basename(path), d, dict(bd=d.tiles[0][0].BitDepth)))
    else:
        for seed in seeds:
            fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
            d, args = gen(seed, W, H, fmt)
            jobs.append((f"seed {seed} {fmt}", d, args))
    for label, d, args in jobs:
        frames = list(getattr(d, "frames", None) or [list(range(len(d.tiles)))])
        frame_of = {ti: fi for fi, tl in enumerate(frames) for ti in tl}
        frames_sel = {int(x) for x in os.environ.get("TS_FRAMES", "").split(",") if x}   # decoded-frame indices to run (default all)
        # the frame sequence: CDFs, the segmentation map and the motion field a frame inherits (seq_model), as the host
        # will program them; frames with show_existing_frame only update the sequence state
        seq = sm.SequenceState()
        hdr_iter = iter(getattr(d, "frame_headers", []))
        for fi, tl in enumerate(frames):
            h = None
            for h in hdr_iter:
                if h.show_existing_frame:
                    seq.show_existing(h)
                    continue
                break
            if h is None or h.show_existing_frame:
                h = d.tiles[tl[0]][0]
            ctx = seq.start_frame(h)
            if frames_sel and fi not in frames_sel:
                decs = [RecDecoder(d.tiles[ti][0], d.tiles[ti][1], frame=ctx) for ti in tl]
                for dec in decs:
                    dec.decode_tile()
                seq.end_frame(h, [sm.TileSummary(dec, keep_cdf=(k == h.context_update_tile_id)) for k, dec in enumerate(decs)], frame_cdfs=ctx.cdfs)
                continue
            rows = M.rows_of(ctx.cdfs) if ctx.cdfs is not None else M.default_rows(h.base_q_idx)
            await load_frame_state(dut, h, ctx)
            decoders = []
            for k, ti in enumerate(tl):
                th, data = d.tiles[ti]
                tag0 = f"{label} bd{args['bd']} frame {fi} tile {ti} ({th.MiColStart},{th.MiRowStart})"
                dec = RecDecoder(th, data, frame=ctx)
                dec.decode_tile()
                hv = hdr_vals(th, dec, fi)
                dut.hdr.value = pack(HDR_FIELDS, hv)
                dut.ihdr.value = pack(IHDR_FIELDS, ihdr_vals(th))
                # the frame's starting CDFs (the defaults for base_q_idx, or the primary reference's saved ones)
                for i, row in enumerate(rows):
                    dut.def_we.value = 1
                    dut.def_addr.value = i
                    dut.def_data.value = row
                    await RisingEdge(dut.clk)
                dut.def_we.value = 0
                symlog.clear()
                nblog.clear()
                dut.tile_start.value = 1
                await RisingEdge(dut.clk)
                await Timer(1, "ns")
                dut.tile_start.value = 0
                feeder = start_feed(dut, feed_bytes(dut, data))
                symlog.clear()
                try:
                    await collect(dut, dec, tag0, stats)
                except AssertionError:
                    dump(dec)
                    raise
                await ClockCycles(dut.clk, 4)
                if not th.FrameIsIntra:
                    stats["inter_blocks"] = stats.get("inter_blocks", 0) + sum(1 for b in dec.blocks if b.get("inter"))
                    stats["inter_frames"] = stats.get("inter_frames", 0) + (1 if k == 0 else 0)
                if k == h.context_update_tile_id and os.environ.get("TS_RB", "1") == "1" and hasattr(dut, "rb_en"):
                    # the frame's saved CDFs: the working store after this tile, read back as the host will (rb port)
                    await check_cdf_readback(dut, dec, tag0, stats)
                decoders.append(sm.TileSummary(dec, keep_cdf=(k == h.context_update_tile_id)))
                del dec
            seq.end_frame(h, decoders, frame_cdfs=ctx.cdfs)
    dut._log.info(f"OK: {stats}")


async def check_cdf_readback(dut, dec, tag0, stats):
    """Every row of the working CDF store after a tile equals the model's adapted CDFs (counters included)."""
    want = M.rows_of(dec.cdf)
    bad = []
    for i in range(M.ROWS):
        dut.rb_en.value = 1
        dut.rb_addr.value = i
        await RisingEdge(dut.clk)
        await ReadOnly()
        v = int(dut.rb_data.value) & ((1 << M.ROW_BITS) - 1)
        if v != want[i]:
            bad.append(i)
        await Timer(1, "ns")
        dut.rb_en.value = 0
    if bad:
        name = next(n for n in M.TABLES if M.MAP[n]["base"] <= bad[0] < M.MAP[n]["base"] + M.MAP[n]["rows"])
        raise AssertionError(f"{tag0}: {len(bad)} CDF rows differ after the tile, first row {bad[0]} ({name}+{bad[0] - M.MAP[name]['base']})")
    stats["rb_rows"] = stats.get("rb_rows", 0) + M.ROWS


async def load_frame_state(dut, h, ctx):
    """The frame-side state of an inter frame into the DUT's band memories: the previous segmentation map (3 bits per
    4x4, 16 per word; zeros when there is none) and the motion field (one entry per 8x8 and reference; valid = 0 for
    the spec's MV_INVALID). Intra frames clear both (nothing reads them)."""
    if h.FrameIsIntra and not hasattr(dut, "sp_we"):
        return
    prev = ctx.prev_segment_ids
    rows, cols = h.MiRows, h.MiCols
    for r in range(rows):
        for wc in range((cols + 15) // 16):
            w = 0
            if prev is not None:
                for k in range(16):
                    c = wc * 16 + k
                    if c < cols:
                        w |= (prev[r][c] & 7) << (3 * k)
            dut.sp_we.value = 1
            dut.sp_wrow.value = r
            dut.sp_wwc.value = wc
            dut.sp_wdata.value = w
            await RisingEdge(dut.clk)
    dut.sp_we.value = 0
    mf = ctx.motion_field
    if not h.FrameIsIntra:
        for ref in range(1, 8):
            for y8 in range(rows >> 1):             # the field is (MiRows >> 1) x (MiCols >> 1); one more row or column
                for x8 in range(cols >> 1):         # aliases onto row / column 0 of an exactly-sized band (test1861 frame 7)
                    mv = mf[ref][y8][x8] if (mf is not None and y8 < len(mf[ref]) and x8 < len(mf[ref][y8])) else [tm.MV_INVALID, tm.MV_INVALID]
                    valid = mv[0] != tm.MV_INVALID
                    dut.mf_we.value = 1
                    dut.mf_wref.value = ref - 1
                    dut.mf_wy8.value = y8
                    dut.mf_wx8.value = x8
                    dut.mf_wvalid.value = int(valid)
                    dut.mf_wrow.value = (mv[0] & 0xFFFF) if valid else 0
                    dut.mf_wcol.value = (mv[1] & 0xFFFF) if valid else 0
                    await RisingEdge(dut.clk)
        dut.mf_we.value = 0


async def collect(dut, dec, tag0, stats):
            blk_i = 0
            tx_i = 0
            lr_i = 0
            cycles = 0
            idle = 0
            while True:
                await RisingEdge(dut.clk)
                await ReadOnly()
                cycles += 1
                idle += 1
                if idle > 100_000:
                    state = (f"tile st={int(dut.st.value)} blk st={int(dut.u_blk.st.value)} coef st={int(dut.u_coef.st.value)} "
                             f"ctx q/w/t/r={int(dut.u_ctx.qst.value)}/{int(dut.u_ctx.wst.value)}/{int(dut.u_ctx.tst.value)}/{int(dut.u_ctx.rstt.value)} "
                             f"msac st={int(dut.u_msac.state.value)} wcnt={int(dut.u_msac.wcnt.value)} eos={int(dut.u_msac.eos.value)} "
                             f"k_busy={int(dut.k_busy.value)} c_busy={int(dut.c_busy.value)} sel_k={int(dut.sel_k.value)} tx_done={int(dut.tx_done.value)} "
                             f"sp={int(dut.sp.value)} blk_active={int(dut.blk_active.value)}")
                    raise AssertionError(f"{tag0}: hang after {blk_i} blocks / {tx_i} tx blocks (model: {len(dec.blocks)} / {len(dec.tx_recs)}); {state}")
                if int(dut.unsupported.value):
                    raise AssertionError(f"{tag0}: RTL flagged unsupported syntax")
                done = int(dut.tile_done.value)
                if int(dut.pal_hold.value):
                    # palette block held: read the onscreen colour index map(s) through pm_*, compare, release
                    idle = 0
                    got = unpack(BLK_FIELDS, int(dut.blk_rec.value))
                    assert blk_i < len(dec.blocks), f"{tag0}: RTL holds a palette block beyond the model's last block"
                    m = dec.blocks[blk_i]
                    await Timer(1, "ns")
                    for plane, size_key, cmap_key in ((0, "pal_y", "cmap_y"), (1, "pal_uv", "cmap_uv")):
                        if got[size_key] == 0:
                            continue
                        cm = m[cmap_key]
                        assert cm is not None, f"{tag0} block {blk_i}: RTL has a {cmap_key} but the model has none"
                        osw, osh = m["os"]
                        if plane:
                            osw >>= dec.h.subsampling_x
                            osh >>= dec.h.subsampling_y
                            if (T.Block_Width[m["size"]] >> dec.h.subsampling_x) < 4:
                                osw += 2
                            if (T.Block_Height[m["size"]] >> dec.h.subsampling_y) < 4:
                                osh += 2
                        bad = []
                        for y in range(osh):
                            for x in range(osw):
                                dut.pm_plane.value = plane
                                dut.pm_x.value = x
                                dut.pm_y.value = y
                                await RisingEdge(dut.clk)
                                await ReadOnly()
                                v = int(dut.pm_idx.value)
                                if v != cm[y][x]:
                                    bad.append((y, x, v, cm[y][x]))
                                await Timer(1, "ns")
                        assert not bad, f"{tag0} block {blk_i} at ({m['r']},{m['c']}) plane {plane}: {len(bad)} colour-map mismatches (y, x, rtl, model), first {bad[:8]}"
                        stats["palpix"] += osw * osh
                    stats["palblocks"] += 1
                    dut.blk_ack.value = 1
                    await RisingEdge(dut.clk)
                    await Timer(1, "ns")
                    dut.blk_ack.value = 0
                    continue
                if int(dut.blk_done.value):
                    idle = 0
                    got = unpack(BLK_FIELDS, int(dut.blk_rec.value))
                    assert blk_i < len(dec.blocks), f"{tag0}: RTL produced extra block {got}"
                    m = dec.blocks[blk_i]
                    exp = dict(r=m["r"], c=m["c"], bsize=m["size"], skip=m["skip"], seg=m["seg"], lossless=m["lossless"],
                               has_chroma=m["has_chroma"], ymode=m["ymode"], uvmode=m["uvmode"], angle_y=m["angle"][0],
                               angle_uv=m["angle"][1], cfl_u=m["cfl"][0], cfl_v=m["cfl"][1], use_fi=m["fi"], fi_mode=m["fim"] if m["fi"] else 0,
                               txsz=m["tx"], qidx=m["qidx"], dlf=m["dlf"],
                               pal_y=m["pal"][0], pal_uv=m["pal"][1], col_y=m["col_y"], col_u=m["col_u"], col_v=m["col_v"],
                               is_inter=m.get("intrabc", 0) or m.get("inter", 0), intrabc=m.get("intrabc", 0),
                               mv_row=(m["mvs"][0][0] if m.get("inter") else (m["mv"][0] if m.get("mv") else 0)),
                               mv_col=(m["mvs"][0][1] if m.get("inter") else (m["mv"][1] if m.get("mv") else 0)))
                    if m.get("inter"):
                        rf = m["ref"]
                        exp.update(skip_mode=m["skip_mode"], ref0=rf[0], ref1=(rf[1] if rf[1] >= 0 else 15),
                                   mv1_row=(m["mvs"][1][0] if len(m["mvs"]) > 1 else 0), mv1_col=(m["mvs"][1][1] if len(m["mvs"]) > 1 else 0),
                                   motion_mode=m["motion_mode"], compound_type=m["compound_type"], interintra=m["interintra"],
                                   interintra_mode=m.get("interintra_mode", 0), wedge_interintra=m.get("wedge_interintra", 0),
                                   wedge_index=m.get("wedge_index", 0), wedge_sign=m.get("wedge_sign", 0), mask_type=m.get("mask_type", 0),
                                   comp_group_idx=m.get("comp_group_idx", 0), compound_idx=m.get("compound_idx", 1),
                                   interp0=m["interp"][0], interp1=m["interp"][1],
                                   cmv_intra=(m["cmv"]["intra"] if m.get("cmv") else 0), cmv=pack_cmv(m.get("cmv")),
                                   obmc=pack_obmc(m.get("obmc")), ls=pack_ls(m.get("ls")))
                    elif "skip_mode" in m:                      # an intra block of an inter frame: the inter fields at their defaults
                        exp.update(skip_mode=0, ref0=0, ref1=15, mv1_row=0, mv1_col=0, motion_mode=0, compound_type=2, interintra=0,
                                   interintra_mode=0, wedge_interintra=0, wedge_index=0, wedge_sign=0, mask_type=0, comp_group_idx=0,
                                   compound_idx=1, interp0=0, interp1=0, cmv_intra=0, cmv=0, obmc=0, ls=0)

                    def cols(v, n):
                        return [(v >> (12 * k)) & 0xFFF for k in range(n)]
                    gsigned = dict(got, angle_y=to_signed(got["angle_y"], 3), angle_uv=to_signed(got["angle_uv"], 3),
                                   cfl_u=to_signed(got["cfl_u"], 6), cfl_v=to_signed(got["cfl_v"], 6),
                                   mv_row=to_signed(got["mv_row"], 18), mv_col=to_signed(got["mv_col"], 18),
                                   mv1_row=to_signed(got["mv1_row"], 18), mv1_col=to_signed(got["mv1_col"], 18),
                                   dlf=[to_signed((got["delta_lf"] >> (7 * i)) & 0x7F, 7) for i in range(4)],
                                   col_y=cols(got["col_y"], m["pal"][0]), col_u=cols(got["col_u"], m["pal"][1]), col_v=cols(got["col_v"], m["pal"][1]))
                    bad = {k: (gsigned[k], exp[k]) for k in exp if gsigned[k] != exp[k]}
                    assert not bad, f"{tag0} block {blk_i} at ({m['r']},{m['c']}) size {m['size']}: mismatches (rtl, model) {bad}"
                    blk_i += 1
                    stats["blocks"] += 1
                if int(dut.lr_done.value):
                    idle = 0
                    got = unpack(LR_FIELDS, int(dut.lr_rec.value))
                    assert lr_i < len(dec.lr_recs), f"{tag0}: RTL produced extra LR unit {got}"
                    m = dec.lr_recs[lr_i]
                    gw = [[to_signed((got["wiener"] >> (7 * (3 * p + j))) & 0x7F, 7) for j in range(3)] for p in range(2)]
                    gx = [to_signed((got["xqd"] >> (8 * i)) & 0xFF, 8) for i in range(2)]
                    g = dict(plane=got["plane"], unit_row=got["unit_row"], unit_col=got["unit_col"], lr_type=got["lr_type"], wiener=gw, sgr_set=got["sgr_set"], xqd=gx)
                    bad = {k: (g[k], m[k]) for k in m if g[k] != m[k]}
                    assert not bad, f"{tag0} LR unit {lr_i}: mismatches (rtl, model) {bad}"
                    lr_i += 1
                    stats["lrunits"] += 1
                if DBG_TX is not None and DBG_TX - 18 <= tx_i <= DBG_TX:
                    # blk_ctx internals around a transform block (TS_DBG_TX=<record index>): the write of the previous
                    # block's categories and the read that forms this block's contexts
                    c = dut.u_ctx
                    if int(c.tx_we.value):
                        dut._log.info(f"DBG tx_we at tx_i {tx_i}: plane {int(c.t_plane.value)} x4 {int(c.t_x4.value)} y4 {int(c.t_y4.value)} w4 {int(c.t_w4.value)} h4 {int(c.t_h4.value)} dccat {int(c.w_dccat.value)} cul {int(c.w_cul.value)} tst {int(c.tst.value)} rstt {int(c.rstt.value)} t_we_pend {int(c.t_we_pend.value)} t_pend {int(c.t_pend.value)} tx_req {int(c.tx_req.value)} rbc_we {int(c.rbc_we.value)}")
                    if int(c.al_we.value):
                        dut._log.info(f"DBG al_we at tx_i {tx_i}: waddr {int(c.al_waddr.value):x} wdata {int(c.al_wdata.value):032x} rdata {int(c.al_rdata.value):032x} tst {int(c.tst.value)} rstt {int(c.rstt.value)}")
                    if int(c.tx_valid.value):
                        dut._log.info(f"DBG tx_valid at tx_i {tx_i}: plane {int(c.t_plane.value)} x4 {int(c.t_x4.value)} y4 {int(c.t_y4.value)} dcs_ctx {int(c.dcs_ctx.value)} dcs_wide_r {int(c.dcs_wide_r.value)} a_msk {int(c.a_msk.value):04x} l_msk {int(c.l_msk.value):04x} t_aw {int(c.t_aw.value):032x} t_lw {int(c.t_lw.value):032x}")
                if int(dut.tx_done.value):
                    idle = 0
                    got = unpack(TX_FIELDS, int(dut.tx_rec.value))
                    assert tx_i < len(dec.tx_recs), f"{tag0}: RTL produced extra tx block {got}"
                    m = dec.tx_recs[tx_i]
                    exp = dict(plane=m["plane"], x=m["x"], y=m["y"], txsz=m["txsz"], txtype=m["txtype"], eob=m["eob"], skip=m["skip"], lossless=m["lossless"])
                    bad = {k: (got[k], exp[k]) for k in exp if got[k] != exp[k]}
                    if bad:
                        br_ = unpack(BLK_FIELDS, int(dut.blk_rec.value))
                        bad["rtl_blk_rec"] = {k: br_[k] for k in ("r", "c", "bsize", "skip", "txsz", "is_inter", "seg", "lossless")}
                    assert not bad, f"{tag0} tx block {tx_i} (block {blk_i - 1}): mismatches (rtl, model) {bad}"
                    if not m["skip"] and m["eob"] > 0:
                        q = m["quant"]
                        await Timer(1, "ns")
                        rtl = []
                        for p in range(len(q)):
                            dut.q_addr.value = p
                            await RisingEdge(dut.clk)
                            await ReadOnly()
                            rtl.append(to_signed(int(dut.q_data.value), 21))
                            await Timer(1, "ns")
                        badq = [(p, rtl[p], q[p]) for p in range(len(q)) if rtl[p] != q[p]]
                        assert not badq, f"{tag0} tx block {tx_i}: {len(badq)} Quant mismatches, first {badq[:6]}"
                        stats["coefs"] += sum(1 for v in q if v)
                    else:
                        await Timer(1, "ns")
                    if ACK_DELAY:
                        await ClockCycles(dut.clk, ACK_DELAY)
                        await Timer(1, "ns")
                    dut.tx_ack.value = 1
                    await RisingEdge(dut.clk)
                    await Timer(1, "ns")
                    dut.tx_ack.value = 0
                    tx_i += 1
                    stats["txblocks"] += 1
                if done:
                    break
            assert blk_i == len(dec.blocks), f"{tag0}: RTL emitted {blk_i} blocks, model {len(dec.blocks)}"
            assert tx_i == len(dec.tx_recs), f"{tag0}: RTL emitted {tx_i} tx blocks, model {len(dec.tx_recs)}"
            assert lr_i == len(dec.lr_recs), f"{tag0}: RTL emitted {lr_i} LR units, model {len(dec.lr_recs)}"
            stats["tiles"] += 1
            dut._log.info(f"{tag0}: OK {len(dec.blocks)} blocks, {len(dec.tx_recs)} tx blocks, {len(dec.lr_recs)} LR units, {cycles} cycles")
