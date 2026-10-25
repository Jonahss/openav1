"""mv_stack (7.10.2 find_mv_stack in RTL) vs tb/tile_model.py's find_mv_stack, on synthetic inter frames.

Frames of random blocks (a random partition of every superblock, in decode order); before each block is "decoded"
the stack is computed by both and compared: NumMvFound, RefStackMv (list 0; list 1 for compound blocks), GlobalMvs,
NewMvContext / RefMvContext / ZeroMvContext, DrlCtxStack. Then the block's motion info is written (mv_mem / the
model's arrays), as the decoder does. The motion field (temporal candidates) is random with holes.

Env: MVS_SEEDS (count or comma list, default "1,2,3,4"), MVS_BLOCKS (blocks per seed, default all), MVS_DEBUG=1.
"""
import os
import random
import sys
from pathlib import Path
from types import SimpleNamespace

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import av1_tables as T          # noqa: E402
import tile_model as tm         # noqa: E402
from host_regs import MIW, HDR_FIELDS, IHDR_FIELDS, ENT_FIELDS, pack   # noqa: E402

NONE_RTL = 15
SINGLE_MODES = [tm.NEARESTMV, tm.NEARMV, tm.GLOBALMV, tm.NEWMV]
COMP_MODES = [tm.NEAREST_NEARESTMV, tm.NEAR_NEARMV, tm.NEAREST_NEWMV, tm.NEW_NEARESTMV, tm.NEAR_NEWMV, tm.NEW_NEARMV,
              tm.GLOBAL_GLOBALMV, tm.NEW_NEWMV]


def to_signed(v, bits):
    return v - (1 << bits) if v >> (bits - 1) else v


def partition(rnd, r, c, bs, mi_rows, mi_cols, out):
    """A random partition of the square block (r, c, bs) into blocks in decode order (z-order), inside the frame."""
    if r >= mi_rows or c >= mi_cols:
        return
    n4 = T.Num_4x4_Blocks_Wide[bs]
    half = n4 // 2
    sq = {16: 15, 8: 12, 4: 9, 2: 6, 1: 3}      # n4 (in 4x4 units, half-size) -> square bsize ... built below
    # square sizes by n4: 32->BLOCK_128X128(15), 16->64X64(12), 8->32X32(9), 4->16X16(6), 2->8X8(3), 1->4X4(0)
    SQ = {32: 15, 16: 12, 8: 9, 4: 6, 2: 3, 1: 0}
    if n4 == 1:
        out.append((r, c, 0))
        return
    p = rnd.random()
    if n4 >= 4 and p < 0.45:                        # SPLIT
        for dr, dc in ((0, 0), (0, half), (half, 0), (half, half)):
            partition(rnd, r + dr, c + dc, SQ[half], mi_rows, mi_cols, out)
    elif p < 0.65:                                  # NONE
        out.append((r, c, bs))
    elif p < 0.78:                                  # HORZ: two n4 x half blocks
        hb = {32: 14, 16: 11, 8: 8, 4: 5, 2: 2}[n4]
        out.append((r, c, hb))
        if r + half < mi_rows:
            out.append((r + half, c, hb))
    elif p < 0.91:                                  # VERT
        vb = {32: 13, 16: 10, 8: 7, 4: 4, 2: 1}[n4]
        out.append((r, c, vb))
        if c + half < mi_cols:
            out.append((r, c + half, vb))
    elif n4 in (4, 8, 16) and p < 0.955:            # HORZ_4
        hb = {16: 21, 8: 19, 4: 17}[n4]
        for k in range(4):
            if r + k * (n4 // 4) < mi_rows:
                out.append((r + k * (n4 // 4), c, hb))
    elif n4 in (4, 8, 16):                          # VERT_4
        vb = {16: 20, 8: 18, 4: 16}[n4]
        for k in range(4):
            if c + k * (n4 // 4) < mi_cols:
                out.append((r, c + k * (n4 // 4), vb))
    else:
        out.append((r, c, bs))


class StackModel(tm.TileDecoder):
    """tile_model's MV stack over synthetic frame state (no stream)."""
    def __init__(self, h):
        self.h = h
        self.use_intrabc = 0
        self.RefFrameSignBias = list(h.RefFrameSignBias)
        R, C = h.MiRows, h.MiCols
        self.IsInters = [[0] * C for _ in range(R)]
        self.RefFrames = [[None] * C for _ in range(R)]
        self.Mvs = [[[[0, 0], [0, 0]] for _ in range(C)] for _ in range(R)]
        self.YModes = [[0] * C for _ in range(R)]
        self.MiSizes = [[0] * C for _ in range(R)]
        self.MotionFieldMvs = [[[[tm.MV_INVALID, tm.MV_INVALID] for _ in range(C // 2 + 1)] for _ in range(R // 2 + 1)] for _ in range(8)]


def rand_header(rnd, seed):
    h = SimpleNamespace()
    h.use_128x128_superblock = rnd.random() < 0.3
    sb4 = 32 if h.use_128x128_superblock else 16
    h.MiRows = rnd.choice([16, 17, 20, 23, 24, 32, 40])
    h.MiCols = rnd.choice([16, 19, 24, 32, 45, 48, 56])
    # the tile: whole frame, or a superblock-aligned sub-rectangle
    if rnd.random() < 0.6:
        h.MiRowStart, h.MiColStart, h.MiRowEnd, h.MiColEnd = 0, 0, h.MiRows, h.MiCols
    else:
        h.MiRowStart = rnd.choice([0, sb4]) if h.MiRows > sb4 else 0
        h.MiColStart = rnd.choice([0, sb4]) if h.MiCols > sb4 else 0
        h.MiRowEnd, h.MiColEnd = h.MiRows, h.MiCols
    h.force_integer_mv = int(rnd.random() < 0.2)
    h.allow_high_precision_mv = 0 if h.force_integer_mv else int(rnd.random() < 0.5)
    h.use_ref_frame_mvs = int(rnd.random() < 0.7)
    h.RefFrameSignBias = [0] + [rnd.randint(0, 1) for _ in range(7)]
    h.GmType = [0] * 8
    h.gm_params = [[0, 0, 1 << 16, 0, 0, 1 << 16] for _ in range(8)]
    for ref in range(1, 8):
        t = rnd.choice([0, 0, 1, 2, 3])
        h.GmType[ref] = t
        if t == 1:
            h.gm_params[ref][0] = rnd.randint(-3000, 3000) << 3
            h.gm_params[ref][1] = rnd.randint(-3000, 3000) << 3
        elif t >= 2:
            h.gm_params[ref][0] = rnd.randint(-(1 << 15), 1 << 15)
            h.gm_params[ref][1] = rnd.randint(-(1 << 15), 1 << 15)
            h.gm_params[ref][2] = (1 << 16) + rnd.randint(-4000, 4000)
            h.gm_params[ref][3] = rnd.randint(-4000, 4000) if t == 3 or True else 0
            h.gm_params[ref][4] = rnd.randint(-4000, 4000)
            h.gm_params[ref][5] = (1 << 16) + rnd.randint(-4000, 4000)
    return h


def hdr_words(h):
    v = dict((n, 0) for n, _ in HDR_FIELDS)
    v.update(mi_rows=h.MiRows, mi_cols=h.MiCols, mi_row_start=h.MiRowStart, mi_row_end=h.MiRowEnd,
             mi_col_start=h.MiColStart, mi_col_end=h.MiColEnd, sb128=int(h.use_128x128_superblock))
    return pack(HDR_FIELDS, v)


def ihdr_words(h):
    v = dict((n, 0) for n, _ in IHDR_FIELDS)
    gt = gp = sb = 0
    for i in range(7):
        ref = 1 + i
        gt |= (h.GmType[ref] & 3) << (2 * i)
        sb |= h.RefFrameSignBias[ref] << i
        for j in range(6):
            gp |= (h.gm_params[ref][j] & 0xFFFFFF) << (24 * (6 * i + j))
    v.update(is_inter=1, allow_hp_mv=h.allow_high_precision_mv, force_integer_mv=h.force_integer_mv,
             use_ref_frame_mvs=h.use_ref_frame_mvs, gm_type=gt, gm_params=gp, ref_sign_bias=sb)
    return pack(IHDR_FIELDS, v)


def rand_mv(rnd, h):
    v = rnd.choice([rnd.randint(-64, 64), rnd.randint(-600, 600), 0, 0])
    return v


async def wait_busy_low(dut, sig, tag):
    for _ in range(100000):
        await RisingEdge(dut.clk)
        if int(sig.value) == 0:
            return
    raise AssertionError(f"{tag}: timeout")


@cocotb.test()
async def mv_stack_vs_model(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("rst", "mvm_we", "mvm_r", "mvm_c", "mvm_bw4", "mvm_bh4", "mvm_data", "mvm_clr", "clr_row", "clr_n", "mf_we", "mf_wref",
              "mf_wy8", "mf_wx8", "mf_wvalid", "mf_wrow", "mf_wcol", "start", "r", "c", "bs", "ref0", "ref1", "is_comp", "hdr", "ihdr"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 4)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)
    seeds = os.environ.get("MVS_SEEDS", "1,2,3,4")
    seeds = list(range(1, int(seeds) + 1)) if seeds.isdigit() else [int(x) for x in seeds.split(",")]
    max_blocks = int(os.environ.get("MVS_BLOCKS", "0"))
    debug = os.environ.get("MVS_DEBUG") == "1"
    stats = dict(blocks=0, compound=0, found=0, tmv=0, n0=0, n1=0, comp_n2=0, sb128=0, subtile=0)
    for seed in seeds:
        rnd = random.Random(seed)
        h = rand_header(rnd, seed)
        m = StackModel(h)
        dut.hdr.value = hdr_words(h)
        dut.ihdr.value = ihdr_words(h)
        # the motion field: random vectors with holes
        for ref in range(1, 8):
            for y8 in range(h.MiRows >> 1):
                for x8 in range(h.MiCols >> 1):
                    valid = rnd.random() < 0.7
                    mv = [rand_mv(rnd, h), rand_mv(rnd, h)] if valid else [tm.MV_INVALID, tm.MV_INVALID]
                    m.MotionFieldMvs[ref][y8][x8] = mv
                    dut.mf_we.value = 1
                    dut.mf_wref.value = ref - 1
                    dut.mf_wy8.value = y8
                    dut.mf_wx8.value = x8
                    dut.mf_wvalid.value = int(valid)
                    dut.mf_wrow.value = mv[0] & 0xFFFF if valid else 0
                    dut.mf_wcol.value = mv[1] & 0xFFFF if valid else 0
                    await RisingEdge(dut.clk)
        dut.mf_we.value = 0
        sb4 = 32 if h.use_128x128_superblock else 16
        sb_bs = 15 if h.use_128x128_superblock else 12
        nblk = 0
        for sr in range(h.MiRowStart, h.MiRowEnd, sb4):
            # new superblock row: its motion-info rows are unwritten
            dut.mvm_clr.value = 1
            dut.clr_row.value = sr
            dut.clr_n.value = sb4
            await RisingEdge(dut.clk)
            dut.mvm_clr.value = 0
            await RisingEdge(dut.clk)
            await wait_busy_low(dut, dut.mvm_busy, "clr")
            for sc in range(h.MiColStart, h.MiColEnd, sb4):
                blocks = []
                partition(rnd, sr, sc, sb_bs, h.MiRowEnd, h.MiColEnd, blocks)
                for (r, c, bs) in blocks:
                    if max_blocks and nblk >= max_blocks:
                        break
                    nblk += 1
                    bw4, bh4 = T.Num_4x4_Blocks_Wide[bs], T.Num_4x4_Blocks_High[bs]
                    is_inter = rnd.random() < 0.85
                    if is_inter:
                        ref0 = rnd.randint(1, 7)
                        comp = rnd.random() < 0.35 and min(bw4, bh4) >= 2
                        ref1 = rnd.choice([x for x in range(1, 8) if x != ref0]) if comp else tm.NONE
                    else:
                        ref0, ref1, comp = tm.INTRA_FRAME, tm.NONE, False
                    # ---- the stack, both sides
                    if is_inter:
                        m.MiRow, m.MiCol, m.MiSize = r, c, bs
                        m.RefFrame = [ref0, ref1]
                        m.find_mv_stack(int(comp))
                        dut.r.value, dut.c.value, dut.bs.value = r, c, bs
                        dut.ref0.value = ref0
                        dut.ref1.value = ref1 if comp else NONE_RTL
                        dut.is_comp.value = int(comp)
                        dut.start.value = 1
                        await RisingEdge(dut.clk)
                        dut.start.value = 0
                        await RisingEdge(dut.clk)
                        await wait_busy_low(dut, dut.busy, f"seed {seed} block ({r},{c}) bs {bs}")
                        await ReadOnly()
                        tag = f"seed {seed} block ({r},{c}) bs {bs} refs {ref0},{ref1}"
                        n = int(dut.n_found.value)
                        assert n == m.NumMvFound, f"{tag}: NumMvFound RTL {n} model {m.NumMvFound}"
                        st0r, st0c = int(dut.st0_row_f.value), int(dut.st0_col_f.value)
                        st1r, st1c = int(dut.st1_row_f.value), int(dut.st1_col_f.value)
                        for k in range(min(4, max(n, 2))):
                            got = [to_signed((st0r >> (18 * k)) & 0x3FFFF, 18), to_signed((st0c >> (18 * k)) & 0x3FFFF, 18)]
                            assert got == m.RefStackMv[k][0], f"{tag}: stack[{k}][0] RTL {got} model {m.RefStackMv[k][0]} (n {n})"
                            if comp and k < n:
                                got = [to_signed((st1r >> (18 * k)) & 0x3FFFF, 18), to_signed((st1c >> (18 * k)) & 0x3FFFF, 18)]
                                assert got == m.RefStackMv[k][1], f"{tag}: stack[{k}][1] RTL {got} model {m.RefStackMv[k][1]}"
                        gr, gc = int(dut.gmv_row_f.value), int(dut.gmv_col_f.value)
                        for k in range(1 + int(comp)):
                            got = [to_signed((gr >> (18 * k)) & 0x3FFFF, 18), to_signed((gc >> (18 * k)) & 0x3FFFF, 18)]
                            assert got == m.GlobalMvs[k], f"{tag}: GlobalMvs[{k}] RTL {got} model {m.GlobalMvs[k]}"
                        assert int(dut.new_mv_ctx.value) == m.NewMvContext, f"{tag}: NewMvContext RTL {int(dut.new_mv_ctx.value)} model {m.NewMvContext}"
                        assert int(dut.ref_mv_ctx.value) == m.RefMvContext, f"{tag}: RefMvContext RTL {int(dut.ref_mv_ctx.value)} model {m.RefMvContext}"
                        assert int(dut.zero_mv_ctx.value) == m.ZeroMvContext, f"{tag}: ZeroMvContext RTL {int(dut.zero_mv_ctx.value)} model {m.ZeroMvContext}"
                        drl = int(dut.drl_ctx_f.value)
                        for k in range(min(3, n)):
                            assert (drl >> (2 * k)) & 3 == m.DrlCtxStack[k], f"{tag}: DrlCtxStack[{k}] RTL {(drl >> (2 * k)) & 3} model {m.DrlCtxStack[k]}"
                        stats["blocks"] += 1
                        stats["compound"] += int(comp)
                        stats["found"] += n
                        stats["tmv"] += h.use_ref_frame_mvs
                        stats["n0"] += int(n == 0)
                        stats["n1"] += int(n == 1)
                        stats["comp_n2"] += int(comp and n == 2)
                        stats["sb128"] += int(h.use_128x128_superblock)
                        stats["subtile"] += int(h.MiRowStart > 0 or h.MiColStart > 0)
                        if debug:
                            dut._log.info(f"{tag}: n {n} stack0 {m.RefStackMv[0]} ctx {m.NewMvContext}/{m.RefMvContext}/{m.ZeroMvContext}")
                        await RisingEdge(dut.clk)
                    # ---- "decode" the block: random mode / MVs, written to both sides
                    if is_inter:
                        mode = rnd.choice(COMP_MODES if comp else SINGLE_MODES)
                        mv0 = [rand_mv(rnd, h), rand_mv(rnd, h)]
                        mv1 = [rand_mv(rnd, h), rand_mv(rnd, h)] if comp else [0, 0]
                    else:
                        mode, mv0, mv1 = tm.DC_PRED, [0, 0], [0, 0]
                    for rr in range(r, min(r + bh4, h.MiRows)):
                        for cc in range(c, min(c + bw4, h.MiCols)):
                            m.IsInters[rr][cc] = int(is_inter)
                            m.RefFrames[rr][cc] = [ref0, ref1]
                            m.Mvs[rr][cc] = [list(mv0), list(mv1)]
                            m.YModes[rr][cc] = mode
                            m.MiSizes[rr][cc] = bs
                    ent = dict(is_inter=int(is_inter), is_intrabc=0, ref0=ref0, ref1=(ref1 if ref1 != tm.NONE else NONE_RTL),
                               mv_row=mv0[0], mv_col=mv0[1], mv1_row=mv1[0], mv1_col=mv1[1],
                               newmv_fam=int(tm.TileDecoder.has_newmv(mode)) if is_inter else 0,
                               globalmv_fam=int(mode in (tm.GLOBALMV, tm.GLOBAL_GLOBALMV)) if is_inter else 0,
                               bsize=bs, skip_mode=0, comp_group_idx=0, compound_idx=1, interp0=0, interp1=0, seg_pred=0)
                    dut.mvm_we.value = 1
                    dut.mvm_r.value, dut.mvm_c.value = r, c
                    dut.mvm_bw4.value, dut.mvm_bh4.value = bw4, bh4
                    dut.mvm_data.value = pack(ENT_FIELDS, ent)
                    await RisingEdge(dut.clk)
                    dut.mvm_we.value = 0
                    await RisingEdge(dut.clk)
                    await wait_busy_low(dut, dut.mvm_busy, "mvm write")
        dut._log.info(f"seed {seed}: {h.MiCols * 4}x{h.MiRows * 4} sb{128 if h.use_128x128_superblock else 64} tile ({h.MiRowStart},{h.MiColStart})"
                      f" hp {h.allow_high_precision_mv} int {h.force_integer_mv} tmv {h.use_ref_frame_mvs}: {nblk} blocks OK")
    dut._log.info(f"OK: {stats}")
