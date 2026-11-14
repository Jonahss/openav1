"""rtl/sr_stream.sv (super-resolution for the streaming filter chain: line segments upscaled horizontally, column by
column) against superres_model.upscale_row. Bands of made-up lines: any upscaled width, every denominator 9..16, 8 /
10 / 12 bit, segments of 8 (luma, 4:4:4 chroma) and of 4 (subsampled chroma), 1 to 150 lines a column, the input with
gaps and the output held back at random; every line of every output column must come once, in order, with its tag and
its last flag, and equal the model inside the upscaled picture. A share of the bands runs with use_sr = 0 (wires).
Env: SR_SEED, SR_BANDS (default 60)."""
import os
import random
import types
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

import superres_model as srm


@cocotb.test()
async def sr_stream_vs_model(dut):
    rng = random.Random(int(os.environ.get("SR_SEED", "1")))
    bands = int(os.environ.get("SR_BANDS", "60"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("bit_depth", "use_sr", "ssx", "down_w", "up_w", "max_x", "band_start", "n_seg_in", "in_valid", "in_line", "in_tag", "in_last", "out_ready"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk); await Timer(1, "ns")
    stats = dict(bands=0, bypass=0, lines_in=0, lines_out=0, samples=0, denoms=set(), stalls=0, max_ratio=0.0)
    got = []
    stall = [0.0]

    async def taker():
        while True:
            await RisingEdge(dut.clk)
            await Timer(1, "ns")                             # the taker is ready at random
            r = 0 if rng.random() < stall[0] else 1
            dut.out_ready.value = r; stats["stalls"] += 1 - r
            await Timer(3, "ns")                             # after the sender has put its line (2 ns after the edge)
            # out_valid and out_ready as the next clock edge will see them: that edge takes the line
            if int(dut.out_valid.value) and r:
                got.append((int(dut.out_line.value), int(dut.out_tag.value), int(dut.out_last.value)))
    cocotb.start_soon(taker())
    for b in range(bands):
        ssx = rng.random() < 0.4
        use_sr = rng.random() >= 0.12
        bd = rng.choice([8, 10, 12]); mx = (1 << bd) - 1
        denom = 9 + (b % 8) if use_sr else 8
        UW = rng.choice([rng.randint(16, 64), rng.randint(65, 320), rng.choice([64, 128, 200, 256])])
        FW = (UW * 8 + denom // 2) // denom
        hdr = types.SimpleNamespace(FrameWidth=FW, UpscaledWidth=UW, FrameHeight=4096, subsampling_x=int(ssx), subsampling_y=0,
                                    MiCols=2 * ((FW + 7) >> 3), BitDepth=bd, use_superres=int(use_sr))
        plane = 1 if ssx else 0
        segw = 4 if ssx else 8
        cw_in = (hdr.MiCols >> ssx) * 4                       # decoded sample columns of the plane
        nseg_in = cw_in // segw
        down_w = (FW + ssx) >> ssx; up_w = (UW + ssx) >> ssx
        nl = rng.choice([1, 2, 4, 8, rng.randint(3, 40), rng.randint(41, 150)])
        kind = rng.random()
        lines = [[(rng.choice([0, mx]) if kind < 0.15 else rng.randint(0, mx)) for _ in range(cw_in)] for _ in range(nl)]
        tags = [rng.getrandbits(9) for _ in range(nl)]
        dut.bit_depth.value = bd; dut.use_sr.value = int(use_sr); dut.ssx.value = int(ssx)
        dut.down_w.value = down_w; dut.up_w.value = up_w; dut.max_x.value = cw_in - 1; dut.n_seg_in.value = nseg_in
        del got[:]
        stall[0] = rng.choice([0.0, 0.0, 0.3, 0.7])
        dut.band_start.value = 1
        await RisingEdge(dut.clk); await Timer(2, "ns")
        dut.band_start.value = 0
        gap = rng.choice([0.0, 0.0, 0.4])
        for c in range(nseg_in):
            for j in range(nl):
                while rng.random() < gap:
                    dut.in_valid.value = 0
                    await RisingEdge(dut.clk); await Timer(2, "ns")
                dut.in_valid.value = 1
                dut.in_line.value = sum(lines[j][c * segw + i] << (12 * i) for i in range(segw)) | (rng.getrandbits(48) << 48 if ssx else 0)
                dut.in_tag.value = tags[j]; dut.in_last.value = int(j == nl - 1)
                n = 0
                while True:
                    await ReadOnly()
                    rdy = int(dut.in_ready.value)
                    await RisingEdge(dut.clk); await Timer(2, "ns")
                    if rdy:
                        break
                    n += 1
                    assert n < 100000, f"band {b}: the stage stopped taking lines at column {c} line {j}"
        dut.in_valid.value = 0
        if use_sr:
            exp_rows = [srm.upscale_row(hdr, plane, j, lines[j]) for j in range(nl)]
            w_out = up_w
            nseg_out = (up_w + segw - 1) // segw
        else:
            exp_rows = lines; w_out = cw_in; nseg_out = nseg_in
        n = 0
        while len(got) < nseg_out * nl:
            await RisingEdge(dut.clk); n += 1
            assert n < 400000, f"band {b}: {len(got)} of {nseg_out * nl} lines came out (use_sr {use_sr} ssx {ssx} {down_w} -> {up_w}, {nl} lines)"
        await ClockCycles(dut.clk, 12); await Timer(2, "ns")
        assert len(got) == nseg_out * nl, f"band {b}: {len(got)} lines came out, {nseg_out * nl} expected"
        assert int(dut.n_seg_out.value) == nseg_out
        tag = f"band {b} ({'sr' if use_sr else 'wires'} ssx {int(ssx)} bd{bd} denom {denom} {down_w} -> {up_w}, {nl} lines)"
        for i, (v, t, last) in enumerate(got):
            c, j = divmod(i, nl)
            assert t == tags[j] and last == int(j == nl - 1), f"{tag}: output {i} has tag {t} last {last}, expected line {j} (tag {tags[j]})"
            for k in range(segw):
                x = c * segw + k
                if x >= w_out:
                    continue
                g = (v >> (12 * k)) & 0xFFF
                assert g == exp_rows[j][x], f"{tag}: column {c} line {j} sample {x} = {g}, model {exp_rows[j][x]}"
                stats["samples"] += 1
        stats["bands"] += 1; stats["bypass"] += not use_sr; stats["lines_in"] += nseg_in * nl; stats["lines_out"] += len(got)
        if use_sr:
            stats["denoms"].add(denom); stats["max_ratio"] = max(stats["max_ratio"], round(up_w / down_w, 3))
    stats["denoms"] = sorted(stats["denoms"])
    assert stats["denoms"] == list(range(9, 17)) and stats["bypass"] > 0 and stats["stalls"] > 1000, stats
    dut._log.info(f"OK: {stats}")
