"""Fuzz the deblocking sample filter (rtl/lf_filt.sv, combinational) against lf_model.LoopFilter.sample_filter on one
line of 14 samples: random bit depth, plane kind, filter size and strength; the samples are drawn so that every filter
(none, narrow with and without hev, 6-tap, 8-tap, 14-tap) fires often. The line after the model's filter must equal the
RTL's out where mod is set and the input elsewhere, and mod must be set exactly where the model changed... a sample
the filter rewrites with the same value counts as changed for the RTL only, so the check is on the values.
Env: LFF_ITERS (default 30000), LFF_SEED."""
import os
import random
import types
import cocotb
from cocotb.triggers import Timer

import lf_model as lfm


@cocotb.test()
async def lf_filt_vs_model(dut):
    iters = int(os.environ.get("LFF_ITERS", "30000"))
    rng = random.Random(int(os.environ.get("LFF_SEED", "1")))
    kinds = {}
    for it in range(iters):
        bd = rng.choice([8, 10, 12])
        mx = (1 << bd) - 1
        chroma = rng.random() < 0.35
        fsize = rng.choice([4, 8]) if chroma else rng.choice([4, 8, 16])
        lvl = rng.randint(1, 63)
        sharp = rng.choice([0, 0, 3, 7])
        shift = 2 if sharp > 4 else 1 if sharp > 0 else 0
        limit = max(1, min(9 - sharp, lvl >> shift)) if sharp > 0 else max(1, lvl >> shift)
        blimit = 2 * (lvl + 2) + limit
        thresh = lvl >> 4
        # samples: a base level per side with small or large steps, so flat, edge and busy lines all occur
        style = rng.random()
        step = (1 << (bd - 8))
        base_p = rng.randint(0, mx); base_q = max(0, min(mx, base_p + rng.choice([0, 1, 2, 4, 8, 20, 60]) * step * rng.choice([-1, 1])))
        noise = 0 if style < 0.35 else step if style < 0.7 else rng.choice([2, 4, 16, 64]) * step
        line = [max(0, min(mx, (base_p if i < 7 else base_q) + rng.randint(-noise, noise))) for i in range(14)]
        if rng.random() < 0.1:
            line = [rng.randint(0, mx) for _ in range(14)]
        dut.bit_depth.value = bd; dut.chroma.value = int(chroma); dut.filter_size.value = fsize
        dut.limit.value = limit; dut.blimit.value = blimit; dut.thresh.value = thresh
        dut.pix.value = sum(v << (12 * i) for i, v in enumerate(line))
        await Timer(1, "ns")
        out = int(dut.out.value); mod = int(dut.mod.value)
        got = list(line)
        for k in range(-6, 6):
            if (mod >> (k + 6)) & 1:
                got[7 + k] = (out >> (12 * (k + 6))) & 0xFFF
            else:
                assert ((out >> (12 * (k + 6))) & 0xFFF) == line[7 + k], f"iter {it}: out at an unmodified position {k} differs from the input"
        plane = 1 if chroma else 0
        m = lfm.LoopFilter(types.SimpleNamespace(BitDepth=bd), None, [[list(line)], [list(line)], [list(line)]])
        m.sample_filter(7, 0, plane, limit, blimit, thresh, 1, 0, fsize)
        exp = m.planes[plane][0]
        assert got == exp, (f"iter {it}: bd {bd} chroma {chroma} size {fsize} limit {limit} blimit {blimit} thresh {thresh}\n"
                            f"  in  {line}\n  rtl {got}\n  exp {exp}\n  mod {mod:012b}")
        nmod = bin(mod).count("1")
        kind = "none" if nmod == 0 else "narrow_hev" if nmod == 2 else "narrow" if (nmod == 4 and (fsize == 4 or mod == 0b000011110000 and not _wide4(exp, line))) else f"wide{nmod}"
        kinds[kind] = kinds.get(kind, 0) + 1
    need = ["none", "narrow_hev", "wide6", "wide12"]
    assert all(kinds.get(k, 0) > 50 for k in need) and (kinds.get("narrow", 0) + kinds.get("wide4", 0)) > 100, kinds
    dut._log.info(f"OK: {kinds}")


def _wide4(exp, line):
    return False
