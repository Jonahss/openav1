"""The decoder through its AXI-Lite host interface (rtl/dec_host.sv): the same generated streams and model as
test_dec_top, but every header, CDF default row, tile byte, start and pixel read goes through the register map,
exactly as software on the board does it. With TS_TRANSCRIPT=<file> the register traffic is written out
(tb/axil.py format) for tools/kv260/play.py to replay on the KV260 against the bitstream.
Env: TS_SEEDS, TS_W / TS_H (default 64x64), TS_FMT, TS_STAGE (recon | lf), TS_TRANSCRIPT.
"""
import os
import random
import sys
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import cdf_map as M                     # noqa: E402
import lf_model as lfm                  # noqa: E402
import test_dec_top as TD               # noqa: E402
import test_lf_top as TL                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402
from axil import AxiLite, write_transcript   # noqa: E402

STAGE = os.environ.get("TS_STAGE", "recon")

# register map (rtl/dec_host.sv)
R_ID, R_CFG, R_CTRL, R_START, R_STATUS, R_PERF_SYN, R_PERF_REC = 0x000, 0x004, 0x008, 0x00C, 0x010, 0x014, 0x018
R_DATA1, R_DATA4, R_HADDR, R_HDATA, R_DEF_ADDR = 0x01C, 0x020, 0x024, 0x028, 0x02C
R_DEF, R_HDR, R_RH, R_LH, R_CH = 0x040, 0x080, 0x0C0, 0x100, 0x140
ST_TILE_BUSY, ST_TILE_DONE, ST_UNSUP, ST_LF_DONE, ST_FIFO_EMPTY = 1 << 0, 1 << 1, 1 << 2, 1 << 6, 1 << 15
START_TILE, START_LF = 1, 4
CTRL_RST, CTRL_EOS = 1, 1 << 8


def words(v, n):
    return [(v >> (32 * i)) & 0xFFFFFFFF for i in range(n)]


async def write_wide(ax, base, v, nwords):
    for i, w in enumerate(words(v, nwords)):
        await ax.write(base + 4 * i, w)


async def run_tile(ax, th, dec, data, tag, stats):
    await write_wide(ax, R_HDR, TT.pack(TT.HDR_FIELDS, TT.hdr_vals(th, dec, 0)), 8)
    await write_wide(ax, R_RH, TT.pack(TD.REC_FIELDS, TD.rec_vals(th)), 7)
    for i, row in enumerate(M.default_rows(th.base_q_idx)):
        await write_wide(ax, R_DEF, row, 8)
        await ax.write(R_DEF_ADDR, i)
    await ax.write(R_START, START_TILE)
    # the bytes: whole words, the FIFO's free count read before each burst
    pos = 0
    while pos < len(data):
        st = await ax.read(R_STATUS, record=False)
        free = st >> 16
        if free == 0:
            await ClockCycles(ax.clk, 64)
            continue
        n = min(free, (len(data) - pos + 3) // 4)
        for _ in range(n):
            chunk = data[pos:pos + 4]
            if len(chunk) == 4:
                await ax.write(R_DATA4, int.from_bytes(chunk, "little"))
            else:
                for b in chunk:
                    await ax.write(R_DATA1, b)
            pos += 4
    ctrl = await ax.read(R_CTRL, record=False)
    await ax.write(R_CTRL, ctrl | CTRL_EOS)
    st = await ax.poll(R_STATUS, ST_TILE_DONE | ST_UNSUP, ST_TILE_DONE, 400_000_000, tag)
    assert not (st & ST_UNSUP), f"{tag}: RTL flagged unsupported syntax"
    stats["tiles"] += 1
    stats["bytes"] += len(data)


async def read_back(ax, hdr, planes, tag, stats):
    for plane in range(hdr.NumPlanes):
        sx = hdr.subsampling_x if plane else 0
        sy = hdr.subsampling_y if plane else 0
        W = (hdr.MiCols * 4) >> sx
        H = (hdr.MiRows * 4) >> sy
        bad = []
        for y in range(H):
            for x in range(W):
                await ax.write(R_HADDR, (plane << 28) | (y << 14) | x)
                v = await ax.read(R_HDATA, expect=planes[plane][y][x])
                if v != planes[plane][y][x]:
                    bad.append((y, x, v, planes[plane][y][x]))
        stats["pixels"] += W * H
        assert not bad, f"{tag}: plane {plane}: {len(bad)} / {W * H} samples differ, first (y, x, rtl, model): {bad[:8]}"


@cocotb.test()
async def dec_host_streams(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    if hasattr(dut, "clk2x"):
        cocotb.start_soon(Clock(dut.clk2x, 5, unit="ns").start())

        async def phase_gen():
            while True:
                await RisingEdge(dut.clk)
                await Timer(2.5, "ns")
                dut.phase.value = 1
                await Timer(5, "ns")
                dut.phase.value = 0
        dut.phase.value = 0
        cocotb.start_soon(phase_gen())
    transcript = [] if os.environ.get("TS_TRANSCRIPT") else None
    ax = AxiLite(dut, dut.clk, transcript=transcript)
    dut.aresetn.value = 0
    await ClockCycles(dut.clk, 4)
    dut.aresetn.value = 1
    await ClockCycles(dut.clk, 2)
    ident = await ax.read(R_ID, expect=0xA71D0001)
    assert ident == 0xA71D0001, f"ID register reads 0x{ident:08x}"
    cfg = await ax.read(R_CFG)
    dut._log.info(f"dec_host ID 0x{ident:08x} CFG 0x{cfg:08x}: FBX {cfg >> 16 & 0xFF} FBY {cfg >> 8 & 0xFF} lanes {cfg >> 4 & 0xF} uram {cfg & 1}")
    seeds = [int(s) for s in os.environ.get("TS_SEEDS", "1").split(",")]
    W = int(os.environ.get("TS_W", "64"))
    H = int(os.environ.get("TS_H", "64"))
    stats = dict(frames=0, tiles=0, bytes=0, pixels=0, stage=STAGE)
    for seed in seeds:
        fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
        d, args = TT.gen(seed, W, H, fmt)
        tag = f"seed {seed} {fmt} bd{args['bd']}"
        await ax.write(R_CTRL, CTRL_RST)
        await ClockCycles(dut.clk, 4)
        await ax.write(R_CTRL, 0)
        frame = {}
        hdr0 = None
        decs = []
        for ti, (th, data) in enumerate(d.tiles):
            dec = TD.RecFrame(th, data, frame)
            dec.decode_tile()
            decs.append(dec)
            hdr0 = th
            await run_tile(ax, th, dec, data, f"{tag} tile {ti} ({th.MiColStart},{th.MiRowStart})", stats)
        planes = frame["planes"]
        if STAGE == "lf":
            state = xf.FrameState(hdr0, decs)
            lfm.LoopFilter(hdr0, state, planes).apply()
            await write_wide(ax, R_LH, TT.pack(TL.LF_FIELDS, TL.lf_vals(hdr0)), 11)
            await ax.write(R_START, START_LF)
            await ax.poll(R_STATUS, ST_LF_DONE, ST_LF_DONE, 50_000_000, tag)
        await read_back(ax, hdr0, planes, tag, stats)
        stats["frames"] += 1
        dut._log.info(f"{tag} {W}x{H}: frame identical ({len(d.tiles)} tiles, {len(transcript) if transcript else 0} register ops so far)")
    if transcript is not None:
        write_transcript(os.environ["TS_TRANSCRIPT"], transcript)
        dut._log.info(f"transcript: {len(transcript)} ops -> {os.environ['TS_TRANSCRIPT']}")
    dut._log.info(f"OK: {stats}")
