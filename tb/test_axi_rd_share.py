"""axi_rd_share: two readers on one AXI read channel. Each reader issues random bursts (it holds a request until it is
taken, as ref_fetch and mf_project do); a slave answers every burst in request order, after a random delay and with
gaps between beats. Checked: every reader receives exactly the beats of its own bursts, in its own order (the slave
puts the burst's address and the beat index on the data lines); a request never changes while it waits; nothing is
answered that was not asked. Env: RS_BURSTS (bursts a reader, default 400), RS_SEED."""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer

N = int(os.environ.get("RS_BURSTS", "400"))
SEED = int(os.environ.get("RS_SEED", "1"))


@cocotb.test()
async def share(dut):
    rng = random.Random(SEED)
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("s_araddr", "s_arlen", "s_arvalid", "s_rready", "m_arready", "m_rvalid", "m_rlast"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    # what each reader will ask for: (address, beats); addresses carry the reader in bit 39
    plan = [[((i << 39) | (k << 8) | rng.randrange(256), rng.randrange(1, 17)) for k in range(N)] for i in range(2)]
    sent = [0, 0]                       # bursts taken
    got = [[], []]                      # (address, beat) per reader, as received
    pend = [None, None]                 # the request a reader is presenting
    gap = [0, 0]
    inflight = []                       # the slave: (address, beats, ready time)
    beat = 0
    cur = None
    cycle = 0
    presented = None                    # (addr, len) seen on the channel while not taken
    stats = dict(bursts=0, beats=0, both_asking=0, held=0, max_inflight=0)
    while sent != [N, N] or inflight or cur is not None:
        # readers: a new request after a random gap; once presented it stays
        av = 0
        ad = 0
        al = 0
        for i in range(2):
            if pend[i] is None and sent[i] < N:
                if gap[i] == 0:
                    pend[i] = plan[i][sent[i]]
                    gap[i] = rng.choice([0, 0, 0, 1, 2, 5, 30]) if i == 0 else rng.choice([0, 3, 10, 80])
                else:
                    gap[i] -= 1
            if pend[i] is not None:
                av |= 1 << i
                ad |= pend[i][0] << (40 * i)
                al |= (pend[i][1] - 1) << (8 * i)
        dut.s_arvalid.value = av
        dut.s_araddr.value = ad
        dut.s_arlen.value = al
        dut.s_rready.value = 3
        if av == 3:
            stats["both_asking"] += 1
        # the slave
        dut.m_arready.value = int(rng.random() < 0.7)
        if cur is None and inflight and inflight[0][2] <= cycle:
            cur = inflight.pop(0)
            beat = 0
        send = cur is not None and rng.random() < 0.8
        dut.m_rvalid.value = int(send)
        dut.m_rlast.value = int(send and beat == cur[1] - 1)
        await ReadOnly()
        m_av, m_ar = int(dut.m_arvalid.value), int(dut.m_arready.value)
        m_ad, m_al = int(dut.m_araddr.value), int(dut.m_arlen.value)
        if presented is not None:
            assert m_av and (m_ad, m_al) == presented, f"cycle {cycle}: a waiting request changed: {presented} -> {(m_av, m_ad, m_al)}"
            stats["held"] += 1
        presented = (m_ad, m_al) if (m_av and not m_ar) else None
        s_ar = int(dut.s_arready.value)
        s_rv = int(dut.s_rvalid.value)
        if m_av and m_ar:
            i = m_ad >> 39
            assert (s_ar & av) == (1 << i), f"cycle {cycle}: the channel took reader {i}'s request, arready says {s_ar:02b} to requests {av:02b}"
            assert pend[i] is not None and (m_ad, m_al + 1) == pend[i], f"cycle {cycle}: took {(m_ad, m_al + 1)}, reader {i} asked {pend[i]}"
            inflight.append((m_ad, m_al + 1, cycle + 1 + rng.choice([0, 0, 1, 4, 20])))
            stats["max_inflight"] = max(stats["max_inflight"], len(inflight) + (cur is not None))
        else:
            assert (s_ar & av) == 0, f"cycle {cycle}: arready {s_ar:02b} to requests {av:02b} with nothing taken downstream"   # (ready without a request is legal)
        taken = None
        if send:
            assert int(dut.m_rready.value) == 1, f"cycle {cycle}: the readers are ready, the channel is not"
            i = cur[0] >> 39
            assert s_rv == (1 << i), f"cycle {cycle}: a beat of reader {i}'s burst went to {s_rv:02b}"
            got[i].append((cur[0], beat))
            stats["beats"] += 1
            taken = beat == cur[1] - 1
        else:
            assert s_rv == 0, f"cycle {cycle}: rvalid {s_rv:02b} without a beat"
        await RisingEdge(dut.clk)
        await Timer(1, "ns")
        if m_av and m_ar:
            i = m_ad >> 39
            pend[i] = None
            sent[i] += 1
            stats["bursts"] += 1
        if send:
            beat += 1
            if taken:
                cur = None
        cycle += 1
        assert cycle < 400 * N, "the test did not finish"
    for i in range(2):
        want = [(a, b) for a, n in plan[i] for b in range(n)]
        assert got[i] == want, f"reader {i}: {len(got[i])} beats received, {len(want)} asked; first difference at {next((k for k, (x, y) in enumerate(zip(got[i], want)) if x != y), None)}"
    assert stats["both_asking"] > N // 10 and stats["held"] > N // 10 and stats["max_inflight"] >= 4, f"the run did not exercise the arbiter: {stats}"
    dut._log.info(f"OK: {stats}")
