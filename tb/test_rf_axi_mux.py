"""rf_axi_mux: two read masters on one read port. Two sources issue random bursts with random gaps; a slave takes
the read addresses with random stalls and answers in order with random gaps. Checked: every burst of a side reaches
the port unchanged and in that side's order; every beat's valid goes to the side that owns the burst being answered
and to no other; with both sides always asking, neither is served twice in a row (the turn rule); nothing is left
outstanding. Env: MX_BURSTS (bursts a side, default 3000), MX_SEED."""
import os
import random
from collections import deque
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer


async def run(dut, n, seed, gap_p, stall_p, rgap_p):
    rng = random.Random(seed)
    dut.rst.value = 1
    dut.s_arvalid.value = 0
    dut.m_arready.value = 0; dut.m_rvalid.value = 0; dut.m_rlast.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    todo = [deque(((i << 39) | rng.getrandbits(30), rng.randrange(16)) for _ in range(n)) for i in range(2)]
    sent = [deque(todo[0]), deque(todo[1])]               # what the port must show, per side, in order
    valid = [False, False]
    pending = deque()                                      # bursts the slave has taken: [owner, beats left]
    seen = [0, 0]; beats = [0, 0]; order = []
    idle = 0
    while (todo[0] or todo[1] or valid[0] or valid[1] or pending or int(dut.m_arvalid.value)) and idle < 5000:
        # sources: raise a request, hold it until taken
        for i in range(2):
            if not valid[i] and todo[i] and rng.random() >= gap_p:
                a, l = todo[i][0]
                dut.s_araddr[i].value = a; dut.s_arlen[i].value = l; valid[i] = True
        dut.s_arvalid.value = (valid[1] << 1) | valid[0]
        # slave: address ready at random, one beat of the oldest burst at random
        arr = rng.random() >= stall_p
        dut.m_arready.value = int(arr)
        rv = bool(pending) and rng.random() >= rgap_p
        dut.m_rvalid.value = int(rv)
        dut.m_rlast.value = int(rv and pending[0][1] == 1)
        await ReadOnly()
        rdy = int(dut.s_arready.value)
        srv = int(dut.s_rvalid.value)
        mv, ma, ml = int(dut.m_arvalid.value), int(dut.m_araddr.value), int(dut.m_arlen.value)
        assert rdy in (0, 1, 2), "both sides taken in one cycle"
        if rv:
            own = pending[0][0]
            assert srv == (1 << own), f"beat of side {own}'s burst: s_rvalid = {srv:02b}"
            beats[own] += 1
        else:
            assert srv == 0, f"s_rvalid = {srv:02b} with no beat"
        await RisingEdge(dut.clk)
        await Timer(1, "ns")
        idle += 1
        for i in range(2):
            if valid[i] and (rdy >> i) & 1:
                todo[i].popleft(); valid[i] = False; order.append(i); idle = 0
        if mv and arr:
            own = ma >> 39
            assert sent[own] and sent[own][0] == (ma, ml), f"port shows {ma:#x} len {ml}, side {own} expected {sent[own][0] if sent[own] else None}"
            sent[own].popleft(); seen[own] += 1
            pending.append([own, ml + 1]); idle = 0
        if rv:
            pending[0][1] -= 1
            if pending[0][1] == 0:
                pending.popleft()
            idle = 0
    assert idle < 5000, "stuck"
    assert seen == [n, n] and not pending, f"bursts through the port {seen}, outstanding {len(pending)}"
    return order, beats


@cocotb.test()
async def two_sides(dut):
    n = int(os.environ.get("MX_BURSTS", "3000"))
    seed = int(os.environ.get("MX_SEED", "1"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    order, beats = await run(dut, n, seed, 0.5, 0.3, 0.3)
    dut._log.info(f"random gaps: {2 * n} bursts, beats a side {beats}")
    # both sides always asking, a slow slave so that the outstanding FIFO fills: strict alternation
    order, beats2 = await run(dut, n, seed + 1, 0.0, 0.6, 0.9)
    twice = sum(1 for a, b in zip(order[:2 * n - 2], order[1:2 * n - 1]) if a == b)
    assert twice == 0, f"a side was served twice in a row {twice} times while the other was asking"
    dut._log.info(f"OK: {4 * n} bursts, beats a side {beats} and {beats2}, alternation held with both sides asking")
