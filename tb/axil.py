"""A minimal AXI4-Lite master for cocotb (one transaction at a time) that can also record a transcript of the
register traffic: W <addr> <data>, R <addr> <expect> (a read whose value is checked), P <addr> <mask> <value>
(poll until (read & mask) == value). tools/kv260/play.py replays a transcript against the real register file
over /dev/mem, so a board run needs no model: the expected pixel values are in the transcript."""
import os
_REG_LOG = open(os.environ["TS_REG_LOG"], "w") if os.environ.get("TS_REG_LOG") else None
_GAP = int(os.environ.get("TS_AXIL_GAP", "0"))   # idle clocks after every write / read: a host over PCIe is ~1000 cycles per access   # TS_REG_LOG: every write as "w addr value" (tb/host_regs.py logs the board run the same way)
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer


class AxiLite:
    def __init__(self, dut, clk, prefix="s_axi_", transcript=None, poll_gap=32):
        self.dut = dut
        self.clk = clk
        self.p = prefix
        self.t = transcript                     # list or None
        self.poll_gap = poll_gap
        for n in ("awvalid", "wvalid", "bready", "arvalid", "rready"):
            self.sig(n).value = 0

    def sig(self, name):
        return getattr(self.dut, self.p + name)

    async def write(self, addr, data, strb=0xF):
        if _REG_LOG: _REG_LOG.write(f"w {addr:03x} {data & 0xFFFFFFFF:08x}\n")
        s = self.sig
        s("awaddr").value = addr
        s("awvalid").value = 1
        s("wdata").value = data & 0xFFFFFFFF
        s("wstrb").value = strb
        s("wvalid").value = 1
        aw = w = False
        while not (aw and w):
            await ReadOnly()
            aw_now = int(s("awready").value) == 1 and not aw
            w_now = int(s("wready").value) == 1 and not w
            await RisingEdge(self.clk)
            await Timer(1, "ns")
            if aw_now:
                aw = True
                s("awvalid").value = 0
            if w_now:
                w = True
                s("wvalid").value = 0
        s("bready").value = 1
        while True:
            await ReadOnly()
            ok = int(s("bvalid").value) == 1
            await RisingEdge(self.clk)
            if ok:
                break
        await Timer(1, "ns")
        s("bready").value = 0
        if self.t is not None:
            self.t.append(("W", addr, data & 0xFFFFFFFF))
        if _GAP: await ClockCycles(self.clk, _GAP)

    async def read(self, addr, expect=None, record=True):
        s = self.sig
        s("araddr").value = addr
        s("arvalid").value = 1
        while True:
            await ReadOnly()
            ok = int(s("arready").value) == 1
            await RisingEdge(self.clk)
            if ok:
                break
        await Timer(1, "ns")
        s("arvalid").value = 0
        s("rready").value = 1
        while True:
            await ReadOnly()
            ok = int(s("rvalid").value) == 1
            v = int(s("rdata").value) if ok else None
            await RisingEdge(self.clk)
            if ok:
                break
        await Timer(1, "ns")
        s("rready").value = 0
        if self.t is not None and record:
            self.t.append(("R", addr, v if expect is None else expect))
        if _GAP: await ClockCycles(self.clk, _GAP)
        return v

    async def poll(self, addr, mask, value, limit=10_000_000, tag=""):
        """Read until (v & mask) == value; returns the last value read. Recorded as one P line."""
        cycles = 0
        while True:
            v = await self.read(addr, record=False)
            if (v & mask) == value:
                break
            await ClockCycles(self.clk, self.poll_gap)
            cycles += self.poll_gap
            assert cycles < limit, f"{tag}: poll 0x{addr:03x} & 0x{mask:x} != 0x{value:x} (last 0x{v:08x}) after {cycles} cycles"
        if self.t is not None:
            self.t.append(("P", addr, mask, value))
        return v


def write_transcript(path, ops):
    with open(path, "w") as fh:
        for op in ops:
            fh.write(" ".join([op[0]] + [f"{x:x}" for x in op[1:]]) + "\n")
