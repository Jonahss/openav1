"""A minimal AXI4 slave memory for cocotb: the write channels of the picture DMA (INCR bursts, byte strobes) into
a dict of bytes keyed by address, the read channels of the reference fetch (run_read: INCR bursts of 8-byte beats
out of the same dict), and a read side for the testbench. Independent handshakes with random ready / valid gaps so
the masters' back-pressure handling is exercised; one burst outstanding per direction at a time."""
import random
from cocotb.triggers import ReadOnly, RisingEdge, Timer


class AxiMem:
    import os as _os
    fill = int(_os.environ.get("TS_MEM_FILL", "0"), 0) & 0xFF   # unwritten bytes read as this (TS_MEM_FILL=0x5a: poison, 2026-10-31)
    def __init__(self, dut, clk, prefix="m_axi_", seed=1, stall=0.3):
        self.dut = dut
        self.clk = clk
        self.p = prefix
        self.mem = {}                       # byte address -> value
        self.bursts = 0
        self.beats = 0
        self.errors = []
        self.rng = random.Random(seed)
        self.stall = stall
        # a master may have only the write or only the read channels (ref_fetch: read only)
        for name in ("awready", "wready", "bvalid", "bresp", "arready", "rvalid", "rdata", "rresp", "rlast"):
            try:
                self.sig(name).value = 0
            except AttributeError:
                pass

    def sig(self, name):
        return getattr(self.dut, self.p + name)

    async def run(self):
        s = self.sig
        while True:
            # address
            while self.rng.random() < self.stall:
                await RisingEdge(self.clk)
            s("awready").value = 1
            while True:
                await ReadOnly()
                if int(s("awvalid").value) == 1:
                    addr = int(s("awaddr").value)
                    length = int(s("awlen").value) + 1
                    size = int(s("awsize").value)
                    burst = int(s("awburst").value)
                    break
                await RisingEdge(self.clk)
            await RisingEdge(self.clk)
            await Timer(1, "ns")
            s("awready").value = 0
            if size != 3 or burst != 1:
                self.errors.append(f"burst at 0x{addr:x}: size {size} burst {burst}")
            if (addr & 0xFFF) + 8 * length > 0x1000:
                self.errors.append(f"burst at 0x{addr:x} of {length} beats crosses 4 KB")
            # data
            a = addr & ~7
            for i in range(length):
                while self.rng.random() < self.stall:
                    await RisingEdge(self.clk)
                s("wready").value = 1
                while True:
                    await ReadOnly()
                    if int(s("wvalid").value) == 1:
                        data = int(s("wdata").value)
                        strb = int(s("wstrb").value)
                        last = int(s("wlast").value)
                        break
                    await RisingEdge(self.clk)
                await RisingEdge(self.clk)
                await Timer(1, "ns")
                s("wready").value = 0
                if last != (1 if i == length - 1 else 0):
                    self.errors.append(f"burst at 0x{addr:x}: wlast on beat {i} of {length}")
                for b in range(8):
                    if strb >> b & 1:
                        self.mem[a + b] = (data >> (8 * b)) & 0xFF
                a += 8
                self.beats += 1
            # response
            while self.rng.random() < self.stall:
                await RisingEdge(self.clk)
            s("bvalid").value = 1
            s("bresp").value = 0
            while True:
                await ReadOnly()
                if int(s("bready").value) == 1:
                    break
                await RisingEdge(self.clk)
            await RisingEdge(self.clk)
            await Timer(1, "ns")
            s("bvalid").value = 0
            self.bursts += 1

    async def run_read(self):
        """Serve the read channels: one AR burst at a time, 8-byte INCR beats from self.mem (unwritten bytes read 0)."""
        s = self.sig
        while True:
            while self.rng.random() < self.stall:
                await RisingEdge(self.clk)
            s("arready").value = 1
            while True:
                await ReadOnly()
                if int(s("arvalid").value) == 1:
                    addr = int(s("araddr").value)
                    length = int(s("arlen").value) + 1
                    size = int(s("arsize").value)
                    burst = int(s("arburst").value)
                    break
                await RisingEdge(self.clk)
            await RisingEdge(self.clk)
            await Timer(1, "ns")
            s("arready").value = 0
            if size != 3 or burst != 1:
                self.errors.append(f"read burst at 0x{addr:x}: size {size} burst {burst}")
            if addr & 7:
                self.errors.append(f"read burst at 0x{addr:x}: unaligned")
            if (addr & 0xFFF) + 8 * length > 0x1000:
                self.errors.append(f"read burst at 0x{addr:x} of {length} beats crosses 4 KB")
            a = addr & ~7
            for i in range(length):
                while self.rng.random() < self.stall:
                    await RisingEdge(self.clk)
                data = 0
                for b in range(8):
                    data |= self._get(a + b) << (8 * b)
                s("rvalid").value = 1
                s("rdata").value = data
                s("rresp").value = 0
                s("rlast").value = 1 if i == length - 1 else 0
                while True:
                    await ReadOnly()
                    if int(s("rready").value) == 1:
                        break
                    await RisingEdge(self.clk)
                await RisingEdge(self.clk)
                await Timer(1, "ns")
                s("rvalid").value = 0
                s("rlast").value = 0
                a += 8
                self.read_beats = getattr(self, "read_beats", 0) + 1
            self.read_bursts = getattr(self, "read_bursts", 0) + 1

    def _get(self, a):
        """A byte for the read channel; with TS_MEM_FILL set, a read of a byte nobody wrote is logged (the first 24,
        then counted): the decoder reading memory the stream never produced (2026-10-31, test5466 after test5366)."""
        if a in self.mem:
            return self.mem[a]
        if self.fill:
            self.unwritten = getattr(self, "unwritten", 0) + 1
            if self.unwritten <= 24:
                print(f"[axi] UNWRITTEN read at 0x{a:010x} (#{self.unwritten})", flush=True)
            elif self.unwritten == 25:
                print("[axi] UNWRITTEN reads: more (counting only)", flush=True)
        return self.fill

    def read(self, addr, n):
        return bytes(self.mem.get(addr + i, self.fill) for i in range(n))

    def load(self, addr, data):
        """Place bytes in memory (the testbench's reference pictures)."""
        for i, v in enumerate(data):
            self.mem[addr + i] = v

    def written(self):
        return len(self.mem)
