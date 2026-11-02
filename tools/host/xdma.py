"""The Alveo U200 host side (fpga/u200): the decoder's register file through the XDMA driver's user BAR and the
card's DDR4 through its DMA channels, with the interfaces decode.py's Regs / MemStage have, so HostDecoder runs
unchanged. Device nodes (AMD dma_ip_drivers, `modprobe xdma`): /dev/xdma<N>_user is BAR0 (AXI-Lite: the
register file at 0x0, the DDR4 controller's ECC registers at 0x1_0000), /dev/xdma<N>_h2c_0 / _c2h_0 move bytes
to / from the card's DDR4 (a pread / pwrite at offset A is a DMA to card address A). Every register access is one
32-bit PCIe transaction (ctypes c_uint32 on the mapping: the same one-store rule as /dev/mem on the KV260)."""
import ctypes
import mmap
import os


class XdmaRegs:
    """32-bit register access through the mapped user BAR."""
    def __init__(self, card=0, base=0, span=0x1000):
        self.path = f"/dev/xdma{card}_user"
        self.fd = os.open(self.path, os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, span, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        self.regs = {}

    def reg(self, a):
        r = self.regs.get(a)
        if r is None:
            r = self.regs[a] = ctypes.c_uint32.from_buffer(self.m, a)
        return r

    def rd(self, a):
        return self.reg(a).value

    def wr(self, a, v):
        self.reg(a).value = v & 0xFFFFFFFF


class XdmaStage:
    """A region of the card's DDR4 as the picture DMA target: the decoder writes it through its AXI master (the
    flush / DMA engine, addresses programmed by HostDecoder from .base), the host reads it back over the C2H
    channel. .read(off, n) is relative to .base, as MemStage's. write() loads bytes (reference pictures, test
    patterns) over H2C."""
    def __init__(self, card=0, base=0x1000_0000, size=256 << 20):
        self.c2h = os.open(f"/dev/xdma{card}_c2h_0", os.O_RDONLY)
        self.h2c = os.open(f"/dev/xdma{card}_h2c_0", os.O_WRONLY)
        self.base, self.stage_off, self.stage_len = base, 0, size

    def read(self, off, n):
        out = bytearray()
        pos = self.base + off
        while len(out) < n:                       # the driver may return short reads at its transfer limit
            chunk = os.pread(self.c2h, n - len(out), pos)
            if not chunk:
                raise OSError(f"xdma: short read at 0x{pos:x} ({len(out)} of {n} bytes)")
            out += chunk
            pos += len(chunk)
        return bytes(out)

    def write(self, off, data, chunk=1 << 20):
        """Host to card; in 1 MB pieces (a 3 MB model-decoded picture in one pwrite returned EIO on 2026-10-27)."""
        pos, view = self.base + off, memoryview(data)
        while view:
            k = os.pwrite(self.h2c, view[:chunk], pos)
            if k <= 0:
                raise OSError(f"xdma: short write at 0x{pos:x}")
            view, pos = view[k:], pos + k

    def show(self, lib, hdr, planes_layout, flip=False):
        return False                               # no screen on the host path; pictures go to files
