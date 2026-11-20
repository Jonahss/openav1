"""ctypes binding of tools/host/hostlib.c (libhostregs.so next to this file, built by build_hostlib.sh): the bulk
register traffic of the board driver in C. Used by tb/host_regs.HostDecoder as its `fast` backend."""
import array
import ctypes
from pathlib import Path

HERE = Path(__file__).resolve().parent


class HostLib:
    def __init__(self, dev, base=0):
        """dev: the device node to map (the XDMA user BAR, /dev/xdma0_user); base: the offset of the register file
        in it (0 in fpga/u200/build.tcl)."""
        self.lib = ctypes.CDLL(str(HERE / "libhostregs.so"))
        lib = self.lib
        lib.hl_open_dev.argtypes = [ctypes.c_char_p, ctypes.c_uint64]
        lib.hl_wr.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
        lib.hl_rd.argtypes = [ctypes.c_uint32]
        lib.hl_rd.restype = ctypes.c_uint32
        lib.hl_write_seq.argtypes = [ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32), ctypes.c_int]
        lib.hl_push.argtypes = [ctypes.c_char_p, ctypes.c_long, ctypes.c_double]
        lib.hl_push.restype = ctypes.c_long
        lib.hl_poll.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_double]
        lib.hl_poll.restype = ctypes.c_uint32
        lib.hl_read_plane.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_uint16)]
        rc = lib.hl_open_dev(str(dev).encode(), base)
        if rc:
            raise OSError(f"hostlib: cannot map 0x{base:08x} of {dev} (rc {rc})")

    def wr(self, a, v):
        self.lib.hl_wr(a, v & 0xFFFFFFFF)

    def rd(self, a):
        return self.lib.hl_rd(a)

    @staticmethod
    def seq_arrays(addrs, vals):
        """(address, value) lists as the C arrays write_seq takes; build once per repeated sequence (the CDF
        default rows of a base_q_idx: 11 k words), since the marshalling costs more than the bus."""
        return array.array("I", addrs), array.array("I", [v & 0xFFFFFFFF for v in vals])

    def write_seq(self, addrs, vals):
        if not isinstance(addrs, array.array):
            addrs, vals = self.seq_arrays(addrs, vals)
        n = len(addrs)
        A = (ctypes.c_uint32 * n).from_buffer(addrs)
        V = (ctypes.c_uint32 * n).from_buffer(vals)
        self.lib.hl_write_seq(A, V, n)

    def push(self, data, timeout):
        return self.lib.hl_push(bytes(data), len(data), timeout)

    def poll(self, a, mask, value, timeout):
        v = self.lib.hl_poll(a, mask, value, timeout)
        return None if v == 0xFFFFFFFF else v

    def read_plane(self, plane, W, H, bd8):
        buf = (ctypes.c_uint16 * (W * H))()
        if self.lib.hl_read_plane(plane, W, H, 1 if bd8 else 0, buf):
            raise OSError("hostlib: HADDR never read back")
        flat = memoryview(buf).cast("B").cast("H").tolist()      # list(buf) on a ctypes array is 25x slower
        return [flat[y * W:(y + 1) * W] for y in range(H)]
