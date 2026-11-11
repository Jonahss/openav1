"""The host's own screen (/dev/fb0, a 32-bit console framebuffer: nouveau on the P400 driving the 7-inch panel) for the
card path: decode.py --xdma --show puts every output picture on it, 1:1 from the top-left corner (no scaling; a
picture larger than the panel shows its top-left part). The pictures are the read-back planes decode.py already has
(rows of bytes for 8 bits, rows of 16-bit samples above), any bit depth, 4:2:0 / 4:4:4 / mono; BT.601 limited range,
as hostlib's hl_yuv420_rgb565 on the board. 2026-11-10."""
import fcntl, mmap, os, struct
import numpy as np

FBIOGET_VSCREENINFO, FBIOGET_FSCREENINFO = 0x4600, 0x4602


class Screen:
    def __init__(self, path="/dev/fb0"):
        self.fd = os.open(path, os.O_RDWR)
        fix = bytearray(80)
        fcntl.ioctl(self.fd, FBIOGET_FSCREENINFO, fix)
        self.smem_len = struct.unpack("I", fix[24:28])[0]
        self.line_length = struct.unpack("I", fix[48:52])[0]
        var = bytearray(160)
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, var)
        self.xres, self.yres, _, _, _, _, self.bpp = struct.unpack("7I", var[0:28])
        self.r_off, _, _, self.g_off, _, _, self.b_off = struct.unpack("7I", var[32:60])
        if self.bpp != 32:
            raise OSError(f"{path}: {self.bpp} bits per pixel, this screen path wants 32")
        self.m = mmap.mmap(self.fd, self.line_length * self.yres, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        self.px = np.frombuffer(self.m, dtype=np.uint32).reshape(self.yres, self.line_length // 4)
        self.last = None                                    # size of the last picture: a smaller one clears the screen first

    def _plane(self, rows, w, h, bd):
        """The top-left w x h of a plane as int32, scaled to 8 bits."""
        if bd == 8:
            a = np.stack([np.frombuffer(bytes(r), dtype=np.uint8, count=w) for r in rows[:h]]).astype(np.int32)
        else:
            a = np.stack([np.asarray(r[:w], dtype=np.uint16) for r in rows[:h]]).astype(np.int32) >> (bd - 8)
        return a

    def show(self, hdr, planes):
        if planes is None:
            return False
        W, H = len(planes[0][0]), len(planes[0])
        w, h = min(W, self.xres), min(H, self.yres)
        y = self._plane(planes[0], w, h, hdr.BitDepth)
        if hdr.NumPlanes == 1:
            dd = e = np.zeros_like(y)
        else:
            sx, sy = int(bool(hdr.subsampling_x)), int(bool(hdr.subsampling_y))
            cw, ch = (w + sx) >> sx, (h + sy) >> sy
            u = self._plane(planes[1], cw, ch, hdr.BitDepth)
            v = self._plane(planes[2], cw, ch, hdr.BitDepth)
            if sx: u, v = np.repeat(u, 2, axis=1)[:, :w], np.repeat(v, 2, axis=1)[:, :w]
            if sy: u, v = np.repeat(u, 2, axis=0)[:h], np.repeat(v, 2, axis=0)[:h]
            dd, e = u - 128, v - 128
        c = 298 * (y - 16)
        r = np.clip((c + 409 * e + 128) >> 8, 0, 255).astype(np.uint32)
        g = np.clip((c - 100 * dd - 208 * e + 128) >> 8, 0, 255).astype(np.uint32)
        b = np.clip((c + 516 * dd + 128) >> 8, 0, 255).astype(np.uint32)
        if self.last != (w, h):
            self.px[:, :self.xres] = 0
            self.last = (w, h)
        self.px[:h, :w] = (r << self.r_off) | (g << self.g_off) | (b << self.b_off)
        return True
