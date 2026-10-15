#!/usr/bin/env python3
"""Decode an AV1 stream on the KV260 with the openav1 bitstream: stream file in, raw picture out. No model, no
transcript: the OBU parser runs on the PS, and tb/host_regs.py drives the decoder's register file over /dev/mem
exactly as the simulation test does.
    sudo tools/kv260/decode.py streams/synth/natural_lf_only_8bit.ivf -o out.yuv [--ref refout/natural_lf_only_8bit.yuv]
Output is planar, visible size, 8-bit as bytes and >8-bit as 16-bit little-endian (dav1d's `-o x.yuv` layout), shown
frames in output order; --ref compares every sample against such a file and reports the mismatches. --stage recon
skips the loop filter (and then the picture is the reconstruction, not dav1d's). Exit status 0 when every frame
decoded and (with --ref) matched.
"""
import argparse
import ctypes
import mmap
import os
import struct
import sys
import time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tb"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import obu_parser as op                                     # noqa: E402
from host_regs import HostDecoder, HostError, plane_size    # noqa: E402


class FbStage:
    """The DisplayPort framebuffer (/dev/fb0, xlnxdrmfb: a physically contiguous, uncached CMA buffer) as the DMA
    target: the visible screen is its first yres lines, the rest of its virtual height is free and holds the
    decoded picture. The PS reads the picture back through the same mapping, so no /dev/mem is needed (the
    kernel refuses /dev/mem on RAM anyway)."""
    FBIOGET_VSCREENINFO, FBIOGET_FSCREENINFO = 0x4600, 0x4602

    def __init__(self, path="/dev/fb0"):
        import fcntl
        self.fd = os.open(path, os.O_RDWR)
        fix = bytearray(72)
        fcntl.ioctl(self.fd, self.FBIOGET_FSCREENINFO, fix)
        self.smem_start, self.smem_len = struct.unpack("QI", fix[16:28])
        self.line_length = struct.unpack("I", fix[48:52])[0]
        var = bytearray(160)
        fcntl.ioctl(self.fd, self.FBIOGET_VSCREENINFO, var)
        self.xres, self.yres, self.xres_v, self.yres_v, _, _, self.bpp = struct.unpack("7I", var[0:28])
        if not self.smem_start:
            raise OSError("framebuffer reports no physical address")
        self.m = mmap.mmap(self.fd, self.smem_len, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        self.screen_bytes = self.line_length * self.yres
        self.stage_off = self.screen_bytes
        self.stage_len = self.smem_len - self.stage_off
        self.base = self.smem_start + self.stage_off          # physical address of the staging area

    def read(self, off, n):
        return bytes(self.m[self.stage_off + off:self.stage_off + off + n])

    def ptr(self, off):
        return ctypes.addressof(ctypes.c_char.from_buffer(self.m, off))


class MemStage:
    """A reserved physical region as the DMA target (memory the kernel was told to leave alone, e.g. mem=3584M on
    the KV260 leaves 0x8_6000_0000 .. 0x8_7FFF_FFFF): mapped through /dev/mem, which the kernel allows for pages
    that are not System RAM. Big enough for full-size pictures, unlike the framebuffer's spare lines."""
    def __init__(self, base, size):
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        self.base, self.stage_off, self.stage_len = base, 0, size
        self.xres = self.yres = self.bpp = 0                   # no screen

    def read(self, off, n):
        return bytes(self.m[off:off + n])

    def ptr(self, off):
        return ctypes.addressof(ctypes.c_char.from_buffer(self.m, off))

    def show(self, lib, hdr, planes_layout):
        return False

    def show(self, lib, hdr, planes_layout):
        """8-bit 4:2:0 pictures onto the screen, centred (or the top-left corner when larger)."""
        if hdr.BitDepth != 8 or hdr.NumPlanes != 3 or not (hdr.subsampling_x and hdr.subsampling_y) or self.bpp != 16:
            return False
        (yb, ys, W, H), (ub, cs, _, _), (vb, _, _, _) = planes_layout
        w, h = min(W, self.xres), min(H, self.yres)
        x0, y0 = (self.xres - w) // 2, (self.yres - h) // 2
        lib.yuv420_rgb565(self.ptr(self.stage_off + yb - self.base), self.ptr(self.stage_off + ub - self.base),
                          self.ptr(self.stage_off + vb - self.base), w, h, ys, cs, self.ptr(0), self.line_length // 2, x0, y0)
        return True


class Regs:
    """32-bit register access through /dev/mem; every access ONE aligned 32-bit bus transaction (ctypes'
    c_uint32: struct.pack_into and slice stores split a word into 2-3 transfers, first board run 2026-10-14)."""
    def __init__(self, base, span=0x1000):
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
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


def frame_bytes(hdr):
    bps = 1 if hdr.BitDepth == 8 else 2
    return sum(w * h for w, h in (plane_size(hdr, p) for p in range(hdr.NumPlanes))) * bps


def write_planes(fh, hdr, planes):
    for p in range(hdr.NumPlanes):
        W, _ = plane_size(hdr, p)
        for row in planes[p]:
            fh.write(bytes(row) if hdr.BitDepth == 8 else struct.pack("<%dH" % W, *row))


def compare(ref, hdr, planes):
    """Mismatches of planes against the reference frame bytes: list of (plane, y, x, got, expected)."""
    bad = []
    bps = 1 if hdr.BitDepth == 8 else 2
    off = 0
    for p in range(hdr.NumPlanes):
        W, H = plane_size(hdr, p)
        for y in range(H):
            row = ref[off:off + W * bps]
            exp = row if bps == 1 else struct.unpack("<%dH" % W, row)
            got = planes[p][y]
            if (bytes(got) if bps == 1 else tuple(got)) != exp:
                for x in range(W):
                    if got[x] != exp[x]:
                        bad.append((p, y, x, got[x], exp[x]))
            off += W * bps
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("stream", help=".ivf, .obu (Annex B) or raw OBU stream")
    ap.add_argument("-o", "--out", help="raw planar output (every shown frame, output order)")
    ap.add_argument("--ref", help="reference .yuv (dav1d -o layout, same frames) to compare against")
    ap.add_argument("--base", type=lambda x: int(x, 0), default=0xA0000000)
    ap.add_argument("--stage", choices=("lf", "recon"), default="lf")
    ap.add_argument("--frames", type=int, help="stop after this many coded frames")
    ap.add_argument("--single", action="store_true", help="read the picture sample by sample (HDATA) instead of packed")
    ap.add_argument("--slow", action="store_true", help="pure Python register traffic (no libhostregs.so)")
    ap.add_argument("--dma", action="store_true", help="the picture through the DMA engine into the framebuffer's spare lines")
    ap.add_argument("--show", action="store_true", help="with --dma: also put each picture on the screen (8-bit 4:2:0, RGB565 fb)")
    ap.add_argument("--pipe", action="store_true", help="deblocking under the frame sequencer, overlapped with the tiles")
    ap.add_argument("--window", action="store_true", help="window build (FRAME_WIN > 0): rows flushed to DDR as they finish; implies --dma --pipe")
    ap.add_argument("--dma-base", type=lambda x: int(x, 0), help="DMA target: a reserved physical region (with --dma-size) instead of the framebuffer's spare lines")
    ap.add_argument("--dma-size", type=lambda x: int(x, 0), default=256 << 20)
    ap.add_argument("--max-bad", type=int, default=8)
    ap.add_argument("--poll-timeout", type=float, default=30.0)
    args = ap.parse_args()
    if args.window:
        args.dma = True; args.pipe = True

    d = op.Decoder()
    data = Path(args.stream).read_bytes()
    if args.stream.endswith(".ivf"):
        d.feed_ivf(data)
    elif args.stream.endswith(".obu"):
        d.feed_annexb(data)
    else:
        d.feed_temporal_unit(data)
    shown = d.shown_frames()                     # coded-frame index of each output picture, in output order
    out = open(args.out, "wb") if args.out else None
    ref = Path(args.ref).read_bytes() if args.ref else None
    ref_off = 0
    ref_frames = {}                              # coded frame -> reference byte range, in output order
    if ref is not None:
        for k in shown:
            h_k = d.tiles[d.frames[k][0]][0]
            ref_frames[k] = (ref_off, ref_off + frame_bytes(h_k))
            ref_off += frame_bytes(h_k)
        if ref_off != len(ref):
            print(f"warning: reference holds {len(ref)} bytes, the stream's shown frames need {ref_off}")

    regs = Regs(args.base)
    fast = None
    if not args.slow and not args.single and (Path(__file__).resolve().parent / "libhostregs.so").exists():
        from hostlib import HostLib
        fast = HostLib(args.base)
    host = HostDecoder(regs, poll_timeout=args.poll_timeout, fast=fast)
    stage = None
    if args.dma and args.dma_base is not None:
        stage = MemStage(args.dma_base, args.dma_size)
        print(f"DMA target: reserved memory, {stage.stage_len} bytes at 0x{stage.base:x}")
    elif args.dma:
        stage = FbStage()
        print(f"framebuffer {stage.xres}x{stage.yres} ({stage.bpp} bpp) at 0x{stage.smem_start:x}; staging {stage.stage_len} bytes at 0x{stage.base:x}")
    cfg = host.identify()
    print(f"decoder at 0x{args.base:08x} ({'libhostregs.so' if fast else 'python'} traffic): FBX {cfg['fbx']} FBY {cfg['fby']} lanes {cfg['lanes']} uram {cfg['uram']}; "
          f"{len(d.frames)} coded frames, {len(shown)} shown")
    n_bad_frames = 0
    t_all = time.monotonic()
    try:
        for fi, hdr, planes, t in host.decode_frames(d, stage=args.stage, name=Path(args.stream).name, packed=not args.single,
                                                     dma=(stage.base, stage.read) if stage else None, pipe=args.pipe,
                                                     want_planes=bool(out or ref) or stage is None, window=args.window):
            if stage is not None and args.show and fast is not None:
                from host_regs import dma_layout
                stage.show(fast, hdr, dma_layout(hdr, stage.base)[0])
            if args.frames is not None and fi >= args.frames:
                break
            W, H = plane_size(hdr, 0)
            fmt = "mono" if hdr.NumPlanes == 1 else ("444" if not hdr.subsampling_x else ("420" if hdr.subsampling_y else "422"))
            line = (f"frame {fi}: {W}x{H} {fmt} bd{hdr.BitDepth}, {len(d.frames[fi])} tile(s): "
                    f"tiles {t['tiles']:.2f}s, lf {t['lf']:.2f}s, read-back {t['read']:.2f}s")
            if fi in ref_frames:
                lo, hi = ref_frames[fi]
                bad = compare(ref[lo:hi], hdr, planes)
                if bad:
                    n_bad_frames += 1
                    line += f": {len(bad)} samples differ (plane, y, x, board, reference): {bad[:args.max_bad]}"
                else:
                    line += ": identical to the reference"
            elif ref is not None:
                line += " (not shown: no reference picture)"
            print(line, flush=True)
            if out is not None and fi in shown:
                write_planes(out, hdr, planes)
    except HostError as e:
        print(f"error: {e}")
        sys.exit(2)
    finally:
        if out is not None:
            out.close()
    s = host.stats
    print(f"{s['frames']} frames, {s['tiles']} tiles, {s['bytes']} stream bytes, {s['pixels']} samples read back, "
          f"{s['writes']} register writes + {s['reads']} reads in {time.monotonic() - t_all:.1f}s"
          + (f"; {n_bad_frames} frame(s) differ from the reference" if ref is not None else ""))
    sys.exit(1 if n_bad_frames else 0)


if __name__ == "__main__":
    main()
