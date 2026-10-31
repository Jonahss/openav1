#!/usr/bin/env python3
"""Decode an AV1 stream on the KV260 with the openav1 bitstream: stream file in, raw picture out. No model, no
transcript: the OBU parser runs on the PS, and tb/host_regs.py drives the decoder's register file over /dev/mem
exactly as the simulation test does.
    sudo tools/kv260/decode.py streams/synth/natural_lf_only_8bit.ivf -o out.yuv [--ref refout/natural_lf_only_8bit.yuv]
    python tools/kv260/decode.py clip.ivf --xdma 0 --inter --y4m clip.y4m      # the U200: a playable file (fpga/u200/stream_to_video.sh)
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

    def show(self, lib, hdr, planes_layout, flip=False):
        return show_on(lib, hdr, planes_layout, lambda pa: self.ptr(self.stage_off + pa - self.base), self, flip)


def show_on(lib, hdr, planes_layout, src, screen, flip=False):
    """8-bit 4:2:0 pictures onto the screen (an FbStage), centred (or the top-left corner when larger); src(pa)
    gives the pointer to the staged picture at physical address pa; flip turns the picture by 180 degrees for a
    panel mounted upside down (the console has fbcon=rotate:2 for the same)."""
    if hdr.BitDepth != 8 or hdr.NumPlanes != 3 or not (hdr.subsampling_x and hdr.subsampling_y) or screen.bpp != 16:
        return False
    (yb, ys, W, H), (ub, cs, _, _), (vb, _, _, _) = planes_layout
    w, h = min(W, screen.xres), min(H, screen.yres)
    x0, y0 = (screen.xres - w) // 2, (screen.yres - h) // 2
    lib.yuv420_rgb565(src(yb), src(ub), src(vb), w, h, ys, cs, screen.ptr(0), screen.line_length // 2, x0, y0, flip)
    return True


class MemStage:
    """A reserved physical region as the DMA target (memory the kernel was told to leave alone, e.g. mem=3584M on
    the KV260 leaves 0x8_6000_0000 .. 0x8_7FFF_FFFF): mapped through /dev/mem, which the kernel allows for pages
    that are not System RAM. Big enough for full-size pictures, unlike the framebuffer's spare lines."""
    def __init__(self, base, size):
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        self.base, self.stage_off, self.stage_len = base, 0, size
        self.screen = None                                     # an FbStage, opened by show()

    def read(self, off, n):
        return bytes(self.m[off:off + n])

    def ptr(self, off):
        return ctypes.addressof(ctypes.c_char.from_buffer(self.m, off))

    def show(self, lib, hdr, planes_layout, flip=False):
        """The picture is in the reserved region; the screen is still the framebuffer, opened on first use."""
        if self.screen is None:
            try:
                self.screen = FbStage()
            except OSError as e:
                print(f"no screen: {e}")
                self.screen = False
        if not self.screen:
            return False
        return show_on(lib, hdr, planes_layout, lambda pa: self.ptr(pa - self.base), self.screen, flip)


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
    ap.add_argument("--y4m", help="write every shown frame as YUV4MPEG2 (constant picture size): ffplay x.y4m, or ffmpeg -i x.y4m out.mp4")
    ap.add_argument("--fps", help="frame rate written into the .y4m header (default: the IVF header's rate/scale, else 30)")
    ap.add_argument("--base", type=lambda x: int(x, 0), default=0xA0000000)
    ap.add_argument("--stage", choices=("lf", "recon"), default="lf")
    ap.add_argument("--frames", type=int, help="stop after this many coded frames")
    ap.add_argument("--single", action="store_true", help="read the picture sample by sample (HDATA) instead of packed")
    ap.add_argument("--slow", action="store_true", help="pure Python register traffic (no libhostregs.so)")
    ap.add_argument("--dma", action="store_true", help="the picture through the DMA engine into the framebuffer's spare lines")
    ap.add_argument("--show", action="store_true", help="with --dma / --dma-base: also put each picture on the screen (8-bit 4:2:0, RGB565 fb0)")
    ap.add_argument("--flip", action="store_true", help="with --show: the picture turned by 180 degrees (panel mounted upside down)")
    ap.add_argument("--pipe", action="store_true", help="deblocking under the frame sequencer, overlapped with the tiles")
    ap.add_argument("--window", action="store_true", help="window build (FRAME_WIN > 0): rows flushed to DDR as they finish; implies --dma --pipe")
    ap.add_argument("--inter", action="store_true", help="RING build with inter: every stage under the sequencer, the model decoding alongside for the "
                    "frame sequence, references = the flushed pictures in a pool of --pool frame buffers in the DMA region; implies --window")
    ap.add_argument("--pool", type=int, default=10, help="with --inter: frame buffers carved from the DMA region (at least 9: 8 reference slots + the frame being decoded; 8 stalled the 1080p clip at frame 8, 2026-10-30)")
    ap.add_argument("--card-state", action="store_true", help="with --inter: no model on the host; the frame-to-frame state comes back from the card "
                    "(the sideband areas + CDF read-back, docs/frame-state.md stage A); every frame must fit the image")
    ap.add_argument("--dma-base", type=lambda x: int(x, 0), help="DMA target: a reserved physical region (with --dma-size) instead of the framebuffer's spare lines")
    ap.add_argument("--dma-size", type=lambda x: int(x, 0), default=256 << 20)
    ap.add_argument("--xdma", type=int, metavar="N", help="Alveo U200 host path: registers through /dev/xdmaN_user, pictures in the card's DDR4 "
                    "through /dev/xdmaN_c2h_0 (--dma-base is then a card address, default 0x10000000); implies --dma")
    ap.add_argument("--max-bad", type=int, default=8)
    ap.add_argument("--poll-timeout", type=float, default=30.0)
    args = ap.parse_args()
    if args.inter:
        args.window = True
    if args.window:
        args.dma = True; args.pipe = True
    if args.xdma is not None:
        args.dma = True
        if args.dma_base is None:
            args.dma_base = 0x1000_0000

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
    y4m = open(args.y4m, "wb") if args.y4m else None
    y4m_hdr = None                               # the first shown frame's (width, height, colour space) once written
    fps = args.fps
    if fps is None and args.stream.endswith(".ivf"):
        rate, scale = int.from_bytes(data[16:20], "little"), int.from_bytes(data[20:24], "little")
        fps = f"{rate}:{scale}" if rate and scale else None
    fps = fps or "30"
    if ":" not in fps:
        fps = f"{fps}:1"
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

    fast = None
    have_lib = not args.slow and not args.single and (Path(__file__).resolve().parent / "libhostregs.so").exists()
    if args.xdma is not None:
        from xdma import XdmaRegs, XdmaStage
        regs = XdmaRegs(args.xdma)
        if have_lib:
            from hostlib import HostLib
            fast = HostLib(0, dev=regs.path)
    else:
        regs = Regs(args.base)
        if have_lib:
            from hostlib import HostLib
            fast = HostLib(args.base)
    host = HostDecoder(regs, poll_timeout=args.poll_timeout, fast=fast)
    stage = None
    if args.xdma is not None:
        stage = XdmaStage(args.xdma, args.dma_base, args.dma_size)
        print(f"DMA target: card DDR4, {stage.stage_len} bytes at 0x{stage.base:x} (/dev/xdma{args.xdma}_c2h_0)")
    elif args.dma and args.dma_base is not None:
        stage = MemStage(args.dma_base, args.dma_size)
        print(f"DMA target: reserved memory, {stage.stage_len} bytes at 0x{stage.base:x}")
    elif args.dma:
        stage = FbStage()
        print(f"framebuffer {stage.xres}x{stage.yres} ({stage.bpp} bpp) at 0x{stage.smem_start:x}; staging {stage.stage_len} bytes at 0x{stage.base:x}")
    cfg = host.identify()
    where = regs.path if args.xdma is not None else f"0x{args.base:08x}"
    print(f"decoder at {where} ({'libhostregs.so' if fast else 'python'} traffic): FBX {cfg['fbx']} FBY {cfg['fby']} lanes {cfg['lanes']} uram {cfg['uram']} sb128 {int(cfg['sb128'])}; "
          f"{len(d.frames)} coded frames, {len(shown)} shown")
    n_bad_frames = 0
    t_all = time.monotonic()
    if args.inter and stage is None:
        sys.exit("--inter needs a DMA region (--dma-base / --xdma) for the frame-buffer pool")

    def frames():
        if args.inter:
            for fi, hdr, planes, mp, t in host.decode_frames_inter(d, (stage.base, stage.stage_len, stage.read, stage.write), name=Path(args.stream).name, card_state=args.card_state,
                                                                   pool=args.pool, want_planes=bool(out or ref)):
                t["lf"] = t.get("filters", 0.0)
                t["tiles"] = t.get("tiles", 0.0) + t.get("model", 0.0) + t.get("setup", 0.0)
                t["read"] = t.get("read", 0.0)
                if t.get("model_only"):
                    t["model_planes"] = mp
                yield fi, hdr, planes, t
        else:
            yield from host.decode_frames(d, stage=args.stage, name=Path(args.stream).name, packed=not args.single,
                                          dma=(stage.base, stage.read) if stage else None, pipe=args.pipe,
                                          want_planes=bool(out or ref) or stage is None, window=args.window)
    try:
        for fi, hdr, planes, t in frames():
            if stage is not None and args.show and fast is not None:
                from host_regs import dma_layout
                stage.show(fast, hdr, dma_layout(hdr, host.last_target if args.inter else stage.base)[0], flip=args.flip)
            if args.frames is not None and fi >= args.frames:
                break
            W, H = plane_size(hdr, 0)
            fmt = "mono" if hdr.NumPlanes == 1 else ("444" if not hdr.subsampling_x else ("420" if hdr.subsampling_y else "422"))
            line = (f"frame {fi}: {W}x{H} {fmt} bd{hdr.BitDepth}, {len(d.frames[fi])} tile(s): "
                    f"tiles {t['tiles']:.2f}s, lf {t['lf']:.2f}s, read-back {t['read']:.2f}s")
            if t.get("model_only"):
                # the image could not take this frame: the model decoded it and the card got its picture as a reference
                line = f"frame {fi}: {W}x{H} {fmt} bd{hdr.BitDepth}: model only ({t['model_only']})"
                if fi in ref_frames:
                    lo, hi = ref_frames[fi]
                    mbad = compare(ref[lo:hi], hdr, t["model_planes"])
                    line += f": model {'differs from' if mbad else 'identical to'} the reference" + (f" ({len(mbad)} samples)" if mbad else "")
                print(line, flush=True)
                continue
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
            if t.get("fs"):
                f_ = t["fs"]
                line += (f"\n        state -> card DDR: MV words at 0x{f_['mv_base']:x} ({f_['mv_bytes']} bytes, {f_['refs']} cells with a saved reference), "
                         f"segment ids at 0x{f_['seg_base']:x} ({f_['seg_bytes']} bytes); {f_['cdf_rows']} CDF rows read back")
            print(line, flush=True)
            if out is not None and fi in shown:
                write_planes(out, hdr, planes)
            if y4m is not None and fi in shown:
                w, h = plane_size(hdr, 0)
                cs = ("mono" if hdr.NumPlanes == 1 else "420jpeg" if hdr.subsampling_x and hdr.subsampling_y
                      else "422" if hdr.subsampling_x else "444") + (f"p{hdr.BitDepth}" if hdr.BitDepth > 8 else "")
                if cs == "monop10": cs = "mono10"
                if y4m_hdr is None:
                    y4m_hdr = (w, h, cs)
                    y4m.write(f"YUV4MPEG2 W{w} H{h} F{fps} Ip A1:1 C{cs}\n".encode())
                if (w, h, cs) != y4m_hdr:
                    print(f"warning: frame {fi} is {w}x{h} {cs}, the .y4m is {y4m_hdr[0]}x{y4m_hdr[1]} {y4m_hdr[2]}: frame skipped")
                else:
                    y4m.write(b"FRAME\n")
                    write_planes(y4m, hdr, planes)
    except HostError as e:
        print(f"error: {e}")
        sys.exit(2)
    finally:
        if out is not None:
            out.close()
        if y4m is not None:
            y4m.close()
    s = host.stats
    dt_all = time.monotonic() - t_all
    print(f"{s['frames']} frames, {s['tiles']} tiles, {s['bytes']} stream bytes, {s['pixels']} samples read back, "
          f"{s['writes']} register writes + {s['reads']} reads in {dt_all:.1f}s ({s['frames'] / dt_all:.1f} fps)"
          + (f"; {n_bad_frames} frame(s) differ from the reference" if ref is not None else ""))
    sys.exit(1 if n_bad_frames else 0)


if __name__ == "__main__":
    main()
