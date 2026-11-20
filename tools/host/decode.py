#!/usr/bin/env python3
"""Decode an AV1 stream on the Alveo U200 with the openav1 image: stream file in, pictures out. No model: the OBU
parser runs on the host and tb/host_regs.py drives the decoder's register file through the XDMA driver, exactly as
the simulation test does. One path: the inter decoder with the frame-to-frame state kept on the card
(docs/frame-state.md), register traffic through libhostregs.so (tools/host/build_hostlib.sh).
    python tools/host/decode.py clip.ivf --ref refout/clip.yuv      # every output picture against dav1d's
    python tools/host/decode.py clip.ivf --y4m clip.y4m             # a playable file (fpga/u200/stream_to_video.sh)
    python tools/host/decode.py clip.ivf --show                     # on the host's screen (bin/dec.sh)
Output is planar, visible size, 8-bit as bytes and >8-bit as 16-bit little-endian (dav1d's `-o x.yuv` layout), shown
frames in output order; --ref compares every sample against such a file and reports the mismatches. Exit status 0
when every frame decoded and (with --ref) matched.
"""
import argparse
import struct
import sys
import time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tb"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import obu_parser as op                                     # noqa: E402
from host_regs import HostDecoder, HostError, plane_size    # noqa: E402


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
    ap.add_argument("--frames", type=int, help="stop after this many coded frames")
    ap.add_argument("--card", type=int, default=0, metavar="N", help="which card: registers through /dev/xdmaN_user, pictures in the card's DDR4 through /dev/xdmaN_c2h_0")
    ap.add_argument("--pool", type=int, default=12, help="frame buffers carved from the card's DDR4 region (at least 9: 8 reference slots + the frame being decoded; 8 stalled the 1080p clip at frame 8, 2026-10-30)")
    ap.add_argument("--overlap", action="store_true", help="read each picture back while the card decodes the next frame (output lags one frame; --show shows the newest)")
    ap.add_argument("--show", action="store_true", help="every output picture on the host's /dev/fb0 (32 bpp, 1:1 from the top-left corner)")
    ap.add_argument("--max-bad", type=int, default=8)
    ap.add_argument("--poll-timeout", type=float, default=30.0)
    ap.add_argument("--perf-tiles", metavar="FILE", help="per tile (row): bytes, host seconds to the last byte pushed and to tile done, and the card's "
                    "PERF_SYN / PERF_REC cycle counters, as CSV; a summary line on stdout (is the byte push on the critical path?)")
    args = ap.parse_args()
    if not (Path(__file__).resolve().parent / "libhostregs.so").exists():
        sys.exit("tools/host/libhostregs.so is missing: run tools/host/build_hostlib.sh")

    d = op.Decoder()
    data = Path(args.stream).read_bytes()
    if args.stream.endswith(".ivf"):
        d.feed_ivf(data)
    elif args.stream.endswith(".obu"):
        d.feed_annexb(data)
    else:
        d.feed_temporal_unit(data)
    shown = d.shown_frames()                     # coded-frame index of each output picture, in output order (a coded frame
    import collections                           # shown twice, or shown later through show_existing_frame, appears where it is displayed)
    owed = collections.Counter(shown)            # displays still owed per coded frame
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
    ref_events = []                              # per output picture: its reference byte range (dav1d writes output order)
    if ref is not None:
        for k in shown:
            h_k = d.tiles[d.frames[k][0]][0]
            ref_events.append((ref_off, ref_off + frame_bytes(h_k)))
            ref_off += frame_bytes(h_k)
        if ref_off != len(ref):
            print(f"warning: reference holds {len(ref)} bytes, the stream's shown frames need {ref_off}")

    from hostlib import HostLib
    from xdma import XdmaRegs, XdmaStage
    regs = XdmaRegs(args.card)
    fast = HostLib(regs.path)
    host = HostDecoder(regs, poll_timeout=args.poll_timeout, fast=fast)
    if args.perf_tiles:
        host.perf_tiles = []
    host.raw_planes = True                       # pictures as bytes / array rows: compare() and write_planes() take them as they are
    host.overlap_read = bool(args.overlap)
    stage = XdmaStage(args.card)
    print(f"DMA target: card DDR4, {stage.stage_len} bytes at 0x{stage.base:x} (/dev/xdma{args.card}_c2h_0)")
    screen = None                                # the host's own screen (--show): tools/host/fbshow.py
    if args.show:
        try:
            from fbshow import Screen
            screen = Screen()
            print(f"screen /dev/fb0 {screen.xres}x{screen.yres} ({screen.bpp} bpp): output pictures 1:1 from the top-left corner")
        except (OSError, ImportError) as e:
            print(f"--show: no screen ({e})")
    cfg = host.identify()
    print(f"decoder at {regs.path} (libhostregs.so traffic): FBX {cfg['fbx']} FBY {cfg['fby']} lanes {cfg['lanes']} uram {cfg['uram']} sb128 {int(cfg['sb128'])}; "
          f"{len(d.frames)} coded frames, {len(shown)} shown")
    n_bad_frames = 0
    t_all = time.monotonic()

    def frames():
        for fi, hdr, planes, t in host.decode_frames_inter(d, (stage.base, stage.stage_len, stage.read), name=Path(args.stream).name,
                                                           pool=args.pool, want_planes=bool(out or ref or y4m or screen)):
            t["lf"] = t.get("filters", 0.0)
            t["tiles"] = t.get("tiles", 0.0) + t.get("setup", 0.0)
            t["read"] = t.get("read", 0.0)
            yield fi, hdr, planes, t
    pending = {}                                 # coded frame -> (hdr, planes) until its last display
    phase = {"tiles": 0.0, "finish": 0.0, "lf": 0.0, "read": 0.0, "n": 0}   # host phase totals over the board-decoded frames
    ev_next = 0                                  # the next output picture to emit

    def emit():
        """Output pictures in display order, as far as the decoded frames allow: the comparison with the reference
        and the -o / --y4m writes happen here, once per display (show_existing_frame repeats a picture)."""
        nonlocal ev_next, n_bad_frames, y4m_hdr
        while ev_next < len(shown) and shown[ev_next] in pending:
            k = shown[ev_next]
            hdr_k, planes_k = pending[k]
            line = f"  shown #{ev_next} = coded frame {k}"
            if screen is not None:
                screen.show(hdr_k, planes_k)
            if ref is not None and planes_k is not None:
                lo, hi = ref_events[ev_next]
                bad = compare(ref[lo:hi], hdr_k, planes_k)
                if bad:
                    n_bad_frames += 1
                    line += f": {len(bad)} samples differ (plane, y, x, board, reference): {bad[:args.max_bad]}"
                else:
                    line += ": identical to the reference"
            if planes_k is not None and out is not None:
                write_planes(out, hdr_k, planes_k)
            if planes_k is not None and y4m is not None:
                w, h = plane_size(hdr_k, 0)
                cs = ("mono" if hdr_k.NumPlanes == 1 else "420jpeg" if hdr_k.subsampling_x and hdr_k.subsampling_y
                      else "422" if hdr_k.subsampling_x else "444") + (f"p{hdr_k.BitDepth}" if hdr_k.BitDepth > 8 else "")
                if cs == "monop10": cs = "mono10"
                if y4m_hdr is None:
                    y4m_hdr = (w, h, cs)
                    y4m.write(f"YUV4MPEG2 W{w} H{h} F{fps} Ip A1:1 C{cs}\n".encode())
                if (w, h, cs) != y4m_hdr:
                    line += f" (not in the .y4m: {w}x{h} {cs} vs {y4m_hdr[0]}x{y4m_hdr[1]} {y4m_hdr[2]})"
                else:
                    y4m.write(b"FRAME\n")
                    write_planes(y4m, hdr_k, planes_k)
            if ref is not None or out is not None or y4m is not None:
                print(line, flush=True)
            owed[k] -= 1
            if owed[k] == 0:
                del pending[k]
            ev_next += 1

    def resolve(item):
        # a frame whose picture is still being read back (--overlap): wait for it here, a frame later
        fi, hdr, planes, t = item
        if planes is not None and hasattr(planes, "result"):
            t0_ = time.monotonic()
            planes = planes.result()
            t["read"] = t.get("read", 0.0) + time.monotonic() - t0_
        return fi, hdr, planes, t

    def lagged(gen):
        prev = None
        for item in gen:
            if prev is not None:
                yield resolve(prev)
            prev = item
        if prev is not None:
            yield resolve(prev)

    try:
        for fi, hdr, planes, t in (lagged(frames()) if args.overlap else frames()):
            if args.frames is not None and fi >= args.frames:
                break
            W, H = plane_size(hdr, 0)
            fmt = "mono" if hdr.NumPlanes == 1 else ("444" if not hdr.subsampling_x else ("420" if hdr.subsampling_y else "422"))
            line = (f"frame {fi}: {W}x{H} {fmt} bd{hdr.BitDepth}, {len(d.frames[fi])} tile(s): "
                    f"tiles {t['tiles']:.2f}s, lf {t['lf']:.2f}s, read-back {t['read']:.2f}s")
            for k in ("tiles", "finish", "lf", "read"):
                phase[k] += t.get(k, 0.0)
            phase["n"] += 1
            if owed[fi] == 0:
                line += " (never shown)"
            if t.get("fsb"):
                f_ = t["fsb"]
                line += (f"\n        state on the card: segment map {f_['seg']}, CDF rows {f_['cdf']}"
                         + (f", motion field projected from {f_['mf_srcs']} source(s)" if f_["mf_srcs"] is not None else "")
                         + (f"; CDF rows saved at 0x{f_['cdf_base']:x}" if f_["saved"] else "; frame-end CDF update disabled")
                         + f", MV words at 0x{f_['mv_base']:x}, segment ids at 0x{f_['seg_base']:x}")
            print(line, flush=True)
            if owed[fi]:
                pending[fi] = (hdr, planes)
                emit()
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
          f"{s['writes']} register writes + {s['reads']} reads in {dt_all:.1f}s ({s['frames'] / dt_all:.1f} fps); "
          f"{ev_next} of {len(shown)} pictures output"
          + (f"; card-side state: {s.get('ld_frames', 0)} loads, {s.get('sv_frames', 0)} CDF saves, {s.get('mf_srcs', 0)} projected sources; host waits: loads {s.get('ld_time', 0.0):.2f}s, CDF saves {s.get('sv_time', 0.0):.2f}s")
          + (f"; {n_bad_frames} board picture(s) differ from the reference" if ref is not None else ""))
    if phase["n"]:
        n = phase["n"]; other = dt_all - phase["tiles"] - phase["lf"] - phase["read"]
        print(f"host phases per board frame ({n}): tiles {1000 * phase['tiles'] / n:.1f} ms, filter tail {1000 * phase['lf'] / n:.1f} ms "
              f"(frame done {1000 * phase['finish'] / n:.1f} + flush {1000 * (phase['lf'] - phase['finish']) / n:.1f}), "
              f"read-back {1000 * phase['read'] / n:.1f} ms, other host work {1000 * other / n:.1f} ms = {1000 * dt_all / n:.1f} ms a frame")
    if args.perf_tiles and host.perf_tiles:
        pt = host.perf_tiles
        with open(args.perf_tiles, "w") as f:
            f.write("tag,bytes,push_ms,tile_ms,syn_stall_cycles,rec_idle_cycles\n")
            for r in pt:
                f.write(f"{r['tag']},{r['bytes']},{1000 * r['push_s']:.3f},{1000 * r['tile_s']:.3f},{r['syn_stall']},{r['rec_idle']}\n")
        n = len(pt); tile = sum(r["tile_s"] for r in pt); push = sum(r["push_s"] for r in pt)
        print(f"tiles ({n}): {1000 * tile / n:.1f} ms a tile on the host, the push ends after {1000 * push / n:.1f} ms; card counters a tile: "
              f"syntax waited for reconstruction {sum(r['syn_stall'] for r in pt) / n:,.0f} cycles, reconstruction idle "
              f"{sum(r['rec_idle'] for r in pt) / n:,.0f} cycles; {sum(r['bytes'] for r in pt) / n:,.0f} bytes a tile -> {args.perf_tiles}")
    sys.exit(1 if n_bad_frames else 0)


if __name__ == "__main__":
    main()
