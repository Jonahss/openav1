#!/usr/bin/env python3
"""Generate a random-content but valid AV1 intra stream (IVF) from the spec model itself.

How: write a sequence header + frame header with random coding parameters (tb/obu_writer.py), parse
them back with tb/obu_parser.py to get the same per-tile header objects tools/decode.py uses, then
run tile_model.TileDecoder over each tile with a RecordingDecoder whose "pick" chooses every symbol's
value (random, biased toward small coefficients so the transforms stay in conformance range). The
syntax that follows is whatever those choices imply. The recorded (cdf, value) sequence is
arithmetic-encoded (tb/msac_enc.py) into tile bytes and wrapped into an OBU_FRAME.

The result decodes with dav1d and with tools/decode.py; comparing the two is a differential fuzz test
of the whole decoder with syntax coverage no real encoder gives (every partition shape, palette,
filter-intra, every tx type, delta-q/lf, segmentation, tiles, ...).

Usage: tools/gen_stream.py out.ivf [--seed N] [--frames N] [--w W --h H] [--bd 8|10|12]
       [--fmt 420|422|444|mono] [--sb128] [--tiles] [--screen] [--lossless]
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm     # noqa: E402
import msac_enc as me       # noqa: E402
import obu_writer as ow     # noqa: E402
import obu_parser as op     # noqa: E402
import av1_tables as T      # noqa: E402


def make_params(rng, args):
    fmt = args.get("fmt", "420")
    bd = args.get("bd", 8)
    mono = fmt == "mono"
    sx, sy = {"420": (1, 1), "422": (1, 0), "444": (0, 0), "mono": (1, 1)}[fmt]
    profile = 0 if (fmt in ("420", "mono") and bd <= 10) else (1 if fmt == "444" and bd <= 10 else 2)
    seq = dict(profile=profile, width=args.get("w", 128), height=args.get("h", 96), bit_depth=bd, mono=int(mono),
               subsampling_x=sx, subsampling_y=sy, sb128=int(args.get("sb128", False)),
               filter_intra=1, intra_edge_filter=rng.randint(0, 1), screen_content=2 if args.get("screen") else 0,
               superres=0, cdef=1, restoration=1, separate_uv_delta_q=rng.randint(0, 1) if not mono else 0)
    lossless = bool(args.get("lossless"))
    q = {}
    q["disable_cdf_update"] = rng.choice([0, 0, 0, 1])
    q["disable_frame_end_update_cdf"] = 1
    q["allow_screen_content_tools"] = 1 if seq["screen_content"] else 0
    q["tile_cols_log2"] = 1 if args.get("tiles") else 0
    q["tile_rows_log2"] = 1 if args.get("tiles") else 0
    if lossless:
        q["base_q_idx"] = 0
        q["DeltaQYDc"] = q["DeltaQUDc"] = q["DeltaQUAc"] = q["DeltaQVDc"] = q["DeltaQVAc"] = 0
        q["diff_uv_delta"] = 0
        q["using_qmatrix"] = 0
    else:
        q["base_q_idx"] = rng.randint(30, 200)
        q["DeltaQYDc"] = rng.choice([0, 0, rng.randint(-8, 8)])
        q["DeltaQUDc"] = rng.choice([0, 0, rng.randint(-8, 8)])
        q["DeltaQUAc"] = rng.choice([0, 0, rng.randint(-8, 8)])
        q["diff_uv_delta"] = rng.randint(0, 1) if seq["separate_uv_delta_q"] else 0
        if q["diff_uv_delta"]:
            q["DeltaQVDc"] = rng.randint(-8, 8); q["DeltaQVAc"] = rng.randint(-8, 8)
        else:
            q["DeltaQVDc"], q["DeltaQVAc"] = q["DeltaQUDc"], q["DeltaQUAc"]
        q["using_qmatrix"] = rng.choice([0, 0, 1])
    q["qm_y"], q["qm_u"], q["qm_v"] = rng.randint(0, 14), rng.randint(0, 14), rng.randint(0, 14)
    if not seq["separate_uv_delta_q"]:
        q["qm_v"] = q["qm_u"]
    q["segmentation_enabled"] = rng.choice([0, 0, 1]) if not lossless else 0
    q["FeatureEnabled"] = [[0] * 8 for _ in range(8)]
    q["FeatureData"] = [[0] * 8 for _ in range(8)]
    if q["segmentation_enabled"]:
        for i in range(rng.randint(1, 8)):
            if rng.random() < 0.7:
                q["FeatureEnabled"][i][0] = 1
                q["FeatureData"][i][0] = rng.randint(-30, 30)
            for j in range(1, 5):
                if rng.random() < 0.3:
                    q["FeatureEnabled"][i][j] = 1
                    q["FeatureData"][i][j] = rng.randint(-20, 20)
    q["delta_q_present"] = rng.choice([0, 1]) if q["base_q_idx"] > 0 else 0
    q["delta_q_res"] = rng.randint(0, 3)
    q["delta_lf_present"] = rng.choice([0, 1]) if q["delta_q_present"] else 0
    q["delta_lf_res"] = rng.randint(0, 3)
    q["delta_lf_multi"] = rng.randint(0, 1)
    q["CodedLossless"] = 1 if lossless else 0
    q["loop_filter_level"] = [rng.randint(0, 63) for _ in range(4)]
    if rng.random() < 0.2:
        q["loop_filter_level"][0] = q["loop_filter_level"][1] = 0
    q["loop_filter_sharpness"] = rng.randint(0, 7)
    q["loop_filter_delta_enabled"] = rng.randint(0, 1)
    q["loop_filter_ref_deltas"] = [rng.randint(-63, 63) for _ in range(8)]
    q["loop_filter_mode_deltas"] = [rng.randint(-63, 63) for _ in range(2)]
    q["cdef_damping"] = rng.randint(3, 6)
    q["cdef_bits"] = rng.randint(0, 3)
    n = 1 << q["cdef_bits"]
    q["cdef_y_pri"] = [rng.randint(0, 15) for _ in range(n)]
    q["cdef_y_sec"] = [rng.randint(0, 3) for _ in range(n)]
    q["cdef_uv_pri"] = [rng.randint(0, 15) for _ in range(n)]
    q["cdef_uv_sec"] = [rng.randint(0, 3) for _ in range(n)]
    q["FrameRestorationType"] = [rng.choice([0, 1, 2, 3]) for _ in range(3)]
    q["lr_unit_shift"] = rng.randint(1 if seq["sb128"] else 0, 2)
    q["lr_uv_shift"] = rng.randint(0, 1)
    q["TxMode"] = 0 if lossless else rng.choice([1, 2, 2])
    q["reduced_tx_set"] = rng.randint(0, 1)
    return seq, q


def make_pick(rng, holder):
    """Random symbol choices, biased so coefficient magnitudes stay small (conformance ranges), and
    constrained where the spec puts a conformance requirement on the *decoded value*:
      - partitions must not create a block whose chroma residual size is BLOCK_INVALID (4:2:2),
      - segment_id must decode to 0..LastActiveSegId.
    holder["dec"] is the TileDecoder being driven (set after construction)."""
    def partition_ok(part):
        dec = holder["dec"]
        if dec.h.NumPlanes == 1:
            return True
        b = dec.part_bsize
        sizes = [T.Partition_Subsize[part][b]]
        if part in (tm.PARTITION_HORZ_A, tm.PARTITION_HORZ_B, tm.PARTITION_VERT_A, tm.PARTITION_VERT_B):
            sizes.append(T.Partition_Subsize[tm.PARTITION_SPLIT][b])
        return all(dec.get_plane_residual_size(sz, 1) != tm.BLOCK_INVALID for sz in sizes)

    def pick(cdf, N, name):
        if name in ("coeff_base", "coeff_base_eob"):
            return min(N - 1, rng.choice([0, 0, 0, 1, 1, 2, 3]))
        if name == "coeff_br":
            return rng.choice([0, 0, 0, 1, 2, 3]) if rng.random() < 0.9 else rng.randint(0, N - 1)
        if name == "golomb_length_bit":
            return 1 if rng.random() < 0.7 else 0
        if name.startswith("eob_pt"):
            return min(N - 1, rng.choice([0, 1, 2, 3, 4, rng.randint(0, N - 1)]))
        if name == "all_zero":
            return rng.choice([0, 1, 1])
        if name in ("delta_q_abs", "delta_lf_abs"):
            return min(N - 1, rng.choice([0, 0, 1, 2, 3]))
        if name in ("delta_q_rem_bits", "delta_lf_rem_bits"):
            return 0
        if name in ("has_palette_y", "has_palette_uv"):
            return rng.choice([0, 0, 1])
        if name == "partition":
            return rng.choice([p for p in range(N) if partition_ok(p)])
        if name == "split_or_vert":
            return 1 if not partition_ok(tm.PARTITION_VERT) else rng.randint(0, 1)
        if name == "segment_id":
            dec = holder["dec"]
            last = dec.h.LastActiveSegId
            ok = [v for v in range(N) if 0 <= dec.neg_deinterleave(v, dec.seg_pred, last + 1) <= last]
            return rng.choice(ok)
        return rng.randint(0, N - 1)
    return pick


def parse_headers(seq, q):
    """Round the sequence + frame header through obu_parser; returns (Decoder, frame header object)."""
    d = op.Decoder()
    d.feed_temporal_unit(ow.temporal_delimiter() + ow.obu(1, ow.sequence_header(seq)) + ow.frame_header_obu(seq, q))
    return d, d.cur


def generate(seed, args):
    rng = random.Random(seed)
    seq, q = make_params(rng, args)
    frames = []
    nframes = args.get("frames", 1)
    info = None
    for fi in range(nframes):
        d, h = parse_headers(seq, q)
        q["tile_cols_log2"], q["tile_rows_log2"] = h.TileColsLog2, h.TileRowsLog2
        tiles = []
        for tr in range(h.TileRows):
            for tc in range(h.TileCols):
                th = d.tile_header(h, tr, tc)
                holder = {}
                rec = me.RecordingDecoder(make_pick(rng, holder), th.disable_cdf_update)
                dec = tm.TileDecoder(th, b"", None)
                dec.dec = rec
                holder["dec"] = dec
                dec.decode_tile()
                tiles.append(me.encode_events(rec.events))
        tu = ow.temporal_delimiter() + (ow.obu(1, ow.sequence_header(seq)) if fi == 0 else b"") + ow.frame_obu(seq, q, tiles)
        frames.append(tu)
        if info is None:
            info = dict(seq=seq, q=dict(q), tiles=len(tiles), tile_bytes=sum(len(t) for t in tiles))
        if not q["CodedLossless"]:
            q["base_q_idx"] = rng.randint(30, 200)
    return ow.ivf(frames, seq["width"], seq["height"]), info


def main():
    argv = sys.argv[1:]
    out = [a for a in argv if not a.startswith("--")][0]
    args = {}

    def opt(name, default=None, conv=int):
        if name in argv:
            return conv(argv[argv.index(name) + 1])
        return default
    args["seed"] = opt("--seed", 1)
    args["frames"] = opt("--frames", 1)
    args["w"] = opt("--w", 128)
    args["h"] = opt("--h", 96)
    args["bd"] = opt("--bd", 8)
    args["fmt"] = opt("--fmt", "420", str)
    for flag in ("sb128", "tiles", "screen", "lossless"):
        args[flag] = ("--" + flag) in argv
    data, info = generate(args["seed"], args)
    open(out, "wb").write(data)
    q = info["q"]
    print(f"{out}: {len(data)} bytes, {args['frames']} frame(s) {args['w']}x{args['h']} {args['fmt']} {args['bd']}-bit "
          f"qidx {q['base_q_idx']} tiles {info['tiles']} ({info['tile_bytes']} tile bytes) seg {q['segmentation_enabled']} "
          f"dq {q['delta_q_present']} dlf {q['delta_lf_present']} lf {q['loop_filter_level']} cdef_bits {q['cdef_bits']} "
          f"lr {q['FrameRestorationType']} txmode {q['TxMode']} rts {q['reduced_tx_set']} cdfupd {1 - q['disable_cdf_update']}",
          file=sys.stderr)


if __name__ == "__main__":
    main()
