#!/usr/bin/env python3
"""Check tb/obu_parser.py against dav1d: parse the stream file, and compare every header field the
tile model uses with the H event dav1d logged for the same tile (and the tile bytes with the T event).

Usage: tools/xcheck_hdr.py <stream.ivf|.obu> <trace.txt>
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm     # noqa: E402
import obu_parser as op     # noqa: E402

FIELDS = ["tile_row", "tile_col", "MiColStart", "MiColEnd", "MiRowStart", "MiRowEnd", "MiCols", "MiRows", "BitDepth",
          "layout", "use_128x128_superblock", "frame_type", "primary_ref_frame", "TxMode", "reduced_tx_set",
          "disable_cdf_update", "base_q_idx", "DeltaQYDc", "DeltaQUDc", "DeltaQUAc", "DeltaQVDc", "DeltaQVAc",
          "using_qmatrix", "qm_y", "qm_u", "qm_v", "delta_q_present", "delta_q_res", "delta_lf_present", "delta_lf_res",
          "delta_lf_multi", "allow_screen_content_tools", "allow_intrabc", "enable_filter_intra",
          "enable_intra_edge_filter", "enable_cdef", "cdef_damping", "cdef_bits", "CodedLossless",
          "FrameRestorationType", "LoopRestorationSize", "FrameWidth", "UpscaledWidth", "FrameHeight", "use_superres",
          "SuperresDenom", "subsampling_x", "subsampling_y", "segmentation_enabled", "segmentation_update_map",
          "segmentation_temporal_update", "SegIdPreSkip", "LastActiveSegId", "LosslessArray", "seg_qidx",
          "FeatureEnabled", "FeatureData", "NumPlanes", "FrameIsIntra", "SegQMLevel", "loop_filter_level",
          "loop_filter_sharpness", "loop_filter_delta_enabled", "loop_filter_ref_deltas", "loop_filter_mode_deltas",
          "cdef_y_strengths", "cdef_uv_strengths"]


def main():
    stream, trace = sys.argv[1], sys.argv[2]
    d = op.parse_file(stream)
    ref = []
    cur = None
    with open(trace) as fh:
        for line in fh:
            if line.startswith("T "):
                f = line.split()
                cur = dict(data=bytes.fromhex(f[3]) if len(f) > 3 else b"", hdr=None)
                ref.append(cur)
            elif line.startswith("H "):
                cur["hdr"] = tm.FrameHeader([int(x) for x in line.split()[1:]])
    if len(d.tiles) != len(ref):
        print(f"tile count: parser {len(d.tiles)} vs dav1d {len(ref)}")
    bad = 0
    for i, ((th, data), r) in enumerate(zip(d.tiles, ref)):
        if data != r["data"]:
            print(f"tile {i}: tile bytes differ (parser {len(data)} B, dav1d {len(r['data'])} B)")
            bad += 1
        dh = r["hdr"]
        for fld in FIELDS:
            a = getattr(th, fld, "MISSING")
            b = getattr(dh, fld, "MISSING")
            if fld == "FeatureData":
                # dav1d stores ref = -1 when disabled; the spec stores 0 with FeatureEnabled = 0
                b = [[v if th.FeatureEnabled[s][j] else 0 for j, v in enumerate(row)] for s, row in enumerate(b)]
            if fld == "FeatureEnabled":
                # dav1d cannot distinguish "enabled with value 0" from disabled for value features (see FrameHeader)
                a = [[(1 if (a[s][j] and (th.FeatureData[s][j] != 0 or j >= 5)) else 0) for j in range(8)] for s in range(8)]
            if fld == "LoopRestorationSize" and not any(t != 0 for t in dh.FrameRestorationType):
                continue                                  # unit size is unused (and unset in dav1d) without LR
            if fld == "cdef_damping" and (not dh.enable_cdef or dh.CodedLossless or dh.allow_intrabc):
                continue                                  # CDEF off: damping unused
            if a != b:
                print(f"tile {i}: {fld}: parser {a} vs dav1d {b}")
                bad += 1
    print("OK" if bad == 0 else f"{bad} differences")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
