"""Minimal AV1 OBU writer for synthetic intra streams: sequence header, key-frame header with chosen
coding parameters, tile group, IVF container. Mirrors tb/obu_parser.py (spec 5.3 - 5.11).

The header fields it writes are exactly those tile_model.FrameHeader needs, so a generated stream is
decodable by both dav1d and tools/decode.py from the same parameter dict.
"""
import struct

RESTORE_NONE, RESTORE_WIENER, RESTORE_SGRPROJ, RESTORE_SWITCHABLE = 0, 1, 2, 3
LR_TYPE_CODE = {RESTORE_NONE: 0, RESTORE_SWITCHABLE: 1, RESTORE_WIENER: 2, RESTORE_SGRPROJ: 3}


class BitWriter:
    def __init__(self):
        self.bits = []

    def f(self, n, v):
        assert 0 <= v < (1 << n) or n == 0, (n, v)
        for i in range(n - 1, -1, -1):
            self.bits.append((v >> i) & 1)

    def su(self, n, v):
        self.f(n, v & ((1 << n) - 1))

    def ns(self, n, v):
        w = n.bit_length()
        m = (1 << w) - n
        if v < m:
            self.f(w - 1, v)
        else:
            self.f(w - 1, m + ((v - m) >> 1))
            self.f(1, (v - m) & 1)

    def byte_align(self):
        while len(self.bits) % 8:
            self.bits.append(0)

    def trailing_bits(self):
        self.bits.append(1)
        self.byte_align()

    def tobytes(self):
        assert len(self.bits) % 8 == 0
        out = bytearray()
        for i in range(0, len(self.bits), 8):
            b = 0
            for k in range(8):
                b = (b << 1) | self.bits[i + k]
            out.append(b)
        return bytes(out)


def leb128(v):
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def obu(obu_type, payload):
    hdr = bytes([(obu_type << 3) | (1 << 1)])          # has_size_field = 1, no extension
    return hdr + leb128(len(payload)) + payload


def sequence_header(p):
    """p: dict with profile, width, height, bit_depth, mono, subsampling_x/y, sb128, filter_intra,
    intra_edge_filter, screen_content(0/1), superres, cdef, restoration."""
    w = BitWriter()
    w.f(3, p["profile"])
    w.f(1, 0)                          # still_picture
    w.f(1, 0)                          # reduced_still_picture_header
    w.f(1, 0)                          # timing_info_present_flag
    w.f(1, 0)                          # initial_display_delay_present_flag
    w.f(5, 0)                          # operating_points_cnt_minus_1
    w.f(12, 0)                         # operating_point_idc[0]
    w.f(5, 8)                          # seq_level_idx[0] = 4.0 (> 7 -> tier bit follows)
    w.f(1, 0)                          # seq_tier
    wb = max(1, (p["width"] - 1).bit_length())
    hb = max(1, (p["height"] - 1).bit_length())
    w.f(4, wb - 1)
    w.f(4, hb - 1)
    w.f(wb, p["width"] - 1)
    w.f(hb, p["height"] - 1)
    w.f(1, 0)                          # frame_id_numbers_present_flag
    w.f(1, p["sb128"])
    w.f(1, p["filter_intra"])
    w.f(1, p["intra_edge_filter"])
    w.f(1, 0); w.f(1, 0); w.f(1, 0); w.f(1, 0)     # interintra, masked, warped, dual_filter
    w.f(1, 0)                          # enable_order_hint
    if p["screen_content"] == 2:       # SELECT: per-frame allow_screen_content_tools bit
        w.f(1, 1)                      # seq_choose_screen_content_tools (-> seq_force_integer_mv = SELECT)
    else:
        w.f(1, 0)                      # seq_choose_screen_content_tools = 0 -> explicit
        w.f(1, p["screen_content"])    # seq_force_screen_content_tools
        if p["screen_content"] > 0:
            w.f(1, 0)                  # seq_choose_integer_mv = 0
            w.f(1, 1)                  # seq_force_integer_mv
    w.f(1, p["superres"])
    w.f(1, p["cdef"])
    w.f(1, p["restoration"])
    # color_config
    bd = p["bit_depth"]
    if p["profile"] == 2 and bd > 8:
        w.f(1, 1); w.f(1, 1 if bd == 12 else 0)
    else:
        w.f(1, 1 if bd == 10 else 0)
    if p["profile"] != 1:
        w.f(1, p["mono"])
    w.f(1, 0)                          # color_description_present_flag
    if p["mono"]:
        w.f(1, 0)                      # color_range
    else:
        w.f(1, 0)                      # color_range
        if p["profile"] == 2 and bd == 12:
            w.f(1, p["subsampling_x"])
            if p["subsampling_x"]:
                w.f(1, p["subsampling_y"])
        if p["subsampling_x"] and p["subsampling_y"]:
            w.f(2, 0)                  # chroma_sample_position
        w.f(1, p["separate_uv_delta_q"])
    w.f(1, 0)                          # film_grain_params_present
    w.trailing_bits()
    return w.tobytes()


def frame_header_bits(p, q, seq):
    """Key frame, shown, error resilient (as the spec forces for shown key frames).
    q: coding parameter dict (see gen_stream.py: base_q_idx, deltas, qm, segmentation, delta q/lf,
    loop filter, cdef, lr, tx_mode, reduced_tx_set, tile layout)."""
    w = BitWriter()
    w.f(1, 0)                          # show_existing_frame
    w.f(2, 0)                          # frame_type = KEY_FRAME
    w.f(1, 1)                          # show_frame
    # error_resilient_mode implied 1 for shown key frames
    w.f(1, q["disable_cdf_update"])
    if seq["screen_content"] == 2:
        w.f(1, q["allow_screen_content_tools"])
    if seq["screen_content"] == 2 and q["allow_screen_content_tools"]:
        w.f(1, 1)                      # force_integer_mv (seq_force_integer_mv == SELECT); intra forces 1 anyway
    w.f(1, 0)                          # frame_size_override_flag
    # order_hint: OrderHintBits = 0 -> nothing
    # primary_ref_frame: intra -> none
    # refresh_frame_flags: key+show -> allFrames, nothing read
    # frame_size(): no override -> superres_params
    if seq["superres"]:
        w.f(1, 0)                      # use_superres = 0
    w.f(1, 0)                          # render_and_frame_size_different
    if q["allow_screen_content_tools"]:
        w.f(1, 0)                      # allow_intrabc = 0 (UpscaledWidth == FrameWidth)
    if not q["disable_cdf_update"]:
        w.f(1, q["disable_frame_end_update_cdf"])
    # tile_info
    MiCols = 2 * ((seq["width"] + 7) >> 3)
    MiRows = 2 * ((seq["height"] + 7) >> 3)
    sb128 = seq["sb128"]
    sbCols = (MiCols + 31) >> 5 if sb128 else (MiCols + 15) >> 4
    sbRows = (MiRows + 31) >> 5 if sb128 else (MiRows + 15) >> 4
    sbShift = 5 if sb128 else 4
    sbSize = sbShift + 2
    maxTileWidthSb = 4096 >> sbSize
    maxTileAreaSb = (4096 * 2304) >> (2 * sbSize)

    def tile_log2(b, t):
        k = 0
        while (b << k) < t:
            k += 1
        return k
    minLog2TileCols = tile_log2(maxTileWidthSb, sbCols)
    maxLog2TileCols = tile_log2(1, min(sbCols, 64))
    maxLog2TileRows = tile_log2(1, min(sbRows, 64))
    minLog2Tiles = max(minLog2TileCols, tile_log2(maxTileAreaSb, sbRows * sbCols))
    w.f(1, 1)                          # uniform_tile_spacing_flag
    TileColsLog2 = minLog2TileCols
    want_cols = q["tile_cols_log2"]
    while TileColsLog2 < maxLog2TileCols:
        if TileColsLog2 < want_cols:
            w.f(1, 1); TileColsLog2 += 1
        else:
            w.f(1, 0); break
    minLog2TileRows = max(minLog2Tiles - TileColsLog2, 0)
    TileRowsLog2 = minLog2TileRows
    want_rows = q["tile_rows_log2"]
    while TileRowsLog2 < maxLog2TileRows:
        if TileRowsLog2 < want_rows:
            w.f(1, 1); TileRowsLog2 += 1
        else:
            w.f(1, 0); break
    if TileColsLog2 > 0 or TileRowsLog2 > 0:
        w.f(TileColsLog2 + TileRowsLog2, 0)     # context_update_tile_id
        w.f(2, 3)                               # tile_size_bytes_minus_1 -> 4 bytes
    # quantization_params
    w.f(8, q["base_q_idx"])
    for name in ("DeltaQYDc",):
        v = q[name]
        if v:
            w.f(1, 1); w.su(7, v)
        else:
            w.f(1, 0)
    if seq["mono"] == 0:
        if seq["separate_uv_delta_q"]:
            w.f(1, q["diff_uv_delta"])
        for name in ("DeltaQUDc", "DeltaQUAc"):
            v = q[name]
            if v:
                w.f(1, 1); w.su(7, v)
            else:
                w.f(1, 0)
        if seq["separate_uv_delta_q"] and q["diff_uv_delta"]:
            for name in ("DeltaQVDc", "DeltaQVAc"):
                v = q[name]
                if v:
                    w.f(1, 1); w.su(7, v)
                else:
                    w.f(1, 0)
    w.f(1, q["using_qmatrix"])
    if q["using_qmatrix"]:
        w.f(4, q["qm_y"]); w.f(4, q["qm_u"])
        if seq["separate_uv_delta_q"]:
            w.f(4, q["qm_v"])
    # segmentation_params
    w.f(1, q["segmentation_enabled"])
    if q["segmentation_enabled"]:
        bits = [8, 6, 6, 6, 6, 3, 0, 0]
        signed = [1, 1, 1, 1, 1, 0, 0, 0]
        for i in range(8):
            for j in range(8):
                en = q["FeatureEnabled"][i][j]
                w.f(1, en)
                if en:
                    if signed[j]:
                        w.su(1 + bits[j], q["FeatureData"][i][j])
                    elif bits[j]:
                        w.f(bits[j], q["FeatureData"][i][j])
    # delta_q_params / delta_lf_params
    if q["base_q_idx"] > 0:
        w.f(1, q["delta_q_present"])
    if q["delta_q_present"]:
        w.f(2, q["delta_q_res"])
        w.f(1, q["delta_lf_present"])            # allow_intrabc == 0
        if q["delta_lf_present"]:
            w.f(2, q["delta_lf_res"]); w.f(1, q["delta_lf_multi"])
    # CodedLossless decides the rest
    coded_lossless = q["CodedLossless"]
    if not coded_lossless:
        w.f(6, q["loop_filter_level"][0]); w.f(6, q["loop_filter_level"][1])
        if seq["mono"] == 0 and (q["loop_filter_level"][0] or q["loop_filter_level"][1]):
            w.f(6, q["loop_filter_level"][2]); w.f(6, q["loop_filter_level"][3])
        w.f(3, q["loop_filter_sharpness"])
        w.f(1, q["loop_filter_delta_enabled"])
        if q["loop_filter_delta_enabled"]:
            w.f(1, 1)                              # loop_filter_delta_update
            for i in range(8):
                w.f(1, 1); w.su(7, q["loop_filter_ref_deltas"][i])
            for i in range(2):
                w.f(1, 1); w.su(7, q["loop_filter_mode_deltas"][i])
        if seq["cdef"]:
            w.f(2, q["cdef_damping"] - 3)
            w.f(2, q["cdef_bits"])
            for i in range(1 << q["cdef_bits"]):
                w.f(4, q["cdef_y_pri"][i]); w.f(2, q["cdef_y_sec"][i])
                if seq["mono"] == 0:
                    w.f(4, q["cdef_uv_pri"][i]); w.f(2, q["cdef_uv_sec"][i])
        if seq["restoration"]:
            uses_lr = any(t != RESTORE_NONE for t in q["FrameRestorationType"][:3 if not seq["mono"] else 1])
            uses_chroma = any(t != RESTORE_NONE for t in q["FrameRestorationType"][1:3]) and not seq["mono"]
            for i in range(1 if seq["mono"] else 3):
                w.f(2, LR_TYPE_CODE[q["FrameRestorationType"][i]])
            if uses_lr:
                shift = q["lr_unit_shift"]           # 0..2 (size 64/128/256)
                if sb128:
                    w.f(1, shift - 1)
                else:
                    w.f(1, 1 if shift else 0)
                    if shift:
                        w.f(1, shift - 1)
                if seq["subsampling_x"] and seq["subsampling_y"] and uses_chroma:
                    w.f(1, q["lr_uv_shift"])
        w.f(1, 1 if q["TxMode"] == 2 else 0)       # tx_mode_select
    # frame_reference_mode / skip_mode: intra -> nothing; allow_warped_motion: nothing
    w.f(1, q["reduced_tx_set"])
    # global_motion_params: intra -> nothing; film grain: not present
    return w


def frame_header_obu(p, q):
    """OBU_FRAME_HEADER (type 3): header + trailing bits. Used to let obu_parser compute the derived state."""
    w = frame_header_bits(p, q, p)
    w.trailing_bits()
    return obu(3, w.tobytes())


def frame_obu(p, q, tiles):
    """tiles: list of tile byte strings in tile order. Returns the OBU_FRAME bytes."""
    w = frame_header_bits(p, q, p)
    w.byte_align()
    hdr_bytes = w.tobytes()
    # tile group (all tiles): no tile_start_and_end_present_flag bit if NumTiles == 1
    tg = BitWriter()
    if len(tiles) > 1:
        tg.f(1, 0)
    tg.byte_align()
    payload = bytearray(hdr_bytes + tg.tobytes())
    for i, t in enumerate(tiles):
        if i < len(tiles) - 1:
            payload += struct.pack("<I", len(t) - 1)      # tile_size_minus_1, 4 bytes (le)
        payload += t
    return obu(6, bytes(payload))


def temporal_delimiter():
    return obu(2, b"")


def ivf(frames, width, height):
    """frames: list of temporal-unit byte strings. Returns IVF file bytes."""
    out = bytearray(b"DKIF")
    out += struct.pack("<HH4sHHIIII", 0, 32, b"AV01", width, height, 30, 1, len(frames), 0)
    for i, f in enumerate(frames):
        out += struct.pack("<IQ", len(f), i)
        out += f
    return bytes(out)
