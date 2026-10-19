"""Spec-literal AV1 OBU / sequence header / frame header parser (spec 5.3-5.9, 5.11 tile group,
Annex B), for the intra-frame decoder. Produces, per tile, a header object with the same attribute
names as tile_model.FrameHeader, so TileDecoder / FrameRecon / the post filters run unchanged.

Supports: low-overhead OBU streams (IVF container or raw .obu), Annex B (length delimited,
as used by the Argon streams). Inter-frame-only header fields are parsed (so the bit position
stays right) but reference-frame state is not tracked beyond what intra decoding needs.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import av1_tables as T  # noqa: E402

OBU_SEQUENCE_HEADER, OBU_TEMPORAL_DELIMITER, OBU_FRAME_HEADER, OBU_TILE_GROUP, OBU_METADATA, OBU_FRAME, \
    OBU_REDUNDANT_FRAME_HEADER, OBU_TILE_LIST, OBU_PADDING = 1, 2, 3, 4, 5, 6, 7, 8, 15
KEY_FRAME, INTER_FRAME, INTRA_ONLY_FRAME, SWITCH_FRAME = 0, 1, 2, 3
NUM_REF_FRAMES, REFS_PER_FRAME, TOTAL_REFS_PER_FRAME = 8, 7, 8
# reference frame names (RefFrame values): INTRA_FRAME = 0, LAST_FRAME .. ALTREF_FRAME = 1 .. 7
INTRA_FRAME, LAST_FRAME, LAST2_FRAME, LAST3_FRAME, GOLDEN_FRAME, BWDREF_FRAME, ALTREF2_FRAME, ALTREF_FRAME = range(8)
# global motion (5.9.24 / 7.10.2.1): types and the parameter coding constants (spec constants table)
IDENTITY, TRANSLATION, ROTZOOM, AFFINE = 0, 1, 2, 3
WARPEDMODEL_PREC_BITS = 16
GM_ABS_TRANS_BITS, GM_ABS_TRANS_ONLY_BITS, GM_ABS_ALPHA_BITS = 12, 9, 12
GM_ALPHA_PREC_BITS, GM_TRANS_PREC_BITS, GM_TRANS_ONLY_PREC_BITS = 15, 6, 3
GM_IDENTITY = [0, 0, 1 << WARPEDMODEL_PREC_BITS, 0, 0, 1 << WARPEDMODEL_PREC_BITS]     # gm_params[ref] of an IDENTITY model
SWITCHABLE = 4                                             # interpolation_filter
DEFAULT_LF_REF_DELTAS = [1, 0, 0, 0, -1, 0, -1, -1]
PRIMARY_REF_NONE = 7
SELECT_SCREEN_CONTENT_TOOLS, SELECT_INTEGER_MV = 2, 2
MAX_SEGMENTS, SEG_LVL_MAX, SEG_LVL_REF_FRAME = 8, 8, 5
MAX_TILE_WIDTH, MAX_TILE_AREA, MAX_TILE_COLS, MAX_TILE_ROWS = 4096, 4096 * 2304, 64, 64
SUPERRES_NUM, SUPERRES_DENOM_MIN, SUPERRES_DENOM_BITS = 8, 9, 3
RESTORATION_TILESIZE_MAX = 256
RESTORE_NONE, RESTORE_WIENER, RESTORE_SGRPROJ, RESTORE_SWITCHABLE = 0, 1, 2, 3
REMAP_LR_TYPE = [RESTORE_NONE, RESTORE_SWITCHABLE, RESTORE_WIENER, RESTORE_SGRPROJ]
SEGMENTATION_FEATURE_BITS = [8, 6, 6, 6, 6, 3, 0, 0]
SEGMENTATION_FEATURE_SIGNED = [1, 1, 1, 1, 1, 0, 0, 0]
SEGMENTATION_FEATURE_MAX = [255, 63, 63, 63, 63, 7, 0, 0]
CP_BT_709, TC_SRGB, MC_IDENTITY = 1, 13, 0


def clip3(lo, hi, x):
    return lo if x < lo else hi if x > hi else x


class BitReader:
    def __init__(self, data, pos=0):
        self.data = data
        self.pos = pos * 8      # bit position

    def f(self, n):
        v = 0
        for _ in range(n):
            byte = self.data[self.pos >> 3]
            v = (v << 1) | ((byte >> (7 - (self.pos & 7))) & 1)
            self.pos += 1
        return v

    def su(self, n):
        v = self.f(n)
        sign = 1 << (n - 1)
        return v - 2 * sign if (v & sign) else v

    def ns(self, n):
        w = n.bit_length()
        m = (1 << w) - n
        v = self.f(w - 1)
        if v < m:
            return v
        return (v << 1) - m + self.f(1)

    def le(self, n):
        t = 0
        for i in range(n):
            t += self.f(8) << (i * 8)
        return t

    def leb128(self):
        value = 0
        for i in range(8):
            b = self.f(8)
            value |= (b & 0x7F) << (i * 7)
            if not (b & 0x80):
                break
        return value

    def uvlc(self):
        leading = 0
        while True:
            if self.f(1):
                break
            leading += 1
        if leading >= 32:
            return (1 << 32) - 1
        return self.f(leading) + (1 << leading) - 1

    def byte_alignment(self):
        while self.pos & 7:
            self.f(1)

    def byte_pos(self):
        return self.pos >> 3


class SequenceHeader:
    pass


class ParsedFrameHeader:
    """Attribute names match tile_model.FrameHeader (which is built from the trace's H event)."""
    pass


class Decoder:
    """Parses OBUs; yields (frame_header_object, tile_row, tile_col, tile_bytes) per tile via .tiles."""

    def __init__(self):
        self.seq = None
        self.SeenFrameHeader = 0
        self.tiles = []            # list of (hdr, tile_bytes)
        self.frames = []           # list of lists of tile indices (decoded frames, decode order)
        self.frame_headers = []    # every frame header in decode order, show_existing_frame ones included
        self.RefValid = [0] * 8
        self.RefFrameType = [0] * 8
        self.RefOrderHint = [0] * 8
        # the rest of the reference frame update process (7.20): what later frame headers read back
        self.RefFrameId = [0] * 8
        self.RefUpscaledWidth = [0] * 8
        self.RefFrameWidth = [0] * 8
        self.RefFrameHeight = [0] * 8
        self.RefRenderWidth = [0] * 8
        self.RefRenderHeight = [0] * 8
        self.RefMiCols = [0] * 8
        self.RefMiRows = [0] * 8
        self.RefShowableFrame = [0] * 8
        self.SavedOrderHints = [[0] * 8 for _ in range(8)]              # [slot][refFrame]
        self.SavedGmParams = [[list(GM_IDENTITY) for _ in range(8)] for _ in range(8)]   # [slot][refFrame][6]
        self.SavedLoopFilterRefDeltas = [list(DEFAULT_LF_REF_DELTAS) for _ in range(8)]
        self.SavedLoopFilterModeDeltas = [[0, 0] for _ in range(8)]
        self.SavedFeatureEnabled = [[[0] * SEG_LVL_MAX for _ in range(MAX_SEGMENTS)] for _ in range(8)]
        self.SavedFeatureData = [[[0] * SEG_LVL_MAX for _ in range(MAX_SEGMENTS)] for _ in range(8)]
        self.SavedBitDepth = [0] * 8
        self.SavedSubsampling = [(0, 0)] * 8

    # ---- containers --------------------------------------------------------------------------------
    def feed_ivf(self, data):
        assert data[:4] == b"DKIF", "not an IVF file"
        hdr_size = int.from_bytes(data[6:8], "little")
        off = hdr_size
        while off + 12 <= len(data):
            sz = int.from_bytes(data[off:off + 4], "little")
            off += 12
            self.feed_temporal_unit(data[off:off + sz])
            off += sz

    def feed_annexb(self, data):
        off = 0
        while off < len(data):
            br = BitReader(data, off)
            tu_size = br.leb128()
            off = br.byte_pos()
            end = off + tu_size
            while off < end:
                br = BitReader(data, off)
                fu_size = br.leb128()
                off = br.byte_pos()
                fend = off + fu_size
                while off < fend:
                    br = BitReader(data, off)
                    obu_len = br.leb128()
                    off = br.byte_pos()
                    self.parse_obu(data[off:off + obu_len], sized=False)
                    off += obu_len
            off = end

    def feed_temporal_unit(self, data):
        off = 0
        while off < len(data):
            n = self.parse_obu(data[off:], sized=True)
            off += n

    # ---- 5.3 OBU --------------------------------------------------------------------------------------
    def parse_obu(self, data, sized):
        br = BitReader(data)
        br.f(1)                                   # obu_forbidden_bit
        obu_type = br.f(4)
        ext = br.f(1)
        has_size = br.f(1)
        br.f(1)
        self.temporal_id = 0
        self.spatial_id = 0
        if ext:
            self.temporal_id = br.f(3); self.spatial_id = br.f(2); br.f(3)
        # drop OBUs outside the selected operating point (spec 5.3.1 / 7.1: op 0 unless chosen otherwise)
        opidc = getattr(self.seq, "OperatingPointIdc", 0) if getattr(self, "seq", None) is not None else 0
        if ext and opidc != 0 and obu_type not in (OBU_SEQUENCE_HEADER, OBU_TEMPORAL_DELIMITER):
            in_temporal = (opidc >> self.temporal_id) & 1
            in_spatial = (opidc >> (self.spatial_id + 8)) & 1
            if not in_temporal or not in_spatial:
                # consume the header the same way and skip the payload
                if has_size:
                    obu_size = br.leb128()
                else:
                    obu_size = len(data) - 1 - ext
                return br.byte_pos() + obu_size
        if has_size:
            obu_size = br.leb128()
        else:
            obu_size = len(data) - 1 - ext
        start = br.byte_pos()
        payload = data[start:start + obu_size]
        if obu_type == OBU_SEQUENCE_HEADER:
            self.sequence_header(BitReader(payload))
        elif obu_type == OBU_TEMPORAL_DELIMITER:
            self.SeenFrameHeader = 0
        elif obu_type in (OBU_FRAME_HEADER, OBU_REDUNDANT_FRAME_HEADER):
            self.frame_header_obu(BitReader(payload), payload)
        elif obu_type == OBU_TILE_GROUP:
            self.tile_group_obu(BitReader(payload), payload)
        elif obu_type == OBU_FRAME:
            br2 = BitReader(payload)
            self.frame_header_obu(br2, payload)
            br2.byte_alignment()
            self.tile_group_obu(br2, payload)
        # metadata / padding / tile list: ignored
        return start + obu_size

    # ---- 5.5 sequence header -----------------------------------------------------------------------------
    def sequence_header(self, br):
        s = SequenceHeader()
        s.seq_profile = br.f(3)
        s.still_picture = br.f(1)
        s.reduced_still_picture_header = br.f(1)
        s.decoder_model_info_present_flag = 0
        s.equal_picture_interval = 0
        if s.reduced_still_picture_header:
            s.timing_info_present_flag = 0
            s.initial_display_delay_present_flag = 0
            s.operating_points_cnt_minus_1 = 0
            s.operating_point_idc = [0]
            br.f(5)
            s.decoder_model_present_for_this_op = [0]
        else:
            s.timing_info_present_flag = br.f(1)
            if s.timing_info_present_flag:
                br.f(32); br.f(32)
                s.equal_picture_interval = br.f(1)
                if s.equal_picture_interval:
                    br.uvlc()
                s.decoder_model_info_present_flag = br.f(1)
                if s.decoder_model_info_present_flag:
                    s.buffer_delay_length_minus_1 = br.f(5)
                    br.f(32)
                    s.buffer_removal_time_length_minus_1 = br.f(5)
                    s.frame_presentation_time_length_minus_1 = br.f(5)
            s.initial_display_delay_present_flag = br.f(1)
            s.operating_points_cnt_minus_1 = br.f(5)
            s.operating_point_idc = []
            s.decoder_model_present_for_this_op = []
            for i in range(s.operating_points_cnt_minus_1 + 1):
                s.operating_point_idc.append(br.f(12))
                seq_level_idx = br.f(5)
                if seq_level_idx > 7:
                    br.f(1)
                if s.decoder_model_info_present_flag:
                    present = br.f(1)
                    s.decoder_model_present_for_this_op.append(present)
                    if present:
                        n = s.buffer_delay_length_minus_1 + 1
                        br.f(n); br.f(n); br.f(1)
                else:
                    s.decoder_model_present_for_this_op.append(0)
                if s.initial_display_delay_present_flag:
                    if br.f(1):
                        br.f(4)
        s.OperatingPointIdc = s.operating_point_idc[0]
        s.frame_width_bits_minus_1 = br.f(4)
        s.frame_height_bits_minus_1 = br.f(4)
        s.max_frame_width_minus_1 = br.f(s.frame_width_bits_minus_1 + 1)
        s.max_frame_height_minus_1 = br.f(s.frame_height_bits_minus_1 + 1)
        s.frame_id_numbers_present_flag = 0 if s.reduced_still_picture_header else br.f(1)
        if s.frame_id_numbers_present_flag:
            s.delta_frame_id_length_minus_2 = br.f(4)
            s.additional_frame_id_length_minus_1 = br.f(3)
        s.use_128x128_superblock = br.f(1)
        s.enable_filter_intra = br.f(1)
        s.enable_intra_edge_filter = br.f(1)
        if s.reduced_still_picture_header:
            s.enable_interintra_compound = s.enable_masked_compound = s.enable_warped_motion = 0
            s.enable_dual_filter = s.enable_order_hint = s.enable_jnt_comp = s.enable_ref_frame_mvs = 0
            s.seq_force_screen_content_tools = SELECT_SCREEN_CONTENT_TOOLS
            s.seq_force_integer_mv = SELECT_INTEGER_MV
            s.OrderHintBits = 0
        else:
            s.enable_interintra_compound = br.f(1)
            s.enable_masked_compound = br.f(1)
            s.enable_warped_motion = br.f(1)
            s.enable_dual_filter = br.f(1)
            s.enable_order_hint = br.f(1)
            if s.enable_order_hint:
                s.enable_jnt_comp = br.f(1)
                s.enable_ref_frame_mvs = br.f(1)
            else:
                s.enable_jnt_comp = s.enable_ref_frame_mvs = 0
            if br.f(1):
                s.seq_force_screen_content_tools = SELECT_SCREEN_CONTENT_TOOLS
            else:
                s.seq_force_screen_content_tools = br.f(1)
            if s.seq_force_screen_content_tools > 0:
                if br.f(1):
                    s.seq_force_integer_mv = SELECT_INTEGER_MV
                else:
                    s.seq_force_integer_mv = br.f(1)
            else:
                s.seq_force_integer_mv = SELECT_INTEGER_MV
            if s.enable_order_hint:
                s.OrderHintBits = br.f(3) + 1
            else:
                s.OrderHintBits = 0
        s.enable_superres = br.f(1)
        s.enable_cdef = br.f(1)
        s.enable_restoration = br.f(1)
        self.color_config(br, s)
        s.film_grain_params_present = br.f(1)
        self.seq = s

    def color_config(self, br, s):
        high_bitdepth = br.f(1)
        if s.seq_profile == 2 and high_bitdepth:
            s.BitDepth = 12 if br.f(1) else 10
        else:
            s.BitDepth = 10 if high_bitdepth else 8
        s.mono_chrome = 0 if s.seq_profile == 1 else br.f(1)
        s.NumPlanes = 1 if s.mono_chrome else 3
        if br.f(1):
            cp, tc, mc = br.f(8), br.f(8), br.f(8)
        else:
            cp, tc, mc = 2, 2, 2
        s.color_primaries, s.transfer_characteristics, s.matrix_coefficients = cp, tc, mc
        if s.mono_chrome:
            s.color_range = br.f(1)
            s.subsampling_x = s.subsampling_y = 1
            s.separate_uv_delta_q = 0
            return
        if cp == CP_BT_709 and tc == TC_SRGB and mc == MC_IDENTITY:
            s.color_range = 1
            s.subsampling_x = s.subsampling_y = 0
        else:
            s.color_range = br.f(1)
            if s.seq_profile == 0:
                s.subsampling_x = s.subsampling_y = 1
            elif s.seq_profile == 1:
                s.subsampling_x = s.subsampling_y = 0
            else:
                if s.BitDepth == 12:
                    s.subsampling_x = br.f(1)
                    s.subsampling_y = br.f(1) if s.subsampling_x else 0
                else:
                    s.subsampling_x, s.subsampling_y = 1, 0
            if s.subsampling_x and s.subsampling_y:
                br.f(2)                           # chroma_sample_position
        s.separate_uv_delta_q = br.f(1)

    # ---- 5.9 frame header --------------------------------------------------------------------------------
    def shown_frames(self):
        """Output pictures in display order as indices into self.frames: a frame with show_frame = 1 is output
        when decoded; show_existing_frame outputs the decoded frame held in the reference slot (7.20 reference
        frame update with refresh_frame_flags; a shown key frame refreshes every slot)."""
        out = []
        slots = [None] * 8
        k = 0
        for h in self.frame_headers:
            if h.show_existing_frame:
                out.append(slots[h.frame_to_show_map_idx])
                continue
            if h.show_frame:
                out.append(k)
            flags = 0xFF if (h.frame_type == KEY_FRAME and h.show_frame) else h.refresh_frame_flags
            for i in range(8):
                if (flags >> i) & 1:
                    slots[i] = k
            k += 1
        return out

    def frame_header_obu(self, br, payload):
        if self.SeenFrameHeader:
            return                                # frame_header_copy: identical; nothing to do
        self.SeenFrameHeader = 1
        self.uncompressed_header(br)
        h = self.cur
        self.frame_headers.append(h)
        if h.show_existing_frame:
            self.SeenFrameHeader = 0
        else:
            self.TileNum = 0
            self.SeenFrameHeader = 1

    def uncompressed_header(self, br):
        s = self.seq
        h = ParsedFrameHeader()
        self.cur = h
        h.show_existing_frame = 0
        if s.frame_id_numbers_present_flag:
            idLen = s.additional_frame_id_length_minus_1 + s.delta_frame_id_length_minus_2 + 3
        allFrames = (1 << NUM_REF_FRAMES) - 1
        if s.reduced_still_picture_header:
            h.frame_type = KEY_FRAME
            h.FrameIsIntra = 1
            h.show_frame = 1
            h.showable_frame = 0
            h.error_resilient_mode = 1
        else:
            h.show_existing_frame = br.f(1)
            if h.show_existing_frame:
                h.frame_to_show_map_idx = br.f(3)
                if s.decoder_model_info_present_flag and not s.equal_picture_interval:
                    br.f(s.frame_presentation_time_length_minus_1 + 1)
                if s.frame_id_numbers_present_flag:
                    br.f(idLen)                               # display_frame_id
                h.frame_type = self.RefFrameType[h.frame_to_show_map_idx]
                h.refresh_frame_flags = allFrames if h.frame_type == KEY_FRAME else 0
                # (load_grain_params: film grain is not modelled)
                self.ref_load(h, h.frame_to_show_map_idx)         # 7.21 reference frame loading process
                if h.frame_type == KEY_FRAME:
                    h.show_frame = 1
                    self.ref_update(h)                            # 7.20 with refresh_frame_flags = allFrames
                return
            h.frame_type = br.f(2)
            h.FrameIsIntra = int(h.frame_type in (INTRA_ONLY_FRAME, KEY_FRAME))
            h.show_frame = br.f(1)
            if h.show_frame and s.decoder_model_info_present_flag and not s.equal_picture_interval:
                br.f(s.frame_presentation_time_length_minus_1 + 1)
            if h.show_frame:
                h.showable_frame = int(h.frame_type != KEY_FRAME)
            else:
                h.showable_frame = br.f(1)
            if h.frame_type == SWITCH_FRAME or (h.frame_type == KEY_FRAME and h.show_frame):
                h.error_resilient_mode = 1
            else:
                h.error_resilient_mode = br.f(1)
        if h.frame_type == KEY_FRAME and h.show_frame:
            self.RefValid = [0] * 8
            self.RefOrderHint = [0] * 8
        h.disable_cdf_update = br.f(1)
        if s.seq_force_screen_content_tools == SELECT_SCREEN_CONTENT_TOOLS:
            h.allow_screen_content_tools = br.f(1)
        else:
            h.allow_screen_content_tools = s.seq_force_screen_content_tools
        if h.allow_screen_content_tools:
            if s.seq_force_integer_mv == SELECT_INTEGER_MV:
                h.force_integer_mv = br.f(1)
            else:
                h.force_integer_mv = s.seq_force_integer_mv
        else:
            h.force_integer_mv = 0
        if h.FrameIsIntra:
            h.force_integer_mv = 1
        if s.frame_id_numbers_present_flag:
            h.current_frame_id = br.f(idLen)
        else:
            h.current_frame_id = 0
        if h.frame_type == SWITCH_FRAME:
            h.frame_size_override_flag = 1
        elif s.reduced_still_picture_header:
            h.frame_size_override_flag = 0
        else:
            h.frame_size_override_flag = br.f(1)
        h.order_hint = br.f(s.OrderHintBits)
        if h.FrameIsIntra or h.error_resilient_mode:
            h.primary_ref_frame = PRIMARY_REF_NONE
        else:
            h.primary_ref_frame = br.f(3)
        if s.decoder_model_info_present_flag:
            if br.f(1):                               # buffer_removal_time_present_flag
                for op in range(s.operating_points_cnt_minus_1 + 1):
                    if s.decoder_model_present_for_this_op[op]:
                        idc = s.operating_point_idc[op]
                        in_temporal = (idc >> self.temporal_id) & 1
                        in_spatial = (idc >> (self.spatial_id + 8)) & 1
                        if idc == 0 or (in_temporal and in_spatial):
                            br.f(s.buffer_removal_time_length_minus_1 + 1)   # buffer_removal_time[opNum]
        h.allow_high_precision_mv = 0
        h.use_ref_frame_mvs = 0
        h.allow_intrabc = 0
        if h.frame_type == SWITCH_FRAME or (h.frame_type == KEY_FRAME and h.show_frame):
            h.refresh_frame_flags = allFrames
        else:
            h.refresh_frame_flags = br.f(8)
        if not h.FrameIsIntra or h.refresh_frame_flags != allFrames:
            if h.error_resilient_mode and s.enable_order_hint:
                for i in range(NUM_REF_FRAMES):
                    ref_order_hint = br.f(s.OrderHintBits)
                    if ref_order_hint != self.RefOrderHint[i]:
                        self.RefValid[i] = 0                  # (this spec text; the errata also set RefOrderHint[i])
        h.ref_frame_idx = [0] * REFS_PER_FRAME
        h.OrderHints = [0] * 8                                # by RefFrame (INTRA_FRAME unused)
        h.RefFrameSignBias = [0] * 8
        h.allow_high_precision_mv = 0
        h.interpolation_filter = 0
        h.is_motion_mode_switchable = 0
        h.use_ref_frame_mvs = 0
        if h.FrameIsIntra:
            self.frame_size(br, h)
            self.render_size(br, h)
            if h.allow_screen_content_tools and h.UpscaledWidth == h.FrameWidth:
                h.allow_intrabc = br.f(1)
        else:
            if not s.enable_order_hint:
                frame_refs_short_signaling = 0
            else:
                frame_refs_short_signaling = br.f(1)
                if frame_refs_short_signaling:
                    last_frame_idx = br.f(3)
                    gold_frame_idx = br.f(3)
                    self.set_frame_refs(h, last_frame_idx, gold_frame_idx)
            for i in range(REFS_PER_FRAME):
                if not frame_refs_short_signaling:
                    h.ref_frame_idx[i] = br.f(3)
                if s.frame_id_numbers_present_flag:
                    br.f(s.delta_frame_id_length_minus_2 + 2)     # delta_frame_id_minus_1 (expectedFrameId: not checked)
            if h.frame_size_override_flag and not h.error_resilient_mode:
                self.frame_size_with_refs(br, h)
            else:
                self.frame_size(br, h)
                self.render_size(br, h)
            h.allow_high_precision_mv = 0 if h.force_integer_mv else br.f(1)
            # read_interpolation_filter
            if br.f(1):                                       # is_filter_switchable
                h.interpolation_filter = SWITCHABLE
            else:
                h.interpolation_filter = br.f(2)
            h.is_motion_mode_switchable = br.f(1)
            if h.error_resilient_mode or not s.enable_ref_frame_mvs:
                h.use_ref_frame_mvs = 0
            else:
                h.use_ref_frame_mvs = br.f(1)
            for i in range(REFS_PER_FRAME):
                refFrame = LAST_FRAME + i
                hint = self.RefOrderHint[h.ref_frame_idx[i]]
                h.OrderHints[refFrame] = hint
                h.RefFrameSignBias[refFrame] = int(self.get_relative_dist(hint, h.order_hint) > 0) if s.enable_order_hint else 0
        if s.reduced_still_picture_header or h.disable_cdf_update:
            h.disable_frame_end_update_cdf = 1
        else:
            h.disable_frame_end_update_cdf = br.f(1)
        h.loop_filter_delta_enabled = 1
        if h.primary_ref_frame == PRIMARY_REF_NONE:
            # init_non_coeff_cdfs (the tile model loads the defaults) + setup_past_independence
            h.prev_frame_slot = None
            h.loop_filter_ref_deltas = list(DEFAULT_LF_REF_DELTAS)
            h.loop_filter_mode_deltas = [0, 0]
            h.FeatureEnabled = [[0] * SEG_LVL_MAX for _ in range(MAX_SEGMENTS)]
            h.FeatureData = [[0] * SEG_LVL_MAX for _ in range(MAX_SEGMENTS)]
            h.PrevGmParams = [list(GM_IDENTITY) for _ in range(8)]
        else:
            # load_cdfs(ref_frame_idx[primary_ref_frame]) is the tile model's (saved CDFs per slot); load_previous():
            prev = h.ref_frame_idx[h.primary_ref_frame]
            h.prev_frame_slot = prev
            h.PrevGmParams = [list(g) for g in self.SavedGmParams[prev]]
            h.loop_filter_ref_deltas = list(self.SavedLoopFilterRefDeltas[prev])     # load_loop_filter_params
            h.loop_filter_mode_deltas = list(self.SavedLoopFilterModeDeltas[prev])
            h.FeatureEnabled = [list(r) for r in self.SavedFeatureEnabled[prev]]      # load_segmentation_params
            h.FeatureData = [list(r) for r in self.SavedFeatureData[prev]]
        # (motion_field_estimation: the tile model's, from the saved MVs)
        self.tile_info(br, h)
        self.quantization_params(br, h)
        self.segmentation_params(br, h)
        self.delta_q_params(br, h)
        self.delta_lf_params(br, h)
        h.CodedLossless = 1
        h.LosslessArray = [0] * 8
        h.seg_qidx = [0] * 8
        h.SegQMLevel = [[15] * 8 for _ in range(3)]
        for sid in range(MAX_SEGMENTS):
            q = self.get_qindex(h, 1, sid)
            h.seg_qidx[sid] = q
            ll = int(q == 0 and h.DeltaQYDc == 0 and h.DeltaQUAc == 0 and h.DeltaQUDc == 0 and h.DeltaQVAc == 0 and h.DeltaQVDc == 0)
            h.LosslessArray[sid] = ll
            if not ll:
                h.CodedLossless = 0
            if h.using_qmatrix and not ll:
                h.SegQMLevel[0][sid], h.SegQMLevel[1][sid], h.SegQMLevel[2][sid] = h.qm_y, h.qm_u, h.qm_v
        h.AllLossless = int(h.CodedLossless and h.FrameWidth == h.UpscaledWidth)
        self.loop_filter_params(br, h)
        self.cdef_params(br, h)
        self.lr_params(br, h)
        self.read_tx_mode(br, h)
        h.reference_select = 0 if h.FrameIsIntra else br.f(1)      # frame_reference_mode
        self.skip_mode_params(br, h)
        if h.FrameIsIntra or h.error_resilient_mode or not s.enable_warped_motion:
            h.allow_warped_motion = 0
        else:
            h.allow_warped_motion = br.f(1)
        h.reduced_tx_set = br.f(1)
        self.global_motion_params(br, h)
        self.film_grain_params(br, h)
        # derived / renamed for tile_model compatibility
        h.BitDepth = s.BitDepth
        h.NumPlanes = s.NumPlanes
        h.subsampling_x, h.subsampling_y = s.subsampling_x, s.subsampling_y
        h.layout = 0 if s.mono_chrome else (1 if (s.subsampling_x and s.subsampling_y) else (2 if s.subsampling_x else 3))
        h.use_128x128_superblock = s.use_128x128_superblock
        h.enable_filter_intra = s.enable_filter_intra
        h.enable_intra_edge_filter = s.enable_intra_edge_filter
        h.enable_cdef = s.enable_cdef
        h.enable_restoration = s.enable_restoration
        h.enable_superres = s.enable_superres
        h.enable_interintra_compound = s.enable_interintra_compound
        h.enable_masked_compound = s.enable_masked_compound
        h.enable_jnt_comp = s.enable_jnt_comp
        h.enable_dual_filter = s.enable_dual_filter
        h.enable_order_hint = s.enable_order_hint
        h.OrderHintBits = s.OrderHintBits
        h.enable_warped_motion = s.enable_warped_motion
        h.RefUpscaledWidths = [0] * 8                          # by RefFrame (7.11.3.3 is_scaled)
        h.RefFrameHeights = [0] * 8
        if not h.FrameIsIntra:
            for i in range(REFS_PER_FRAME):
                h.RefUpscaledWidths[LAST_FRAME + i] = self.RefUpscaledWidth[h.ref_frame_idx[i]]
                h.RefFrameHeights[LAST_FRAME + i] = self.RefFrameHeight[h.ref_frame_idx[i]]

    @staticmethod
    def get_qindex(h, ignore_delta_q, sid):
        if h.segmentation_enabled and h.FeatureEnabled[sid][0]:
            return clip3(0, 255, h.base_q_idx + h.FeatureData[sid][0])
        return h.base_q_idx

    def frame_size(self, br, h):
        s = self.seq
        if h.frame_size_override_flag:
            h.FrameWidth = br.f(s.frame_width_bits_minus_1 + 1) + 1
            h.FrameHeight = br.f(s.frame_height_bits_minus_1 + 1) + 1
        else:
            h.FrameWidth = s.max_frame_width_minus_1 + 1
            h.FrameHeight = s.max_frame_height_minus_1 + 1
        self.superres_params(br, h)

    def superres_params(self, br, h):
        """superres_params() + compute_image_size(): FrameWidth holds the upscaled width on entry."""
        s = self.seq
        h.use_superres = br.f(1) if s.enable_superres else 0
        if h.use_superres:
            h.SuperresDenom = br.f(SUPERRES_DENOM_BITS) + SUPERRES_DENOM_MIN
        else:
            h.SuperresDenom = SUPERRES_NUM
        h.UpscaledWidth = h.FrameWidth
        h.FrameWidth = (h.UpscaledWidth * SUPERRES_NUM + (h.SuperresDenom // 2)) // h.SuperresDenom
        # compute_image_size
        h.MiCols = 2 * ((h.FrameWidth + 7) >> 3)
        h.MiRows = 2 * ((h.FrameHeight + 7) >> 3)

    def render_size(self, br, h):
        if br.f(1):
            h.RenderWidth = br.f(16) + 1
            h.RenderHeight = br.f(16) + 1
        else:
            h.RenderWidth, h.RenderHeight = h.UpscaledWidth, h.FrameHeight

    def frame_size_with_refs(self, br, h):
        for i in range(REFS_PER_FRAME):
            if br.f(1):                                       # found_ref
                slot = h.ref_frame_idx[i]
                h.FrameWidth = self.RefUpscaledWidth[slot]    # UpscaledWidth, then superres_params derives FrameWidth
                h.FrameHeight = self.RefFrameHeight[slot]
                h.RenderWidth = self.RefRenderWidth[slot]
                h.RenderHeight = self.RefRenderHeight[slot]
                self.superres_params(br, h)
                return
        self.frame_size(br, h)
        self.render_size(br, h)

    def get_relative_dist(self, a, b):
        s = self.seq
        if not s.enable_order_hint:
            return 0
        diff = a - b
        m = 1 << (s.OrderHintBits - 1)
        return (diff & (m - 1)) - (diff & m)

    def set_frame_refs(self, h, last_frame_idx, gold_frame_idx):
        """7.8 set frame refs process: ref_frame_idx[] from the LAST / GOLDEN slots and the slots' order hints."""
        s = self.seq
        idx = [-1] * REFS_PER_FRAME
        idx[LAST_FRAME - LAST_FRAME] = last_frame_idx
        idx[GOLDEN_FRAME - LAST_FRAME] = gold_frame_idx
        used = [0] * NUM_REF_FRAMES
        used[last_frame_idx] = used[gold_frame_idx] = 1
        cur = 1 << (s.OrderHintBits - 1)
        shifted = [cur + self.get_relative_dist(self.RefOrderHint[i], h.order_hint) for i in range(NUM_REF_FRAMES)]

        def find_latest_backward():
            ref, latest = -1, 0
            for i in range(NUM_REF_FRAMES):
                hint = shifted[i]
                if not used[i] and hint >= cur and (ref < 0 or hint >= latest):
                    ref, latest = i, hint
            return ref

        def find_earliest_backward():
            ref, earliest = -1, 0
            for i in range(NUM_REF_FRAMES):
                hint = shifted[i]
                if not used[i] and hint >= cur and (ref < 0 or hint < earliest):
                    ref, earliest = i, hint
            return ref

        def find_latest_forward():
            ref, latest = -1, 0
            for i in range(NUM_REF_FRAMES):
                hint = shifted[i]
                if not used[i] and hint < cur and (ref < 0 or hint >= latest):
                    ref, latest = i, hint
            return ref

        ref = find_latest_backward()
        if ref >= 0:
            idx[ALTREF_FRAME - LAST_FRAME] = ref; used[ref] = 1
        ref = find_earliest_backward()
        if ref >= 0:
            idx[BWDREF_FRAME - LAST_FRAME] = ref; used[ref] = 1
        ref = find_earliest_backward()
        if ref >= 0:
            idx[ALTREF2_FRAME - LAST_FRAME] = ref; used[ref] = 1
        for refFrame in (LAST2_FRAME, LAST3_FRAME, BWDREF_FRAME, ALTREF2_FRAME, ALTREF_FRAME):
            if idx[refFrame - LAST_FRAME] < 0:
                ref = find_latest_forward()
                if ref >= 0:
                    idx[refFrame - LAST_FRAME] = ref; used[ref] = 1
        ref, earliest = -1, 0
        for i in range(NUM_REF_FRAMES):
            hint = shifted[i]
            if ref < 0 or hint < earliest:
                ref, earliest = i, hint
        for i in range(REFS_PER_FRAME):
            if idx[i] < 0:
                idx[i] = ref
        h.ref_frame_idx = idx

    def skip_mode_params(self, br, h):
        s = self.seq
        h.skip_mode_present = 0
        h.SkipModeFrame = [0, 0]
        if h.FrameIsIntra or not h.reference_select or not s.enable_order_hint:
            return
        forwardIdx = backwardIdx = -1
        forwardHint = backwardHint = 0
        for i in range(REFS_PER_FRAME):
            refHint = self.RefOrderHint[h.ref_frame_idx[i]]
            if self.get_relative_dist(refHint, h.order_hint) < 0:
                if forwardIdx < 0 or self.get_relative_dist(refHint, forwardHint) > 0:
                    forwardIdx, forwardHint = i, refHint
            elif self.get_relative_dist(refHint, h.order_hint) > 0:
                if backwardIdx < 0 or self.get_relative_dist(refHint, backwardHint) < 0:
                    backwardIdx, backwardHint = i, refHint
        if forwardIdx < 0:
            return
        if backwardIdx >= 0:
            h.SkipModeFrame = [LAST_FRAME + min(forwardIdx, backwardIdx), LAST_FRAME + max(forwardIdx, backwardIdx)]
        else:
            secondForwardIdx, secondForwardHint = -1, 0
            for i in range(REFS_PER_FRAME):
                refHint = self.RefOrderHint[h.ref_frame_idx[i]]
                if self.get_relative_dist(refHint, forwardHint) < 0:
                    if secondForwardIdx < 0 or self.get_relative_dist(refHint, secondForwardHint) > 0:
                        secondForwardIdx, secondForwardHint = i, refHint
            if secondForwardIdx < 0:
                return
            h.SkipModeFrame = [LAST_FRAME + min(forwardIdx, secondForwardIdx), LAST_FRAME + max(forwardIdx, secondForwardIdx)]
        h.skip_mode_present = br.f(1)

    # ---- 5.9.24 / 5.9.25 global motion
    def global_motion_params(self, br, h):
        h.GmType = [IDENTITY] * 8
        h.gm_params = [list(GM_IDENTITY) for _ in range(8)]
        if h.FrameIsIntra:
            return
        for ref in range(LAST_FRAME, ALTREF_FRAME + 1):
            if br.f(1):                                       # is_global
                if br.f(1):                                   # is_rot_zoom
                    typ = ROTZOOM
                else:
                    typ = TRANSLATION if br.f(1) else AFFINE  # is_translation
            else:
                typ = IDENTITY
            h.GmType[ref] = typ
            if typ >= ROTZOOM:
                self.read_global_param(br, h, typ, ref, 2)
                self.read_global_param(br, h, typ, ref, 3)
                if typ == AFFINE:
                    self.read_global_param(br, h, typ, ref, 4)
                    self.read_global_param(br, h, typ, ref, 5)
                else:
                    h.gm_params[ref][4] = -h.gm_params[ref][3]
                    h.gm_params[ref][5] = h.gm_params[ref][2]
            if typ >= TRANSLATION:
                self.read_global_param(br, h, typ, ref, 0)
                self.read_global_param(br, h, typ, ref, 1)

    def read_global_param(self, br, h, typ, ref, idx):
        absBits, precBits = GM_ABS_ALPHA_BITS, GM_ALPHA_PREC_BITS
        if idx < 2:
            if typ == TRANSLATION:
                absBits = GM_ABS_TRANS_ONLY_BITS - (not h.allow_high_precision_mv)
                precBits = GM_TRANS_ONLY_PREC_BITS - (not h.allow_high_precision_mv)
            else:
                absBits, precBits = GM_ABS_TRANS_BITS, GM_TRANS_PREC_BITS
        precDiff = WARPEDMODEL_PREC_BITS - precBits
        rnd = (1 << WARPEDMODEL_PREC_BITS) if idx % 3 == 2 else 0
        sub = (1 << precBits) if idx % 3 == 2 else 0
        mx = 1 << absBits
        r = (h.PrevGmParams[ref][idx] >> precDiff) - sub
        h.gm_params[ref][idx] = (self.decode_signed_subexp_with_ref(br, -mx, mx + 1, r) << precDiff) + rnd

    def decode_signed_subexp_with_ref(self, br, low, high, r):
        return self.decode_unsigned_subexp_with_ref(br, high - low, r - low) + low

    def decode_unsigned_subexp_with_ref(self, br, mx, r):
        v = self.decode_subexp(br, mx)
        if (r << 1) <= mx:
            return self.inverse_recenter(r, v)
        return mx - 1 - self.inverse_recenter(mx - 1 - r, v)

    @staticmethod
    def decode_subexp(br, numSyms):
        i = mk = 0
        k = 3
        while True:
            b2 = k + i - 1 if i else k
            a = 1 << b2
            if numSyms <= mk + 3 * a:
                return br.ns(numSyms - mk) + mk             # subexp_final_bits
            if br.f(1):                                       # subexp_more_bits
                i += 1
                mk += a
            else:
                return br.f(b2) + mk                          # subexp_bits

    @staticmethod
    def inverse_recenter(r, v):
        if v > 2 * r:
            return v
        if v & 1:
            return r - ((v + 1) >> 1)
        return r + (v >> 1)

    # ---- 7.20 / 7.21 reference frame update and loading (the header-level state; pictures, MVs, segment ids and
    # CDFs are the tile model's)
    def ref_update(self, h):
        s = self.seq
        for i in range(NUM_REF_FRAMES):
            if (h.refresh_frame_flags >> i) & 1:
                self.RefValid[i] = 1
                self.RefFrameId[i] = h.current_frame_id
                self.RefUpscaledWidth[i] = h.UpscaledWidth
                self.RefFrameWidth[i] = h.FrameWidth
                self.RefFrameHeight[i] = h.FrameHeight
                self.RefRenderWidth[i] = h.RenderWidth
                self.RefRenderHeight[i] = h.RenderHeight
                self.RefMiCols[i] = h.MiCols
                self.RefMiRows[i] = h.MiRows
                self.RefFrameType[i] = h.frame_type
                self.RefShowableFrame[i] = h.showable_frame
                self.SavedBitDepth[i] = s.BitDepth
                self.SavedSubsampling[i] = (s.subsampling_x, s.subsampling_y)
                self.RefOrderHint[i] = h.order_hint
                self.SavedOrderHints[i] = list(h.OrderHints)
                self.SavedGmParams[i] = [list(g) for g in h.gm_params]
                self.SavedLoopFilterRefDeltas[i] = list(h.loop_filter_ref_deltas)
                self.SavedLoopFilterModeDeltas[i] = list(h.loop_filter_mode_deltas)
                self.SavedFeatureEnabled[i] = [list(r) for r in h.FeatureEnabled]
                self.SavedFeatureData[i] = [list(r) for r in h.FeatureData]

    def ref_load(self, h, i):
        """7.21: the frame-level state of slot i into h (show_existing_frame)."""
        h.current_frame_id = self.RefFrameId[i]
        h.UpscaledWidth = self.RefUpscaledWidth[i]
        h.FrameWidth = self.RefFrameWidth[i]
        h.FrameHeight = self.RefFrameHeight[i]
        h.RenderWidth = self.RefRenderWidth[i]
        h.RenderHeight = self.RefRenderHeight[i]
        h.MiCols = self.RefMiCols[i]
        h.MiRows = self.RefMiRows[i]
        h.order_hint = self.RefOrderHint[i]
        h.showable_frame = self.RefShowableFrame[i]
        h.OrderHints = list(self.SavedOrderHints[i])
        h.gm_params = [list(g) for g in self.SavedGmParams[i]]
        h.loop_filter_ref_deltas = list(self.SavedLoopFilterRefDeltas[i])
        h.loop_filter_mode_deltas = list(self.SavedLoopFilterModeDeltas[i])
        h.FeatureEnabled = [list(r) for r in self.SavedFeatureEnabled[i]]
        h.FeatureData = [list(r) for r in self.SavedFeatureData[i]]

    @staticmethod
    def tile_log2(blkSize, target):
        k = 0
        while (blkSize << k) < target:
            k += 1
        return k

    def tile_info(self, br, h):
        s = self.seq
        sb128 = s.use_128x128_superblock
        sbCols = (h.MiCols + 31) >> 5 if sb128 else (h.MiCols + 15) >> 4
        sbRows = (h.MiRows + 31) >> 5 if sb128 else (h.MiRows + 15) >> 4
        sbShift = 5 if sb128 else 4
        sbSize = sbShift + 2
        maxTileWidthSb = MAX_TILE_WIDTH >> sbSize
        maxTileAreaSb = MAX_TILE_AREA >> (2 * sbSize)
        minLog2TileCols = self.tile_log2(maxTileWidthSb, sbCols)
        maxLog2TileCols = self.tile_log2(1, min(sbCols, MAX_TILE_COLS))
        maxLog2TileRows = self.tile_log2(1, min(sbRows, MAX_TILE_ROWS))
        minLog2Tiles = max(minLog2TileCols, self.tile_log2(maxTileAreaSb, sbRows * sbCols))
        h.MiColStarts, h.MiRowStarts = [], []
        uniform = br.f(1)
        if uniform:
            TileColsLog2 = minLog2TileCols
            while TileColsLog2 < maxLog2TileCols:
                if br.f(1):
                    TileColsLog2 += 1
                else:
                    break
            tileWidthSb = (sbCols + (1 << TileColsLog2) - 1) >> TileColsLog2
            for startSb in range(0, sbCols, tileWidthSb):
                h.MiColStarts.append(startSb << sbShift)
            h.MiColStarts.append(h.MiCols)
            h.TileCols = len(h.MiColStarts) - 1
            minLog2TileRows = max(minLog2Tiles - TileColsLog2, 0)
            TileRowsLog2 = minLog2TileRows
            while TileRowsLog2 < maxLog2TileRows:
                if br.f(1):
                    TileRowsLog2 += 1
                else:
                    break
            tileHeightSb = (sbRows + (1 << TileRowsLog2) - 1) >> TileRowsLog2
            for startSb in range(0, sbRows, tileHeightSb):
                h.MiRowStarts.append(startSb << sbShift)
            h.MiRowStarts.append(h.MiRows)
            h.TileRows = len(h.MiRowStarts) - 1
        else:
            widestTileSb = 0
            startSb = 0
            while startSb < sbCols:
                h.MiColStarts.append(startSb << sbShift)
                maxWidth = min(sbCols - startSb, maxTileWidthSb)
                sizeSb = br.ns(maxWidth) + 1
                widestTileSb = max(sizeSb, widestTileSb)
                startSb += sizeSb
            h.MiColStarts.append(h.MiCols)
            h.TileCols = len(h.MiColStarts) - 1
            TileColsLog2 = self.tile_log2(1, h.TileCols)
            if minLog2Tiles > 0:
                maxTileAreaSb = (sbRows * sbCols) >> (minLog2Tiles + 1)
            else:
                maxTileAreaSb = sbRows * sbCols
            maxTileHeightSb = max(maxTileAreaSb // widestTileSb, 1)
            startSb = 0
            while startSb < sbRows:
                h.MiRowStarts.append(startSb << sbShift)
                maxHeight = min(sbRows - startSb, maxTileHeightSb)
                sizeSb = br.ns(maxHeight) + 1
                startSb += sizeSb
            h.MiRowStarts.append(h.MiRows)
            h.TileRows = len(h.MiRowStarts) - 1
            TileRowsLog2 = self.tile_log2(1, h.TileRows)
        h.TileColsLog2, h.TileRowsLog2 = TileColsLog2, TileRowsLog2
        if TileColsLog2 > 0 or TileRowsLog2 > 0:
            h.context_update_tile_id = br.f(TileRowsLog2 + TileColsLog2)
            h.TileSizeBytes = br.f(2) + 1
        else:
            h.context_update_tile_id = 0
            h.TileSizeBytes = 4

    def quantization_params(self, br, h):
        s = self.seq
        h.base_q_idx = br.f(8)
        h.DeltaQYDc = self.read_delta_q(br)
        if s.NumPlanes > 1:
            diff_uv_delta = br.f(1) if s.separate_uv_delta_q else 0
            h.DeltaQUDc = self.read_delta_q(br)
            h.DeltaQUAc = self.read_delta_q(br)
            if diff_uv_delta:
                h.DeltaQVDc = self.read_delta_q(br)
                h.DeltaQVAc = self.read_delta_q(br)
            else:
                h.DeltaQVDc, h.DeltaQVAc = h.DeltaQUDc, h.DeltaQUAc
        else:
            h.DeltaQUDc = h.DeltaQUAc = h.DeltaQVDc = h.DeltaQVAc = 0
        h.using_qmatrix = br.f(1)
        h.qm_y = h.qm_u = h.qm_v = 0
        if h.using_qmatrix:
            h.qm_y = br.f(4)
            h.qm_u = br.f(4)
            h.qm_v = h.qm_u if not s.separate_uv_delta_q else br.f(4)

    @staticmethod
    def read_delta_q(br):
        return br.su(7) if br.f(1) else 0

    def segmentation_params(self, br, h):
        h.segmentation_enabled = br.f(1)
        h.segmentation_update_map = h.segmentation_temporal_update = 0
        if h.segmentation_enabled:
            if h.primary_ref_frame == PRIMARY_REF_NONE:
                h.segmentation_update_map = 1
                h.segmentation_temporal_update = 0
                update_data = 1
            else:                                             # FeatureEnabled / FeatureData hold the loaded values
                h.segmentation_update_map = br.f(1)
                if h.segmentation_update_map:
                    h.segmentation_temporal_update = br.f(1)
                update_data = br.f(1)
            if update_data:
                for i in range(MAX_SEGMENTS):
                    for j in range(SEG_LVL_MAX):
                        en = br.f(1)
                        h.FeatureEnabled[i][j] = en
                        val = 0
                        if en:
                            bits = SEGMENTATION_FEATURE_BITS[j]
                            lim = SEGMENTATION_FEATURE_MAX[j]
                            if SEGMENTATION_FEATURE_SIGNED[j]:
                                val = clip3(-lim, lim, br.su(1 + bits))
                            else:
                                val = clip3(0, lim, br.f(bits))
                        h.FeatureData[i][j] = val
        else:
            h.FeatureEnabled = [[0] * SEG_LVL_MAX for _ in range(MAX_SEGMENTS)]
            h.FeatureData = [[0] * SEG_LVL_MAX for _ in range(MAX_SEGMENTS)]
        h.SegIdPreSkip = 0
        h.LastActiveSegId = 0
        for i in range(MAX_SEGMENTS):
            for j in range(SEG_LVL_MAX):
                if h.FeatureEnabled[i][j]:
                    h.LastActiveSegId = i
                    if j >= SEG_LVL_REF_FRAME:
                        h.SegIdPreSkip = 1

    def delta_q_params(self, br, h):
        h.delta_q_res = 0
        h.delta_q_present = 0
        if h.base_q_idx > 0:
            h.delta_q_present = br.f(1)
        if h.delta_q_present:
            h.delta_q_res = br.f(2)

    def delta_lf_params(self, br, h):
        h.delta_lf_present = h.delta_lf_res = h.delta_lf_multi = 0
        if h.delta_q_present:
            if not h.allow_intrabc:
                h.delta_lf_present = br.f(1)
            if h.delta_lf_present:
                h.delta_lf_res = br.f(2)
                h.delta_lf_multi = br.f(1)

    def loop_filter_params(self, br, h):
        s = self.seq
        h.loop_filter_level = [0, 0, 0, 0]
        h.loop_filter_sharpness = 0
        if h.CodedLossless or h.allow_intrabc:
            h.loop_filter_ref_deltas = [1, 0, 0, 0, -1, 0, -1, -1]
            h.loop_filter_mode_deltas = [0, 0]
            return
        h.loop_filter_level[0] = br.f(6)
        h.loop_filter_level[1] = br.f(6)
        if s.NumPlanes > 1 and (h.loop_filter_level[0] or h.loop_filter_level[1]):
            h.loop_filter_level[2] = br.f(6)
            h.loop_filter_level[3] = br.f(6)
        h.loop_filter_sharpness = br.f(3)
        h.loop_filter_delta_enabled = br.f(1)
        if h.loop_filter_delta_enabled:
            if br.f(1):                               # loop_filter_delta_update
                for i in range(TOTAL_REFS_PER_FRAME):
                    if br.f(1):
                        h.loop_filter_ref_deltas[i] = br.su(7)
                for i in range(2):
                    if br.f(1):
                        h.loop_filter_mode_deltas[i] = br.su(7)

    def cdef_params(self, br, h):
        s = self.seq
        h.cdef_y_strengths = [0] * 8
        h.cdef_uv_strengths = [0] * 8
        if h.CodedLossless or h.allow_intrabc or not s.enable_cdef:
            h.cdef_bits = 0
            h.cdef_damping = 3
            return
        h.cdef_damping = br.f(2) + 3
        h.cdef_bits = br.f(2)
        for i in range(1 << h.cdef_bits):
            pri = br.f(4)
            sec = br.f(2)
            h.cdef_y_strengths[i] = (pri << 2) | sec         # packed like dav1d (sec 3 means 4)
            if s.NumPlanes > 1:
                pri = br.f(4)
                sec = br.f(2)
                h.cdef_uv_strengths[i] = (pri << 2) | sec

    def lr_params(self, br, h):
        s = self.seq
        h.FrameRestorationType = [RESTORE_NONE] * 3
        h.LoopRestorationSize = [RESTORATION_TILESIZE_MAX] * 3
        h.UsesLr = 0
        if h.AllLossless or h.allow_intrabc or not s.enable_restoration:
            return
        usesChromaLr = 0
        for i in range(s.NumPlanes):
            lr_type = br.f(2)
            h.FrameRestorationType[i] = REMAP_LR_TYPE[lr_type]
            if h.FrameRestorationType[i] != RESTORE_NONE:
                h.UsesLr = 1
                if i > 0:
                    usesChromaLr = 1
        if h.UsesLr:
            if s.use_128x128_superblock:
                lr_unit_shift = br.f(1) + 1
            else:
                lr_unit_shift = br.f(1)
                if lr_unit_shift:
                    lr_unit_shift += br.f(1)
            h.LoopRestorationSize[0] = RESTORATION_TILESIZE_MAX >> (2 - lr_unit_shift)
            lr_uv_shift = br.f(1) if (s.subsampling_x and s.subsampling_y and usesChromaLr) else 0
            h.LoopRestorationSize[1] = h.LoopRestorationSize[0] >> lr_uv_shift
            h.LoopRestorationSize[2] = h.LoopRestorationSize[0] >> lr_uv_shift

    def read_tx_mode(self, br, h):
        if h.CodedLossless:
            h.TxMode = 0
        else:
            h.TxMode = 2 if br.f(1) else 1

    def film_grain_params(self, br, h):
        s = self.seq
        h.apply_grain = 0
        if not s.film_grain_params_present or (not h.show_frame and not h.showable_frame):
            return
        h.apply_grain = br.f(1)
        if not h.apply_grain:
            return
        br.f(16)                                          # grain_seed
        update_grain = br.f(1) if h.frame_type == INTER_FRAME else 1
        if not update_grain:
            br.f(3)
            return
        num_y = br.f(4)
        for _ in range(num_y):
            br.f(8); br.f(8)
        csfl = 0 if s.mono_chrome else br.f(1)
        if s.mono_chrome or csfl or (s.subsampling_x == 1 and s.subsampling_y == 1 and num_y == 0):
            num_cb = num_cr = 0
        else:
            num_cb = br.f(4)
            for _ in range(num_cb):
                br.f(8); br.f(8)
            num_cr = br.f(4)
            for _ in range(num_cr):
                br.f(8); br.f(8)
        br.f(2)                                           # grain_scaling_minus_8
        lag = br.f(2)
        numPosLuma = 2 * lag * (lag + 1)
        if num_y:
            numPosChroma = numPosLuma + 1
            for _ in range(numPosLuma):
                br.f(8)
        else:
            numPosChroma = numPosLuma
        if csfl or num_cb:
            for _ in range(numPosChroma):
                br.f(8)
        if csfl or num_cr:
            for _ in range(numPosChroma):
                br.f(8)
        br.f(2); br.f(2)                                  # ar_coeff_shift_minus_6, grain_scale_shift
        if num_cb:
            br.f(8); br.f(8); br.f(9)
        if num_cr:
            br.f(8); br.f(8); br.f(9)
        br.f(1); br.f(1)                                  # overlap_flag, clip_to_restricted_range

    # ---- 5.11.1 tile group -------------------------------------------------------------------------------
    def tile_group_obu(self, br, payload):
        h = self.cur
        NumTiles = h.TileCols * h.TileRows
        startBitPos = br.pos
        tile_start_and_end_present_flag = br.f(1) if NumTiles > 1 else 0
        if NumTiles == 1 or not tile_start_and_end_present_flag:
            tg_start, tg_end = 0, NumTiles - 1
        else:
            tileBits = h.TileColsLog2 + h.TileRowsLog2
            tg_start = br.f(tileBits)
            tg_end = br.f(tileBits)
        br.byte_alignment()
        pos = br.byte_pos()
        sz = len(payload) - pos
        if tg_start == 0:
            self.frames.append([])
        for TileNum in range(tg_start, tg_end + 1):
            tileRow = TileNum // h.TileCols
            tileCol = TileNum % h.TileCols
            last = TileNum == tg_end
            if last:
                tileSize = sz
            else:
                tileSize = BitReader(payload, pos).le(h.TileSizeBytes) + 1
                pos += h.TileSizeBytes
                sz -= tileSize + h.TileSizeBytes
            th = self.tile_header(h, tileRow, tileCol)
            self.tiles.append((th, payload[pos:pos + tileSize]))
            self.frames[-1].append(len(self.tiles) - 1)
            pos += tileSize
        if tg_end == NumTiles - 1:
            self.SeenFrameHeader = 0
            self.ref_update(h)                                # 7.20: the decoded frame into its slots

    def tile_header(self, h, tileRow, tileCol):
        """A per-tile copy of the frame header with the tile bounds, shaped like tile_model.FrameHeader."""
        import copy
        t = copy.copy(h)
        t.tile_row, t.tile_col = tileRow, tileCol
        t.MiRowStart, t.MiRowEnd = h.MiRowStarts[tileRow], h.MiRowStarts[tileRow + 1]
        t.MiColStart, t.MiColEnd = h.MiColStarts[tileCol], h.MiColStarts[tileCol + 1]
        return t


def parse_file(path):
    data = open(path, "rb").read()
    d = Decoder()
    if data[:4] == b"DKIF":
        d.feed_ivf(data)
    elif path.endswith(".obu") or path.endswith(".av1"):
        # try Annex B first (Argon), fall back to low-overhead
        try:
            d.feed_annexb(data)
        except Exception:
            d = Decoder()
            d.feed_temporal_unit(data)
    else:
        d.feed_temporal_unit(data)
    return d
