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
        self.frames = []           # list of lists of tile indices
        self.RefValid = [0] * 8
        self.RefFrameType = [0] * 8
        self.RefOrderHint = [0] * 8

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
    def frame_header_obu(self, br, payload):
        if self.SeenFrameHeader:
            return                                # frame_header_copy: identical; nothing to do
        self.SeenFrameHeader = 1
        self.uncompressed_header(br)
        h = self.cur
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
                    br.f(idLen)
                h.frame_type = self.RefFrameType[h.frame_to_show_map_idx]
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
                    br.f(s.OrderHintBits)
        if h.FrameIsIntra:
            self.frame_size(br, h)
            self.render_size(br, h)
            if h.allow_screen_content_tools and h.UpscaledWidth == h.FrameWidth:
                h.allow_intrabc = br.f(1)
        else:
            raise NotImplementedError("inter frame headers are not parsed")
        if s.reduced_still_picture_header or h.disable_cdf_update:
            h.disable_frame_end_update_cdf = 1
        else:
            h.disable_frame_end_update_cdf = br.f(1)
        # primary_ref_frame == NONE here (intra): setup_past_independence
        h.loop_filter_delta_enabled = 1
        h.loop_filter_ref_deltas = [1, 0, 0, 0, -1, 0, -1, -1]
        h.loop_filter_mode_deltas = [0, 0]
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
        h.reference_select = 0                        # frame_reference_mode: intra
        h.skip_mode_present = 0                       # skip_mode_params: intra
        h.allow_warped_motion = 0
        h.reduced_tx_set = br.f(1)
        # global_motion_params: nothing read for intra frames
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
        # superres_params
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
        h.FeatureEnabled = [[0] * 8 for _ in range(8)]
        h.FeatureData = [[0] * 8 for _ in range(8)]
        h.segmentation_update_map = h.segmentation_temporal_update = 0
        if h.segmentation_enabled:
            # primary_ref_frame == NONE for intra frames
            h.segmentation_update_map = 1
            h.segmentation_temporal_update = 0
            update_data = 1
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
