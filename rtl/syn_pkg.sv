// Shared types for the tile syntax decoder.
package syn_pkg;
    // Coordinate widths. Every position and count in the decoder is sized from these three, so a taller or wider
    // frame needs only the frame-buffer parameters (FBX/FBY -> ML2R/ML2C) raised to match: the widths below are
    // the hard ceiling the hardware can address at all. Each covers both coordinates (0 .. 2^N - 1) and the
    // matching count (mi_rows, frame_height, ...), which is why they hold one bit more than the coordinate alone.
    localparam int MIW = 13;         // 4x4 (MI) row/col: mi_rows / mi_cols up to 4096 = 16384 px (FBY/FBX = 14)
    localparam int SBW = 9;          // 64x64 superblock rows / cols: up to 256 = 16384 px
    localparam int PXW = 15;         // pixel row/col: frame_height / upscaled_width up to 16384
    localparam int MIS = MIW + 2;    // signed MI position with headroom for neighbour offsets (-32 .. +)
    localparam int PXS = PXW + 2;    // signed pixel position with headroom (filter taps / MV offsets below 0)

    // Frame/tile-level parameters the syntax needs (software parses the headers and fills this in).
    typedef struct packed {
        logic [MIW-1:0] mi_rows;
        logic [MIW-1:0] mi_cols;
        logic [MIW-1:0] mi_row_start;
        logic [MIW-1:0] mi_row_end;
        logic [MIW-1:0] mi_col_start;
        logic [MIW-1:0] mi_col_end;
        logic        ssx;
        logic        ssy;
        logic        mono;
        logic        sb128;
        logic [3:0]  bit_depth;
        logic        seg_enabled;
        logic        seg_preskip;              // SegIdPreSkip
        logic [2:0]  last_active_segid;
        logic [7:0]  seg_skip_en;              // FeatureEnabled[s][SEG_LVL_SKIP]
        logic [7:0]  lossless;                 // LosslessArray[s]
        logic [63:0] seg_qidx;                 // get_qindex(1, s) per segment, 8 x 8 bits
        logic [7:0]  base_q_idx;
        logic [1:0]  tx_mode;                  // 0 ONLY_4X4, 1 LARGEST, 2 SELECT
        logic        reduced_tx_set;
        logic        allow_sct;                // allow_screen_content_tools
        logic        allow_intrabc;            // use_intrabc is read per block
        logic        enable_filter_intra;
        logic        enable_cdef;
        logic [1:0]  cdef_bits;
        logic        coded_lossless;
        logic        delta_q_present;
        logic [1:0]  delta_q_res;
        logic        delta_lf_present;
        logic [1:0]  delta_lf_res;
        logic        delta_lf_multi;
        logic        disable_cdf_update;
        logic [5:0]  lr_type;                  // FrameRestorationType[p] at [2p+1:2p] (0 NONE, 1 WIENER, 2 SGRPROJ, 3 SWITCHABLE)
        logic [5:0]  lr_size;                  // log2(LoopRestorationSize[p]) - 5 at [2p+1:2p] (32 .. 256; chroma units can be 32)
        logic [PXW-1:0] frame_height;
        logic [PXW-1:0] upscaled_width;
        logic        use_superres;             // 7.16 upscaling: LR unit columns scale by SuperresDenom / SUPERRES_NUM
        logic [4:0]  superres_denom;           // SuperresDenom (9..16)
    } hdr_t;

    // Inter frame header (software fills this in; zero for intra frames). 64-word host window.
    typedef struct packed {
        logic        is_inter;                 // !FrameIsIntra
        logic        reference_select;
        logic        skip_mode_present;
        logic [2:0]  skip_mode_frame0;         // SkipModeFrame[0..1] (LAST_FRAME .. ALTREF_FRAME = 1 .. 7)
        logic [2:0]  skip_mode_frame1;
        logic        allow_hp_mv;              // allow_high_precision_mv
        logic        force_integer_mv;         // (cur_frame_force_integer_mv)
        logic [2:0]  interp_filter;            // interpolation_filter (4 = SWITCHABLE)
        logic        motion_mode_switchable;   // is_motion_mode_switchable
        logic        allow_warped_motion;
        logic        use_ref_frame_mvs;
        logic        enable_dual_filter;
        logic        enable_interintra_compound;
        logic        enable_masked_compound;
        logic        enable_jnt_comp;
        logic        enable_order_hint;
        logic [2:0]  order_hint_bits;          // OrderHintBits
        logic [7:0]  order_hint;               // OrderHint of this frame
        logic [55:0] order_hints;              // OrderHints[LAST_FRAME + i] at [8i +: 8]
        logic [6:0]  ref_sign_bias;            // RefFrameSignBias[LAST_FRAME + i] at bit i
        logic [6:0]  ref_scaled;               // is_scaled(LAST_FRAME + i) (7.11.3.3)
        logic [13:0] gm_type;                  // GmType[LAST_FRAME + i] at [2i +: 2]
        logic [1007:0] gm_params;              // gm_params[LAST_FRAME + i][j], signed 24 bits at [24 (6i + j) +: 24]
        logic [7:0]  seg_ref_en;               // FeatureEnabled[s][SEG_LVL_REF_FRAME]
        logic [23:0] seg_ref_data;             // FeatureData[s][SEG_LVL_REF_FRAME] at [3s +: 3]
        logic [7:0]  seg_globalmv_en;          // FeatureEnabled[s][SEG_LVL_GLOBALMV]
        logic        seg_update_map;           // segmentation_update_map (1 when primary_ref_frame is NONE)
        logic        seg_temporal_update;      // segmentation_temporal_update
    } ihdr_t;

    // Frame-level parameters only the reconstruction stage needs (software fills this in).
    typedef struct packed {
        logic        enable_intra_edge_filter;
        logic signed [6:0] dq_ydc;             // DeltaQYDc, DeltaQUDc, DeltaQUAc, DeltaQVDc, DeltaQVAc
        logic signed [6:0] dq_udc;
        logic signed [6:0] dq_uac;
        logic signed [6:0] dq_vdc;
        logic signed [6:0] dq_vac;
        logic [7:0]  seg_altq_en;              // FeatureEnabled[s][SEG_LVL_ALT_Q]
        logic [71:0] seg_altq;                 // FeatureData[s][SEG_LVL_ALT_Q], 8 x signed 9 bits
        logic        using_qmatrix;
        logic [95:0] qm_level;                 // SegQMLevel[plane][seg] at [(plane*8+seg)*4 +: 4]
    } rec_hdr_t;

    // Frame-level parameters of the deblocking loop filter (spec 7.14; software fills this in).
    typedef struct packed {
        logic [PXW-1:0] frame_width;              // FrameWidth (pixels; edges at x >= FrameWidth are not filtered)
        logic [PXW-1:0] frame_height;
        logic [23:0] level;                    // loop_filter_level[0..3], 6 bits each at [6i +: 6]
        logic [2:0]  sharpness;
        logic        delta_enabled;            // loop_filter_delta_enabled
        logic signed [6:0] ref_delta_intra;    // loop_filter_ref_deltas[INTRA_FRAME]
        logic        delta_lf_multi;
        logic [31:0] seg_lf_en;                // FeatureEnabled[seg][SEG_LVL_ALT_LF_Y_V + i] at [8i + seg]
        logic [223:0] seg_lf_data;             // FeatureData[seg][SEG_LVL_ALT_LF_Y_V + i], signed 7 bits at [7 (8i + seg) +: 7]
    } lf_hdr_t;

    // Frame-level CDEF parameters (spec 7.15; software fills this in; only run when enable_cdef && !CodedLossless && !allow_intrabc).
    typedef struct packed {
        logic [2:0]  damping;                  // cdef_damping (3..6)
        logic [1:0]  bits;                     // cdef_bits
        logic [47:0] y_str;                    // cdef_y_strengths[i] = (pri << 2) | sec at [6i +: 6]
        logic [47:0] uv_str;
    } cdef_hdr_t;

    // Per-4x4 loop-filter state kept by mi_store (from the block records).
    typedef struct packed {
        logic [4:0]  bsize;
        logic        skip;
        logic [2:0]  seg;
        logic [27:0] delta_lf;                 // 4 x signed 7 bits
    } mi_lf_t;

    // Per loop-restoration-unit record (read_lr_unit).
    typedef struct packed {
        logic [1:0]  plane;
        logic [SBW-1:0] unit_row;              // LR units are >= 64 px, so unit positions / counts fit the superblock width
        logic [SBW-1:0] unit_col;
        logic [1:0]  lr_type;                  // RESTORE_NONE / WIENER / SGRPROJ
        logic [41:0] wiener;                   // [pass][tap] signed 7 bits: pass1 tap2 .. pass0 tap0 (LSB)
        logic [3:0]  sgr_set;
        logic [15:0] xqd;                      // 2 x signed 8 bits, xqd[1] in the upper byte
    } lr_rec_t;

    // Per-block record emitted by the syntax decoder.
    typedef struct packed {
        logic [MIW-1:0] r;
        logic [MIW-1:0] c;
        logic [4:0]  bsize;
        logic        skip;
        logic [2:0]  seg;
        logic        lossless;
        logic        has_chroma;
        logic [4:0]  ymode;                    // YMode: intra 0 .. 12, inter 13 .. 24
        logic [3:0]  uvmode;
        logic signed [2:0] angle_y;
        logic signed [2:0] angle_uv;
        logic signed [5:0] cfl_u;
        logic signed [5:0] cfl_v;
        logic        use_fi;
        logic [2:0]  fi_mode;
        logic [4:0]  txsz;
        logic [7:0]  qidx;                     // CurrentQIndex after this block's delta
        logic [27:0] delta_lf;                 // 4 x signed 7 bits
        logic        cdef_valid;
        logic [2:0]  cdef_idx;
        logic [3:0]  cdef_units;               // 64x64 units of the superblock this cdef_idx applies to
        logic [3:0]  pal_y;                    // palette sizes (0 = no palette)
        logic [3:0]  pal_uv;
        logic [95:0] col_y;                    // sorted palette colours, colour k at [12k +: 12]
        logic [95:0] col_u;
        logic [95:0] col_v;                    // (not sorted)
        logic        is_inter;                 // inter block or intra block copy (use_intrabc)
        logic        intrabc;                  // use_intrabc
        logic signed [17:0] mv_row;            // Mv[0] in 1/8 luma samples (integer for intrabc)
        logic signed [17:0] mv_col;
        // inter blocks (5.11.23 .. 5.11.33); zero otherwise
        logic        skip_mode;
        logic [3:0]  ref0;                     // RefFrame[0..1]: 0 INTRA_FRAME, 1 .. 7 LAST .. ALTREF, 15 NONE
        logic [3:0]  ref1;
        logic signed [17:0] mv1_row;           // Mv[1]
        logic signed [17:0] mv1_col;
        logic [1:0]  motion_mode;              // 0 SIMPLE, 1 OBMC, 2 LOCALWARP
        logic [2:0]  compound_type;            // 0 WEDGE, 1 DIFFWTD, 2 AVERAGE, 3 INTRA, 4 DISTANCE
        logic        interintra;
        logic [1:0]  interintra_mode;
        logic        wedge_interintra;
        logic [3:0]  wedge_index;
        logic        wedge_sign;
        logic        mask_type;
        logic        comp_group_idx;
        logic        compound_idx;
        logic [1:0]  interp0;                  // interp_filter[dir]
        logic [1:0]  interp1;
        // chroma of a sub-8x8 luma block (7.11.3.1 / compute_prediction: a 4-wide block with 4:2:x or a 4-high block
        // with x:2:0): someUseIntra over the covered luma 4x4s, and when it is clear the motion of the luma 4x4 each
        // chroma prediction unit takes (unit k at [76k +: 76]: ref0[3:0], ref1[7:4], mv0 row[23:8] col[39:24],
        // mv1 row[55:40] col[71:56] (signed 16), interp0[73:72], interp1[75:74]); units in row-major order, unused zero
        logic        cmv_intra;
        logic [303:0] cmv;
        // OBMC (7.11.3.10): the overlap neighbours, above entries 0 .. 3 then left 4 .. 7, entry k at [51k +: 51]:
        // valid[0], offset from the block in 4x4s[5:1], step4[10:6], ref0[14:11], mv row[30:15] col[46:31] (signed 16),
        // interp0[48:47], interp1[50:49]; zero unless motion_mode == OBMC
        logic [407:0] obmc;
        // LOCALWARP blocks (stage 2c.2): the least-squares sums of 7.11.3.8 over the warp samples (A00, A01, A11,
        // Bx0, Bx1, By0, By1, signed 32 bits each, A00 at [31:0]); zero for every other block
        logic [223:0] ls;
    } blk_rec_t;

    // Per-4x4 motion information of the current frame (spec IsInters / RefFrames / Mvs / MiSizes as the MV
    // stack needs them); "written this frame" is a separate bitmap in mv_mem, cleared per superblock row
    // Motion info of one 4x4 unit of the current frame (mv_mem): what the MV stack, the warp-sample and OBMC scans and
    // the inter neighbour contexts read back (spec IsInters / RefFrames / Mvs / YModes (two bits of it) / MiSizes /
    // SkipModes / CompGroupIdxs / CompoundIdxs / InterpFilters / Above|LeftSegPredContext). 85 bits.
    typedef struct packed {
        logic        is_inter;                 // IsInters (intra block copy included)
        logic        is_intrabc;
        logic [3:0]  ref0;                     // RefFrames[0]: 0 INTRA_FRAME, 1 .. 7 LAST_FRAME .. ALTREF_FRAME
        logic [3:0]  ref1;                     // RefFrames[1]: 15 = NONE
        logic signed [15:0] mv_row;            // Mvs[0] (|mv| < 1 << 14 by conformance; 16 bits hold it)
        logic signed [15:0] mv_col;
        logic signed [15:0] mv1_row;           // Mvs[1]
        logic signed [15:0] mv1_col;
        logic        newmv_fam;                // YMode has a NEWMV component (NEWMV, NEW_NEWMV, NEAR_NEWMV, NEW_NEARMV, NEAREST_NEWMV, NEW_NEARESTMV)
        logic        globalmv_fam;             // YMode is GLOBALMV or GLOBAL_GLOBALMV
        logic [4:0]  bsize;
        logic        skip_mode;
        logic        comp_group_idx;
        logic        compound_idx;
        logic [1:0]  interp0;                  // InterpFilters[dir]
        logic [1:0]  interp1;
        logic        seg_pred;                 // seg_id_predicted of the block (Above / LeftSegPredContext)
    } mv_ent_t;

    // Per-transform-block record; Quant is read through coef_rd's port while tx_done is held.
    typedef struct packed {
        logic [1:0]  plane;
        logic [PXW-1:0] x;                        // start pixel (plane units)
        logic [PXW-1:0] y;
        logic [4:0]  txsz;
        logic [3:0]  txtype;
        logic [10:0] eob;
        logic        skip;                     // block skip: no coefficients
        logic        lossless;
        logic [2:0]  slot;                     // coefficient slot in coef_rd (rec_fifo)
    } tx_rec_t;
endpackage
