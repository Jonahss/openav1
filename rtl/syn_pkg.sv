// Shared types for the tile syntax decoder.
package syn_pkg;
    // Frame/tile-level parameters the syntax needs (software parses the headers and fills this in).
    typedef struct packed {
        logic [10:0] mi_rows;
        logic [10:0] mi_cols;
        logic [10:0] mi_row_start;
        logic [10:0] mi_row_end;
        logic [10:0] mi_col_start;
        logic [10:0] mi_col_end;
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
        logic        allow_sct;                // allow_screen_content_tools (palette: not yet supported)
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
        logic [5:0]  lr_size;                  // log2(LoopRestorationSize[p]) - 6 at [2p+1:2p]
        logic [12:0] frame_height;
        logic [12:0] upscaled_width;
    } hdr_t;

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
        logic [12:0] frame_width;              // FrameWidth (pixels; edges at x >= FrameWidth are not filtered)
        logic [12:0] frame_height;
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
        logic [7:0]  unit_row;
        logic [7:0]  unit_col;
        logic [1:0]  lr_type;                  // RESTORE_NONE / WIENER / SGRPROJ
        logic [41:0] wiener;                   // [pass][tap] signed 7 bits: pass1 tap2 .. pass0 tap0 (LSB)
        logic [3:0]  sgr_set;
        logic [15:0] xqd;                      // 2 x signed 8 bits, xqd[1] in the upper byte
    } lr_rec_t;

    // Per-block record emitted by the syntax decoder.
    typedef struct packed {
        logic [10:0] r;
        logic [10:0] c;
        logic [4:0]  bsize;
        logic        skip;
        logic [2:0]  seg;
        logic        lossless;
        logic        has_chroma;
        logic [3:0]  ymode;
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
    } blk_rec_t;

    // Per-transform-block record; Quant is read through coef_rd's port while tx_done is held.
    typedef struct packed {
        logic [1:0]  plane;
        logic [12:0] x;                        // start pixel (plane units)
        logic [12:0] y;
        logic [4:0]  txsz;
        logic [3:0]  txtype;
        logic [10:0] eob;
        logic        skip;                     // block skip: no coefficients
        logic        lossless;
    } tx_rec_t;
endpackage
