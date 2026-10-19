// find_mv_stack (7.10.2) for one block: single or compound reference, inter or intra (block copy) frame.
// Builds RefStackMv / WeightStack with the spec's processes in the spec's order: global motion vectors (7.10.2.1),
// the scans of row -1, col -1 and the top-right point (candidates with the neighbour's reference(s) matching,
// GLOBALMV neighbours substituted by the global vector), the nearest bookkeeping (REF_CAT_LEVEL), the temporal
// candidates from the motion field band (7.10.2.5 / .6), the top-left point, rows -3 / -5, cols -3 / -5, the two
// sorts (7.10.2.11), the extra search when fewer than two candidates were found (7.10.2.12: sign-bias-flipped
// neighbours; compound combinations of same-reference and different-reference neighbours), the contexts
// (NewMvContext / RefMvContext / ZeroMvContext / DrlCtxStack) and the clamping (7.10.2.14). PredMv for intra block
// copy (assign_mv's stack[0] | stack[1] | fallback) is delivered as before.
// Neighbour data comes from mv_mem (mv_ent_t, one registered read per scanned 4x4 position); the motion field
// from mf_band (one registered read per 8x8 position and reference). tb/tile_model.find_mv_stack is the reference.
module mv_stack
  import syn_pkg::*;
  import blk_tables_pkg::*;
(
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  ihdr_t       ihdr,
    input  logic        start,
    input  logic [MIW-1:0] r, c,                 // MiRow, MiCol
    input  logic [4:0]  bs,                   // MiSize
    input  logic [3:0]  ref0, ref1,           // RefFrame[0..1]: 0 INTRA_FRAME, 1 .. 7 LAST .. ALTREF, 15 NONE
    input  logic        is_comp,              // RefFrame[1] > INTRA_FRAME
    output logic        busy,
    output logic        done,                 // 1-cycle pulse; every output valid from then until the next start
    output logic signed [17:0] pred_row, pred_col,          // intra block copy: assign_mv's PredMv
    output logic [3:0]  n_found,              // NumMvFound
    output logic signed [17:0] st0_row [4], st0_col [4],    // RefStackMv[0..3][0] (clamped)
    output logic signed [17:0] st1_row [4], st1_col [4],    // RefStackMv[0..3][1]
    output logic signed [17:0] gmv_row [2], gmv_col [2],    // GlobalMvs
    output logic [2:0]  new_mv_ctx, ref_mv_ctx,
    output logic        zero_mv_ctx,
    output logic [1:0]  drl_ctx [3],          // DrlCtxStack[0..2]
    // mv_mem read port (entry and its written-this-frame bit valid the cycle after the address)
    output logic [MIW-1:0] rd_row, rd_col,
    input  mv_ent_t     rd_ent,
    input  logic        rd_written,
    // motion field band read port (1-cycle latency)
    output logic [2:0]  mf_ref,
    output logic [MIW-2:0] mf_y8, mf_x8,
    input  logic        mf_valid,
    input  logic signed [15:0] mf_row, mf_col
);
    localparam int REF_CAT_LEVEL = 640;
    localparam int MV_BORDER = 128;
    localparam int INTRABC_DELAY_PIXELS = 256;
    localparam logic [1:0] TRANSLATION = 2'd1;

    // ---------------------------------------------------------------- block geometry
    logic [5:0] bw4, bh4;
    assign bw4 = num4x4w(bs);
    assign bh4 = num4x4h(bs);
    logic [MIW-1:0] mi_rows, mi_cols;
    assign mi_rows = hdr.mi_rows;
    assign mi_cols = hdr.mi_cols;

    function automatic logic is_inside(input logic signed [MIS-1:0] rr, input logic signed [MIS-1:0] cc);
        is_inside = (cc >= MIS'(signed'({2'b0, hdr.mi_col_start}))) && (cc < MIS'(signed'({2'b0, hdr.mi_col_end}))) &&
                    (rr >= MIS'(signed'({2'b0, hdr.mi_row_start}))) && (rr < MIS'(signed'({2'b0, hdr.mi_row_end})));
    endfunction

    // ---------------------------------------------------------------- header lookups by reference frame (1 .. 7)
    function automatic logic [1:0] gmt(input logic [3:0] rf);            // GmType[rf] (IDENTITY for INTRA / NONE)
        gmt = (rf >= 4'd1 && rf <= 4'd7) ? ihdr.gm_type[2 * (rf - 4'd1) +: 2] : 2'd0;
    endfunction
    function automatic logic sbias(input logic [3:0] rf);                // RefFrameSignBias[rf]
        sbias = (rf >= 4'd1 && rf <= 4'd7) ? ihdr.ref_sign_bias[3'(rf - 4'd1)] : 1'b0;
    endfunction
    function automatic logic signed [23:0] gmp(input logic [3:0] rf, input int j);   // gm_params[rf][j]
        int k;
        k = 24 * (6 * (int'(rf) - 1) + j);
        gmp = signed'(ihdr.gm_params[k +: 24]);
    endfunction

    // lower_mv_precision (7.10.2.10)
    function automatic logic signed [17:0] lower_prec(input logic signed [17:0] v);
        logic [17:0] a, a_int;
        if (ihdr.allow_hp_mv) lower_prec = v;
        else if (ihdr.force_integer_mv) begin
            a = (v < 0) ? 18'(-v) : 18'(v);
            a_int = (a + 18'd3) >> 3;
            lower_prec = (v > 0) ? 18'(signed'(a_int << 3)) : -18'(signed'(a_int << 3));
        end else if (v[0]) lower_prec = (v > 0) ? v - 18'sd1 : v + 18'sd1;
        else lower_prec = v;
    endfunction
    // round2signed on the global motion sums
    function automatic logic signed [43:0] r2s(input logic signed [43:0] v, input int n);
        logic signed [43:0] a;
        a = (v < 0) ? -v : v;
        a = (a + (44'sd1 <<< (n - 1))) >>> n;
        r2s = (v < 0) ? -a : a;
    endfunction

    // ---------------------------------------------------------------- the stack
    logic signed [17:0] st_row0 [8], st_col0 [8], st_row1 [8], st_col1 [8];
    logic [15:0] weight [8];
    logic [3:0]  n_nearest;
    logic [5:0]  newmv_count, num_new;
    logic        found_above, found_left, fm;        // fm: FoundMatch of the scan in progress
    logic [1:0]  close_matches;
    logic signed [17:0] g_row [2], g_col [2];        // GlobalMvs
    logic        zero_ctx;
    logic [1:0]  drl [3];
    logic [2:0]  nctx, rctx;
    always_comb begin
        for (int k = 0; k < 4; k++) begin st0_row[k] = st_row0[k]; st0_col[k] = st_col0[k]; st1_row[k] = st_row1[k]; st1_col[k] = st_col1[k]; end
        for (int k = 0; k < 2; k++) begin gmv_row[k] = g_row[k]; gmv_col[k] = g_col[k]; end
        for (int k = 0; k < 3; k++) drl_ctx[k] = drl[k];
        new_mv_ctx = nctx; ref_mv_ctx = rctx; zero_mv_ctx = zero_ctx;
    end

    // ---------------------------------------------------------------- control
    typedef enum logic [3:0] {P_ROW1, P_COL1, P_TR, P_TPL, P_TL, P_ROW3, P_COL3, P_ROW5, P_COL5, P_END} phase_t;
    phase_t ph, ph_done;
    typedef enum logic [5:0] {M_IDLE, M_GM_A, M_GM_B, M_GM_C, M_PH, M_ISSUE, M_WAIT, M_PROC, M_NEAR,
                              M_TPL_ADV, M_TPL_ISSUE, M_TPL_WAIT0, M_TPL_PROC0, M_TPL_WAIT1, M_TPL_PROC1, M_TPL_ADD,
                              M_SORT_END, M_SORT, M_EXTRA, M_EX_ISSUE, M_EX_WAIT, M_EX_P0, M_EX_P1, M_EX_NEXT, M_EX_COMB, M_FILL,
                              M_CTX, M_CLAMP, M_SEL, M_DONE} st_t;
    st_t st;
    logic        is_row, is_point;
    logic signed [MIS-1:0] d_row, d_col;         // scan offsets (after the odd-position adjustment)
    logic [5:0]  end4, i;
    logic        step16, far;                 // far: |delta| > 1
    logic signed [MIS-1:0] mv_r, mv_c;           // position being scanned
    logic [5:0]  cand_len;
    logic [15:0] cand_w;
    logic [7:0]  hit;                         // stack match (idx < n_found)
    // candidate of the current step (spatial: from rd_ent; temporal: tc*; extra: ex*)
    logic signed [17:0] cand_r0, cand_c0, cand_r1, cand_c1;
    logic        sp_ok;                       // spatial: the neighbour is a candidate
    logic [1:0]  sp_cl;                       // single: which of the neighbour's lists matches (2 = none)
    // sorting
    logic [3:0]  s_start, s_end, s_idx, s_new;
    logic        s_second;
    // global motion
    logic        g_sel;
    logic [1:0]  g_typ;
    logic signed [23:0] gp [6];
    logic signed [16:0] g_x, g_y;
    logic signed [43:0] g_p1, g_p2, g_p3, g_p4;
    // temporal
    logic [5:0]  t_dr, t_maxr, t_maxc, t_stepr, t_stepc;
    logic signed [6:0] t_dc;
    logic [1:0]  t_ext;                       // 0 grid, 1 .. 3 extension positions
    logic        t_zero;                      // the (0, 0) position
    logic signed [MIS-1:0] t_row, t_col;
    logic signed [17:0] tc_r0, tc_c0, tc_r1, tc_c1;
    // extra search
    logic        ex_pass;
    logic [5:0]  ex_idx, ex_num;
    logic signed [MIS-1:0] ex_row, ex_col;
    logic [1:0]  id_cnt [2], diff_cnt [2];
    logic signed [17:0] id_r [2][2], id_c [2][2], diff_r [2][2], diff_c [2][2];
    logic [3:0]  ex_ref;
    logic signed [17:0] ex_mr, ex_mc;         // the neighbour's MV of the list being processed
    logic signed [17:0] ex_r0, ex_c0;         // single: the (sign-flipped) candidate

    logic        use_g, big;                  // spatial single-reference candidate: GLOBALMV substitution
    logic signed [17:0] e_mv0r, e_mv0c, e_mv1r, e_mv1c;      // rd_ent MVs sign-extended
    assign e_mv0r = 18'(signed'(rd_ent.mv_row));  assign e_mv0c = 18'(signed'(rd_ent.mv_col));
    assign e_mv1r = 18'(signed'(rd_ent.mv1_row)); assign e_mv1c = 18'(signed'(rd_ent.mv1_col));

    always_comb begin
        // extra search: the neighbour's reference / MV of the list being processed, sign-flipped for a single reference
        ex_ref = (st == M_EX_P1) ? rd_ent.ref1 : rd_ent.ref0;
        ex_mr  = (st == M_EX_P1) ? e_mv1r : e_mv0r;
        ex_mc  = (st == M_EX_P1) ? e_mv1c : e_mv0c;
        if (sbias(ex_ref) != sbias(ref0)) begin ex_r0 = -ex_mr; ex_c0 = -ex_mc; end else begin ex_r0 = ex_mr; ex_c0 = ex_mc; end
        // temporal read address
        mf_ref = 3'((st == M_TPL_WAIT1 || st == M_TPL_PROC1 ? ref1 : ref0) - 4'd1);
        mf_y8  = (MIW-1)'(t_row >>> 1);
        mf_x8  = (MIW-1)'(t_col >>> 1);
        // position of the current spatial scan step
        if (is_point) begin mv_r = MIS'(signed'({2'b0, r})) + d_row; mv_c = MIS'(signed'({2'b0, c})) + d_col; end
        else if (is_row) begin mv_r = MIS'(signed'({2'b0, r})) + d_row; mv_c = MIS'(signed'({2'b0, c})) + d_col + MIS'(signed'({7'b0, i})); end
        else begin mv_r = MIS'(signed'({2'b0, r})) + d_row + MIS'(signed'({7'b0, i})); mv_c = MIS'(signed'({2'b0, c})) + d_col; end
        // mv_mem address: the extra search holds its own
        if (st == M_EX_ISSUE || st == M_EX_WAIT || st == M_EX_P0 || st == M_EX_P1 || st == M_EX_NEXT) begin
            rd_row = MIW'(ex_row); rd_col = MIW'(ex_col);
        end else begin rd_row = MIW'(mv_r); rd_col = MIW'(mv_c); end
        // step length from the entry just read
        if (is_point) cand_len = 6'd0;
        else if (is_row) cand_len = (bw4 < num4x4w(rd_ent.bsize)) ? bw4 : num4x4w(rd_ent.bsize);
        else cand_len = (bh4 < num4x4h(rd_ent.bsize)) ? bh4 : num4x4h(rd_ent.bsize);
        if (!is_point && far && cand_len < 6'd2) cand_len = 6'd2;
        if (!is_point && step16 && cand_len < 6'd4) cand_len = 6'd4;
        cand_w = is_point ? 16'd4 : 16'(cand_len) * 16'd2;
        // spatial candidate (add_ref_mv_candidate 7.10.2.7 -> search_stack 7.10.2.8 / compound_search_stack 7.10.2.9)
        sp_cl = (rd_ent.ref0 == ref0) ? 2'd0 : (rd_ent.ref1 == ref0) ? 2'd1 : 2'd2;
        big = (blk_w(rd_ent.bsize) >= 8'd8) && (blk_h(rd_ent.bsize) >= 8'd8);
        use_g = rd_ent.globalmv_fam && (gmt(ref0) > TRANSLATION) && big;
        if (!is_comp) begin
            sp_ok = rd_written && rd_ent.is_inter && (sp_cl != 2'd2);
            cand_r0 = lower_prec(use_g ? g_row[0] : (sp_cl == 2'd1) ? e_mv1r : e_mv0r);
            cand_c0 = lower_prec(use_g ? g_col[0] : (sp_cl == 2'd1) ? e_mv1c : e_mv0c);
            cand_r1 = 18'sd0; cand_c1 = 18'sd0;
        end else begin
            sp_ok = rd_written && rd_ent.is_inter && (rd_ent.ref0 == ref0) && (rd_ent.ref1 == ref1);
            cand_r0 = lower_prec((rd_ent.globalmv_fam && gmt(ref0) > TRANSLATION) ? g_row[0] : e_mv0r);
            cand_c0 = lower_prec((rd_ent.globalmv_fam && gmt(ref0) > TRANSLATION) ? g_col[0] : e_mv0c);
            cand_r1 = lower_prec((rd_ent.globalmv_fam && gmt(ref1) > TRANSLATION) ? g_row[1] : e_mv1r);
            cand_c1 = lower_prec((rd_ent.globalmv_fam && gmt(ref1) > TRANSLATION) ? g_col[1] : e_mv1c);
        end
        // temporal / extra candidates replace the spatial one
        if (st == M_TPL_ADD) begin cand_r0 = tc_r0; cand_c0 = tc_c0; cand_r1 = tc_r1; cand_c1 = tc_c1; end
        if (st == M_EX_P0 || st == M_EX_P1) begin cand_r0 = ex_r0; cand_c0 = ex_c0; cand_r1 = 18'sd0; cand_c1 = 18'sd0; end
        for (int k = 0; k < 8; k++)
            hit[k] = (4'(k) < n_found) && (st_row0[k] == cand_r0) && (st_col0[k] == cand_c0) &&
                     (!is_comp || st == M_EX_P0 || st == M_EX_P1 || ((st_row1[k] == cand_r1) && (st_col1[k] == cand_c1)));
    end

    // clamp_mv_row / clamp_mv_col (border = MV_BORDER + bh * 8 / bw * 8)
    logic signed [19:0] top_lo, top_hi, left_lo, left_hi;
    always_comb begin
        logic signed [19:0] border_r, border_c;
        border_r = 20'(MV_BORDER) + 20'(signed'({14'b0, bh4})) * 20'sd32;       // bh * 8 = bh4 * 4 * 8
        border_c = 20'(MV_BORDER) + 20'(signed'({14'b0, bw4})) * 20'sd32;
        top_lo  = -(20'(signed'({{(20 - MIW){1'b0}}, r})) * 20'sd32) - border_r;             // mbToTopEdge - border
        top_hi  = (20'(signed'({{(20 - MIW){1'b0}}, mi_rows})) - 20'(signed'({14'b0, bh4})) - 20'(signed'({{(20 - MIW){1'b0}}, r}))) * 20'sd32 + border_r;
        left_lo = -(20'(signed'({{(20 - MIW){1'b0}}, c})) * 20'sd32) - border_c;
        left_hi = (20'(signed'({{(20 - MIW){1'b0}}, mi_cols})) - 20'(signed'({14'b0, bw4})) - 20'(signed'({{(20 - MIW){1'b0}}, c}))) * 20'sd32 + border_c;
    end
    function automatic logic signed [17:0] clamp18(input logic signed [17:0] v, input logic signed [19:0] lo, input logic signed [19:0] hi);
        logic signed [19:0] x;
        x = 20'(v);
        if (x < lo) x = lo;
        else if (x > hi) x = hi;
        clamp18 = 18'(x);
    endfunction
    function automatic logic big_diff(input logic signed [17:0] a, input logic signed [17:0] b);   // |a - b| >= 16
        logic signed [18:0] d;
        d = 19'(a) - 19'(b);
        big_diff = (d >= 19'sd16) || (d <= -19'sd16);
    endfunction

    // fallback vector (assign_mv for intra block copy when both stack entries are zero)
    logic [5:0] sb_size4;
    assign sb_size4 = hdr.sb128 ? 6'd32 : 6'd16;
    logic top_of_tile;
    assign top_of_tile = (MIS'(signed'({2'b0, r})) - MIS'(signed'({7'b0, sb_size4}))) < MIS'(signed'({2'b0, hdr.mi_row_start}));

    // combined MVs of the compound extra search (7.10.2.12): per list, RefIdMvs, then RefDiffMvs, then the global MV
    logic signed [17:0] cmb_r [2][2], cmb_c [2][2];      // [compCount][list]
    always_comb begin
        for (int l = 0; l < 2; l++) begin
            // entry 0
            if (id_cnt[l] != 2'd0) begin cmb_r[0][l] = id_r[l][0]; cmb_c[0][l] = id_c[l][0]; end
            else if (diff_cnt[l] != 2'd0) begin cmb_r[0][l] = diff_r[l][0]; cmb_c[0][l] = diff_c[l][0]; end
            else begin cmb_r[0][l] = g_row[l]; cmb_c[0][l] = g_col[l]; end
            // entry 1
            if (id_cnt[l] == 2'd2) begin cmb_r[1][l] = id_r[l][1]; cmb_c[1][l] = id_c[l][1]; end
            else if (id_cnt[l] == 2'd1) begin
                if (diff_cnt[l] != 2'd0) begin cmb_r[1][l] = diff_r[l][0]; cmb_c[1][l] = diff_c[l][0]; end
                else begin cmb_r[1][l] = g_row[l]; cmb_c[1][l] = g_col[l]; end
            end else begin
                if (diff_cnt[l] == 2'd2) begin cmb_r[1][l] = diff_r[l][1]; cmb_c[1][l] = diff_c[l][1]; end
                else begin cmb_r[1][l] = g_row[l]; cmb_c[1][l] = g_col[l]; end
            end
        end
    end

    assign busy = (st != M_IDLE);

    // the position of a temporal sample and its validity
    logic t_pos_ok;
    logic signed [7:0] t_er, t_ec;           // extension position inside its 64x64 (row / col within the superblock)
    always_comb begin
        t_row = (MIS'(signed'({2'b0, r})) + MIS'(signed'({9'b0, t_dr}))) | MIS'(1);
        t_col = (MIS'(signed'({2'b0, c})) + MIS'(t_dc)) | MIS'(1);
        t_er = 8'(signed'({4'b0, r[3:0]})) + 8'(signed'({2'b0, t_dr}));
        t_ec = 8'(signed'({4'b0, c[3:0]})) + 8'(t_dc);
        t_pos_ok = is_inside(t_row, t_col);
        if (t_ext != 2'd0 && (t_er < 8'sd0 || t_er >= 8'sd16 || t_ec < 8'sd0 || t_ec >= 8'sd16)) t_pos_ok = 1'b0;
    end
    logic allow_ext;
    assign allow_ext = (bh4 >= 6'd2) && (bh4 < 6'd16) && (bw4 >= 6'd2) && (bw4 < 6'd16);

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            st <= M_IDLE;
        end else case (st)
            M_IDLE: if (start) begin
                n_found <= 4'd0; n_nearest <= 4'd0; newmv_count <= 6'd0; num_new <= 6'd0;
                found_above <= 1'b0; found_left <= 1'b0; fm <= 1'b0; close_matches <= 2'd0; zero_ctx <= 1'b0;
                for (int k = 0; k < 8; k++) begin
                    weight[k] <= 16'd0; st_row0[k] <= 18'sd0; st_col0[k] <= 18'sd0; st_row1[k] <= 18'sd0; st_col1[k] <= 18'sd0;
                end
                g_row[0] <= 18'sd0; g_col[0] <= 18'sd0; g_row[1] <= 18'sd0; g_col[1] <= 18'sd0;
                g_sel <= 1'b0; st <= M_GM_A;
            end
            // ---- setup_global_mv (7.10.2.1) for list g_sel: 3 cycles (parameters / products / sum, round, lower)
            M_GM_A: begin
                logic [3:0] rf;
                rf = g_sel ? ref1 : ref0;
                g_typ <= (rf == 4'd0) ? 2'd0 : gmt(rf);
                for (int j = 0; j < 6; j++) gp[j] <= (rf >= 4'd1 && rf <= 4'd7) ? gmp(rf, j) : 24'sd0;
                g_x <= 17'(signed'({2'b0, c, 2'b0})) + 17'(signed'({10'b0, blk_w(bs) >> 1})) - 17'sd1;   // MiCol * 4 + bw / 2 - 1
                g_y <= 17'(signed'({2'b0, r, 2'b0})) + 17'(signed'({10'b0, blk_h(bs) >> 1})) - 17'sd1;
                st <= M_GM_B;
            end
            M_GM_B: begin
                g_p1 <= 44'(gp[2] - 24'sd65536) * 44'(g_x);
                g_p2 <= 44'(gp[3]) * 44'(g_y);
                g_p3 <= 44'(gp[4]) * 44'(g_x);
                g_p4 <= 44'(gp[5] - 24'sd65536) * 44'(g_y);
                st <= M_GM_C;
            end
            M_GM_C: begin
                logic signed [43:0] xc, yc;
                logic signed [17:0] mr, mc;
                xc = g_p1 + g_p2 + 44'(gp[0]);
                yc = g_p3 + g_p4 + 44'(gp[1]);
                if (g_typ == 2'd0) begin mr = 18'sd0; mc = 18'sd0; end
                else if (g_typ == TRANSLATION) begin mr = 18'(gp[0] >>> 13); mc = 18'(gp[1] >>> 13); end
                else if (ihdr.allow_hp_mv) begin mr = 18'(r2s(yc, 13)); mc = 18'(r2s(xc, 13)); end
                else begin mr = 18'(r2s(yc, 14) <<< 1); mc = 18'(r2s(xc, 14) <<< 1); end
                g_row[g_sel] <= lower_prec(mr); g_col[g_sel] <= lower_prec(mc);
                if (!g_sel && is_comp) begin g_sel <= 1'b1; st <= M_GM_A; end
                else begin ph <= P_ROW1; ph_done <= P_END; st <= M_PH; end
            end
            // ---- phase setup (scan row / scan col / scan point / temporal / sorts); folds the finished scan's FoundMatch
            M_PH: begin
                if (fm) begin
                    if (ph_done == P_COL1 || ph_done == P_COL3 || ph_done == P_COL5) found_left <= 1'b1;
                    else found_above <= 1'b1;
                end
                fm <= 1'b0;
                i <= 6'd0; is_point <= 1'b0; far <= 1'b0; is_row <= 1'b0; step16 <= 1'b0; d_row <= MIS'(0); d_col <= MIS'(0);
                case (ph)
                    P_ROW1, P_ROW3, P_ROW5: begin
                        logic signed [MIS-1:0] dr;
                        dr = (ph == P_ROW1) ? -MIS'(1) : (ph == P_ROW3) ? -MIS'(3) : -MIS'(5);
                        is_row <= 1'b1;
                        begin   // end4 = Min(Min(bw4, MiCols - MiCol), 16), compared at full width
                            logic [MIW-1:0] rem, e;
                            rem = mi_cols - c;
                            e = (MIW'(bw4) < rem) ? MIW'(bw4) : rem;
                            end4 <= (e > MIW'(16)) ? 6'd16 : 6'(e);
                        end
                        step16 <= (bw4 >= 6'd16);
                        if (ph != P_ROW1) begin far <= 1'b1; d_row <= dr + MIS'(signed'({12'b0, r[0]})); d_col <= MIS'(1) - MIS'(signed'({12'b0, c[0]})); end
                        else d_row <= dr;
                        // rows -5 only for bh4 > 1
                        if (ph == P_ROW5 && bh4 == 6'd1) begin ph <= P_COL5; ph_done <= P_END; st <= M_PH; end
                        else st <= M_ISSUE;
                    end
                    P_COL1, P_COL3, P_COL5: begin
                        logic signed [MIS-1:0] dc;
                        dc = (ph == P_COL1) ? -MIS'(1) : (ph == P_COL3) ? -MIS'(3) : -MIS'(5);
                        begin
                            logic [MIW-1:0] rem, e;
                            rem = mi_rows - r;
                            e = (MIW'(bh4) < rem) ? MIW'(bh4) : rem;
                            end4 <= (e > MIW'(16)) ? 6'd16 : 6'(e);
                        end
                        step16 <= (bh4 >= 6'd16);
                        if (ph != P_COL1) begin far <= 1'b1; d_row <= MIS'(1) - MIS'(signed'({12'b0, r[0]})); d_col <= dc + MIS'(signed'({12'b0, c[0]})); end
                        else d_col <= dc;
                        if (ph == P_COL5 && bw4 == 6'd1) begin ph <= P_END; ph_done <= P_END; st <= M_PH; end
                        else st <= M_ISSUE;
                    end
                    P_TR: begin
                        is_point <= 1'b1; d_row <= -MIS'(1); d_col <= MIS'(signed'({7'b0, bw4}));
                        if ((bw4 > bh4 ? bw4 : bh4) <= 6'd16) st <= M_ISSUE;
                        else st <= M_NEAR;                          // no top-right scan for big blocks
                    end
                    P_TPL: begin
                        if (ihdr.use_ref_frame_mvs) begin
                            t_dr <= 6'd0; t_dc <= 7'sd0; t_ext <= 2'd0;
                            t_maxr <= (bh4 < 6'd16) ? bh4 : 6'd16; t_maxc <= (bw4 < 6'd16) ? bw4 : 6'd16;
                            t_stepr <= (bh4 >= 6'd16) ? 6'd4 : 6'd2; t_stepc <= (bw4 >= 6'd16) ? 6'd4 : 6'd2;
                            st <= M_TPL_ISSUE;
                        end else begin ph <= P_TL; ph_done <= P_END; st <= M_PH; end
                    end
                    P_TL: begin is_point <= 1'b1; d_row <= -MIS'(1); d_col <= -MIS'(1); st <= M_ISSUE; end
                    default: begin                                  // P_END: sorts
                        s_start <= 4'd0; s_end <= n_nearest; s_second <= 1'b0; st <= M_SORT_END;
                    end
                endcase
            end
            // ---- one scanned position: address out, entry in, stack update
            M_ISSUE: begin
                if (!is_inside(mv_r, mv_c)) begin
                    // scan row / col: stop the whole scan; point: nothing to add
                    st <= (ph == P_TR) ? M_NEAR : M_PH;
                    if (ph != P_TR) begin ph <= phase_t'(ph + 4'd1); ph_done <= ph; end
                end else st <= M_WAIT;
            end
            M_WAIT: st <= M_PROC;
            M_PROC: begin
                if (sp_ok) begin
                    fm <= 1'b1;
                    if (rd_ent.newmv_fam) newmv_count <= newmv_count + 6'd1;
                    if (|hit) begin
                        for (int k = 0; k < 8; k++) if (hit[k]) weight[k] <= weight[k] + cand_w;
                    end else if (n_found < 4'd8) begin
                        st_row0[n_found[2:0]] <= cand_r0; st_col0[n_found[2:0]] <= cand_c0;
                        st_row1[n_found[2:0]] <= cand_r1; st_col1[n_found[2:0]] <= cand_c1;
                        weight[n_found[2:0]] <= cand_w;
                        n_found <= n_found + 4'd1;
                    end
                end
                if (is_point || (i + cand_len >= end4)) begin
                    st <= (ph == P_TR) ? M_NEAR : M_PH;
                    if (ph != P_TR) begin ph <= phase_t'(ph + 4'd1); ph_done <= ph; end
                end else begin
                    i <= i + cand_len; st <= M_ISSUE;
                end
            end
            // ---- after the three nearest scans: CloseMatches, numNearest, numNew, REF_CAT_LEVEL bonus
            M_NEAR: begin
                if (fm) found_above <= 1'b1;
                fm <= 1'b0;
                close_matches <= 2'(found_above | fm) + 2'(found_left);
                n_nearest <= n_found; num_new <= newmv_count;
                for (int k = 0; k < 8; k++) if (4'(k) < n_found) weight[k] <= weight[k] + 16'(REF_CAT_LEVEL);
                zero_ctx <= 1'b0;
                ph <= P_TPL; ph_done <= P_END; st <= M_PH;
            end
            // ---- temporal candidates (7.10.2.5 / 7.10.2.6)
            M_TPL_ADV: begin
                if (t_ext == 2'd0) begin
                    if (6'(t_dc) + t_stepc < t_maxc) begin t_dc <= t_dc + 7'(t_stepc); st <= M_TPL_ISSUE; end
                    else if (t_dr + t_stepr < t_maxr) begin t_dc <= 7'sd0; t_dr <= t_dr + t_stepr; st <= M_TPL_ISSUE; end
                    else if (allow_ext) begin t_ext <= 2'd1; t_dr <= bh4; t_dc <= -7'sd2; st <= M_TPL_ISSUE; end
                    else begin ph <= P_TL; ph_done <= P_END; st <= M_PH; end
                end else if (t_ext == 2'd1) begin t_ext <= 2'd2; t_dr <= bh4; t_dc <= 7'(bw4); st <= M_TPL_ISSUE; end
                else if (t_ext == 2'd2) begin t_ext <= 2'd3; t_dr <= bh4 - 6'd2; t_dc <= 7'(bw4); st <= M_TPL_ISSUE; end
                else begin ph <= P_TL; ph_done <= P_END; st <= M_PH; end
            end
            M_TPL_ISSUE: begin
                t_zero <= (t_dr == 6'd0) && (t_dc == 7'sd0);
                if (!t_pos_ok) st <= M_TPL_ADV;
                else begin
                    if (t_dr == 6'd0 && t_dc == 7'sd0) zero_ctx <= 1'b1;
                    st <= M_TPL_WAIT0;
                end
            end
            M_TPL_WAIT0: st <= M_TPL_PROC0;
            M_TPL_PROC0: begin
                if (!mf_valid) st <= M_TPL_ADV;
                else begin
                    tc_r0 <= lower_prec(18'(signed'(mf_row))); tc_c0 <= lower_prec(18'(signed'(mf_col)));
                    tc_r1 <= 18'sd0; tc_c1 <= 18'sd0;
                    st <= is_comp ? M_TPL_WAIT1 : M_TPL_ADD;
                end
            end
            M_TPL_WAIT1: st <= M_TPL_PROC1;
            M_TPL_PROC1: begin
                if (!mf_valid) st <= M_TPL_ADV;
                else begin
                    tc_r1 <= lower_prec(18'(signed'(mf_row))); tc_c1 <= lower_prec(18'(signed'(mf_col)));
                    st <= M_TPL_ADD;
                end
            end
            M_TPL_ADD: begin
                if (t_zero) zero_ctx <= big_diff(tc_r0, g_row[0]) || big_diff(tc_c0, g_col[0]) ||
                                        (is_comp && (big_diff(tc_r1, g_row[1]) || big_diff(tc_c1, g_col[1])));
                if (|hit) begin
                    for (int k = 0; k < 8; k++) if (hit[k]) weight[k] <= weight[k] + 16'd2;
                end else if (n_found < 4'd8) begin
                    st_row0[n_found[2:0]] <= tc_r0; st_col0[n_found[2:0]] <= tc_c0;
                    st_row1[n_found[2:0]] <= tc_r1; st_col1[n_found[2:0]] <= tc_c1;
                    weight[n_found[2:0]] <= 16'd2;
                    n_found <= n_found + 4'd1;
                end
                st <= M_TPL_ADV;
            end
            // ---- sorting process (7.10.2.11): the spec's loop, one compare/swap per cycle
            M_SORT_END: begin
                // TotalMatches is final here (the last scan's FoundMatch was folded in M_PH of P_END)
                if (s_end > s_start) begin s_new <= s_start; s_idx <= s_start + 4'd1; st <= M_SORT; end
                else if (!s_second) begin s_second <= 1'b1; s_start <= n_nearest; s_end <= n_found; end
                else if (n_found < 4'd2) st <= M_EXTRA;
                else st <= M_CTX;
            end
            M_SORT: begin
                if (s_idx < s_end) begin
                    if (weight[s_idx[2:0] - 3'd1] < weight[s_idx[2:0]]) begin
                        weight[s_idx[2:0] - 3'd1] <= weight[s_idx[2:0]]; weight[s_idx[2:0]] <= weight[s_idx[2:0] - 3'd1];
                        st_row0[s_idx[2:0] - 3'd1] <= st_row0[s_idx[2:0]]; st_row0[s_idx[2:0]] <= st_row0[s_idx[2:0] - 3'd1];
                        st_col0[s_idx[2:0] - 3'd1] <= st_col0[s_idx[2:0]]; st_col0[s_idx[2:0]] <= st_col0[s_idx[2:0] - 3'd1];
                        st_row1[s_idx[2:0] - 3'd1] <= st_row1[s_idx[2:0]]; st_row1[s_idx[2:0]] <= st_row1[s_idx[2:0] - 3'd1];
                        st_col1[s_idx[2:0] - 3'd1] <= st_col1[s_idx[2:0]]; st_col1[s_idx[2:0]] <= st_col1[s_idx[2:0] - 3'd1];
                        s_new <= s_idx;
                    end
                    s_idx <= s_idx + 4'd1;
                end else begin
                    s_end <= s_new; st <= M_SORT_END;
                end
            end
            // ---- extra search (7.10.2.12): row -1 then col -1 neighbours while fewer than two candidates
            M_EXTRA: begin
                logic [MIW-1:0] w4, h4, rem;
                w4 = (bw4 < 6'd16) ? MIW'(bw4) : MIW'(16); rem = mi_cols - c; if (rem < w4) w4 = rem;
                h4 = (bh4 < 6'd16) ? MIW'(bh4) : MIW'(16); rem = mi_rows - r; if (rem < h4) h4 = rem;
                ex_num <= 6'((w4 < h4) ? w4 : h4);
                ex_pass <= 1'b0; ex_idx <= 6'd0;
                for (int l = 0; l < 2; l++) begin id_cnt[l] <= 2'd0; diff_cnt[l] <= 2'd0; end
                ex_row <= MIS'(signed'({2'b0, r})) - MIS'(1); ex_col <= MIS'(signed'({2'b0, c}));
                st <= M_EX_ISSUE;
            end
            M_EX_ISSUE: begin
                if (ex_idx >= ex_num || n_found >= 4'd2 || !is_inside(ex_row, ex_col)) begin
                    if (!ex_pass) begin
                        ex_pass <= 1'b1; ex_idx <= 6'd0;
                        ex_row <= MIS'(signed'({2'b0, r})); ex_col <= MIS'(signed'({2'b0, c})) - MIS'(1);
                    end else st <= is_comp ? M_EX_COMB : M_FILL;
                end else st <= M_EX_WAIT;
            end
            M_EX_WAIT: st <= M_EX_P0;
            M_EX_P0, M_EX_P1: begin
                if (ex_ref >= 4'd1 && ex_ref <= 4'd7) begin
                    if (!is_comp) begin
                        if (!(|hit)) begin
                            st_row0[n_found[2:0]] <= ex_r0; st_col0[n_found[2:0]] <= ex_c0;
                            weight[n_found[2:0]] <= 16'd2;
                            n_found <= n_found + 4'd1;
                        end
                    end else begin
                        for (int l = 0; l < 2; l++) begin
                            logic [3:0] rl;
                            rl = l[0] ? ref1 : ref0;
                            if (ex_ref == rl && id_cnt[l] != 2'd2) begin
                                id_r[l][id_cnt[l][0]] <= ex_mr; id_c[l][id_cnt[l][0]] <= ex_mc; id_cnt[l] <= id_cnt[l] + 2'd1;
                            end else if (ex_ref != rl && diff_cnt[l] != 2'd2) begin
                                if (sbias(ex_ref) != sbias(rl)) begin diff_r[l][diff_cnt[l][0]] <= -ex_mr; diff_c[l][diff_cnt[l][0]] <= -ex_mc; end
                                else begin diff_r[l][diff_cnt[l][0]] <= ex_mr; diff_c[l][diff_cnt[l][0]] <= ex_mc; end
                                diff_cnt[l] <= diff_cnt[l] + 2'd1;
                            end
                        end
                    end
                end
                st <= (st == M_EX_P0) ? M_EX_P1 : M_EX_NEXT;
            end
            M_EX_NEXT: begin
                if (!ex_pass) begin ex_idx <= ex_idx + num4x4w(rd_ent.bsize); ex_col <= ex_col + MIS'(signed'({9'b0, num4x4w(rd_ent.bsize)})); end
                else begin ex_idx <= ex_idx + num4x4h(rd_ent.bsize); ex_row <= ex_row + MIS'(signed'({9'b0, num4x4h(rd_ent.bsize)})); end
                st <= M_EX_ISSUE;
            end
            M_EX_COMB: begin
                if (n_found == 4'd1) begin
                    logic same;
                    same = (cmb_r[0][0] == st_row0[0]) && (cmb_c[0][0] == st_col0[0]) && (cmb_r[0][1] == st_row1[0]) && (cmb_c[0][1] == st_col1[0]);
                    st_row0[1] <= same ? cmb_r[1][0] : cmb_r[0][0]; st_col0[1] <= same ? cmb_c[1][0] : cmb_c[0][0];
                    st_row1[1] <= same ? cmb_r[1][1] : cmb_r[0][1]; st_col1[1] <= same ? cmb_c[1][1] : cmb_c[0][1];
                    weight[1] <= 16'd2; n_found <= 4'd2;
                end else begin
                    for (int k = 0; k < 2; k++) begin
                        st_row0[k] <= cmb_r[k][0]; st_col0[k] <= cmb_c[k][0]; st_row1[k] <= cmb_r[k][1]; st_col1[k] <= cmb_c[k][1];
                        weight[k] <= 16'd2;
                    end
                    n_found <= 4'd2;
                end
                st <= M_CTX;
            end
            // ---- single reference: the global MV fills entries NumMvFound .. 1 (NumMvFound unchanged)
            M_FILL: begin
                for (int k = 0; k < 2; k++) if (4'(k) >= n_found) begin st_row0[k] <= g_row[0]; st_col0[k] <= g_col[0]; end
                st <= M_CTX;
            end
            // ---- contexts (7.10.2.14): DrlCtxStack, NewMvContext, RefMvContext
            M_CTX: begin
                logic [1:0] total;
                total = 2'(found_above) + 2'(found_left);
                for (int k = 0; k < 3; k++) begin
                    if (4'(k) + 4'd1 < n_found) begin
                        if (weight[k] >= 16'(REF_CAT_LEVEL)) drl[k] <= (weight[k + 1] < 16'(REF_CAT_LEVEL)) ? 2'd1 : 2'd0;
                        else drl[k] <= 2'd2;
                    end else drl[k] <= 2'd0;
                end
                if (close_matches == 2'd0) begin nctx <= 3'((total < 2'd1) ? total : 2'd1); rctx <= 3'(total); end
                else if (close_matches == 2'd1) begin nctx <= 3'd3 - 3'(num_new != 6'd0); rctx <= 3'd2 + 3'(total); end
                else begin nctx <= 3'd5 - 3'(num_new != 6'd0); rctx <= 3'd5; end
                st <= M_CLAMP;
            end
            // ---- clamping: the found entries, both lists of a compound block
            M_CLAMP: begin
                for (int k = 0; k < 8; k++) if (4'(k) < n_found) begin
                    st_row0[k] <= clamp18(st_row0[k], top_lo, top_hi);
                    st_col0[k] <= clamp18(st_col0[k], left_lo, left_hi);
                    if (is_comp) begin
                        st_row1[k] <= clamp18(st_row1[k], top_lo, top_hi);
                        st_col1[k] <= clamp18(st_col1[k], left_lo, left_hi);
                    end
                end
                st <= M_SEL;
            end
            // ---- assign_mv for intra block copy: PredMv = stack[0] | stack[1] | fallback
            M_SEL: begin
                if (st_row0[0] != 18'sd0 || st_col0[0] != 18'sd0) begin pred_row <= st_row0[0]; pred_col <= st_col0[0]; end
                else if (st_row0[1] != 18'sd0 || st_col0[1] != 18'sd0) begin pred_row <= st_row0[1]; pred_col <= st_col0[1]; end
                else if (top_of_tile) begin pred_row <= 18'sd0; pred_col <= -18'(signed'((18'(sb_size4) * 18'd4 + 18'(INTRABC_DELAY_PIXELS)) * 18'd8)); end
                else begin pred_row <= -18'(signed'(18'(sb_size4) * 18'd32)); pred_col <= 18'sd0; end
                st <= M_DONE;
            end
            M_DONE: begin done <= 1'b1; st <= M_IDLE; end
            default: st <= M_IDLE;
        endcase
    end
endmodule
