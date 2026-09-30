// Palette syntax for one block (spec 5.11.46 palette_mode_info, 5.11.49 palette_tokens, 5.11.50
// get_palette_cache, 5.11.51 get_palette_color_context). Driven by blk_syntax in two phases:
//   1. start_mi  : palette_mode_info -> pal_y / pal_uv sizes and the sorted colour lists (col_y/col_u/col_v)
//   2. start_tok : palette_tokens    -> colour index maps (cmap_y / cmap_uv, read through pm_*)
// One symbol at a time through the shared symbol sequencer (sq_*: kind 0 = CDF symbol, 2 = literal bit).
//
// Colour-index context: the wavefront visits anti-diagonals; pixel (r, c) on diagonal i = r + c needs
// (r, c-1) and (r-1, c) from diagonal i-1 and (r-1, c-1) from diagonal i-2, all indexed by column c, so
// two 64-entry diagonal buffers (d1 = previous, d2 = the one before) give the three neighbours without
// random reads of the map memory. The map itself is written once per pixel into a plain memory; only the
// onscreen part is written, and the read port replicates the last onscreen column / row into the offscreen
// part (spec 5.11.49). That part is not dead: the palette prediction writes whole transform blocks, and CfL
// averages the block's luma up to MaxLumaW / MaxLumaH (7.11.5), which reaches past MiCols * 4 for a block
// straddling the frame edge (Argon test10469 / test10467: a constant offset on the CfL plane).
// Note the palette size test is "MiSize >= BLOCK_8X8" on the enum, so 4x16 / 16x4 / 8x32 ... blocks (enum
// values 16+) qualify too; their subsampled chroma map is widened by 2 when narrower than 4 (spec 5.11.49).
module pal_syntax
  import cdf_map_pkg::*;
  import blk_tables_pkg::*;
  import syn_pkg::*;
(
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  logic        clr,                    // block start: sizes := 0
    // phase 1: palette_mode_info
    input  logic        start_mi,
    input  logic [4:0]  bs,
    input  logic [3:0]  ymode, uvmode,
    input  logic        hc,
    input  logic        avail_u, avail_l,
    input  logic [10:0] br,
    input  logic [3:0]  a_pal_y, l_pal_y, a_pal_uv, l_pal_uv,
    input  logic [95:0] a_col_y, l_col_y, a_col_u, l_col_u,
    output logic        done_mi,
    // phase 2: palette_tokens
    input  logic        start_tok,
    input  logic [7:0]  os_w, os_h,             // onscreen luma width/height of the block (pixels)
    output logic        done_tok,
    // results
    output logic [3:0]  pal_y, pal_uv,
    output logic [95:0] col_y, col_u, col_v,    // 8 x 12 bits, colour k at [12k +: 12]
    // colour map read port (1-cycle latency)
    input  logic        pm_plane,
    input  logic [5:0]  pm_x, pm_y,
    output logic [2:0]  pm_idx,
    // symbol sequencer master
    output logic              sq_go,
    output logic [CDF_AW-1:0] sq_addr,
    output logic [3:0]        sq_n,
    output logic [1:0]        sq_kind,
    input  logic              sq_done,
    input  logic [3:0]        sq_sym
);
    localparam logic [3:0] DC_PRED = 4'd0;

    typedef enum logic [5:0] {
        P_IDLE,
        P_HAS, P_HAS_W,                          // has_palette_y / has_palette_uv
        P_SIZE, P_SIZE_W,                        // palette_size_*_minus_2
        P_CACHE, P_CACHE_BIT_W,                  // get_palette_cache merge + use_palette_color_cache bits
        P_LIT_COL, P_LIT_COL_W,                  // first explicit colour
        P_EXTRA, P_EXTRA_W,                      // palette_num_extra_bits
        P_DELTA, P_DELTA_W,                      // delta-coded colours
        P_SORT,                                  // sort + commit plane
        P_UV_CHK,
        P_V_DE, P_V_DE_W, P_V_EXTRA_W, P_V0_W, P_V_DELTA, P_V_DELTA_W, P_V_SIGN_W, P_V_LIT, P_V_LIT_W,
        P_DONE_MI,
        T_START, T_NS, T_NS_W, T_NS_X_W, T_DIAG, T_PIX, T_PIX_W, T_NEXT_PLANE, T_DONE,
        P_LIT, P_LIT_W
    } st_t;
    st_t st, lit_ret;

    // ---------------------------------------------------------------- registers
    logic        pl;                             // 0 = Y, 1 = UV
    logic [3:0]  n;                              // palette size of the plane being decoded
    logic [11:0] wc [8];                         // working colours
    logic [3:0]  idx, ai, li, above_n, left_n;
    logic [11:0] last;
    logic        have_last;
    logic [3:0]  pbits;
    logic [3:0]  bd;
    logic [12:0] maxv;                           // (1 << bd) - 1
    logic [12:0] vdelta;                         // |palette_delta_v|
    // literal reader
    logic [3:0]  lit_n;
    logic [15:0] lit_val;
    // tokens
    logic [6:0]  tw, th;                         // onscreen plane dims
    logic [6:0]  tw_p [2], th_p [2];             // the same per plane, kept for the map read port
    logic [7:0]  di;                             // diagonal index i
    logic [6:0]  dj, djmin;                      // column j and its lower bound on this diagonal
    logic [2:0]  d1 [64];
    logic [2:0]  d2 [64];
    logic [2:0]  dcur [64];
    logic [3:0]  ns_w, ns_m, ns_v;
    // colour maps
    logic [2:0]  cmap_y  [0:4095];
    logic [2:0]  cmap_uv [0:4095];

    // ---------------------------------------------------------------- helpers
    function automatic logic [3:0] ceil_log2(input logic [12:0] x);
        // CeilLog2(x): 0 for x < 2, else the smallest i with (1 << i) >= x
        logic [3:0] r;
        r = 4'd0;
        for (int i = 12; i >= 1; i--)
            if (r == 4'd0 && x > (13'd1 << (i - 1))) r = 4'(i);
        ceil_log2 = r;
    endfunction

    function automatic logic [2:0] bsize_ctx(input logic [4:0] b);
        bsize_ctx = 3'(mi_w_log2(b) + mi_h_log2(b) - 3'd2);
    endfunction

    function automatic logic [11:0] col_at(input logic [95:0] v, input logic [3:0] k);
        col_at = v[12 * k +: 12];
    endfunction

    // 8-entry bubble sorting network (entries >= n hold 12'hFFF so they sink to the end)
    function automatic logic [95:0] sort8(input logic [95:0] v);
        logic [11:0] a [8];
        logic [11:0] t;
        for (int k = 0; k < 8; k++) a[k] = v[12 * k +: 12];
        for (int p = 0; p < 7; p++)
            for (int k = 0; k < 7 - p; k++)
                if (a[k] > a[k + 1]) begin t = a[k]; a[k] = a[k + 1]; a[k + 1] = t; end
        for (int k = 0; k < 8; k++) sort8[12 * k +: 12] = a[k];
    endfunction

    // get_palette_color_context: scores from the three neighbours, partial selection sort of the top 3,
    // hash -> ctx; returns {ctx[2:0], order[7] .. order[0]} (order k at [3k +: 3])
    function automatic logic [26:0] color_ctx(input logic has_l, input logic has_a,
                                              input logic [2:0] lv, input logic [2:0] alv, input logic [2:0] av);
        logic [2:0] s [8];
        logic [2:0] o [8];
        logic [2:0] ms, ts, mo;
        int         mi;
        logic [3:0] hsh;
        logic [2:0] ctx;
        for (int k = 0; k < 8; k++) begin
            s[k] = 3'd0; o[k] = 3'(k);
            if (has_l && lv == 3'(k)) s[k] = s[k] + 3'd2;
            if (has_l && has_a && alv == 3'(k)) s[k] = s[k] + 3'd1;
            if (has_a && av == 3'(k)) s[k] = s[k] + 3'd2;
        end
        for (int i = 0; i < 3; i++) begin
            ms = s[i]; mi = i;
            for (int j = 1; j < 8; j++)
                if (j > i && s[j] > ms) begin ms = s[j]; mi = j; end
            if (mi != i) begin
                ts = s[mi]; mo = o[mi];
                for (int k = 7; k >= 1; k--)
                    if (k <= mi && k > i) begin s[k] = s[k - 1]; o[k] = o[k - 1]; end
                s[i] = ts; o[i] = mo;
            end
        end
        hsh = 4'(s[0]) + 4'(s[1]) * 4'd2 + 4'(s[2]) * 4'd2;   // Palette_Color_Hash_Multipliers = {1, 2, 2}
        case (hsh)                                              // Palette_Color_Context
            4'd2: ctx = 3'd0;
            4'd5: ctx = 3'd4;
            4'd6: ctx = 3'd3;
            4'd7: ctx = 3'd2;
            4'd8: ctx = 3'd1;
            default: ctx = 3'd0;                                // unreachable hashes (-1 in the spec table)
        endcase
        color_ctx[26:24] = ctx;
        for (int k = 0; k < 8; k++) color_ctx[3 * k +: 3] = o[k];
    endfunction

    function automatic logic [CDF_AW-1:0] color_cdf(input logic p, input logic [3:0] nn, input logic [2:0] ctx);
        int base;
        case (nn)
            4'd2: base = p ? CDF_PALETTE_SIZE_2_UV_COLOR : CDF_PALETTE_SIZE_2_Y_COLOR;
            4'd3: base = p ? CDF_PALETTE_SIZE_3_UV_COLOR : CDF_PALETTE_SIZE_3_Y_COLOR;
            4'd4: base = p ? CDF_PALETTE_SIZE_4_UV_COLOR : CDF_PALETTE_SIZE_4_Y_COLOR;
            4'd5: base = p ? CDF_PALETTE_SIZE_5_UV_COLOR : CDF_PALETTE_SIZE_5_Y_COLOR;
            4'd6: base = p ? CDF_PALETTE_SIZE_6_UV_COLOR : CDF_PALETTE_SIZE_6_Y_COLOR;
            4'd7: base = p ? CDF_PALETTE_SIZE_7_UV_COLOR : CDF_PALETTE_SIZE_7_Y_COLOR;
            default: base = p ? CDF_PALETTE_SIZE_8_UV_COLOR : CDF_PALETTE_SIZE_8_Y_COLOR;
        endcase
        color_cdf = CDF_AW'(base + int'(ctx));
    endfunction

    // ---------------------------------------------------------------- cache merge candidate (combinational)
    // get_palette_cache is a sorted merge of the above and left lists with duplicates dropped; the merge
    // is done on the fly, one candidate per P_CACHE visit, so the cache never needs to be stored.
    logic [95:0] above_cols, left_cols;
    logic [11:0] above_c, left_c, cand;
    logic        have_a, have_l, cand_valid;
    logic [3:0]  n_ai, n_li;
    always_comb begin
        above_cols = pl ? a_col_u : a_col_y;
        left_cols  = pl ? l_col_u : l_col_y;
        above_c = col_at(above_cols, ai);
        left_c  = col_at(left_cols, li);
        have_a = ai < above_n;
        have_l = li < left_n;
        cand = 12'd0; cand_valid = 1'b1; n_ai = ai; n_li = li;
        if (have_a && have_l) begin
            if (left_c < above_c) begin cand = left_c; n_li = li + 4'd1; end
            else begin cand = above_c; n_ai = ai + 4'd1; if (left_c == above_c) n_li = li + 4'd1; end
        end else if (have_a) begin cand = above_c; n_ai = ai + 4'd1; end
        else if (have_l) begin cand = left_c; n_li = li + 4'd1; end
        else cand_valid = 1'b0;
    end

    // delta-coded colour step (Y: delta + 1 and range - 1; U: as read)
    logic [12:0] d_sum, d_new, d_rng;
    logic [3:0]  d_bits;
    always_comb begin
        d_sum = 13'(wc[3'(idx - 4'd1)]) + 13'(lit_val[12:0]) + (pl ? 13'd0 : 13'd1);
        d_new = (d_sum > maxv) ? maxv : d_sum;
        d_rng = (13'd1 << bd) - d_new - (pl ? 13'd0 : 13'd1);
        d_bits = ceil_log2(d_rng);
    end
    // v colour step: prev +/- vdelta with wrap-around modulo (1 << bd); sign bit = lit_val[0] in P_V_SIGN_W
    logic signed [14:0] v_val;
    logic [11:0] v_wrapped;
    always_comb begin
        v_val = $signed({3'b0, wc[3'(idx - 4'd1)]}) + (lit_val[0] ? -$signed({2'b0, vdelta}) : $signed({2'b0, vdelta}));
        if (v_val < 0) v_wrapped = 12'(v_val + $signed({2'b0, maxv}) + 15'sd1);
        else if (v_val > $signed({2'b0, maxv})) v_wrapped = 12'(v_val - $signed({2'b0, maxv}) - 15'sd1);
        else v_wrapped = 12'(v_val);
    end

    // token context (combinational from the diagonal buffers)
    logic [26:0] cc;
    logic [6:0]  dr;
    logic [2:0]  pix_val;
    always_comb begin
        dr = 7'(di) - dj;
        cc = color_ctx(dj != 7'd0, dr != 7'd0, (dj != 7'd0) ? d1[6'(dj - 7'd1)] : 3'd0, (dj != 7'd0) ? d2[6'(dj - 7'd1)] : 3'd0, d1[dj[5:0]]);
        pix_val = cc[3 * sq_sym[2:0] +: 3];
    end
    // read_ns(n) parameters
    logic [3:0] ns_w_c;
    always_comb ns_w_c = (n >= 4'd8) ? 4'd4 : (n >= 4'd4) ? 4'd3 : 4'd2;   // FloorLog2(n) + 1

    // ---------------------------------------------------------------- symbol requests
    always_comb begin
        sq_go = 1'b0; sq_addr = '0; sq_n = 4'd1; sq_kind = 2'd0;
        case (st)
            P_HAS: begin
                sq_go = 1'b1; sq_n = 4'd1;
                if (!pl) sq_addr = CDF_AW'(CDF_PALETTE_Y_MODE + int'(bsize_ctx(bs)) * CDF_PALETTE_Y_MODE_S0
                                          + ((avail_u && a_pal_y != 4'd0) ? 1 : 0) + ((avail_l && l_pal_y != 4'd0) ? 1 : 0));
                else sq_addr = CDF_AW'(CDF_PALETTE_UV_MODE + ((pal_y != 4'd0) ? 1 : 0));
            end
            P_SIZE: begin
                sq_go = 1'b1; sq_n = 4'd6;
                sq_addr = pl ? CDF_AW'(CDF_PALETTE_UV_SIZE + int'(bsize_ctx(bs))) : CDF_AW'(CDF_PALETTE_Y_SIZE + int'(bsize_ctx(bs)));
            end
            T_PIX: begin sq_go = 1'b1; sq_addr = color_cdf(pl, n, cc[26:24]); sq_n = n - 4'd1; end
            P_LIT: if (lit_n != 4'd0) begin sq_go = 1'b1; sq_kind = 2'd2; end
            default: ;
        endcase
    end

    // ---------------------------------------------------------------- map read port
    // offscreen positions read the last onscreen column / row (the spec's ColorMap replication)
    logic [5:0] pm_xc, pm_yc;
    always_comb begin
        pm_xc = (7'(pm_x) >= tw_p[pm_plane]) ? 6'(tw_p[pm_plane] - 7'd1) : pm_x;
        pm_yc = (7'(pm_y) >= th_p[pm_plane]) ? 6'(th_p[pm_plane] - 7'd1) : pm_y;
    end
    always_ff @(posedge clk) pm_idx <= pm_plane ? cmap_uv[{pm_yc, pm_xc}] : cmap_y[{pm_yc, pm_xc}];

    // ---------------------------------------------------------------- FSM
    always_ff @(posedge clk) begin
        done_mi <= 1'b0; done_tok <= 1'b0;
        if (rst) begin
            st <= P_IDLE; pal_y <= 4'd0; pal_uv <= 4'd0;
        end else begin
            if (clr) begin pal_y <= 4'd0; pal_uv <= 4'd0; end
            case (st)
                P_IDLE: begin
                    bd <= hdr.bit_depth; maxv <= (13'd1 << hdr.bit_depth) - 13'd1;
                    if (start_mi) begin
                        pal_y <= 4'd0; pal_uv <= 4'd0; pl <= 1'b0;
                        st <= (ymode == DC_PRED) ? P_HAS : P_UV_CHK;
                    end else if (start_tok) begin
                        pl <= (pal_y != 4'd0) ? 1'b0 : 1'b1;
                        st <= T_START;
                    end
                end
                // ---- palette_mode_info: has / size
                P_HAS: st <= P_HAS_W;
                P_HAS_W: if (sq_done) st <= sq_sym[0] ? P_SIZE : (pl ? P_DONE_MI : P_UV_CHK);
                P_SIZE: st <= P_SIZE_W;
                P_SIZE_W: if (sq_done) begin
                    n <= sq_sym + 4'd2;
                    if (pl) pal_uv <= sq_sym + 4'd2; else pal_y <= sq_sym + 4'd2;
                    // get_palette_cache: above colours only inside the same 64x64 ((MiRow * 4) % 64 != 0), left if AvailL
                    above_n <= (br[3:0] != 4'd0) ? (pl ? a_pal_uv : a_pal_y) : 4'd0;
                    left_n  <= avail_l ? (pl ? l_pal_uv : l_pal_y) : 4'd0;
                    ai <= 4'd0; li <= 4'd0; idx <= 4'd0; have_last <= 1'b0;
                    for (int k = 0; k < 8; k++) wc[k] <= 12'hFFF;
                    st <= P_CACHE;
                end
                // ---- cache colours
                P_CACHE: begin
                    if (idx >= n || !cand_valid) st <= P_LIT_COL;
                    else begin
                        ai <= n_ai; li <= n_li;
                        if (!have_last || cand != last) begin
                            last <= cand; have_last <= 1'b1;
                            lit_n <= 4'd1; lit_val <= '0; lit_ret <= P_CACHE_BIT_W; st <= P_LIT;
                        end
                    end
                end
                P_CACHE_BIT_W: begin
                    if (lit_val[0]) begin wc[idx[2:0]] <= last; idx <= idx + 4'd1; end
                    st <= P_CACHE;
                end
                // ---- explicit + delta colours
                P_LIT_COL: begin
                    if (idx < n) begin lit_n <= bd; lit_val <= '0; lit_ret <= P_LIT_COL_W; st <= P_LIT; end
                    else st <= P_SORT;
                end
                P_LIT_COL_W: begin wc[idx[2:0]] <= lit_val[11:0]; idx <= idx + 4'd1; st <= P_EXTRA; end
                P_EXTRA: begin
                    if (idx < n) begin lit_n <= 4'd2; lit_val <= '0; lit_ret <= P_EXTRA_W; st <= P_LIT; end
                    else st <= P_SORT;
                end
                P_EXTRA_W: begin pbits <= bd - 4'd3 + lit_val[3:0]; st <= P_DELTA; end
                P_DELTA: begin
                    if (idx < n) begin lit_n <= pbits; lit_val <= '0; lit_ret <= P_DELTA_W; st <= P_LIT; end
                    else st <= P_SORT;
                end
                P_DELTA_W: begin
                    wc[idx[2:0]] <= d_new[11:0]; idx <= idx + 4'd1;
                    pbits <= (d_bits < pbits) ? d_bits : pbits;
                    st <= P_DELTA;
                end
                P_SORT: begin
                    begin
                        logic [95:0] packed_wc;
                        for (int k = 0; k < 8; k++) packed_wc[12 * k +: 12] = wc[k];
                        if (pl) col_u <= sort8(packed_wc); else col_y <= sort8(packed_wc);
                    end
                    st <= pl ? P_V_DE : P_UV_CHK;
                end
                P_UV_CHK: begin
                    pl <= 1'b1;
                    if (hc && uvmode == DC_PRED) st <= P_HAS;
                    else st <= P_DONE_MI;
                end
                // ---- v colours
                P_V_DE: begin lit_n <= 4'd1; lit_val <= '0; lit_ret <= P_V_DE_W; st <= P_LIT; idx <= 4'd0; end
                P_V_DE_W: begin
                    if (lit_val[0]) begin lit_n <= 4'd2; lit_val <= '0; lit_ret <= P_V_EXTRA_W; st <= P_LIT; end
                    else st <= P_V_LIT;
                end
                P_V_EXTRA_W: begin pbits <= bd - 4'd4 + lit_val[3:0]; lit_n <= bd; lit_val <= '0; lit_ret <= P_V0_W; st <= P_LIT; end
                P_V0_W: begin wc[0] <= lit_val[11:0]; idx <= 4'd1; st <= P_V_DELTA; end
                P_V_DELTA: begin
                    if (idx < n) begin lit_n <= pbits; lit_val <= '0; lit_ret <= P_V_DELTA_W; st <= P_LIT; end
                    else st <= P_DONE_MI;
                end
                P_V_DELTA_W: begin
                    vdelta <= lit_val[12:0];
                    if (lit_val[12:0] != 13'd0) begin lit_n <= 4'd1; lit_val <= '0; lit_ret <= P_V_SIGN_W; st <= P_LIT; end
                    else begin wc[idx[2:0]] <= wc[3'(idx - 4'd1)]; idx <= idx + 4'd1; st <= P_V_DELTA; end
                end
                P_V_SIGN_W: begin wc[idx[2:0]] <= v_wrapped; idx <= idx + 4'd1; st <= P_V_DELTA; end
                P_V_LIT: begin
                    if (idx < n) begin lit_n <= bd; lit_val <= '0; lit_ret <= P_V_LIT_W; st <= P_LIT; end
                    else st <= P_DONE_MI;
                end
                P_V_LIT_W: begin wc[idx[2:0]] <= lit_val[11:0]; idx <= idx + 4'd1; st <= P_V_LIT; end
                P_DONE_MI: begin
                    if (pl) for (int k = 0; k < 8; k++) col_v[12 * k +: 12] <= wc[k];
                    done_mi <= 1'b1; st <= P_IDLE;
                end
                // ---- palette_tokens
                T_START: begin
                    // palette_tokens dims: chroma is subsampled, and a chroma block narrower / shorter than 4
                    // (a 4xN / Nx4 block in 4:2:0 or 4:2:2) is widened by 2 (spec: blockWidth += 2, onscreenWidth += 2)
                    n <= pl ? pal_uv : pal_y;
                    tw <= pl ? 7'(os_w >> hdr.ssx) + ((hdr.ssx && blk_w(bs) == 8'd4) ? 7'd2 : 7'd0) : 7'(os_w);
                    th <= pl ? 7'(os_h >> hdr.ssy) + ((hdr.ssy && blk_h(bs) == 8'd4) ? 7'd2 : 7'd0) : 7'(os_h);
                    tw_p[pl] <= pl ? 7'(os_w >> hdr.ssx) + ((hdr.ssx && blk_w(bs) == 8'd4) ? 7'd2 : 7'd0) : 7'(os_w);
                    th_p[pl] <= pl ? 7'(os_h >> hdr.ssy) + ((hdr.ssy && blk_h(bs) == 8'd4) ? 7'd2 : 7'd0) : 7'(os_h);
                    st <= T_NS;
                end
                T_NS: begin
                    // color_index_map_y/uv = read_ns(n): w = FloorLog2(n) + 1, m = (1 << w) - n, v = L(w - 1)
                    ns_w <= ns_w_c;
                    ns_m <= 4'((5'd1 << ns_w_c) - 5'(n));
                    lit_n <= ns_w_c - 4'd1; lit_val <= '0; lit_ret <= T_NS_W; st <= P_LIT;
                end
                T_NS_W: begin
                    ns_v <= lit_val[3:0];
                    if (lit_val[3:0] < ns_m) begin
                        d1[0] <= lit_val[2:0]; dcur[0] <= lit_val[2:0];
                        if (pl) cmap_uv[0] <= lit_val[2:0]; else cmap_y[0] <= lit_val[2:0];
                        di <= 8'd1; st <= T_DIAG;
                    end else begin lit_n <= 4'd1; lit_val <= '0; lit_ret <= T_NS_X_W; st <= P_LIT; end
                end
                T_NS_X_W: begin
                    begin
                        logic [3:0] v;
                        v = (ns_v << 1) - ns_m + lit_val[3:0];
                        d1[0] <= v[2:0]; dcur[0] <= v[2:0];
                        if (pl) cmap_uv[0] <= v[2:0]; else cmap_y[0] <= v[2:0];
                    end
                    di <= 8'd1; st <= T_DIAG;
                end
                T_DIAG: begin
                    // for i in 1 .. onscreenHeight + onscreenWidth - 2: j from min(i, W-1) down to max(0, i-H+1)
                    if (di > 8'(tw) + 8'(th) - 8'd2) st <= T_NEXT_PLANE;
                    else begin
                        dj <= (di < 8'(tw) - 8'd1) ? 7'(di) : tw - 7'd1;
                        djmin <= (di + 8'd1 > 8'(th)) ? 7'(di + 8'd1 - 8'(th)) : 7'd0;
                        st <= T_PIX;
                    end
                end
                T_PIX: st <= T_PIX_W;
                T_PIX_W: if (sq_done) begin
                    if (pl) cmap_uv[{6'(dr), 6'(dj)}] <= pix_val; else cmap_y[{6'(dr), 6'(dj)}] <= pix_val;
                    dcur[dj[5:0]] <= pix_val;
                    if (dj == djmin) begin
                        for (int k = 0; k < 64; k++) begin
                            d2[k] <= d1[k];
                            d1[k] <= (7'(k) == dj) ? pix_val : dcur[k];
                        end
                        di <= di + 8'd1; st <= T_DIAG;
                    end else begin dj <= dj - 7'd1; st <= T_PIX; end
                end
                T_NEXT_PLANE: begin
                    if (!pl && pal_uv != 4'd0) begin pl <= 1'b1; st <= T_START; end
                    else st <= T_DONE;
                end
                T_DONE: begin done_tok <= 1'b1; st <= P_IDLE; end
                // ---- literal reader: lit_n equiprobable bools, MSB first
                P_LIT: begin
                    if (lit_n == 4'd0) st <= lit_ret;
                    else st <= P_LIT_W;
                end
                P_LIT_W: if (sq_done) begin
                    lit_val <= {lit_val[14:0], sq_sym[0]};
                    lit_n <= lit_n - 4'd1;
                    st <= P_LIT;
                end
                default: st <= P_IDLE;
            endcase
        end
    end
endmodule
