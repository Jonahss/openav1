// Motion compensation filter (spec 7.11.3.4 block_inter_prediction): the separable 8-tap interpolation of one
// prediction unit of up to 64 x 64 samples from a patch of reference samples, four output columns per clock, with
// the compound blend (7.11.3.1: AVERAGE and DISTANCE) of two such predictions. Unscaled references stream (one
// group of four columns per clock in both passes); scaled references (x_step / y_step != 1024, 7.11.3.3) use the
// same datapath with per-lane source positions and filter phases, at a lower rate (below).
//
// Patch (written by ref_fetch, or the testbench): per list, rows 0 .. nrow - 1 of the clamped reference region
// starting at (x0 - 3, y0 - 3), 4 samples per 48-bit word, 18 words per row; x0 / y0 = startX >> 10 / startY >> 10
// of 7.11.3.3 and the fractions x_frac0 / y_frac0 = startX & 1023 / startY & 1023 come with the run. The writer
// must fill words 0 .. ngrp + 1 of every row (ngrp = ceil(w / 4): the four lanes of the last group read past w + 6)
// unscaled, and words 0 .. ceil(cols / 4) - 1 scaled, cols = (((w - 1) x_step + 1023) >> 10) + 8; rows:
// nrow = (((h - 1) y_step + 1023) >> 10) + 8 (= h + 7 unscaled; intermediateHeight of the spec).
// Scaled units are at most 32 x 32 (recon_top splits larger ones): then cols <= 70 and nrow <= 70 fit the patch.
//
// A run (start, with the list to filter) is two passes:
//   horizontal: for every patch row r and output column c (position p_c = x_frac0 + x_step c, source column
//       (p_c >> 10) + t - 3 = patch sample (p_c >> 10) + t): strip[r][c] =
//       Round2(sum_t taps[(p_c >> 6) & 15][t] * patch[r][(p_c >> 10) + t], InterRound0), 16-bit signed;
//   vertical: output row i (q_i = y_frac0 + y_step i): p = Round2(sum_t taps[(q_i >> 6) & 15][t] * strip[(q_i >> 10) + t][c],
//       InterRound1);
//       mode 0 (single): out = clip(p)                   (InterPostRound is 0 for a single prediction)
//       mode 1 (park):   pred[i][c] = p                  (list 0 of a compound block)
//       mode 2 (avg):    out = clip(Round2(pred + p, 1 + InterPostRound))
//       mode 3 (dist):   out = clip(Round2(wt0 * pred + wt1 * p, 4 + InterPostRound))
//       mode 4 (pair):   out_pair = {p (list 1), pred (list 0)} unblended, 17-bit signed lanes, for a masked blend outside
//       (compound intermediates exceed 16 bits for extreme 10/12-bit content: up to ~40000 with the sharp filter)
// Filters: interp_x / interp_y are the block's interp_filter[1] / [0] (0 EIGHTTAP, 1 SMOOTH, 2 SHARP, 3 BILINEAR);
// the 4-tap rows (4, 5) replace EIGHTTAP / SHARP and SMOOTH when w (resp. h) <= 4. Rounding: InterRound0 = 3 (5 at
// 12-bit), InterRound1 = 11 single (9 at 12-bit) / 7 compound, InterPostRound = 14 - InterRound0 - InterRound1.
// Output: 4 samples at (out_x .. out_x + 3, out_y) with a lane mask (w = 2 predicts two columns), unit-relative,
// in group-major order (all rows of group 0, then group 1, ...).
// Warp (warp = 1): one 8 x 8 of block_warp per run from a 15-row x 4-word patch at (ix4 - 7, iy4 - 7): the unscaled
// streams (15 rows, 2 groups; 8 output rows) with Warped_Filters taps per sample from (sx4, alpha, beta) and
// (sy4, gamma, delta); the blend modes apply as for any unit.
// Cycles, unscaled: (h + 7) (ngrp + 2) + ngrp (h + 7) + ~12; a 64 x 64 unit is ~2,430. Scaled: 16 (position table)
// + nrow (words + ngrp) + ngrp h 8 + ~12: a 32 x 32 unit with 2:1 steps is ~5,400.
module mc_filt
  import mc_tables_pkg::*;
#(
    parameter int PW = 12                      // sample width (covers 8 / 10 / 12-bit)
) (
    input  logic          clk,
    input  logic          rst,
    // ---- patch write port ----
    input  logic          patch_we,
    input  logic          patch_list,
    input  logic [6:0]    patch_row,           // 0 .. 71
    input  logic [4:0]    patch_w4,            // word 0 .. 17 (samples 4 w4 .. 4 w4 + 3)
    input  logic [4*PW-1:0] patch_d4,
    // ---- run (parameters sampled at start) ----
    input  logic          start,
    input  logic          lst,                 // the patch list to filter
    input  logic [6:0]    w,                   // 2, 4, 8, .. 64 (<= 32 when scaled)
    input  logic [6:0]    h,
    input  logic [1:0]    interp_x,            // interp_filter[1]
    input  logic [1:0]    interp_y,            // interp_filter[0]
    input  logic [9:0]    x_frac0,             // startX & 1023 (unscaled: phase << 6 | 32)
    input  logic [9:0]    y_frac0,
    input  logic [11:0]   x_step,              // 1/1024 sample per output column; 1024 = unscaled (64 .. 2048)
    input  logic [11:0]   y_step,
    // ---- warped motion (7.11.3.5 block_warp, one 8 x 8 per run; w, h, the fractions and steps are ignored) ----
    input  logic          warp,
    input  logic signed [15:0] w_alpha, w_beta, w_gamma, w_delta,   // setup_shear
    input  logic [15:0]   w_sx4,               // the 8x8's projected position, fractional parts (x4 & 0xFFFF)
    input  logic [15:0]   w_sy4,
    input  logic [3:0]    bit_depth,
    input  logic [2:0]    mode,                // 0 single, 1 park, 2 average, 3 distance, 4 pair (masked blends outside)
    input  logic [4:0]    wt0,                 // distance weights (Quant_Dist_Lookup): parked list, this list
    input  logic [4:0]    wt1,
    output logic          busy,
    output logic          done,
    // ---- output stream (modes 0, 2, 3) ----
    output logic          out_valid,
    output logic [6:0]    out_x,
    output logic [6:0]    out_y,
    output logic [4*PW-1:0] out_pix4,
    output logic [3:0]    out_mask,
    output logic [135:0]  out_pair             // mode 4: lane l: list 0 at [17l +: 17], list 1 at [68 + 17l +: 17] (signed)
);
    localparam int PATCH_WPR = 18;                        // words per patch row
    localparam int PATCH_ROWS = 72;
    localparam int PATCH_N = 2 * PATCH_ROWS * PATCH_WPR;  // 2592
    localparam int STRIP_ROWS = 71;                       // nrow <= 71
    localparam int GRP = 16;                              // groups per row (64 / 4)

    // ================================================================ memories
    logic [4*PW-1:0] patch_ram [0:PATCH_N-1];
    logic [63:0]     strip_ram [0:STRIP_ROWS*GRP-1];      // intermediate: 4 x 16-bit signed per word
    logic [67:0]     pred_ram  [0:64*GRP-1];              // parked list-0 prediction: 4 x 17-bit signed per word

    function automatic logic [11:0] patch_addr(input logic l, input logic [6:0] r, input logic [4:0] w4);
        return 12'(l) * 12'(PATCH_ROWS * PATCH_WPR) + 12'(r) * 12'(PATCH_WPR) + 12'(w4);
    endfunction

    logic [11:0] patch_waddr;
    assign patch_waddr = patch_addr(patch_list, patch_row, patch_w4);
    always_ff @(posedge clk) if (patch_we) patch_ram[patch_waddr] <= patch_d4;

    // ================================================================ run parameters
    logic        p_list;
    logic        p_scaled;                               // x_step or y_step != 1024
    logic        p_warp;                                 // block_warp: 15 x 8 horizontal, 8 x 8 vertical, per-sample taps
    logic signed [15:0] p_wa, p_wb, p_wg, p_wd;
    logic [15:0] p_wsx, p_wsy;
    logic [4:0]  p_ngrp;                                 // ceil(w / 4)
    logic [6:0]  p_h;
    logic [6:0]  p_nrow;                                 // patch / strip rows to filter
    logic [3:0]  p_bd;
    logic [2:0]  p_mode;
    logic [4:0]  p_wt0, p_wt1;
    logic [3:0]  p_r0, p_r1, p_post;
    logic [3:0]  p_mask;
    logic [2:0]  p_fidx, p_fidy;                         // filter rows of the two passes
    logic [9:0]  p_xf0, p_yf0;
    logic [11:0] p_xs, p_ys;
    logic signed [8:0] tx [0:7];                         // taps of the two passes, unscaled (one phase per run)
    logic signed [8:0] ty [0:7];
    // scaled: per output group, the four lanes' patch sample offsets and phases, and the last patch word they need
    logic [6:0]  g_int [0:GRP-1][0:3];
    logic [3:0]  g_ph  [0:GRP-1][0:3];
    logic [4:0]  g_lw  [0:GRP-1];

    function automatic logic [2:0] filt_id(input logic [1:0] interp, input logic is_small);
        if (!is_small) return 3'(interp);
        case (interp)
            2'd1: return 3'd5;                           // EIGHTTAP_SMOOTH -> 4-tap smooth
            2'd3: return 3'd3;                           // BILINEAR
            default: return 3'd4;                        // EIGHTTAP, EIGHTTAP_SHARP -> 4-tap regular
        endcase
    endfunction

    // ================================================================ control
    typedef enum logic [2:0] {S_IDLE, S_PRE, S_H, S_HS, S_V, S_VS, S_DRAIN} st_t;
    st_t st;
    logic [6:0] hr;                                      // horizontal: patch row
    logic [4:0] hk;                                      // horizontal: next word of the row to read
    logic [4:0] hg;                                      // horizontal, scaled: the group being completed
    logic [4:0] pg;                                      // S_PRE: group whose positions are computed
    logic [16:0] ppos;                                   // S_PRE: position of lane 0 of group pg
    logic [4:0] vg;                                      // vertical: group
    logic [6:0] vk;                                      // vertical, unscaled: strip row read (0 .. nrow - 1)
    logic [6:0] vi;                                      // vertical, scaled: output row
    logic [2:0] vt;                                      // vertical, scaled: tap row of the output row (0 .. 7)
    logic [16:0] vpos;                                   // vertical, scaled: q_i = y_frac0 + y_step i
    logic [3:0] drain;
    assign busy = st != S_IDLE;

    // ---------------------------------------------------------------- memory ports
    logic [11:0] prd_addr;  logic [4*PW-1:0] prd_q, prd_q1;   // patch read (registered data; q1 = one cycle later, with the h1 tags)
    logic [10:0] srd_addr;  logic [63:0] srd_q, srd_q1;  // strip read (q1 with the v1 tags)
    logic [10:0] swr_addr;  logic [63:0] swr_d;  logic swr_we;
    logic [9:0]  pdr_addr;  logic [67:0] pdr_q, pdr_q1;  // parked prediction read (q1 with the v1 tags)
    logic [9:0]  pwr_addr;  logic [67:0] pwr_d;  logic pwr_we;
    logic [6:0]  srd_row;
    always_ff @(posedge clk) begin
        prd_q <= patch_ram[prd_addr];
        if (swr_we) strip_ram[swr_addr] <= swr_d;
        srd_q <= strip_ram[srd_addr];
        if (pwr_we) pred_ram[pwr_addr] <= pwr_d;
        pdr_q <= pred_ram[pdr_addr];
        prd_q1 <= prd_q; srd_q1 <= srd_q; pdr_q1 <= pdr_q;
    end
    always_comb begin
        prd_addr = patch_addr(p_list, hr, hk);
        srd_row  = p_scaled ? 7'(vpos[16:10]) + 7'(vt) : vk;
        srd_addr = 11'(srd_row) * 11'(GRP) + 11'(vg);
        pdr_addr = (p_scaled ? 10'(vi) : 10'(vk - 7'd7)) * 10'(GRP) + 10'(vg);
    end

    // ---------------------------------------------------------------- horizontal pass pipeline
    // h0: read issued (hr, hk), or (scaled) an emit-only cycle for a group whose words are already in the window
    // h1: prd_q1 is word hk of row hr; the window W0 .. W4 holds the previous words (W4 = newest); the 32 samples
    //     and taps of the group are selected and registered
    // h2: 32 products registered      h3: sums rounded      h4: strip[r][g] written
    logic        h0_v, h0_rd, h0_em;  logic [6:0] h0_r;  logic [4:0] h0_k, h0_g;
    logic        h1_v, h1_rd, h1_em;  logic [6:0] h1_r;  logic [4:0] h1_k, h1_g;
    logic [4*PW-1:0] W0, W1, W2, W3, W4;
    logic        h2_v;  logic [6:0] h2_r;  logic [4:0] h2_g;
    logic [PW-1:0]      hsamp [0:3][0:7];
    logic signed [8:0]  htap  [0:3][0:7];
    logic        h3_v;  logic [6:0] h3_r;  logic [4:0] h3_g;
    logic signed [20:0] hp [0:3][0:7];
    logic        h4_v;  logic [6:0] h4_r;  logic [4:0] h4_g;
    logic signed [15:0] hs [0:3];

    // sample s (0 .. 19) of the 20-sample window {a, b, c, d, e} (a = oldest)
    function automatic logic [PW-1:0] wsamp(input logic [4*PW-1:0] a, input logic [4*PW-1:0] b, input logic [4*PW-1:0] c,
                                            input logic [4*PW-1:0] d, input logic [4*PW-1:0] e, input logic [4:0] s);
        case (s[4:2])
            3'd0: return a[s[1:0]*PW +: PW];
            3'd1: return b[s[1:0]*PW +: PW];
            3'd2: return c[s[1:0]*PW +: PW];
            3'd3: return d[s[1:0]*PW +: PW];
            default: return e[s[1:0]*PW +: PW];
        endcase
    endfunction

    // warp filter row for lane l of group g (i2 = 4 g + l - 4) at row i1 (horizontal: patch row - 7; vertical:
    // output row - 4): Round2(base + a i2 + b i1, 10) + 64, in 0 .. 192 when the shear is valid
    function automatic logic [7:0] warp_offs(input logic [15:0] base, input logic signed [15:0] a, input logic signed [15:0] b,
                                             input logic [4:0] g, input int l, input logic signed [7:0] i1);
        logic signed [23:0] sx;
        sx = 24'(signed'({8'b0, base})) + 24'(a) * 24'(signed'(5'(g) * 5'd4 + 5'(l) - 5'd4)) + 24'(b) * 24'(i1);
        return 8'(((sx + 24'sd512) >>> 10) + 24'sd64);
    endfunction

    // ---------------------------------------------------------------- vertical pass pipeline
    // v0: reads issued (strip row, vg) and the parked word (output row, vg); scaled: the 8 tap rows of an output row
    //     in turn, the last one tagged to compute
    // v1: srd_q1 / pdr_q1 are their data; 8-row window shift; samples and taps registered
    // v2: 32 products registered      v3: sums rounded (p); parked word delayed      v4: blend / clip / output, or park
    logic        v0_v, v0_c;  logic [6:0] v0_k, v0_i;  logic [4:0] v0_g;  logic [3:0] v0_ph;
    logic        v1_v, v1_c;  logic [6:0] v1_k, v1_i;  logic [4:0] v1_g;  logic [3:0] v1_ph;
    logic [63:0] vwin [0:6];                             // the 7 strip words before srd_q1 (vwin[0] = oldest)
    logic        v2_v;  logic [6:0] v2_i;  logic [4:0] v2_g;  logic [67:0] v2_pred;
    logic signed [15:0] vsamp [0:3][0:7];
    logic signed [8:0]  vtap  [0:3][0:7];
    logic        v3_v;  logic [6:0] v3_i;  logic [4:0] v3_g;  logic [67:0] v3_pred;
    logic signed [24:0] vp [0:3][0:7];
    logic        v4_v;  logic [6:0] v4_i;  logic [4:0] v4_g;  logic [67:0] v4_pred;
    logic signed [16:0] vs [0:3];                       // the vertical output: 17 bits (compound intermediates)
    logic [PW-1:0] pix_max;
    assign pix_max = PW'((13'd1 << p_bd) - 13'd1);

    // rows the passes cover: (((h - 1) y_step + 1023) >> 10) + 8
    logic [18:0] nrow_calc;
    assign nrow_calc = 19'(h - 7'd1) * 19'(y_step) + 19'd1023;

    always_ff @(posedge clk) begin
        done <= 1'b0;
        swr_we <= 1'b0; pwr_we <= 1'b0;
        out_valid <= 1'b0;
        h0_v <= 1'b0; v0_v <= 1'b0;
        if (rst) begin
            st <= S_IDLE; h1_v <= 1'b0; h2_v <= 1'b0; h3_v <= 1'b0; h4_v <= 1'b0;
            v1_v <= 1'b0; v2_v <= 1'b0; v3_v <= 1'b0; v4_v <= 1'b0;
        end else begin
            // ---------------- FSM: the address streams
            case (st)
                S_IDLE: if (start) begin
                    p_list <= lst;
                    p_scaled <= !warp && ((x_step != 12'd1024) || (y_step != 12'd1024));
                    p_warp <= warp; p_wa <= w_alpha; p_wb <= w_beta; p_wg <= w_gamma; p_wd <= w_delta; p_wsx <= w_sx4; p_wsy <= w_sy4;
                    p_ngrp <= warp ? 5'd2 : 5'((w + 7'd3) >> 2); p_h <= warp ? 7'd8 : h;
                    p_nrow <= warp ? 7'd15 : 7'(nrow_calc[18:10] + 9'd8);
                    p_bd <= bit_depth; p_mode <= mode; p_wt0 <= wt0; p_wt1 <= wt1;
                    p_r0 <= (bit_depth == 4'd12) ? 4'd5 : 4'd3;
                    p_r1 <= (mode != 3'd0) ? 4'd7 : (bit_depth == 4'd12) ? 4'd9 : 4'd11;
                    p_post <= 4'd14 - ((bit_depth == 4'd12) ? 4'd5 : 4'd3)
                                    - ((mode != 3'd0) ? 4'd7 : (bit_depth == 4'd12) ? 4'd9 : 4'd11);
                    p_mask <= (w == 7'd2 && !warp) ? 4'b0011 : 4'b1111;
                    p_fidx <= filt_id(interp_x, w <= 7'd4); p_fidy <= filt_id(interp_y, h <= 7'd4);
                    p_xf0 <= x_frac0; p_yf0 <= y_frac0; p_xs <= x_step; p_ys <= y_step;
                    for (int t = 0; t < 8; t++) begin
                        tx[t] <= subpel_tap(filt_id(interp_x, w <= 7'd4), x_frac0[9:6], 3'(t));
                        ty[t] <= subpel_tap(filt_id(interp_y, h <= 7'd4), y_frac0[9:6], 3'(t));
                    end
                    hr <= 7'd0; hk <= 5'd0; hg <= 5'd0;
                    if (!warp && ((x_step != 12'd1024) || (y_step != 12'd1024))) begin
                        pg <= 5'd0; ppos <= 17'(x_frac0); st <= S_PRE;
                    end else st <= S_H;
                end
                S_PRE: begin                             // one group per cycle: lane positions p = ppos + l x_step
                    for (int l = 0; l < 4; l++) begin
                        logic [16:0] p;
                        p = ppos + 17'(p_xs) * 17'(l);
                        g_int[pg[3:0]][l] <= p[16:10]; g_ph[pg[3:0]][l] <= p[9:6];
                        if (l == 3) g_lw[pg[3:0]] <= 5'((8'(p[16:10]) + 8'd7) >> 2);
                    end
                    ppos <= ppos + 17'(p_xs) * 17'd4;
                    if (pg == 5'(GRP - 1)) st <= S_HS; else pg <= pg + 5'd1;
                end
                S_H: begin                               // unscaled: one word per cycle, group g emitted at word g + 2
                    h0_v <= 1'b1; h0_rd <= 1'b1; h0_em <= (hk >= 5'd2); h0_r <= hr; h0_k <= hk; h0_g <= hk - 5'd2;
                    if (hk == p_ngrp + 5'd1) begin
                        hk <= 5'd0;
                        if (hr == p_nrow - 7'd1) begin vg <= 5'd0; vk <= 7'd0; st <= S_V; end
                        else hr <= hr + 7'd1;
                    end else hk <= hk + 5'd1;
                end
                S_HS: begin                              // scaled: read until group hg's last word is in, emit, next group
                    h0_v <= 1'b1; h0_r <= hr; h0_g <= hg;
                    if (hk <= g_lw[hg[3:0]]) begin
                        h0_rd <= 1'b1; h0_k <= hk; h0_em <= (hk == g_lw[hg[3:0]]);
                        hk <= hk + 5'd1;
                    end else begin
                        h0_rd <= 1'b0; h0_k <= hk - 5'd1; h0_em <= 1'b1;
                    end
                    if (hk >= g_lw[hg[3:0]]) begin            // this cycle completes group hg
                        if (hg == p_ngrp - 5'd1) begin
                            hg <= 5'd0; hk <= 5'd0;
                            if (hr == p_nrow - 7'd1) begin
                                vg <= 5'd0; vi <= 7'd0; vt <= 3'd0; vpos <= 17'(p_yf0); st <= S_VS;
                            end else hr <= hr + 7'd1;
                        end else hg <= hg + 5'd1;
                    end
                end
                S_V: begin
                    // the strip is complete before any read of it that matters: the last horizontal write lands 4
                    // cycles after its issue, and the vertical pass reads rows 0 .. 6 of group 0 first (written long
                    // before); the last-written word is read at the earliest nrow cycles into S_V
                    v0_v <= 1'b1; v0_k <= vk; v0_g <= vg; v0_c <= (vk >= 7'd7); v0_i <= vk - 7'd7; v0_ph <= p_yf0[9:6];
                    if (vk == p_nrow - 7'd1) begin
                        vk <= 7'd0;
                        if (vg == p_ngrp - 5'd1) begin drain <= 4'd8; st <= S_DRAIN; end
                        else vg <= vg + 5'd1;
                    end else vk <= vk + 7'd1;
                end
                S_VS: begin                              // scaled: 8 tap rows per output row, compute on the eighth
                    v0_v <= 1'b1; v0_k <= srd_row; v0_g <= vg; v0_c <= (vt == 3'd7); v0_i <= vi; v0_ph <= vpos[9:6];
                    if (vt == 3'd7) begin
                        vt <= 3'd0;
                        if (vi == p_h - 7'd1) begin
                            vi <= 7'd0; vpos <= 17'(p_yf0);
                            if (vg == p_ngrp - 5'd1) begin drain <= 4'd8; st <= S_DRAIN; end
                            else vg <= vg + 5'd1;
                        end else begin vi <= vi + 7'd1; vpos <= vpos + 17'(p_ys); end
                    end else vt <= vt + 3'd1;
                end
                S_DRAIN: if (drain == 4'd0) begin done <= 1'b1; st <= S_IDLE; end else drain <= drain - 4'd1;
                default: st <= S_IDLE;
            endcase

            // ---------------- horizontal pipeline
            h1_v <= h0_v; h1_rd <= h0_rd; h1_em <= h0_em; h1_r <= h0_r; h1_k <= h0_k; h1_g <= h0_g;
            if (h1_v && h1_rd) begin W0 <= W1; W1 <= W2; W2 <= W3; W3 <= W4; W4 <= prd_q1; end
            h2_v <= h1_v && h1_em; h2_r <= h1_r; h2_g <= h1_g;
            for (int l = 0; l < 4; l++)
                for (int t = 0; t < 8; t++)
                    if (!p_scaled) begin
                        // unscaled (and warp): lane l's taps are samples l + t of words k - 2, k - 1, k = W3, W4, prd_q1
                        hsamp[l][t] <= wsamp(W1, W2, W3, W4, prd_q1, 5'(8 + l + t));
                        htap[l][t]  <= p_warp ? 9'(warp_tap(warp_offs(p_wsx, p_wa, p_wb, h1_g, l, 8'(signed'({1'b0, h1_r})) - 8'sd7), 3'(t))) : tx[t];
                    end else begin
                        // scaled: window words kw - 4 .. kw = {W1 .. W4, prd_q1} on a read cycle (kw = h1_k), else
                        // {W0 .. W4}; lane l's first tap is sample g_int - 4 (kw - 4) of it
                        logic [6:0] off;
                        off = g_int[h1_g[3:0]][l] + 7'd16 - {h1_k, 2'b00};
                        hsamp[l][t] <= h1_rd ? wsamp(W1, W2, W3, W4, prd_q1, 5'(off + 7'(t)))
                                             : wsamp(W0, W1, W2, W3, W4, 5'(off + 7'(t)));
                        htap[l][t]  <= subpel_tap(p_fidx, g_ph[h1_g[3:0]][l], 3'(t));
                    end
            h3_v <= h2_v; h3_r <= h2_r; h3_g <= h2_g;
            for (int l = 0; l < 4; l++)
                for (int t = 0; t < 8; t++)
                    hp[l][t] <= 21'(signed'({1'b0, hsamp[l][t]})) * 21'(htap[l][t]);
            h4_v <= h3_v; h4_r <= h3_r; h4_g <= h3_g;
            for (int l = 0; l < 4; l++) begin
                logic signed [23:0] s;
                s = 24'sd0;
                for (int t = 0; t < 8; t++) s = s + 24'(hp[l][t]);
                hs[l] <= 16'((s + (24'sd1 <<< (p_r0 - 4'd1))) >>> p_r0);
            end
            if (h4_v) begin
                swr_we <= 1'b1; swr_addr <= 11'(h4_r) * 11'(GRP) + 11'(h4_g);
                swr_d <= {hs[3], hs[2], hs[1], hs[0]};
            end

            // ---------------- vertical pipeline
            v1_v <= v0_v; v1_c <= v0_c; v1_k <= v0_k; v1_i <= v0_i; v1_g <= v0_g; v1_ph <= v0_ph;
            if (v1_v) begin
                for (int j = 0; j < 6; j++) vwin[j] <= vwin[j + 1];
                vwin[6] <= srd_q1;
            end
            // output row i from the 8 strip rows {vwin[0..6], srd_q1}
            v2_v <= v1_v && v1_c; v2_i <= v1_i; v2_g <= v1_g; v2_pred <= pdr_q1;
            for (int l = 0; l < 4; l++)
                for (int t = 0; t < 8; t++)
                    vsamp[l][t] <= signed'((t < 7) ? vwin[t][l*16 +: 16] : srd_q1[l*16 +: 16]);
            for (int l = 0; l < 4; l++)
                for (int t = 0; t < 8; t++)
                    vtap[l][t] <= p_warp ? 9'(warp_tap(warp_offs(p_wsy, p_wg, p_wd, v1_g, l, 8'(signed'({1'b0, v1_i})) - 8'sd4), 3'(t)))
                                : p_scaled ? subpel_tap(p_fidy, v1_ph, 3'(t)) : ty[t];
            v3_v <= v2_v; v3_i <= v2_i; v3_g <= v2_g; v3_pred <= v2_pred;
            for (int l = 0; l < 4; l++)
                for (int t = 0; t < 8; t++)
                    vp[l][t] <= 25'(vsamp[l][t]) * 25'(vtap[l][t]);
            v4_v <= v3_v; v4_i <= v3_i; v4_g <= v3_g; v4_pred <= v3_pred;
            for (int l = 0; l < 4; l++) begin
                logic signed [27:0] s;
                s = 28'sd0;
                for (int t = 0; t < 8; t++) s = s + 28'(vp[l][t]);
                vs[l] <= 17'((s + (28'sd1 <<< (p_r1 - 4'd1))) >>> p_r1);
            end
            // v5: park, or blend / clip / output
            if (v4_v) begin
                if (p_mode == 3'd1) begin
                    pwr_we <= 1'b1; pwr_addr <= 10'(v4_i) * 10'(GRP) + 10'(v4_g);
                    pwr_d <= {vs[3], vs[2], vs[1], vs[0]};
                end else begin
                    out_valid <= 1'b1; out_x <= {v4_g, 2'b00}; out_y <= v4_i; out_mask <= p_mask;
                    out_pair <= {vs[3], vs[2], vs[1], vs[0], v4_pred};
                    for (int l = 0; l < 4; l++) begin
                        logic signed [21:0] b, p0, p1, w0s, w1s;
                        w0s = 22'(signed'({1'b0, p_wt0})); w1s = 22'(signed'({1'b0, p_wt1}));
                        p0 = 22'(signed'(v4_pred[l*17 +: 17]));
                        p1 = 22'(vs[l]);
                        case (p_mode)
                            3'd2: b = (p0 + p1 + (22'sd1 <<< p_post)) >>> (p_post + 4'd1);
                            3'd3: b = (w0s * p0 + w1s * p1 + (22'sd1 <<< (p_post + 4'd3))) >>> (p_post + 4'd4);   // weights as signed: an unsigned operand would make the product unsigned
                            default: b = p1;
                        endcase
                        out_pix4[l*PW +: PW] <= (b < 22'sd0) ? PW'(0) : (b > 22'(pix_max)) ? pix_max : PW'(b);
                    end
                end
            end
        end
    end
endmodule
