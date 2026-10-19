// Motion compensation filter (spec 7.11.3.4 block_inter_prediction, unscaled references): the separable 8-tap
// interpolation of one prediction unit of up to 64 x 64 samples from a patch of reference samples, four output
// columns per clock, with the compound blend (7.11.3.1: AVERAGE and DISTANCE) of two such predictions.
//
// Patch (written by ref_fetch, or the testbench): per list, rows 0 .. h + 6 of the clamped reference region
// [y0 - 3, y0 + h + 4) x [x0 - 3, x0 + w + 4), 4 samples per 48-bit word, 18 words per row. The writer must fill
// words 0 .. ngrp + 1 of every row (ngrp = ceil(w / 4)): the four lanes of the last group read past w + 6.
//
// A run (start, with the list to filter) is two passes:
//   horizontal: for every patch row r (h + 7 of them) and group g: strip[r][4g + l] =
//       Round2(sum_t taps_x[t] * patch[r][4g + l + t], InterRound0), 16-bit signed;
//   vertical: for every group g and output row i: p = Round2(sum_t taps_y[t] * strip[i + t][4g + l], InterRound1);
//       mode 0 (single): out = clip(p)                   (InterPostRound is 0 for a single prediction)
//       mode 1 (park):   pred[i][4g + l] = p             (list 0 of a compound block)
//       mode 2 (avg):    out = clip(Round2(pred + p, 1 + InterPostRound))
//       mode 3 (dist):   out = clip(Round2(wt0 * pred + wt1 * p, 4 + InterPostRound))
//       mode 4 (pair):   out_pair = {p (list 1), pred (list 0)} unblended, 16-bit signed lanes, for a masked blend outside
// Filters: interp_x / interp_y are the block's interp_filter[1] / [0] (0 EIGHTTAP, 1 SMOOTH, 2 SHARP, 3 BILINEAR);
// the 4-tap rows (4, 5) replace EIGHTTAP / SHARP and SMOOTH when w (resp. h) <= 4. Rounding: InterRound0 = 3 (5 at
// 12-bit), InterRound1 = 11 single (9 at 12-bit) / 7 compound, InterPostRound = 14 - InterRound0 - InterRound1.
// Output: 4 samples at (out_x .. out_x + 3, out_y) with a lane mask (w = 2 predicts two columns), unit-relative,
// in group-major order (all rows of group 0, then group 1, ...).
// Cycles: (h + 7) (ngrp + 2) + ngrp (h + 7) + ~10; a 64 x 64 unit is 2,422.
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
    input  logic [6:0]    w,                   // 2, 4, 8, .. 64
    input  logic [6:0]    h,
    input  logic [1:0]    interp_x,            // interp_filter[1]
    input  logic [1:0]    interp_y,            // interp_filter[0]
    input  logic [3:0]    fx,                  // subpel phase (p >> 6) & 15
    input  logic [3:0]    fy,
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
    output logic [127:0]  out_pair             // mode 4: lane l: list 0 at [16l +: 16], list 1 at [64 + 16l +: 16] (signed)
);
    localparam int PATCH_WPR = 18;                        // words per patch row
    localparam int PATCH_ROWS = 72;
    localparam int PATCH_N = 2 * PATCH_ROWS * PATCH_WPR;  // 2592
    localparam int STRIP_ROWS = 71;                       // h + 7 <= 71
    localparam int GRP = 16;                              // groups per row (64 / 4)

    // ================================================================ memories
    logic [4*PW-1:0] patch_ram [0:PATCH_N-1];
    logic [63:0]     strip_ram [0:STRIP_ROWS*GRP-1];      // intermediate: 4 x 16-bit signed per word
    logic [63:0]     pred_ram  [0:64*GRP-1];              // parked list-0 prediction: 4 x 16-bit signed per word

    function automatic logic [11:0] patch_addr(input logic l, input logic [6:0] r, input logic [4:0] w4);
        return 12'(l) * 12'(PATCH_ROWS * PATCH_WPR) + 12'(r) * 12'(PATCH_WPR) + 12'(w4);
    endfunction

    logic [11:0] patch_waddr;
    assign patch_waddr = patch_addr(patch_list, patch_row, patch_w4);
    always_ff @(posedge clk) if (patch_we) patch_ram[patch_waddr] <= patch_d4;

    // ================================================================ run parameters
    logic        p_list;
    logic [4:0]  p_ngrp;                                 // ceil(w / 4)
    logic [6:0]  p_nrow;                                 // h + 7
    logic [3:0]  p_bd;
    logic [2:0]  p_mode;
    logic [4:0]  p_wt0, p_wt1;
    logic [3:0]  p_r0, p_r1, p_post;
    logic [3:0]  p_mask;
    logic signed [8:0] tx [0:7];                         // taps of the two passes
    logic signed [8:0] ty [0:7];

    function automatic logic [2:0] filt_id(input logic [1:0] interp, input logic is_small);
        if (!is_small) return 3'(interp);
        case (interp)
            2'd1: return 3'd5;                           // EIGHTTAP_SMOOTH -> 4-tap smooth
            2'd3: return 3'd3;                           // BILINEAR
            default: return 3'd4;                        // EIGHTTAP, EIGHTTAP_SHARP -> 4-tap regular
        endcase
    endfunction

    // ================================================================ control
    typedef enum logic [1:0] {S_IDLE, S_H, S_V, S_DRAIN} st_t;
    st_t st;
    logic [6:0] hr;                                      // horizontal: patch row
    logic [4:0] hk;                                      // horizontal: word index within the row (0 .. ngrp + 1)
    logic [4:0] vg;                                      // vertical: group
    logic [6:0] vk;                                      // vertical: strip row read (0 .. h + 6)
    logic [3:0] drain;
    assign busy = st != S_IDLE;

    // ---------------------------------------------------------------- memory ports
    logic [11:0] prd_addr;  logic [4*PW-1:0] prd_q, prd_q1;   // patch read (registered data; q1 = one cycle later, with the h1 tags)
    logic [10:0] srd_addr;  logic [63:0] srd_q, srd_q1;  // strip read (q1 with the v1 tags)
    logic [10:0] swr_addr;  logic [63:0] swr_d;  logic swr_we;
    logic [9:0]  pdr_addr;  logic [63:0] pdr_q, pdr_q1;  // parked prediction read (q1 with the v1 tags)
    logic [9:0]  pwr_addr;  logic [63:0] pwr_d;  logic pwr_we;
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
        srd_addr = 11'(vk) * 11'(GRP) + 11'(vg);
        pdr_addr = 10'(vk - 7'd7) * 10'(GRP) + 10'(vg);
    end

    // ---------------------------------------------------------------- horizontal pass pipeline
    // h0: read issued (hr, hk)        h1: prd_q1 is word hk of row hr; window {win0, win1, prd_q1} = words k-2, k-1, k
    // h2: 32 products registered      h3: sums rounded -> strip[r][g], g = k - 2
    logic        h0_v;  logic [6:0] h0_r;  logic [4:0] h0_k;
    logic        h1_v;  logic [6:0] h1_r;  logic [4:0] h1_k;
    logic [4*PW-1:0] win0, win1;
    logic        h2_v;  logic [6:0] h2_r;  logic [4:0] h2_g;
    logic signed [20:0] hp [0:3][0:7];
    logic        h3_v;  logic [6:0] h3_r;  logic [4:0] h3_g;
    logic signed [15:0] hs [0:3];

    function automatic logic [PW-1:0] wsamp(input logic [4*PW-1:0] a, input logic [4*PW-1:0] b, input logic [4*PW-1:0] c,
                                            input int s);
        if (s < 4) return a[s*PW +: PW];
        else if (s < 8) return b[(s-4)*PW +: PW];
        else return c[(s-8)*PW +: PW];
    endfunction

    // ---------------------------------------------------------------- vertical pass pipeline
    // v0: reads issued (vk, vg) and the parked word (vk - 7, vg)    v1: srd_q1 / pdr_q1 are their data; 8-row window shift
    // v2: 32 products registered                                     v3: sums rounded (p); parked word delayed
    // v4: blend / clip / output, or park
    logic        v0_v;  logic [6:0] v0_k;  logic [4:0] v0_g;
    logic        v1_v;  logic [6:0] v1_k;  logic [4:0] v1_g;
    logic [63:0] vwin [0:6];                             // the 7 strip words before srd_q1 (vwin[0] = oldest)
    logic        v2_v;  logic [6:0] v2_i;  logic [4:0] v2_g;  logic [63:0] v2_pred;
    logic signed [24:0] vp [0:3][0:7];
    logic        v3_v;  logic [6:0] v3_i;  logic [4:0] v3_g;  logic [63:0] v3_pred;
    logic signed [15:0] vs [0:3];
    logic [PW-1:0] pix_max;
    assign pix_max = PW'((13'd1 << p_bd) - 13'd1);

    always_ff @(posedge clk) begin
        done <= 1'b0;
        swr_we <= 1'b0; pwr_we <= 1'b0;
        out_valid <= 1'b0;
        h0_v <= 1'b0; v0_v <= 1'b0;
        if (rst) begin
            st <= S_IDLE; h1_v <= 1'b0; h2_v <= 1'b0; h3_v <= 1'b0; v1_v <= 1'b0; v2_v <= 1'b0; v3_v <= 1'b0;
        end else begin
            // ---------------- FSM: the address streams
            case (st)
                S_IDLE: if (start) begin
                    p_list <= lst;
                    p_ngrp <= 5'((w + 7'd3) >> 2); p_nrow <= h + 7'd7;
                    p_bd <= bit_depth; p_mode <= mode; p_wt0 <= wt0; p_wt1 <= wt1;
                    p_r0 <= (bit_depth == 4'd12) ? 4'd5 : 4'd3;
                    p_r1 <= (mode != 3'd0) ? 4'd7 : (bit_depth == 4'd12) ? 4'd9 : 4'd11;
                    p_post <= 4'd14 - ((bit_depth == 4'd12) ? 4'd5 : 4'd3)
                                    - ((mode != 3'd0) ? 4'd7 : (bit_depth == 4'd12) ? 4'd9 : 4'd11);
                    p_mask <= (w == 7'd2) ? 4'b0011 : 4'b1111;
                    for (int t = 0; t < 8; t++) begin
                        tx[t] <= subpel_tap(filt_id(interp_x, w <= 7'd4), fx, 3'(t));
                        ty[t] <= subpel_tap(filt_id(interp_y, h <= 7'd4), fy, 3'(t));
                    end
                    hr <= 7'd0; hk <= 5'd0; st <= S_H;
                end
                S_H: begin
                    h0_v <= 1'b1; h0_r <= hr; h0_k <= hk;
                    if (hk == p_ngrp + 5'd1) begin
                        hk <= 5'd0;
                        if (hr == p_nrow - 7'd1) begin vg <= 5'd0; vk <= 7'd0; st <= S_V; end
                        else hr <= hr + 7'd1;
                    end else hk <= hk + 5'd1;
                end
                S_V: begin
                    // the strip is complete before any read of it that matters: the last horizontal write lands 3
                    // cycles after its issue, and the vertical pass reads rows 0 .. 6 of group 0 first (written long
                    // before); the last-written word is read at the earliest nrow cycles into S_V
                    v0_v <= 1'b1; v0_k <= vk; v0_g <= vg;
                    if (vk == p_nrow - 7'd1) begin
                        vk <= 7'd0;
                        if (vg == p_ngrp - 5'd1) begin drain <= 4'd6; st <= S_DRAIN; end
                        else vg <= vg + 5'd1;
                    end else vk <= vk + 7'd1;
                end
                S_DRAIN: if (drain == 4'd0) begin done <= 1'b1; st <= S_IDLE; end else drain <= drain - 4'd1;
                default: st <= S_IDLE;
            endcase

            // ---------------- horizontal pipeline
            h1_v <= h0_v; h1_r <= h0_r; h1_k <= h0_k;
            if (h1_v) begin win0 <= win1; win1 <= prd_q1; end
            h2_v <= h1_v && (h1_k >= 5'd2); h2_r <= h1_r; h2_g <= h1_k - 5'd2;
            for (int l = 0; l < 4; l++)
                for (int t = 0; t < 8; t++)
                    hp[l][t] <= 21'(signed'({1'b0, wsamp(win0, win1, prd_q1, l + t)})) * 21'(tx[t]);
            h3_v <= h2_v; h3_r <= h2_r; h3_g <= h2_g;
            for (int l = 0; l < 4; l++) begin
                logic signed [23:0] s;
                s = 24'sd0;
                for (int t = 0; t < 8; t++) s = s + 24'(hp[l][t]);
                hs[l] <= 16'((s + (24'sd1 <<< (p_r0 - 4'd1))) >>> p_r0);
            end
            if (h3_v) begin
                swr_we <= 1'b1; swr_addr <= 11'(h3_r) * 11'(GRP) + 11'(h3_g);
                swr_d <= {hs[3], hs[2], hs[1], hs[0]};
            end

            // ---------------- vertical pipeline
            v1_v <= v0_v; v1_k <= v0_k; v1_g <= v0_g;
            if (v1_v) begin
                for (int j = 0; j < 6; j++) vwin[j] <= vwin[j + 1];
                vwin[6] <= srd_q1;
            end
            // output row i = k - 7 from strip rows i .. i + 7 = {vwin[0..6], srd_q1}
            v2_v <= v1_v && (v1_k >= 7'd7); v2_i <= v1_k - 7'd7; v2_g <= v1_g; v2_pred <= pdr_q1;
            for (int l = 0; l < 4; l++)
                for (int t = 0; t < 8; t++)
                    vp[l][t] <= 25'(signed'((t < 7) ? vwin[t][l*16 +: 16] : srd_q1[l*16 +: 16])) * 25'(ty[t]);
            v3_v <= v2_v; v3_i <= v2_i; v3_g <= v2_g; v3_pred <= v2_pred;
            for (int l = 0; l < 4; l++) begin
                logic signed [27:0] s;
                s = 28'sd0;
                for (int t = 0; t < 8; t++) s = s + 28'(vp[l][t]);
                vs[l] <= 16'((s + (28'sd1 <<< (p_r1 - 4'd1))) >>> p_r1);
            end
            // v4: park, or blend / clip / output
            if (v3_v) begin
                if (p_mode == 3'd1) begin
                    pwr_we <= 1'b1; pwr_addr <= 10'(v3_i) * 10'(GRP) + 10'(v3_g);
                    pwr_d <= {vs[3], vs[2], vs[1], vs[0]};
                end else begin
                    out_valid <= 1'b1; out_x <= {v3_g, 2'b00}; out_y <= v3_i; out_mask <= p_mask;
                    out_pair <= {vs[3], vs[2], vs[1], vs[0], v3_pred};
                    for (int l = 0; l < 4; l++) begin
                        logic signed [21:0] b, p0, p1, w0s, w1s;
                        w0s = 22'(signed'({1'b0, p_wt0})); w1s = 22'(signed'({1'b0, p_wt1}));
                        p0 = 22'(signed'(v3_pred[l*16 +: 16]));
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
