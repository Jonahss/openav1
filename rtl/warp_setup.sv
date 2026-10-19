// Warp parameter setup (spec 7.11.3.6 setup_shear, 7.11.3.7 resolve_divisor, 7.11.3.8 warp_estimation): the
// per-block, per-list computation that turns either the least-squares sums of the warp samples (LOCALWARP; the
// sums come from the syntax stage, blk_rec_t.ls) or the frame's global motion parameters (GLOBALMV with a
// non-translational gm_type) into the affine parameters p[0..5] and the shear terms alpha, beta, gamma, delta
// that block_warp (mc_filt mode 5) uses, plus the validity. Sequential: one shared 64 x 64 signed multiplier
// (operands set in one state, product read in the next) and a 64-bit variable round-shift; ~20 cycles per start,
// only warp blocks pay.
//
// Local (mode 0): det = A00 A11 - A01^2; LocalValid = det != 0; (divShift, divFactor) = resolve_divisor(det),
//   divShift -= 16 (the factor is shifted left when that goes negative); p2 = diag(A11 Bx0 - A01 Bx1),
//   p3 = nondiag(A00 Bx1 - A01 Bx0), p4 = nondiag(A11 By0 - A01 By1), p5 = diag(A00 By1 - A01 By0) with
//   x -> clip(round2signed(x divFactor, divShift)); p0 = clip(mvx 2^13 - (midX (p2 - 2^16) + midY p3), +-2^23),
//   p1 likewise with p4, p5 - 2^16 and mvy. Then the shear of those parameters.
// Global (mode 1): the shear of the given parameters.
// Shear: alpha0 = clip16(p2 - 2^16), beta0 = clip16(p3), (divShift, divFactor) = resolve_divisor(p2),
//   gamma0 = clip16(round2signed((p4 << 16) divFactor, divShift)), delta0 = clip16(p5 - round2signed(p3 p4
//   divFactor, divShift) - 2^16); alpha..delta = round2signed(x, 6) << 6; valid when 4|alpha| + 7|beta| < 2^16
//   and 4|gamma| + 4|delta| < 2^16 (and, local, det != 0).
module warp_setup
  import mc_tables_pkg::*;
(
    input  logic          clk,
    input  logic          rst,
    input  logic          start,
    input  logic          mode,                 // 0 local (sums), 1 global (params)
    input  logic signed [31:0] ls_a00, ls_a01, ls_a11, ls_bx0, ls_bx1, ls_by0, ls_by1,
    input  logic signed [15:0] mv_row, mv_col,  // Mv[0] of the block (1/8 luma samples)
    input  logic [15:0]   mid_x, mid_y,         // MiCol * 4 + w4 * 2 - 1, MiRow * 4 + h4 * 2 - 1
    input  logic signed [23:0] gm_p [0:5],      // gm_params of the reference (mode 1)
    output logic          busy,
    output logic          done,
    output logic          valid,                // warpValid (and LocalValid for mode 0)
    output logic signed [23:0] p [0:5],
    output logic signed [15:0] alpha, beta, gamma, delta
);
    // ---------------------------------------------------------------- helpers
    function automatic logic [5:0] floor_log2_64(input logic [63:0] v);
        for (int i = 63; i >= 0; i--) if (v[i]) return 6'(i);
        return 6'd0;
    endfunction
    // Round2Signed(v, s), s = 0 .. 63
    function automatic logic signed [63:0] r2s(input logic signed [63:0] v, input logic [5:0] s);
        logic [63:0] a, r;
        if (s == 6'd0) return v;
        a = v[63] ? 64'(-v) : 64'(v);
        r = (a + (64'd1 << (s - 6'd1))) >> s;
        return v[63] ? -64'(signed'(r)) : 64'(signed'(r));
    endfunction
    function automatic logic signed [15:0] clip16(input logic signed [63:0] v);
        if (v < -64'sd32768) return -16'sd32768;
        if (v > 64'sd32767) return 16'sd32767;
        return 16'(v);
    endfunction
    function automatic logic signed [23:0] clip24(input logic signed [63:0] v, input logic signed [23:0] lo, input logic signed [23:0] hi);
        if (v < 64'(lo)) return lo;
        if (v > 64'(hi)) return hi;
        return 24'(v);
    endfunction
    // resolve_divisor of a 64-bit signed d != 0: {divShift[5:0], divFactor[15:0] signed}
    function automatic logic [21:0] resolve_div(input logic signed [63:0] d);
        logic [63:0] a, e;
        logic [5:0] n;
        logic [8:0] f;
        logic signed [15:0] df;
        a = d[63] ? 64'(-d) : 64'(d);
        n = floor_log2_64(a);
        e = a - (64'd1 << n);
        if (n > 6'd8) f = 9'((e + (64'd1 << (n - 6'd9))) >> (n - 6'd8));           // Round2(e, n - DIV_LUT_BITS)
        else f = 9'(e << (6'd8 - n));
        df = 16'(DIV_LUT[f]);
        return {6'(n + 6'd14), d[63] ? -df : df};
    endfunction

    // ---------------------------------------------------------------- state
    typedef enum logic [4:0] {W_IDLE, W_DET, W_DIV, W_N2, W_N3, W_N4, W_N5, W_M2, W_M3, W_M4, W_M5,
                              W_Q2, W_Q3, W_Q4, W_Q5, W_SH0, W_SG, W_SD1, W_SD2, W_FIN} st_t;
    st_t st;
    logic signed [63:0] m_a, m_b, prod;                   // the shared multiplier: operands registered, product combinational
    assign prod = m_a * m_b;
    logic signed [63:0] num2, num3, num4, num5;           // local numerators
    logic signed [63:0] det;
    logic [5:0]  dsh;                                     // divShift
    logic signed [31:0] dfac;                             // divFactor (after the left shift of the local case)
    logic signed [23:0] q [0:5];                          // the parameters being built
    logic signed [15:0] a0, b0, g0, d0;
    logic lvalid;
    assign busy = st != W_IDLE;

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) st <= W_IDLE;
        else case (st)
            W_IDLE: if (start) begin
                lvalid <= 1'b1;
                if (mode) begin
                    for (int i = 0; i < 6; i++) q[i] <= gm_p[i];
                    st <= W_SH0;
                end else begin m_a <= 64'(ls_a00); m_b <= 64'(ls_a11); st <= W_DET; end
            end
            W_DET: begin det <= prod; m_a <= 64'(ls_a01); m_b <= 64'(ls_a01); st <= W_DIV; end
            W_DIV: begin
                logic signed [63:0] d;
                logic [21:0] rd;
                logic [5:0] sh;
                d = det - prod;
                det <= d;
                rd = resolve_div(d);
                sh = rd[21:16];
                if (d == 64'sd0) lvalid <= 1'b0;
                // divShift -= WARPEDMODEL_PREC_BITS; a negative shift becomes a left shift of the factor
                if (sh >= 6'd16) begin dsh <= sh - 6'd16; dfac <= 32'(signed'(rd[15:0])); end
                else begin dsh <= 6'd0; dfac <= 32'(signed'(rd[15:0])) <<< (6'd16 - sh); end
                m_a <= 64'(ls_a11); m_b <= 64'(ls_bx0); st <= W_N2;
            end
            // numerators: each the difference of two products
            W_N2: begin num2 <= prod; m_a <= 64'(ls_a01); m_b <= 64'(ls_bx1); st <= W_N3; end
            W_N3: begin num2 <= num2 - prod; m_a <= 64'(ls_a00); m_b <= 64'(ls_bx1); st <= W_N4; end
            W_N4: begin num3 <= prod; m_a <= 64'(ls_a01); m_b <= 64'(ls_bx0); st <= W_N5; end
            W_N5: begin num3 <= num3 - prod; m_a <= 64'(ls_a11); m_b <= 64'(ls_by0); st <= W_M2; end
            W_M2: begin num4 <= prod; m_a <= 64'(ls_a01); m_b <= 64'(ls_by1); st <= W_M3; end
            W_M3: begin num4 <= num4 - prod; m_a <= 64'(ls_a00); m_b <= 64'(ls_by1); st <= W_M4; end
            W_M4: begin num5 <= prod; m_a <= 64'(ls_a01); m_b <= 64'(ls_by0); st <= W_M5; end
            W_M5: begin num5 <= num5 - prod; m_a <= num2; m_b <= 64'(dfac); st <= W_Q2; end
            // the four parameters: numerator * divFactor, rounded by divShift, clamped (diag / nondiag)
            W_Q2: begin q[2] <= clip24(r2s(prod, dsh), 24'sd57345, 24'sd73727); m_a <= num3; m_b <= 64'(dfac); st <= W_Q3; end
            W_Q3: begin q[3] <= clip24(r2s(prod, dsh), -24'sd8191, 24'sd8191); m_a <= num4; m_b <= 64'(dfac); st <= W_Q4; end
            W_Q4: begin q[4] <= clip24(r2s(prod, dsh), -24'sd8191, 24'sd8191); m_a <= num5; m_b <= 64'(dfac); st <= W_Q5; end
            W_Q5: begin
                logic signed [63:0] p5, vx, vy;
                p5 = 64'(clip24(r2s(prod, dsh), 24'sd57345, 24'sd73727));
                q[5] <= 24'(p5);
                // p0 / p1 from Mv[0] and the block centre
                vx = (64'(mv_col) <<< 13) - (64'(signed'({1'b0, mid_x})) * (64'(q[2]) - 64'sd65536) + 64'(signed'({1'b0, mid_y})) * 64'(q[3]));
                vy = (64'(mv_row) <<< 13) - (64'(signed'({1'b0, mid_x})) * 64'(q[4]) + 64'(signed'({1'b0, mid_y})) * (p5 - 64'sd65536));
                q[0] <= clip24(vx, -24'sd8388608, 24'sd8388607);
                q[1] <= clip24(vy, -24'sd8388608, 24'sd8388607);
                st <= W_SH0;
            end
            // ---- setup_shear of q
            W_SH0: begin
                logic [21:0] rd;
                a0 <= clip16(64'(q[2]) - 64'sd65536);
                b0 <= clip16(64'(q[3]));
                rd = resolve_div(64'(q[2]));
                dsh <= rd[21:16]; dfac <= 32'(signed'(rd[15:0]));
                m_a <= 64'(q[4]) <<< 16; m_b <= 64'(signed'(rd[15:0])); st <= W_SG;
            end
            W_SG: begin g0 <= clip16(r2s(prod, dsh)); m_a <= 64'(q[3]); m_b <= 64'(q[4]); st <= W_SD1; end
            W_SD1: begin m_a <= prod; m_b <= 64'(dfac); st <= W_SD2; end
            W_SD2: begin d0 <= clip16(64'(q[5]) - r2s(prod, dsh) - 64'sd65536); st <= W_FIN; end
            W_FIN: begin
                logic signed [17:0] al, be, ga, de;
                logic [19:0] s1, s2;
                al = 18'(r2s(64'(a0), 6'd6)) <<< 6; be = 18'(r2s(64'(b0), 6'd6)) <<< 6;
                ga = 18'(r2s(64'(g0), 6'd6)) <<< 6; de = 18'(r2s(64'(d0), 6'd6)) <<< 6;
                s1 = 20'(18'(al[17] ? -al : al)) * 20'd4 + 20'(18'(be[17] ? -be : be)) * 20'd7;
                s2 = 20'(18'(ga[17] ? -ga : ga)) * 20'd4 + 20'(18'(de[17] ? -de : de)) * 20'd4;
                alpha <= 16'(al); beta <= 16'(be); gamma <= 16'(ga); delta <= 16'(de);
                valid <= lvalid && (s1 < 20'd65536) && (s2 < 20'd65536);
                for (int i = 0; i < 6; i++) p[i] <= q[i];
                done <= 1'b1; st <= W_IDLE;
            end
            default: st <= W_IDLE;
        endcase
    end
endmodule
