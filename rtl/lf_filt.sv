// Deblocking sample filter (spec 7.14.6) for one line of samples across an edge, combinational: the masks, the narrow
// filter and the 6 / 8 / 14-tap wide filters, as in lf_top (which filters one such line per clock out of the frame
// memory). The streaming deblock stage (docs/filter-chain.md) instantiates four of these side by side per pass, one per
// sample line of a 4x4 edge unit.
//   pix    : p6 .. p0, q0 .. q6 (pix[0] = p6, pix[6] = p0, pix[7] = q0, pix[13] = q6), 12 bits each
//   out    : the filtered samples at positions k = -6 .. 5 (out[0] = position -6 = p5, out[6] = q0), valid where mod
//   mod    : position k is changed by the filter (a position not flagged keeps its sample)
module lf_filt (
    input  logic [3:0]      bit_depth,
    input  logic            chroma,        // plane != 0: the wide filter is the 6-tap one
    input  logic [4:0]      filter_size,   // 4, 8 or 16 (7.14.3)
    input  logic [5:0]      limit,         // 7.14.4
    input  logic [6:0]      blimit,
    input  logic [3:0]      thresh,
    input  logic [14*12-1:0] pix,
    output logic [12*12-1:0] out,
    output logic [11:0]     mod
);
    logic [3:0]  bd8;
    assign bd8 = bit_depth - 4'd8;
    logic [12:0] p [0:6];
    logic [12:0] q [0:6];
    always_comb for (int j = 0; j < 7; j++) begin p[j] = 13'(pix[(6 - j) * 12 +: 12]); q[j] = 13'(pix[(7 + j) * 12 +: 12]); end
    function automatic logic [12:0] adiff(input logic [12:0] a, input logic [12:0] b);
        adiff = (a > b) ? a - b : b - a;
    endfunction
    logic [4:0]  filter_len;
    logic        hev, filter_mask, flat, flat2;
    logic [15:0] thresh_bd, limit_bd, blimit_bd, one_bd;
    always_comb begin
        filter_len = (filter_size == 5'd4) ? 5'd4 : chroma ? 5'd6 : (filter_size == 5'd8) ? 5'd8 : 5'd16;
        thresh_bd = 16'(thresh) << bd8; limit_bd = 16'(limit) << bd8; blimit_bd = 16'(blimit) << bd8; one_bd = 16'd1 << bd8;
        hev = (16'(adiff(p[1], p[0])) > thresh_bd) || (16'(adiff(q[1], q[0])) > thresh_bd);
        begin
            logic m;
            m = (16'(adiff(p[1], p[0])) > limit_bd) || (16'(adiff(q[1], q[0])) > limit_bd) ||
                ((16'(adiff(p[0], q[0])) * 2 + (16'(adiff(p[1], q[1])) >> 1)) > blimit_bd);
            if (filter_len >= 5'd6) m = m || (16'(adiff(p[2], p[1])) > limit_bd) || (16'(adiff(q[2], q[1])) > limit_bd);
            if (filter_len >= 5'd8) m = m || (16'(adiff(p[3], p[2])) > limit_bd) || (16'(adiff(q[3], q[2])) > limit_bd);
            filter_mask = !m;
        end
        begin
            logic m;
            m = (16'(adiff(p[1], p[0])) > one_bd) || (16'(adiff(q[1], q[0])) > one_bd) ||
                (16'(adiff(p[2], p[0])) > one_bd) || (16'(adiff(q[2], q[0])) > one_bd);
            if (filter_len >= 5'd8) m = m || (16'(adiff(p[3], p[0])) > one_bd) || (16'(adiff(q[3], q[0])) > one_bd);
            flat = (filter_size >= 5'd8) && !m;
        end
        begin
            logic m;
            m = (16'(adiff(p[6], p[0])) > one_bd) || (16'(adiff(q[6], q[0])) > one_bd) ||
                (16'(adiff(p[5], p[0])) > one_bd) || (16'(adiff(q[5], q[0])) > one_bd) ||
                (16'(adiff(p[4], p[0])) > one_bd) || (16'(adiff(q[4], q[0])) > one_bd);
            flat2 = (filter_size >= 5'd16) && !m;
        end
    end
    // narrow filter (7.14.6.3)
    logic signed [13:0] ps1, ps0, qs0, qs1, filt, filter1, filter2, filt_r;
    logic [11:0] nq0, np0, nq1, np1;
    logic signed [13:0] off_s, lo_s, hi_s;
    function automatic logic signed [13:0] c4(input logic signed [15:0] v, input logic signed [13:0] lo, input logic signed [13:0] hi);
        c4 = (v < 16'(lo)) ? lo : (v > 16'(hi)) ? hi : 14'(v);
    endfunction
    always_comb begin
        off_s = 14'(13'd128 << bd8);
        lo_s = -(14'sd1 <<< (bit_depth - 4'd1));
        hi_s = (14'sd1 <<< (bit_depth - 4'd1)) - 14'sd1;
        ps1 = 14'(p[1]) - off_s; ps0 = 14'(p[0]) - off_s; qs0 = 14'(q[0]) - off_s; qs1 = 14'(q[1]) - off_s;
        filt = hev ? c4(16'(ps1) - 16'(qs1), lo_s, hi_s) : 14'sd0;
        filt = c4(16'(filt) + 16'sd3 * (16'(qs0) - 16'(ps0)), lo_s, hi_s);
        filter1 = c4(16'(filt) + 16'sd4, lo_s, hi_s) >>> 3;
        filter2 = c4(16'(filt) + 16'sd3, lo_s, hi_s) >>> 3;
        nq0 = 12'(c4(16'(qs0) - 16'(filter1), lo_s, hi_s) + off_s);
        np0 = 12'(c4(16'(ps0) + 16'(filter2), lo_s, hi_s) + off_s);
        filt_r = (filter1 + 14'sd1) >>> 1;                                   // Round2(filter1, 1)
        nq1 = 12'(c4(16'(qs1) - 16'(filt_r), lo_s, hi_s) + off_s);
        np1 = 12'(c4(16'(ps1) + 16'(filt_r), lo_s, hi_s) + off_s);
    end
    // wide filters (7.14.6.4): F[i] = Round2(sum_j pix[clip3(-(n+1), n, i+j)] * tap(j), log2Size), i in -n..n-1
    function automatic logic [11:0] wide_val(input int i, input int n, input int n2, input int log2sz);
        logic [17:0] t;
        int pidx;
        t = 18'd0;
        for (int j = -6; j <= 6; j++) begin
            if (j >= -n && j <= n) begin
                pidx = i + j;
                if (pidx < -(n + 1)) pidx = -(n + 1);
                if (pidx > n) pidx = n;
                t = t + (((j < 0 ? -j : j) <= n2) ? 18'(pix[(7 + pidx) * 12 +: 12]) * 18'd2 : 18'(pix[(7 + pidx) * 12 +: 12]));
            end
        end
        wide_val = 12'((t + (18'd1 << (log2sz - 1))) >> log2sz);
    endfunction
    logic [11:0] w16 [0:11];                            // luma 16: n=6, n2=1, log2 4: i = -6..5
    logic [11:0] w8  [0:5];                             // luma 8 : n=3, n2=0, log2 3: i = -3..2
    logic [11:0] w6  [0:3];                             // chroma : n=2, n2=1, log2 3: i = -2..1
    always_comb begin
        for (int i = -6; i <= 5; i++) w16[i + 6] = wide_val(i, 6, 1, 4);
        for (int i = -3; i <= 2; i++) w8[i + 3] = wide_val(i, 3, 0, 3);
        for (int i = -2; i <= 1; i++) w6[i + 2] = wide_val(i, 2, 1, 3);
    end
    typedef enum logic [1:0] {F_NONE, F_NARROW, F_WIDE8, F_WIDE16} fmode_t;
    fmode_t fmode;
    always_comb begin
        if (!filter_mask) fmode = F_NONE;
        else if (filter_size == 5'd4 || !flat) fmode = F_NARROW;
        else if (filter_size == 5'd8 || !flat2) fmode = F_WIDE8;
        else fmode = F_WIDE16;
    end
    always_comb begin
        for (int k = -6; k <= 5; k++) begin
            logic [11:0] o; logic v;
            o = pix[(7 + k) * 12 +: 12]; v = 1'b0;
            case (fmode)
                F_NARROW: begin
                    if (k == 0)  begin o = nq0; v = 1'b1; end
                    if (k == -1) begin o = np0; v = 1'b1; end
                    if (k == 1)  begin o = nq1; v = !hev; end
                    if (k == -2) begin o = np1; v = !hev; end
                end
                F_WIDE8: begin
                    if (!chroma && k >= -3 && k <= 2) begin o = w8[k + 3]; v = 1'b1; end
                    if (chroma && k >= -2 && k <= 1)  begin o = w6[k + 2]; v = 1'b1; end
                end
                F_WIDE16: begin o = w16[k + 6]; v = 1'b1; end
                default: ;
            endcase
            out[(k + 6) * 12 +: 12] = v ? o : pix[(7 + k) * 12 +: 12];
            mod[k + 6] = v;
        end
    end
endmodule
