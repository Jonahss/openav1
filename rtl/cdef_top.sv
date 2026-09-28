// CDEF (spec 7.15), frame-level, window version: for every 8x8 luma block (raster order) read the deblocked
// frame (src), write CdefFrame (dst). Blocks whose 64x64 unit has no cdef_idx or whose four 4x4s are all
// skipped are copied unfiltered. Otherwise, per plane, the block plus a 2-sample border is loaded once into
// a 12x12 window register file (one aligned group of 4 samples per clock; samples outside the MI-aligned frame are marked absent:
// is_inside_filter_region), the direction search (7.15.2: 8 x 15 partial sums, the 64 cost terms with
// Div_Table, argmax, variance) is evaluated from the luma window in one cycle, and the constrained
// directional filter (7.15.3) produces one output sample per clock from the window. Chroma uses the chroma
// direction map and damping - 1. ~4 cycles per sample against ~16 for the sample-at-a-time version.
module cdef_top
  import syn_pkg::*;
  import cdef_pkg::*;
#(
    parameter int FBX = 10,
    parameter int FBY = 9
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  cdef_hdr_t   ch,
    input  logic        start,
    output logic        busy,
    output logic        done,
    // mi_store: skips (lf_info) and cdef_idx
    output logic [10:0] rd_row, rd_col,
    input  mi_lf_t      rd_info,
    output logic [6:0]  cdr_row64, cdr_col64,
    input  logic [3:0]  cdr_val,
    // deblocked frame (read) and CdefFrame (write)
    output logic        src_re,
    output logic [1:0]  src_plane,
    output logic [FBX-1:0] src_x,
    output logic [FBY-1:0] src_y,
    input  logic [11:0] src_rdata,
    input  logic [47:0] src_rdata4,      // aligned group of 4 samples containing src_x (window loads, block copies)
    output logic        dst_we,
    output logic [1:0]  dst_plane,
    output logic [FBX-1:0] dst_x,
    output logic [FBY-1:0] dst_y,
    output logic [11:0] dst_wdata,
    output logic        dst4_we,         // 4-wide write (block copies), aligned at dst4_x
    output logic [1:0]  dst4_plane,
    output logic [FBX-1:0] dst4_x,
    output logic [FBY-1:0] dst4_y,
    output logic [47:0] dst4_wdata
);
    typedef enum logic [3:0] {
        C_IDLE, C_BLK, C_RD_IDX, C_RD_S0, C_RD_S1, C_RD_S2, C_RD_S3, C_DECIDE,
        C_PLANE, C_WIN, C_WIN_LAST, C_DIR, C_PIX,
        C_COPY_RD, C_COPY_WR, C_NEXT
    } st_t;
    st_t st;
    logic done_pend;

    // ---------------------------------------------------------------- block loop
    logic [10:0] r, c;                                  // 8x8 block position in 4x4 units (step 2)
    logic [3:0]  cidx;                                  // {valid, idx}
    logic [3:0]  skips;
    logic [10:0] r1, c1;
    logic [3:0]  cs;                                    // coeffShift = BitDepth - 8
    always_comb begin
        r1 = (r + 11'd1 < hdr.mi_rows) ? r + 11'd1 : hdr.mi_rows - 11'd1;
        c1 = (c + 11'd1 < hdr.mi_cols) ? c + 11'd1 : hdr.mi_cols - 11'd1;
        cs = hdr.bit_depth - 4'd8;
        cdr_row64 = 7'(r >> 4); cdr_col64 = 7'(c >> 4);
        case (st)
            C_RD_S0: begin rd_row = r;  rd_col = c;  end
            C_RD_S1: begin rd_row = r1; rd_col = c;  end
            C_RD_S2: begin rd_row = r;  rd_col = c1; end
            default: begin rd_row = r1; rd_col = c1; end
        endcase
    end

    // ---------------------------------------------------------------- plane geometry
    logic [1:0]  fplane;
    logic        sub_x, sub_y;
    logic [3:0]  pw, ph;                                // block dims in the plane (8 or 4)
    logic [12:0] px0, py0, fw, fh;
    always_comb begin
        sub_x = (fplane != 2'd0) && hdr.ssx; sub_y = (fplane != 2'd0) && hdr.ssy;
        pw = 4'd8 >> sub_x; ph = 4'd8 >> sub_y;
        px0 = (13'(c) << 2) >> sub_x; py0 = (13'(r) << 2) >> sub_y;
        fw = (13'(hdr.mi_cols) << 2) >> sub_x; fh = (13'(hdr.mi_rows) << 2) >> sub_y;
    end

    // ---------------------------------------------------------------- window: block + 2-sample border, [row][col] at +2
    logic [11:0] win [0:11][0:11];
    logic        win_ok [0:11][0:11];
    logic [3:0]  wr;                                    // load row 0 .. ph+3
    logic [1:0]  wg, wg_last;                           // aligned group of 4 within the row: x = px0 - 4 + 4 wg
    logic        wpend; logic [3:0] wpr; logic [1:0] wpg; logic wp_rowok; logic signed [14:0] wpx;
    logic signed [14:0] wy, wx;
    logic        w_row_in;
    always_comb begin
        wy = 15'(signed'({2'b0, py0})) + 15'(signed'({11'b0, wr})) - 15'sd2;
        wx = 15'(signed'({2'b0, px0})) + 15'(signed'({13'b0, wg}) << 2) - 15'sd4;
        w_row_in = (wy >= 0) && (wy < 15'(signed'({2'b0, fh})));
        wg_last = (pw == 4'd8) ? 2'd3 : 2'd2;           // 12 window columns span 4 groups, 8 span 3
    end

    // ---------------------------------------------------------------- 7.15.2 direction search from the luma window
    logic signed [15:0] part [0:7][0:14];
    always_comb begin
        for (int d = 0; d < 8; d++) for (int k = 0; k < 15; k++) part[d][k] = 16'sd0;
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++) begin
                logic signed [15:0] s;
                s = 16'(signed'({4'b0, win[i + 2][j + 2] >> cs})) - 16'sd128;
                part[0][i + j] = part[0][i + j] + s;
                part[1][i + (j >> 1)] = part[1][i + (j >> 1)] + s;
                part[2][i] = part[2][i] + s;
                part[3][3 + i - (j >> 1)] = part[3][3 + i - (j >> 1)] + s;
                part[4][7 + i - j] = part[4][7 + i - j] + s;
                part[5][3 - (i >> 1) + j] = part[5][3 - (i >> 1) + j] + s;
                part[6][j] = part[6][j] + s;
                part[7][(i >> 1) + j] = part[7][(i >> 1) + j] + s;
            end
    end
    function automatic logic signed [42:0] sq(input logic signed [15:0] p);
        sq = 43'(p) * 43'(p);
    endfunction
    logic signed [42:0] cost [0:7];
    always_comb begin
        for (int d = 0; d < 8; d++) cost[d] = 43'sd0;
        for (int i = 0; i < 8; i++) begin
            cost[2] = cost[2] + sq(part[2][i]) * 43'sd105;
            cost[6] = cost[6] + sq(part[6][i]) * 43'sd105;
        end
        for (int i = 0; i < 7; i++) begin
            cost[0] = cost[0] + (sq(part[0][i]) + sq(part[0][14 - i])) * 43'(signed'({33'b0, div_table(4'(i + 1))}));
            cost[4] = cost[4] + (sq(part[4][i]) + sq(part[4][14 - i])) * 43'(signed'({33'b0, div_table(4'(i + 1))}));
        end
        cost[0] = cost[0] + sq(part[0][7]) * 43'sd105;
        cost[4] = cost[4] + sq(part[4][7]) * 43'sd105;
        for (int o = 0; o < 4; o++) begin
            int d;
            d = 2 * o + 1;
            for (int j = 0; j < 5; j++) cost[d] = cost[d] + sq(part[d][3 + j]) * 43'sd105;
            for (int j = 0; j < 3; j++) cost[d] = cost[d] + (sq(part[d][j]) + sq(part[d][10 - j])) * 43'(signed'({33'b0, div_table(4'(2 * j + 2))}));
        end
    end
    logic [2:0]  best_dir_c;
    logic signed [42:0] best_cost_c;
    logic [31:0] var_c;
    always_comb begin
        best_cost_c = 43'sd0; best_dir_c = 3'd0;
        for (int d = 0; d < 8; d++) if (cost[d] > best_cost_c) begin best_cost_c = cost[d]; best_dir_c = 3'(d); end
        var_c = 32'((best_cost_c - cost[(best_dir_c + 3'd4) & 3'd7]) >>> 10);
    end
    logic [2:0]  ydir;
    logic [31:0] var_;

    // ---------------------------------------------------------------- filter (7.15.3): one sample per clock from the window
    logic [11:0] pri_str, sec_str;
    logic [4:0]  damping;
    logic [2:0]  dr;
    logic [2:0]  pi, pj;                                // sample in block
    function automatic logic [3:0] floor_log2_12(input logic [11:0] v);
        floor_log2_12 = 4'd0;
        for (int i = 11; i >= 0; i--) if (floor_log2_12 == 4'd0 && v[i]) floor_log2_12 = 4'(i);
    endfunction
    function automatic logic signed [12:0] constrain(input logic signed [12:0] diff, input logic [11:0] thr, input logic [4:0] damp);
        logic [12:0] ad;
        logic [4:0]  adj;
        logic signed [13:0] v;
        if (thr == 12'd0) constrain = 13'sd0;
        else begin
            ad = (diff < 0) ? 13'(-diff) : 13'(diff);
            adj = (damp > 5'(floor_log2_12(thr))) ? damp - 5'(floor_log2_12(thr)) : 5'd0;
            v = 14'(signed'({2'b0, thr})) - 14'(signed'({1'b0, ad >> adj}));
            if (v < 0) v = 14'sd0;
            if (v > 14'(signed'({1'b0, ad}))) v = 14'(signed'({1'b0, ad}));
            constrain = (diff < 0) ? -13'(v) : 13'(v);
        end
    endfunction
    logic pri_odd;
    assign pri_odd = pri_str[cs];                       // (priStr >> coeffShift) & 1
    logic [11:0] out_pix;
    always_comb begin
        logic signed [17:0] total;
        logic [11:0] mn, mx, ctr;
        ctr = win[int'(pi) + 2][int'(pj) + 2];
        total = 18'sd0; mn = ctr; mx = ctr;
        // taps: kind 0 primary direction, 1 dir - 2, 2 dir + 2; k 0..1; sign -/+
        for (int kind = 0; kind < 3; kind++)
            for (int k = 0; k < 2; k++)
                for (int sg = 0; sg < 2; sg++) begin
                    logic [2:0] tdir; logic signed [3:0] dy, dx; logic [3:0] ry, rx; logic [11:0] t;
                    tdir = (kind == 0) ? dr : (kind == 1) ? dr - 3'd2 : dr + 3'd2;
                    dy = (sg == 1) ? 4'(cdef_dy(tdir, 1'(k))) : -4'(cdef_dy(tdir, 1'(k)));
                    dx = (sg == 1) ? 4'(cdef_dx(tdir, 1'(k))) : -4'(cdef_dx(tdir, 1'(k)));
                    ry = 4'(int'(pi) + 2 + int'(dy)); rx = 4'(int'(pj) + 2 + int'(dx));
                    t = win[ry][rx];
                    if (win_ok[ry][rx]) begin
                        total = total + ((kind == 0) ? 18'(pri_tap(pri_odd, 1'(k))) * 18'(constrain(13'(signed'({1'b0, t})) - 13'(signed'({1'b0, ctr})), pri_str, damping))
                                                     : 18'(sec_tap(pri_odd, 1'(k))) * 18'(constrain(13'(signed'({1'b0, t})) - 13'(signed'({1'b0, ctr})), sec_str, damping)));
                        if (t > mx) mx = t;
                        if (t < mn) mn = t;
                    end
                end
        begin
            logic signed [18:0] v;
            v = 19'(signed'({7'b0, ctr})) + ((19'sd8 + 19'(total) - (total < 0 ? 19'sd1 : 19'sd0)) >>> 4);
            if (v < 19'(signed'({7'b0, mn}))) out_pix = mn;
            else if (v > 19'(signed'({7'b0, mx}))) out_pix = mx;
            else out_pix = 12'(v);
        end
    end
    // copy path
    logic [5:0] cpi;

    // ---------------------------------------------------------------- memory ports
    always_comb begin
        src_re = 1'b0; src_plane = fplane; src_x = FBX'(wx); src_y = FBY'(wy);
        dst_we = 1'b0; dst_plane = fplane; dst_x = FBX'(px0 + 13'(pj)); dst_y = FBY'(py0 + 13'(pi)); dst_wdata = out_pix;
        dst4_we = 1'b0; dst4_plane = fplane; dst4_x = FBX'(px0 + 13'(cpi[2:0])); dst4_y = FBY'(py0 + 13'(cpi[5:3])); dst4_wdata = src_rdata4;
        case (st)
            C_WIN: src_re = w_row_in;                    // absent columns of a present row are masked at landing
            C_PIX: dst_we = 1'b1;
            C_COPY_RD: begin src_re = 1'b1; src_x = FBX'(px0 + 13'(cpi[2:0])); src_y = FBY'(py0 + 13'(cpi[5:3])); end
            C_COPY_WR: dst4_we = 1'b1;
            default: ;
        endcase
    end
    assign busy = (st != C_IDLE);

    // ---------------------------------------------------------------- FSM
    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (wpend)                                       // the group issued last cycle lands: lane l -> column 4 wpg - 2 + l
            for (int l = 0; l < 4; l++) begin
                int col;
                col = 4 * int'(wpg) - 2 + l;
                if (col >= 0 && col < int'(pw) + 4) begin
                    win[wpr][col] <= src_rdata4[l * 12 +: 12];
                    win_ok[wpr][col] <= wp_rowok && (wpx + 15'(l) >= 0) && (wpx + 15'(l) < 15'(signed'({2'b0, fw})));
                end
            end
        wpend <= 1'b0;
        if (rst) st <= C_IDLE;
        else case (st)
            C_IDLE: if (start) begin r <= 11'd0; c <= 11'd0; st <= C_BLK; end
            C_BLK: st <= C_RD_IDX;                       // cdr_* addressed; value lands next cycle
            C_RD_IDX: begin cidx <= cdr_val; st <= C_RD_S0; end
            C_RD_S0: st <= C_RD_S1;
            C_RD_S1: begin skips[0] <= rd_info.skip; st <= C_RD_S2; end
            C_RD_S2: begin skips[1] <= rd_info.skip; st <= C_RD_S3; end
            C_RD_S3: begin skips[2] <= rd_info.skip; st <= C_DECIDE; end
            C_DECIDE: begin
                skips[3] <= rd_info.skip;
                fplane <= 2'd0;
                if (!cidx[3] || (skips[0] && skips[1] && skips[2] && rd_info.skip)) begin cpi <= 6'd0; st <= C_COPY_RD; end
                else st <= C_PLANE;
            end
            // ---- per plane: load the window (one read per clock, lands next cycle)
            C_PLANE: begin wr <= 4'd0; wg <= 2'd0; st <= C_WIN; end
            C_WIN: begin
                wpend <= 1'b1; wpr <= wr; wpg <= wg; wp_rowok <= w_row_in; wpx <= wx;
                if (wg != wg_last) wg <= wg + 2'd1;
                else begin
                    wg <= 2'd0;
                    if (wr + 4'd1 < ph + 4'd4) wr <= wr + 4'd1;
                    else st <= C_WIN_LAST;
                end
            end
            C_WIN_LAST: st <= C_DIR;                     // the last read lands here
            // ---- direction (luma) and strengths, one cycle
            C_DIR: begin
                logic [5:0] sv; logic [3:0] pri, sec; logic [11:0] ps; logic [3:0] var_str; logic [31:0] v6;
                logic [2:0] yd; logic [31:0] vr;
                yd = (fplane == 2'd0) ? best_dir_c : ydir;
                vr = (fplane == 2'd0) ? var_c : var_;
                if (fplane == 2'd0) begin ydir <= best_dir_c; var_ <= var_c; end
                sv = (fplane == 2'd0) ? ch.y_str[6 * cidx[2:0] +: 6] : ch.uv_str[6 * cidx[2:0] +: 6];
                pri = 4'(sv >> 2); sec = 4'(sv[1:0]) + ((sv[1:0] == 2'd3) ? 4'd1 : 4'd0);
                ps = 12'(pri) << cs;
                if (fplane == 2'd0) begin
                    v6 = vr >> 6;
                    var_str = 4'd0;
                    if (v6 != 32'd0) begin
                        for (int b = 31; b >= 0; b--) if (var_str == 4'd0 && v6[b]) var_str = (b > 12) ? 4'd12 : 4'(b);
                    end
                    dr <= (ps == 12'd0) ? 3'd0 : yd;
                    pri_str <= (vr != 32'd0) ? 12'((16'(ps) * (16'd4 + 16'(var_str)) + 16'd8) >> 4) : 12'd0;
                    damping <= 5'(ch.damping) + 5'(cs);
                end else begin
                    dr <= (ps == 12'd0) ? 3'd0 : cdef_uv_dir(hdr.ssx, hdr.ssy, yd);
                    pri_str <= ps;
                    damping <= 5'(ch.damping) + 5'(cs) - 5'd1;
                end
                sec_str <= 12'(sec) << cs;
                pi <= 3'd0; pj <= 3'd0;
                st <= C_PIX;
            end
            // ---- one filtered sample per clock
            C_PIX: begin
                if (4'(pj) + 4'd1 < pw) pj <= pj + 3'd1;
                else begin
                    pj <= 3'd0;
                    if (4'(pi) + 4'd1 < ph) pi <= pi + 3'd1;
                    else begin
                        if (fplane == 2'd0 && !hdr.mono) begin fplane <= 2'd1; st <= C_PLANE; end
                        else if (fplane == 2'd1) begin fplane <= 2'd2; st <= C_PLANE; end
                        else st <= C_NEXT;
                    end
                end
            end
            // ---- unfiltered block: copy src -> dst for all planes
            C_COPY_RD: st <= C_COPY_WR;
            C_COPY_WR: begin
                if (4'(cpi[2:0]) + 4'd4 < pw) begin cpi <= cpi + 6'd4; st <= C_COPY_RD; end
                else if (4'(cpi[5:3]) + 4'd1 < ph) begin cpi <= {cpi[5:3] + 3'd1, 3'd0}; st <= C_COPY_RD; end
                else begin
                    cpi <= 6'd0;
                    if (fplane == 2'd0 && !hdr.mono) begin fplane <= 2'd1; st <= C_COPY_RD; end
                    else if (fplane == 2'd1) begin fplane <= 2'd2; st <= C_COPY_RD; end
                    else st <= C_NEXT;
                end
            end
            C_NEXT: begin
                if (c + 11'd2 < hdr.mi_cols) begin c <= c + 11'd2; st <= C_BLK; end
                else begin
                    c <= 11'd0;
                    if (r + 11'd2 < hdr.mi_rows) begin r <= r + 11'd2; st <= C_BLK; end
                    else begin done <= 1'b1; st <= C_IDLE; end
                end
            end
            default: st <= C_IDLE;
        endcase
    end
endmodule
