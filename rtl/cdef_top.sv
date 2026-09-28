// CDEF (spec 7.15), frame-level: for every 8x8 luma block (in raster order) read the deblocked frame (src),
// write CdefFrame (dst). Blocks whose 64x64 unit has no cdef_idx or whose four 4x4s are all skipped are copied
// unfiltered; otherwise: direction search on the 8x8 luma block (7.15.2: 8 x 15 partial sums, 64 multiply-
// accumulates of squares with Div_Table, argmax, variance), variance-adaptive primary strength, then the
// constrained directional filter (7.15.3) on luma and, unless monochrome, both chroma blocks with the
// chroma direction map and damping - 1. Neighbour taps outside the MI-aligned frame are ignored
// (is_inside_filter_region). Slow-and-correct: ~16 cycles per pixel, one MAC per cycle in the search.
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
    output logic        dst_we,
    output logic [1:0]  dst_plane,
    output logic [FBX-1:0] dst_x,
    output logic [FBY-1:0] dst_y,
    output logic [11:0] dst_wdata
);
    typedef enum logic [4:0] {
        C_IDLE, C_BLK, C_RD_IDX, C_RD_S0, C_RD_S1, C_RD_S2, C_RD_S3, C_DECIDE,
        C_DIR_RD, C_COST, C_BEST, C_STR,
        C_PIX_RD, C_PIX_CALC, C_PIX_WR, C_PLANE_NEXT,
        C_COPY_RD, C_COPY_WR, C_NEXT, C_DONE
    } st_t;
    st_t st;

    // ---------------------------------------------------------------- block loop
    logic [10:0] r, c;                                  // 8x8 block position in 4x4 units (step 2)
    logic [3:0]  cidx;                                  // {valid, idx}
    logic [3:0]  skips;
    logic [10:0] r1, c1;                                // clamped r+1, c+1
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

    // ---------------------------------------------------------------- direction search state
    logic [6:0]  di;                                    // read counter 0..64 (pixel i = di[5:3], j = di[2:0])
    logic        rd_pend;
    logic [5:0]  rd_pos;
    logic signed [15:0] part [0:7][0:14];
    logic signed [42:0] cost [0:7];
    logic [6:0]  ti;                                    // cost term index 0..89
    logic [2:0]  ydir, best_dir;
    logic signed [42:0] best_cost;
    logic [31:0] var_;
    // term table: k -> (dir, partial index a, partial index b (or 15 = none), div index)
    function automatic logic [14:0] term(input logic [6:0] k);
        // returns {dir[2:0], a[3:0], b[3:0] (15 = single), div[3:0]}
        logic [2:0] d; logic [3:0] a, b, dv;
        int kk;
        d = 3'd0; a = 4'd0; b = 4'd15; dv = 4'd8;
        kk = int'(k);
        if (kk < 8) begin d = 3'd2; a = 4'(kk); end                                   // cost[2]: p[i]^2 * 105
        else if (kk < 16) begin d = 3'd6; a = 4'(kk - 8); end
        else if (kk < 23) begin d = 3'd0; a = 4'(kk - 16); b = 4'(14 - (kk - 16)); dv = 4'(kk - 16 + 1); end   // (p[i]^2 + p[14-i]^2) * DIV[i+1]
        else if (kk == 23) begin d = 3'd0; a = 4'd7; end                                  // p[7]^2 * DIV[8]
        else if (kk < 31) begin d = 3'd4; a = 4'(kk - 24); b = 4'(14 - (kk - 24)); dv = 4'(kk - 24 + 1); end
        else if (kk == 31) begin d = 3'd4; a = 4'd7; end
        else begin
            // odd dirs 1,3,5,7: 5 terms p[3+j]^2 * DIV[8] (j 0..4), then 3 terms (p[j]^2 + p[10-j]^2) * DIV[2j+2]
            int o, t;
            o = (kk - 32) / 8; t = (kk - 32) % 8;
            d = 3'(2 * o + 1);
            if (t < 5) begin a = 4'(3 + t); end
            else begin a = 4'(t - 5); b = 4'(10 - (t - 5)); dv = 4'(2 * (t - 5) + 2); end
        end
        term = {d, a, b, dv};
    endfunction
    logic [14:0] tm;
    logic signed [15:0] pa, pb;
    logic signed [42:0] term_val;
    always_comb begin
        tm = term(ti);
        pa = part[tm[14:12]][tm[11:8]];
        pb = (tm[7:4] == 4'd15) ? 16'sd0 : part[tm[14:12]][tm[7:4]];
        term_val = (43'(pa) * 43'(pa) + 43'(pb) * 43'(pb)) * 43'(signed'({33'b0, div_table(tm[3:0])}));
    end
    // the direction-search sample: x = (pixel >> cs) - 128
    logic signed [15:0] dsamp;
    assign dsamp = 16'(signed'({4'b0, src_rdata >> cs})) - 16'sd128;

    // ---------------------------------------------------------------- filter state
    logic [1:0]  fplane;
    logic        sub_x, sub_y;
    logic [3:0]  pw, ph;                                // block dims in the plane (8 or 4)
    logic [12:0] px0, py0, fw, fh;                      // block origin, frame dims in the plane
    logic [11:0] pri_str, sec_str;
    logic [4:0]  damping;
    logic [2:0]  dr;
    logic [5:0]  pi_;                                   // pixel in block: i = pi_[5:3], j = pi_[2:0]
    logic [3:0]  tk;                                    // tap counter 0..12 (0 = centre)
    logic [11:0] tap [0:12];
    logic        tap_ok [0:12];
    logic        trd_pend; logic [3:0] trd_slot; logic trd_ok;
    always_comb begin
        sub_x = (fplane != 2'd0) && hdr.ssx; sub_y = (fplane != 2'd0) && hdr.ssy;
        pw = 4'd8 >> sub_x; ph = 4'd8 >> sub_y;
        px0 = (13'(c) << 2) >> sub_x; py0 = (13'(r) << 2) >> sub_y;
        fw = (13'(hdr.mi_cols) << 2) >> sub_x; fh = (13'(hdr.mi_rows) << 2) >> sub_y;
    end
    // tap t (1..12) -> position offset: t-1 = {sec[1:0]? , ...}: order k in 0..1, sign in {-,+}, kind in {pri, sec-2, sec+2}
    //   t = 1 + kind*4 + k*2 + s   (kind 0 primary dir, 1 dir-2, 2 dir+2; s 0 = -1, 1 = +1)
    logic signed [3:0] tdy, tdx;
    logic [2:0] tdir;
    logic tkk, tsg;
    always_comb begin
        begin
            logic [3:0] t1;
            t1 = tk - 4'd1;
            tkk = t1[1]; tsg = t1[0];
            case (t1[3:2])
                2'd0: tdir = dr;
                2'd1: tdir = dr - 3'd2;
                default: tdir = dr + 3'd2;
            endcase
        end
        tdy = tsg ? 4'(cdef_dy(tdir, tkk)) : -4'(cdef_dy(tdir, tkk));
        tdx = tsg ? 4'(cdef_dx(tdir, tkk)) : -4'(cdef_dx(tdir, tkk));
    end
    logic signed [14:0] tyy, txx;
    logic        tinside;
    always_comb begin
        tyy = 15'(signed'({2'b0, py0})) + 15'(signed'({2'b0, 13'(pi_[5:3])})) + ((tk == 4'd0) ? 15'sd0 : 15'(tdy));
        txx = 15'(signed'({2'b0, px0})) + 15'(signed'({2'b0, 13'(pi_[2:0])})) + ((tk == 4'd0) ? 15'sd0 : 15'(tdx));
        tinside = (tyy >= 0) && (tyy < 15'(signed'({2'b0, fh}))) && (txx >= 0) && (txx < 15'(signed'({2'b0, fw})));
    end
    // constrain + filter output (combinational from tap[])
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
    logic signed [17:0] total;
    logic [11:0] mn, mx, out_pix;
    always_comb begin
        total = 18'sd0; mn = tap[0]; mx = tap[0];
        for (int t = 1; t <= 12; t++) begin
            logic [3:0] t1; logic kkk; logic is_pri;
            t1 = 4'(t - 1); kkk = t1[1]; is_pri = (t1[3:2] == 2'd0);
            if (tap_ok[t]) begin
                total = total + (is_pri ? 18'(pri_tap(pri_odd, kkk)) * 18'(constrain(13'(signed'({1'b0, tap[t]})) - 13'(signed'({1'b0, tap[0]})), pri_str, damping))
                                        : 18'(sec_tap(pri_odd, kkk)) * 18'(constrain(13'(signed'({1'b0, tap[t]})) - 13'(signed'({1'b0, tap[0]})), sec_str, damping)));
                if (tap[t] > mx) mx = tap[t];
                if (tap[t] < mn) mn = tap[t];
            end
        end
        begin
            logic signed [18:0] v;
            v = 19'(signed'({7'b0, tap[0]})) + ((19'sd8 + 19'(total) - (total < 0 ? 19'sd1 : 19'sd0)) >>> 4);
            if (v < 19'(signed'({7'b0, mn}))) out_pix = mn;
            else if (v > 19'(signed'({7'b0, mx}))) out_pix = mx;
            else out_pix = 12'(v);
        end
    end
    // copy path
    logic [5:0] cpi;
    logic cp_pend;

    // ---------------------------------------------------------------- memory ports
    always_comb begin
        src_re = 1'b0; src_plane = fplane; src_x = '0; src_y = '0;
        dst_we = 1'b0; dst_plane = fplane; dst_x = '0; dst_y = '0; dst_wdata = out_pix;
        case (st)
            C_DIR_RD: if (di < 7'd64) begin
                src_re = 1'b1; src_plane = 2'd0;
                src_x = FBX'((13'(c) << 2) + 13'(di[2:0])); src_y = FBY'((13'(r) << 2) + 13'(di[5:3]));
            end
            C_PIX_RD: if (tk <= 4'd12) begin
                src_re = (tk == 4'd0) || tinside; src_x = FBX'(txx); src_y = FBY'(tyy);
            end
            C_PIX_WR: begin dst_we = 1'b1; dst_x = FBX'(px0 + 13'(pi_[2:0])); dst_y = FBY'(py0 + 13'(pi_[5:3])); end
            C_COPY_RD: begin src_re = 1'b1; src_x = FBX'(px0 + 13'(cpi[2:0])); src_y = FBY'(py0 + 13'(cpi[5:3])); end
            C_COPY_WR: begin dst_we = 1'b1; dst_x = FBX'(px0 + 13'(cpi[2:0])); dst_y = FBY'(py0 + 13'(cpi[5:3])); dst_wdata = src_rdata; end
            default: ;
        endcase
    end
    assign busy = (st != C_IDLE);

    // ---------------------------------------------------------------- FSM
    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) st <= C_IDLE;
        else case (st)
            C_IDLE: if (start) begin r <= 11'd0; c <= 11'd0; st <= C_BLK; end
            C_BLK: st <= C_RD_IDX;                       // cdr_* addressed; value lands next cycle
            C_RD_IDX: begin cidx <= cdr_val; st <= C_RD_S0; end     // (also rd_* = (r, c) issued)
            C_RD_S0: st <= C_RD_S1;
            C_RD_S1: begin skips[0] <= rd_info.skip; st <= C_RD_S2; end
            C_RD_S2: begin skips[1] <= rd_info.skip; st <= C_RD_S3; end
            C_RD_S3: begin skips[2] <= rd_info.skip; st <= C_DECIDE; end
            C_DECIDE: begin
                skips[3] <= rd_info.skip;
                fplane <= 2'd0;
                if (!cidx[3] || (skips[0] && skips[1] && skips[2] && rd_info.skip)) begin cpi <= 6'd0; cp_pend <= 1'b0; st <= C_COPY_RD; end
                else begin
                    di <= 7'd0; rd_pend <= 1'b0;
                    for (int d = 0; d < 8; d++) begin cost[d] <= 43'sd0; for (int k = 0; k < 15; k++) part[d][k] <= 16'sd0; end
                    st <= C_DIR_RD;
                end
            end
            // ---- 7.15.2 direction: read the 64 luma samples (one per cycle), accumulating the 8 partial-sum arrays
            C_DIR_RD: begin
                if (rd_pend) begin
                    logic [2:0] i, j;
                    i = rd_pos[5:3]; j = rd_pos[2:0];
                    part[0][4'(i) + 4'(j)] <= part[0][4'(i) + 4'(j)] + dsamp;
                    part[1][4'(i) + 4'(j >> 1)] <= part[1][4'(i) + 4'(j >> 1)] + dsamp;
                    part[2][4'(i)] <= part[2][4'(i)] + dsamp;
                    part[3][4'd3 + 4'(i) - 4'(j >> 1)] <= part[3][4'd3 + 4'(i) - 4'(j >> 1)] + dsamp;
                    part[4][4'd7 + 4'(i) - 4'(j)] <= part[4][4'd7 + 4'(i) - 4'(j)] + dsamp;
                    part[5][4'd3 - 4'(i >> 1) + 4'(j)] <= part[5][4'd3 - 4'(i >> 1) + 4'(j)] + dsamp;
                    part[6][4'(j)] <= part[6][4'(j)] + dsamp;
                    part[7][4'(i >> 1) + 4'(j)] <= part[7][4'(i >> 1) + 4'(j)] + dsamp;
                end
                rd_pend <= 1'b0;
                if (di < 7'd64) begin rd_pend <= 1'b1; rd_pos <= di[5:0]; di <= di + 7'd1; end
                else if (!rd_pend) begin ti <= 7'd0; st <= C_COST; end
            end
            C_COST: begin                                // one multiply-accumulate term per cycle (64 terms)
                cost[tm[14:12]] <= cost[tm[14:12]] + term_val;
                if (ti == 7'd63) begin ti <= 7'd0; best_cost <= 43'sd0; best_dir <= 3'd0; st <= C_BEST; end
                else ti <= ti + 7'd1;
            end
            C_BEST: begin                                // argmax (first strict maximum), then var
                if (cost[ti[2:0]] > best_cost) begin best_cost <= cost[ti[2:0]]; best_dir <= ti[2:0]; end
                if (ti == 7'd7) st <= C_STR;
                else ti <= ti + 7'd1;
            end
            C_STR: begin
                ydir <= best_dir;
                var_ <= 32'((best_cost - cost[(best_dir + 3'd4) & 3'd7]) >>> 10);
                fplane <= 2'd0; st <= C_PLANE_NEXT;
            end
            // ---- per plane: strengths, then the filter over the block
            C_PLANE_NEXT: begin
                begin
                    logic [5:0] sv; logic [3:0] pri, sec; logic [11:0] ps; logic [3:0] var_str; logic [31:0] v6;
                    sv = (fplane == 2'd0) ? ch.y_str[6 * cidx[2:0] +: 6] : ch.uv_str[6 * cidx[2:0] +: 6];
                    pri = 4'(sv >> 2); sec = 4'(sv[1:0]) + ((sv[1:0] == 2'd3) ? 4'd1 : 4'd0);
                    ps = 12'(pri) << cs;
                    if (fplane == 2'd0) begin
                        v6 = var_ >> 6;
                        var_str = 4'd0;
                        if (v6 != 32'd0) begin
                            for (int b = 31; b >= 0; b--) if (var_str == 4'd0 && v6[b]) var_str = (b > 12) ? 4'd12 : 4'(b);
                        end
                        dr <= (ps == 12'd0) ? 3'd0 : ydir;
                        pri_str <= (var_ != 32'd0) ? 12'((16'(ps) * (16'd4 + 16'(var_str)) + 16'd8) >> 4) : 12'd0;
                        damping <= 5'(ch.damping) + 5'(cs);
                    end else begin
                        dr <= (ps == 12'd0) ? 3'd0 : cdef_uv_dir(hdr.ssx, hdr.ssy, ydir);
                        pri_str <= ps;
                        damping <= 5'(ch.damping) + 5'(cs) - 5'd1;
                    end
                    sec_str <= 12'(sec) << cs;
                end
                pi_ <= 6'd0; tk <= 4'd0; trd_pend <= 1'b0;
                for (int t = 0; t <= 12; t++) tap_ok[t] <= 1'b0;
                st <= C_PIX_RD;
            end
            C_PIX_RD: begin
                // land the previous read
                if (trd_pend) begin tap[trd_slot] <= src_rdata; tap_ok[trd_slot] <= trd_ok; end
                trd_pend <= 1'b0;
                if (tk <= 4'd12) begin
                    if (tk == 4'd0 || tinside) begin trd_pend <= 1'b1; trd_slot <= tk; trd_ok <= 1'b1; end
                    else begin tap_ok[tk] <= 1'b0; end
                    tk <= tk + 4'd1;
                end else if (!trd_pend) st <= C_PIX_CALC;
            end
            C_PIX_CALC: st <= C_PIX_WR;
            C_PIX_WR: begin
                tk <= 4'd0;
                for (int t = 0; t <= 12; t++) tap_ok[t] <= 1'b0;
                if (4'(pi_[2:0]) + 4'd1 < pw) begin pi_ <= pi_ + 6'd1; st <= C_PIX_RD; end
                else if (4'(pi_[5:3]) + 4'd1 < ph) begin pi_ <= {pi_[5:3] + 3'd1, 3'd0}; st <= C_PIX_RD; end
                else begin
                    // next plane
                    if (fplane == 2'd0 && !hdr.mono) begin fplane <= 2'd1; st <= C_PLANE_NEXT; end
                    else if (fplane == 2'd1) begin fplane <= 2'd2; st <= C_PLANE_NEXT; end
                    else st <= C_NEXT;
                end
            end
            // ---- unfiltered block: copy src -> dst for all planes
            C_COPY_RD: st <= C_COPY_WR;
            C_COPY_WR: begin
                if (4'(cpi[2:0]) + 4'd1 < pw) begin cpi <= cpi + 6'd1; st <= C_COPY_RD; end
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
                    else st <= C_DONE;
                end
            end
            C_DONE: begin done <= 1'b1; st <= C_IDLE; end
            default: st <= C_IDLE;
        endcase
    end
endmodule
