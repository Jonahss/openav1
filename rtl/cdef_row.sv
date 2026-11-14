// CDEF sample filter (spec 7.15.3) for N samples of one line at a time: cdef_top's four-lane pipeline (stages B to E
// there) lifted out with the lane count as a parameter, for the streaming CDEF stage (docs/filter-chain.md): eight
// lanes filter one line of an 8x8 luma block a clock, four a line of a 4:2:0 chroma block.
//   nb   : the line's neighbourhood, 5 lines x (N + 4) columns of deblocked samples, 12 bits each: sample (line rr,
//          column cc) at bits (rr * (N + 4) + cc) * 12; output sample l is the one at line 2, column l + 2
//   nbok : the same positions: the sample exists (inside the picture's coded area, CdefAvailable)
//   dr, pri, sec, adjp, adjs, priodd : the block's direction, strengths, the two constrain() shifts
//          (max(0, damping - FloorLog2(strength))) and (priStr >> coeffShift) & 1, as cdef_top forms them
//   out  : the N filtered samples, four clocks after v_in (v_out)
// Stages: taps selected by direction; constrain() times the tap weight and the min / max over the present taps;
// three partial sums; the sum, rounding and clamp.
module cdef_row
  import cdef_pkg::*;
#(
    parameter int N = 8
) (
    input  logic                    clk,
    input  logic                    rst,
    input  logic                    v_in,
    input  logic [5*(N+4)*12-1:0]   nb,
    input  logic [5*(N+4)-1:0]      nbok,
    input  logic [2:0]              dr,
    input  logic [11:0]             pri, sec,
    input  logic [4:0]              adjp, adjs,
    input  logic                    priodd,
    output logic                    v_out,
    output logic [N*12-1:0]         out
);
    localparam int NC = N + 4;
    function automatic logic signed [12:0] constrain_s(input logic signed [12:0] diff, input logic [11:0] thr, input logic [4:0] adj);
        logic [12:0] ad;
        logic signed [13:0] v;
        if (thr == 12'd0) constrain_s = 13'sd0;
        else begin
            ad = (diff < 0) ? 13'(-diff) : 13'(diff);
            v = 14'(signed'({2'b0, thr})) - 14'(signed'({1'b0, ad >> adj}));
            if (v < 0) v = 14'sd0;
            if (v > 14'(signed'({1'b0, ad}))) v = 14'(signed'({1'b0, ad}));
            constrain_s = (diff < 0) ? -13'(v) : 13'(v);
        end
    endfunction
    // tap (kind, k, sign) of lane l for direction d: line 2 + dy, column l + 2 + dx.
    // kind 0 primary direction, 1 dir - 2, 2 dir + 2; k 0..1; sg 0 -, 1 +. Tap index: kind * 4 + k * 2 + sg.
    function automatic int tap_dy(input int d, input int kind, input int k, input int sg);
        logic [2:0] tdir;
        tdir = (kind == 0) ? 3'(d) : (kind == 1) ? 3'(d - 2) : 3'(d + 2);
        tap_dy = (sg == 1) ? int'(cdef_dy(tdir, 1'(k))) : -int'(cdef_dy(tdir, 1'(k)));
    endfunction
    function automatic int tap_dx(input int d, input int kind, input int k, input int sg);
        logic [2:0] tdir;
        tdir = (kind == 0) ? 3'(d) : (kind == 1) ? 3'(d - 2) : 3'(d + 2);
        tap_dx = (sg == 1) ? int'(cdef_dx(tdir, 1'(k))) : -int'(cdef_dx(tdir, 1'(k)));
    endfunction
    function automatic logic [11:0] tap_at(input int d, input int kind, input int k, input int sg, input int l);
        tap_at = nb[((2 + tap_dy(d, kind, k, sg)) * NC + l + 2 + tap_dx(d, kind, k, sg)) * 12 +: 12];
    endfunction
    function automatic logic tap_ok(input int d, input int kind, input int k, input int sg, input int l);
        tap_ok = nbok[(2 + tap_dy(d, kind, k, sg)) * NC + l + 2 + tap_dx(d, kind, k, sg)];
    endfunction
    function automatic int tap_kind(input int ti); tap_kind = ti / 4; endfunction
    function automatic int tap_k(input int ti); tap_k = (ti / 2) % 2; endfunction
    function automatic int tap_sg(input int ti); tap_sg = ti % 2; endfunction

    // ---- stage B: the 12 taps of each lane selected by direction (8-way muxes), with their presence
    logic        vB;
    logic [11:0] tB  [0:N-1][0:11];
    logic        okB [0:N-1][0:11];
    logic [11:0] tmaxB [0:N-1][0:11], tminB [0:N-1][0:11];   // the tap for the max / min trees: an absent tap does not count
    logic [11:0] ctrB [0:N-1];
    logic [11:0] priB, secB; logic [4:0] adjpB, adjsB; logic prioddB;
    logic [11:0] tB_c  [0:N-1][0:11];
    logic        okB_c [0:N-1][0:11];
    always_comb
        for (int l = 0; l < N; l++)
            for (int ti = 0; ti < 12; ti++) begin
                logic [11:0] t; logic ok;
                t = '0; ok = 1'b0;
                case (dr)                                   // constant offsets in every arm: an 8-way mux per tap
                    3'd0: begin t = tap_at(0, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(0, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd1: begin t = tap_at(1, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(1, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd2: begin t = tap_at(2, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(2, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd3: begin t = tap_at(3, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(3, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd4: begin t = tap_at(4, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(4, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd5: begin t = tap_at(5, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(5, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd6: begin t = tap_at(6, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(6, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    default: begin t = tap_at(7, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(7, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                endcase
                tB_c[l][ti] = t; okB_c[l][ti] = ok;
            end
    always_ff @(posedge clk) begin
        vB <= v_in && !rst;
        for (int l = 0; l < N; l++) begin
            ctrB[l] <= nb[(2 * NC + l + 2) * 12 +: 12];
            for (int ti = 0; ti < 12; ti++) begin
                tB[l][ti] <= tB_c[l][ti]; okB[l][ti] <= okB_c[l][ti];
                tmaxB[l][ti] <= okB_c[l][ti] ? tB_c[l][ti] : 12'd0;
                tminB[l][ti] <= okB_c[l][ti] ? tB_c[l][ti] : 12'hFFF;
            end
        end
        priB <= pri; secB <= sec; adjpB <= adjp; adjsB <= adjs; prioddB <= priodd;
    end

    // ---- stage C: constrain() of every present tap's difference to the centre, times its weight; min / max
    logic        vC;
    logic signed [15:0] termC [0:N-1][0:11];
    logic [11:0] ctrC [0:N-1], mnC [0:N-1], mxC [0:N-1];
    logic signed [15:0] termC_c [0:N-1][0:11];
    logic [11:0] mnC_c [0:N-1], mxC_c [0:N-1];
    function automatic logic [11:0] max13(input logic [11:0] v [0:11], input logic [11:0] ctr);
        logic [11:0] l1 [0:5]; logic [11:0] l2 [0:2]; logic [11:0] a, b;
        for (int k = 0; k < 6; k++) l1[k] = (v[2 * k] > v[2 * k + 1]) ? v[2 * k] : v[2 * k + 1];
        for (int k = 0; k < 3; k++) l2[k] = (l1[2 * k] > l1[2 * k + 1]) ? l1[2 * k] : l1[2 * k + 1];
        a = (l2[0] > l2[1]) ? l2[0] : l2[1];
        b = (l2[2] > ctr) ? l2[2] : ctr;
        max13 = (a > b) ? a : b;
    endfunction
    function automatic logic [11:0] min13(input logic [11:0] v [0:11], input logic [11:0] ctr);
        logic [11:0] l1 [0:5]; logic [11:0] l2 [0:2]; logic [11:0] a, b;
        for (int k = 0; k < 6; k++) l1[k] = (v[2 * k] < v[2 * k + 1]) ? v[2 * k] : v[2 * k + 1];
        for (int k = 0; k < 3; k++) l2[k] = (l1[2 * k] < l1[2 * k + 1]) ? l1[2 * k] : l1[2 * k + 1];
        a = (l2[0] < l2[1]) ? l2[0] : l2[1];
        b = (l2[2] < ctr) ? l2[2] : ctr;
        min13 = (a < b) ? a : b;
    endfunction
    always_comb
        for (int l = 0; l < N; l++) begin
            for (int ti = 0; ti < 12; ti++) begin
                logic signed [12:0] d, cv; logic [2:0] w;
                d = 13'(signed'({1'b0, tB[l][ti]})) - 13'(signed'({1'b0, ctrB[l]}));
                cv = okB[l][ti] ? ((tap_kind(ti) == 0) ? constrain_s(d, priB, adjpB) : constrain_s(d, secB, adjsB)) : 13'sd0;
                w = (tap_kind(ti) == 0) ? pri_tap(prioddB, 1'(tap_k(ti))) : sec_tap(prioddB, 1'(tap_k(ti)));
                case (w)
                    3'd1: termC_c[l][ti] = 16'(cv);
                    3'd2: termC_c[l][ti] = 16'(cv) <<< 1;
                    3'd3: termC_c[l][ti] = (16'(cv) <<< 1) + 16'(cv);
                    default: termC_c[l][ti] = 16'(cv) <<< 2;
                endcase
            end
            mxC_c[l] = max13(tmaxB[l], ctrB[l]); mnC_c[l] = min13(tminB[l], ctrB[l]);
        end
    always_ff @(posedge clk) begin
        vC <= vB && !rst;
        for (int l = 0; l < N; l++) begin
            ctrC[l] <= ctrB[l]; mnC[l] <= mnC_c[l]; mxC[l] <= mxC_c[l];
            for (int ti = 0; ti < 12; ti++) termC[l][ti] <= termC_c[l][ti];
        end
    end

    // ---- stage D: three partial sums of four terms each per lane
    logic        vD;
    logic signed [17:0] partD [0:N-1][0:2];
    logic [11:0] ctrD [0:N-1], mnD [0:N-1], mxD [0:N-1];
    always_ff @(posedge clk) begin
        vD <= vC && !rst;
        for (int l = 0; l < N; l++) begin
            for (int g = 0; g < 3; g++)
                partD[l][g] <= 18'(termC[l][4 * g]) + 18'(termC[l][4 * g + 1]) + 18'(termC[l][4 * g + 2]) + 18'(termC[l][4 * g + 3]);
            ctrD[l] <= ctrC[l]; mnD[l] <= mnC[l]; mxD[l] <= mxC[l];
        end
    end

    // ---- stage E: the sum, rounding and the clamp to the taps' range
    function automatic logic [11:0] cdef_out(input int l);
        logic signed [17:0] total;
        logic signed [18:0] v;
        total = partD[l][0] + partD[l][1] + partD[l][2];
        v = 19'(signed'({7'b0, ctrD[l]})) + ((19'sd8 + 19'(total) - (total < 0 ? 19'sd1 : 19'sd0)) >>> 4);
        if (v < 19'(signed'({7'b0, mnD[l]}))) cdef_out = mnD[l];
        else if (v > 19'(signed'({7'b0, mxD[l]}))) cdef_out = mxD[l];
        else cdef_out = 12'(v);
    endfunction
    always_ff @(posedge clk) begin
        v_out <= vD && !rst;
        for (int l = 0; l < N; l++) out[l * 12 +: 12] <= cdef_out(l);
    end
endmodule
