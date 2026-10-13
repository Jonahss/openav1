// openav1 — one pair unit of the level-folded inverse transform engine (rtl/itx_lvl.sv): the spec's B
// (rotation by cos128/sin128 with Round2 12, optional output swap) or H (Hadamard with clamp to r bits,
// flip swaps the roles of a and b), same arithmetic as the slot datapath of the microcoded itx1d.
//   mode 0: idle (outputs don't care)   1: B   2: H
module itx_pair #(
    parameter int TW = 20
) (
    input  logic signed [TW-1:0] ia,
    input  logic signed [TW-1:0] ib,
    input  logic signed [12:0]   c,        // cos128(angle)
    input  logic signed [12:0]   s,        // sin128(angle)
    input  logic [1:0]           mode,
    input  logic                 flip,
    input  logic signed [TW-1:0] clip_lo,
    input  logic signed [TW-1:0] clip_hi,
    output logic signed [TW-1:0] oa,
    output logic signed [TW-1:0] ob
);
    logic signed [TW+13:0] x, y, rx, ry;
    logic signed [TW:0]    ha, hb, sum, dif;
    logic signed [TW-1:0]  csum, cdif;
    always_comb begin
        // B: rotation
        x  = ia * c - ib * s;
        y  = ia * s + ib * c;
        rx = (x + (TW+14)'(2048)) >>> 12;
        ry = (y + (TW+14)'(2048)) >>> 12;
        // H: Hadamard with clamp (flip swaps the roles of a and b)
        ha  = (TW+1)'(flip ? ib : ia);
        hb  = (TW+1)'(flip ? ia : ib);
        sum = ha + hb;
        dif = ha - hb;
        csum = (sum < (TW+1)'(clip_lo)) ? clip_lo : (sum > (TW+1)'(clip_hi)) ? clip_hi : TW'(sum);
        cdif = (dif < (TW+1)'(clip_lo)) ? clip_lo : (dif > (TW+1)'(clip_hi)) ? clip_hi : TW'(dif);
        if (mode == 2'd1) begin
            oa = flip ? TW'(ry) : TW'(rx);
            ob = flip ? TW'(rx) : TW'(ry);
        end else begin
            oa = flip ? cdif : csum;          // lane a is "bb" when flipped
            ob = flip ? csum : cdif;
        end
    end
endmodule
