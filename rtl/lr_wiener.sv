// Wiener loop restoration (spec 7.17.4) as two line modules for the streaming filter chain (docs/filter-chain.md):
// lr_top's arithmetic lifted out with the lane count as a parameter.
//   lr_wiener_h : one line, N output positions: 7 taps over N + 6 source samples (the caller has applied the plane
//                 and stripe rules to them) -> N intermediate values (21 bits, signed), two clocks.
//   lr_wiener_v : N positions: 7 intermediate lines -> N samples, two clocks.
// Coefficients as the block records carry them: three signed 7-bit values c0, c1, c2 for taps 0 / 6, 1 / 5, 2 / 4;
// tap 3 is 128 - 2 (c0 + c1 + c2). Rounding by bit depth: InterRound0 = 3 (5 at 12 bits), InterRound1 = 11 (9).
package lr_wiener_pkg;
    function automatic logic signed [9:0] wcoef(input logic [20:0] c3, input logic [2:0] t);
        logic signed [6:0] c0, c1, c2;
        c0 = c3[6:0]; c1 = c3[13:7]; c2 = c3[20:14];
        case (t)
            3'd0, 3'd6: wcoef = 10'(c0);
            3'd1, 3'd5: wcoef = 10'(c1);
            3'd2, 3'd4: wcoef = 10'(c2);
            default: wcoef = 10'sd128 - 10'sd2 * (10'(c0) + 10'(c1) + 10'(c2));
        endcase
    endfunction
endpackage

module lr_wiener_h
  import lr_wiener_pkg::*;
#(
    parameter int N = 8
) (
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  v_in,
    input  logic [3:0]            bd,
    input  logic [20:0]           coef,        // the horizontal filter's c0, c1, c2
    input  logic [(N+6)*12-1:0]   px,          // source samples x - 3 .. x + N + 2
    output logic                  v_out,
    output logic [N*21-1:0]       out          // N signed 21-bit intermediates
);
    logic [3:0]  round0;
    logic signed [20:0] w_offset, w_limit;
    always_comb begin
        round0 = (bd == 4'd12) ? 4'd5 : 4'd3;
        w_offset = 21'sd1 <<< (bd + 4'd7 - round0 - 4'd1);
        w_limit = (21'sd1 <<< (bd + 4'd8 - round0)) - 21'sd1;
    end
    logic signed [9:0] hf [0:6];
    always_comb for (int t = 0; t < 7; t++) hf[t] = wcoef(coef, 3'(t));
    logic v1;
    logic signed [30:0] pa [N], pb [N];
    always_ff @(posedge clk) begin
        v1 <= v_in && !rst; v_out <= v1 && !rst;
        for (int l = 0; l < N; l++) begin
            logic signed [30:0] a, b, s, v;
            a = 31'sd0; b = 31'sd0;
            for (int t = 0; t < 4; t++) a = a + 31'(hf[t]) * 31'(signed'({19'b0, px[(l + t) * 12 +: 12]}));
            for (int t = 4; t < 7; t++) b = b + 31'(hf[t]) * 31'(signed'({19'b0, px[(l + t) * 12 +: 12]}));
            pa[l] <= a; pb[l] <= b;
            s = pa[l] + pb[l];
            v = (s + (31'sd1 <<< (round0 - 4'd1))) >>> round0;
            out[l * 21 +: 21] <= (v < -31'(w_offset)) ? -w_offset : (v > 31'(w_limit) - 31'(w_offset)) ? 21'(w_limit - w_offset) : 21'(v);
        end
    end
endmodule

module lr_wiener_v
  import lr_wiener_pkg::*;
#(
    parameter int N = 8
) (
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  v_in,
    input  logic [3:0]            bd,
    input  logic [20:0]           coef,        // the vertical filter's c0, c1, c2
    input  logic [7*N*21-1:0]     hv,          // intermediate (line r, position l) at bits (r * N + l) * 21, lines y - 3 .. y + 3
    output logic                  v_out,
    output logic [N*12-1:0]       out
);
    logic [3:0]  round1;
    assign round1 = (bd == 4'd12) ? 4'd9 : 4'd11;
    logic signed [9:0] vf [0:6];
    always_comb for (int t = 0; t < 7; t++) vf[t] = wcoef(coef, 3'(t));
    logic v1;
    logic signed [30:0] pa [N], pb [N];
    always_ff @(posedge clk) begin
        v1 <= v_in && !rst; v_out <= v1 && !rst;
        for (int l = 0; l < N; l++) begin
            logic signed [30:0] a, b, s, v;
            a = 31'sd0; b = 31'sd0;
            for (int r = 0; r < 4; r++) a = a + 31'(vf[r]) * 31'(signed'(hv[(r * N + l) * 21 +: 21]));
            for (int r = 4; r < 7; r++) b = b + 31'(vf[r]) * 31'(signed'(hv[(r * N + l) * 21 +: 21]));
            pa[l] <= a; pb[l] <= b;
            s = pa[l] + pb[l];
            v = (s + (31'sd1 <<< (round1 - 4'd1))) >>> round1;
            out[l * 12 +: 12] <= (v < 0) ? 12'd0 : (v > 31'(signed'({19'b0, 12'((13'd1 << bd) - 13'd1)}))) ? 12'((13'd1 << bd) - 13'd1) : 12'(v);
        end
    end
endmodule

// Test wrapper: seven lr_wiener_h (the seven source lines of an output line) into one lr_wiener_v.
module lr_wiener_tb_top #(
    parameter int N = 8
) (
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  v_in,
    input  logic [3:0]            bd,
    input  logic [20:0]           hcoef, vcoef,
    input  logic [7*(N+6)*12-1:0] px,          // line r's samples at bits r * (N + 6) * 12
    output logic                  v_out,
    output logic [N*12-1:0]       out
);
    logic [6:0] hv_v;
    logic [7*N*21-1:0] hv;
    generate for (genvar r = 0; r < 7; r++) begin : g_h
        lr_wiener_h #(.N(N)) u_h (.clk, .rst, .v_in, .bd, .coef(hcoef), .px(px[r * (N + 6) * 12 +: (N + 6) * 12]),
                                  .v_out(hv_v[r]), .out(hv[r * N * 21 +: N * 21]));
    end endgenerate
    lr_wiener_v #(.N(N)) u_v (.clk, .rst, .v_in(hv_v[0]), .bd, .coef(vcoef), .hv, .v_out, .out);
endmodule
