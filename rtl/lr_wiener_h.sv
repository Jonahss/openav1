// Wiener loop restoration, horizontal pass of one line (see rtl/lr_wiener_pkg.sv).
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
