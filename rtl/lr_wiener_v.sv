// Wiener loop restoration, vertical pass over seven intermediate lines (see rtl/lr_wiener_pkg.sv).
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
