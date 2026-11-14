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
