// Self-guided loop restoration (spec 7.17.3) for the streaming filter chain: one line of N samples from the a2 / b2
// values around it, three clocks (the weighted sums, the two filtered values, the blend with the source).
//   ab0_* : pass 0 (radius 2), ab1_* : pass 1 (radius 1); _m, _c, _p = the rows above, of and below the line; a row
//   holds N + 2 positions (x - 1 .. x + N), position k at bits k * 31 as {a2[8:0], b2[21:0]}.
//   odd = the line's parity in the plane: pass 0 has values on odd lines only (an odd line is filtered from its own
//   row with weights 5 6 5, an even line from the rows above and below it); pass 1 weighs 3 4 3 / 4 4 4 / 3 4 3.
//   en0 / en1 = the pass is used by the unit's parameter set; w0, w1 = the unit's weights (xqd). An unused pass
//   stands in as the source sample (a bitstream codes w0 = 0 with en0 = 0, so only en1 changes a legal unit's output).
module lr_sgr_f #(
    parameter int N = 8
) (
    input  logic                    clk,
    input  logic                    rst,
    input  logic                    v_in,
    input  logic [3:0]              bd,
    input  logic                    odd,
    input  logic                    en0, en1,
    input  logic signed [7:0]       w0, w1,
    input  logic [(N+2)*31-1:0]     ab0_m, ab0_c, ab0_p,
    input  logic [(N+2)*31-1:0]     ab1_m, ab1_c, ab1_p,
    input  logic [N*12-1:0]         pix,
    output logic                    v_out,
    output logic [N*12-1:0]         out
);
    function automatic logic [15:0] a_of(input logic [(N+2)*31-1:0] row, input int k);
        a_of = 16'(row[k * 31 + 22 +: 9]);
    endfunction
    function automatic logic [28:0] b_of(input logic [(N+2)*31-1:0] row, input int k);
        b_of = 29'(row[k * 31 +: 22]);
    endfunction
    logic               v1, v2;
    logic [15:0]        fa [2][N];
    logic [28:0]        fb [2][N];
    logic [N*12-1:0]    pix1, pix2;
    logic               odd1, en0_1, en1_1, en0_2, en1_2;
    logic signed [7:0]  w0_1, w1_1, w0_2, w1_2;
    logic [23:0]        f [2][N];
    always_ff @(posedge clk) begin
        v1 <= v_in && !rst; v2 <= v1 && !rst; v_out <= v2 && !rst;
        // the weighted sums of a2 and b2
        for (int l = 0; l < N; l++) begin
            if (odd) begin
                fa[0][l] <= 16'd5 * (a_of(ab0_c, l) + a_of(ab0_c, l + 2)) + 16'd6 * a_of(ab0_c, l + 1);
                fb[0][l] <= 29'd5 * (b_of(ab0_c, l) + b_of(ab0_c, l + 2)) + 29'd6 * b_of(ab0_c, l + 1);
            end else begin
                fa[0][l] <= 16'd5 * (a_of(ab0_m, l) + a_of(ab0_m, l + 2) + a_of(ab0_p, l) + a_of(ab0_p, l + 2))
                          + 16'd6 * (a_of(ab0_m, l + 1) + a_of(ab0_p, l + 1));
                fb[0][l] <= 29'd5 * (b_of(ab0_m, l) + b_of(ab0_m, l + 2) + b_of(ab0_p, l) + b_of(ab0_p, l + 2))
                          + 29'd6 * (b_of(ab0_m, l + 1) + b_of(ab0_p, l + 1));
            end
            fa[1][l] <= 16'd3 * (a_of(ab1_m, l) + a_of(ab1_m, l + 2) + a_of(ab1_p, l) + a_of(ab1_p, l + 2))
                      + 16'd4 * (a_of(ab1_m, l + 1) + a_of(ab1_p, l + 1) + a_of(ab1_c, l) + a_of(ab1_c, l + 1) + a_of(ab1_c, l + 2));
            fb[1][l] <= 29'd3 * (b_of(ab1_m, l) + b_of(ab1_m, l + 2) + b_of(ab1_p, l) + b_of(ab1_p, l + 2))
                      + 29'd4 * (b_of(ab1_m, l + 1) + b_of(ab1_p, l + 1) + b_of(ab1_c, l) + b_of(ab1_c, l + 1) + b_of(ab1_c, l + 2));
        end
        pix1 <= pix; odd1 <= odd; en0_1 <= en0; en1_1 <= en1; w0_1 <= w0; w1_1 <= w1;
        // the filtered values: (a * sample + b) rounded by 9 bits, by 8 for pass 0 on an odd line
        for (int l = 0; l < N; l++) begin
            logic [40:0] va, vb;
            va = 41'(fa[0][l]) * 41'(pix1[l * 12 +: 12]) + 41'(fb[0][l]);
            vb = 41'(fa[1][l]) * 41'(pix1[l * 12 +: 12]) + 41'(fb[1][l]);
            f[0][l] <= odd1 ? 24'((va + 41'd128) >> 8) : 24'((va + 41'd256) >> 9);
            f[1][l] <= 24'((vb + 41'd256) >> 9);
        end
        pix2 <= pix1; en0_2 <= en0_1; en1_2 <= en1_1; w0_2 <= w0_1; w1_2 <= w1_1;
        // the blend
        for (int l = 0; l < N; l++) begin
            logic signed [9:0] w2; logic signed [35:0] v, sv; logic [16:0] u;
            w2 = 10'sd128 - 10'(w0_2) - 10'(w1_2);
            u = 17'(pix2[l * 12 +: 12]) << 4;
            v = 36'(w1_2) * 36'(signed'({19'b0, u}));
            v = v + 36'(w0_2) * (en0_2 ? 36'(signed'({12'b0, f[0][l]})) : 36'(signed'({19'b0, u})));
            v = v + 36'(w2) * (en1_2 ? 36'(signed'({12'b0, f[1][l]})) : 36'(signed'({19'b0, u})));
            sv = (v + 36'sd1024) >>> 11;
            out[l * 12 +: 12] <= (sv < 0) ? 12'd0 : (sv > 36'(signed'({24'b0, 12'((13'd1 << bd) - 13'd1)}))) ? 12'((13'd1 << bd) - 13'd1) : 12'(sv);
        end
    end
endmodule
