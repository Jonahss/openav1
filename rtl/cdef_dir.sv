// CDEF direction search (spec 7.15.2) for one 8x8 luma block: cdef_top's four-stage search lifted out for the
// streaming CDEF stage (docs/filter-chain.md). A block may enter every clock.
//   px  : the block's 64 deblocked samples, sample (line i, column j) at bits (8 i + j) * 12
//   cs  : bit_depth - 8
//   dir, var : the direction and the variance (as the specification's yDir and var), four clocks after v_in
// Stages: partial sums; their squares (in DSPs: only the sums the costs use); the eight costs (the squares that share
// the weight 105 are summed first); the best direction and the variance.
module cdef_dir
  import cdef_pkg::*;
(
    input  logic            clk,
    input  logic            rst,
    input  logic            v_in,
    input  logic [64*12-1:0] px,
    input  logic [3:0]      cs,
    output logic            v_out,
    output logic [2:0]      dir,
    output logic [31:0]     var_o
);
    logic signed [15:0] part [0:7][0:14];
    always_comb begin
        for (int d = 0; d < 8; d++) for (int k = 0; k < 15; k++) part[d][k] = 16'sd0;
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++) begin
                logic signed [15:0] s;
                s = 16'(signed'({4'b0, px[(8 * i + j) * 12 +: 12] >> cs})) - 16'sd128;
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
    logic v1, v2, v3;
    logic signed [15:0] part_r [0:7][0:14];
    always_ff @(posedge clk) for (int d = 0; d < 8; d++) for (int k = 0; k < 15; k++) part_r[d][k] <= part[d][k];
    (* use_dsp = "yes" *) logic signed [31:0] sq_r [0:7][0:14];     // |part| < 2^15, so the square fits 31 bits
    // only the partial sums the costs use are squared: d = 0, 4 all 15; d = 2, 6 the first 8; odd d the first 11
    always_ff @(posedge clk) for (int d = 0; d < 8; d++) for (int k = 0; k < 15; k++)
        if (d % 4 == 0 || (d % 2 == 0 && k < 8) || (d % 2 == 1 && k < 11)) sq_r[d][k] <= part_r[d][k] * part_r[d][k];
    function automatic logic signed [42:0] sq(input logic signed [31:0] q);
        sq = 43'(q);
    endfunction
    logic signed [42:0] cost [0:7];
    always_comb begin
        logic signed [42:0] s105 [0:7];
        for (int d = 0; d < 8; d++) begin cost[d] = 43'sd0; s105[d] = 43'sd0; end
        for (int i = 0; i < 8; i++) begin
            s105[2] = s105[2] + sq(sq_r[2][i]);
            s105[6] = s105[6] + sq(sq_r[6][i]);
        end
        for (int i = 0; i < 7; i++) begin
            cost[0] = cost[0] + (sq(sq_r[0][i]) + sq(sq_r[0][14 - i])) * 43'(signed'({33'b0, div_table(4'(i + 1))}));
            cost[4] = cost[4] + (sq(sq_r[4][i]) + sq(sq_r[4][14 - i])) * 43'(signed'({33'b0, div_table(4'(i + 1))}));
        end
        s105[0] = sq(sq_r[0][7]);
        s105[4] = sq(sq_r[4][7]);
        for (int o = 0; o < 4; o++) begin
            int d;
            d = 2 * o + 1;
            for (int j = 0; j < 5; j++) s105[d] = s105[d] + sq(sq_r[d][3 + j]);
            for (int j = 0; j < 3; j++) cost[d] = cost[d] + (sq(sq_r[d][j]) + sq(sq_r[d][10 - j])) * 43'(signed'({33'b0, div_table(4'(2 * j + 2))}));
        end
        for (int d = 0; d < 8; d++) cost[d] = cost[d] + s105[d] * 43'sd105;
    end
    logic signed [42:0] cost_r [0:7];
    always_ff @(posedge clk) for (int d = 0; d < 8; d++) cost_r[d] <= cost[d];
    logic [2:0]  best_dir_w;
    logic signed [42:0] best_cost_w;
    logic [31:0] var_w;
    always_comb begin
        best_cost_w = 43'sd0; best_dir_w = 3'd0;
        for (int d = 0; d < 8; d++) if (cost_r[d] > best_cost_w) begin best_cost_w = cost_r[d]; best_dir_w = 3'(d); end
        var_w = 32'((best_cost_w - cost_r[(best_dir_w + 3'd4) & 3'd7]) >>> 10);
    end
    always_ff @(posedge clk) begin
        v1 <= v_in && !rst; v2 <= v1 && !rst; v3 <= v2 && !rst; v_out <= v3 && !rst;
        dir <= best_dir_w; var_o <= var_w;
    end
endmodule
