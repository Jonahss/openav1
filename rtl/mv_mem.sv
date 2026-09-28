// Motion information of the frame being decoded, per 4x4 luma unit (spec IsInters / RefFrames / Mvs / MiSizes
// as the MV stack needs them): written per block by the syntax stage as soon as the block's mode info is
// complete (area fill, `busy` while in flight), read one position per cycle by mv_stack (registered read).
// Entries carry the frame parity of their write, so a stale entry from the previous frame reads as
// "not written yet" without clearing the memory between frames.
module mv_mem
  import syn_pkg::*;
#(
    parameter int ML2R = 9,
    parameter int ML2C = 10
) (
    input  logic        clk,
    input  logic        rst,
    input  logic        we,
    input  logic [10:0] w_r, w_c,
    input  logic [5:0]  w_bw4, w_bh4,
    input  mv_ent_t     w_data,
    output logic        busy,
    input  logic [10:0] rd_row, rd_col,
    output mv_ent_t     rd_ent
);
    localparam int N = 1 << (ML2R + ML2C);
    mv_ent_t mem [0:N-1];
    function automatic logic [ML2R+ML2C-1:0] idx(input logic [10:0] row, input logic [10:0] col);
        idx = {row[ML2R-1:0], col[ML2C-1:0]};
    endfunction
    logic [5:0]  wi, wj;
    logic [10:0] b_r, b_c;
    logic [5:0]  b_w, b_h;
    mv_ent_t     b_d;
    always_ff @(posedge clk) begin
        if (rst) busy <= 1'b0;
        else if (!busy) begin
            if (we) begin
                busy <= 1'b1; b_r <= w_r; b_c <= w_c; b_w <= w_bw4; b_h <= w_bh4; b_d <= w_data; wi <= 6'd0; wj <= 6'd0;
            end
        end else begin
            logic [ML2R+ML2C-1:0] a;
            a = idx(b_r + 11'(wi), b_c + 11'(wj));
            mem[a] <= b_d;
            if (wj + 6'd1 < b_w) wj <= wj + 6'd1;
            else begin
                wj <= 6'd0;
                if (wi + 6'd1 < b_h) wi <= wi + 6'd1;
                else busy <= 1'b0;
            end
        end
        rd_ent <= mem[idx(rd_row, rd_col)];
    end
endmodule
