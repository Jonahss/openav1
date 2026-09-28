// Motion information of the frame being decoded, per 4x4 luma unit (spec IsInters / RefFrames / Mvs / MiSizes
// as the MV stack needs them): written per block by the syntax stage as soon as the block's mode info is
// complete (area fill, `busy` while in flight), read one position per cycle by mv_stack (registered read).
// A 64-row bitmap of "written" bits (per 4x4 column) says whether a position has been written in this
// frame; the tile decoder clears the bitmap rows of a superblock row when it starts one (the MV stack looks
// at most one superblock row up), so no state depends on memory initialisation.
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
    input  logic        clr,                  // clear the written bits of rows clr_row .. clr_row + clr_n - 1
    input  logic [10:0] clr_row,
    input  logic [5:0]  clr_n,
    input  logic [10:0] rd_row, rd_col,
    output mv_ent_t     rd_ent,
    output logic        rd_written
);
    localparam int N = 1 << (ML2R + ML2C);
    localparam int WC = 1 << ML2C;
    mv_ent_t mem [0:N-1];
    logic [WC-1:0] written [0:63];
    logic        c_busy, w_busy;
    logic [10:0] c_row;
    logic [5:0]  c_n;
    assign busy = c_busy || w_busy;
    always_ff @(posedge clk) begin
        if (rst) c_busy <= 1'b0;
        else if (!c_busy) begin
            if (clr) begin c_busy <= 1'b1; c_row <= clr_row; c_n <= clr_n; end
        end else begin
            written[c_row[5:0]] <= '0;
            c_row <= c_row + 11'd1;
            if (c_n == 6'd1) c_busy <= 1'b0;
            c_n <= c_n - 6'd1;
        end
    end
    function automatic logic [ML2R+ML2C-1:0] idx(input logic [10:0] row, input logic [10:0] col);
        idx = {row[ML2R-1:0], col[ML2C-1:0]};
    endfunction
    logic [5:0]  wi, wj;
    logic [10:0] b_r, b_c;
    logic [5:0]  b_w, b_h;
    mv_ent_t     b_d;
    always_ff @(posedge clk) begin
        if (rst) w_busy <= 1'b0;
        else if (!w_busy) begin
            if (we) begin
                w_busy <= 1'b1; b_r <= w_r; b_c <= w_c; b_w <= w_bw4; b_h <= w_bh4; b_d <= w_data; wi <= 6'd0; wj <= 6'd0;
            end
        end else begin
            logic [ML2R+ML2C-1:0] a;
            logic [10:0] rr, cc;
            rr = b_r + 11'(wi); cc = b_c + 11'(wj);
            a = idx(rr, cc);
            mem[a] <= b_d;
            written[rr[5:0]][cc[ML2C-1:0]] <= 1'b1;
            if (wj + 6'd1 < b_w) wj <= wj + 6'd1;
            else begin
                wj <= 6'd0;
                if (wi + 6'd1 < b_h) wi <= wi + 6'd1;
                else w_busy <= 1'b0;
            end
        end
        rd_ent <= mem[idx(rd_row, rd_col)];
        rd_written <= written[rd_row[5:0]][rd_col[ML2C-1:0]];
    end
endmodule
