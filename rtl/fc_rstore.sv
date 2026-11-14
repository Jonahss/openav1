// Row store of the streaming filter chain (docs/filter-chain.md): the pre-filter samples of the superblock rows being
// reconstructed, organised so that the chain reads a whole 4x4 block in a clock. It is a mirror: every write of the
// reconstruction stage to the frame memory (frame_mem_w's output: an aligned group of four samples with lane
// enables) is repeated here, as edge_line repeats the last lines. The frame memory stays what reconstruction reads.
// Per plane four memories, one per line of a block (y mod 4), each addressed by {4x4 row, 4x4 column}: a group write
// goes to one of them, a block read takes the same address from all four.
// The 4x4 rows are a ring: a plane keeps its last 1 << RBY (luma) or 1 << RBC (chroma) block rows. A row must be read
// by the chain before reconstruction comes round to its place again (the decoder's sequencer holds reconstruction
// back as it does for the frame memory's ring).
// Read: block (row0 of the band + bk_row, bk_col) of plane p, 192 bits the clock after bk_re, held until the next
// read; sample (line l, column c) at bits (4 l + c) * 12, as lf_feed takes it.
// Words are 72 bits: a sample in the low 12 bits of an 18-bit lane (two 9-bit bytes), so that the lane enables are
// byte-write enables of block RAM.
module fc_rstore
  import syn_pkg::*;
#(
    parameter int FBX = 11,                  // log2 of the frame memory's sample columns / lines
    parameter int FBY = 11,
    parameter int CW = 10,                   // log2 of the most 4x4 columns of a plane (>= FBX - 2)
    parameter int RBY = 6,                   // log2 of the 4x4 rows kept: luma, chroma
    parameter int RBC = 5
) (
    input  logic                clk,
    input  logic                ssy,         // chroma is vertically subsampled (the band's first chroma row is row0 >> 1)
    // the reconstruction stage's writes (frame_mem_w's output)
    input  logic                we,
    input  logic [3:0]          be,
    input  logic [1:0]          wplane,
    input  logic [FBX-1:0]      wx,
    input  logic [FBY-1:0]      wy,
    input  logic [47:0]         wdata,
    // the chain's reads, per plane
    input  logic [MIW-1:0]      row0,        // the band's first 4x4 luma row
    input  logic [2:0]          bk_re,
    input  logic [2:0][5:0]     bk_row,
    input  logic [2:0][CW-1:0]  bk_col,
    output logic [2:0][191:0]   bk_data
);
    logic [71:0] wd72;
    always_comb begin
        wd72 = '0;
        for (int i = 0; i < 4; i++) wd72[18 * i +: 12] = wdata[12 * i +: 12];
    end
    generate for (genvar p = 0; p < 3; p++) begin : g_p
        localparam int RB = (p == 0) ? RBY : RBC;
        localparam int AW = RB + CW;
        logic [AW-1:0]  wa, ra;
        logic [MIW-1:0] rrow;
        always_comb begin
            wa = {RB'(wy >> 2), CW'(wx >> 2)};
            rrow = ((p != 0 && ssy) ? (row0 >> 1) : row0) + MIW'(bk_row[p]);
            ra = {RB'(rrow), bk_col[p]};
        end
        for (genvar l = 0; l < 4; l++) begin : g_l
            (* ram_style = "block" *) logic [71:0] mem [1 << AW];
            logic [71:0] q;
            always_ff @(posedge clk) begin
                for (int b = 0; b < 8; b++)
                    if (we && wplane == 2'(p) && wy[1:0] == 2'(l) && be[b / 2]) mem[wa][9 * b +: 9] <= wd72[9 * b +: 9];
                if (bk_re[p]) q <= mem[ra];
            end
            for (genvar i = 0; i < 4; i++) begin : g_i
                assign bk_data[p][(4 * l + i) * 12 +: 12] = q[18 * i +: 12];
            end
        end
    end endgenerate
endmodule
