// Boundary line store of the streaming filter chain's restoration stage (docs/filter-chain.md), one plane.
// Restoration reads across a stripe boundary in the deblocked picture, not in CDEF's output (7.17.6): the two lines
// above a stripe's first line B and the two from B on. This store takes them out of the deblock stage's output as the
// blocks go by and answers lr_stream's port.
//   In: lf_stream's finished 4x4 blocks (out_valid && out_ready) with their column and their row counted from the
//   band's first block row (wr_row0 = that row in the plane). With B4 = B / 4: block row B4 - 1 holds lines B - 2 and
//   B - 1 (its lines 2 and 3), block row B4 holds B and B + 1; every other block passes by.
//   Read: boundary bl_k (B = 64 k - 8 in luma, 32 k - 4 in a vertically subsampled plane) at 4x4 column bl_col, the
//   first column of a segment of 8 samples (4 in a horizontally subsampled plane); one clock later the lines B - 2,
//   B - 1, B, B + 1 of the segment, line i at bits i * 96.
// Four boundaries are kept (bl_k modulo 4). In a band of 128 lines restoration reads boundaries k and k + 1 while the
// deblock stage of the same band writes into k, k + 1 and k + 2 (the two lines above k + 2 are final by then), so
// three are alive at once; the deblock stage must not be a band ahead of restoration's reads.
// Lines that were never written (outside the picture) read as whatever the memory holds: lr_stream does not use them.
// Memory: 96 bits (two lines of four samples) a 4x4 column, boundary and half: 4 * 2 * 96 bits a 4x4 column.
module lr_bstore
  import syn_pkg::*;
#(
    parameter int CW = 10                    // log2 of the most 4x4 columns
) (
    input  logic            clk,
    input  logic            ssx, ssy,        // the plane's subsampling (0 for luma)
    input  logic [MIW-1:0]  wr_row0,         // the deblock stage's band: its first 4x4 row in the plane
    input  logic            wr_valid,
    input  logic [191:0]    wr_blk,          // sample (line l, column c) at bits (4 l + c) * 12
    input  logic [CW-1:0]   wr_col,
    input  logic signed [6:0] wr_row,
    input  logic [SBW:0]    bl_k,
    input  logic [CW-1:0]   bl_col,
    output logic [383:0]    bl_rd
);
    // a segment's first four samples (a 4x4 column of a subsampled plane, an even one else) and its last four
    (* ram_style = "block" *) logic [95:0] top_a [1 << (CW + 2)];
    (* ram_style = "block" *) logic [95:0] bot_a [1 << (CW + 2)];
    (* ram_style = "block" *) logic [95:0] top_b [1 << (CW + 1)];
    (* ram_style = "block" *) logic [95:0] bot_b [1 << (CW + 1)];
    logic [MIW:0]   ra, m_bot, m_top;
    logic           is_bot, is_top;
    logic [1:0]     w_slot;
    logic [CW-1:0]  w_seg, r_seg;
    logic           w_half;
    logic [95:0]    q_ta, q_ba, q_tb, q_bb;
    always_comb begin
        ra = (MIW+1)'(signed'({1'b0, wr_row0}) + (MIW+1)'(wr_row));
        m_bot = ra + (ssy ? (MIW+1)'(1) : (MIW+1)'(2));          // 4 (ra + 2) = 64 k - 8 + 16: block row B4 of boundary k
        m_top = m_bot + (MIW+1)'(1);
        is_bot = ssy ? (m_bot[2:0] == 3'd0) : (m_bot[3:0] == 4'd0);
        is_top = ssy ? (m_top[2:0] == 3'd0) : (m_top[3:0] == 4'd0);
        w_slot = is_bot ? (ssy ? m_bot[4:3] : m_bot[5:4]) : (ssy ? m_top[4:3] : m_top[5:4]);
        w_seg = ssx ? wr_col : {1'b0, wr_col[CW-1:1]};
        w_half = !ssx && wr_col[0];
        r_seg = ssx ? bl_col : {1'b0, bl_col[CW-1:1]};
    end
    always_ff @(posedge clk) begin
        if (wr_valid && is_top && !w_half) top_a[{w_slot, w_seg}] <= wr_blk[191:96];
        if (wr_valid && is_bot && !w_half) bot_a[{w_slot, w_seg}] <= wr_blk[95:0];
        if (wr_valid && is_top && w_half) top_b[{w_slot, w_seg[CW-2:0]}] <= wr_blk[191:96];
        if (wr_valid && is_bot && w_half) bot_b[{w_slot, w_seg[CW-2:0]}] <= wr_blk[95:0];
        q_ta <= top_a[{bl_k[1:0], r_seg}];
        q_ba <= bot_a[{bl_k[1:0], r_seg}];
        q_tb <= top_b[{bl_k[1:0], r_seg[CW-2:0]}];
        q_bb <= bot_b[{bl_k[1:0], r_seg[CW-2:0]}];
    end
    assign bl_rd = {q_bb[95:48], q_ba[95:48], q_bb[47:0], q_ba[47:0], q_tb[95:48], q_ta[95:48], q_tb[47:0], q_ta[47:0]};
endmodule
