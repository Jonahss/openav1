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
// The write port goes through a register (a write lands in the memory one clock later than frame_mem_w issues it):
// its sources are the reconstruction stage's write logic and its sinks block RAM pins all over the store, a net that
// was the longest of the decoder in the routed 100 MHz image (2026-11-17: 9.4 ns of route in a 10 ns path). The chain
// reads a band only after the band's reconstruction is over, so the clock goes unseen.
module fc_rstore
  import syn_pkg::*;
#(
    parameter int FBX = 11,                  // log2 of the frame memory's sample columns / lines
    parameter int FBY = 11,
    parameter int CW = 10,                   // log2 of the most 4x4 columns of a plane (>= FBX - 2)
    parameter int CWC = CW,                  // the same for a chroma plane's memories (CW - 1 when chroma is always subsampled)
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
    // its second 4-wide write of a cycle (the next group of the row, all four lanes)
    input  logic                we_b,
    input  logic [1:0]          wplane_b,
    input  logic [FBX-1:0]      wx_b,
    input  logic [FBY-1:0]      wy_b,
    input  logic [47:0]         wdata_b,
    // the chain's reads, per plane
    input  logic [MIW-1:0]      row0,        // the band's first 4x4 luma row
    input  logic [2:0]          bk_re,
    input  logic [2:0][5:0]     bk_row,
    input  logic [2:0][CW-1:0]  bk_col,
    output logic [2:0][191:0]   bk_data
);
    logic            we_r, we_b_r;
    logic [3:0]      be_r;
    logic [1:0]      wplane_r, wplane_b_r;
    logic [FBX-1:0]  wx_r, wx_b_r;
    logic [FBY-1:0]  wy_r, wy_b_r;
    logic [71:0]     wd72, wd72_b;
    always_ff @(posedge clk) begin
        we_r <= we; be_r <= be; wplane_r <= wplane; wx_r <= wx; wy_r <= wy;
        we_b_r <= we_b; wplane_b_r <= wplane_b; wx_b_r <= wx_b; wy_b_r <= wy_b;
        wd72 <= '0; wd72_b <= '0;
        for (int i = 0; i < 4; i++) begin wd72[18 * i +: 12] <= wdata[12 * i +: 12]; wd72_b[18 * i +: 12] <= wdata_b[12 * i +: 12]; end
    end
    generate for (genvar p = 0; p < 3; p++) begin : g_p
        localparam int RB = (p == 0) ? RBY : RBC;
        localparam int CWP = (p == 0) ? CW : CWC;
        localparam int AW = RB + CWP - 1;                    // {4x4 row, 4x4 column pair}
        logic [AW-1:0]  wa_a, wa_b, ra;
        logic [MIW-1:0] rrow;
        logic           rpar_q;                              // column parity of the block last read
        always_comb begin
            wa_a = {RB'(wy_r >> 2), (CWP-1)'(wx_r >> 3)};
            wa_b = {RB'(wy_b_r >> 2), (CWP-1)'(wx_b_r >> 3)};
            rrow = ((p != 0 && ssy) ? (row0 >> 1) : row0) + MIW'(bk_row[p]);
            ra = {RB'(rrow), (CWP-1)'(bk_col[p] >> 1)};
        end
        always_ff @(posedge clk) if (bk_re[p]) rpar_q <= bk_col[p][0];
        for (genvar l = 0; l < 4; l++) begin : g_l
            logic [71:0] q [0:1];
            // one memory per column parity: the two writes of a cycle are neighbouring groups of one line, so each
            // memory sees at most one of them (2026-11-23, K60-92)
            for (genvar c = 0; c < 2; c++) begin : g_c
                (* ram_style = "block" *) logic [71:0] mem [1 << AW];
                logic hit_a, hit_b;
                logic [AW-1:0] wa;
                logic [71:0] wd;
                logic [3:0]  wbe;
                always_comb begin
                    hit_a = we_r && wplane_r == 2'(p) && wy_r[1:0] == 2'(l) && wx_r[2] == 1'(c);
                    hit_b = we_b_r && wplane_b_r == 2'(p) && wy_b_r[1:0] == 2'(l) && wx_b_r[2] == 1'(c);
                    wa  = hit_a ? wa_a : wa_b;
                    wd  = hit_a ? wd72 : wd72_b;
                    wbe = hit_a ? be_r : {4{hit_b}};
                end
                always_ff @(posedge clk) begin
                    for (int b = 0; b < 8; b++)
                        if (wbe[b / 2]) mem[wa][9 * b +: 9] <= wd[9 * b +: 9];
                    if (bk_re[p]) q[c] <= mem[ra];
                end
`ifndef SYNTHESIS
                always_ff @(posedge clk) begin
                    if (hit_a && hit_b) $fatal(1, "fc_rstore: plane %0d line %0d: both writes of a cycle in column parity %0d", p, l, c);
                    // the write register is unseen only if no read asks for a word whose write is still in it
                    if (bk_re[p] && (hit_a || hit_b) && bk_col[p][0] == 1'(c) && ra == wa)
                        $fatal(1, "fc_rstore: plane %0d line %0d read of a word still in the write register", p, l);
                end
`endif
            end
            for (genvar i = 0; i < 4; i++) begin : g_i
                assign bk_data[p][(4 * l + i) * 12 +: 12] = rpar_q ? q[1][18 * i +: 12] : q[0][18 * i +: 12];
            end
        end
    end endgenerate
endmodule
