// Unfiltered copy of each superblock row's last line (2026-11-13). Intra prediction of superblock row r+1 reads its
// above row from row r's last line as reconstructed; the deblocking filter's vertical edges change that line in
// place. With this copy the edge reads no longer need the frame memory's version, so deblocking of row r can start
// as soon as row r is decoded instead of waiting for row r+1 (dec_top's frame sequencer, lf_ok).
// Every reconstruction write to a last line (y + 1 a multiple of the plane's superblock height) is mirrored here;
// two copies, by the parity of the superblock row, so that row r+1 writing its own last line never replaces a
// sample of row r that a later block of row r+1 still reads (a 32x64 block's top-left neighbour lies under the
// block to its left). Read: the aligned group of 4 containing rx, two cycles after the request, like the frame
// memory's port; sel says the request was for a last line and is served from here.
module edge_line #(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int PW = 12
) (
    input  logic            clk,
    input  logic            sb128,
    input  logic            ssy,
    // the reconstruction stage's writes (frame_mem_w output)
    input  logic            we,
    input  logic [3:0]      be,
    input  logic [1:0]      wplane,
    input  logic [FBX-1:0]  wx,
    input  logic [FBY-1:0]  wy,
    input  logic [4*PW-1:0] wdata,
    // its second 4-wide write of a cycle (the group after the first one's: the other group parity; all four lanes)
    input  logic            we_b,
    input  logic [1:0]      wplane_b,
    input  logic [FBX-1:0]  wx_b,
    input  logic [FBY-1:0]  wy_b,
    input  logic [4*PW-1:0] wdata_b,
    // the chroma lane's two 4-wide writes (K60-105; a chroma plane, neighbouring groups of one line; never in a cycle
    // in which the writes above are to a chroma plane)
    input  logic            we_c,
    input  logic [3:0]      be_c,
    input  logic [1:0]      wplane_c,
    input  logic [FBX-1:0]  wx_c,
    input  logic [FBY-1:0]  wy_c,
    input  logic [4*PW-1:0] wdata_c,
    input  logic            we_d,
    input  logic [FBX-1:0]  wx_d,
    input  logic [4*PW-1:0] wdata_d,
    // an intra edge read
    input  logic            re,
    input  logic [1:0]      rplane,
    input  logic [FBX-1:0]  rx,
    input  logic [FBY-1:0]  ry,
    output logic            sel,
    output logic [PW-1:0]   rdata,
    output logic [4*PW-1:0] rdata4
);
    function automatic logic [2:0] lg_h(input logic [1:0] plane);    // log2 of the plane's superblock height
        lg_h = (sb128 ? 3'd7 : 3'd6) - 3'((plane != 2'd0) && ssy);
    endfunction
    function automatic logic is_last(input logic [1:0] plane, input logic [FBY-1:0] y);
        logic [FBY-1:0] m;
        m = FBY'((1 << lg_h(plane)) - 1);
        is_last = ((y & m) == m);
    endfunction
    function automatic logic par(input logic [1:0] plane, input logic [FBY-1:0] y);
        logic [FBY-1:0] s;
        s = y >> lg_h(plane);
        par = s[0];
    endfunction
    // One set of memories per plane class (luma; the two chroma planes), so that the luma writes and the chroma lane's
    // writes of a cycle never meet in one memory (K60-105). Address: {row parity, group pair} for luma, {row parity,
    // which chroma plane, group pair} for chroma (4:2:0: a chroma line has half the groups), FBX - 2 bits either way.
    localparam int AW = FBX - 2;
    function automatic logic [AW-1:0] ea(input logic [1:0] plane, input logic [FBX-1:0] x, input logic [FBY-1:0] y);
        ea = (plane == 2'd0) ? {par(plane, y), x[FBX-1:3]} : {par(plane, y), plane[1], x[FBX-2:3]};
    endfunction
    logic [AW-1:0] raddr;
    assign raddr = ea(rplane, rx, ry);
    logic [4*PW-1:0] q1 [0:1][0:1];                          // [plane class][group parity]
    logic [4*PW-1:0] q2;
    logic            s1, rp1, rc1;
    logic [1:0]      l1, l2;
    // per class, one set of lane memories per group parity (x[2]): the two writes of a cycle of one class are
    // neighbouring groups, so each set sees at most one of them
    generate for (genvar cl = 0; cl < 2; cl++) begin : g_cl
      for (genvar gp = 0; gp < 2; gp++) begin : g_gp
        logic [PW-1:0] mem0 [1 << AW];
        logic [PW-1:0] mem1 [1 << AW];
        logic [PW-1:0] mem2 [1 << AW];
        logic [PW-1:0] mem3 [1 << AW];
        logic          hit_a, hit_b, hit_c, hit_d;
        logic [AW-1:0] waddr;
        logic [4*PW-1:0] wd;
        logic [3:0]    wbe;
        always_comb begin
            hit_a = we && ((wplane != 2'd0) == 1'(cl)) && is_last(wplane, wy) && (wx[2] == 1'(gp));
            hit_b = we_b && ((wplane_b != 2'd0) == 1'(cl)) && is_last(wplane_b, wy_b) && (wx_b[2] == 1'(gp));
            hit_c = (cl == 1) && we_c && is_last(wplane_c, wy_c) && (wx_c[2] == 1'(gp));
            hit_d = (cl == 1) && we_d && is_last(wplane_c, wy_c) && (wx_d[2] == 1'(gp));
            waddr = hit_a ? ea(wplane, wx, wy) : hit_b ? ea(wplane_b, wx_b, wy_b) : hit_c ? ea(wplane_c, wx_c, wy_c) : ea(wplane_c, wx_d, wy_c);
            wd    = hit_a ? wdata : hit_b ? wdata_b : hit_c ? wdata_c : wdata_d;
            wbe   = hit_a ? be : hit_b ? 4'hF : hit_c ? be_c : {4{hit_d}};
        end
        always_ff @(posedge clk) begin
            if (wbe[0]) mem0[waddr] <= wd[0*PW +: PW];
            if (wbe[1]) mem1[waddr] <= wd[1*PW +: PW];
            if (wbe[2]) mem2[waddr] <= wd[2*PW +: PW];
            if (wbe[3]) mem3[waddr] <= wd[3*PW +: PW];
            q1[cl][gp] <= {mem3[raddr], mem2[raddr], mem1[raddr], mem0[raddr]};
        end
`ifndef SYNTHESIS
        always_ff @(posedge clk)
            if (32'(hit_a) + 32'(hit_b) + 32'(hit_c) + 32'(hit_d) > 1) $fatal(1, "edge_line: two writes of a cycle in plane class %0d, group parity %0d", cl, gp);
`endif
      end
    end endgenerate
    always_ff @(posedge clk) begin
        rp1 <= rx[2]; rc1 <= (rplane != 2'd0);
        q2 <= q1[rc1][rp1];
        s1 <= re && is_last(rplane, ry); sel <= s1;
        l1 <= rx[1:0]; l2 <= l1;
    end
    assign rdata4 = q2;
    assign rdata  = q2[l2 * PW +: PW];
endmodule
