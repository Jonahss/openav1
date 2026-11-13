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
    // an intra edge read
    input  logic            re,
    input  logic [1:0]      rplane,
    input  logic [FBX-1:0]  rx,
    input  logic [FBY-1:0]  ry,
    output logic            sel,
    output logic [PW-1:0]   rdata,
    output logic [4*PW-1:0] rdata4
);
    localparam int AW = FBX + 1;                             // {row parity, plane, group}
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
    logic [PW-1:0] mem0 [1 << AW];
    logic [PW-1:0] mem1 [1 << AW];
    logic [PW-1:0] mem2 [1 << AW];
    logic [PW-1:0] mem3 [1 << AW];
    logic [AW-1:0] waddr, raddr;
    assign waddr = {par(wplane, wy), wplane, wx[FBX-1:2]};
    assign raddr = {par(rplane, ry), rplane, rx[FBX-1:2]};
    logic          wl;
    assign wl = we && is_last(wplane, wy);
    logic [4*PW-1:0] q1, q2;
    logic            s1;
    logic [1:0]      l1, l2;
    always_ff @(posedge clk) begin
        if (wl && be[0]) mem0[waddr] <= wdata[0*PW +: PW];
        if (wl && be[1]) mem1[waddr] <= wdata[1*PW +: PW];
        if (wl && be[2]) mem2[waddr] <= wdata[2*PW +: PW];
        if (wl && be[3]) mem3[waddr] <= wdata[3*PW +: PW];
        q1 <= {mem3[raddr], mem2[raddr], mem1[raddr], mem0[raddr]};
        q2 <= q1;
        s1 <= re && is_last(rplane, ry); sel <= s1;
        l1 <= rx[1:0]; l2 <= l1;
    end
    assign rdata4 = q2;
    assign rdata  = q2[l2 * PW +: PW];
endmodule
