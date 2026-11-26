// The same memory with the planes in two memories, one per plane class (K60-103): a luma memory and a chroma memory,
// each double-pumped with its own two read and two write slots. Read ports and timing are frame_mem_uram's; there are
// four write ports. Ports 0 and 1 write any plane (reconstruction's two streams): a luma write goes to the luma
// memory's slot of that port, a chroma write to the chroma memory's. Ports 2 and 3 are the chroma lane's (K60-94,
// K60-105: chroma planes only) and take the chroma memory's two slots in the cycles they write; ports 0 and 1 do
// not write a chroma plane in such a cycle (recon_top's rule). So a luma pair and a chroma pair can be written in
// one cycle. No storage is added: the planes already lay in different UltraRAMs.
module frame_mem_uram_pc #(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int PW = 12,
    parameter int FRAME_WIN = 0,
    parameter int SB128 = 0
) (
    input  logic                clk,
    input  logic                clk2x,
    input  logic                phase,
    input  logic                sb128,
    input  logic [1:0]          re,
    input  logic [3:0]          r_plane,
    input  logic [2*FBX-1:0]    r_x,
    input  logic [2*FBY-1:0]    r_y,
    output logic [2*PW-1:0]     rdata,
    output logic [2*4*PW-1:0]   rdata4,
    input  logic [3:0]          we,
    input  logic [15:0]         w_be,
    input  logic [7:0]          w_plane,
    input  logic [4*FBX-1:0]    w_x,
    input  logic [4*FBY-1:0]    w_y,
    input  logic [4*4*PW-1:0]   w_data,
    input  logic [1:0]          h_plane,
    input  logic [FBX-1:0]      h_x,
    input  logic [FBY-1:0]      h_y,
    output logic [PW-1:0]       h_rdata,
    output logic [4*PW-1:0]     h_rdata4,
    output logic                h_ok
);
    // the class of each request: 0 luma, 1 chroma
    logic [1:0] r_c, w_c;
    logic       h_c;
    assign r_c = {r_plane[3:2] != 2'd0, r_plane[1:0] != 2'd0};
    assign w_c = {w_plane[3:2] != 2'd0, w_plane[1:0] != 2'd0};
    // the chroma memory's two write slots: the chroma lane's port (2 + k) when it writes, else port k's chroma write
    logic [1:0]        cw_we;
    logic [7:0]        cw_be;
    logic [3:0]        cw_plane;
    logic [2*FBX-1:0]  cw_x;
    logic [2*FBY-1:0]  cw_y;
    logic [2*4*PW-1:0] cw_data;
    always_comb
        for (int k = 0; k < 2; k++) begin
            logic [1:0] pl;
            pl = we[2 + k] ? w_plane[(2 + k)*2 +: 2] : w_plane[k*2 +: 2];
            cw_we[k]                = we[2 + k] || (we[k] && w_c[k]);
            cw_be[k*4 +: 4]         = we[2 + k] ? w_be[(2 + k)*4 +: 4] : w_be[k*4 +: 4];
            cw_plane[k*2 +: 2]      = (pl != 2'd0) ? pl : 2'd1;
            cw_x[k*FBX +: FBX]      = we[2 + k] ? w_x[(2 + k)*FBX +: FBX] : w_x[k*FBX +: FBX];
            cw_y[k*FBY +: FBY]      = we[2 + k] ? w_y[(2 + k)*FBY +: FBY] : w_y[k*FBY +: FBY];
            cw_data[k*4*PW +: 4*PW] = we[2 + k] ? w_data[(2 + k)*4*PW +: 4*PW] : w_data[k*4*PW +: 4*PW];
        end
`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if ((we[2] || we[3]) && ((we[0] && w_c[0]) || (we[1] && w_c[1]))) $fatal(1, "frame_mem_uram_pc: the chroma lane and a shared port write chroma in one cycle");
        if ((we[2] && w_plane[5:4] == 2'd0) || (we[3] && w_plane[7:6] == 2'd0)) $fatal(1, "frame_mem_uram_pc: a luma write on a chroma lane port");
    end
`endif
    assign h_c = h_plane != 2'd0;
    logic [2*PW-1:0]   m_rdata [2];
    logic [2*4*PW-1:0] m_rdata4 [2];
    logic [PW-1:0]     m_h_rdata [2];
    logic [4*PW-1:0]   m_h_rdata4 [2];
    logic              m_h_ok [2];                           // unused: h_ok is formed here from the shared read port
    // each memory sees only its class's requests; a plane number of the other class is replaced by one of its own, so
    // that the address it forms for a request it does not perform stays inside the memory
    frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(PW), .FRAME_WIN(FRAME_WIN), .SB128(SB128), .PCLASS(1)) u_l (.clk, .clk2x, .phase, .sb128,
        .re({re[1] && !r_c[1], re[0] && !r_c[0]}), .r_plane(4'd0), .r_x, .r_y, .rdata(m_rdata[0]), .rdata4(m_rdata4[0]),
        .we({we[1] && !w_c[1], we[0] && !w_c[0]}), .w_be(w_be[7:0]), .w_plane(4'd0), .w_x(w_x[2*FBX-1:0]), .w_y(w_y[2*FBY-1:0]), .w_data(w_data[2*4*PW-1:0]),
        .h_plane(2'd0), .h_x, .h_y, .h_rdata(m_h_rdata[0]), .h_rdata4(m_h_rdata4[0]), .h_ok(m_h_ok[0]));
    frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(PW), .FRAME_WIN(FRAME_WIN), .SB128(SB128), .PCLASS(2)) u_c (.clk, .clk2x, .phase, .sb128,
        .re({re[1] && r_c[1], re[0] && r_c[0]}), .r_plane({r_c[1] ? r_plane[3:2] : 2'd1, r_c[0] ? r_plane[1:0] : 2'd1}), .r_x, .r_y, .rdata(m_rdata[1]), .rdata4(m_rdata4[1]),
        .we(cw_we), .w_be(cw_be), .w_plane(cw_plane), .w_x(cw_x), .w_y(cw_y), .w_data(cw_data),
        .h_plane(h_c ? h_plane : 2'd1), .h_x, .h_y, .h_rdata(m_h_rdata[1]), .h_rdata4(m_h_rdata4[1]), .h_ok(m_h_ok[1]));
    // the data comes two cycles after the request, from the memory of the request's class
    logic [1:0] r_c1, r_c2;
    logic       h_c1, h_c2, h_ok1;
    always_ff @(posedge clk) begin
        r_c1 <= r_c; r_c2 <= r_c1; h_c1 <= h_c; h_c2 <= h_c1;
        h_ok1 <= !re[1]; h_ok <= h_ok1;                      // as in one memory: the host read is served when read-port 1 is idle
    end
    always_comb begin
        for (int i = 0; i < 2; i++) begin
            rdata[i*PW +: PW]       = m_rdata[32'(r_c2[i])][i*PW +: PW];
            rdata4[i*4*PW +: 4*PW]  = m_rdata4[32'(r_c2[i])][i*4*PW +: 4*PW];
        end
        h_rdata  = m_h_rdata[32'(h_c2)];
        h_rdata4 = m_h_rdata4[32'(h_c2)];
    end
endmodule
