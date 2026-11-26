// The same memory with the planes in two memories, one per plane class (K60-103): a luma memory and a chroma memory,
// each double-pumped with its own two read and two write slots. Ports and timing are frame_mem_uram's. A request goes
// to the memory of its plane; as long as the two write ports are shared by all planes (today) nothing is gained, but
// the classes no longer share a write slot, which is what a prediction lane per plane class needs (K60-94): the
// chroma memory's write ports can then take a stream of their own. No storage is added: the planes already lay in
// different UltraRAMs.
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
    input  logic [1:0]          we,
    input  logic [7:0]          w_be,
    input  logic [3:0]          w_plane,
    input  logic [2*FBX-1:0]    w_x,
    input  logic [2*FBY-1:0]    w_y,
    input  logic [2*4*PW-1:0]   w_data,
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
        .we({we[1] && !w_c[1], we[0] && !w_c[0]}), .w_be, .w_plane(4'd0), .w_x, .w_y, .w_data,
        .h_plane(2'd0), .h_x, .h_y, .h_rdata(m_h_rdata[0]), .h_rdata4(m_h_rdata4[0]), .h_ok(m_h_ok[0]));
    frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(PW), .FRAME_WIN(FRAME_WIN), .SB128(SB128), .PCLASS(2)) u_c (.clk, .clk2x, .phase, .sb128,
        .re({re[1] && r_c[1], re[0] && r_c[0]}), .r_plane({r_c[1] ? r_plane[3:2] : 2'd1, r_c[0] ? r_plane[1:0] : 2'd1}), .r_x, .r_y, .rdata(m_rdata[1]), .rdata4(m_rdata4[1]),
        .we({we[1] && w_c[1], we[0] && w_c[0]}), .w_be, .w_plane({w_c[1] ? w_plane[3:2] : 2'd1, w_c[0] ? w_plane[1:0] : 2'd1}), .w_x, .w_y, .w_data,
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
