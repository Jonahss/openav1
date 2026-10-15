// Place-and-route stand-in for rtl/frame_mem.sv: the same ports, a tiny registered memory behind each read
// port so that the decoder's paths into and out of the frame-buffer interface are real for timing and
// routing. NOT functional as a frame buffer: each read port owns a 256-entry array indexed by a hash of
// (plane, x, y) that is written by one write port (port i mod NWR), so the decoder logic is fully driven
// and loaded while the picture itself is nonsense. Used by run.sh -f pr for implementation trials; the real
// design puts the pictures in DRAM behind per-stage line buffers (see frame_mem_bb.sv).
module frame_mem #(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int PW = 12,
    parameter int NRD = 1,
    parameter int NWR = 1
) (
    input  logic                clk,
    input  logic [NRD-1:0]      re,
    input  logic [NRD*2-1:0]    r_plane,
    input  logic [NRD*FBX-1:0]  r_x,
    input  logic [NRD*FBY-1:0]  r_y,
    output logic [NRD*PW-1:0]   rdata,
    output logic [NRD*4*PW-1:0] rdata4,
    input  logic [NWR-1:0]      we,
    input  logic [NWR*4-1:0]    w_be,
    input  logic [NWR*2-1:0]    w_plane,
    input  logic [NWR*FBX-1:0]  w_x,
    input  logic [NWR*FBY-1:0]  w_y,
    input  logic [NWR*4*PW-1:0] w_data,
    input  logic [1:0]          h_plane,
    input  logic [FBX-1:0]      h_x,
    input  logic [FBY-1:0]      h_y,
    output logic [PW-1:0]       h_rdata,
    output logic [4*PW-1:0]     h_rdata4,
    output logic                h_ok
);
    function automatic logic [7:0] hsh(input logic [1:0] p, input logic [FBX-1:0] x, input logic [FBY-1:0] y);
        return 8'(x >> 2) ^ 8'(y << 2) ^ {6'd0, p};
    endfunction
    genvar gi;
    generate
        for (gi = 0; gi < NRD; gi++) begin : g_rd
            localparam int WI = gi % NWR;
            (* ram_style = "distributed" *) logic [4*PW-1:0] mem [0:255];
            logic [7:0] ra;
            logic [4*PW-1:0] q;
            assign ra = hsh(r_plane[gi*2 +: 2], r_x[gi*FBX +: FBX], r_y[gi*FBY +: FBY]);
            always_ff @(posedge clk) begin
                if (we[WI])
                    for (int l = 0; l < 4; l++)
                        if (w_be[WI*4 + l]) mem[hsh(w_plane[WI*2 +: 2], w_x[WI*FBX +: FBX], w_y[WI*FBY +: FBY])][l*PW +: PW] <= w_data[(WI*4 + l)*PW +: PW];
                if (re[gi]) q <= mem[ra];
            end
            assign rdata4[gi*4*PW +: 4*PW] = q;
            assign rdata[gi*PW +: PW] = q[PW-1:0];
        end
    endgenerate
    assign h_ok = 1'b1;
    always_ff @(posedge clk) begin
        h_rdata <= g_rd[0].mem[hsh(h_plane, h_x, h_y)][PW-1:0];
        for (int l = 0; l < 4; l++) h_rdata4[l*PW +: PW] <= g_rd[0].mem[hsh(h_plane, {h_x[FBX-1:2], 2'(l)}, h_y)][PW-1:0];
    end
endmodule
