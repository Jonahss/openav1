// Synthesis stand-in for rtl/frame_mem.sv: the same ports, no body. Vivado treats it as a black box, so the
// decoder logic synthesises with the frame buffer left external. That is the intended architecture (the
// pictures live in DRAM behind per-stage line buffers; the multi-port array in frame_mem.sv is a simulation
// model that no RAM block can implement), and it keeps the utilisation report honest: every LUT, flop and
// block RAM counted is decoder logic, and the frame-buffer interface shows up as unconnected ports.
(* black_box *)
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
    output logic [4*PW-1:0]     h_rdata4
);
endmodule
