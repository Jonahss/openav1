// The funcsim netlist is flat: it instantiates the black-boxed frame memory by Vivado's per-parameterisation
// module names with NO parameter overrides, so each name must be a wrapper with that instance's parameters
// baked in (dec_top fb10x9: u_fb = frame_mem NRD5/NWR3, u_fb1 = __parameterized0 2/2, u_fb2 = __parameterized1 1/1).
// Before 2026-10-12 the base name fell back to the model defaults (FBX9, 1 port each): wrong shape, silent.
module frame_mem #(parameter int FBX = 10, parameter int FBY = 9, parameter int PW = 12, parameter int NRD = 5, parameter int NWR = 3) (
    input  logic                clk,
    // read ports
    input  logic [NRD-1:0]      re,
    input  logic [NRD*2-1:0]    r_plane,
    input  logic [NRD*FBX-1:0]  r_x,
    input  logic [NRD*FBY-1:0]  r_y,
    output logic [NRD*PW-1:0]   rdata,
    output logic [NRD*4*PW-1:0] rdata4,
    // write ports
    input  logic [NWR-1:0]      we,
    input  logic [NWR*4-1:0]    w_be,
    input  logic [NWR*2-1:0]    w_plane,
    input  logic [NWR*FBX-1:0]  w_x,
    input  logic [NWR*FBY-1:0]  w_y,
    input  logic [NWR*4*PW-1:0] w_data,
    // host read port
    input  logic [1:0]          h_plane,
    input  logic [FBX-1:0]      h_x,
    input  logic [FBY-1:0]      h_y,
    output logic [PW-1:0]       h_rdata
);
    frame_mem_sim #(.FBX(FBX), .FBY(FBY), .PW(PW), .NRD(NRD), .NWR(NWR)) u (.*);
endmodule
module frame_mem__parameterized0 #(parameter int FBX = 10, parameter int FBY = 9, parameter int PW = 12, parameter int NRD = 2, parameter int NWR = 2) (
    input  logic                clk,
    // read ports
    input  logic [NRD-1:0]      re,
    input  logic [NRD*2-1:0]    r_plane,
    input  logic [NRD*FBX-1:0]  r_x,
    input  logic [NRD*FBY-1:0]  r_y,
    output logic [NRD*PW-1:0]   rdata,
    output logic [NRD*4*PW-1:0] rdata4,
    // write ports
    input  logic [NWR-1:0]      we,
    input  logic [NWR*4-1:0]    w_be,
    input  logic [NWR*2-1:0]    w_plane,
    input  logic [NWR*FBX-1:0]  w_x,
    input  logic [NWR*FBY-1:0]  w_y,
    input  logic [NWR*4*PW-1:0] w_data,
    // host read port
    input  logic [1:0]          h_plane,
    input  logic [FBX-1:0]      h_x,
    input  logic [FBY-1:0]      h_y,
    output logic [PW-1:0]       h_rdata
);
    frame_mem_sim #(.FBX(FBX), .FBY(FBY), .PW(PW), .NRD(NRD), .NWR(NWR)) u (.*);
endmodule
module frame_mem__parameterized1 #(parameter int FBX = 10, parameter int FBY = 9, parameter int PW = 12, parameter int NRD = 1, parameter int NWR = 1) (
    input  logic                clk,
    // read ports
    input  logic [NRD-1:0]      re,
    input  logic [NRD*2-1:0]    r_plane,
    input  logic [NRD*FBX-1:0]  r_x,
    input  logic [NRD*FBY-1:0]  r_y,
    output logic [NRD*PW-1:0]   rdata,
    output logic [NRD*4*PW-1:0] rdata4,
    // write ports
    input  logic [NWR-1:0]      we,
    input  logic [NWR*4-1:0]    w_be,
    input  logic [NWR*2-1:0]    w_plane,
    input  logic [NWR*FBX-1:0]  w_x,
    input  logic [NWR*FBY-1:0]  w_y,
    input  logic [NWR*4*PW-1:0] w_data,
    // host read port
    input  logic [1:0]          h_plane,
    input  logic [FBX-1:0]      h_x,
    input  logic [FBY-1:0]      h_y,
    output logic [PW-1:0]       h_rdata
);
    frame_mem_sim #(.FBX(FBX), .FBY(FBY), .PW(PW), .NRD(NRD), .NWR(NWR)) u (.*);
endmodule
