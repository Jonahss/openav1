// Frame buffer model: three planes of (1 << FBY) x (1 << FBX) PW-bit pixels with NRD independent read ports
// and NWR independent write ports, so that pipelined stages (reconstruction, deblocking, CDEF, super-resolution,
// loop restoration) access the picture concurrently; plus a host read port for the testbench.
//   read port i:  re[i] with (r_plane, r_x, r_y) -> rdata[i] (the sample at x) and rdata4[i] (the aligned group
//                 of 4 samples containing x: lane l = sample at {x[FBX-1:2], l}), registered, 1-cycle latency
//   write port i: we[i] with w_be[i] lane enables writes lane l of w_data[i] to {w_x[FBX-1:2], l}; a single-sample
//                 write is one enabled lane (the frame_mem_w1 helper below builds be/data from x and a sample)
// Ports are packed vectors (port i at [i*W +: W]). Two write ports never target the same sample in one cycle
// by construction of the stage schedule (rows in flight are disjoint).
// A real design puts this in DRAM behind a cache with the picture banked by x mod 4; this module is the
// sim/FPGA-BRAM stand-in.
module frame_mem_sim #(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int PW = 12,
    parameter int NRD = 1,
    parameter int NWR = 1
) (
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
    output logic [PW-1:0]       h_rdata,
    output logic [4*PW-1:0]     h_rdata4,        // the four samples of h_x's aligned group (x & ~3 .. +3), lane 0 lowest
    output logic                h_ok             // always: the host port is independent here
);
    assign h_ok = 1'b1;
    localparam int N = 1 << (FBX + FBY);
    // gate-replay instrumentation (atium copy only): +fminit=zero|rand initialises the arrays (xsim starts them X;
    // the Verilator build starts them 0); X writes are reported (first 20) with $time so the source stage can be found.
    string fminit; int xwrites = 0;
    initial begin
        if (!$value$plusargs("fminit=%s", fminit)) fminit = "none";
        if (fminit == "zero") for (int k = 0; k < N; k++) begin mem0[k] = '0; mem1[k] = '0; mem2[k] = '0; end
        else if (fminit == "rand") for (int k = 0; k < N; k++) begin mem0[k] = $urandom; mem1[k] = $urandom; mem2[k] = $urandom; end
        $display("frame_mem %m: init=%s N=%0d", fminit, N);
    end
    logic [PW-1:0] mem0 [0:N-1];
    logic [PW-1:0] mem1 [0:N-1];
    logic [PW-1:0] mem2 [0:N-1];

    always_ff @(posedge clk) begin
        for (int i = 0; i < NWR; i++) begin
            if (we[i]) begin
                for (int l = 0; l < 4; l++) begin
                    if (w_be[i*4 + l]) begin
                        logic [FBX+FBY-1:0] a;
                        a = {w_y[i*FBY +: FBY], w_x[i*FBX + 2 +: FBX-2], 2'(l)};
                        if ($isunknown(w_data[(i*4 + l)*PW +: PW]) || $isunknown(a) || $isunknown(w_plane[i*2 +: 2])) begin
                            xwrites++;
                            if (xwrites <= 20) $display("XWRITE %m t=%0t port=%0d lane=%0d plane=%b x=%0d y=%0d data=%b", $time, i, l, w_plane[i*2 +: 2], w_x[i*FBX +: FBX], w_y[i*FBY +: FBY], w_data[(i*4 + l)*PW +: PW]);
                        end
                        case (w_plane[i*2 +: 2])
                            2'd0: mem0[a] <= w_data[(i*4 + l)*PW +: PW];
                            2'd1: mem1[a] <= w_data[(i*4 + l)*PW +: PW];
                            default: mem2[a] <= w_data[(i*4 + l)*PW +: PW];
                        endcase
                    end
                end
            end
        end
        for (int i = 0; i < NRD; i++) begin
            if (re[i]) begin
                logic [FBX+FBY-1:0] a;
                a = {r_y[i*FBY +: FBY], r_x[i*FBX +: FBX]};
                case (r_plane[i*2 +: 2])
                    2'd0: rdata[i*PW +: PW] <= mem0[a];
                    2'd1: rdata[i*PW +: PW] <= mem1[a];
                    default: rdata[i*PW +: PW] <= mem2[a];
                endcase
                for (int l = 0; l < 4; l++) begin
                    logic [FBX+FBY-1:0] g;
                    g = {r_y[i*FBY +: FBY], r_x[i*FBX + 2 +: FBX-2], 2'(l)};
                    case (r_plane[i*2 +: 2])
                        2'd0: rdata4[(i*4 + l)*PW +: PW] <= mem0[g];
                        2'd1: rdata4[(i*4 + l)*PW +: PW] <= mem1[g];
                        default: rdata4[(i*4 + l)*PW +: PW] <= mem2[g];
                    endcase
                end
            end
        end
        case (h_plane)
            2'd0: h_rdata <= mem0[{h_y, h_x}];
            2'd1: h_rdata <= mem1[{h_y, h_x}];
            default: h_rdata <= mem2[{h_y, h_x}];
        endcase
        for (int l = 0; l < 4; l++) begin
            logic [FBX+FBY-1:0] hg;
            hg = {h_y, h_x[FBX-1:2], 2'(l)};
            case (h_plane)
                2'd0: h_rdata4[l*PW +: PW] <= mem0[hg];
                2'd1: h_rdata4[l*PW +: PW] <= mem1[hg];
                default: h_rdata4[l*PW +: PW] <= mem2[hg];
            endcase
        end
    end
endmodule
