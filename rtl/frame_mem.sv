// Frame buffer model: three planes of (1 << FBY) x (1 << FBX) PW-bit pixels, one read/write port for the
// reconstruction stage (registered read, 1-cycle latency) plus an independent read port for the host/test.
// A real design puts this in external DRAM behind a cache; this module is the sim/FPGA-BRAM stand-in.
module frame_mem #(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int PW = 12
) (
    input  logic          clk,
    // decoder port
    input  logic          re,
    input  logic          we,
    input  logic [1:0]    plane,
    input  logic [FBX-1:0] x,
    input  logic [FBY-1:0] y,
    input  logic [PW-1:0] wdata,
    output logic [PW-1:0] rdata,
    // host read port
    input  logic [1:0]    h_plane,
    input  logic [FBX-1:0] h_x,
    input  logic [FBY-1:0] h_y,
    output logic [PW-1:0] h_rdata
);
    localparam int N = 1 << (FBX + FBY);
    logic [PW-1:0] mem0 [0:N-1];
    logic [PW-1:0] mem1 [0:N-1];
    logic [PW-1:0] mem2 [0:N-1];

    always_ff @(posedge clk) begin
        if (we) begin
            case (plane)
                2'd0: mem0[{y, x}] <= wdata;
                2'd1: mem1[{y, x}] <= wdata;
                default: mem2[{y, x}] <= wdata;
            endcase
        end
        if (re) begin
            case (plane)
                2'd0: rdata <= mem0[{y, x}];
                2'd1: rdata <= mem1[{y, x}];
                default: rdata <= mem2[{y, x}];
            endcase
        end
        case (h_plane)
            2'd0: h_rdata <= mem0[{h_y, h_x}];
            2'd1: h_rdata <= mem1[{h_y, h_x}];
            default: h_rdata <= mem2[{h_y, h_x}];
        endcase
    end
endmodule
