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
    output logic [4*PW-1:0] rdata4,    // the aligned group of 4 samples containing x (lane l = x[FBX-1:2]*4 + l)
    // second write port (reconstruction writes while it reads through the first port)
    input  logic          w2_we,
    input  logic [1:0]    w2_plane,
    input  logic [FBX-1:0] w2_x,
    input  logic [FBY-1:0] w2_y,
    input  logic [PW-1:0] w2_wdata,
    // 4-wide write port (reconstruction: prediction and residual-add output, aligned groups of 4 samples)
    input  logic          w4_we,
    input  logic [1:0]    w4_plane,
    input  logic [FBX-1:0] w4_x,
    input  logic [FBY-1:0] w4_y,
    input  logic [4*PW-1:0] w4_wdata,
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
        if (w2_we) begin
            case (w2_plane)
                2'd0: mem0[{w2_y, w2_x}] <= w2_wdata;
                2'd1: mem1[{w2_y, w2_x}] <= w2_wdata;
                default: mem2[{w2_y, w2_x}] <= w2_wdata;
            endcase
        end
        if (w4_we) begin
            for (int l = 0; l < 4; l++) begin
                case (w4_plane)
                    2'd0: mem0[{w4_y, w4_x[FBX-1:2], 2'(l)}] <= w4_wdata[l*PW +: PW];
                    2'd1: mem1[{w4_y, w4_x[FBX-1:2], 2'(l)}] <= w4_wdata[l*PW +: PW];
                    default: mem2[{w4_y, w4_x[FBX-1:2], 2'(l)}] <= w4_wdata[l*PW +: PW];
                endcase
            end
        end
        if (re) begin
            case (plane)
                2'd0: rdata <= mem0[{y, x}];
                2'd1: rdata <= mem1[{y, x}];
                default: rdata <= mem2[{y, x}];
            endcase
            for (int l = 0; l < 4; l++) begin
                case (plane)
                    2'd0: rdata4[l*PW +: PW] <= mem0[{y, x[FBX-1:2], 2'(l)}];
                    2'd1: rdata4[l*PW +: PW] <= mem1[{y, x[FBX-1:2], 2'(l)}];
                    default: rdata4[l*PW +: PW] <= mem2[{y, x[FBX-1:2], 2'(l)}];
                endcase
            end
        end
        case (h_plane)
            2'd0: h_rdata <= mem0[{h_y, h_x}];
            2'd1: h_rdata <= mem1[{h_y, h_x}];
            default: h_rdata <= mem2[{h_y, h_x}];
        endcase
    end
endmodule
