// On-chip frame memory for the FPGA builds (docs/frame-memory.md): the same ports as the frame_mem simulation
// model for NRD = NWR = 2 (the core build's fb0), one UltraRAM copy of the picture, double-pumped. The RAM runs on
// clk2x (2x clk, phase-aligned from the same MMCM): on the mid-cycle clk2x edge its two ports read for the two
// read ports (or the host), on the clk-aligned edge they write for the two write ports. Every decoder-side port
// keeps its one-cycle latency at clk; nothing arbitrates or stalls.
//
// Word = one aligned group of four samples, 4 x 18-bit lanes = 72 bits (a sample in the low 12 bits of its lane;
// the lane is two URAM bytes, so the lane write enables are byte-write enables). Address = plane base +
// y * (W/4) + x/4 with W = 1 << FBX: plane 0 holds (1 << FBY) rows, planes 1 and 2 (4:2:0) a quarter each.
module frame_mem_uram #(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int PW = 12
) (
    input  logic                clk,
    input  logic                clk2x,
    input  logic                phase,      // 1 at the mid-cycle clk2x edge, 0 at the clk-aligned one (clock generator)
    // read ports (2)
    input  logic [1:0]          re,
    input  logic [3:0]          r_plane,
    input  logic [2*FBX-1:0]    r_x,
    input  logic [2*FBY-1:0]    r_y,
    output logic [2*PW-1:0]     rdata,
    output logic [2*4*PW-1:0]   rdata4,
    // write ports (2)
    input  logic [1:0]          we,
    input  logic [7:0]          w_be,
    input  logic [3:0]          w_plane,
    input  logic [2*FBX-1:0]    w_x,
    input  logic [2*FBY-1:0]    w_y,
    input  logic [2*4*PW-1:0]   w_data,
    // host read port (served in read-port 1's slot when that port is idle, i.e. whenever the decoder is idle)
    input  logic [1:0]          h_plane,
    input  logic [FBX-1:0]      h_x,
    input  logic [FBY-1:0]      h_y,
    output logic [PW-1:0]       h_rdata
);
    localparam int WPR   = 1 << (FBX - 2);                   // words per luma row
    localparam int NW0   = WPR << FBY;                       // luma words
    localparam int NW1   = NW0 >> 2;                         // chroma plane words (4:2:0)
    localparam int NWORD = NW0 + 2 * NW1;
    localparam int AW    = $clog2(NWORD);

    function automatic logic [AW-1:0] waddr(input logic [1:0] p, input logic [FBX-1:0] x, input logic [FBY-1:0] y);
        logic [AW-1:0] a;
        case (p)
            2'd0:    a = AW'(y) * AW'(WPR) + AW'(x >> 2);
            2'd1:    a = AW'(NW0) + AW'(y) * AW'(WPR >> 1) + AW'(x >> 2);
            default: a = AW'(NW0 + NW1) + AW'(y) * AW'(WPR >> 1) + AW'(x >> 2);
        endcase
        return a;
    endfunction

    // ---------------------------------------------------------------- the RAM (clk2x domain)
    // Slot schedule per clk cycle (clk2x edges E_a mid-cycle, E_b aligned with the next clk edge):
    //   E_a: port A reads read-port 0, port B reads read-port 1 (or the host when read-port 1 is idle); the
    //        write requests are captured into clk2x registers.
    //   E_b: port A writes write-port 0, port B writes write-port 1 (from the captured requests).
    // So a request issued at a clk edge has half a cycle to reach the RAM, read data has half a cycle to reach
    // the clk-domain capture at the next clk edge (one-cycle latency, as the model), a write issued in cycle t
    // is visible to reads from cycle t+1 on, and a read in the same cycle as a write to the same word sees the
    // old data: exactly the model's semantics. phase = 1 at E_a, 0 at E_b (from the clock generator).
    (* ram_style = "ultra" *) logic [71:0] mem [0:NWORD-1];
    logic           a_en, b_en;
    logic           a_we, b_we;
    logic [7:0]     a_bwe, b_bwe;                            // byte enables, two per lane
    logic [AW-1:0]  a_addr, b_addr;
    logic [71:0]    a_d, b_d;
    logic [71:0]    a_q, b_q;
    always_ff @(posedge clk2x) begin
        if (a_en) begin
            if (a_we) begin for (int l = 0; l < 4; l++) if (a_bwe[2*l]) mem[a_addr][l*18 +: 18] <= a_d[l*18 +: 18]; end
            else a_q <= mem[a_addr];
        end
        if (b_en) begin
            if (b_we) begin for (int l = 0; l < 4; l++) if (b_bwe[2*l]) mem[b_addr][l*18 +: 18] <= b_d[l*18 +: 18]; end
            else b_q <= mem[b_addr];
        end
    end

    // ---------------------------------------------------------------- requests
    logic [FBX-1:0] rx0, rx1, wx0, wx1;
    logic [FBY-1:0] ry0, ry1, wy0, wy1;
    assign {rx1, rx0} = r_x; assign {ry1, ry0} = r_y;
    assign {wx1, wx0} = w_x; assign {wy1, wy0} = w_y;
    logic host_slot;                                         // the host read takes read-port 1's slot when idle
    assign host_slot = !re[1];
    // write requests captured at E_a, applied at E_b
    logic           wq_we [2];
    logic [7:0]     wq_bwe [2];
    logic [AW-1:0]  wq_addr [2];
    logic [71:0]    wq_d [2];
    always_ff @(posedge clk2x) if (phase) begin
        wq_we[0] <= we[0]; wq_we[1] <= we[1];
        wq_addr[0] <= waddr(w_plane[1:0], wx0, wy0);
        wq_addr[1] <= waddr(w_plane[3:2], wx1, wy1);
        for (int l = 0; l < 4; l++) begin
            wq_d[0][l*18 +: 18] <= 18'(w_data[l*PW +: PW]);       wq_bwe[0][2*l +: 2] <= {2{w_be[l]}};
            wq_d[1][l*18 +: 18] <= 18'(w_data[(4 + l)*PW +: PW]); wq_bwe[1][2*l +: 2] <= {2{w_be[4 + l]}};
        end
    end
    always_comb begin
        if (phase) begin                                     // E_a: reads
            a_en = re[0];               a_we = 1'b0; a_addr = waddr(r_plane[1:0], rx0, ry0); a_d = '0; a_bwe = '0;
            b_en = re[1] || host_slot;  b_we = 1'b0; a_d = '0; b_d = '0; b_bwe = '0;
            b_addr = host_slot ? waddr(h_plane, h_x, h_y) : waddr(r_plane[3:2], rx1, ry1);
        end else begin                                       // E_b: writes
            a_en = wq_we[0]; a_we = 1'b1; a_addr = wq_addr[0]; a_d = wq_d[0]; a_bwe = wq_bwe[0];
            b_en = wq_we[1]; b_we = 1'b1; b_addr = wq_addr[1]; b_d = wq_d[1]; b_bwe = wq_bwe[1];
        end
    end

    // ---------------------------------------------------------------- results back into the clk domain
    // a_q / b_q are updated at E_a and hold through E_b; the clk edge coincident with E_b captures them.
    logic [71:0] q0, q1, hq;
    logic [1:0]  rx0l, rx1l, hxl;
    always_ff @(posedge clk) begin
        q0 <= a_q; q1 <= b_q; hq <= b_q;
        rx0l <= rx0[1:0]; rx1l <= rx1[1:0]; hxl <= h_x[1:0];
    end
    always_comb begin
        rdata4[0*4*PW +: 4*PW] = {q0[3*18 +: PW], q0[2*18 +: PW], q0[1*18 +: PW], q0[0*18 +: PW]};
        rdata4[1*4*PW +: 4*PW] = {q1[3*18 +: PW], q1[2*18 +: PW], q1[1*18 +: PW], q1[0*18 +: PW]};
        rdata[0*PW +: PW] = q0[32'(rx0l)*18 +: PW];
        rdata[1*PW +: PW] = q1[32'(rx1l)*18 +: PW];
        h_rdata = hq[32'(hxl)*18 +: PW];
    end
endmodule
