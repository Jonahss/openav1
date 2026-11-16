// The streaming filter chain's way into a picture memory (docs/filter-chain.md): takes the three planes' final line
// segments from fc_top and writes them through two frame-memory write ports, an aligned group of four samples each
// (frame_mem_uram's shape), so eight samples a clock in all. A luma segment is two groups, a segment of a
// horizontally subsampled chroma plane one.
// Each plane has a FIFO of segments; out_hold rises while a FIFO has fewer than HOLD_AT free entries, which leaves
// room for the lines the chain still sends after a hold (fc_plane's OUT_SLACK). The two ports are dealt out plane
// after plane starting from a plane that moves on every clock, so no plane waits behind another for long; a
// two-group segment may be split over two clocks. The write ports are registers (one clock after the deal).
// A segment's line is counted from the band's first line in its plane (signed: the chain's output for a band starts
// above it); y0 is that line in the picture, per plane, and must hold from the band's start until the band's last
// segment has been taken in (not until it is written: the FIFO keeps the picture line).
module fc_out #(
    parameter int FBX = 11,
    parameter int FBY = 11,
    parameter int CW = 10,
    parameter int FAW = 6,                   // log2 of a plane's FIFO depth
    parameter int HOLD_AT = 24
) (
    input  logic                    clk,
    input  logic                    rst,
    input  logic [2:0][FBY-1:0]     y0,      // the band's first line in each plane
    input  logic [2:0]              in_valid,
    input  logic [2:0][95:0]        in_line,
    input  logic [2:0][1:0]         in_n4,   // groups of four in the segment: 1 or 2
    input  logic [2:0][CW-1:0]      in_col,  // the segment's first 4x4 column
    input  logic signed [2:0][8:0]  in_y,
    output logic [2:0]              out_hold,
    output logic                    busy,    // a segment is waiting to be written
    output logic                    err_ovf, // a segment found its FIFO full (never, if the hold's arithmetic holds); stays set
    // two write ports: an aligned group of four samples of a plane at (x, y)
    output logic [1:0]              we,
    output logic [1:0][1:0]         wplane,
    output logic [1:0][FBX-1:0]     wx,
    output logic [1:0][FBY-1:0]     wy,
    output logic [1:0][47:0]        wdata
);
    localparam int W = 96 + 1 + CW + FBY;
    logic [2:0]            f_v, f_pop, f_in_ready;
    logic [2:0][W-1:0]     f_q;
    logic [2:0][FAW:0]     f_free;
    logic [2:0]            two;              // the head segment has two groups
    logic [2:0]            half;             // its first group is written already
    logic [1:0]            rr;               // the plane served first
    generate for (genvar p = 0; p < 3; p++) begin : g_f
        logic [FBY-1:0] ya;
        assign ya = y0[p] + FBY'($signed(in_y[p]));   // a slice of a packed array is unsigned: the line's sign is restored here
        fc_lfifo #(.W(W), .AW(FAW)) u_f (.clk, .rst, .in_valid(in_valid[p]), .in_ready(f_in_ready[p]),
            .in_data({in_n4[p] == 2'd2, in_col[p], ya, in_line[p]}),
            .out_valid(f_v[p]), .out_ready(f_pop[p]), .out_data(f_q[p]), .free(f_free[p]));
        assign out_hold[p] = f_free[p] < (FAW+1)'(HOLD_AT);
        assign two[p] = f_q[p][W-1];
    end endgenerate
    // The two ports are dealt out and then REGISTERED (2026-11-16). Until then they left this module as logic behind the
    // FIFOs' block-RAM outputs: the deal, the group's column adder, dec_top's mux and the frame memory's address
    // arithmetic in one clock, 25 logic levels into the memory's request registers. Once the clk -> clk2x hold check
    // was anchored correctly (fpga/u200/sys.xdc) that path missed setup at 100 and at 90 MHz. The deal is written out
    // as the few cases it has, instead of a loop counting the ports left: the FIFOs' pop (their read enable) hangs
    // on it.
    //   port 0: the first plane, in this clock's order, with a segment: its next group
    //   port 1: that segment's second group if it still has both; else the next plane's next group
    // A plane served on port 0 always finishes its segment in this clock; a plane served on port 1 finishes unless
    // its segment has both groups left (then the second waits: half).
    logic [2:0][FBX-1:0]   gx0, gx1;         // the head segment's two group columns
    logic [2:0][FBY-1:0]   gy;
    logic [2:0]            both;             // the head segment has both groups still to write
    generate for (genvar p = 0; p < 3; p++) begin : g_h
        assign gx0[p] = FBX'({f_q[p][96 + FBY +: CW], 2'b00});
        assign gx1[p] = FBX'({f_q[p][96 + FBY +: CW], 2'b00}) + FBX'(4);
        assign gy[p]  = f_q[p][96 +: FBY];
        assign both[p] = f_v[p] && two[p] && !half[p];
    end endgenerate
    logic [1:0]            o0, o1, o2;       // the planes in this clock's order
    logic [1:0]            pa, pb;           // the first plane with a segment, and the one after it with a segment
    logic                  va, vb;
    logic [1:0]            n_we;
    logic [1:0][1:0]       n_wplane;
    logic [1:0][FBX-1:0]   n_wx;
    logic [1:0][FBY-1:0]   n_wy;
    logic [1:0][47:0]      n_wdata;
    logic [2:0]            half_n;
    always_comb begin
        case (rr)
            2'd0:    begin o0 = 2'd0; o1 = 2'd1; o2 = 2'd2; end
            2'd1:    begin o0 = 2'd1; o1 = 2'd2; o2 = 2'd0; end
            default: begin o0 = 2'd2; o1 = 2'd0; o2 = 2'd1; end
        endcase
        va = 1'b1; vb = 1'b1; pa = o0; pb = o1;
        if (f_v[o0])      begin pa = o0; if (f_v[o1]) pb = o1; else if (f_v[o2]) pb = o2; else vb = 1'b0; end
        else if (f_v[o1]) begin pa = o1; if (f_v[o2]) pb = o2; else vb = 1'b0; end
        else if (f_v[o2]) begin pa = o2; vb = 1'b0; end
        else              begin va = 1'b0; vb = 1'b0; end
        n_we = '0; n_wplane = '0; n_wx = '0; n_wy = '0; n_wdata = '0; f_pop = '0; half_n = half;
        if (va) begin
            // port 0: plane pa's next group (its second if the first is written already)
            n_we[0] = 1'b1; n_wplane[0] = pa; n_wy[0] = gy[pa];
            n_wx[0] = half[pa] ? gx1[pa] : gx0[pa];
            n_wdata[0] = half[pa] ? f_q[pa][48 +: 48] : f_q[pa][0 +: 48];
            f_pop[pa] = 1'b1; half_n[pa] = 1'b0;
            if (both[pa]) begin
                // port 1: the same segment's second group
                n_we[1] = 1'b1; n_wplane[1] = pa; n_wy[1] = gy[pa]; n_wx[1] = gx1[pa]; n_wdata[1] = f_q[pa][48 +: 48];
            end else if (vb) begin
                // port 1: the next plane's next group; a segment with both groups left keeps its second
                n_we[1] = 1'b1; n_wplane[1] = pb; n_wy[1] = gy[pb];
                n_wx[1] = half[pb] ? gx1[pb] : gx0[pb];
                n_wdata[1] = half[pb] ? f_q[pb][48 +: 48] : f_q[pb][0 +: 48];
                if (both[pb]) half_n[pb] = 1'b1;
                else begin f_pop[pb] = 1'b1; half_n[pb] = 1'b0; end
            end
        end
    end
    assign busy = (|f_v) || (|we);           // a segment waits in a FIFO, or a group is on the ports in this clock
    always_ff @(posedge clk) begin
        wplane <= n_wplane; wx <= n_wx; wy <= n_wy; wdata <= n_wdata;
        if (rst) begin half <= '0; rr <= 2'd0; err_ovf <= 1'b0; we <= '0; end
        else begin
            we <= n_we;
            half <= half_n;
            rr <= (rr == 2'd2) ? 2'd0 : rr + 2'd1;
            if (|(in_valid & ~f_in_ready)) err_ovf <= 1'b1;
        end
    end
endmodule
