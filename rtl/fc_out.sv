// The streaming filter chain's way into a picture memory (docs/filter-chain.md): takes the three planes' final line
// segments from fc_top and writes them through two frame-memory write ports, an aligned group of four samples each
// (frame_mem_uram's shape), so eight samples a clock in all. A luma segment is two groups, a segment of a
// horizontally subsampled chroma plane one.
// Each plane has a FIFO of segments; out_hold rises while a FIFO has fewer than HOLD_AT free entries, which leaves
// room for the lines the chain still sends after a hold (fc_plane's OUT_SLACK). The two ports are dealt out plane
// after plane starting from a plane that moves on every clock, so no plane waits behind another for long; a
// two-group segment may be split over two clocks.
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
    assign busy = |f_v;
    // deal the two ports
    logic [2:0] half_n;
    always_comb begin
        int left;
        left = 2;
        we = '0; wplane = '0; wx = '0; wy = '0; wdata = '0; f_pop = '0; half_n = half;
        for (int k = 0; k < 3; k++) begin
            int p;
            p = (int'(rr) + k) % 3;
            if (f_v[p] && left > 0) begin
                // the groups of the head segment still to write: the second alone, or the first (and the second)
                for (int g = 0; g < 2; g++) begin
                    if (left > 0 && ((g == 0 && !half[p]) || (g == 1 && two[p]))) begin
                        we[2 - left] = 1'b1;
                        wplane[2 - left] = 2'(p);
                        wx[2 - left] = FBX'({f_q[p][96 + FBY +: CW], 2'b00}) + FBX'(4 * g);
                        wy[2 - left] = f_q[p][96 +: FBY];
                        wdata[2 - left] = f_q[p][48 * g +: 48];
                        left = left - 1;
                        if (g == 1 || !two[p]) begin f_pop[p] = 1'b1; half_n[p] = 1'b0; end
                        else half_n[p] = 1'b1;
                    end
                end
            end
        end
    end
    always_ff @(posedge clk) begin
        if (rst) begin half <= '0; rr <= 2'd0; err_ovf <= 1'b0; end
        else begin
            half <= half_n;
            rr <= (rr == 2'd2) ? 2'd0 : rr + 2'd1;
            if (|(in_valid & ~f_in_ready)) err_ovf <= 1'b1;
        end
    end
endmodule
