// Per-4x4 frame state for the in-loop filters (spec arrays MiSizes, Skips, SegmentIds, DeltaLFs and
// LoopfilterTxSizes[plane]). Two independent memories:
//   lf_info[mi_row][mi_col]         : mi_lf_t, written per block (blk_we iterates the block's bh4 x bw4 area)
//   lf_tx[plane][row_p][col_p]      : transform size, written per transform block (tx_we iterates h4 x w4)
// Both writers run as small FSMs with busy flags; reads are registered (1-cycle latency).
// Storage is written so that synthesis maps it to block RAM: see the notes at each array.
// Capacity: (1 << ML2R) x (1 << ML2C) 4x4 units per plane array.
module mi_store
  import syn_pkg::*;
#(
    parameter int ML2R = 8,
    parameter int ML2C = 8
) (
    input  logic        clk,
    input  logic        rst,
    // block write: the block at (r, c), bw4 x bh4 units
    input  logic        blk_we,
    input  logic [MIW-1:0] blk_r, blk_c,
    input  logic [5:0]  blk_bw4, blk_bh4,
    input  mi_lf_t      blk_data,
    output logic        blk_busy,
    // transform-block write: plane, position and size in 4x4 units of the plane
    input  logic        tx_we,
    input  logic [1:0]  tx_plane,
    input  logic [MIW-1:0] tx_row, tx_col,
    input  logic [4:0]  tx_w4, tx_h4,
    input  logic [4:0]  tx_sz,
    output logic        tx_busy,
    // reads (registered)
    input  logic [MIW-1:0] rd_row, rd_col,
    output mi_lf_t      rd_info,
    input  logic [MIW-1:0] rd2_row, rd2_col,     // second read port (CDEF, concurrent with the deblocking filter)
    output mi_lf_t      rd2_info,
    input  logic [1:0]  txr_plane,
    input  logic [MIW-1:0] txr_row, txr_col,
    output logic [4:0]  txr_sz,
    // cdef_idx per 64x64 unit: cleared per superblock (cd_clr: 1 or 4 units from the superblock's top-left
    // unit), set per block (cd_we: units of the mask, relative to the superblock's top-left unit), read (registered)
    input  logic        cd_clr,
    input  logic        cd_we,
    input  logic [SBW-1:0]  cd_row64, cd_col64,   // superblock top-left unit
    input  logic        cd_sb128,
    input  logic [3:0]  cd_mask,              // units (i*2+j) covered; ignored (unit 0) when !cd_sb128
    input  logic [2:0]  cd_idx,
    input  logic [SBW-1:0]  cdr_row64, cdr_col64,
    output logic [3:0]  cdr_val,              // {valid, idx}
    // loop-restoration unit records (from tile_syntax lr_done / lr_rec); read registered
    input  logic        lr_we,
    input  lr_rec_t     lr_rec,
    input  logic [1:0]  lrr_plane,
    input  logic [SBW-1:0] lrr_row, lrr_col,
    output lr_rec_t     lrr_rec
);
    localparam int N = 1 << (ML2R + ML2C);
    // lf_info is read by two stages at once (deblocking and CDEF), so it is kept as two identically written
    // copies, each a simple dual-port RAM (one write, one synchronous read): that is the shape block RAM has.
    // Flat vector arrays rather than `mi_lf_t [...]` for the same reason (synthesis infers RAM from vectors only).
    // Each store is one always_ff holding only the write and the registered read of a flat vector array, with
    // plain address/data signals and the struct cast outside: the shape synthesis infers RAM from.
    localparam int IW = $bits(mi_lf_t), LW = $bits(lr_rec_t);
    (* ram_style = "ultra" *) logic [IW-1:0] lf_info_a [0:N-1];
    (* ram_style = "ultra" *) logic [IW-1:0] lf_info_b [0:N-1];
    logic          i_we; logic [ML2R+ML2C-1:0] i_waddr, i_raddr_a, i_raddr_b; logic [IW-1:0] i_wdata, i_rdata_a, i_rdata_b;
    always_ff @(posedge clk) begin
        if (i_we) lf_info_a[i_waddr] <= i_wdata;
        i_rdata_a <= lf_info_a[i_raddr_a];
    end
    always_ff @(posedge clk) begin
        if (i_we) lf_info_b[i_waddr] <= i_wdata;
        i_rdata_b <= lf_info_b[i_raddr_b];
    end
    assign rd_info = mi_lf_t'(i_rdata_a);
    assign rd2_info = mi_lf_t'(i_rdata_b);
    // declared ahead of their first use: Vivado makes an implicit 1-bit net of an identifier used before its declaration
    (* ram_style = "block" *) logic [4:0] lf_tx0 [0:N-1];
    (* ram_style = "block" *) logic [4:0] lf_tx1 [0:N-1];
    (* ram_style = "block" *) logic [4:0] lf_tx2 [0:N-1];
    logic t_we0, t_we1, t_we2; logic [ML2R+ML2C-1:0] t_waddr, t_raddr; logic [4:0] t_wdata, t_rdata0, t_rdata1, t_rdata2; logic [1:0] txr_plane_q;
    always_ff @(posedge clk) begin
        if (t_we0) lf_tx0[t_waddr] <= t_wdata;
        t_rdata0 <= lf_tx0[t_raddr];
    end
    always_ff @(posedge clk) begin
        if (t_we1) lf_tx1[t_waddr] <= t_wdata;
        t_rdata1 <= lf_tx1[t_raddr];
    end
    always_ff @(posedge clk) begin
        if (t_we2) lf_tx2[t_waddr] <= t_wdata;
        t_rdata2 <= lf_tx2[t_raddr];
    end
    always_ff @(posedge clk) txr_plane_q <= txr_plane;
    assign txr_sz = (txr_plane_q == 2'd0) ? t_rdata0 : (txr_plane_q == 2'd1) ? t_rdata1 : t_rdata2;

    function automatic logic [ML2R+ML2C-1:0] idx(input logic [MIW-1:0] row, input logic [MIW-1:0] col);
        idx = {ML2R'(row), ML2C'(col)};
    endfunction

    // ---- block writer
    logic [5:0]  bi, bj;
    logic [MIW-1:0] b_r, b_c;
    logic [5:0]  b_w, b_h;
    mi_lf_t      b_d;
    always_ff @(posedge clk) begin
        if (rst) blk_busy <= 1'b0;
        else if (!blk_busy) begin
            if (blk_we) begin
                blk_busy <= 1'b1; b_r <= blk_r; b_c <= blk_c; b_w <= blk_bw4; b_h <= blk_bh4; b_d <= blk_data; bi <= 6'd0; bj <= 6'd0;
            end
        end else begin
            if (bj + 6'd1 < b_w) bj <= bj + 6'd1;
            else begin
                bj <= 6'd0;
                if (bi + 6'd1 < b_h) bi <= bi + 6'd1;
                else blk_busy <= 1'b0;
            end
        end
    end

    // ---- transform-block writer
    logic [4:0]  ti, tj;
    logic [1:0]  t_p;
    logic [MIW-1:0] t_r, t_c;
    logic [4:0]  t_w, t_h, t_sz;
    always_ff @(posedge clk) begin
        if (rst) tx_busy <= 1'b0;
        else if (!tx_busy) begin
            if (tx_we) begin
                tx_busy <= 1'b1; t_p <= tx_plane; t_r <= tx_row; t_c <= tx_col; t_w <= tx_w4; t_h <= tx_h4; t_sz <= tx_sz; ti <= 5'd0; tj <= 5'd0;
            end
        end else begin
            if (tj + 5'd1 < t_w) tj <= tj + 5'd1;
            else begin
                tj <= 5'd0;
                if (ti + 5'd1 < t_h) ti <= ti + 5'd1;
                else tx_busy <= 1'b0;
            end
        end
    end

    // ---- cdef_idx per 64x64 unit (flops: up to 4 units written per cycle)
    localparam int NU = 1 << (ML2R - 4 + ML2C - 4);
    logic [3:0] cdef_arr [0:NU-1];
    function automatic logic [ML2R+ML2C-9:0] uidx(input logic [SBW-1:0] r64, input logic [SBW-1:0] c64);
        uidx = {(ML2R - 4)'(r64), (ML2C - 4)'(c64)};
    endfunction
    // The write is pipelined: cycle 1 registers the request (the four unit addresses, which lanes write, the value), cycle 2
    // decodes and writes. The header's sb128 fan-out, the +1 adders and the 4-lane x 2^(ML2R+ML2C-8) enable decode in one
    // cycle were 35 logic levels (-4.5 ns at 100 MHz, U200 inter build 2026-10-24). CDEF reads a unit a superblock row
    // after the block that set it, so the extra cycle is invisible to the reader.
    logic              cdq_v, cdq_clr;
    logic [3:0]        cdq_lane;                                 // lanes (i*2+j) that write
    logic [3:0]        cdq_val;
    logic [ML2R+ML2C-9:0] cdq_u [4];
    always_ff @(posedge clk) begin
        cdq_v <= cd_clr || cd_we; cdq_clr <= cd_clr; cdq_val <= {1'b1, cd_idx};
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) begin
                cdq_u[i * 2 + j] <= uidx(cd_row64 + SBW'(i), cd_col64 + SBW'(j));
                cdq_lane[i * 2 + j] <= ((i == 0 && j == 0) || cd_sb128) && (cd_clr || !cd_sb128 || cd_mask[i * 2 + j]);
            end
        if (cdq_v)
            for (int l = 0; l < 4; l++)
                if (cdq_lane[l]) cdef_arr[cdq_u[l]] <= cdq_clr ? 4'd0 : cdq_val;
        cdr_val <= cdef_arr[uidx(cdr_row64, cdr_col64)];
    end

    // ---- loop-restoration units: [plane][unitRow][unitCol], 2^(ML2R-4) x 2^(ML2C-4) units per plane (64-px units over the per-4x4 capacity)
    // units are at least 64 px, so the per-4x4 capacity (ML2R/ML2C) bounds the unit grid at 2^(ML2-4) per axis
    localparam int LUR = ML2R - 4, LUC = ML2C - 4;
    logic [LW-1:0] lr_units [0:3 * (1 << (LUR + LUC)) - 1];
    logic [LUR+LUC+1:0] u_waddr, u_raddr; logic [LW-1:0] u_wdata, u_rdata;
    assign u_waddr = {lr_rec.plane, LUR'(lr_rec.unit_row), LUC'(lr_rec.unit_col)};
    assign u_wdata = lr_rec;
    assign u_raddr = {lrr_plane, LUR'(lrr_row), LUC'(lrr_col)};
    always_ff @(posedge clk) begin
        if (lr_we) lr_units[u_waddr] <= u_wdata;
        u_rdata <= lr_units[u_raddr];
    end
    assign lrr_rec = lr_rec_t'(u_rdata);

    // ---- store ports: the writers store one entry per cycle while busy (their FSMs step the indices)
    assign i_we = blk_busy && !rst;
    assign i_waddr = idx(b_r + MIW'(bi), b_c + MIW'(bj));
    assign i_wdata = b_d;
    assign i_raddr_a = idx(rd_row, rd_col);
    assign i_raddr_b = idx(rd2_row, rd2_col);
    assign t_we0 = tx_busy && !rst && (t_p == 2'd0);
    assign t_we1 = tx_busy && !rst && (t_p == 2'd1);
    assign t_we2 = tx_busy && !rst && (t_p == 2'd2 || t_p == 2'd3);
    assign t_waddr = idx(t_r + MIW'(ti), t_c + MIW'(tj));
    assign t_wdata = t_sz;
    assign t_raddr = idx(txr_row, txr_col);
endmodule
