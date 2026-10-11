// Motion information of the frame being decoded, per 4x4 luma unit (spec IsInters / RefFrames / Mvs / MiSizes
// as the MV stack needs them): written per block by the syntax stage as soon as the block's mode info is
// complete (area fill, `busy` while in flight), read one position per cycle by mv_stack (registered read).
// A 64-row bitmap of "written" bits (per 4x4 column) says whether a position has been written in this
// frame; the tile decoder clears the bitmap rows of a superblock row when it starts one (the MV stack looks
// at most one superblock row up), so no state depends on memory initialisation.
module mv_mem
  import syn_pkg::*;
#(
    parameter int ML2R = 9,
    parameter int ML2C = 10
) (
    input  logic        clk,
    input  logic        rst,
    input  logic        we,
    input  logic [MIW-1:0] w_r, w_c,
    input  logic [5:0]  w_bw4, w_bh4,
    input  mv_ent_t     w_data,
    output logic        busy,
    input  logic        clr,                  // clear the written bits of rows clr_row .. clr_row + clr_n - 1
    input  logic [MIW-1:0] clr_row,
    input  logic [5:0]  clr_n,
    input  logic [MIW-1:0] rd_row, rd_col,
    output mv_ent_t     rd_ent,
    output logic        rd_written
);
    localparam int N = 1 << (ML2R + ML2C);
    localparam int WC = 1 << ML2C;
    // The entry store is written in the canonical RAM shape synthesis recognises (Vivado: one always_ff holding
    // only the write and the registered read of a flat vector array, addresses and data as plain signals, the
    // struct cast on outside). Anything fancier and the 2^(ML2R+ML2C) x 42-bit array becomes flops.
    localparam int EW = $bits(mv_ent_t);
    (* ram_style = "ultra" *) logic [EW-1:0] mem [0:N-1];       // 2^15 x 42 at 1024x512: 5 UltraRAMs, not 42 BRAMs
    logic           m_we;
    logic [ML2R+ML2C-1:0] m_waddr, m_raddr;
    logic [EW-1:0]  m_wdata, m_rdata;
    always_ff @(posedge clk) begin
        if (m_we) mem[m_waddr] <= m_wdata;
        m_rdata <= mem[m_raddr];
    end
    assign rd_ent = mv_ent_t'(m_rdata);
    // "written" bitmap: 64 row words of WC bits in distributed RAM (one write port: a row clear writes zeros, a
    // bit set writes the row word read asynchronously with the bit merged in, so both land in the same cycle the
    // flop version did). As flops it grew with the frame width: 16k at 1024 wide, 32k at 1080p.
    (* ram_style = "distributed" *) logic [WC-1:0] written [0:63];
    logic          wb_we;
    logic [5:0]    wb_addr;
    logic [WC-1:0] wb_data, wb_cur;
    logic [MIW-1:0] w_rr, w_cc;        // position the area-fill writer stores this cycle
    logic        c_busy, w_busy;
    logic [MIW-1:0] c_row;
    logic [5:0]  c_n;
    assign busy = c_busy || w_busy;
    always_ff @(posedge clk) begin
        if (rst) c_busy <= 1'b0;
        else if (!c_busy) begin
            if (clr) begin c_busy <= 1'b1; c_row <= clr_row; c_n <= clr_n; end
        end else begin
            c_row <= c_row + MIW'(1);
            if (c_n == 6'd1) c_busy <= 1'b0;
            c_n <= c_n - 6'd1;
        end
    end
    function automatic logic [ML2R+ML2C-1:0] idx(input logic [MIW-1:0] row, input logic [MIW-1:0] col);
        idx = {ML2R'(row), ML2C'(col)};
    endfunction
    logic [5:0]  wi, wj;
    logic [MIW-1:0] b_r, b_c;
    logic [5:0]  b_w, b_h;
    mv_ent_t     b_d;
    always_ff @(posedge clk) begin
        if (rst) w_busy <= 1'b0;
        else if (!w_busy) begin
            if (we) begin
                w_busy <= 1'b1; b_r <= w_r; b_c <= w_c; b_w <= w_bw4; b_h <= w_bh4; b_d <= w_data; wi <= 6'd0; wj <= 6'd0;
            end
        end else begin
            if (wj + 6'd1 < b_w) wj <= wj + 6'd1;
            else begin
                wj <= 6'd0;
                if (wi + 6'd1 < b_h) wi <= wi + 6'd1;
                else w_busy <= 1'b0;
            end
        end
        rd_written <= written[rd_row[5:0]][rd_col[ML2C-1:0]];   // one async read port, bit select, registered
    end
    // the area-fill writer stores one entry per cycle while busy (the FSM above steps wi/wj over the block)
    assign w_rr = b_r + MIW'(wi); assign w_cc = b_c + MIW'(wj);
    assign m_we = w_busy && !rst;
    assign m_waddr = idx(w_rr, w_cc);
    // the clear engine zeroes a row, the area-fill sets a bit; the two never overlap (the tile decoder waits on busy)
    always_comb begin
        wb_cur  = written[w_rr[5:0]];
        wb_we   = !rst && (c_busy || w_busy);
        wb_addr = c_busy ? c_row[5:0] : w_rr[5:0];
        wb_data = c_busy ? '0 : (wb_cur | (WC'(1) << w_cc[ML2C-1:0]));
    end
    always_ff @(posedge clk) if (wb_we) written[wb_addr] <= wb_data;
    assign m_wdata = b_d;
    assign m_raddr = idx(rd_row, rd_col);
endmodule
