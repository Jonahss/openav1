// CDF store: one 246-bit row per adaptive CDF (address map: cdf_map_pkg, generated). Holds the tile's
// working copy (mem) and the frame's defaults (def_mem, written by the host once per frame with the
// base_q_idx-dependent coefficient tables). `init` copies def_mem -> mem (CDF_ROWS cycles, `busy` high).
//
// Read port: rd_en/rd_addr -> rd_data one cycle later. Write-back port for the adapted row. A write-back
// in the same cycle as a read of the same row is forwarded, and so is one announced for the next cycle
// (wb_pre / wb_pre_addr, the sequencer's done cycle: the row arrives with wb_we a cycle later, as the
// read data would), so a requester may read the row that is about to be overwritten without a stall.
module cdf_store
  import cdf_map_pkg::*;
(
    input  logic                clk,
    input  logic                rst,
    // host: load defaults
    input  logic                def_we,
    input  logic [CDF_AW-1:0]   def_addr,
    input  logic [245:0]        def_data,
    // tile start: copy defaults into the working store
    input  logic                init,
    output logic                busy,
    // requester read
    input  logic                rd_en,
    input  logic [CDF_AW-1:0]   rd_addr,
    output logic [245:0]        rd_data,
    // requester write-back (adapted row)
    input  logic                wb_we,
    input  logic [CDF_AW-1:0]   wb_addr,
    input  logic [245:0]        wb_data,
    input  logic                wb_pre,
    input  logic [CDF_AW-1:0]   wb_pre_addr
);
    // 1269 x 246 bits each. Block RAM (14 BRAM36 apiece on the K26): the working store's read is the start of
    // the symbol loop and a BRAM's clock-to-out is a third of an UltraRAM's with its cascade (the auto choice
    // was LUT RAM, 5.6k LUTs; UltraRAM was used until 2026-10-14).
    (* ram_style = "block" *) logic [245:0] mem     [0:CDF_ROWS-1];
    (* ram_style = "block" *) logic [245:0] def_mem [0:CDF_ROWS-1];

    always_ff @(posedge clk)
        if (def_we) def_mem[def_addr] <= def_data;

    // init copy engine: read def_mem[i] this cycle, write mem[i] next cycle
    logic [CDF_AW-1:0] cp_i;
    logic              cp_run, cp_wr;
    logic [CDF_AW-1:0] cp_wr_addr;
    logic [245:0]      cp_rd;
    always_ff @(posedge clk) begin
        if (rst) begin
            cp_run <= 1'b0; cp_wr <= 1'b0; cp_i <= '0;
        end else begin
            cp_wr <= cp_run;
            cp_wr_addr <= cp_i;
            cp_rd <= def_mem[cp_i];
            if (init) begin
                cp_run <= 1'b1; cp_i <= '0;
            end else if (cp_run) begin
                if (cp_i == CDF_AW'(CDF_ROWS - 1)) cp_run <= 1'b0;
                cp_i <= cp_i + 1'b1;
            end
        end
    end
    assign busy = cp_run | cp_wr | init;

    // working store: one write port (copy engine has priority; requesters never write while busy)
    logic              we;
    logic [CDF_AW-1:0] wa;
    logic [245:0]      wd;
    always_comb begin
        we = cp_wr | wb_we;
        wa = cp_wr ? cp_wr_addr : wb_addr;
        wd = cp_wr ? cp_rd : wb_data;
    end
    always_ff @(posedge clk)
        if (we) mem[wa] <= wd;

    // Read: the memory's own output register (plain synchronous read, so it maps to the RAM's port register),
    // plus a forwarding register for a same-cycle write-back of the row being read. Both hold until the next rd_en.
    logic [245:0] rd_raw, fwd_d;
    logic         fwd_v, fwd_next;
    always_ff @(posedge clk)
        if (rd_en) rd_raw <= mem[rd_addr];
    always_ff @(posedge clk) begin
        if (rd_en) begin
            fwd_v <= we && wa == rd_addr;
            fwd_d <= wd;
        end
        fwd_next <= rd_en && wb_pre && wb_pre_addr == rd_addr;   // one cycle: the write-back is on wb_data then
    end
    assign rd_data = fwd_next ? wb_data : fwd_v ? fwd_d : rd_raw;
endmodule
