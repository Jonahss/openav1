// rf_ddr_bridge (K60-108): reference fetch's read port taken to the DDR4 controller past the smartconnect.
//
// Why: the smartconnect's slave-side converter takes one read address every 4 decoder clocks whatever is declared on
// the port (measured on the memory twin and on the card, 2026-11-27: 2,000 two-beat reads = 8,040 clocks at any stride;
// narrow-burst support off, 32 reads outstanding, burst length 16: no change; its source is encrypted). Reference
// fetch issues one short burst per patch row, 530,145 a 4K frame, and waited 1.51 M of a frame's 2.71 M clocks there.
// The controller itself takes such reads 4 to 13 times faster (7 to 33 ns each, depending on the stride).
//
// What: sits between the smartconnect's master port and the controller's AXI slave port, in the controller's clock
// (uclk). Writes pass through untouched. Read addresses come from two sources: the smartconnect (XDMA and the
// decoder's 64-bit master, ID 0) and reference fetch (ID 1), whose addresses cross from the decoder clock (dclk) in
// an asynchronous FIFO. The controller answers in the order it took the addresses and returns the ID with every
// beat: beats with ID 1 go into an asynchronous data FIFO back to dclk, the others to the smartconnect.
//
// The data FIFO can never overflow: a reference-fetch address is issued only when the FIFO has room for all its
// beats on top of the beats already issued and not yet arrived (`owed`), counted with the FIFO's own write-side
// fill, which errs high. So the controller is never held by reference fetch (rready = 1 for its beats) and a slow
// decoder cannot block the smartconnect's reads behind its own.
//
// Reference fetch must accept every beat it is offered within a bounded time (it does: rready = 1). Bursts are
// INCR, 64 bytes a beat, at most 16 beats (ref_fetch's rule); MAXB is the bound the credit uses.
`timescale 1ps/1ps
module rf_ddr_bridge #(
    parameter int AW    = 34,
    parameter int LGD   = 7,                     // log2 of the data FIFO's depth in beats (128)
    parameter int LGA   = 4                      // log2 of the address FIFO's depth (16)
) (
    // ---- decoder clock: reference fetch's read master
    input  wire             dclk,
    input  wire [AW-1:0]    s_rf_araddr,
    input  wire [7:0]       s_rf_arlen,
    input  wire             s_rf_arvalid,
    output wire             s_rf_arready,
    output wire [511:0]     s_rf_rdata,
    output wire [1:0]       s_rf_rresp,
    output wire             s_rf_rlast,
    output wire             s_rf_rvalid,
    input  wire             s_rf_rready,
    // ---- controller clock
    input  wire             uclk,
    input  wire             urst,                // synchronous to uclk, active high (the controller's ui_clk_sync_rst)
    // the smartconnect's master port
    input  wire [AW-1:0]    s_sc_awaddr,
    input  wire [7:0]       s_sc_awlen,
    input  wire [2:0]       s_sc_awsize,
    input  wire [1:0]       s_sc_awburst,
    input  wire [3:0]       s_sc_awcache,
    input  wire [2:0]       s_sc_awprot,
    input  wire             s_sc_awvalid,
    output wire             s_sc_awready,
    input  wire [511:0]     s_sc_wdata,
    input  wire [63:0]      s_sc_wstrb,
    input  wire             s_sc_wlast,
    input  wire             s_sc_wvalid,
    output wire             s_sc_wready,
    output wire [1:0]       s_sc_bresp,
    output wire             s_sc_bvalid,
    input  wire             s_sc_bready,
    input  wire [AW-1:0]    s_sc_araddr,
    input  wire [7:0]       s_sc_arlen,
    input  wire [2:0]       s_sc_arsize,
    input  wire [1:0]       s_sc_arburst,
    input  wire [3:0]       s_sc_arcache,
    input  wire             s_sc_arvalid,
    output wire             s_sc_arready,
    output wire [511:0]     s_sc_rdata,
    output wire [1:0]       s_sc_rresp,
    output wire             s_sc_rlast,
    output wire             s_sc_rvalid,
    input  wire             s_sc_rready,
    // the controller's AXI slave port
    output wire [0:0]       m_awid,
    output wire [AW-1:0]    m_awaddr,
    output wire [7:0]       m_awlen,
    output wire [2:0]       m_awsize,
    output wire [1:0]       m_awburst,
    output wire [3:0]       m_awcache,
    output wire [2:0]       m_awprot,
    output wire             m_awvalid,
    input  wire             m_awready,
    output wire [511:0]     m_wdata,
    output wire [63:0]      m_wstrb,
    output wire             m_wlast,
    output wire             m_wvalid,
    input  wire             m_wready,
    input  wire [1:0]       m_bresp,
    input  wire             m_bvalid,
    output wire             m_bready,
    output reg  [0:0]       m_arid,
    output reg  [AW-1:0]    m_araddr,
    output reg  [7:0]       m_arlen,
    output reg  [2:0]       m_arsize,
    output reg  [1:0]       m_arburst,
    output reg  [3:0]       m_arcache,
    output reg              m_arvalid,
    input  wire             m_arready,
    input  wire [0:0]       m_rid,
    input  wire [511:0]     m_rdata,
    input  wire [1:0]       m_rresp,
    input  wire             m_rlast,
    input  wire             m_rvalid,
    output wire             m_rready
);
    // ---- writes: straight through
    assign m_awid = 1'b0; assign m_awaddr = s_sc_awaddr; assign m_awlen = s_sc_awlen; assign m_awsize = s_sc_awsize;
    assign m_awburst = s_sc_awburst; assign m_awcache = s_sc_awcache; assign m_awprot = s_sc_awprot;
    assign m_awvalid = s_sc_awvalid; assign s_sc_awready = m_awready;
    assign m_wdata = s_sc_wdata; assign m_wstrb = s_sc_wstrb; assign m_wlast = s_sc_wlast; assign m_wvalid = s_sc_wvalid;
    assign s_sc_wready = m_wready;
    assign s_sc_bresp = m_bresp; assign s_sc_bvalid = m_bvalid; assign m_bready = s_sc_bready;

    // ---- reference fetch's addresses: dclk -> uclk
    wire            af_full, af_empty, af_wr_busy, af_rd_busy;
    wire [AW+7:0]   af_dout;
    wire            af_pop;
    wire            drst;                                // the address FIFO is reset from its write side: urst seen in dclk
    xpm_cdc_sync_rst #(.DEST_SYNC_FF(4), .INIT(1), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0)) u_drst (.src_rst(urst), .dest_clk(dclk), .dest_rst(drst));
    assign s_rf_arready = !af_full && !af_wr_busy;
    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("distributed"), .FIFO_WRITE_DEPTH(1 << LGA), .WRITE_DATA_WIDTH(AW + 8), .READ_DATA_WIDTH(AW + 8),
        .READ_MODE("fwft"), .FIFO_READ_LATENCY(0), .CDC_SYNC_STAGES(2), .RELATED_CLOCKS(0), .USE_ADV_FEATURES("0000"),
        .FULL_RESET_VALUE(1), .DOUT_RESET_VALUE("0"), .ECC_MODE("no_ecc"), .SIM_ASSERT_CHK(0), .WAKEUP_TIME(0),
        .RD_DATA_COUNT_WIDTH(LGA + 1), .WR_DATA_COUNT_WIDTH(LGA + 1), .PROG_EMPTY_THRESH(5), .PROG_FULL_THRESH(5)
    ) u_af (
        .rst(drst), .wr_clk(dclk), .wr_en(s_rf_arvalid && s_rf_arready), .din({s_rf_arlen, s_rf_araddr}), .full(af_full), .wr_rst_busy(af_wr_busy),
        .rd_clk(uclk), .rd_en(af_pop), .dout(af_dout), .empty(af_empty), .rd_rst_busy(af_rd_busy),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0),
        .almost_empty(), .almost_full(), .data_valid(), .dbiterr(), .sbiterr(), .overflow(), .underflow(), .prog_empty(), .prog_full(),
        .rd_data_count(), .wr_data_count(), .wr_ack());

    // ---- reference fetch's data: uclk -> dclk
    wire            df_full, df_empty, df_wr_busy, df_rd_busy, df_wr;
    wire [LGD:0]    df_fill;                             // write-side fill: never below the true fill
    wire [512:0]    df_dout;
    assign df_wr = m_rvalid && m_rid[0];
    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("block"), .FIFO_WRITE_DEPTH(1 << LGD), .WRITE_DATA_WIDTH(513), .READ_DATA_WIDTH(513),
        .READ_MODE("fwft"), .FIFO_READ_LATENCY(0), .CDC_SYNC_STAGES(2), .RELATED_CLOCKS(0), .USE_ADV_FEATURES("0004"),
        .FULL_RESET_VALUE(1), .DOUT_RESET_VALUE("0"), .ECC_MODE("no_ecc"), .SIM_ASSERT_CHK(0), .WAKEUP_TIME(0),
        .RD_DATA_COUNT_WIDTH(LGD + 1), .WR_DATA_COUNT_WIDTH(LGD + 1), .PROG_EMPTY_THRESH(5), .PROG_FULL_THRESH(5)
    ) u_df (
        .rst(urst), .wr_clk(uclk), .wr_en(df_wr), .din({m_rlast, m_rdata}), .full(df_full), .wr_rst_busy(df_wr_busy), .wr_data_count(df_fill),
        .rd_clk(dclk), .rd_en(s_rf_rvalid && s_rf_rready), .dout(df_dout), .empty(df_empty), .rd_rst_busy(df_rd_busy),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0),
        .almost_empty(), .almost_full(), .data_valid(), .dbiterr(), .sbiterr(), .overflow(), .underflow(), .prog_empty(), .prog_full(),
        .rd_data_count(), .wr_ack());
    assign s_rf_rvalid = !df_empty && !df_rd_busy;
    assign s_rf_rdata  = df_dout[511:0];
    assign s_rf_rlast  = df_dout[512];
    assign s_rf_rresp  = 2'b00;

    // ---- the credit: beats issued for reference fetch that have not arrived yet
    reg  [LGD+1:0]  owed;
    wire [8:0]      rf_beats = {1'b0, af_dout[AW+7:AW]} + 9'd1;
    wire [LGD+2:0]  need = (LGD+3)'(owed) + (LGD+3)'(df_fill) + (LGD+3)'(rf_beats) + (LGD+3)'(4);   // 4: the fill count's own delay
    wire            rf_ok = !af_empty && !af_rd_busy && !df_wr_busy && (need <= (1 << LGD));

    // ---- read addresses: one register stage; the source that did not go last wins a tie
    reg             last_rf;
    wire            can     = !m_arvalid || m_arready;
    wire            pick_rf = rf_ok && (!s_sc_arvalid || !last_rf);
    wire            pick_sc = s_sc_arvalid && !pick_rf;
    assign af_pop       = can && pick_rf;
    assign s_sc_arready = can && pick_sc;
    always @(posedge uclk) begin
        if (urst) begin m_arvalid <= 1'b0; last_rf <= 1'b0; owed <= '0; end
        else begin
            if (can) begin
                m_arvalid <= pick_rf || pick_sc;
                if (pick_rf) begin
                    m_arid <= 1'b1; m_araddr <= af_dout[AW-1:0]; m_arlen <= af_dout[AW+7:AW];
                    m_arsize <= 3'd6; m_arburst <= 2'b01; m_arcache <= 4'b0011; last_rf <= 1'b1;
                end else if (pick_sc) begin
                    m_arid <= 1'b0; m_araddr <= s_sc_araddr; m_arlen <= s_sc_arlen;
                    m_arsize <= s_sc_arsize; m_arburst <= s_sc_arburst; m_arcache <= s_sc_arcache; last_rf <= 1'b0;
                end
            end
            owed <= owed + (af_pop ? (LGD+2)'(rf_beats) : '0) - (LGD+2)'(df_wr);
        end
    end

    // ---- read data: by the ID the controller returns
    assign s_sc_rdata  = m_rdata;
    assign s_sc_rresp  = m_rresp;
    assign s_sc_rlast  = m_rlast;
    assign s_sc_rvalid = m_rvalid && !m_rid[0];
    assign m_rready    = m_rid[0] ? 1'b1 : s_sc_rready;
`ifndef SYNTHESIS
    always @(posedge uclk) if (!urst && df_wr && df_full) $error("rf_ddr_bridge: a reference beat arrived with the data FIFO full");
`endif
endmodule
