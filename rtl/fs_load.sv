// Frame-state loader (docs/frame-state.md, stage B): at a frame start, before its first tile, the card reads the frame's
// context back from the reference slot's areas in DDR and writes the bands the host used to write register by register:
//   segment map: the primary slot's segment-id area (one byte per 4x4 unit, lines of ld_seg_rowb bytes, the layout
//     rtl/frame_state_w.sv writes) -> seg_band words (16 ids of 3 bits), the whole frame (INTER builds keep the full map);
//   CDF rows: the primary slot's CDF area (CDF_ROWS rows of 32 bytes: 246 bits + padding) -> def_we rows into every
//     engine, the adaptation counter at [245:240] zeroed (load_cdfs).
// One AXI read master, bursts of up to 16 beats inside 4 KB, up to OUT bursts in flight (2026-11-03; one at a time
// before: 1080p's segment map is 16k beats, a thousand round trips): an address side issues the bursts and queues
// {job, line, first beat} per burst, the data side takes the in-order beats straight into the bands (never stalls,
// rready = 1). The motion-field projection (rtl/mf_project.sv) runs beside this loader: it clears its band while the
// loader reads and takes the read channel afterwards (dec_top).
module fs_load
  import syn_pkg::*;
  import cdf_map_pkg::*;
#(
    parameter int OUT = 8                                    // bursts in flight (a power of two)
) (
    input  logic          clk,
    input  logic          rst,
    input  hdr_t          hdr,
    input  logic          ld_start,            // one-cycle pulse (dec_host START bit 6)
    input  logic [2:0]    ld_cfg,              // bit 0: the segment map from DDR, bit 1: the CDF rows, bit 2: zero the segment map
    input  logic [39:0]   ld_seg_base,
    input  logic [15:0]   ld_seg_rowb,         // bytes per 4x4 line (multiple of 8)
    input  logic [39:0]   ld_cdf_base,
    output logic          ld_busy,
    output logic          ld_done,             // one-cycle pulse
    output logic          ld_err,              // a read answered with an error (sticky until the next start)
    // AXI4 read master
    output logic [39:0]   m_axi_araddr,
    output logic [7:0]    m_axi_arlen,
    output logic [2:0]    m_axi_arsize,
    output logic [1:0]    m_axi_arburst,
    output logic [3:0]    m_axi_arcache,
    output logic [2:0]    m_axi_arprot,
    output logic          m_axi_arvalid,
    input  logic          m_axi_arready,
    input  logic [63:0]   m_axi_rdata,
    input  logic [1:0]    m_axi_rresp,
    input  logic          m_axi_rlast,
    input  logic          m_axi_rvalid,
    output logic          m_axi_rready,
    // the bands
    output logic          sp_we,
    output logic [MIW-1:0] sp_wrow,
    output logic [MIW-5:0] sp_wwc,
    output logic [47:0]   sp_wdata,
    output logic          def_we,
    output logic [CDF_AW-1:0] def_addr,
    output logic [245:0]  def_data
);
    localparam int OW = $clog2(OUT);
    // ---------------------------------------------------------------- the address side
    typedef enum logic [3:0] {L_IDLE, L_SEG_LINE, L_CDF_START, L_LEN, L_AR, L_DRAIN, L_DONE, L_ZERO} lstate_t;
    lstate_t        st;
    logic           job;                                     // 0 segment map, 1 CDF rows
    logic [39:0]    addr;                                    // the next burst's address
    logic [15:0]    beats;                                   // beats left in the current run (a line, or the CDF area)
    logic [15:0]    beat_a;                                  // beat index of the next burst within the run
    logic [4:0]     len;
    logic [9:0]     to4k;
    assign to4k = 10'((13'd4096 - 13'(addr[11:0])) >> 3);
    logic [MIW-1:0] line;                                    // segment map: the 4x4 line being issued
    logic [15:0]    zbeat;                                   // the zero pass: the word within the line
    logic [OW:0]    inflight;
    assign m_axi_arsize  = 3'b011;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_araddr  = addr;
    assign m_axi_arlen   = 8'(len - 5'd1);
    assign m_axi_arvalid = (st == L_AR);
    assign m_axi_rready  = 1'b1;                             // the data side never stalls
    assign ld_busy = (st != L_IDLE);
    logic [4:0] burst_len;
    always_comb begin
        burst_len = 5'd16;
        if (beats < 16'd16) burst_len = 5'(beats);
        if (to4k < 10'(burst_len)) burst_len = 5'(to4k);
    end
    typedef struct packed { logic job; logic [MIW-1:0] line; logic [15:0] beat0; } desc_t;
    desc_t          dq [OUT];
    logic [OW:0]    dq_w, dq_r;
    logic           issue, retire;
    assign issue  = (st == L_AR) && m_axi_arready;
    assign retire = m_axi_rvalid && m_axi_rlast;
    logic [15:0]    d_beat;                                  // beat index within the run
    logic [MIW-1:0] d_line;
    logic           d_job, d_sp_we, d_last;
    always_ff @(posedge clk) begin
        ld_done <= 1'b0; sp_we <= 1'b0;
        if (rst) begin
            st <= L_IDLE; job <= 1'b0; addr <= '0; beats <= '0; beat_a <= '0; len <= '0; line <= '0; zbeat <= '0; inflight <= '0; dq_w <= '0;
            sp_wrow <= '0; sp_wwc <= '0;
        end else begin
            inflight <= inflight + (OW+1)'(issue) - (OW+1)'(retire);
            if (issue) begin dq[dq_w[OW-1:0]] <= '{job: job, line: line, beat0: beat_a}; dq_w <= dq_w + 1'b1; end
            case (st)
                L_IDLE: if (ld_start) begin
                    line <= '0; inflight <= '0; dq_w <= '0;
                    if (ld_cfg[0]) begin job <= 1'b0; addr <= ld_seg_base; st <= L_SEG_LINE; end
                    else if (ld_cfg[2]) begin job <= 1'b0; zbeat <= '0; st <= L_ZERO; end
                    else if (ld_cfg[1]) begin job <= 1'b1; st <= L_CDF_START; end
                    else st <= L_DONE;
                end
                L_SEG_LINE: begin                            // one 4x4 line: ld_seg_rowb / 8 beats from the running address
                    beats <= ld_seg_rowb >> 3; beat_a <= '0;
                    if (line < hdr.mi_rows && ld_seg_rowb != 16'd0) st <= L_LEN;
                    else begin st <= ld_cfg[1] ? L_CDF_START : L_DRAIN; if (ld_cfg[1]) job <= 1'b1; end
                end
                L_CDF_START: begin addr <= ld_cdf_base; beats <= 16'(CDF_ROWS) * 16'd4; beat_a <= '0; st <= L_LEN; end
                L_LEN: begin len <= burst_len; if (inflight < (OW+1)'(OUT)) st <= L_AR; end   // arlen settled a cycle before arvalid (the sideband DMA's lesson)
                L_AR: if (m_axi_arready) begin
                    addr <= addr + 40'({len, 3'b000}); beats <= beats - 16'(len); beat_a <= beat_a + 16'(len);
                    if (beats == 16'(len)) begin
                        if (!job) begin line <= line + MIW'(1); st <= L_SEG_LINE; end
                        else st <= L_DRAIN;
                    end else st <= L_LEN;
                end
                L_ZERO: begin                                // PrevSegmentIds = zeros: one zero word per 16 units per line
                    sp_we <= 1'b1; sp_wrow <= line; sp_wwc <= (MIW-4)'(zbeat);
                    if (MIW'({zbeat[MIW-5:0], 4'd0}) + MIW'(16) < hdr.mi_cols) zbeat <= zbeat + 16'd1;
                    else begin
                        zbeat <= '0;
                        if (line + MIW'(1) < hdr.mi_rows) line <= line + MIW'(1);
                        else begin st <= ld_cfg[1] ? L_CDF_START : L_DONE; if (ld_cfg[1]) job <= 1'b1; end
                    end
                end
                L_DRAIN: if (inflight == '0) st <= L_DONE;   // every issued beat has arrived (and was written the same cycle)
                L_DONE: begin ld_done <= 1'b1; st <= L_IDLE; end
                default: st <= L_IDLE;
            endcase
            // the data side's segment-map writes share sp_we / sp_wrow / sp_wwc with the zero pass (never both at once)
            if (d_sp_we) begin sp_we <= 1'b1; sp_wrow <= d_line; sp_wwc <= (MIW-4)'(d_beat >> 1); end
        end
    end

    // ---------------------------------------------------------------- the data side: beats into the bands, in order
    logic [3:0]     bcnt;                                    // beat index within the burst
    logic [47:0]    acc;                                     // 16 segment ids being gathered
    logic [255:0]   row_acc;                                 // a CDF row's four beats
    logic [1:0]     row_q;
    logic [CDF_AW-1:0] row;
    assign d_job  = dq[dq_r[OW-1:0]].job;
    assign d_line = dq[dq_r[OW-1:0]].line;
    assign d_beat = dq[dq_r[OW-1:0]].beat0 + 16'(bcnt);
    assign d_last = (d_beat == (ld_seg_rowb >> 3) - 16'd1);  // the line's last beat: a partial word pads with zeros
    assign d_sp_we = m_axi_rvalid && !d_job && (d_beat[0] || d_last);
    always_ff @(posedge clk) begin
        def_we <= 1'b0;
        if (rst) begin
            bcnt <= '0; dq_r <= '0; acc <= '0; row_acc <= '0; row_q <= '0; row <= '0; ld_err <= 1'b0; sp_wdata <= '0; def_addr <= '0; def_data <= '0;
        end else begin
            if (ld_start) begin bcnt <= '0; dq_r <= '0; acc <= '0; row_q <= '0; row <= '0; ld_err <= 1'b0; end
            if (st == L_ZERO) sp_wdata <= '0;            // the zero pass's words
            if (m_axi_rvalid) begin
                if (m_axi_rresp[1]) ld_err <= 1'b1;
                bcnt <= m_axi_rlast ? 4'd0 : bcnt + 4'd1;
                if (m_axi_rlast) dq_r <= dq_r + 1'b1;
                if (!d_job) begin                            // 8 ids a beat, two beats a 16-unit word
                    for (int k = 0; k < 8; k++) begin
                        if (!d_beat[0]) acc[3*k +: 3] <= m_axi_rdata[8*k +: 3];
                        else            acc[3*(8+k) +: 3] <= m_axi_rdata[8*k +: 3];
                    end
                    if (d_beat[0] || d_last) begin
                        for (int k = 0; k < 8; k++) begin  // the word as it will be after this beat
                            if (!d_beat[0]) begin sp_wdata[3*k +: 3] <= m_axi_rdata[8*k +: 3]; sp_wdata[3*(8+k) +: 3] <= 3'd0; end
                            else begin sp_wdata[3*k +: 3] <= acc[3*k +: 3]; sp_wdata[3*(8+k) +: 3] <= m_axi_rdata[8*k +: 3]; end
                        end
                    end
                end else begin                               // four beats a row; the counter field [245:240] zeroed on load
                    row_acc[64 * row_q +: 64] <= m_axi_rdata;
                    row_q <= row_q + 2'd1;
                    if (row_q == 2'd3) begin
                        def_we <= 1'b1; def_addr <= row;
                        def_data <= {6'd0, m_axi_rdata[47:0], row_acc[191:0]};
                        row <= row + CDF_AW'(1);
                    end
                end
            end
        end
    end
endmodule
