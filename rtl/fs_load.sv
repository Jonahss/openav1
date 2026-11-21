// Frame-state loader (docs/frame-state.md, stage B): at a frame start, before its first tile, the card reads the frame's
// context back from the reference slot's areas in DDR and writes the bands the host used to write register by register:
//   segment map: the primary slot's segment-id area (one byte per 4x4 unit, lines of ld_seg_rowb bytes, the layout
//     rtl/frame_state_w.sv writes) -> seg_band words (16 ids of 3 bits);
//   CDF rows: the primary slot's CDF area (CDF_ROWS rows of 32 bytes: 246 bits + padding) -> def_we rows into every
//     engine, the adaptation counter at [245:240] zeroed (load_cdfs).
// Order (docs/segment-map-band.md): the CDF rows first, then the segment map (or the zero map) band by band: band k
// = lines 16k .. 16k+15, each when band_go allows it; bands_done counts the finished bands and bands_all says the
// last one is in (set at once when the frame has no map to load). Each band's beats have all arrived and been
// written before it is counted.
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
    output logic          ld_done,             // one-cycle pulse: the CDF rows are in (the host waits for this); the map follows band by band
    output logic          ld_err,              // a read answered with an error (sticky until the next start)
    input  logic          band_go,             // the next band of the segment map may be loaded (its rows in seg_band are free)
    output logic          band_en,             // this frame's map goes band by band (set with ld_start; else nothing waits for it)
    output logic          rd_req,              // the loader wants the read channel (the CDF rows, or a band it may load)
    input  logic          rd_grant,            // it has it (dec_top's owner register; held while rd_req or rd_busy)
    output logic          rd_busy,             // reads issued or in flight
    output logic [MIW-5:0] bands_done,         // bands of the map loaded since ld_start
    output logic          bands_all,           // every band of the frame's map is in (or the frame loads none)
    // AXI4 read master: 8-byte beats, INCR bursts (the attributes are driven with ref_fetch's in dec_top)
    output logic [39:0]   m_axi_araddr,
    output logic [7:0]    m_axi_arlen,
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
    typedef enum logic [3:0] {L_IDLE, L_CDF_START, L_LEN, L_AR, L_DRAIN, L_BGO, L_SEG_LINE, L_ZERO, L_BDRAIN, L_DONE} lstate_t;
    lstate_t        st;
    logic           job;                                     // 0 segment map, 1 CDF rows
    logic [39:0]    addr;                                    // the next burst's address
    logic [15:0]    beats;                                   // beats left in the current run (a line, or the CDF area)
    logic [15:0]    beat_a;                                  // beat index of the next burst within the run
    logic [4:0]     len;
    logic [9:0]     to4k;
    assign to4k = 10'((13'd4096 - 13'(addr[11:0])) >> 3);
    logic [MIW-1:0] line;                                    // segment map: the 4x4 line being issued
    logic [MIW:0]   b_end;                                   // the line after the band being loaded
    logic [39:0]    saddr;                                   // segment map: the address of the next line (kept across the CDF job and between bands)
    logic           map_on;                                  // the frame has a map to load (from DDR) or to zero
    assign map_on = (ld_cfg[0] && ld_seg_rowb != 16'd0) || (!ld_cfg[0] && ld_cfg[2]);
    logic [15:0]    zbeat;                                   // the zero pass: the word within the line
    logic [OW:0]    inflight;
    assign m_axi_araddr  = addr;
    assign m_axi_arlen   = 8'(len - 5'd1);
    assign m_axi_arvalid = (st == L_AR);
    assign m_axi_rready  = 1'b1;                             // the data side never stalls
    assign ld_busy = (st != L_IDLE);
    logic           done_sent;
    assign rd_req  = (st == L_CDF_START) || ((st == L_BGO) && map_on && ld_cfg[0] && (line < hdr.mi_rows) && band_go);
    assign rd_busy = (st == L_LEN) || (st == L_AR) || (st == L_DRAIN) || (st == L_SEG_LINE) || (st == L_BDRAIN);
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
            sp_wrow <= '0; sp_wwc <= '0; b_end <= '0; saddr <= '0; bands_done <= '0; bands_all <= 1'b0; band_en <= 1'b0; done_sent <= 1'b0;
        end else begin
            inflight <= inflight + (OW+1)'(issue) - (OW+1)'(retire);
            if (issue) begin dq[dq_w[OW-1:0]] <= '{job: job, line: line, beat0: beat_a}; dq_w <= dq_w + 1'b1; end
            case (st)
                L_IDLE: if (ld_start) begin
                    line <= '0; inflight <= '0; dq_w <= '0; saddr <= ld_seg_base; bands_done <= '0; bands_all <= 1'b0; band_en <= map_on; done_sent <= 1'b0;
                    if (ld_cfg[1]) begin job <= 1'b1; st <= L_CDF_START; end
                    else begin job <= 1'b0; st <= L_BGO; end
                end
                L_CDF_START: if (rd_grant) begin addr <= ld_cdf_base; beats <= 16'(CDF_ROWS) * 16'd4; beat_a <= '0; st <= L_LEN; end
                L_LEN: begin len <= burst_len; if (inflight < (OW+1)'(OUT)) st <= L_AR; end   // arlen settled a cycle before arvalid (the sideband DMA's lesson)
                L_AR: if (m_axi_arready) begin
                    addr <= addr + 40'({len, 3'b000}); beats <= beats - 16'(len); beat_a <= beat_a + 16'(len);
                    if (beats == 16'(len)) begin
                        if (!job) begin line <= line + MIW'(1); st <= L_SEG_LINE; end
                        else st <= L_DRAIN;
                    end else st <= L_LEN;
                end
                L_DRAIN: if (inflight == '0) begin job <= 1'b0; st <= L_BGO; end   // the CDF rows are in
                L_BGO: begin                                 // the next band of the map, or the end
                    if (!done_sent) begin ld_done <= 1'b1; done_sent <= 1'b1; end   // the CDF rows are in (or the frame loads none)
                    b_end <= (MIW+1)'(line) + (MIW+1)'(16);
                    if (!map_on || line >= hdr.mi_rows) begin bands_all <= 1'b1; st <= L_DONE; end
                    else if (ld_cfg[0]) begin if (rd_grant && band_go) begin addr <= saddr; st <= L_SEG_LINE; end end   // (the grant outlives a band by a cycle: band_go again)
                    else if (band_go) begin zbeat <= '0; st <= L_ZERO; end
                end
                L_SEG_LINE: begin                            // one 4x4 line: ld_seg_rowb / 8 beats from the running address
                    beats <= ld_seg_rowb >> 3; beat_a <= '0;
                    if (line < hdr.mi_rows && (MIW+1)'(line) < b_end) st <= L_LEN;
                    else begin saddr <= addr; st <= L_BDRAIN; end
                end
                L_ZERO: begin                                // PrevSegmentIds = zeros: one zero word per 16 units per line
                    sp_we <= 1'b1; sp_wrow <= line; sp_wwc <= (MIW-4)'(zbeat);
                    if (MIW'({zbeat[MIW-5:0], 4'd0}) + MIW'(16) < hdr.mi_cols) zbeat <= zbeat + 16'd1;
                    else begin
                        zbeat <= '0; line <= line + MIW'(1);
                        if (!(line + MIW'(1) < hdr.mi_rows && (MIW+1)'(line) + (MIW+1)'(1) < b_end)) st <= L_BDRAIN;
                    end
                end
                L_BDRAIN: if (inflight == '0) begin bands_done <= bands_done + 1'b1; st <= L_BGO; end   // the band's beats have arrived and were written
                L_DONE: st <= L_IDLE;
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
