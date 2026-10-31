// Frame-state loader (docs/frame-state.md, stage B): at a frame start, before its first tile, the card reads the frame's
// context back from the reference slot's areas in DDR and writes the bands the host used to write register by register:
//   segment map: the primary slot's segment-id area (one byte per 4x4 unit, lines of ld_seg_rowb bytes, the layout
//     rtl/frame_state_w.sv writes) -> seg_band words (16 ids of 3 bits), the whole frame (INTER builds keep the full map);
//   CDF rows: the primary slot's CDF area (CDF_ROWS rows of 32 bytes: 246 bits + padding) -> def_we rows into every
//     engine, the adaptation counter at [245:240] zeroed (load_cdfs).
// One AXI read master, bursts of up to 16 beats inside 4 KB. The motion-field projection (7.9) is the next job here.
module fs_load
  import syn_pkg::*;
  import cdf_map_pkg::*;
(
    input  logic          clk,
    input  logic          rst,
    input  hdr_t          hdr,
    input  logic          ld_start,            // one-cycle pulse (dec_host START bit 6)
    input  logic [1:0]    ld_cfg,              // bit 0: the segment map, bit 1: the CDF rows
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
    typedef enum logic [2:0] {L_IDLE, L_SEG_LINE, L_CDF_START, L_AR, L_R, L_NEXT, L_DONE} lstate_t;
    lstate_t        st;
    logic           job;                                     // 0 segment map, 1 CDF rows
    logic [39:0]    addr;                                    // the next beat's address
    logic [31:0]    beats;                                   // beats left in the current run (a line, or the CDF area)
    logic [4:0]     len, cnt;
    logic [9:0]     to4k;
    assign to4k = 10'((13'd4096 - 13'(addr[11:0])) >> 3);
    logic [MIW-1:0] line;                                    // segment map: the 4x4 line being loaded
    logic [15:0]    beat;                                    // beat index within the line
    logic [47:0]    acc;                                     // 16 segment ids being gathered
    logic [255:0]   row_acc;                                 // a CDF row's four beats
    logic [1:0]     row_q;
    logic [CDF_AW-1:0] row;
    assign m_axi_arsize  = 3'b011;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_araddr  = addr;
    assign m_axi_arlen   = 8'(len - 5'd1);
    assign m_axi_arvalid = (st == L_AR);
    assign m_axi_rready  = (st == L_R);
    assign ld_busy = (st != L_IDLE);
    logic [4:0] burst_len;
    always_comb begin
        burst_len = 5'd16;
        if (beats < 32'd16) burst_len = 5'(beats);
        if (to4k < 10'(burst_len)) burst_len = 5'(to4k);
    end
    always_ff @(posedge clk) begin
        ld_done <= 1'b0; sp_we <= 1'b0; def_we <= 1'b0;
        if (rst) begin
            st <= L_IDLE; job <= 1'b0; addr <= '0; beats <= '0; len <= '0; cnt <= '0; line <= '0; beat <= '0; acc <= '0;
            row_acc <= '0; row_q <= '0; row <= '0; ld_err <= 1'b0; sp_wrow <= '0; sp_wwc <= '0; sp_wdata <= '0; def_addr <= '0; def_data <= '0;
        end else begin
            case (st)
                L_IDLE: if (ld_start) begin
                    ld_err <= 1'b0; line <= '0; row <= '0; row_q <= '0;
                    if (ld_cfg[0]) begin job <= 1'b0; addr <= ld_seg_base; st <= L_SEG_LINE; end
                    else if (ld_cfg[1]) begin job <= 1'b1; st <= L_CDF_START; end
                    else st <= L_DONE;
                end
                L_SEG_LINE: begin                            // one 4x4 line: ld_seg_rowb / 8 beats from the running address
                    beats <= 32'(ld_seg_rowb >> 3); beat <= '0; acc <= '0;
                    st <= (line < hdr.mi_rows && ld_seg_rowb != 16'd0) ? L_AR : (ld_cfg[1] ? L_CDF_START : L_DONE);
                    if (!(line < hdr.mi_rows && ld_seg_rowb != 16'd0) && ld_cfg[1]) job <= 1'b1;
                end
                L_CDF_START: begin addr <= ld_cdf_base; beats <= 32'(CDF_ROWS) * 32'd4; row <= '0; row_q <= '0; st <= L_AR; end
                L_AR: begin len <= burst_len; cnt <= '0; if (m_axi_arready) st <= L_R; end
                L_R: if (m_axi_rvalid) begin
                    if (m_axi_rresp[1]) ld_err <= 1'b1;
                    addr <= addr + 40'd8; beats <= beats - 32'd1; cnt <= cnt + 5'd1;
                    if (!job) begin                          // 8 ids a beat, two beats a 16-unit word; a partial last word pads with zeros
                        for (int k = 0; k < 8; k++) begin
                            if (!beat[0]) acc[3*k +: 3] <= m_axi_rdata[8*k +: 3];
                            else          acc[3*(8+k) +: 3] <= m_axi_rdata[8*k +: 3];
                        end
                        if (beat[0] || beats == 32'd1) begin
                            sp_we <= 1'b1; sp_wrow <= line; sp_wwc <= (MIW-4)'(beat >> 1);
                            for (int k = 0; k < 8; k++) begin  // the word as it will be after this beat
                                if (!beat[0]) begin sp_wdata[3*k +: 3] <= m_axi_rdata[8*k +: 3]; sp_wdata[3*(8+k) +: 3] <= 3'd0; end
                                else begin sp_wdata[3*k +: 3] <= acc[3*k +: 3]; sp_wdata[3*(8+k) +: 3] <= m_axi_rdata[8*k +: 3]; end
                            end
                        end
                        beat <= beat + 16'd1;
                    end else begin                           // four beats a row; the counter field [245:240] zeroed on load
                        row_acc[64 * row_q +: 64] <= m_axi_rdata;
                        row_q <= row_q + 2'd1;
                        if (row_q == 2'd3) begin
                            def_we <= 1'b1; def_addr <= row;
                            def_data <= {6'd0, m_axi_rdata[47:0], row_acc[191:0]};
                            row <= row + CDF_AW'(1);
                        end
                    end
                    if (m_axi_rlast) st <= L_NEXT;
                end
                L_NEXT: begin
                    if (beats != 32'd0) st <= L_AR;
                    else if (!job) begin line <= line + MIW'(1); st <= L_SEG_LINE; end
                    else st <= L_DONE;
                end
                L_DONE: begin ld_done <= 1'b1; st <= L_IDLE; end
                default: st <= L_IDLE;
            endcase
        end
    end
endmodule
