// A prediction lane (K60-94, K60-105): a reference fetch and a filter of their own, with the run queue, the run
// records, the run thread and the writer of recon_top's lane, for runs whose words go straight to the frame memory
// (direct runs: no overlapped-block pass, no inter-intra, no mask). recon_top's block FSM hands a strip to this lane
// the way it hands one to its own: a fetch request and, with its accept, the run that filters the fetched patch.
// The lane starts each run when its patch is in (or with its request, a chasing run) and writes its output words
// through a pair of 4-sample write streams (two words a clock). A second lane lets the chroma runs of a block be
// fetched, filtered and written while the first lane works on its luma runs.
//
// What recon_top keeps: which strips come here, the count of runs started and finished per block parity (start /
// done pulses with the run's parity, and pend: a queued run of that parity), and the rule for the write streams
// (v_hold: the lane takes no run into its vertical side while another writer owns its write ports; v_active: a run
// is in or waiting for the vertical side or has words on their way).
//
// The queue, the request numbering (acc_cnt / done_cnt), the patch buffers taking turns per accepted request and the
// four run records are recon_top's (docs/reference-prefetch.md, steps B and C2), without the blend fields.
module mc_lane #(
    parameter int PW = 12,
    parameter int PXW = 13,                                  // picture coordinates
    parameter int FBX = 10,
    parameter int FBY = 9,
    parameter int RING = 16                                  // ref_fetch line buffers
) (
    input  logic          clk,
    input  logic          rst,
    input  logic [3:0]    bit_depth,
    // ---- a strip: the fetch request, and the run queued with its accept ----
    input  logic          req_valid,
    output logic          req_acc,                           // accepted this cycle (there was room for the strip)
    input  logic          req_lst,
    input  logic          req_fmt16,
    input  logic [39:0]   req_base,
    input  logic [31:0]   req_stride,
    input  logic [15:0]   req_lastx,
    input  logic [15:0]   req_lasty,
    input  logic signed [16:0] req_x0,
    input  logic signed [16:0] req_y0,
    input  logic [6:0]    req_w,
    input  logic [6:0]    req_h,
    input  logic [6:0]    req_r_lo,
    input  logic [6:0]    req_r_hi,
    input  logic          q_chase,                           // the run may start with its request (unscaled and warp runs)
    input  logic          q_warp,
    input  logic [6:0]    q_sw,
    input  logic [6:0]    q_sh,
    input  logic [1:0]    q_i0,
    input  logic [1:0]    q_i1,
    input  logic [9:0]    q_xf0,
    input  logic [9:0]    q_yf0,
    input  logic [11:0]   q_xs,
    input  logic [11:0]   q_ys,
    input  logic signed [15:0] q_wa, q_wb, q_wg, q_wd,
    input  logic [15:0]   q_wsx,
    input  logic [15:0]   q_wsy,
    input  logic [2:0]    q_mode,
    input  logic [4:0]    q_wt0,
    input  logic [4:0]    q_wt1,
    input  logic [PXW-1:0] q_px,                             // strip origin in its plane
    input  logic [PXW-1:0] q_py,
    input  logic [1:0]    q_pl,
    input  logic          q_par,                             // the block's parity
    // ---- for recon_top's tracker and its write-port rule ----
    output logic          start,                             // a run starts (pulse), of a block of parity start_par
    output logic          start_par,
    output logic          done,                              // a run has finished and its last word has left (pulse)
    output logic          done_par,
    output logic [1:0]    pend,                              // per parity: a queued run that has not started
    input  logic          v_hold,
    output logic          v_active,
    output logic          err,                               // sticky: a reference read answered with an error
    output logic [3:0]    mc_pf,                             // the filter's and the fetch's flags for the counters
    output logic [5:0]    rf_pf,
    // ---- the two write streams (4 samples each; the second is the next group of the row) ----
    output logic          w_we,
    output logic [1:0]    w_plane,
    output logic [FBX-1:0] w_x,
    output logic [FBY-1:0] w_y,
    output logic [47:0]   w_data,
    output logic [3:0]    w_be,
    output logic          wb_we,
    output logic [FBX-1:0] wb_x,
    output logic [47:0]   wb_data,
    // ---- reference read (AXI4 read master, 64 bytes a beat) ----
    output logic [39:0]   m_axi_araddr,
    output logic [7:0]    m_axi_arlen,
    output logic [2:0]    m_axi_arsize,
    output logic [1:0]    m_axi_arburst,
    output logic [3:0]    m_axi_arcache,
    output logic [2:0]    m_axi_arprot,
    output logic          m_axi_arvalid,
    input  logic          m_axi_arready,
    input  logic [511:0]  m_axi_rdata,
    input  logic [1:0]    m_axi_rresp,
    input  logic          m_axi_rlast,
    input  logic          m_axi_rvalid,
    output logic          m_axi_rready
);
    // ---------------------------------------------------------------- fetch and filter
    logic        rf_req_ready, rf_done;
    logic        pw_we, pw_buf, pw_list; logic [2:0] pw_seq; logic [6:0] pw_row; logic [4:0] pw_w4; logic [1:0] pw_n; logic [16*PW-1:0] pw_dq;
    logic [1:0]  buf_busy;
    logic        h_chase; logic [2:0] h_seq;
    logic        pbuf;                                       // patch buffer of the next accepted request
    logic [2:0]  acc_cnt, done_cnt;
    logic        mc_start, mc_busy, mc_h_ready, mc_done, mc_out_valid, mc_out_valid_b;
    logic [1:0]  mc_tag, mc_done_tag, mc_out_tag, srun;
    logic        run_buf, run_v;
    logic [6:0]  mc_out_x, mc_out_y; logic [47:0] mc_out_pix4, mc_out_pix4_b; logic [3:0] mc_out_mask;
    logic [135:0] unused_pair;
    // the run being started (mc_filt samples these with mc_start)
    logic        st_l, st_warp, st_chase; logic [2:0] st_seq;
    logic [6:0]  st_sw, st_sh; logic [1:0] st_i0, st_i1; logic [9:0] st_xf0, st_yf0; logic [11:0] st_xs, st_ys;
    logic signed [15:0] st_wa, st_wb, st_wg, st_wd; logic [15:0] st_wsx, st_wsy; logic [2:0] st_mode; logic [4:0] st_wt0, st_wt1;
    // ---------------------------------------------------------------- the run queue (two deep)
    logic [1:0]  pq_h, pq_t, pq_n;
    logic        pq_empty, pq_full, qh_ready, run_go, rec_held;
    logic [2:0]  qh_d;
    logic [2:0]  pq_seq [0:1]; logic pq_buf [0:1], pq_par [0:1], pq_chase [0:1], pq_l [0:1], pq_warp [0:1];
    logic [6:0]  pq_sw [0:1], pq_sh [0:1];
    logic [1:0]  pq_i0 [0:1], pq_i1 [0:1], pq_pl [0:1];
    logic [9:0]  pq_xf0 [0:1], pq_yf0 [0:1];
    logic [11:0] pq_xs [0:1], pq_ys [0:1];
    logic signed [15:0] pq_wa [0:1], pq_wb [0:1], pq_wg [0:1], pq_wd [0:1];
    logic [15:0] pq_wsx [0:1], pq_wsy [0:1];
    logic [2:0]  pq_mode [0:1]; logic [4:0] pq_wt0 [0:1], pq_wt1 [0:1];
    logic [PXW-1:0] pq_px [0:1], pq_py [0:1];
    // ---------------------------------------------------------------- run records (four: mc_filt keeps four runs alive)
    logic [PXW-1:0] rq_px [0:3], rq_py [0:3];
    logic [1:0]  rq_pl [0:3]; logic rq_par [0:3];
    assign pq_n = pq_t - pq_h;
    assign pq_empty = (pq_n == 2'd0); assign pq_full = (pq_n == 2'd2);
    assign qh_d = done_cnt - pq_seq[pq_h[0]];
    assign qh_ready = pq_chase[pq_h[0]] || (qh_d == 3'd1) || (qh_d == 3'd2);   // the head's patch is in, or the run chases its writer
    // a record is reused by the fourth start after it: wait while words of the record the start would take are still leaving
    assign rec_held = mc_out_valid && (mc_out_tag == srun);
    assign run_go = !pq_empty && qh_ready && mc_h_ready && !run_v && !rec_held;
    // ---------------------------------------------------------------- strips waiting for the fetch engine
    // The block FSM is one issuer for both lanes, and the fetch engine takes a request only when its address side is
    // idle: without a place to leave a strip the FSM stood in its fetch state until this lane's engine was free, and
    // the other lane got no new strips meanwhile (crop frame, 2026-11-26: block FSM in B_MC_FETCH 82 % of the time,
    // the other lane's run queue empty 85 %, block period 1,242 clocks where each lane alone needs ~750). A strip is
    // accepted into this FIFO at once (SQ entries) and goes to the engine from its head.
    localparam int SQ = 4;
    typedef struct packed {
        logic lst, fmt16; logic [39:0] base; logic [31:0] stride; logic [15:0] lastx, lasty; logic signed [16:0] x0, y0;
        logic [6:0] w, h, r_lo, r_hi;
        logic chase, warp; logic [6:0] sw, sh; logic [1:0] i0, i1; logic [9:0] xf0, yf0; logic [11:0] xs, ys;
        logic signed [15:0] wa, wb, wg, wd; logic [15:0] wsx, wsy; logic [2:0] mode; logic [4:0] wt0, wt1;
        logic [PXW-1:0] px, py; logic [1:0] pl; logic par;
    } strip_t;
    strip_t      sq [SQ];
    strip_t      sh_;                                        // the head
    logic [$clog2(SQ)-1:0] sq_h, sq_t;
    logic [$clog2(SQ):0]   sq_n;
    logic        s_acc;                                      // the head goes to the fetch engine and its run to the queue
    logic [1:0]  sq_pend;
    assign sh_ = sq[sq_h];
    assign req_acc = req_valid && (sq_n != ($clog2(SQ)+1)'(SQ));
    assign s_acc = (sq_n != '0) && rf_req_ready && !pq_full;
    always_comb begin
        sq_pend = 2'b00;
        for (int i = 0; i < SQ; i++)
            if (($clog2(SQ)+1)'(($clog2(SQ))'(i) - sq_h) < sq_n) sq_pend[sq[i].par] = 1'b1;
    end
    assign pend[0] = (pq_n != 2'd0 && pq_par[pq_h[0]] == 1'b0) || (pq_n == 2'd2 && pq_par[!pq_h[0]] == 1'b0) || sq_pend[0];
    assign pend[1] = (pq_n != 2'd0 && pq_par[pq_h[0]] == 1'b1) || (pq_n == 2'd2 && pq_par[!pq_h[0]] == 1'b1) || sq_pend[1];
    always_ff @(posedge clk) begin
        if (rst) begin sq_h <= '0; sq_t <= '0; sq_n <= '0; end
        else begin
            if (req_acc) begin
                sq[sq_t] <= '{lst: req_lst, fmt16: req_fmt16, base: req_base, stride: req_stride, lastx: req_lastx, lasty: req_lasty, x0: req_x0, y0: req_y0,
                              w: req_w, h: req_h, r_lo: req_r_lo, r_hi: req_r_hi, chase: q_chase, warp: q_warp, sw: q_sw, sh: q_sh, i0: q_i0, i1: q_i1,
                              xf0: q_xf0, yf0: q_yf0, xs: q_xs, ys: q_ys, wa: q_wa, wb: q_wb, wg: q_wg, wd: q_wd, wsx: q_wsx, wsy: q_wsy,
                              mode: q_mode, wt0: q_wt0, wt1: q_wt1, px: q_px, py: q_py, pl: q_pl, par: q_par};
                sq_t <= sq_t + 1'b1;
            end
            if (s_acc) sq_h <= sq_h + 1'b1;
            sq_n <= sq_n + ($clog2(SQ)+1)'(req_acc) - ($clog2(SQ)+1)'(s_acc);
        end
    end
    assign start = mc_start; assign start_par = rq_par[mc_tag];
    assign done = mc_done;   assign done_par = rq_par[mc_done_tag];

    always_ff @(posedge clk) begin
        mc_start <= 1'b0;
        if (mc_busy) run_v <= 1'b0;
        if (rf_done) done_cnt <= done_cnt + 3'd1;
        if (run_go) begin
            // the run thread: mc_filt samples st_* with mc_start next cycle, the writer reads the record rq_*[srun]
            mc_start <= 1'b1; run_v <= 1'b1; mc_tag <= srun; srun <= srun + 2'd1;
            run_buf <= pq_buf[pq_h[0]]; st_chase <= pq_chase[pq_h[0]]; st_seq <= pq_seq[pq_h[0]];
            st_l <= pq_l[pq_h[0]]; st_warp <= pq_warp[pq_h[0]]; st_sw <= pq_sw[pq_h[0]]; st_sh <= pq_sh[pq_h[0]];
            st_i0 <= pq_i0[pq_h[0]]; st_i1 <= pq_i1[pq_h[0]]; st_xf0 <= pq_xf0[pq_h[0]]; st_yf0 <= pq_yf0[pq_h[0]]; st_xs <= pq_xs[pq_h[0]]; st_ys <= pq_ys[pq_h[0]];
            st_wa <= pq_wa[pq_h[0]]; st_wb <= pq_wb[pq_h[0]]; st_wg <= pq_wg[pq_h[0]]; st_wd <= pq_wd[pq_h[0]]; st_wsx <= pq_wsx[pq_h[0]]; st_wsy <= pq_wsy[pq_h[0]];
            st_mode <= pq_mode[pq_h[0]]; st_wt0 <= pq_wt0[pq_h[0]]; st_wt1 <= pq_wt1[pq_h[0]];
            rq_px[srun] <= pq_px[pq_h[0]]; rq_py[srun] <= pq_py[pq_h[0]]; rq_pl[srun] <= pq_pl[pq_h[0]]; rq_par[srun] <= pq_par[pq_h[0]];
            pq_h <= pq_h + 2'd1;
        end
        if (s_acc) begin
            // the request is with the fetch engine: its run joins the queue
            acc_cnt <= acc_cnt + 3'd1; pbuf <= !pbuf;
            pq_seq[pq_t[0]] <= acc_cnt; pq_buf[pq_t[0]] <= pbuf; pq_par[pq_t[0]] <= sh_.par; pq_chase[pq_t[0]] <= sh_.chase;
            pq_l[pq_t[0]] <= sh_.lst; pq_warp[pq_t[0]] <= sh_.warp; pq_sw[pq_t[0]] <= sh_.sw; pq_sh[pq_t[0]] <= sh_.sh;
            pq_i0[pq_t[0]] <= sh_.i0; pq_i1[pq_t[0]] <= sh_.i1; pq_xf0[pq_t[0]] <= sh_.xf0; pq_yf0[pq_t[0]] <= sh_.yf0; pq_xs[pq_t[0]] <= sh_.xs; pq_ys[pq_t[0]] <= sh_.ys;
            pq_wa[pq_t[0]] <= sh_.wa; pq_wb[pq_t[0]] <= sh_.wb; pq_wg[pq_t[0]] <= sh_.wg; pq_wd[pq_t[0]] <= sh_.wd; pq_wsx[pq_t[0]] <= sh_.wsx; pq_wsy[pq_t[0]] <= sh_.wsy;
            pq_mode[pq_t[0]] <= sh_.mode; pq_wt0[pq_t[0]] <= sh_.wt0; pq_wt1[pq_t[0]] <= sh_.wt1;
            pq_px[pq_t[0]] <= sh_.px; pq_py[pq_t[0]] <= sh_.py; pq_pl[pq_t[0]] <= sh_.pl;
            pq_t <= pq_t + 2'd1;
        end
        if (rst) begin
            mc_start <= 1'b0; run_v <= 1'b0; pbuf <= 1'b0; srun <= 2'd0; mc_tag <= 2'd0;
            acc_cnt <= 3'd0; done_cnt <= 3'd0; pq_h <= 2'd0; pq_t <= 2'd0;
        end
    end

    ref_fetch #(.PW(PW), .RING(RING)) u_rf (.clk, .rst,
        .req_valid((sq_n != '0) && !pq_full), .req_ready(rf_req_ready), .req_lst(sh_.lst), .req_fmt16(sh_.fmt16), .req_base(sh_.base), .req_stride(sh_.stride),
        .req_lastx(sh_.lastx), .req_lasty(sh_.lasty), .req_x0(sh_.x0), .req_y0(sh_.y0), .req_w(sh_.w), .req_h(sh_.h), .req_r_lo(sh_.r_lo), .req_r_hi(sh_.r_hi), .req_buf(pbuf), .req_seq(acc_cnt), .chase_v(h_chase), .chase_seq(h_seq),
        .buf_busy, .done(rf_done), .err, .perf_flags(rf_pf),
        .m_axi_araddr, .m_axi_arlen, .m_axi_arsize, .m_axi_arburst, .m_axi_arcache, .m_axi_arprot, .m_axi_arvalid, .m_axi_arready,
        .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid, .m_axi_rready,
        .patch_we(pw_we), .patch_buf(pw_buf), .patch_list(pw_list), .patch_seq(pw_seq), .patch_row(pw_row), .patch_w4(pw_w4), .patch_n(pw_n), .patch_dq(pw_dq));
    mc_filt #(.PW(PW)) u_mc (.clk, .rst,
        .patch_we(pw_we), .patch_buf(pw_buf), .patch_list(pw_list), .patch_seq(pw_seq), .patch_row(pw_row), .patch_w4(pw_w4), .patch_n(pw_n), .patch_dq(pw_dq),
        .start(mc_start), .run_tag(mc_tag), .lst(st_l), .rbuf(run_buf), .chase(st_chase), .run_seq(st_seq), .h_chase, .h_seq,
        .w(st_sw), .h(st_sh), .interp_x(st_i1), .interp_y(st_i0),
        .x_frac0(st_xf0), .y_frac0(st_yf0), .x_step(st_xs), .y_step(st_ys),
        .warp(st_warp), .w_alpha(st_wa), .w_beta(st_wb), .w_gamma(st_wg), .w_delta(st_wd), .w_sx4(st_wsx), .w_sy4(st_wsy),
        .bit_depth, .mode(st_mode), .wt0(st_wt0), .wt1(st_wt1), .busy(mc_busy), .h_ready(mc_h_ready), .h_buf_busy(buf_busy), .done(mc_done), .done_tag(mc_done_tag),
        .v_active, .perf_flags(mc_pf), .v_hold,
        .out_wait(1'b0), .out_valid(mc_out_valid), .out_tag(mc_out_tag), .out_x(mc_out_x), .out_y(mc_out_y), .out_pix4(mc_out_pix4), .out_mask(mc_out_mask), .out_pair(unused_pair),
        .direct(1'b1), .out_valid_b(mc_out_valid_b), .out_pix4_b(mc_out_pix4_b));

    // ---------------------------------------------------------------- the writer
    // 4 samples at (strip origin + out_x, + out_y); a 2-wide unit at an x offset of 2 within its 4-group is lane-shifted
    // (the frame memory's 4-wide port is aligned). The second word of a cycle is the next group of the row.
    logic [PXW-1:0] ox;
    logic [1:0]     osh;
    always_comb begin
        ox  = rq_px[mc_out_tag] + PXW'(mc_out_x);
        osh = ox[1:0];
        w_we = mc_out_valid; w_plane = rq_pl[mc_out_tag];
        w_x = FBX'({ox[PXW-1:2], 2'b00}); w_y = FBY'(rq_py[mc_out_tag] + PXW'(mc_out_y));
        w_data = mc_out_pix4 << (12 * osh); w_be = mc_out_mask << osh;
        wb_we = mc_out_valid && mc_out_valid_b; wb_x = FBX'({ox[PXW-1:2], 2'b00}) + FBX'(4); wb_data = mc_out_pix4_b;
    end
`ifndef SYNTHESIS
    always_ff @(posedge clk)
        if (!rst && mc_out_valid_b && !(mc_out_valid && osh == 2'd0)) $error("mc_lane: a second word without an aligned first word");
`endif
endmodule
