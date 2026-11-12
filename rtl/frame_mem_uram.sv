// On-chip frame memory for the FPGA builds (docs/frame-memory.md): the same ports as the frame_mem simulation
// model for NRD = NWR = 2 (the core build's fb0), one UltraRAM copy of the picture, double-pumped. The RAM runs on
// clk2x (2x clk, phase-aligned from the same MMCM). Two-cycle read (2026-10-31): a request presented in cycle k is
// captured into clk2x registers at the clk-aligned edge E_b(k); the UltraRAMs read at the mid-cycle edge E_a(k+1)
// (a half cycle of pure routing from replicable registers to the RAM enables), their own output registers capture
// the read at E_b(k+1) (OREG, clock-enabled at E_b only, so they hold through the write) and the data is valid for
// the whole of cycle k+2: the output mux and the consumer's logic have a full clk cycle. Writes captured at E_b(k)
// land at E_b(k+1). Before this the RAMs read at E_b and a fabric register captured at E_a, which left half a clk
// cycle for the mux and every hop into a consumer: -0.214 ns on the 2026-10-31 stage B image, -0.354 ns with sb128
// (twice the RAMs), placement-bound (4.3 to 4.9 ns of routing per build). Nothing arbitrates or stalls.
//
// Word = one aligned group of four samples, 4 x 16-bit lanes in a 72-bit URAM word (a sample in the low 12 bits
// of its lane; the lane is two URAM bytes, so the lane write enables are byte-write enables; byte 8 unused). Address = plane base +
// y * (W/4) + x/4 with W = 1 << FBX: plane 0 holds (1 << FBY) rows, planes 1 and 2 (4:2:0) a quarter each.
module frame_mem_uram #(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int PW = 12,
    parameter int FRAME_WIN = 0,                            // 0: the whole picture; N: a window of N superblock rows
    parameter int SB128 = 0                                 // 1: the window's slots hold 128-line superblock rows (sb128 streams; twice the words)
) (
    input  logic                clk,
    input  logic                clk2x,
    input  logic                phase,      // 1 at the mid-cycle clk2x edge, 0 at the clk-aligned one (clock generator)
    input  logic                sb128,      // the stream's superblock size (SB128 builds: 128-line rows; else ignored)
    // read ports (2)
    input  logic [1:0]          re,
    input  logic [3:0]          r_plane,
    input  logic [2*FBX-1:0]    r_x,
    input  logic [2*FBY-1:0]    r_y,
    output logic [2*PW-1:0]     rdata,
    output logic [2*4*PW-1:0]   rdata4,
    // write ports (2)
    input  logic [1:0]          we,
    input  logic [7:0]          w_be,
    input  logic [3:0]          w_plane,
    input  logic [2*FBX-1:0]    w_x,
    input  logic [2*FBY-1:0]    w_y,
    input  logic [2*4*PW-1:0]   w_data,
    // host read port (served in read-port 1's slot when that port is idle, i.e. whenever the decoder is idle)
    input  logic [1:0]          h_plane,
    input  logic [FBX-1:0]      h_x,
    input  logic [FBY-1:0]      h_y,
    output logic [PW-1:0]       h_rdata,
    output logic [4*PW-1:0]     h_rdata4,        // the four samples of h_x's aligned group (x & ~3 .. +3), lane 0 lowest
    output logic                h_ok             // the host read sampled at the last clk edge was served (read-port 1 was idle)
);
    localparam int WPR   = 1 << (FBX - 2);                   // words per luma row
    // FRAME_WIN > 0: a window of FRAME_WIN superblock rows (64 lines) of the full width instead of the whole picture,
    // the lines addressed by superblock row modulo FRAME_WIN (docs/frame-memory.md, window version); the
    // y coordinate keeps its FBY bits
    localparam int WLINES = (FRAME_WIN > 0) ? FRAME_WIN * (SB128 != 0 ? 128 : 64) : (1 << FBY);
    localparam int NW0   = WPR * WLINES;                     // luma words
    localparam int NW1   = NW0 >> 2;                         // chroma plane words (4:2:0)
    localparam int NWORD = NW0 + 2 * NW1;
    localparam int AW    = $clog2(NWORD);

    // The window line of picture line y: luma superblock rows are 64 lines, chroma (4:2:0) 32, slot = row mod FRAME_WIN,
    // as a table over the superblock row (two LUT levels) rather than a modulo. The 80 MHz 1080p window build's worst
    // path (-1.03 ns, 39 logic levels, 2026-10-17) ran from the reconstruction stage's transform-size register into the
    // UltraRAM address through this function and the plane arithmetic below, synthesised as a divider and adders.
    localparam int NROW = 1 << (FBY - 5);                    // superblock rows a y coordinate can address (chroma rows are 32 lines)
    // SB128 builds: an sb128 stream's rows are 128 luma / 64 chroma lines, slot pitch 128 / 64 (the window holds
    // FRAME_WIN of them: WLINES); an sb64 stream in the same build keeps the 64 / 32 pitch and uses half the words.
    function automatic logic [FBY-1:0] wline(input logic [1:0] p, input logic [FBY-1:0] y);
        logic [FBY-6:0] row;
        logic [FBY-1:0] slot;
        logic big;
        if (FRAME_WIN == 0) return y;
        big = (SB128 != 0) && sb128;
        row = (p == 2'd0) ? (big ? (FBY-5)'(y >> 7) : (FBY-5)'(y >> 6)) : (big ? (FBY-5)'(y >> 6) : (FBY-5)'(y >> 5));
        slot = '0;
        for (int r = 0; r < NROW; r++) if (row == (FBY-5)'(r)) slot = FBY'(r % FRAME_WIN);
        if (p == 2'd0) return big ? FBY'((slot << 7) | (y & FBY'(127))) : FBY'((slot << 6) | (y & FBY'(63)));
        return big ? FBY'((slot << 6) | (y & FBY'(63))) : FBY'((slot << 5) | (y & FBY'(31)));
    endfunction
    // Address = plane base + line * words-per-row + x / 4. The words-per-row are powers of two and x / 4 is below them
    // (chroma planes are half as wide), so line and group concatenate; only the plane base is added, and its low bits
    // are zero.
    function automatic logic [AW-1:0] waddr(input logic [1:0] p, input logic [FBX-1:0] x, input logic [FBY-1:0] y);
        logic [AW-1:0] a;
        logic [FBY-1:0] yw;
        yw = wline(p, y);
        case (p)
            2'd0:    a = (AW'(yw) << (FBX - 2)) | AW'(x[FBX-1:2]);
            2'd1:    a = AW'(NW0) + ((AW'(yw) << (FBX - 3)) | AW'(x[FBX-2:2]));
            default: a = AW'(NW0 + NW1) + ((AW'(yw) << (FBX - 3)) | AW'(x[FBX-2:2]));
        endcase
        return a;
    endfunction

    // ---------------------------------------------------------------- the RAM (clk2x domain)
    // Slot schedule per clk cycle (clk2x edges: E_b aligned with the clk edge that ends the cycle, E_a mid-cycle):
    //   E_b (phase 0): the read requests (read-port 0, read-port 1 or the host) and the write requests of the cycle
    //        that ends are captured into clk2x registers (rq / wq); the write requests captured one cycle earlier are
    //        applied (port A write-port 0, port B write-port 1); the output registers capture the mid-cycle read.
    //   E_a (phase 1): port A reads for read-port 0, port B for read-port 1 (or the host), from rq.
    // Read latency two clk cycles (the model's 2-cycle frame_mem); a write issued in cycle t lands at the end of
    // t+1 and is visible to the reads issued in t+1 (sampled at the end of t+1, performed mid t+2); a read issued
    // in the same cycle as a write to the same word sees the old data: the same ordering as the one-cycle version.
    // phase = 1 at E_a, 0 at E_b (from the clock generator).
    // The RAM: 72-bit words, 9 bytes of 8 bits; lane l = bytes 2l, 2l+1 (bits 16l .. 16l+15, sample in the low 12),
    // byte 8 unused. Vivado's inference accepts UltraRAM only for full-word no-change writes (probed 2026-10-13:
    // read-first and per-byte enables are both "invalid write mode"), but the URAM288 primitive has byte-write
    // enables, so synthesis instantiates the primitives directly (below) and simulation uses the behavioural model;
    // the gate-level replay (tb/gate) checks the two agree.
    localparam int NU   = (NWORD + 4095) / 4096;              // UltraRAMs (4096 x 72 each)
    localparam int SW   = (NU > 1) ? $clog2(NU) : 1;
    logic           a_en, b_en;
    logic [8:0]     a_bwe, b_bwe;                            // byte-write enables; none set = read
    logic [AW-1:0]  a_addr, b_addr;
    logic [71:0]    a_d, b_d;
    logic [71:0]    a_q, b_q;
    // read and write requests captured at E_b (the clk edge); reads applied at E_a, writes at the next E_b
    logic           wq_we [2];
    logic [8:0]     wq_bwe [2];
    logic [AW-1:0]  wq_addr [2];
    logic [71:0]    wq_d [2];
    logic           rq_en [2];
    logic [AW-1:0]  rq_addr [2];
    logic [SW-1:0]  rq_sel [2];
    // The module's own phase (timing rung 6): the request capture (rq_* / wq_*, both builds) and the read select of the UltraRAM build were enabled by
    // the clock generator's phase register directly, one clk2x period (5 ns) for a net from the shell to registers in
    // every frame memory: 187 of the 191 failing endpoints of the ring-16 100 MHz image of 6fbf579 (CE pins, one logic
    // level, 97 % wire). Two clk2x stages equal phase exactly (it alternates every cycle) and give the route two hops.
    (* KEEP = "true" *) logic phm1, phm2;
    always_ff @(posedge clk2x) begin phm1 <= phase; phm2 <= phm1; end
`ifdef SYNTHESIS
    // The UltraRAMs instantiated directly (2026-10-27), not through the XPM macro: NU independent URAM288s (no cascade),
    // each enabled by its own address decode, and an output mux whose select is a register loaded at E_b only. With
    // the XPM the column select came from the URAMs' RDACCESS flags through a LUT and a hold register, logic that
    // transitions at E_a (the write edge): a read-path launch the clk <-> clk2x multicycle exception tells the timer
    // to ignore. Whether such a launch corrupted a clk-side capture depended on path lengths: the 100 MHz images were
    // fine, two 50 MHz images decoded wrongly, and the same RTL with the exceptions removed decoded bit-exact. Here
    // nothing on the read path changes at E_a: the URAMs hold DOUT through a write (no-change), the select changes
    // at E_b, so the data launched at E_b is valid for the whole clk cycle and the exception is honest.
    logic [71:0]    a_dout [NU], b_dout [NU];
    // Per-UltraRAM phase and request mux (2026-11-03): one phase register and one request mux driving the EN / ADDR /
    // BWE / DIN pins of up to 120 UltraRAMs spread over the die missed timing by ~1 ns at 100 MHz (97 % wire; the
    // earlier images closed the same path with +0.001 ns). phase alternates every clk2x cycle, so a two-stage copy
    // equals it exactly and gives the placer two hops; the mux sits next to its UltraRAM, the captured requests
    // (rq_* / wq_*) fan out as registers, which the placer replicates. KEEP stops synthesis from merging the copies.
    // The read data: each UltraRAM's own output register (OREG) captures its array output at E_b, the edge after the
    // mid-cycle read, and only there (OREG_CE = the E_b edge), so it holds the read through the write at that same
    // edge and for the whole clk cycle; the select of the RAM that was read (a_sel_q, loaded at the same E_b) and
    // the output mux then have a full cycle to the consumers' registers (pre_opt.tcl excepts URAM -> clk as 2 clk2x).
    // (Before 2026-10-31 a fabric register captured DOUT at E_a and had half a cycle to the consumers.)
    logic [SW-1:0]  a_sel_q, b_sel_q;
    always_ff @(posedge clk2x) if (!phm2) begin a_sel_q <= rq_sel[0]; b_sel_q <= rq_sel[1]; end
    assign a_q = a_dout[a_sel_q];
    assign b_q = b_dout[b_sel_q];
    for (genvar k = 0; k < NU; k++) begin : g_uram
        (* KEEP = "true" *) logic ph1, ph2;                  // local phase: two clk2x stages = the same value
        always_ff @(posedge clk2x) begin ph1 <= phase; ph2 <= ph1; end
        logic           ka_en, kb_en;
        logic [8:0]     ka_bwe, kb_bwe;
        logic [11:0]    ka_addr, kb_addr;
        logic [71:0]    ka_d, kb_d;
        always_comb begin
            if (ph2) begin                                   // E_a: the captured reads (no byte enabled)
                ka_en = rq_en[0] && (((NU > 1) ? SW'(rq_addr[0] >> 12) : '0) == SW'(k)); ka_addr = rq_addr[0][11:0]; ka_d = '0; ka_bwe = '0;
                kb_en = rq_en[1] && (((NU > 1) ? SW'(rq_addr[1] >> 12) : '0) == SW'(k)); kb_addr = rq_addr[1][11:0]; kb_d = '0; kb_bwe = '0;
            end else begin                                   // E_b: the writes captured a cycle ago
                ka_en = wq_we[0] && (|wq_bwe[0]) && (((NU > 1) ? SW'(wq_addr[0] >> 12) : '0) == SW'(k)); ka_addr = wq_addr[0][11:0]; ka_d = wq_d[0]; ka_bwe = wq_bwe[0];
                kb_en = wq_we[1] && (|wq_bwe[1]) && (((NU > 1) ? SW'(wq_addr[1] >> 12) : '0) == SW'(k)); kb_addr = wq_addr[1][11:0]; kb_d = wq_d[1]; kb_bwe = wq_bwe[1];
            end
        end
        URAM288 #(
            .CASCADE_ORDER_A("NONE"), .CASCADE_ORDER_B("NONE"), .OREG_A("TRUE"), .OREG_B("TRUE"), .REG_CAS_A("FALSE"), .REG_CAS_B("FALSE"),
            .BWE_MODE_A("PARITY_INDEPENDENT"), .BWE_MODE_B("PARITY_INDEPENDENT"), .EN_AUTO_SLEEP_MODE("FALSE"),
            .IREG_PRE_A("FALSE"), .IREG_PRE_B("FALSE"), .RST_MODE_A("SYNC"), .RST_MODE_B("SYNC"), .USE_EXT_CE_A("FALSE"), .USE_EXT_CE_B("FALSE")
        ) u (
            .CLK(clk2x), .SLEEP(1'b0), .RST_A(1'b0), .RST_B(1'b0),
            .EN_A(ka_en), .RDB_WR_A(|ka_bwe), .ADDR_A({11'd0, ka_addr}), .BWE_A(ka_bwe), .DIN_A(ka_d), .DOUT_A(a_dout[k]),
            .EN_B(kb_en), .RDB_WR_B(|kb_bwe), .ADDR_B({11'd0, kb_addr}), .BWE_B(kb_bwe), .DIN_B(kb_d), .DOUT_B(b_dout[k]),
            .OREG_CE_A(!ph2), .OREG_CE_B(!ph2), .OREG_ECC_CE_A(1'b0), .OREG_ECC_CE_B(1'b0),
            .INJECT_SBITERR_A(1'b0), .INJECT_DBITERR_A(1'b0), .INJECT_SBITERR_B(1'b0), .INJECT_DBITERR_B(1'b0),
            .CAS_IN_ADDR_A(23'd0), .CAS_IN_ADDR_B(23'd0), .CAS_IN_BWE_A(9'd0), .CAS_IN_BWE_B(9'd0), .CAS_IN_DBITERR_A(1'b0), .CAS_IN_DBITERR_B(1'b0),
            .CAS_IN_DIN_A(72'd0), .CAS_IN_DIN_B(72'd0), .CAS_IN_DOUT_A(72'd0), .CAS_IN_DOUT_B(72'd0), .CAS_IN_EN_A(1'b0), .CAS_IN_EN_B(1'b0),
            .CAS_IN_RDACCESS_A(1'b0), .CAS_IN_RDACCESS_B(1'b0), .CAS_IN_RDB_WR_A(1'b0), .CAS_IN_RDB_WR_B(1'b0), .CAS_IN_SBITERR_A(1'b0), .CAS_IN_SBITERR_B(1'b0),
            .CAS_OUT_ADDR_A(), .CAS_OUT_ADDR_B(), .CAS_OUT_BWE_A(), .CAS_OUT_BWE_B(), .CAS_OUT_DBITERR_A(), .CAS_OUT_DBITERR_B(),
            .CAS_OUT_DIN_A(), .CAS_OUT_DIN_B(), .CAS_OUT_DOUT_A(), .CAS_OUT_DOUT_B(), .CAS_OUT_EN_A(), .CAS_OUT_EN_B(),
            .CAS_OUT_RDACCESS_A(), .CAS_OUT_RDACCESS_B(), .CAS_OUT_RDB_WR_A(), .CAS_OUT_RDB_WR_B(), .CAS_OUT_SBITERR_A(), .CAS_OUT_SBITERR_B(),
            .DBITERR_A(), .DBITERR_B(), .SBITERR_A(), .SBITERR_B(), .RDACCESS_A(), .RDACCESS_B()
        );
    end
`else
    logic [71:0] mem [0:NWORD-1];
    logic [71:0] a_q_i, b_q_i;                               // the mid-cycle read, as the URAM's array output
    always_ff @(posedge clk2x) if (a_en) begin
        if (|a_bwe) begin for (int by = 0; by < 9; by++) if (a_bwe[by]) mem[a_addr][by*8 +: 8] <= a_d[by*8 +: 8]; end
        else a_q_i <= mem[a_addr];
    end
    always_ff @(posedge clk2x) if (b_en) begin
        if (|b_bwe) begin for (int by = 0; by < 9; by++) if (b_bwe[by]) mem[b_addr][by*8 +: 8] <= b_d[by*8 +: 8]; end
        else b_q_i <= mem[b_addr];
    end
    always_ff @(posedge clk2x) if (!phase) begin a_q <= a_q_i; b_q <= b_q_i; end   // the OREG: captured at E_b, held
`endif

    // ---------------------------------------------------------------- requests
    logic [FBX-1:0] rx0, rx1, wx0, wx1;
    logic [FBY-1:0] ry0, ry1, wy0, wy1;
    assign {rx1, rx0} = r_x; assign {ry1, ry0} = r_y;
    assign {wx1, wx0} = w_x; assign {wy1, wy0} = w_y;
    logic host_slot;                                         // the host read takes read-port 1's slot when idle
    logic h_ok1;
    assign host_slot = !re[1];
    always_ff @(posedge clk2x) if (!phm2) begin
        wq_we[0] <= we[0]; wq_we[1] <= we[1];
        rq_en[0] <= re[0];               rq_addr[0] <= waddr(r_plane[1:0], rx0, ry0);
        rq_en[1] <= re[1] || host_slot;  rq_addr[1] <= host_slot ? waddr(h_plane, h_x, h_y) : waddr(r_plane[3:2], rx1, ry1);
        rq_sel[0] <= (NU > 1) ? SW'(waddr(r_plane[1:0], rx0, ry0) >> 12) : '0;
        rq_sel[1] <= (NU > 1) ? SW'((host_slot ? waddr(h_plane, h_x, h_y) : waddr(r_plane[3:2], rx1, ry1)) >> 12) : '0;
        wq_addr[0] <= waddr(w_plane[1:0], wx0, wy0);
        wq_addr[1] <= waddr(w_plane[3:2], wx1, wy1);
        wq_d[0][71:64] <= '0; wq_d[1][71:64] <= '0; wq_bwe[0][8] <= 1'b0; wq_bwe[1][8] <= 1'b0;
        for (int l = 0; l < 4; l++) begin
            wq_d[0][l*16 +: 16] <= 16'(w_data[l*PW +: PW]);       wq_bwe[0][2*l +: 2] <= {2{w_be[l]}};
            wq_d[1][l*16 +: 16] <= 16'(w_data[(4 + l)*PW +: PW]); wq_bwe[1][2*l +: 2] <= {2{w_be[4 + l]}};
        end
    end
    always_comb begin
        if (phase) begin                                     // E_a: the captured reads (no byte enabled)
            a_en = rq_en[0]; a_addr = rq_addr[0]; a_d = '0; a_bwe = '0;
            b_en = rq_en[1]; b_addr = rq_addr[1]; b_d = '0; b_bwe = '0;
        end else begin                                       // E_b: the writes captured a cycle ago
            a_en = wq_we[0] && (|wq_bwe[0]); a_addr = wq_addr[0]; a_d = wq_d[0]; a_bwe = wq_bwe[0];
            b_en = wq_we[1] && (|wq_bwe[1]); b_addr = wq_addr[1]; b_d = wq_d[1]; b_bwe = wq_bwe[1];
        end
    end

    // ---------------------------------------------------------------- results into the clk domain
    // a_q / b_q are the output registers: loaded at E_b two cycles after the request (the mid-cycle read of the
    // cycle after), held through the writes, so they are the model's rdata for the whole of that cycle.
    logic [71:0] rq0, rq1;
    assign rq0 = a_q; assign rq1 = b_q;
    logic [1:0]  rx0l, rx1l, hxl, rx0l2, rx1l2, hxl2;       // the lane within the word, with the data (two cycles)
    always_ff @(posedge clk) begin
        rx0l <= rx0[1:0]; rx1l <= rx1[1:0]; hxl <= h_x[1:0]; rx0l2 <= rx0l; rx1l2 <= rx1l; hxl2 <= hxl;
        h_ok1 <= host_slot; h_ok <= h_ok1;                   // sampled with the request (read-port 1 idle), with the data
    end
    always_comb begin
        rdata4[0*4*PW +: 4*PW] = {rq0[3*16 +: PW], rq0[2*16 +: PW], rq0[1*16 +: PW], rq0[0*16 +: PW]};
        rdata4[1*4*PW +: 4*PW] = {rq1[3*16 +: PW], rq1[2*16 +: PW], rq1[1*16 +: PW], rq1[0*16 +: PW]};
        rdata[0*PW +: PW] = rq0[32'(rx0l2)*16 +: PW];
        rdata[1*PW +: PW] = rq1[32'(rx1l2)*16 +: PW];
        h_rdata = rq1[32'(hxl2)*16 +: PW];
        h_rdata4 = {rq1[3*16 +: PW], rq1[2*16 +: PW], rq1[1*16 +: PW], rq1[0*16 +: PW]};
    end
endmodule
