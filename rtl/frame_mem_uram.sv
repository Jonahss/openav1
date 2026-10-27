// On-chip frame memory for the FPGA builds (docs/frame-memory.md): the same ports as the frame_mem simulation
// model for NRD = NWR = 2 (the core build's fb0), one UltraRAM copy of the picture, double-pumped. The RAM runs on
// clk2x (2x clk, phase-aligned from the same MMCM): on the clk-aligned clk2x edge its two ports read for the two
// read ports (or the host) and the write requests are captured, on the mid-cycle edge they write for the two
// write ports. Every decoder-side port keeps its one-cycle latency at clk; nothing arbitrates or stalls.
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
    //   E_b (phase 0): port A reads read-port 0, port B reads read-port 1 (or the host when read-port 1 is
    //        idle) — the same edge at which the model samples a request; the write requests are captured into
    //        clk2x registers at this edge too.
    //   E_a (phase 1): port A writes write-port 0, port B writes write-port 1 from the captured requests.
    // So every decoder-side path is a full clk cycle (request flops at one clk edge, sampled at the next: a
    // 2-cycle multicycle path in clk2x terms, see synth/vivado), read data is valid for the whole next cycle
    // (one-cycle latency, as the model), a write issued in cycle t lands mid-cycle t+1 and is visible to the
    // reads sampled at the end of t+1, and a read in the same cycle as a write to the same word sees the old
    // data: exactly the model's semantics. phase = 1 at E_a, 0 at E_b (from the clock generator).
    // The RAM: 72-bit words, 9 bytes of 8 bits; lane l = bytes 2l, 2l+1 (bits 16l .. 16l+15, sample in the low 12),
    // byte 8 unused. Vivado's inference accepts UltraRAM only for full-word no-change writes (probed 2026-10-13:
    // read-first and per-byte enables are both "invalid write mode"), but the URAM288 primitive has byte-write
    // enables, so synthesis instantiates it through the XPM macro and simulation uses the behavioural model
    // below; the gate-level replay (tb/gate) checks the two agree.
    logic           a_en, b_en;
    logic [8:0]     a_bwe, b_bwe;                            // byte-write enables; none set = read
    logic [AW-1:0]  a_addr, b_addr;
    logic [71:0]    a_d, b_d;
    logic [71:0]    a_q, b_q;
`ifdef SYNTHESIS
    xpm_memory_tdpram #(
        .MEMORY_SIZE(NWORD * 72), .MEMORY_PRIMITIVE("ultra"), .CLOCKING_MODE("common_clock"), .MEMORY_INIT_FILE("none"),
        .MEMORY_INIT_PARAM("0"), .USE_MEM_INIT(0), .WAKEUP_TIME("disable_sleep"), .MESSAGE_CONTROL(0),
        .ECC_MODE("no_ecc"), .AUTO_SLEEP_TIME(0), .USE_EMBEDDED_CONSTRAINT(0), .MEMORY_OPTIMIZATION("true"),
        .WRITE_DATA_WIDTH_A(72), .READ_DATA_WIDTH_A(72), .BYTE_WRITE_WIDTH_A(8), .ADDR_WIDTH_A(AW), .READ_RESET_VALUE_A("0"),
        .READ_LATENCY_A(1), .WRITE_MODE_A("no_change"),
        .WRITE_DATA_WIDTH_B(72), .READ_DATA_WIDTH_B(72), .BYTE_WRITE_WIDTH_B(8), .ADDR_WIDTH_B(AW), .READ_RESET_VALUE_B("0"),
        .READ_LATENCY_B(1), .WRITE_MODE_B("no_change"),
        .CASCADE_HEIGHT(4)                                   // 8-deep columns put seven cascade hops (~1.8 ns) on every read
    ) u_ram (
        .clka(clk2x), .ena(a_en), .wea(a_bwe), .addra(a_addr), .dina(a_d), .douta(a_q), .regcea(1'b1), .rsta(1'b0),
        .clkb(clk2x), .enb(b_en), .web(b_bwe), .addrb(b_addr), .dinb(b_d), .doutb(b_q), .regceb(1'b1), .rstb(1'b0),
        .sleep(1'b0), .injectsbiterra(1'b0), .injectdbiterra(1'b0), .injectsbiterrb(1'b0), .injectdbiterrb(1'b0),
        .sbiterra(), .dbiterra(), .sbiterrb(), .dbiterrb()
    );
`else
    logic [71:0] mem [0:NWORD-1];
    always_ff @(posedge clk2x) if (a_en) begin
        if (|a_bwe) begin for (int by = 0; by < 9; by++) if (a_bwe[by]) mem[a_addr][by*8 +: 8] <= a_d[by*8 +: 8]; end
        else a_q <= mem[a_addr];
    end
    always_ff @(posedge clk2x) if (b_en) begin
        if (|b_bwe) begin for (int by = 0; by < 9; by++) if (b_bwe[by]) mem[b_addr][by*8 +: 8] <= b_d[by*8 +: 8]; end
        else b_q <= mem[b_addr];
    end
`endif

    // ---------------------------------------------------------------- requests
    logic [FBX-1:0] rx0, rx1, wx0, wx1;
    logic [FBY-1:0] ry0, ry1, wy0, wy1;
    assign {rx1, rx0} = r_x; assign {ry1, ry0} = r_y;
    assign {wx1, wx0} = w_x; assign {wy1, wy0} = w_y;
    logic host_slot;                                         // the host read takes read-port 1's slot when idle
    assign host_slot = !re[1];
    // write requests captured at E_b (the clk edge), applied at E_a
    logic           wq_we [2];
    logic [8:0]     wq_bwe [2];
    logic [AW-1:0]  wq_addr [2];
    logic [71:0]    wq_d [2];
    always_ff @(posedge clk2x) if (!phase) begin
        wq_we[0] <= we[0]; wq_we[1] <= we[1];
        wq_addr[0] <= waddr(w_plane[1:0], wx0, wy0);
        wq_addr[1] <= waddr(w_plane[3:2], wx1, wy1);
        wq_d[0][71:64] <= '0; wq_d[1][71:64] <= '0; wq_bwe[0][8] <= 1'b0; wq_bwe[1][8] <= 1'b0;
        for (int l = 0; l < 4; l++) begin
            wq_d[0][l*16 +: 16] <= 16'(w_data[l*PW +: PW]);       wq_bwe[0][2*l +: 2] <= {2{w_be[l]}};
            wq_d[1][l*16 +: 16] <= 16'(w_data[(4 + l)*PW +: PW]); wq_bwe[1][2*l +: 2] <= {2{w_be[4 + l]}};
        end
    end
    always_comb begin
        if (!phase) begin                                    // E_b: reads (no byte enabled)
            a_en = re[0];               a_addr = waddr(r_plane[1:0], rx0, ry0); a_d = '0; a_bwe = '0;
            b_en = re[1] || host_slot;  b_d = '0; b_bwe = '0;
            b_addr = host_slot ? waddr(h_plane, h_x, h_y) : waddr(r_plane[3:2], rx1, ry1);
        end else begin                                       // E_a: writes
            a_en = wq_we[0] && (|wq_bwe[0]); a_addr = wq_addr[0]; a_d = wq_d[0]; a_bwe = wq_bwe[0];
            b_en = wq_we[1] && (|wq_bwe[1]); b_addr = wq_addr[1]; b_d = wq_d[1]; b_bwe = wq_bwe[1];
        end
    end

    // ---------------------------------------------------------------- results into the clk domain
    // a_q / b_q are updated at E_b (reads) and hold through E_a (the writes are "no change"), so they are the
    // model's rdata for the whole next clk cycle.
    logic [71:0] rq0, rq1;
    assign rq0 = a_q; assign rq1 = b_q;
    logic [1:0]  rx0l, rx1l, hxl;
    always_ff @(posedge clk) begin
        rx0l <= rx0[1:0]; rx1l <= rx1[1:0]; hxl <= h_x[1:0];
        h_ok <= host_slot;                                   // sampled with the request: read-port 1 idle at this edge
    end
    always_comb begin
        rdata4[0*4*PW +: 4*PW] = {rq0[3*16 +: PW], rq0[2*16 +: PW], rq0[1*16 +: PW], rq0[0*16 +: PW]};
        rdata4[1*4*PW +: 4*PW] = {rq1[3*16 +: PW], rq1[2*16 +: PW], rq1[1*16 +: PW], rq1[0*16 +: PW]};
        rdata[0*PW +: PW] = rq0[32'(rx0l)*16 +: PW];
        rdata[1*PW +: PW] = rq1[32'(rx1l)*16 +: PW];
        h_rdata = rq1[32'(hxl)*16 +: PW];
        h_rdata4 = {rq1[3*16 +: PW], rq1[2*16 +: PW], rq1[1*16 +: PW], rq1[0*16 +: PW]};
    end
endmodule
