// twin_player.sv (K60-110): plays read and write bursts from two text files into the DDR4 controller's AXI port and
// measures what the memory path does with them. One event per line:
//   reads file:   <stream> <gap> <address hex> <len>     (len = AXI arlen, 64-byte beats)
//   writes file:  <gap> <address hex> <len>
// gap = clocks between the acceptance of the previous address of that file and this one being offered. At most OUT
// read bursts are outstanding (+OUT=n, default 16). Every read beat is accepted at once; write data follows its
// address at once. At the end: one RESULT line per stream and one for the writes.
`timescale 1ps/1ps
module twin_player #(parameter string TAG = "", parameter int DW = 512) (
    input  wire         clk,
    input  wire         rst,
    input  wire         go,
    output reg  [33:0]  araddr,
    output reg  [7:0]   arlen,
    output reg          arvalid,
    input  wire         arready,
    input  wire         rvalid,
    input  wire         rlast,
    input  wire [63:0]  rdata_lo,                      // the low 64 bits of the read data (for +CHECK<TAG>=1)
    output wire         rready,
    output reg  [33:0]  awaddr,
    output reg  [7:0]   awlen,
    output reg          awvalid,
    input  wire         awready,
    output wire [DW-1:0] wdata,
    output wire [DW/8-1:0] wstrb,
    output reg          wlast,
    output reg          wvalid,
    input  wire         wready,
    input  wire         bvalid,
    output wire         bready,
    input  wire         other_done,                    // 1 when there is no other player, or it has finished
    output reg          done
);
    initial done = 1'b0;
    assign rready = 1'b1; assign bready = 1'b1; assign wstrb = {(DW/8){1'b1}};
    // write data: every 64-bit word of a beat carries the beat's own address, so a read can be checked (+CHECK<TAG>=1:
    // every beat read must carry its address; only for reads of memory this or another player wrote before)
    reg [63:0] cyc; reg [63:0] w_beat_addr; assign wdata = {(DW/64){w_beat_addr}};
    integer check; longint n_bad; reg [63:0] r_beat_addr [0:1023];
    integer fr, fw, out_max, rc;
    string rname, wname;
    // ---- reads
    integer r_stream, r_gap, r_len, r_cnt; reg [63:0] r_addr; reg r_have, r_eof;
    integer outstanding, own [0:1023], oh, ot;         // owners of the outstanding bursts, in order
    longint t_acc [0:1023];
    longint n_rd [0:7], w_rd [0:7], lat_rd [0:7], beats_rd [0:7], first_cyc, last_cyc;
    integer s, idle_cnt;
    // ---- writes
    integer w_gap, w_len, w_cnt, w_left; reg [63:0] w_addr; reg w_have, w_eof, w_data_phase;
    longint n_wr, w_wr, n_b;
    initial begin
        // (the plusarg names are spelled out per tag: a format built from the TAG parameter crashes xsim 2025.2.1)
        if (TAG == "B") begin
            if (!$value$plusargs("READSB=%s", rname)) rname = "readsB.txt";
            if (!$value$plusargs("WRITESB=%s", wname)) wname = "writesB.txt";
            if (!$value$plusargs("OUTB=%d", out_max)) out_max = 16;
            if (!$value$plusargs("CHECKB=%d", check)) check = 0;
        end else begin
            if (!$value$plusargs("READS=%s", rname)) rname = "reads.txt";
            if (!$value$plusargs("WRITES=%s", wname)) wname = "writes.txt";
            if (!$value$plusargs("OUT=%d", out_max)) out_max = 16;
            if (!$value$plusargs("CHECK=%d", check)) check = 0;
        end
        n_bad = 0;
        fr = $fopen(rname, "r"); fw = $fopen(wname, "r");
        r_eof = (fr == 0); w_eof = (fw == 0);
        $display("TWIN%s reads %s (%0d) writes %s (%0d) OUT %0d CHECK %0d", TAG, rname, fr, wname, fw, out_max, check);
        for (s = 0; s < 8; s = s + 1) begin n_rd[s] = 0; w_rd[s] = 0; lat_rd[s] = 0; beats_rd[s] = 0; end
        n_wr = 0; w_wr = 0; n_b = 0; first_cyc = -1; last_cyc = 0;
    end
    always @(posedge clk) begin
        if (rst || !go) begin
            cyc <= 0; arvalid <= 1'b0; awvalid <= 1'b0; wvalid <= 1'b0; wlast <= 1'b0; r_have = 1'b0; w_have = 1'b0;
            r_cnt = 0; w_cnt = 0; outstanding = 0; oh = 0; ot = 0; w_data_phase = 1'b0; idle_cnt = 0;
        end else begin
            cyc <= cyc + 1;
            if (first_cyc < 0) first_cyc = cyc;
            // read data
            if (rvalid) begin
                beats_rd[own[oh]] = beats_rd[own[oh]] + 1;
                if (check != 0 && rdata_lo !== r_beat_addr[oh]) begin
                    n_bad = n_bad + 1;
                    if (n_bad <= 5) $display("TWIN%s BAD beat: expected %h got %h", TAG, r_beat_addr[oh], rdata_lo);
                end
                r_beat_addr[oh] = r_beat_addr[oh] + (DW / 8);
                if (rlast) begin
                    lat_rd[own[oh]] = lat_rd[own[oh]] + (cyc - t_acc[oh]); oh = (oh + 1) % 1024;
                    outstanding = outstanding - 1; last_cyc = cyc;
                end
            end
            // read address
            if (arvalid && arready) begin
                arvalid <= 1'b0; n_rd[r_stream] = n_rd[r_stream] + 1; own[ot] = r_stream; t_acc[ot] = cyc; r_beat_addr[ot] = {30'b0, araddr}; ot = (ot + 1) % 1024;
                outstanding = outstanding + 1; r_have = 1'b0; r_cnt = 0;
            end else if (arvalid) w_rd[r_stream] = w_rd[r_stream] + 1;
            else begin
                if (!r_have && !r_eof) begin
                    rc = $fscanf(fr, "%d %d %h %d\n", r_stream, r_gap, r_addr, r_len);
                    if (rc == 4) r_have = 1'b1; else r_eof = 1'b1;
                end
                if (r_have) begin
                    if (r_cnt >= r_gap && outstanding < out_max) begin arvalid <= 1'b1; araddr <= r_addr[33:0]; arlen <= r_len[7:0]; end
                    else if (r_cnt >= r_gap) w_rd[7] = w_rd[7] + 1;      // held by the outstanding limit, not by the memory
                    r_cnt = r_cnt + 1;
                end
            end
            // write address, then its data
            if (bvalid) n_b = n_b + 1;
            if (w_data_phase) begin
                if (wvalid && wready) begin
                    if (w_left == 0) begin wvalid <= 1'b0; wlast <= 1'b0; w_data_phase = 1'b0; w_have = 1'b0; w_cnt = 0; end
                    else begin w_left = w_left - 1; wlast <= (w_left == 0); w_beat_addr <= w_beat_addr + (DW / 8); end
                end
            end else if (awvalid && awready) begin
                awvalid <= 1'b0; n_wr = n_wr + 1; w_data_phase = 1'b1; wvalid <= 1'b1; w_left = w_len; wlast <= (w_len == 0); w_beat_addr <= {30'b0, awaddr};
            end else if (awvalid) w_wr = w_wr + 1;
            else begin
                if (!w_have && !w_eof) begin
                    rc = $fscanf(fw, "%d %h %d\n", w_gap, w_addr, w_len);
                    if (rc == 3) w_have = 1'b1; else w_eof = 1'b1;
                end
                if (w_have) begin
                    if (w_cnt >= w_gap) begin awvalid <= 1'b1; awaddr <= w_addr[33:0]; awlen <= w_len[7:0]; end
                    w_cnt = w_cnt + 1;
                end
            end
            // watchdog: nothing accepted and nothing answered for 6,000 clocks while work is left
            if ((arvalid && arready) || rvalid || (awvalid && awready) || bvalid || (wvalid && wready)) idle_cnt = 0;
            else if (arvalid || awvalid || w_data_phase || outstanding != 0 || n_b != n_wr) idle_cnt = idle_cnt + 1;   // only while something is pending
            if (idle_cnt == 6000 && !done) begin
                $display("TWIN%s_STUCK at clock %0d: arvalid %0d arready %0d outstanding %0d reads accepted %0d beats %0d r_have %0d r_eof %0d awvalid %0d",
                         TAG, cyc, arvalid, arready, outstanding, n_rd[0] + n_rd[1], beats_rd[0] + beats_rd[1], r_have, r_eof, awvalid);
                $finish;
            end
            // the end
            if (done) begin if (other_done) begin $display("TWIN_DONE"); $finish; end end
            else if (r_eof && w_eof && !r_have && !w_have && outstanding == 0 && !arvalid && !awvalid && !w_data_phase && n_b == n_wr) begin
                for (s = 0; s < 7; s = s + 1) if (n_rd[s] != 0)
                    $display("RESULT%s stream %0d: %0d reads, %0d beats, address waits %0d clocks (%.2f a read), latency %.1f clocks a read",
                             TAG, s, n_rd[s], beats_rd[s], w_rd[s], 1.0 * w_rd[s] / n_rd[s], 1.0 * lat_rd[s] / n_rd[s]);
                $display("RESULT%s reads held by the outstanding limit (%0d): %0d clocks", TAG, out_max, w_rd[7]);
                $display("RESULT%s writes: %0d bursts, address waits %0d clocks", TAG, n_wr, w_wr);
                if (check != 0) $display("RESULT%s read check: %0d bad beats of %0d", TAG, n_bad, beats_rd[0] + beats_rd[1]);
                $display("RESULT%s clocks from first to last read: %0d", TAG, last_cyc - first_cyc);
                done <= 1'b1;
                if (other_done) begin $display("TWIN_DONE"); $finish; end
            end
        end
    end
endmodule

// twin_clocks: the two clocks the card has besides the controller's own (the decoder clock, period in ps by +DEC_PS=,
// default 12500 = 80 MHz; XDMA's 250 MHz), the interconnect's reset, and `go` for the player: calibration done, seen
// in the decoder clock domain, plus 256 clocks.
module twin_clocks (input wire calib, output reg clk_dec, output reg clk_x, output wire x_aresetn, output wire go, output wire rst_dec);
    integer half;
    initial begin
        if (!$value$plusargs("DEC_PS=%d", half)) half = 12500;
        half = half / 2; clk_dec = 1'b0; clk_x = 1'b0;
        $display("TWIN decoder clock half period %0d ps", half);
        forever #(half) clk_dec = ~clk_dec;
    end
    always #2000 clk_x = ~clk_x;
    reg [8:0] rc = 9'd0; reg x1 = 1'b0, x2 = 1'b0;
    always @(posedge clk_x) begin x1 <= calib; x2 <= x1; if (x2 && !rc[8]) rc <= rc + 1'b1; end
    assign x_aresetn = rc[8];
    reg [11:0] gc = 12'd0; reg c1 = 1'b0, c2 = 1'b0;
    always @(posedge clk_dec) begin c1 <= calib; c2 <= c1; if (c2 && !gc[11]) gc <= gc + 1'b1; end
    assign go = gc[11]; assign rst_dec = !c2;
endmodule
