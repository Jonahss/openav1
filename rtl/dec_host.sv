// Host interface for the FPGA builds: an AXI4-Lite register file around dec_top, so that software on the PS
// (or a JTAG master) drives the decoder the way tb/test_dec_top.py does: headers and CDF defaults written
// into registers, tile bytes pushed into a FIFO, stage starts pulsed, done bits polled, the picture read back
// one sample at a time through the frame memory's host port. Everything runs on clk (the AXI clock is the
// decoder clock; the PS side's own clock domain crossing is in its interconnect). Register map (byte offsets,
// 32-bit words; the words of a wide field are little-endian: word 0 = bits 31:0):
//   0x000 ID        R   32'hA71D_0001
//   0x004 CFG       R   {8'd0, FBX[7:0], FBY[7:0], ITX_LANES[3:0], 3'd0, FRAME_URAM}
//   0x008 CTRL      RW  bit 0 rst (decoder + FIFO reset while set), 1 pipe_en, 2 lr_from_deblocked, 3 sr_buf,
//                       [5:4] h_buf, 8 eos (no more bytes for this tile: in_eos once the FIFO drains; cleared by
//                       a tile start), [21:16] n_tile_cols
//   0x00C START     W   one-cycle pulses: bit 0 tile_start, 1 frame_start, 2 lf_start, 3 cdef_start, 4 sr_start,
//                       5 lr_start (a start clears its stage's sticky done bit)
//   0x010 STATUS    R   bit 0 tile_busy, 1 tile_done_lvl, 2 unsupported, 3 frame_busy, 4 frame_done_lvl,
//                       5 lf_busy, 6 lf_done (sticky), 7 cdef_busy, 8 cdef_done (sticky), 9 sr_busy,
//                       10 sr_done (sticky), 11 lr_busy, 12 lr_done (sticky), 13 in_ready, 14 fifo overflow
//                       (sticky until rst), 15 fifo empty (and no byte pending), [31:16] free FIFO words
//   0x014 PERF_SYN  R   perf_syn_stall       0x018 PERF_REC  R   perf_rec_idle
//   0x01C DATA1     W   push one tile byte [7:0]        0x020 DATA4  W  push four bytes, [7:0] first
//                       (a push into a full FIFO is dropped and sets the overflow bit: poll the free count)
//   0x024 HADDR     RW  host read address {2'd0, plane[1:0], y[13:0], x[13:0]}   0x028 HDATA  R  h_rdata [11:0]
//   0x02C DEF_ADDR  W   CDF default row address; the write strobes def_we with DEF_DATA
//   0x040-0x05C DEF_DATA[0..7]  RW  246-bit default row
//   0x080-0x0BC HDR[0..15]  RW  syn_pkg::hdr_t     0x0C0-0x0FC RH[0..15]  RW  rec_hdr_t
//   0x100-0x13C LH[0..15]   RW  lf_hdr_t           0x140-0x17C CH[0..15]  RW  cdef_hdr_t
// Reads of other words return 0; writes are ignored. (tb/test_dec_host.py drives this map and records the
// register transcript tools/kv260 replays on the board.)
module dec_host
    import syn_pkg::*;
    import cdf_map_pkg::*;
#(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int ITX_LANES = 2,
    parameter int FRAME_URAM = 1,
    parameter int FIFO_LW = 10               // tile-byte FIFO: 2^FIFO_LW words of up to 4 bytes (one BRAM36 at 10)
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET aresetn" *)
    input  logic        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk2x CLK" *)
    input  logic        clk2x,
    input  logic        phase,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  logic        aresetn,
    // AXI4-Lite slave (the block-design flow infers the interface from the s_axi_ names, fpga/kv260/build.tcl)
    input  logic [11:0] s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [11:0] s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready
);
    localparam int HDR_W = $bits(hdr_t), RH_W = $bits(rec_hdr_t), LH_W = $bits(lf_hdr_t), CH_W = $bits(cdef_hdr_t);
    initial begin
        if (HDR_W > 512 || RH_W > 512 || LH_W > 512 || CH_W > 512) $error("dec_host: a header struct exceeds its 16-word window");
        if (FBX > 14 || FBY > 14) $error("dec_host: HADDR holds 14-bit coordinates");
    end

    // ---------------------------------------------------------------- registers
    logic [31:0] ctrl;
    logic [31:0] haddr;
    logic [31:0] def_w [8];
    logic [31:0] hdr_w [16], rh_w [16], lh_w [16], ch_w [16];
    logic        rst;                                        // decoder + FIFO reset
    assign rst = !aresetn || ctrl[0];

    // ---------------------------------------------------------------- the decoder
    hdr_t      hdr;
    rec_hdr_t  rh;
    lf_hdr_t   lh;
    cdef_hdr_t ch;
    logic [511:0] hdr_f, rh_f, lh_f, ch_f;
    always_comb begin
        for (int i = 0; i < 16; i++) begin
            hdr_f[32*i +: 32] = hdr_w[i]; rh_f[32*i +: 32] = rh_w[i]; lh_f[32*i +: 32] = lh_w[i]; ch_f[32*i +: 32] = ch_w[i];
        end
        hdr = hdr_t'(hdr_f[HDR_W-1:0]); rh = rec_hdr_t'(rh_f[RH_W-1:0]); lh = lf_hdr_t'(lh_f[LH_W-1:0]); ch = cdef_hdr_t'(ch_f[CH_W-1:0]);
    end
    logic [7:0]        in_data;
    logic              in_valid, in_ready, in_eos;
    logic              def_we;
    logic [CDF_AW-1:0] def_addr;
    logic [245:0]      def_data;
    logic [5:0]        start;                                // tile, frame, lf, cdef, sr, lr
    logic              tile_busy, tile_done, tile_done_lvl, unsupported;
    logic [31:0]       perf_syn_stall, perf_rec_idle;
    logic              frame_busy, frame_done, frame_done_lvl;
    logic              lf_busy, lf_done, cdef_busy, cdef_done, sr_busy, sr_done, lr_busy, lr_done_o;
    logic              blk_done, tx_done, lr_done;
    blk_rec_t          blk_rec;
    tx_rec_t           tx_rec;
    lr_rec_t           lr_rec;
    logic [11:0]       h_rdata;
    dec_top #(.FBX(FBX), .FBY(FBY), .ITX_LANES(ITX_LANES), .FRAME_URAM(FRAME_URAM)) u_dec (
        .clk, .clk2x, .phase, .rst, .hdr, .rh, .lh, .ch,
        .in_data, .in_valid, .in_ready, .in_eos,
        .def_we, .def_addr, .def_data,
        .tile_start(start[0]), .tile_busy, .tile_done, .tile_done_lvl, .unsupported, .perf_syn_stall, .perf_rec_idle,
        .pipe_en(ctrl[1]), .frame_start(start[1]), .n_tile_cols(ctrl[21:16]), .frame_busy, .frame_done, .frame_done_lvl,
        .lf_start(start[2]), .lf_busy, .lf_done,
        .cdef_start(start[3]), .cdef_busy, .cdef_done,
        .sr_start(start[4]), .sr_buf(ctrl[3]), .sr_busy, .sr_done,
        .lr_start(start[5]), .lr_from_deblocked(ctrl[2]), .lr_busy, .lr_done_o,
        .blk_done, .blk_rec, .tx_done, .tx_rec, .lr_done, .lr_rec,
        .h_buf(ctrl[5:4]), .h_plane(haddr[29:28]), .h_x(haddr[FBX-1:0]), .h_y(haddr[14 +: FBY]), .h_rdata
    );
    logic [CDF_AW-1:0] def_addr_r;
    always_comb begin
        def_addr = def_addr_r;
        for (int i = 0; i < 7; i++) def_data[32*i +: 32] = def_w[i];
        def_data[245:224] = def_w[7][21:0];
    end

    // sticky stage-done bits (the stage outputs are one-cycle pulses)
    logic lf_done_s, cdef_done_s, sr_done_s, lr_done_s;
    always_ff @(posedge clk) begin
        if (rst) begin lf_done_s <= 1'b0; cdef_done_s <= 1'b0; sr_done_s <= 1'b0; lr_done_s <= 1'b0; end
        else begin
            lf_done_s   <= start[2] ? 1'b0 : (lf_done_s   | lf_done);
            cdef_done_s <= start[3] ? 1'b0 : (cdef_done_s | cdef_done);
            sr_done_s   <= start[4] ? 1'b0 : (sr_done_s   | sr_done);
            lr_done_s   <= start[5] ? 1'b0 : (lr_done_s   | lr_done_o);
        end
    end

    // ---------------------------------------------------------------- tile-byte FIFO (words of 1..4 bytes)
    localparam int FD = 1 << FIFO_LW;
    logic [33:0]        fmem [0:FD-1];                       // {count-1, bytes}
    logic [FIFO_LW:0]   wptr, rptr;
    logic               f_empty, f_full;
    logic [FIFO_LW:0]   f_free;
    assign f_empty = (wptr == rptr);
    assign f_full  = (wptr[FIFO_LW-1:0] == rptr[FIFO_LW-1:0]) && (wptr[FIFO_LW] != rptr[FIFO_LW]);
    assign f_free  = (FIFO_LW + 1)'(FD) - (wptr - rptr);
    logic               push;
    logic [33:0]        push_d;
    logic               f_ovf;
    logic               ow_valid;                            // the word being handed out byte by byte
    logic [31:0]        ow;
    logic [1:0]         ow_n, ow_i;
    logic               take, ow_done, fetch;
    assign take    = ow_valid && in_ready;
    assign ow_done = take && (ow_i == ow_n);
    assign fetch   = !f_empty && (!ow_valid || ow_done);
    always_ff @(posedge clk) begin
        if (rst) begin wptr <= '0; rptr <= '0; ow_valid <= 1'b0; f_ovf <= 1'b0; ow_i <= 2'd0; end
        else begin
            if (push) begin
                if (f_full) f_ovf <= 1'b1;
                else begin fmem[wptr[FIFO_LW-1:0]] <= push_d; wptr <= wptr + 1'b1; end
            end
            if (fetch) begin
                {ow_n, ow} <= fmem[rptr[FIFO_LW-1:0]]; rptr <= rptr + 1'b1; ow_valid <= 1'b1; ow_i <= 2'd0;
            end else if (ow_done) ow_valid <= 1'b0;
            else if (take) ow_i <= ow_i + 1'b1;
        end
    end
    assign in_valid = ow_valid;
    assign in_data  = ow[8 * ow_i +: 8];
    assign in_eos   = ctrl[8] && !ow_valid && f_empty;

    // ---------------------------------------------------------------- AXI4-Lite slave
    logic        aw_v, w_v, ar_v;
    logic [11:0] aw_a, ar_a;
    logic [31:0] w_d;
    logic [3:0]  w_s;
    logic        do_write, do_read;
    assign s_axi_awready = !aw_v;
    assign s_axi_wready  = !w_v;
    assign s_axi_arready = !ar_v && !s_axi_rvalid;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_rresp   = 2'b00;
    assign do_write = aw_v && w_v && !s_axi_bvalid;
    assign do_read  = ar_v;
    logic [9:0] wi, ri;                                      // word index
    assign wi = aw_a[11:2];
    assign ri = ar_a[11:2];
    function automatic logic [31:0] strb(input logic [31:0] old, input logic [31:0] nw, input logic [3:0] s);
        logic [31:0] r;
        for (int b = 0; b < 4; b++) r[8*b +: 8] = s[b] ? nw[8*b +: 8] : old[8*b +: 8];
        return r;
    endfunction
    always_ff @(posedge clk) begin
        if (!aresetn) begin
            aw_v <= 1'b0; w_v <= 1'b0; ar_v <= 1'b0; s_axi_bvalid <= 1'b0; s_axi_rvalid <= 1'b0;
            ctrl <= 32'd0; haddr <= 32'd0; def_addr_r <= '0;
            for (int i = 0; i < 8; i++) def_w[i] <= '0;
            for (int i = 0; i < 16; i++) begin hdr_w[i] <= '0; rh_w[i] <= '0; lh_w[i] <= '0; ch_w[i] <= '0; end
            start <= '0; push <= 1'b0; def_we <= 1'b0;
        end else begin
            start <= '0; push <= 1'b0; def_we <= 1'b0;
            if (s_axi_awvalid && s_axi_awready) begin aw_a <= s_axi_awaddr; aw_v <= 1'b1; end
            if (s_axi_wvalid && s_axi_wready) begin w_d <= s_axi_wdata; w_s <= s_axi_wstrb; w_v <= 1'b1; end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
            if (do_write) begin
                aw_v <= 1'b0; w_v <= 1'b0; s_axi_bvalid <= 1'b1;
                case (wi)
                    10'h002: ctrl <= strb(ctrl, w_d, w_s);
                    10'h003: start <= w_d[5:0];
                    10'h007: begin push <= 1'b1; push_d <= {2'd0, 24'd0, w_d[7:0]}; end
                    10'h008: begin push <= 1'b1; push_d <= {2'd3, w_d}; end
                    10'h009: haddr <= strb(haddr, w_d, w_s);
                    10'h00B: begin def_addr_r <= w_d[CDF_AW-1:0]; def_we <= 1'b1; end
                    default: begin
                        if (wi[9:3] == 7'h02)        def_w[wi[2:0]] <= strb(def_w[wi[2:0]], w_d, w_s);       // 0x040
                        else if (wi[9:4] == 6'h02)   hdr_w[wi[3:0]] <= strb(hdr_w[wi[3:0]], w_d, w_s);       // 0x080
                        else if (wi[9:4] == 6'h03)   rh_w[wi[3:0]]  <= strb(rh_w[wi[3:0]], w_d, w_s);        // 0x0C0
                        else if (wi[9:4] == 6'h04)   lh_w[wi[3:0]]  <= strb(lh_w[wi[3:0]], w_d, w_s);        // 0x100
                        else if (wi[9:4] == 6'h05)   ch_w[wi[3:0]]  <= strb(ch_w[wi[3:0]], w_d, w_s);        // 0x140
                    end
                endcase
            end
            // a tile start ends the previous tile's end-of-stream
            if (start[0]) ctrl[8] <= 1'b0;
            if (s_axi_arvalid && s_axi_arready) begin ar_a <= s_axi_araddr; ar_v <= 1'b1; end
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
            if (do_read) begin
                ar_v <= 1'b0; s_axi_rvalid <= 1'b1;
                case (ri)
                    10'h000: s_axi_rdata <= 32'hA71D_0001;
                    10'h001: s_axi_rdata <= {8'd0, 8'(FBX), 8'(FBY), 4'(ITX_LANES), 3'd0, 1'(FRAME_URAM)};
                    10'h002: s_axi_rdata <= ctrl;
                    10'h004: s_axi_rdata <= {16'(f_free), !ow_valid && f_empty, f_ovf, in_ready, lr_done_s, lr_busy, sr_done_s, sr_busy,
                                             cdef_done_s, cdef_busy, lf_done_s, lf_busy, frame_done_lvl, frame_busy, unsupported, tile_done_lvl, tile_busy};
                    10'h005: s_axi_rdata <= perf_syn_stall;
                    10'h006: s_axi_rdata <= perf_rec_idle;
                    10'h009: s_axi_rdata <= haddr;
                    10'h00A: s_axi_rdata <= {20'd0, h_rdata};
                    default: begin
                        if (ri[9:3] == 7'h02)      s_axi_rdata <= def_w[ri[2:0]];
                        else if (ri[9:4] == 6'h02) s_axi_rdata <= hdr_w[ri[3:0]];
                        else if (ri[9:4] == 6'h03) s_axi_rdata <= rh_w[ri[3:0]];
                        else if (ri[9:4] == 6'h04) s_axi_rdata <= lh_w[ri[3:0]];
                        else if (ri[9:4] == 6'h05) s_axi_rdata <= ch_w[ri[3:0]];
                        else                       s_axi_rdata <= 32'd0;
                    end
                endcase
            end
        end
    end
endmodule
