// Loop restoration stage of the streaming filter chain (docs/filter-chain.md) for one plane, one superblock row (band)
// at a time. Wiener units at a line a clock; self-guided units correct but slow (below); super-resolution is not
// handled.
//
// In: the CDEF stage's output as cdef_stream sends it: line segments of 8 samples (4 in a horizontally subsampled
// plane), segment column by segment column, each column's lines top to bottom. A band brings the lines from 16 above
// its first line (12 in a vertically subsampled plane; from line 0 in the first band) to 17 (13) above its end (to the
// last coded line in the last band).
// A line's horizontal pass reads three samples either side, so segment column C is filtered when column C + 1 has
// arrived: four column buffers rotate (three in the window, one filling). The column's lines go top to bottom
// through lr_wiener_h, one a clock; the last six intermediate lines sit in a shift register and lr_wiener_v gives the
// line three above the newest. The source rule (7.17.6) in this flow:
//   - picture borders: samples left of column 0 and right of the plane's last sample repeat the border sample; the
//     plane's first line is pushed three more times before itself, its last line three more times after itself;
//   - stripes (64 luma lines, offset 8): before a stripe's first line S the deblocked lines S - 2, S - 2, S - 1 are
//     pushed (the shift register starts afresh there), after its last line E the deblocked lines E + 1, E + 2, E + 2
//     (clamped to the plane). The four deblocked lines around a boundary come from a line store (bl_* port: boundary
//     k is the first line of stripe k) and are copied into the column buffer when the column has arrived;
//   - the unit (type, coefficients) is read from mi_store's records at every stripe start: a unit row border is a
//     stripe border, and a segment lies in one unit column;
//   - lines and samples of the coded area outside the plane leave as they came.
// Between bands the stage keeps, per column, the six newest intermediate lines and the three newest source lines
// (the lines not yet sent): a band's output runs from three lines above its first input line to three above its last.
// This relies on the picture's geometry: a band's input never ends on a stripe's last line or within three lines
// after a stripe's first (CDEF's band ends 8 lines before a stripe boundary, 4 in a subsampled plane).
// Self-guided units (correctness first): the same pushes, but the line's 14 samples go to a sequential engine
// instead of the horizontal pass and the column waits for it. The engine keeps the last six lines; per line it
// walks the 14 columns (five squares and the column sums of both radii a clock), slides the box sums over them and
// sends the ten box positions x - 1 .. x + 8 through two lr_sgr_ab (one a pass); lr_sgr_f then gives the line three
// above the newest from the three newest rows of a2 / b2. About 26 clocks a line. Between bands the six newest lines
// are carried instead of the Wiener intermediates and pushed again at the column's start to rebuild the rows.
// Out: line segments as they came in: first 4x4 column, line counted from the band's first line (negative above it).
// Rate: one line a clock inside a column; a stripe boundary costs eight more (two unit reads, six extra lines), a
// column one idle clock, and the arrival side six clocks a column and boundary for the copy. A self-guided line
// takes about 26 clocks.
// The band's parameters are held from band_start to band_done.
module lr_stream
  import syn_pkg::*;
  import lr_pkg::*;
#(
    parameter int CW = 10                    // log2 of the most segment columns / 4x4 columns
) (
    input  logic            clk,
    input  logic            rst,
    input  logic [3:0]      bit_depth,
    input  logic            en,              // the plane has restoration units (FrameRestorationType is not NONE)
    input  logic [1:0]      plane,
    input  logic            ssx, ssy,        // the plane's subsampling (0 for luma)
    input  logic [MIW+1:0]  pl_w, pl_h,      // the plane in samples (PlaneEndX + 1, PlaneEndY + 1)
    input  logic [3:0]      u_log2,          // log2 of the unit size in the plane's samples (5 .. 8)
    input  logic [SBW-1:0]  u_rows, u_cols,  // units in the plane
    input  logic            band_start,      // pulse while idle
    input  logic [MIW-1:0]  row0,            // the band's first 4x4 row in the plane
    input  logic [MIW-1:0]  pic_rows,        // the plane's coded 4x4 rows
    input  logic [5:0]      n_blk,           // the plane's 4x4 rows in the band
    input  logic [CW:0]     n_col,           // the plane's coded 4x4 columns
    input  logic            first_band,
    input  logic            last_band,
    output logic            busy,
    output logic            band_done,
    input  logic            in_valid,
    output logic            in_ready,
    input  logic [95:0]     in_line,
    input  logic signed [8:0] in_y,          // the line, counted from the band's first line
    // unit records (mi_store), one clock after the address
    output logic [1:0]      lrr_plane,
    output logic [SBW-1:0]  lrr_row, lrr_col,
    input  lr_rec_t         lrr_rec,
    // deblocked lines around stripe boundary bl_k (the first line B of stripe bl_k): lines B - 2, B - 1, B, B + 1 of
    // the segment at 4x4 column bl_col, line i at bits i * 96, one clock after the address
    output logic [SBW:0]    bl_k,
    output logic [CW-1:0]   bl_col,
    input  logic [383:0]    bl_rd,
    output logic            out_valid,
    output logic [95:0]     out_line,        // out_n4 groups of four samples from bit 0
    output logic [1:0]      out_n4,
    output logic [CW-1:0]   out_col,         // the segment's first 4x4 column
    output logic signed [8:0] out_y
);
    localparam int YW = MIW + 2;
    localparam logic [7:0] BL0 = 8'd240;     // column buffer entries of the boundary lines: BL0 + 4 * (boundary in the band) + line

    // ---------------------------------------------------------------- the band's geometry
    logic [1:0]    n4;
    logic [3:0]    segw;                     // samples in a segment
    logic [YW-1:0] so, smask, y0, ya, yb, hcod;
    logic [2:0]    lsh;
    logic [4:0]    sh;
    logic [CW:0]   n_seg, c_w;               // segment columns; the one with the plane's last sample
    logic [2:0]    lw;                       // that sample's lane
    logic [SBW:0]  kf;                       // the first stripe boundary below the band's first input line
    logic [YW-1:0] xe;
    logic [7:0]    nl;                       // lines a column brings
    always_comb begin
        n4 = ssx ? 2'd1 : 2'd2;
        segw = ssx ? 4'd4 : 4'd8;
        so = ssy ? YW'(4) : YW'(8);
        lsh = ssy ? 3'd5 : 3'd6;
        smask = ssy ? YW'(31) : YW'(63);
        sh = ssy ? 5'd12 : 5'd16;
        y0 = {row0, 2'b00};
        hcod = {pic_rows, 2'b00};
        ya = first_band ? YW'(0) : y0 - YW'(sh);
        yb = last_band ? hcod : y0 + YW'({n_blk, 2'b00}) - YW'(sh);
        nl = 8'(yb - ya);
        n_seg = ssx ? n_col : (CW+1)'((n_col + (CW+1)'(1)) >> 1);
        xe = pl_w - YW'(1);
        c_w = ssx ? (CW+1)'(xe >> 2) : (CW+1)'(xe >> 3);
        lw = ssx ? {1'b0, xe[1:0]} : xe[2:0];
        kf = (SBW+1)'((ya + so) >> lsh) + (SBW+1)'(1);
    end

    // ---------------------------------------------------------------- column buffers
    logic [3:0]  cb_we;
    logic [7:0]  cb_wa, cb_ra;
    logic [95:0] cb_wd;
    logic [95:0] cb_q [4];
    generate for (genvar b = 0; b < 4; b++) begin : g_cb
        (* ram_style = "block" *) logic [95:0] mem [256];
        always_ff @(posedge clk) begin
            if (cb_we[b]) mem[cb_wa] <= cb_wd;
            cb_q[b] <= mem[cb_ra];
        end
    end endgenerate

    // ---------------------------------------------------------------- arrival
    typedef enum logic [1:0] {A_LINE, A_BRD, A_BWR, A_CHK} ast_t;
    ast_t          ast;
    logic          act;
    logic [CW:0]   a_col;                    // segment column arriving = columns complete
    logic [7:0]    a_cnt;
    logic [SBW:0]  a_k;                      // boundary being copied
    logic [1:0]    a_i;
    logic [CW:0]   g_col;                    // segment column being filtered = columns filtered
    logic [YW-1:0] a_s;                      // the boundary's line
    logic          a_has;                    // the band's lines reach it and it is inside the plane
    logic          a_step;
    logic [8:0]    a_addr;
    always_comb begin
        a_s = YW'(YW'(a_k) << lsh) - so;
        a_has = (a_s < yb) && (a_s < pl_h);
        // column c takes the buffer of column c - 4, free when column c - 3 has been filtered
        in_ready = act && (ast == A_LINE) && (a_col < n_seg) && (a_col < (CW+1)'(4) || a_col - g_col <= (CW+1)'(2));
        a_step = in_valid && in_ready;
        a_addr = 9'(in_y) + (first_band ? 9'd0 : 9'(sh));
        cb_we = '0; cb_wa = a_addr[7:0]; cb_wd = in_line;
        if (a_step) cb_we[a_col[1:0]] = 1'b1;
        if (ast == A_BWR) begin
            cb_we[a_col[1:0]] = 1'b1;
            cb_wa = BL0 + {4'd0, 2'(a_k - kf), a_i};
            cb_wd = bl_rd[a_i * 96 +: 96];
        end
        bl_k = a_k;
        bl_col = ssx ? a_col[CW-1:0] : CW'(a_col << 1);
    end
    always_ff @(posedge clk) begin
        if (rst) begin act <= 1'b0; ast <= A_LINE; a_col <= '0; a_cnt <= '0; a_k <= '0; a_i <= '0; end
        else if (band_start) begin act <= 1'b1; ast <= A_LINE; a_col <= '0; a_cnt <= '0; a_k <= kf; end
        else begin
            if (band_done) act <= 1'b0;
            case (ast)
                A_LINE: if (a_step) begin
                    if (a_cnt == nl - 8'd1) begin
                        a_cnt <= '0;
                        if (a_has) ast <= A_BRD; else a_col <= a_col + (CW+1)'(1);
                    end else a_cnt <= a_cnt + 8'd1;
                end
                A_BRD: begin ast <= A_BWR; a_i <= 2'd0; end
                A_BWR: begin
                    a_i <= a_i + 2'd1;
                    if (a_i == 2'd3) begin a_k <= a_k + (SBW+1)'(1); ast <= A_CHK; end
                end
                A_CHK: if (a_has) ast <= A_BRD;
                       else begin a_col <= a_col + (CW+1)'(1); a_k <= kf; ast <= A_LINE; end
            endcase
        end
    end

    // ---------------------------------------------------------------- one segment column, line by line
    typedef enum logic [3:0] {G_IDLE, G_UNIT, G_UNIT2, G_RPL, G_REP0, G_TOP, G_BPRE, G_LINE, G_REP, G_BOT, G_BPOST, G_DRAIN} gst_t;
    gst_t          gst, g_after, g_post, g_disp;
    logic          g_sgr;                    // the stripe's unit is self-guided
    logic [3:0]    g_set;
    logic [15:0]   g_xqd;
    logic [2:0]    g_rc;                     // carried line pushed again
    logic          sg_busy, sg_done;         // the self-guided engine has a line
    logic [YW-1:0] g_y, g_y1;                // the next line of the band to push
    logic [1:0]    g_cnt;
    logic [3:0]    g_dr;
    logic          g_new;                    // nothing of the column pushed yet
    logic          g_wien;
    logic [20:0]   g_hc, g_vc;
    logic          g_start, g_end, g_lastl, g_haspost;
    logic [SBW:0]  g_k;
    logic [1:0]    j_top, j_bot;
    logic [YW-1:0] u_r, u_c;
    always_comb begin
        g_y1 = g_y + YW'(1);
        g_start = ((g_y + so) & smask) == '0;                 // a stripe's first line (never line 0)
        g_end = ((g_y1 + so) & smask) == '0;                  // a stripe's last line
        g_k = (SBW+1)'((g_y + so) >> lsh);
        j_top = 2'(g_k - kf);
        j_bot = 2'(g_k + (SBW+1)'(1) - kf);
        g_lastl = g_y1 == yb;
        // what follows line g_y: the plane's last line is repeated, a stripe's last line is followed by the deblocked
        // lines below, the last line of the coded area below the plane pushes the lines before it out
        g_haspost = 1'b1;
        if (g_y1 == pl_h) g_post = G_REP;
        else if (g_y1 < pl_h && g_end) g_post = G_BOT;
        else if (g_y1 == hcod) g_post = G_BPOST;
        else begin g_post = G_LINE; g_haspost = 1'b0; end
        // and then: the column's end, a new stripe (or the lines below the plane), or the next line
        if (g_lastl) g_after = G_IDLE;
        else if (g_y1 == pl_h || (g_y1 < pl_h && g_end)) g_after = G_UNIT;
        else g_after = G_LINE;
        // before line g_y: the plane's first line three times, the lines below the plane apart, a stripe's top lines
        g_disp = (g_y == '0) ? G_REP0 : (g_y == pl_h) ? G_BPRE : (g_y < pl_h && g_start) ? G_TOP : G_LINE;
        u_r = (g_y + so) >> u_log2;
        u_c = (ssx ? YW'(g_col) << 2 : YW'(g_col) << 3) >> u_log2;
        lrr_plane = plane;
        lrr_row = (u_r >= YW'(u_rows)) ? u_rows - SBW'(1) : SBW'(u_r);
        lrr_col = (u_c >= YW'(u_cols)) ? u_cols - SBW'(1) : SBW'(u_c);
    end

    // a push: one source line of the column into the horizontal pass
    typedef struct packed {
        logic          v;
        logic          seed;                 // the shift register starts afresh with this line
        logic          load;                 // the column's first line of a band: the carried lines come first
        logic          first;                // the column's first push
        logic          save;                 // the column's last line of a band: carry the lines
        logic          wien;
        logic          sgr;                  // the line goes to the self-guided engine
        logic          rpl;                  // a carried line pushed again (self-guided): rebuilds the rows only
        logic          par;                  // the line's parity in the plane; read when the line makes the engine give a
                                             // line (the one three above it), so never from a seed or a carried line
        logic [20:0]   vc;
        logic [3:0]    nv;                   // the segment's samples inside the plane
        logic [CW-1:0] col;
    } tag_t;
    tag_t          is, p1, p2, p3, p4;
    logic [7:0]    is_a;
    logic          is_endc;
    always_comb begin
        is = '0; is_a = 8'(g_y - ya);
        case (gst)
            G_RPL:   begin is.v = 1'b1; is.rpl = 1'b1; is.par = g_rc[0]; end
            G_REP0:  begin is.v = 1'b1; is_a = 8'd0; is.seed = g_cnt == 2'd0; is.par = !g_cnt[0]; end
            G_TOP:   begin is.v = 1'b1; is_a = BL0 + {4'd0, j_top, (g_cnt == 2'd2) ? 2'd1 : 2'd0}; is.seed = g_cnt == 2'd0; is.par = !g_cnt[0]; end
            G_BPRE:  begin is.v = 1'b1; is.seed = g_cnt == 2'd0; end
            G_LINE:  begin is.v = 1'b1; is.par = g_y[0]; end
            G_REP:   begin is.v = 1'b1; is.par = !(g_y[0] ^ g_cnt[0]); end
            G_BOT:   begin is.v = 1'b1; is_a = BL0 + {4'd0, j_bot, (g_cnt == 2'd0 || g_y + YW'(2) >= pl_h) ? 2'd2 : 2'd3}; is.par = !(g_y[0] ^ g_cnt[0]); end
            G_BPOST: is.v = 1'b1;
            default: ;
        endcase
        if (sg_busy) begin is.v = 1'b0; is.seed = 1'b0; is.rpl = 1'b0; end          // the column waits for the engine
        is_endc = is.v && g_lastl && ((gst == G_LINE && !g_haspost) || ((gst == G_REP || gst == G_BOT || gst == G_BPOST) && g_cnt == 2'd2));
        is.first = is.v && g_new && !is.rpl;
        is.load = is.v && g_new && !is.rpl && !first_band;
        is.save = is_endc && !last_band;
        is.wien = g_wien;
        is.sgr = g_sgr;
        is.vc = g_vc;
        is.nv = (g_col == c_w) ? 4'({1'b0, lw}) + 4'd1 : segw;
        is.col = g_col[CW-1:0];
        cb_ra = is_a;
    end
    always_ff @(posedge clk) begin
        band_done <= 1'b0;
        if (rst || band_start) begin gst <= G_IDLE; g_col <= '0; g_new <= 1'b0; g_cnt <= '0; g_dr <= '0; sg_busy <= 1'b0; end
        else if (sg_busy) begin
            if (sg_done) sg_busy <= 1'b0;
        end else begin
            if (is.v && !is.rpl) g_new <= 1'b0;
            if (is.v && is.sgr) sg_busy <= 1'b1;
            case (gst)
                G_IDLE: if (act && g_col < n_seg) begin
                            if (a_col > g_col + (CW+1)'(1) || a_col == n_seg) begin g_y <= ya; g_new <= 1'b1; gst <= G_UNIT; end
                        end else if (act && !band_done) begin gst <= G_DRAIN; g_dr <= 4'd0; end
                G_UNIT: gst <= G_UNIT2;                                           // the unit's address is out
                G_UNIT2: begin
                    g_wien <= en && (g_y < pl_h) && (lrr_rec.lr_type == 2'd1);
                    g_sgr <= en && (g_y < pl_h) && (lrr_rec.lr_type == 2'd2);
                    g_vc <= lrr_rec.wiener[0 +: 21]; g_hc <= lrr_rec.wiener[21 +: 21];
                    g_set <= lrr_rec.sgr_set; g_xqd <= lrr_rec.xqd;
                    g_cnt <= 2'd0; g_rc <= 3'd0;
                    // a self-guided column that continues from the band above: its six carried lines first
                    gst <= (g_new && !first_band && en && (g_y < pl_h) && (lrr_rec.lr_type == 2'd2)) ? G_RPL : g_disp;
                end
                G_RPL: begin
                    g_rc <= g_rc + 3'd1;
                    if (g_rc == 3'd5) gst <= g_disp;
                end
                G_REP0, G_TOP, G_BPRE: begin
                    g_cnt <= g_cnt + 2'd1;
                    if (g_cnt == 2'd2) gst <= G_LINE;
                end
                G_LINE: begin
                    g_cnt <= 2'd0;
                    if (g_haspost) gst <= g_post;
                    else begin
                        g_y <= g_y1; gst <= g_after;
                        if (g_lastl) g_col <= g_col + (CW+1)'(1);
                    end
                end
                G_REP, G_BOT, G_BPOST: begin
                    g_cnt <= g_cnt + 2'd1;
                    if (g_cnt == 2'd2) begin
                        g_y <= g_y1; gst <= g_after;
                        if (g_lastl) g_col <= g_col + (CW+1)'(1);
                    end
                end
                G_DRAIN: begin                                                    // the last lines leave
                    g_dr <= g_dr + 4'd1;
                    if (g_dr == 4'd8) begin band_done <= 1'b1; gst <= G_IDLE; end
                end
                default: gst <= G_IDLE;
            endcase
        end
    end
    assign busy = act;

    // ---------------------------------------------------------------- the line's samples with the picture's side borders
    localparam int SRW = 6 * 128, RSW = 3 * 96, PXL = 6 * 168;
    logic [PXL+RSW-1:0] cy_q;                // the column's carried lines, read at the column's start
    logic [20:0]  p1_hc, p2_hc;
    logic [1:0]   p1_slot;
    logic [2:0]   p1_rc;
    logic         p1_c0, p1_last, p1_nxl;
    logic [95:0]  w_l, w_m, w_r;
    logic [3:0]   nvr;
    logic [167:0] px_c, px2;
    logic [95:0]  raw2, raw3, raw4;
    function automatic logic [11:0] smp(input logic [95:0] s, input logic [2:0] i);
        smp = s[i * 12 +: 12];
    endfunction
    always_comb begin
        w_m = cb_q[p1_slot]; w_l = cb_q[p1_slot - 2'd1]; w_r = cb_q[p1_slot + 2'd1];
        nvr = p1_nxl ? 4'({1'b0, lw}) + 4'd1 : segw;
        for (int i = 0; i < 14; i++) begin
            logic [3:0] xr, rr;
            xr = 4'(i - 3); rr = 4'(i - 3) - segw;
            if (i < 3) px_c[i * 12 +: 12] = p1_c0 ? smp(w_m, 3'd0) : smp(w_l, 3'(int'(segw) + i - 3));
            else if (xr < segw) px_c[i * 12 +: 12] = smp(w_m, (xr >= p1.nv) ? 3'(p1.nv - 4'd1) : 3'(xr));
            else if (p1_last) px_c[i * 12 +: 12] = smp(w_m, 3'(p1.nv - 4'd1));
            else px_c[i * 12 +: 12] = smp(w_r, (rr >= nvr) ? 3'(nvr - 4'd1) : 3'(rr));
        end
    end
    always_ff @(posedge clk) begin
        p1 <= is; p1_hc <= g_hc; p1_slot <= g_col[1:0]; p1_rc <= g_rc;
        p1_c0 <= g_col == '0; p1_last <= g_col == c_w; p1_nxl <= g_col + (CW+1)'(1) == c_w;
        p2 <= p1; p2_hc <= p1_hc; px2 <= p1.rpl ? cy_q[32'(3'd5 - p1_rc) * 168 +: 168] : px_c; raw2 <= w_m;
        p3 <= p2; raw3 <= raw2;
        p4 <= p3; raw4 <= raw3;
        if (rst) begin p1.v <= 1'b0; p2.v <= 1'b0; p3.v <= 1'b0; p4.v <= 1'b0; end
    end

    // ---------------------------------------------------------------- horizontal pass, the last lines, vertical pass
    logic            h_v;
    logic [8*21-1:0] h_out;
    lr_wiener_h #(.N(8)) u_h (.clk, .rst, .v_in(p2.v && !p2.sgr), .bd(bit_depth), .coef(p2_hc), .px(px2), .v_out(h_v), .out(h_out));

    // ---------------------------------------------------------------- self-guided: the engine
    logic [167:0]  wl [6];                   // the last six lines (0 the newest), 14 samples each
    tag_t          sgt;                      // the line in the engine
    logic [95:0]   sg_raw;
    logic          sg_col;                   // walking the columns
    logic [3:0]    sg_i;
    logic [14:0]   c5b, c3b;
    logic [26:0]   c5a, c3a;
    logic [14:0]   h5b [5], h3b [4];         // the last column sums (0 the newest)
    logic [26:0]   h5a [5], h3a [4];
    logic          h_pv, bx_v;               // a box position is complete in the column sums / in the box sums
    logic [29:0]   bx5a, bx3a;
    logic [17:0]   bx5b, bx3b;
    logic [15:0]   sg_sp;
    logic          ab_v, ab_v1;
    logic [8:0]    ab_a0, ab_a1;
    logic [21:0]   ab_b0, ab_b1;
    logic [3:0]    ab_k;
    logic [309:0]  abn [2];                  // the row being formed: [pass], position k at bits k * 31
    logic [309:0]  abr [2][2];               // [pass][0 = the row before it, 1 = the one before that]
    assign sg_sp = sgr_params(g_set);
    always_comb begin
        c5b = '0; c5a = '0; c3b = '0; c3a = '0;
        for (int r = 0; r < 5; r++) begin
            logic [11:0] c; logic [23:0] q;
            c = wl[r][sg_i * 12 +: 12]; q = 24'(c) * 24'(c);
            c5b = c5b + 15'(c); c5a = c5a + 27'(q);
            if (r >= 1 && r <= 3) begin c3b = c3b + 15'(c); c3a = c3a + 27'(q); end
        end
    end
    always_ff @(posedge clk) begin
        sg_done <= 1'b0;
        if (p2.v && p2.sgr) begin                                  // a line arrives
            wl[0] <= px2; for (int r = 1; r < 6; r++) wl[r] <= wl[r - 1];
            sgt <= p2; sg_raw <= raw2;
            sg_col <= 1'b1; sg_i <= 4'd0; ab_k <= 4'd0;
        end
        h_pv <= 1'b0;
        if (sg_col) begin
            h5b[0] <= c5b; h5a[0] <= c5a; h3b[0] <= c3b; h3a[0] <= c3a;
            for (int k = 1; k < 5; k++) begin h5b[k] <= h5b[k - 1]; h5a[k] <= h5a[k - 1]; end
            for (int k = 1; k < 4; k++) begin h3b[k] <= h3b[k - 1]; h3a[k] <= h3a[k - 1]; end
            h_pv <= sg_i >= 4'd4;                                  // columns i - 4 .. i are in: the box of position i - 4
            sg_i <= sg_i + 4'd1;
            if (sg_i == 4'd13) sg_col <= 1'b0;
        end
        bx_v <= h_pv;
        bx5b <= 18'(h5b[0]) + 18'(h5b[1]) + 18'(h5b[2]) + 18'(h5b[3]) + 18'(h5b[4]);
        bx5a <= 30'(h5a[0]) + 30'(h5a[1]) + 30'(h5a[2]) + 30'(h5a[3]) + 30'(h5a[4]);
        bx3b <= 18'(h3b[1]) + 18'(h3b[2]) + 18'(h3b[3]);           // radius 1: the three columns around the position
        bx3a <= 30'(h3a[1]) + 30'(h3a[2]) + 30'(h3a[3]);
        if (ab_v) begin
            abn[0][ab_k * 31 +: 31] <= {ab_a0, ab_b0};
            abn[1][ab_k * 31 +: 31] <= {ab_a1, ab_b1};
            ab_k <= ab_k + 4'd1;
            if (ab_k == 4'd9) sg_done <= 1'b1;
        end
        if (sg_done) for (int p = 0; p < 2; p++) begin abr[p][1] <= abr[p][0]; abr[p][0] <= abn[p]; end
        if (rst) begin sg_col <= 1'b0; h_pv <= 1'b0; bx_v <= 1'b0; sg_done <= 1'b0; end
    end
    lr_sgr_ab u_ab0 (.clk, .rst, .v_in(bx_v), .bd(bit_depth), .r1(1'b0), .s(sgr_s(g_set, 1'b0)), .box_a(bx5a), .box_b(bx5b),
                     .v_out(ab_v), .a2(ab_a0), .b2(ab_b0));
    lr_sgr_ab u_ab1 (.clk, .rst, .v_in(bx_v), .bd(bit_depth), .r1(1'b1), .s(sgr_s(g_set, 1'b1)), .box_a(bx3a), .box_b(bx3b),
                     .v_out(ab_v1), .a2(ab_a1), .b2(ab_b1));

    // ---------------------------------------------------------------- the last lines, vertical pass
    // carried between bands, per column: the six newest intermediate lines (16 bits hold them at every bit depth; a
    // self-guided column: its six newest lines of 14 samples) and the three newest source lines
    (* ram_style = "block" *) logic [PXL+RSW-1:0] cy_mem [1 << CW];
    logic [PXL+RSW-1:0] cy_wd;
    tag_t               ev;                  // the line that enters the count: from the horizontal pass or the engine
    logic               ev_v, sg_ev, fire_s;
    logic [95:0]        ev_raw;
    logic               cy_we;
    logic [SRW-1:0]     sr, sr_b, sr_n;      // line j (0 the newest) at bits j * 128
    logic [RSW-1:0]     rs, rs_b, rs_n;
    logic [127:0]       h16;
    logic [7*8*21-1:0]  hv;
    logic [2:0]         fill, fill_b, fill_n;
    logic               fire;
    logic [YW-1:0]      oy, oy_b;            // the line the vertical pass gives next
    always_comb begin
        for (int l = 0; l < 8; l++) h16[l * 16 +: 16] = h_out[l * 21 +: 16];
        sg_ev = sg_done && !sgt.rpl;
        ev = sg_ev ? sgt : p4;
        ev_v = sg_ev || (p4.v && !p4.sgr);
        ev_raw = sg_ev ? sg_raw : raw4;
        sr_b = ev.load ? cy_q[SRW-1:0] : sr;
        rs_b = ev.load ? cy_q[PXL +: RSW] : rs;
        sr_n = {sr_b[SRW-129:0], h16};
        rs_n = {rs_b[RSW-97:0], ev_raw};
        for (int r = 0; r < 6; r++)
            for (int l = 0; l < 8; l++) hv[(r * 8 + l) * 21 +: 21] = 21'(signed'(sr_b[((5 - r) * 8 + l) * 16 +: 16]));
        for (int l = 0; l < 8; l++) hv[(48 + l) * 21 +: 21] = 21'(signed'(h16[l * 16 +: 16]));
        fill_b = ev.load ? 3'd6 : fill;
        fill_n = ev.seed ? 3'd1 : (fill_b == 3'd7) ? 3'd7 : fill_b + 3'd1;
        fire = ev_v && !sg_ev && (fill_n == 3'd7);
        fire_s = sg_ev && (fill_n == 3'd7);
        oy_b = ev.first ? (first_band ? YW'(0) : ya - YW'(3)) : oy;
        cy_we = ev_v && ev.save;
        cy_wd = sg_ev ? {rs_n, wl[5], wl[4], wl[3], wl[2], wl[1], wl[0]} : {rs_n, (PXL-SRW)'(0), sr_n};
    end
    always_ff @(posedge clk) begin
        if (cy_we) cy_mem[ev.col] <= cy_wd;
        if (gst == G_UNIT && g_new) cy_q <= cy_mem[g_col[CW-1:0]];
        if (ev_v) begin sr <= sr_n; rs <= rs_n; fill <= fill_n; oy <= oy_b + YW'(fire || fire_s); end
        if (rst) fill <= 3'd0;
    end
    logic            v_v;
    logic [95:0]     v_out;
    lr_wiener_v #(.N(8)) u_v (.clk, .rst, .v_in(fire), .bd(bit_depth), .coef(p4.vc), .hv, .v_out(v_v), .out(v_out));
    // self-guided: the line three above the engine's newest, from the rows of a2 / b2 above, of and below it
    logic            f_v;
    logic [95:0]     f_out;
    lr_sgr_f #(.N(8)) u_f (.clk, .rst, .v_in(fire_s), .bd(bit_depth), .odd(!sgt.par), .en0(sg_sp[15:14] != 2'd0), .en1(sg_sp[6:5] != 2'd0),
                           .w0(g_xqd[7:0]), .w1(g_xqd[15:8]),
                           .ab0_m(abr[0][1]), .ab0_c(abr[0][0]), .ab0_p(abn[0]), .ab1_m(abr[1][1]), .ab1_c(abr[1][0]), .ab1_p(abn[1]),
                           .pix(rs_b[2 * 96 +: 96]), .v_out(f_v), .out(f_out));

    // ---------------------------------------------------------------- out: the filtered samples inside the plane of a Wiener unit, else the source
    typedef struct packed {
        logic          wien;
        logic [3:0]    nv;
        logic [CW-1:0] col;
        logic [YW-1:0] y;
        logic [95:0]   raw;
    } otag_t;
    otag_t q5, q6, q7, qo;
    always_ff @(posedge clk) begin
        q5 <= '{wien: ev.wien || ev.sgr, nv: ev.nv, col: ev.col, y: oy_b, raw: rs_b[2 * 96 +: 96]};
        q6 <= q5; q7 <= q6;
    end
    always_comb begin
        out_valid = v_v || f_v;
        qo = f_v ? q7 : q6;                  // the self-guided line leaves a clock later than a Wiener line
        for (int l = 0; l < 8; l++)
            out_line[l * 12 +: 12] = !(qo.wien && 4'(l) < qo.nv) ? qo.raw[l * 12 +: 12] : f_v ? f_out[l * 12 +: 12] : v_out[l * 12 +: 12];
        out_n4 = n4;
        out_col = ssx ? qo.col : CW'(qo.col << 1);
        out_y = 9'(qo.y - y0);
    end
endmodule
