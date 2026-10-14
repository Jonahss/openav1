// openav1 — intra predictor (AV1 spec 7.11.2), four pixels per clock.
//
// The caller loads the prepared edges (AboveRow[0..w+h-1], LeftCol[0..w+h-1], top-left) through
// the edge write port, sets the parameters and pulses start. The block then
//   1. runs the spec's edge preparation passes in place for directional modes
//      (filter corner, intra edge filter per side, 2x upsampling per side),
//   2. for DC, sums the available edges eight per clock and divides in one cycle,
//   3. emits pred[i][j] in raster order, four pixels per clock (an aligned group of columns), with (x, y) of
//      the group's first pixel alongside.
//
// Edge storage: A[] = AboveRow, L[] = LeftCol, indexed -2 .. 257 (index -1 holds the top-left
// sample of that side, exactly like the spec's two arrays; upsampling may rewrite it).
// Filter-intra keeps its own copy of the emitted pixels (max 32x32) for the recursion.
module ipred #(
    parameter int PW = 12                    // pixel width (covers 8/10/12-bit)
) (
    input  logic          clk,
    input  logic          rst,

    // ---- edge write port (while idle). side 0 = AboveRow[idx], 1 = LeftCol[idx], 2 = top-left ----
    input  logic          edge_we,
    input  logic [1:0]    edge_side,
    input  logic [7:0]    edge_idx,
    input  logic [PW-1:0] edge_data,
    input  logic          edge_we4,          // AboveRow[edge_idx .. edge_idx+3] <= edge_data4 lanes (edge_idx a multiple of 4)
    input  logic [4*PW-1:0] edge_data4,

    // ---- parameters (sampled at start) ----
    input  logic          start,
    input  logic [3:0]    mode,             // DC_PRED=0 .. PAETH_PRED=12 (spec order)
    input  logic          use_filter_intra,
    input  logic [2:0]    filter_intra_mode,
    input  logic signed [2:0] angle_delta,  // -3..3
    input  logic [2:0]    log2w,
    input  logic [2:0]    log2h,
    input  logic [3:0]    bit_depth,
    input  logic          have_left,
    input  logic          have_above,
    input  logic          filter_type,      // smooth neighbour (intra filter type process)
    input  logic          edge_filter_en,   // enable_intra_edge_filter
    input  logic [6:0]    above_px,         // Min(w, maxX - x + 1)
    input  logic [6:0]    left_px,          // Min(h, maxY - y + 1)
    output logic          busy,
    output logic          done,

    // ---- prediction output stream ----
    output logic          out_valid,
    output logic [5:0]    out_x,
    output logic [5:0]    out_y,
    output logic [4*PW-1:0] out_pix4       // pixels (out_x .. out_x+3, out_y), lane l at [l*PW +: PW]
);
    localparam int EOFF = 2;                 // edge index k is stored at k + EOFF
    localparam int ESZ  = 260;               // -2 .. 257

    localparam logic [3:0] DC_PRED = 0, SMOOTH_PRED = 9, SMOOTH_V_PRED = 10, SMOOTH_H_PRED = 11, PAETH_PRED = 12;

    // Edge storage in four banks each (entry e at bank e & 3, address e >> 2), as distributed RAM: every read site
    // is then a small asynchronous RAM port instead of a 260:1 mux. All writes come from the sequencer below and
    // never hit one bank twice in a clock (the 4-wide group write is aligned, the upsampler's pairs are adjacent).
    localparam int BKD = (ESZ + 3) / 4;
    (* ram_style = "distributed" *) logic [PW-1:0] A0 [0:BKD-1];
    (* ram_style = "distributed" *) logic [PW-1:0] A1 [0:BKD-1];
    (* ram_style = "distributed" *) logic [PW-1:0] A2 [0:BKD-1];
    (* ram_style = "distributed" *) logic [PW-1:0] A3 [0:BKD-1];
    (* ram_style = "distributed" *) logic [PW-1:0] L0 [0:BKD-1];
    (* ram_style = "distributed" *) logic [PW-1:0] L1 [0:BKD-1];
    (* ram_style = "distributed" *) logic [PW-1:0] L2 [0:BKD-1];
    (* ram_style = "distributed" *) logic [PW-1:0] L3 [0:BKD-1];
    // Edge read ports: NRP per side. Every port is read every clock at the address the sequencer state puts on it
    // (rp_addr, below); each consumer takes its values from its own ports. Reading the banks through a function
    // from many sites let Vivado merge the reads onto shared ports, and the netlist read zeros (LESSONS 2026-10-11).
    localparam int NRP = 8;
    logic signed [9:0] rpA_a [NRP], rpL_a [NRP];        // edge index -2 .. 257
    logic [9:0]        rp_eA [NRP], rp_eL [NRP];        // + EOFF: bank = [1:0], bank address = [8:2]
    logic [PW-1:0]     rpA_d [NRP], rpL_d [NRP];
    always_comb begin
        for (int q = 0; q < NRP; q++) begin
            rp_eA[q] = 10'(rpA_a[q] + 10'sd2);
            rp_eL[q] = 10'(rpL_a[q] + 10'sd2);
            case (rp_eA[q][1:0])
                2'd0: rpA_d[q] = A0[rp_eA[q][8:2]];
                2'd1: rpA_d[q] = A1[rp_eA[q][8:2]];
                2'd2: rpA_d[q] = A2[rp_eA[q][8:2]];
                default: rpA_d[q] = A3[rp_eA[q][8:2]];
            endcase
            case (rp_eL[q][1:0])
                2'd0: rpL_d[q] = L0[rp_eL[q][8:2]];
                2'd1: rpL_d[q] = L1[rp_eL[q][8:2]];
                2'd2: rpL_d[q] = L2[rp_eL[q][8:2]];
                default: rpL_d[q] = L3[rp_eL[q][8:2]];
            endcase
        end
    end
    // Edge writes: each write site of the sequencer posts a request (side, edge index, value) into its own slot
    // for the current clock, and a bank takes the request addressed to it. So every bank has exactly one write
    // port (the LUT-RAM template) while the sequencer keeps its natural "write edge[k]" form. A later slot wins
    // over an earlier one, matching the statement order the sites had in the sequencer.
    localparam int NRQ = 7;
    logic                 rq_v    [NRQ];   // request valid
    logic                 rq_side [NRQ];   // 0 = above, 1 = left
    logic signed [9:0]    rq_idx  [NRQ];   // edge index -2 .. 257
    logic [PW-1:0]        rq_d    [NRQ];
    logic [9:0]           rq_e    [NRQ];   // idx + EOFF (0 .. 259): bank = rq_e[1:0], address = rq_e[8:2]
    logic          bweA [4], bweL [4];
    logic [6:0]    bwaA [4], bwaL [4];
    logic [PW-1:0] bwdA [4], bwdL [4];
    always_comb begin
        for (int b = 0; b < 4; b++) begin
            bweA[b] = 1'b0; bwaA[b] = '0; bwdA[b] = '0;
            bweL[b] = 1'b0; bwaL[b] = '0; bwdL[b] = '0;
            for (int r = 0; r < NRQ; r++)
                if (rq_v[r] && rq_e[r][1:0] == 2'(b)) begin
                    if (!rq_side[r]) begin bweA[b] = 1'b1; bwaA[b] = rq_e[r][8:2]; bwdA[b] = rq_d[r]; end
                    else             begin bweL[b] = 1'b1; bwaL[b] = rq_e[r][8:2]; bwdL[b] = rq_d[r]; end
                end
        end
    end
    always_ff @(posedge clk) begin
        if (bweA[0]) A0[bwaA[0]] <= bwdA[0];
        if (bweA[1]) A1[bwaA[1]] <= bwdA[1];
        if (bweA[2]) A2[bwaA[2]] <= bwdA[2];
        if (bweA[3]) A3[bwaA[3]] <= bwdA[3];
        if (bweL[0]) L0[bwaL[0]] <= bwdL[0];
        if (bweL[1]) L1[bwaL[1]] <= bwdL[1];
        if (bweL[2]) L2[bwaL[2]] <= bwdL[2];
        if (bweL[3]) L3[bwaL[3]] <= bwdL[3];
    end
    logic [PW-1:0] TL_in;                    // top-left as written through the port

    // ------------------------------------------------------------------ latched parameters
    logic [3:0]  l_mode;
    logic        l_fi;
    logic [2:0]  l_fimode;
    logic [2:0]  l_log2w, l_log2h;
    logic [6:0]  w, h;
    logic [7:0]  wh;
    logic [3:0]  l_bd;
    logic        l_hl, l_ha, l_ft, l_efe;
    logic [6:0]  l_apx, l_lpx;
    logic [8:0]  p_angle;
    logic        up_a, up_l;
    logic [10:0] dx, dy;
    // The directional index products (i+1)*dx (per row) and (j+1)*dy (per lane) are kept in registers and
    // stepped as the position advances (+dx per row, +4*dy per lane step), instead of multiplied every cycle in
    // front of the edge-RAM address (a DSP into the read address was the block's worst path, -2.9 ns at 100 MHz).
    // dx/dy are derived as the block starts (S_IDLE) and the registers loaded in S_SETUP, so they are valid from
    // the first S_PIX cycle with no bypass mux in the address path.
    logic [16:0] idx_i_r;
    logic [16:0] idx_j_r [4];
    // The position after this one (the S_PIX / S_FI2 stepping: four lanes along the row, then the next row), used
    // both for the register updates and for the edge-read address lookahead below.
    logic        nx_row_end;
    logic [5:0]  nx_px, nx_py;
    logic [16:0] nx_idx_i;
    logic [16:0] nx_idx_j [4];
    always_comb begin
        nx_row_end = (px + 7'd4 >= w);
        nx_px = nx_row_end ? 6'd0 : px + 6'd4;
        nx_py = nx_row_end ? py + 6'd1 : py;
        nx_idx_i = nx_row_end ? idx_i_r + 17'(dx) : idx_i_r;
        for (int l = 0; l < 4; l++) nx_idx_j[l] = nx_row_end ? 17'((l + 1) * int'(dy)) : idx_j_r[l] + (17'(dy) << 2);
    end
    // Edge-read address lookahead: the S_PIX read addresses of a position are computed the cycle before, from
    // the next-position values (in S_PIX) or from the current position (any other state, so the first S_PIX
    // cycle finds its addresses ready), into registers. The index -> address arithmetic (shifts, the Z1 limit
    // compare, the mode mux) then no longer sits in front of the edge RAMs' high-fanout address pins, which
    // was 8 ns of routing at 100 MHz in the placed core build.
    logic signed [9:0] la_a0 [4], la_a1 [4], la_l0 [4], la_l1 [4];
    // Z1's edge limit (w+h-1) << up_a is constant over a block: registered (it was computed in front of the
    // edge-RAM address compare every cycle; w, h, up_a are stable at least a cycle before the first S_PIX).
    logic [9:0]  maxb_r;
    always_ff @(posedge clk) maxb_r <= 10'((int'(w) + int'(h) - 1) << int'(up_a));
    logic [PW-1:0] pix_max;
    assign pix_max = PW'((1 << l_bd) - 1);
    wire is_dir = !l_fi && (l_mode >= 4'd1) && (l_mode <= 4'd8);

    // ------------------------------------------------------------------ tables (spec)
    function automatic logic [8:0] mode_to_angle(input logic [3:0] m);
        case (m)
            4'd1: mode_to_angle = 90;  4'd2: mode_to_angle = 180; 4'd3: mode_to_angle = 45;  4'd4: mode_to_angle = 135;
            4'd5: mode_to_angle = 113; 4'd6: mode_to_angle = 157; 4'd7: mode_to_angle = 203; 4'd8: mode_to_angle = 67;
            default: mode_to_angle = 0;
        endcase
    endfunction

    function automatic logic [10:0] dr_intra_derivative(input logic [8:0] a);
        case (a)
            9'd3: dr_intra_derivative = 1023; 9'd6: dr_intra_derivative = 547; 9'd9: dr_intra_derivative = 372;
            9'd14: dr_intra_derivative = 273; 9'd17: dr_intra_derivative = 215; 9'd20: dr_intra_derivative = 178;
            9'd23: dr_intra_derivative = 151; 9'd26: dr_intra_derivative = 132; 9'd29: dr_intra_derivative = 116;
            9'd32: dr_intra_derivative = 102; 9'd36: dr_intra_derivative = 90;  9'd39: dr_intra_derivative = 80;
            9'd42: dr_intra_derivative = 71;  9'd45: dr_intra_derivative = 64;  9'd48: dr_intra_derivative = 57;
            9'd51: dr_intra_derivative = 51;  9'd54: dr_intra_derivative = 45;  9'd58: dr_intra_derivative = 40;
            9'd61: dr_intra_derivative = 35;  9'd64: dr_intra_derivative = 31;  9'd67: dr_intra_derivative = 27;
            9'd70: dr_intra_derivative = 23;  9'd73: dr_intra_derivative = 19;  9'd76: dr_intra_derivative = 15;
            9'd81: dr_intra_derivative = 11;  9'd84: dr_intra_derivative = 7;   9'd87: dr_intra_derivative = 3;
            default: dr_intra_derivative = 0;
        endcase
    endfunction

    function automatic logic [1:0] edge_strength(input logic [7:0] blk, input logic ft, input logic [8:0] d);
        logic [1:0] s;
        s = 0;
        if (!ft) begin
            if (blk <= 8)       begin if (d >= 56) s = 1; end
            else if (blk <= 12) begin if (d >= 40) s = 1; end
            else if (blk <= 16) begin if (d >= 40) s = 1; end
            else if (blk <= 24) begin if (d >= 8) s = 1; if (d >= 16) s = 2; if (d >= 32) s = 3; end
            else if (blk <= 32) begin s = 1; if (d >= 4) s = 2; if (d >= 32) s = 3; end
            else s = 3;
        end else begin
            if (blk <= 8)       begin if (d >= 40) s = 1; if (d >= 64) s = 2; end
            else if (blk <= 16) begin if (d >= 20) s = 1; if (d >= 48) s = 2; end
            else if (blk <= 24) begin if (d >= 4) s = 3; end
            else s = 3;
        end
        edge_strength = s;
    endfunction

    function automatic logic use_upsample(input logic [7:0] blk, input logic ft, input logic [8:0] d);
        if (d == 0 || d >= 40) use_upsample = 0;
        else if (!ft)          use_upsample = (blk <= 16);
        else                   use_upsample = (blk <= 8);
    endfunction

    function automatic logic [8:0] abs_diff(input logic [8:0] a, input logic [8:0] b);
        abs_diff = (a >= b) ? a - b : b - a;
    endfunction

    // smooth-prediction weights (spec 7.11.2.6 Sm_Weights_Tx_*), module-level constants (see FI_TAB below)
    localparam logic [7:0] SMW4 [4] = '{255, 149, 85, 64};
    localparam logic [7:0] SMW8 [8] = '{255, 197, 146, 105, 73, 50, 37, 32};
    localparam logic [7:0] SMW16 [16] = '{255, 225, 196, 170, 145, 123, 102, 84, 68, 54, 43, 33, 26, 20, 17, 16};
    localparam logic [7:0] SMW32 [32] = '{255, 240, 225, 210, 196, 182, 169, 157, 145, 133, 122, 111, 101, 92, 83, 74,
                66, 59, 52, 45, 39, 34, 29, 25, 21, 17, 14, 12, 10, 9, 8, 8};
    localparam logic [7:0] SMW64 [64] = '{255, 248, 240, 233, 225, 218, 210, 203, 196, 189, 182, 176, 169, 163, 156,
                150, 144, 138, 133, 127, 121, 116, 111, 106, 101, 96, 91, 86, 82, 77, 73, 69,
                65, 61, 57, 54, 50, 47, 44, 41, 38, 35, 32, 29, 27, 25, 22, 20, 18, 16, 15,
                13, 12, 10, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4};
    function automatic logic [7:0] sm_weight(input logic [2:0] lg, input logic [5:0] i);
        case (lg)
            3'd2:    sm_weight = SMW4[i[1:0]];
            3'd3:    sm_weight = SMW8[i[2:0]];
            3'd4:    sm_weight = SMW16[i[3:0]];
            3'd5:    sm_weight = SMW32[i[4:0]];
            default: sm_weight = SMW64[i];
        endcase
    endfunction

    function automatic logic [8:0] sm_w9(input logic [2:0] lg, input logic [5:0] i);      // weight, 9 bits
        sm_w9 = {1'b0, sm_weight(lg, i)};
    endfunction
    function automatic logic [8:0] sm_w9c(input logic [2:0] lg, input logic [5:0] i);     // 256 - weight
        sm_w9c = 9'd256 - {1'b0, sm_weight(lg, i)};
    endfunction

    // filter-intra taps (spec 7.11.2.3 Intra_Filter_Taps), a module-level constant: as a function-local array
    // initialised inside the function, Vivado folded the whole filter to zero (and dropped the patch buffer as
    // "unused"), while simulation was fine.
    localparam logic signed [5:0] FI_TAB [5][8][7] = '{
            '{'{-6,10,0,0,0,12,0}, '{-5,2,10,0,0,9,0}, '{-3,1,1,10,0,7,0}, '{-3,1,1,2,10,5,0},
              '{-4,6,0,0,0,2,12}, '{-3,2,6,0,0,2,9}, '{-3,2,2,6,0,2,7}, '{-3,1,2,2,6,3,5}},
            '{'{-10,16,0,0,0,10,0}, '{-6,0,16,0,0,6,0}, '{-4,0,0,16,0,4,0}, '{-2,0,0,0,16,2,0},
              '{-10,16,0,0,0,0,10}, '{-6,0,16,0,0,0,6}, '{-4,0,0,16,0,0,4}, '{-2,0,0,0,16,0,2}},
            '{'{-8,8,0,0,0,16,0}, '{-8,0,8,0,0,16,0}, '{-8,0,0,8,0,16,0}, '{-8,0,0,0,8,16,0},
              '{-4,4,0,0,0,0,16}, '{-4,0,4,0,0,0,16}, '{-4,0,0,4,0,0,16}, '{-4,0,0,0,4,0,16}},
            '{'{-2,8,0,0,0,10,0}, '{-1,3,8,0,0,6,0}, '{-1,2,3,8,0,4,0}, '{0,1,2,3,8,2,0},
              '{-1,4,0,0,0,3,10}, '{-1,3,4,0,0,4,6}, '{-1,2,3,4,0,4,4}, '{-1,2,2,3,4,3,3}},
            '{'{-12,14,0,0,0,14,0}, '{-10,0,14,0,0,12,0}, '{-9,0,0,14,0,11,0}, '{-8,0,0,0,14,10,0},
              '{-10,12,0,0,0,0,14}, '{-9,1,12,0,0,0,12}, '{-8,0,0,12,0,1,11}, '{-7,0,0,1,12,1,9}}
        };
    function automatic logic signed [5:0] fi_tap(input logic [2:0] m, input logic [2:0] pos, input logic [2:0] t);
        fi_tap = (m < 3'd5 && t < 3'd7) ? FI_TAB[m][pos][t] : 6'sd0;
    endfunction

    function automatic int ai(input int idx); ai = idx + EOFF; endfunction

    function automatic logic [PW-1:0] clip1i(input int v, input logic [PW-1:0] mx);
        if (v < 0) clip1i = '0;
        else if (v > int'(mx)) clip1i = mx;
        else clip1i = PW'(v);
    endfunction

    // ------------------------------------------------------------------ control state
    typedef enum logic [3:0] {
        S_IDLE, S_SETUP, S_CORNER, S_EF, S_UP_A, S_UP_L, S_DC_SUM, S_PIX, S_FI2, S_DRAIN, S_DONE
    } state_t;
    state_t state;

    logic [8:0]    k;                 // pass counter
    logic          ef_side;           // 0 above, 1 left (edge filter pass)
    logic [1:0]    ef_str;
    logic [8:0]    ef_sz;             // edge filter sz / upsample numPx
    logic [PW-1:0] o0, o1;            // originals of edge[k-2], edge[k-1] during the edge filter
    logic [PW-1:0] up_save1;          // original buf[1] across the descending upsample pass

    logic [19:0]   dc_sum, dc_part;   // running sum and the sum of the next (up to) 8 edge samples
    logic [8:0]    dc_n;              // number of edge samples summed
    logic [PW-1:0] dc_val;
    // Both edges: (sum + (w+h)/2) / (w+h). With m = min(w, h), w+h = m * {2, 3, 5} (ratios 1:1, 2:1, 4:1), so
    // the division is a shift by log2 m followed by an exact division by 2, 3 or 5, done as a multiply by a
    // scaled reciprocal (exact for all x < 65536; x < 20480 here). The general 20-bit division by one of the
    // twelve possible sizes was 41 CARRY8 levels, the block's worst path.
    logic [2:0]    dc_lmin, dc_ratio;
    logic [19:0]   dc_x;
    logic [31:0]   dc_q3, dc_q5;
    logic [PW-1:0] dc_both;
    always_comb begin
        dc_lmin  = (l_log2w < l_log2h) ? l_log2w : l_log2h;
        dc_ratio = (l_log2w < l_log2h) ? l_log2h - l_log2w : l_log2w - l_log2h;   // 0, 1, 2 -> divisor 2, 3, 5
        dc_x     = (dc_sum + 20'(wh >> 1)) >> dc_lmin;
        dc_q3    = (32'(dc_x[15:0]) * 32'd43691) >> 17;                             // x / 3
        dc_q5    = (32'(dc_x[15:0]) * 32'd52429) >> 18;                             // x / 5
        dc_both  = (dc_ratio == 3'd0) ? PW'(dc_x >> 1) : (dc_ratio == 3'd1) ? PW'(dc_q3) : PW'(dc_q5);
    end

    logic [5:0]    px, py;

    // filter-intra: emitted pixels, one 4x2 patch (8 pixels) per entry, entry = (row>>1)*8 + (col>>2)
    (* ram_style = "distributed" *) logic [8*PW-1:0] fi_buf [0:127];   // one write, four lane reads: LUT RAM
    logic [8*PW-1:0] fi_next_packed;
    // Buffer reads as fixed ports: the current patch row for the four output lanes, and the three neighbouring
    // words the next patch's taps come from. (Read through a function, Vivado dropped the buffer as unused and
    // the hardware predicted zeros for the second row of every patch — caught by the gate-level replay.)
    logic [8*PW-1:0] fi_wq, fi_wup, fi_wupl, fi_wleft;
    logic [PW-1:0]   fi_q_pix [4];
    always_comb begin
        fi_wq    = fi_buf[{py[4:1], px[4:2]}];                               // patch (py>>1, px>>2)
        fi_wup   = fi_buf[{4'(py[4:1] - 4'd1), px[4:2]}];                   // patch above
        fi_wupl  = fi_buf[{4'(py[4:1] - 4'd1), 3'(px[4:2] - 3'd1)}];        // above-left
        fi_wleft = fi_buf[{py[4:1], 3'(px[4:2] - 3'd1)}];                   // left
        for (int l = 0; l < 4; l++) fi_q_pix[l] = py[0] ? fi_wq[(4 + l) * PW +: PW] : fi_wq[l * PW +: PW];
    end

    // ------------------------------------------------------------------ pixel datapath: pred[i][j]
    // Address half: the (up to) two above and two left reads a lane needs for pred[i][j], as {a0, a1, l0, l1};
    // value half: the pixel from the fetched values. Split so that every bank read is an explicit port.
    function automatic logic [39:0] pred_addr(input int i, input int j, input int idx_i, input int idx_j, input int maxb_in);
        int base, idx, maxb, idx2, base2, a0, a1, l0, l1;
        base = 0; idx = 0; maxb = 0; idx2 = 0; base2 = 0; a0 = 0; a1 = 0; l0 = 0; l1 = 0;
        if (l_mode == DC_PRED) begin
        end else if (l_mode == PAETH_PRED) begin
            a0 = j; l0 = i; a1 = -1;
        end else if (l_mode == SMOOTH_PRED) begin
            a0 = j; l0 = i; a1 = int'(w) - 1; l1 = int'(h) - 1;
        end else if (l_mode == SMOOTH_V_PRED) begin
            a0 = j; l1 = int'(h) - 1;
        end else if (l_mode == SMOOTH_H_PRED) begin
            l0 = i; a1 = int'(w) - 1;
        end else if (p_angle == 9'd90) begin
            a0 = j;
        end else if (p_angle == 9'd180) begin
            l0 = i;
        end else if (p_angle < 9'd90) begin                                   // Z1
            idx   = idx_i;
            base  = (idx >> (6 - int'(up_a))) + (j << int'(up_a));
            maxb  = maxb_in;
            if (base < maxb) begin a0 = base; a1 = base + 1; end
            else a0 = maxb;
        end else if (p_angle < 9'd180) begin                                  // Z2
            idx  = (j << 6) - idx_i;
            base = idx >>> (6 - int'(up_a));
            if (base >= -(1 << int'(up_a))) begin
                a0 = base; a1 = base + 1;
            end else begin
                idx2  = (i << 6) - idx_j;
                base2 = idx2 >>> (6 - int'(up_l));
                l0 = base2; l1 = base2 + 1;
            end
        end else begin                                                        // Z3
            idx   = idx_j;
            base  = (idx >> (6 - int'(up_l))) + (i << int'(up_l));
            l0 = base; l1 = base + 1;
        end
        return {10'(a0), 10'(a1), 10'(l0), 10'(l1)};
    endfunction
    // The lookahead sees every block constant live: the Z1 limit combinationally (maxb_r lags a cycle, which the
    // S_PIX read tolerated), and in S_SETUP the position / index values being loaded, since S_SETUP may enter
    // S_PIX directly.
    logic [9:0] maxb_c;
    assign maxb_c = 10'((int'(w) + int'(h) - 1) << int'(up_a));
    always_ff @(posedge clk) begin
        for (int l = 0; l < 4; l++) begin
            if (state == S_PIX && !l_fi)                      // addresses for the position after this one
                {la_a0[l], la_a1[l], la_l0[l], la_l1[l]} <= pred_addr(int'(nx_py), int'(nx_px) + l, int'(nx_idx_i), int'(nx_idx_j[l]), int'(maxb_c));
            else if (state == S_SETUP)                        // the first position, from the values S_SETUP loads
                {la_a0[l], la_a1[l], la_l0[l], la_l1[l]} <= pred_addr(0, l, int'(dx), (l + 1) * int'(dy), int'(maxb_c));
            else
                {la_a0[l], la_a1[l], la_l0[l], la_l1[l]} <= pred_addr(int'(py), int'(px) + l, int'(idx_i_r), int'(idx_j_r[l]), int'(maxb_c));
        end
    end
    // The pixel datapath is two stages. Stage 1 (the S_PIX cycle) fetches a lane's edge values and forms its
    // products: the smooth weights times the four values, or the directional interpolation's two products.
    // Stage 2 sums, rounds and selects, into the output register. In one cycle (edge read, mode mux, products,
    // sums, clip) the lanes were a four-deep DSP cascade into out_pix4, the block's worst path (-3.6 ns at 100 MHz).
    // Outputs therefore land one cycle after S_PIX, and S_DRAIN lets the last one out before done.
    localparam int PREPW = 4 * 23 + 3 * PW + 1;
    function automatic logic [PREPW-1:0] pred_prep(input int i, input int j, input int idx_i, input int idx_j,
                                                   input logic [PW-1:0] vA0, input logic [PW-1:0] vA1,
                                                   input logic [PW-1:0] vL0, input logic [PW-1:0] vL1);
        int base, idx, shift, maxb, idx2, shift2;
        logic [22:0] p0, p1, p2, p3;
        logic        inr;                       // Z1: base < maxb (interpolate; otherwise the pixel is A[maxb] = vA0)
        base = 0; idx = 0; shift = 0; maxb = 0; idx2 = 0; shift2 = 0;
        p0 = '0; p1 = '0; p2 = '0; p3 = '0; inr = 1'b0;
        if (l_mode == SMOOTH_PRED || l_mode == SMOOTH_V_PRED || l_mode == SMOOTH_H_PRED) begin
            // explicitly sized unsigned products (9 x 12 bits): as int arithmetic, the synthesised sum came out
            // off by one in scattered lanes while simulation was exact (gate-level replay, 2026-10-11)
            p0 = 23'(sm_w9(l_log2h, 6'(i)) * vA0);  p1 = 23'(sm_w9c(l_log2h, 6'(i)) * vL1);     // vL1 = L[h-1]
            p2 = 23'(sm_w9(l_log2w, 6'(j)) * vL0);  p3 = 23'(sm_w9c(l_log2w, 6'(j)) * vA1);     // vA1 = A[w-1]
        end else if (l_mode != DC_PRED && l_mode != PAETH_PRED && p_angle != 9'd90 && p_angle != 9'd180) begin
            if (p_angle < 9'd90) begin                                        // Z1: vA0 = A[base] (or A[maxb]), vA1 = A[base+1]
                idx   = idx_i;
                base  = (idx >> (6 - int'(up_a))) + (j << int'(up_a));
                shift = ((idx << int'(up_a)) >> 1) & 31;
                maxb  = int'(maxb_r);
                inr   = base < maxb;
                p0 = 23'(int'(vA0) * (32 - shift)); p1 = 23'(int'(vA1) * shift);
            end else if (p_angle < 9'd180) begin                              // Z2
                idx  = (j << 6) - idx_i;
                base = idx >>> (6 - int'(up_a));
                if (base >= -(1 << int'(up_a))) begin
                    shift = ((idx << int'(up_a)) >> 1) & 31;
                    p0 = 23'(int'(vA0) * (32 - shift)); p1 = 23'(int'(vA1) * shift);
                end else begin
                    idx2   = (i << 6) - idx_j;
                    shift2 = ((idx2 << int'(up_l)) >> 1) & 31;
                    p0 = 23'(int'(vL0) * (32 - shift2)); p1 = 23'(int'(vL1) * shift2);
                end
            end else begin                                                    // Z3
                idx   = idx_j;
                shift = ((idx << int'(up_l)) >> 1) & 31;
                p0 = 23'(int'(vL0) * (32 - shift)); p1 = 23'(int'(vL1) * shift);
            end
        end
        return {p0, p1, p2, p3, vA0, vL0, vA1, inr};
    endfunction
    function automatic logic [PW-1:0] pred_fin(input logic [PREPW-1:0] r);
        logic [22:0] p0, p1, p2, p3;
        logic [PW-1:0] a0, l0, a1, pix_val;
        logic inr;
        int tl, base, pl, pt, ptl;
        {p0, p1, p2, p3, a0, l0, a1, inr} = r;
        tl = 0; base = 0; pl = 0; pt = 0; ptl = 0; pix_val = '0;
        if (l_mode == DC_PRED) begin
            pix_val = dc_val;
        end else if (l_mode == PAETH_PRED) begin                             // a0 = A[j], l0 = L[i], a1 = A[-1]
            tl   = int'(a1);
            base = int'(a0) + int'(l0) - tl;
            pl  = base - int'(l0);  if (pl < 0) pl = -pl;
            pt  = base - int'(a0);  if (pt < 0) pt = -pt;
            ptl = base - tl;        if (ptl < 0) ptl = -ptl;
            if (pl <= pt && pl <= ptl) pix_val = l0;
            else if (pt <= ptl)        pix_val = a0;
            else                       pix_val = PW'(tl);
        end else if (l_mode == SMOOTH_PRED) begin
            pix_val = PW'((p0 + p1 + p2 + p3 + 23'd256) >> 9);
        end else if (l_mode == SMOOTH_V_PRED) begin
            pix_val = PW'((p0 + p1 + 23'd128) >> 8);
        end else if (l_mode == SMOOTH_H_PRED) begin
            pix_val = PW'((p2 + p3 + 23'd128) >> 8);
        end else if (p_angle == 9'd90) begin
            pix_val = a0;
        end else if (p_angle == 9'd180) begin
            pix_val = l0;
        end else if (p_angle < 9'd90 && !inr) begin                          // Z1 beyond the edge: A[maxb]
            pix_val = a0;
        end else begin
            pix_val = PW'((p0 + p1 + 23'd16) >> 5);
        end
        return pix_val;
    endfunction
    logic [PREPW-1:0] prep4 [4];                 // stage 1, this cycle
    always_comb for (int l = 0; l < 4; l++)
        prep4[l] = pred_prep(int'(py), int'(px) + l, int'(idx_i_r), int'(idx_j_r[l]),
                             rpA_d[2 * l], rpA_d[2 * l + 1], rpL_d[2 * l], rpL_d[2 * l + 1]);
    // stage-1 registers: either a lane's prepared operands or a ready pixel (filter-intra rows)
    logic             s_valid, s_raw;
    logic [5:0]       s_x, s_y;
    logic [PREPW-1:0] s_prep [4];
    logic [PW-1:0]    s_pix [4];
    always_ff @(posedge clk) begin                // stage 2
        out_valid <= s_valid && !rst;
        out_x <= s_x; out_y <= s_y;
        for (int l = 0; l < 4; l++) out_pix4[l*PW +: PW] <= s_raw ? s_pix[l] : pred_fin(s_prep[l]);
    end

    // filter-intra: the 4x2 patch at (i2 = py>>1, j4 = px>>2), from edges and already-predicted pixels
    logic [PW-1:0] fi_next [2][4];
    // (the taps p[], i2, j4 and pr live at module scope: declared inside this block, Vivado folded the whole
    // filter to a constant and dropped the patch buffer, while simulation matched dav1d)
    // A filter-intra patch takes two cycles: S_PIX forms the 56 tap products (buffer read, tap mux, one
    // multiply) into fi_m_r, and S_FI2 sums, rounds and clips them, writes the patch and emits its top row.
    // Done in one cycle the products-and-sum were a seven-deep DSP cascade into fi_buf, the block's worst path
    // (-7 ns at 100 MHz, balanced tree or not: Vivado chains the adders through the DSP post-adders).
    int p [7];
    int fi_m [2][4][7];                 // the taps' products, per output (this cycle)
    logic signed [PW+6:0] fi_m_r [2][4][7];   // ... registered (|tap| < 32, so PW+6 bits hold the product)
    int i2, j4, pr;
    assign i2 = int'(py) >> 1;          // (own assigns: in the block below they would look to Verilator like a loop
    assign j4 = int'(px) >> 2;          //  through the read ports)
    always_comb begin
        pr = 0;
        for (int t = 0; t < 7; t++) begin
            p[t] = 0;
            if (t < 5) begin
                if (i2 == 0)                p[t] = int'(rpA_d[t]);                        // A[4*j4+t-1] (t=0: top-left)
                else if (j4 == 0 && t == 0) p[t] = int'(rpL_d[0]);                        // L[2*i2-1]
                else if (t == 0)            p[t] = int'(fi_wupl[7 * PW +: PW]);            // row 2*i2-1, col 4*j4-1
                else                        p[t] = int'(fi_wup[(3 + t) * PW +: PW]);       // row 2*i2-1, col 4*j4+t-1
            end else begin
                if (j4 == 0) p[t] = int'(rpL_d[t - 4]);                                  // L[2*i2 + t-5]
                else if (t == 5) p[t] = int'(fi_wleft[3 * PW +: PW]);                  // row 2*i2,   col 4*j4-1
                else             p[t] = int'(fi_wleft[7 * PW +: PW]);                  // row 2*i2+1, col 4*j4-1
            end
        end
        for (int i1 = 0; i1 < 2; i1++)
            for (int j1 = 0; j1 < 4; j1++) begin
                for (int t = 0; t < 7; t++) fi_m[i1][j1][t] = int'(fi_tap(l_fimode, 3'((i1 << 2) + j1), 3'(t))) * p[t];
                // second cycle: the registered products summed
                pr = 0;
                for (int t = 0; t < 7; t++) pr += int'(fi_m_r[i1][j1][t]);
                if (pr >= 0) pr = (pr + 8) >> 4; else pr = -((-pr + 8) >> 4);   // Round2Signed(pr, 4)
                fi_next[i1][j1] = clip1i(pr, pix_max);
            end
        for (int i1 = 0; i1 < 2; i1++)
            for (int j1 = 0; j1 < 4; j1++)
                fi_next_packed[(i1 * 4 + j1) * PW +: PW] = fi_next[i1][j1];
    end

    // ------------------------------------------------------------------ edge filter step
    // edge[m] = side[m-1] for m = 0..sz-1 (edge[0] = top-left at index -1). Output side[k-1] for k = 1..sz-1
    // from ORIGINAL edge values at clip(k-2..k+2): k-2, k-1 come from o0/o1 (already overwritten), k.. from the array.
    function automatic logic [PW-1:0] ef_calc(input logic [PW-1:0] e0, e1, e2, e3, e4, input logic [1:0] str);
        int s;
        case (str)
            2'd1:    s = 4 * int'(e1) + 8 * int'(e2) + 4 * int'(e3);
            2'd2:    s = 5 * int'(e1) + 6 * int'(e2) + 5 * int'(e3);
            default: s = 2 * int'(e0) + 4 * int'(e1) + 4 * int'(e2) + 4 * int'(e3) + 2 * int'(e4);
        endcase
        ef_calc = PW'((s + 8) >> 4);
    endfunction

    logic [PW-1:0] ef_out;
    int ef_m0, ef_m1, ef_m3, ef_m4;                   // positions kk-2, kk-1, kk+1, kk+2 clamped to 0 .. sz-1
    always_comb begin
        ef_m0 = int'(k) - 2; ef_m1 = int'(k) - 1; ef_m3 = int'(k) + 1; ef_m4 = int'(k) + 2;
        if (ef_m0 < 0) ef_m0 = 0; if (ef_m1 < 0) ef_m1 = 0;
        if (ef_m3 > int'(ef_sz) - 1) ef_m3 = int'(ef_sz) - 1; if (ef_m4 > int'(ef_sz) - 1) ef_m4 = int'(ef_sz) - 1;
    end
    function automatic logic [PW-1:0] ef_src(input int m, input int kk, input int q);
        // original value of edge[m] at step kk: positions 1..kk-1 were overwritten (history o0/o1),
        // position 0 (top-left) and positions >= kk are untouched in the array (read port q holds edge[m-1])
        if (m >= 1 && m == kk - 1)      ef_src = o1;
        else if (m >= 1 && m == kk - 2) ef_src = o0;
        else                            ef_src = ef_side ? rpL_d[q] : rpA_d[q];
    endfunction
    always_comb
        ef_out = ef_calc(ef_src(ef_m0, int'(k), 0), ef_src(ef_m1, int'(k), 1), ef_src(int'(k), int'(k), 2),
                         ef_src(ef_m3, int'(k), 3), ef_src(ef_m4, int'(k), 4), ef_str);

    // ------------------------------------------------------------------ upsample step (descending i)
    // dup[i]=buf[i-2] (i==0 -> buf[-1]), dup[i+1]=buf[i-1], dup[i+2]=buf[i], dup[i+3]=buf[i+1] (clamped to numPx-1).
    // Writes buf[2i-1] and buf[2i]. Processing i descending keeps every read original except buf[1] (saved).
    logic [PW-1:0] up_s, up_d2;
    int up_i, up_i0, up_i3;                           // i = k-1 and the clamped tap positions i-2, i+1
    always_comb begin
        up_i  = int'(k) - 1;
        up_i0 = (up_i - 2 < -1) ? -1 : up_i - 2;
        up_i3 = (up_i + 1 > int'(ef_sz) - 1) ? int'(ef_sz) - 1 : up_i + 1;
    end
    always_comb begin
        int i, n, s;
        logic [PW-1:0] d0, d1, d2, d3;
        logic side;
        s = 0; d0 = '0; d1 = '0; d2 = '0; d3 = '0;
        side = (state == S_UP_L);
        i = up_i; n = int'(ef_sz);
        d0 = side ? rpL_d[0] : rpA_d[0];               // ports 0..3 hold buf[i-2], buf[i-1], buf[i], buf[i+1]
        d1 = side ? rpL_d[1] : rpA_d[1];
        d2 = side ? rpL_d[2] : rpA_d[2];
        d3 = side ? rpL_d[3] : rpA_d[3];
        if (i - 2 == 1) d0 = up_save1;
        if (i - 1 == 1) d1 = up_save1;
        if (i == 1)     d2 = up_save1;
        if (i + 1 == 1 && n - 1 >= 1) d3 = up_save1;
        s = -int'(d0) + 9 * int'(d1) + 9 * int'(d2) - int'(d3);
        s = (s + 8) >>> 4;                 // arithmetic: s may be negative (then Clip1 -> 0)
        up_s  = clip1i(s, pix_max);
        up_d2 = d2;
    end

    // DC: the edge samples in order LeftCol[0..h-1] (if have_left) then AboveRow[0..w-1] (if have_above);
    // dc_part sums the 8 of them at k .. k+7 (masked past the end)
    always_comb begin
        dc_n = (l_hl ? 9'(h) : 9'd0) + (l_ha ? 9'(w) : 9'd0);
        dc_part = 20'd0;
        for (int l = 0; l < 8; l++) begin
            int kk;
            kk = int'(k) + l;
            if (kk < int'(dc_n)) begin
                if (l_hl && kk < int'(h)) dc_part += 20'(rpL_d[l]);                     // port l holds L[kk]
                else                      dc_part += 20'(rpA_d[l]);                     // or A[kk - h]
            end
        end
    end
    // ---- read-port addresses by state (ports not named read address 0, harmlessly)
    always_comb begin
        for (int q = 0; q < NRP; q++) begin rpA_a[q] = '0; rpL_a[q] = '0; end
        case (state)
            S_PIX: if (l_fi) begin                                  // filter-intra taps from the edges
                for (int t = 0; t < 5; t++) rpA_a[t] = 10'((j4 << 2) + t - 1);
                for (int t = 0; t < 3; t++) rpL_a[t] = 10'((i2 << 1) - 1 + t);
            end else begin
                for (int l = 0; l < 4; l++)
                    {rpA_a[2 * l], rpA_a[2 * l + 1], rpL_a[2 * l], rpL_a[2 * l + 1]} = {la_a0[l], la_a1[l], la_l0[l], la_l1[l]};
            end
            S_DC_SUM: for (int l = 0; l < 8; l++) begin                 // L[k+l] and A[k+l-h] (A side clamped at 0)
                rpL_a[l] = 10'(int'(k) + l);
                rpA_a[l] = (l_hl && int'(k) + l < int'(h)) ? 10'sd0 : 10'(int'(k) + l - (l_hl ? int'(h) : 0));
            end
            S_EF: begin                                                 // edge[m] = side[m-1] for m0 m1 k m3 m4; edge[k]
                rpA_a[0] = 10'(ef_m0 - 1); rpA_a[1] = 10'(ef_m1 - 1); rpA_a[2] = 10'(int'(k) - 1);
                rpA_a[3] = 10'(ef_m3 - 1); rpA_a[4] = 10'(ef_m4 - 1); rpA_a[5] = 10'(int'(k) - 1);
                for (int q = 0; q < 6; q++) rpL_a[q] = rpA_a[q];
            end
            S_UP_A, S_UP_L: begin                                       // buf[i-2] buf[i-1] buf[i] buf[i+1]; buf[-1]; buf[1]
                rpA_a[0] = 10'(up_i0); rpA_a[1] = 10'(up_i - 1); rpA_a[2] = 10'(up_i); rpA_a[3] = 10'(up_i3);
                rpA_a[4] = -10'sd1; rpA_a[5] = 10'sd1;
                for (int q = 0; q < 6; q++) rpL_a[q] = rpA_a[q];
            end
            S_CORNER: begin rpL_a[0] = 10'sd0; rpA_a[0] = -10'sd1; rpA_a[1] = 10'sd0; end   // L[0], A[-1], A[0]
            default: ;
        endcase
    end
    logic first_of_patch;           // filter-intra: an even row emits the freshly computed 4x2 patch's top row
    assign first_of_patch = l_fi && (py[0] == 1'b0);

    // ------------------------------------------------------------------ sequencer
    // ------------------------------------------------------------------ edge-write requests (one slot per site)
    logic [PW-1:0] corner_v;                 // smoothed top-left for the directional corner case
    assign corner_v = PW'((int'(rpL_d[0]) * 5 + int'(rpA_d[0]) * 6 + int'(rpA_d[1]) * 5 + 8) >> 4);   // L[0], A[-1], A[0]
    task automatic post(input int slot, input logic side, input int idx, input logic [PW-1:0] d);
        rq_v[slot] = 1'b1; rq_side[slot] = side; rq_idx[slot] = 10'(idx); rq_d[slot] = d;
    endtask
    always_comb begin
        for (int r = 0; r < NRQ; r++) begin rq_v[r] = 1'b0; rq_side[r] = 1'b0; rq_idx[r] = '0; rq_d[r] = '0; end
        if (!rst) case (state)
            S_IDLE: begin
                if (edge_we4)                            // edge_idx is a multiple of 4: entries idx .. idx+3
                    for (int l = 0; l < 4; l++) post(l, 1'b0, int'(edge_idx) + l, edge_data4[l * PW +: PW]);
                if (edge_we && edge_side == 2'd0) post(4, 1'b0, int'(edge_idx), edge_data);
                if (edge_we && edge_side == 2'd1) post(4, 1'b1, int'(edge_idx), edge_data);
                if (start) begin post(5, 1'b0, -1, TL_in); post(6, 1'b1, -1, TL_in); end
            end
            S_CORNER: if (p_angle > 90 && p_angle < 180 && wh >= 24) begin
                post(0, 1'b0, -1, corner_v);
                post(1, 1'b1, -1, corner_v);
            end
            S_EF: if (!(ef_str == 0 || k >= ef_sz))    // side[k-1] (= edge[k]) <- filtered
                post(0, ef_side, int'(k) - 1, ef_out);
            S_UP_A: if (k != 0) begin                  // upsampled pair for i = k-1, plus edge[-2] on the last step
                post(0, 1'b0, 2 * (int'(k) - 1) - 1, up_s);
                post(1, 1'b0, 2 * (int'(k) - 1), up_d2);
                if (k == 9'd1) post(2, 1'b0, -2, rpA_d[4]);     // A[-1] on port 4
            end
            S_UP_L: if (k != 0) begin
                post(0, 1'b1, 2 * (int'(k) - 1) - 1, up_s);
                post(1, 1'b1, 2 * (int'(k) - 1), up_d2);
                if (k == 9'd1) post(2, 1'b1, -2, rpL_d[4]);
            end
            default: ;
        endcase
        for (int r = 0; r < NRQ; r++) rq_e[r] = 10'(rq_idx[r] + 10'sd2);
    end

    logic [8:0] pa_c;                        // the block's prediction angle, from the inputs at start
    assign pa_c = mode_to_angle(mode) + 9'(signed'({{6{angle_delta[2]}}, angle_delta}) * 3);
    always_ff @(posedge clk) begin
        done      <= 1'b0;
        s_valid   <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
        end else begin
            case (state)
            S_IDLE: begin
                if (edge_we) begin
                    case (edge_side)
                        2'd0, 2'd1: ;                   // posted as edge-write requests below
                        default: TL_in <= edge_data;
                    endcase
                end
                if (start) begin
                    l_mode <= mode; l_fi <= use_filter_intra; l_fimode <= filter_intra_mode;
                    l_log2w <= log2w; l_log2h <= log2h;
                    w <= 7'd1 << log2w; h <= 7'd1 << log2h;
                    wh <= 8'(7'd1 << log2w) + 8'(7'd1 << log2h);
                    l_bd <= bit_depth;
                    l_hl <= have_left; l_ha <= have_above; l_ft <= filter_type; l_efe <= edge_filter_en;
                    l_apx <= above_px; l_lpx <= left_px;
                    p_angle <= pa_c;
                    dx <= (pa_c < 90) ? dr_intra_derivative(pa_c) :
                          (pa_c < 180) ? dr_intra_derivative(9'd180 - pa_c) : 11'd0;
                    dy <= (pa_c > 90 && pa_c < 180) ? dr_intra_derivative(pa_c - 9'd90) :
                          (pa_c > 180) ? dr_intra_derivative(9'd270 - pa_c) : 11'd0;
                    up_a <= 1'b0; up_l <= 1'b0;
                    busy  <= 1'b1;
                    state <= S_SETUP;
                end
            end
            S_SETUP: begin
                px <= 0; py <= 0; dc_sum <= 0; k <= 0;
                idx_i_r <= 17'(dx);                                     // (0+1)*dx, (l+1)*dy for the first row
                for (int l = 0; l < 4; l++) idx_j_r[l] <= 17'((l + 1) * int'(dy));
                if (is_dir) begin
                    state <= (l_efe && p_angle != 90 && p_angle != 180) ? S_CORNER : S_PIX;
                end else if (!l_fi && l_mode == DC_PRED) begin
                    if (!l_hl && !l_ha) begin dc_val <= PW'(1 << (l_bd - 1)); state <= S_PIX; end
                    else state <= S_DC_SUM;
                end else state <= S_PIX;
            end
            S_CORNER: begin
                // set up the above-edge filter pass (or the left one if there is no above)
                if (l_ha) begin
                    ef_side <= 1'b0;
                    ef_str  <= edge_strength(wh, l_ft, abs_diff(p_angle, 9'd90));
                    ef_sz   <= 9'(l_apx) + (p_angle < 90 ? 9'(h) : 9'd0) + 9'd1;
                end else begin
                    ef_side <= 1'b1;
                    ef_str  <= edge_strength(wh, l_ft, abs_diff(p_angle, 9'd180));
                    ef_sz   <= 9'(l_lpx) + (p_angle > 180 ? 9'(w) : 9'd0) + 9'd1;
                end
                k <= (l_ha || l_hl) ? 9'd1 : 9'd0;      // S_UP_A expects k == 0 on entry
                state <= (l_ha || l_hl) ? S_EF : S_UP_A;
            end
            S_EF: begin
                if (ef_str == 0 || k >= ef_sz) begin
                    if (!ef_side && l_hl) begin
                        ef_side <= 1'b1;
                        ef_str  <= edge_strength(wh, l_ft, abs_diff(p_angle, 9'd180));
                        ef_sz   <= 9'(l_lpx) + (p_angle > 180 ? 9'(w) : 9'd0) + 9'd1;
                        k <= 9'd1;
                    end else begin
                        state <= S_UP_A; k <= 0;
                    end
                end else begin
                    // side[k-1] (= edge[k]) <- filtered; remember its original for the next two steps
                    o0 <= o1;
                    o1 <= ef_side ? rpL_d[5] : rpA_d[5];           // edge[k] = side[k-1], on port 5
                    k  <= k + 9'd1;
                end
            end
            S_UP_A: begin
                if (k == 0) begin
                    if (is_dir && l_efe && use_upsample(wh, l_ft, abs_diff(p_angle, 9'd90))) begin
                        up_a  <= 1'b1;
                        ef_sz <= 9'(w) + (p_angle < 90 ? 9'(h) : 9'd0);
                        k     <= 9'(w) + (p_angle < 90 ? 9'(h) : 9'd0);
                        up_save1 <= rpA_d[5];                      // A[1] on port 5
                    end else begin
                        state <= S_UP_L;
                    end
                end else begin
                    if (k == 9'd1) begin
                        state <= S_UP_L; k <= 0;
                    end else k <= k - 9'd1;
                end
            end
            S_UP_L: begin
                if (k == 0) begin
                    if (is_dir && l_efe && use_upsample(wh, l_ft, abs_diff(p_angle, 9'd180))) begin
                        up_l  <= 1'b1;
                        ef_sz <= 9'(h) + (p_angle > 180 ? 9'(w) : 9'd0);
                        k     <= 9'(h) + (p_angle > 180 ? 9'(w) : 9'd0);
                        up_save1 <= rpL_d[5];
                    end else begin
                        state <= S_PIX;
                    end
                end else begin
                    if (k == 9'd1) begin
                        state <= S_PIX;
                    end else k <= k - 9'd1;
                end
            end
            S_DC_SUM: begin
                if (k < dc_n) begin dc_sum <= dc_sum + dc_part; k <= k + 9'd8; end
                else begin
                    // both edges: (sum + (w+h)/2) / (w+h), an exact integer division by one of 12 possible sizes
                    if (l_hl && l_ha) dc_val <= dc_both;
                    else if (l_hl)    dc_val <= PW'((dc_sum + 20'(h >> 1)) >> l_log2h);
                    else              dc_val <= PW'((dc_sum + 20'(w >> 1)) >> l_log2w);
                    state <= S_PIX;
                end
            end
            S_PIX: begin
                if (first_of_patch) begin                           // filter-intra, even row: products this cycle
                    for (int i1 = 0; i1 < 2; i1++)
                        for (int j1 = 0; j1 < 4; j1++)
                            for (int t = 0; t < 7; t++) fi_m_r[i1][j1][t] <= (PW + 7)'(fi_m[i1][j1][t]);
                    state <= S_FI2;
                end else begin
                    for (int l = 0; l < 4; l++) begin s_prep[l] <= prep4[l]; s_pix[l] <= fi_q_pix[l]; end
                    s_raw <= l_fi;                                  // filter-intra odd rows: read back from the patch buffer
                    s_valid <= 1'b1;
                    s_x <= px; s_y <= py;
                    px <= nx_px; idx_i_r <= nx_idx_i;
                    for (int l = 0; l < 4; l++) idx_j_r[l] <= nx_idx_j[l];
                    if (nx_row_end) begin
                        if (py + 7'd1 == h) state <= S_DRAIN;
                        else py <= nx_py;
                    end
                end
            end
            S_FI2: begin                                            // the patch, summed: stored and its top row emitted
                fi_buf[{py[4:1], px[4:2]}] <= fi_next_packed;       // filter-intra blocks are at most 32x32
                for (int l = 0; l < 4; l++) s_pix[l] <= fi_next[0][l];
                s_raw <= 1'b1;
                s_valid <= 1'b1;
                s_x <= px; s_y <= py;
                state <= S_PIX;
                px <= nx_px; idx_i_r <= nx_idx_i;
                for (int l = 0; l < 4; l++) idx_j_r[l] <= nx_idx_j[l];
                if (nx_row_end) begin
                    if (py + 7'd1 == h) state <= S_DRAIN;
                    else py <= nx_py;
                end
            end
            S_DRAIN: state <= S_DONE;                               // the last pixels leave stage 2 before done
            S_DONE: begin
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
