// Tile syntax decoder top (intra frames): decode_tile( ) = superblock raster loop + decode_partition( )
// quadtree (explicit stack) + blk_syntax per block, over a shared msac / cdf_store / symbol sequencers,
// blk_ctx neighbour storage and coef_rd. Software feeds hdr_t and the tile bytes; the block and
// transform-block records stream out (Quant of the current transform block through q_addr/q_data while
// tx_done is held, until tx_ack).
//
// Palette blocks are held after their record (pal_hold) until blk_ack, so the colour index map can be read
// through pm_* (pm_plane/pm_x/pm_y -> pm_idx, 1-cycle latency) before the next block overwrites it.
// Intra block copy is decoded (mv_mem holds the frame's motion info for the MV stacks). Inter frames: not yet.
module tile_syntax
  import cdf_map_pkg::*;
  import blk_tables_pkg::*;
  import syn_pkg::*;
#(
    parameter int ML2R = 9,                  // log2 of the motion-info memory rows / columns (4x4 units)
    parameter int ML2C = 10
) (
    input  logic              clk,
    input  logic              rst,
    input  hdr_t              hdr,
    // tile bytes -> msac
    input  logic [7:0]        in_data,
    input  logic              in_valid,
    output logic              in_ready,
    input  logic              in_eos,
    // cdf defaults (host)
    input  logic              def_we,
    input  logic [CDF_AW-1:0] def_addr,
    input  logic [245:0]      def_data,
    // control
    input  logic              tile_start,
    output logic              tile_busy,
    output logic              tile_done,
    output logic              unsupported,
    // records
    output logic              sb_start_o,      // superblock start (for the reconstruction stage's BlockDecoded flags)
    output logic [10:0]       sb_r_o, sb_c_o,
    output logic              blk_info,        // blk_rec valid, before the block's transform blocks
    output logic              blk_done,
    output blk_rec_t          blk_rec,
    output logic              tx_done,
    output tx_rec_t           tx_rec,
    input  logic              tx_ack,
    output logic              lr_done,
    output lr_rec_t           lr_rec,
    input  logic [2:0]        q_slot_w,        // coef_rd slot for the next transform block (rec_fifo w_slot; 0 without a queue)
    input  logic [2:0]        q_slot_r,
    input  logic [9:0]        q_addr,
    output logic signed [20:0] q_data,
    // palette colour map of the block held on pal_hold
    output logic              pal_hold,
    input  logic              blk_ack,
    input  logic              pm_plane,
    input  logic [5:0]        pm_x, pm_y,
    output logic [2:0]        pm_idx
);
    localparam logic [4:0] BLOCK_8X8 = 5'd3, BLOCK_128X128 = 5'd15;
    localparam logic [3:0] P_NONE = 4'd0, P_HORZ = 4'd1, P_VERT = 4'd2, P_SPLIT = 4'd3, P_HORZ_A = 4'd4, P_HORZ_B = 4'd5,
                           P_VERT_A = 4'd6, P_VERT_B = 4'd7, P_HORZ_4 = 4'd8, P_VERT_4 = 4'd9;

    // ------------------------------------------------------------------ shared decode resources
    logic req_valid, req_ready, resp_valid;
    logic [1:0] req_kind; logic [3:0] req_n; logic [239:0] req_cdf; logic [5:0] req_cnt;
    logic [3:0] resp_sym; logic [15:0] resp_rng; logic [239:0] resp_cdf; logic [5:0] resp_cnt;
    logic msac_init;
    msac u_msac (.clk, .rst, .in_data, .in_valid, .in_ready, .in_eos, .init(msac_init), .cdf_update_en(!hdr.disable_cdf_update),
                 .req_valid, .req_ready, .req_kind, .req_n, .req_cdf, .req_cnt,
                 .resp_valid, .resp_sym, .resp_rng, .resp_cdf, .resp_cnt);

    logic rd_en, wb_we, cdf_init, cdf_busy; logic [CDF_AW-1:0] rd_addr, wb_addr; logic [245:0] rd_data, wb_data;
    cdf_store u_cdf (.clk, .rst, .def_we, .def_addr, .def_data, .init(cdf_init), .busy(cdf_busy),
                     .rd_en, .rd_addr, .rd_data, .wb_we, .wb_addr, .wb_data);

    // two sequencer masters: 'c' = coef_rd, 'k' = control (partition FSM + blk_syntax)
    logic c_go, c_done, c_busy, k_go, k_done, k_busy; logic [CDF_AW-1:0] c_addr, k_addr; logic [3:0] c_n, k_n, c_sym, k_sym; logic [1:0] c_kind, k_kind;
    logic [15:0] k_f; logic [245:0] k_row;
    logic c_rd_en, k_rd_en, c_wb_we, k_wb_we, c_req_valid, k_req_valid;
    logic [CDF_AW-1:0] c_rd_addr, k_rd_addr, c_wb_addr, k_wb_addr;
    logic [245:0] c_wb_data, k_wb_data;
    logic [1:0] c_req_kind, k_req_kind; logic [3:0] c_req_n, k_req_n; logic [239:0] c_req_cdf, k_req_cdf; logic [5:0] c_req_cnt, k_req_cnt;
    logic sel_k;
    assign sel_k = k_busy || k_go;            // control never issues while coef_rd has a symbol in flight

    sym_seq u_sq_c (.clk, .rst, .go(c_go), .addr(c_addr), .n(c_n), .kind(c_kind), .f_in(16'd0), .row_out(), .busy(c_busy), .done(c_done), .sym(c_sym),
                    .cdf_rd_en(c_rd_en), .cdf_rd_addr(c_rd_addr), .cdf_rd_data(rd_data),
                    .cdf_wb_we(c_wb_we), .cdf_wb_addr(c_wb_addr), .cdf_wb_data(c_wb_data),
                    .req_valid(c_req_valid), .req_ready(req_ready && !sel_k), .req_kind(c_req_kind), .req_n(c_req_n), .req_cdf(c_req_cdf), .req_cnt(c_req_cnt),
                    .resp_valid(resp_valid && !sel_k), .resp_sym, .resp_cdf, .resp_cnt);
    sym_seq u_sq_k (.clk, .rst, .go(k_go), .addr(k_addr), .n(k_n), .kind(k_kind), .f_in(k_f), .row_out(k_row), .busy(k_busy), .done(k_done), .sym(k_sym),
                    .cdf_rd_en(k_rd_en), .cdf_rd_addr(k_rd_addr), .cdf_rd_data(rd_data),
                    .cdf_wb_we(k_wb_we), .cdf_wb_addr(k_wb_addr), .cdf_wb_data(k_wb_data),
                    .req_valid(k_req_valid), .req_ready(req_ready && sel_k), .req_kind(k_req_kind), .req_n(k_req_n), .req_cdf(k_req_cdf), .req_cnt(k_req_cnt),
                    .resp_valid(resp_valid && sel_k), .resp_sym, .resp_cdf, .resp_cnt);
    always_comb begin
        rd_en   = sel_k ? k_rd_en   : c_rd_en;
        rd_addr = sel_k ? k_rd_addr : c_rd_addr;
        wb_we   = sel_k ? k_wb_we   : c_wb_we;
        wb_addr = sel_k ? k_wb_addr : c_wb_addr;
        wb_data = sel_k ? k_wb_data : c_wb_data;
        req_valid = sel_k ? k_req_valid : c_req_valid;
        req_kind  = sel_k ? k_req_kind  : c_req_kind;
        req_n     = sel_k ? k_req_n     : c_req_n;
        req_cdf   = sel_k ? k_req_cdf   : c_req_cdf;
        req_cnt   = sel_k ? k_req_cnt   : c_req_cnt;
    end

    // ------------------------------------------------------------------ coef_rd
    logic [4:0] cf_tx; logic cf_ptype, cf_start_a, cf_done_a, cf_all_zero, cf_start_b, cf_done_b;
    logic [3:0] cf_az_ctx, cf_tx_type; logic [1:0] cf_dcs_ctx; logic [10:0] cf_eob; logic [5:0] cf_cul; logic [1:0] cf_dccat;
    logic cf_nonconf;
    coef_rd u_coef (.clk, .rst, .cfg_tx(cf_tx), .cfg_ptype(cf_ptype), .start_a(cf_start_a), .az_ctx(cf_az_ctx), .done_a(cf_done_a), .all_zero(cf_all_zero),
                    .start_b(cf_start_b), .tx_type(cf_tx_type), .dcs_ctx(cf_dcs_ctx), .done_b(cf_done_b), .eob_o(cf_eob), .cul_level(cf_cul), .dc_category(cf_dccat),
                    .nonconformant(cf_nonconf), .w_slot(q_slot_w), .q_slot(q_slot_r), .q_addr, .q_data,
                    .sq_go(c_go), .sq_addr(c_addr), .sq_n(c_n), .sq_kind(c_kind), .sq_done(c_done), .sq_sym(c_sym));

    // ------------------------------------------------------------------ blk_ctx
    logic clear_above, clear_left, sbrow_end;
    logic nb_req_b, nb_req_p, nb_req, nb_valid;
    logic [10:0] nb_r, nb_c; logic [4:0] nb_bs;
    logic avail_u, avail_l, has_chroma, avail_u_chroma, avail_l_chroma;
    logic [3:0] a_ymode, l_ymode; logic a_skip, l_skip; logic [4:0] a_misize, l_misize, a_txsz, l_txsz;
    logic [3:0] a_pal_y, l_pal_y, a_pal_uv, l_pal_uv, seg_ul, seg_u, seg_l;
    logic [95:0] a_col_y, l_col_y, a_col_u, l_col_u;
    logic ctx_we, ctx_wbusy; logic [3:0] w_ymode; logic w_skip; logic [2:0] w_seg; logic [4:0] w_txsz;
    logic [3:0] w_pal_y, w_pal_uv; logic [95:0] w_col_y, w_col_u;
    logic tx_req, tx_valid, tx_we, rbc_we, ctx_tbusy; logic [1:0] tx_plane; logic [10:0] tx_x4, tx_y4; logic [4:0] tx_sz, tx_bsize;
    logic [3:0] az_ctx; logic [1:0] dcs_ctx; logic [5:0] w_cul; logic [1:0] w_dccat;
    // block position for the ctx query: partition FSM (node) or blk_syntax (block)
    logic blk_active;
    logic [10:0] p_r, p_c; logic [4:0] p_bs;
    logic [10:0] b_r, b_c; logic [4:0] b_bs;
    assign nb_req = blk_active ? nb_req_b : nb_req_p;
    assign nb_r = blk_active ? b_r : p_r;
    assign nb_c = blk_active ? b_c : p_c;
    assign nb_bs = blk_active ? b_bs : p_bs;

    blk_ctx u_ctx (.clk, .rst, .mi_cols(hdr.mi_cols), .mi_rows(hdr.mi_rows),
                   .mi_col_start(hdr.mi_col_start), .mi_col_end(hdr.mi_col_end), .mi_row_start(hdr.mi_row_start), .mi_row_end(hdr.mi_row_end),
                   .ssx(hdr.ssx), .ssy(hdr.ssy), .mono(hdr.mono), .sb128(hdr.sb128),
                   .clear_above, .clear_left, .sbrow_end,
                   .blk_req(nb_req), .blk_r(nb_r), .blk_c(nb_c), .blk_bsize(nb_bs), .nb_valid,
                   .avail_u, .avail_l, .has_chroma, .avail_u_chroma, .avail_l_chroma,
                   .a_ymode, .l_ymode, .a_skip, .l_skip, .a_misize, .l_misize, .a_txsz, .l_txsz, .a_is_inter, .l_is_inter, .a_recs, .l_recs,
                   .a_pal_y, .l_pal_y, .a_pal_uv, .l_pal_uv, .seg_ul, .seg_u, .seg_l, .a_col_y, .l_col_y, .a_col_u, .l_col_u,
                   .blk_we(ctx_we), .w_ymode, .w_skip, .w_seg, .w_txsz, .w_is_inter, .w_vartx, .w_txsz_col, .w_txsz_row, .w_pal_y, .w_pal_uv, .w_col_y, .w_col_u, .wbusy(ctx_wbusy),
                   .tx_req, .tx_plane, .tx_x4, .tx_y4, .tx_sz, .tx_bsize, .tx_valid, .az_ctx, .dcs_ctx,
                   .tx_we, .w_cul, .w_dccat, .rbc_we);
    assign ctx_tbusy = 1'b0;   // blk_ctx's tx path is single-request; blk_syntax serialises its own use

    // ------------------------------------------------------------------ blk_syntax
    logic b_start, b_busy, b_unsup, sb_start;
    logic bq_go; logic [CDF_AW-1:0] bq_addr; logic [3:0] bq_n; logic [1:0] bq_kind;
    // motion info of the frame (intra block copy)
    logic mvm_we, mvm_busy, mvm_clr, mvr_written; logic [10:0] mvm_r, mvm_c, mvr_row, mvr_col; logic [5:0] mvm_bw4, mvm_bh4; mv_ent_t mvm_data, mvr_ent;
    mv_mem #(.ML2R(ML2R), .ML2C(ML2C)) u_mvm (.clk, .rst, .we(mvm_we), .w_r(mvm_r), .w_c(mvm_c), .w_bw4(mvm_bw4), .w_bh4(mvm_bh4),
                                              .w_data(mvm_data), .busy(mvm_busy), .clr(mvm_clr), .clr_row(sb_r), .clr_n(sb4),
                                              .rd_row(mvr_row), .rd_col(mvr_col), .rd_ent(mvr_ent), .rd_written(mvr_written));
    logic a_is_inter, l_is_inter, w_is_inter, w_vartx; logic [16*24*2-1:0] a_recs, l_recs; logic [159:0] w_txsz_col, w_txsz_row;
    tx_rec_t b_tx;
    always_comb begin tx_rec = b_tx; tx_rec.slot = q_slot_w; end
    blk_syntax u_blk (.clk, .rst, .hdr, .sb_start, .tile_start, .start(b_start), .r(b_r), .c(b_c), .bsize(b_bs), .busy(b_busy),
                      .blk_info, .blk_done, .blk_rec, .unsupported(b_unsup), .tx_done, .tx_rec(b_tx), .tx_ack, .blk_ack, .pal_hold,
                      .a_pal_y, .l_pal_y, .a_pal_uv, .l_pal_uv, .a_col_y, .l_col_y, .a_col_u, .l_col_u, .w_pal_y, .w_pal_uv, .w_col_y, .w_col_u,
                      .pm_plane, .pm_x, .pm_y, .pm_idx,
                      .sq_go(bq_go), .sq_addr(bq_addr), .sq_n(bq_n), .sq_kind(bq_kind), .sq_done(k_done), .sq_sym(k_sym),
                      .nb_req(nb_req_b), .nb_valid, .avail_u, .avail_l, .has_chroma, .a_ymode, .l_ymode, .a_skip, .l_skip, .a_txsz, .l_txsz,
                      .a_misize, .l_misize, .a_is_inter, .l_is_inter, .a_recs, .l_recs,
                      .seg_ul, .seg_u, .seg_l, .ctx_we, .w_ymode, .w_skip, .w_seg, .w_txsz, .w_is_inter, .w_vartx, .w_txsz_col, .w_txsz_row, .ctx_wbusy,
                      .tx_req, .tx_plane, .tx_x4, .tx_y4, .tx_sz_o(tx_sz), .tx_bsize, .tx_valid, .az_ctx, .dcs_ctx, .tx_we, .w_cul, .w_dccat, .rbc_we, .ctx_tbusy,
                      .cf_tx, .cf_ptype, .cf_start_a, .cf_az_ctx, .cf_done_a, .cf_all_zero, .cf_start_b, .cf_tx_type, .cf_dcs_ctx, .cf_done_b, .cf_eob, .cf_cul, .cf_dccat,
                      .mvm_we, .mvm_r, .mvm_c, .mvm_bw4, .mvm_bh4, .mvm_data, .mvm_busy, .mvr_row, .mvr_col, .mvr_ent, .mvr_written);

    // ------------------------------------------------------------------ partition / superblock FSM
    typedef struct packed {
        logic [10:0] r;
        logic [10:0] c;
        logic [4:0]  bs;
        logic [3:0]  part;
        logic [2:0]  child;
        logic        has_rows;
        logic        has_cols;
    } node_t;
    node_t stack [0:7];
    logic [3:0] sp;
    logic [2:0] spi, spp;                      // index of top, of parent
    assign spi = 3'(sp - 4'd1);
    assign spp = 3'(sp - 4'd2);
    node_t top;
    assign top = stack[spi];

    typedef enum logic [4:0] {T_IDLE, T_INIT, T_SB, T_LR_W, T_SB_ROW_END, T_PUSH_ROOT, T_NODE, T_NB_W, T_PART_W, T_PEEK_W, T_BOOL_W, T_CHILD, T_BLK_W, T_DONE} t_t;
    t_t st;
    logic [10:0] sb_r, sb_c;
    logic [5:0]  sb4;                          // 16 or 32
    assign sb4 = hdr.sb128 ? 6'd32 : 6'd16;
    logic [5:0] half4, quarter4;
    logic [4:0] sub_size, split_size;
    logic [3:0] part_now;
    logic [15:0] psum;
    logic [1:0] part_ctx;
    logic [2:0] bsl;
    always_comb begin
        half4 = num4x4w(top.bs) >> 1; quarter4 = half4 >> 1;
        bsl = mi_w_log2(top.bs);
        part_ctx = {avail_l && (mi_h_log2(l_misize) < bsl), avail_u && (mi_w_log2(a_misize) < bsl)};
        sub_size = partition_subsize(top.part, top.bs);
        split_size = partition_subsize(P_SPLIT, top.bs);
    end
    // psum over the peeked partition row (inverted entries: mass(p) = icdf[p-1] - icdf[p])
    function automatic logic [15:0] mass(input logic [245:0] row, input int p);
        logic [15:0] a, b;
        a = row[16*(p-1) +: 16];
        b = (p < 15) ? row[16*p +: 16] : 16'd0;
        mass = a - b;
    endfunction
    logic [15:0] psum_h, psum_v;    // split_or_horz / split_or_vert
    always_comb begin
        psum_h = mass(k_row, 2) + mass(k_row, 3) + mass(k_row, 4) + mass(k_row, 6) + mass(k_row, 7) + ((top.bs != BLOCK_128X128) ? mass(k_row, 9) : 16'd0);
        psum_v = mass(k_row, 1) + mass(k_row, 3) + mass(k_row, 4) + mass(k_row, 5) + mass(k_row, 6) + ((top.bs != BLOCK_128X128) ? mass(k_row, 8) : 16'd0);
    end
    logic [CDF_AW-1:0] part_addr;
    logic [3:0] part_n;
    always_comb begin
        case (bsl)
            3'd1: begin part_addr = CDF_AW'(CDF_PARTITION_W8 + int'(part_ctx));  part_n = 4'd3; end
            3'd2: begin part_addr = CDF_AW'(CDF_PARTITION_W16 + int'(part_ctx)); part_n = 4'd9; end
            3'd3: begin part_addr = CDF_AW'(CDF_PARTITION_W32 + int'(part_ctx)); part_n = 4'd9; end
            3'd4: begin part_addr = CDF_AW'(CDF_PARTITION_W64 + int'(part_ctx)); part_n = 4'd9; end
            default: begin part_addr = CDF_AW'(CDF_PARTITION_W128 + int'(part_ctx)); part_n = 4'd7; end
        endcase
    end
    logic want_horz;                           // which bool we are decoding (1: split_or_horz, 0: split_or_vert)

    // loop-restoration unit syntax at superblock start
    logic lr_start, lr_busy, lr_fin, lr_active;
    logic lq_go; logic [CDF_AW-1:0] lq_addr; logic [3:0] lq_n; logic [1:0] lq_kind;
    lr_syntax u_lr (.clk, .rst, .hdr, .tile_start, .start(lr_start), .r(sb_r), .c(sb_c), .sb128(hdr.sb128), .busy(lr_busy), .done(lr_fin),
                    .lr_done, .lr_rec, .sq_go(lq_go), .sq_addr(lq_addr), .sq_n(lq_n), .sq_kind(lq_kind), .sq_done(k_done), .sq_sym(k_sym));

    // control sequencer mux: blk_syntax when a block is active, lr_syntax at superblock start, else the partition FSM
    logic pq_go; logic [CDF_AW-1:0] pq_addr; logic [3:0] pq_n; logic [1:0] pq_kind;
    always_comb begin
        k_go = blk_active ? bq_go : lr_active ? lq_go : pq_go;
        k_addr = blk_active ? bq_addr : lr_active ? lq_addr : pq_addr;
        k_n = blk_active ? bq_n : lr_active ? lq_n : pq_n;
        k_kind = blk_active ? bq_kind : lr_active ? lq_kind : pq_kind;
        k_f = want_horz ? psum_h : psum_v;
    end
    // partition requests are issued from the FSM via registered strobes
    logic p_go_sym, p_go_peek, p_go_bool;
    assign pq_go = p_go_sym | p_go_peek | p_go_bool;
    assign pq_kind = p_go_peek ? 2'd3 : p_go_bool ? 2'd1 : 2'd0;
    assign pq_addr = part_addr;
    assign pq_n = part_n;

    // child descriptor for (part, child index)
    logic child_is_blk, child_valid;
    logic [10:0] ch_r, ch_c;
    logic [4:0] ch_bs;
    always_comb begin
        child_valid = 1'b0; child_is_blk = 1'b1; ch_r = top.r; ch_c = top.c; ch_bs = sub_size;
        case (top.part)
            P_NONE: child_valid = (top.child == 0);
            P_HORZ: begin
                if (top.child == 0) child_valid = 1'b1;
                else if (top.child == 1 && top.has_rows) begin child_valid = 1'b1; ch_r = top.r + 11'(half4); end
            end
            P_VERT: begin
                if (top.child == 0) child_valid = 1'b1;
                else if (top.child == 1 && top.has_cols) begin child_valid = 1'b1; ch_c = top.c + 11'(half4); end
            end
            P_SPLIT: begin
                child_is_blk = 1'b0; child_valid = (top.child < 4);
                ch_r = top.r + (top.child[1] ? 11'(half4) : 11'd0);
                ch_c = top.c + (top.child[0] ? 11'(half4) : 11'd0);
            end
            P_HORZ_A: begin
                child_valid = (top.child < 3);
                case (top.child)
                    3'd0: begin ch_bs = split_size; end
                    3'd1: begin ch_bs = split_size; ch_c = top.c + 11'(half4); end
                    default: begin ch_r = top.r + 11'(half4); end
                endcase
            end
            P_HORZ_B: begin
                child_valid = (top.child < 3);
                case (top.child)
                    3'd0: ;
                    3'd1: begin ch_bs = split_size; ch_r = top.r + 11'(half4); end
                    default: begin ch_bs = split_size; ch_r = top.r + 11'(half4); ch_c = top.c + 11'(half4); end
                endcase
            end
            P_VERT_A: begin
                child_valid = (top.child < 3);
                case (top.child)
                    3'd0: begin ch_bs = split_size; end
                    3'd1: begin ch_bs = split_size; ch_r = top.r + 11'(half4); end
                    default: begin ch_c = top.c + 11'(half4); end
                endcase
            end
            P_VERT_B: begin
                child_valid = (top.child < 3);
                case (top.child)
                    3'd0: ;
                    3'd1: begin ch_bs = split_size; ch_c = top.c + 11'(half4); end
                    default: begin ch_bs = split_size; ch_r = top.r + 11'(half4); ch_c = top.c + 11'(half4); end
                endcase
            end
            P_HORZ_4: begin
                ch_r = top.r + 11'(quarter4) * 11'(top.child);
                child_valid = (top.child < 3) || (top.child == 3 && (top.r + 11'(quarter4) * 11'd3) < hdr.mi_rows);
            end
            default: begin   // P_VERT_4
                ch_c = top.c + 11'(quarter4) * 11'(top.child);
                child_valid = (top.child < 3) || (top.child == 3 && (top.c + 11'(quarter4) * 11'd3) < hdr.mi_cols);
            end
        endcase
    end

    assign tile_busy = (st != T_IDLE);
    assign unsupported = b_unsup;
    assign sb_start_o = sb_start; assign sb_r_o = sb_r; assign sb_c_o = sb_c;
    assign p_r = top.r; assign p_c = top.c; assign p_bs = top.bs;

    always_ff @(posedge clk) begin
        tile_done <= 1'b0; msac_init <= 1'b0; cdf_init <= 1'b0; clear_above <= 1'b0; clear_left <= 1'b0; sbrow_end <= 1'b0; mvm_clr <= 1'b0;
        sb_start <= 1'b0; nb_req_p <= 1'b0; p_go_sym <= 1'b0; p_go_peek <= 1'b0; p_go_bool <= 1'b0; b_start <= 1'b0; lr_start <= 1'b0;
        if (rst) begin
            st <= T_IDLE; sp <= 4'd0; blk_active <= 1'b0; lr_active <= 1'b0;
        end else case (st)
            T_IDLE: if (tile_start) begin
                cdf_init <= 1'b1; msac_init <= 1'b1; clear_above <= 1'b1; clear_left <= 1'b1;
                sb_r <= hdr.mi_row_start; sb_c <= hdr.mi_col_start;
                st <= T_INIT;
            end
            T_INIT: if (!cdf_busy && !cdf_init) st <= T_SB;
            T_SB: begin
                // superblock start: ReadDeltas, cdef flags (blk_syntax), read_lr, then decode_partition
                sb_start <= 1'b1;
                if (sb_c == hdr.mi_col_start) mvm_clr <= 1'b1;          // new superblock row: its motion-info rows are unwritten
                sp <= 4'd1;
                stack[0].r <= sb_r; stack[0].c <= sb_c; stack[0].bs <= sb_size_bsize(hdr.sb128); stack[0].child <= 3'd0;
                lr_start <= 1'b1; lr_active <= 1'b1;
                st <= T_LR_W;
            end
            T_LR_W: if (lr_fin) begin lr_active <= 1'b0; st <= T_NODE; end
            // ---- decode_partition(top)
            T_NODE: begin
                if (top.r >= hdr.mi_rows || top.c >= hdr.mi_cols) begin
                    // nothing here: pop, parent advances to its next child
                    sp <= sp - 4'd1;
                    if (sp != 4'd1) stack[spp].child <= stack[spp].child + 3'd1;
                    st <= (sp == 4'd1) ? T_SB_ROW_END : T_CHILD;
                end else begin
                    stack[spi].has_rows <= (top.r + 11'(half4)) < hdr.mi_rows;
                    stack[spi].has_cols <= (top.c + 11'(half4)) < hdr.mi_cols;
                    stack[spi].child <= 3'd0;
                    if (top.bs < BLOCK_8X8) begin stack[spi].part <= P_NONE; st <= T_CHILD; end
                    else begin nb_req_p <= 1'b1; st <= T_NB_W; end
                end
            end
            T_NB_W: if (nb_valid) begin
                if (top.has_rows && top.has_cols) begin p_go_sym <= 1'b1; st <= T_PART_W; end
                else if (top.has_cols) begin want_horz <= 1'b1; p_go_peek <= 1'b1; st <= T_PEEK_W; end
                else if (top.has_rows) begin want_horz <= 1'b0; p_go_peek <= 1'b1; st <= T_PEEK_W; end
                else begin stack[spi].part <= P_SPLIT; st <= T_CHILD; end
            end
            T_PART_W: if (k_done) begin stack[spi].part <= k_sym; st <= T_CHILD; end
            T_PEEK_W: if (k_done) begin p_go_bool <= 1'b1; st <= T_BOOL_W; end    // k_row holds the row; k_f = psum
            T_BOOL_W: if (k_done) begin
                stack[spi].part <= k_sym[0] ? P_SPLIT : (want_horz ? P_HORZ : P_VERT);
                st <= T_CHILD;
            end
            // ---- next child of top
            T_CHILD: begin
                if (!child_valid) begin
                    // node finished: pop; parent advances
                    if (sp == 4'd1) st <= T_SB_ROW_END;
                    else begin
                        sp <= sp - 4'd1;
                        stack[spp].child <= stack[spp].child + 3'd1;
                        st <= T_CHILD;
                    end
                end else if (child_is_blk) begin
                    b_r <= ch_r; b_c <= ch_c; b_bs <= ch_bs;
                    blk_active <= 1'b1; b_start <= 1'b1;
                    stack[spi].child <= top.child + 3'd1;
                    st <= T_BLK_W;
                end else begin
                    // push sub-partition (parent's child index advances when the child pops)
                    stack[sp[2:0]].r <= ch_r; stack[sp[2:0]].c <= ch_c; stack[sp[2:0]].bs <= ch_bs; stack[sp[2:0]].child <= 3'd0;
                    sp <= sp + 4'd1;
                    st <= T_NODE;
                end
            end
            T_BLK_W: if (blk_done) begin blk_active <= 1'b0; st <= T_CHILD; end
            // ---- next superblock
            T_SB_ROW_END: begin
                if (sb_c + 11'(sb4) < hdr.mi_col_end) begin sb_c <= sb_c + 11'(sb4); st <= T_SB; end
                else begin
                    sbrow_end <= 1'b1;
                    if (sb_r + 11'(sb4) < hdr.mi_row_end) begin
                        sb_r <= sb_r + 11'(sb4); sb_c <= hdr.mi_col_start; clear_left <= 1'b1; st <= T_SB;
                    end else st <= T_DONE;
                end
            end
            T_DONE: if (!ctx_wbusy) begin tile_done <= 1'b1; st <= T_IDLE; end
            default: st <= T_IDLE;
        endcase
    end
endmodule
