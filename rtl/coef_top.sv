// Test/integration top for the coefficient reader: msac + cdf_store + coef_rd, plus a second symbol
// sequencer the testbench (later: the block-level FSM) uses for the symbols that sit between phase A and
// phase B of coeffs( ) -- intra_tx_type -- and for anything else at block level. Only one master is active
// at a time; the mux selects whichever sequencer is busy (or the one being started).
module coef_top
  import cdf_map_pkg::*;
(
    input  logic              clk,
    input  logic              rst,
    // tile byte stream -> msac
    input  logic [7:0]        in_data,
    input  logic              in_valid,
    output logic              in_ready,
    input  logic              in_eos,
    input  logic              init,            // msac init (new tile)
    input  logic              cdf_update_en,
    // cdf store defaults + tile start copy
    input  logic              def_we,
    input  logic [CDF_AW-1:0] def_addr,
    input  logic [245:0]      def_data,
    input  logic              cdf_init,
    output logic              cdf_busy,
    // coef_rd
    input  logic [4:0]        cfg_tx,
    input  logic              cfg_ptype,
    input  logic              start_a,
    input  logic [3:0]        az_ctx,
    output logic              done_a,
    output logic              all_zero,
    input  logic              start_b,
    input  logic [3:0]        tx_type,
    input  logic [1:0]        dcs_ctx,
    output logic              done_b,
    output logic [10:0]       eob_o,
    output logic [5:0]        cul_level,
    output logic [1:0]        dc_category,
    output logic              nonconformant,
    input  logic [9:0]        q_addr,
    output logic signed [20:0] q_data,
    // block-level symbol port
    input  logic              tb_go,
    input  logic [CDF_AW-1:0] tb_addr,
    input  logic [3:0]        tb_n,
    input  logic [1:0]        tb_kind,
    output logic              tb_done,
    output logic [3:0]        tb_sym
);
    // msac
    logic req_valid, req_ready, resp_valid;
    logic [1:0] req_kind; logic [3:0] req_n; logic [239:0] req_cdf; logic [5:0] req_cnt;
    logic [3:0] resp_sym; logic [15:0] resp_rng; logic [239:0] resp_cdf; logic [5:0] resp_cnt;
    msac u_msac (.clk, .rst, .in_data, .in_valid, .in_ready, .in_eos, .init, .cdf_update_en,
                 .req_valid, .req_ready, .req_kind, .req_n, .req_cdf, .req_cnt,
                 .resp_valid, .resp_sym, .resp_rng, .resp_cdf, .resp_cnt);

    // cdf store
    logic rd_en, wb_we; logic [CDF_AW-1:0] rd_addr, wb_addr; logic [245:0] rd_data, wb_data;
    cdf_store u_cdf (.clk, .rst, .def_we, .def_addr, .def_data, .init(cdf_init), .busy(cdf_busy),
                     .rd_en, .rd_addr, .rd_data, .wb_we, .wb_addr, .wb_data);

    // two sequencers
    logic c_go, c_done, t_busy, c_busy; logic [CDF_AW-1:0] c_addr; logic [3:0] c_n, c_sym; logic [1:0] c_kind;
    logic c_rd_en, t_rd_en, c_wb_we, t_wb_we, c_req_valid, t_req_valid;
    logic [CDF_AW-1:0] c_rd_addr, t_rd_addr, c_wb_addr, t_wb_addr;
    logic [245:0] c_wb_data, t_wb_data;
    logic [1:0] c_req_kind, t_req_kind; logic [3:0] c_req_n, t_req_n; logic [239:0] c_req_cdf, t_req_cdf; logic [5:0] c_req_cnt, t_req_cnt;
    logic sel_t;                       // 1: testbench sequencer owns msac/cdf
    assign sel_t = t_busy || tb_go;   // the block level never starts a symbol while coef_rd is active

    sym_seq u_sq_c (.clk, .rst, .go(c_go), .addr(c_addr), .n(c_n), .kind(c_kind), .busy(c_busy), .done(c_done), .sym(c_sym),
                    .cdf_rd_en(c_rd_en), .cdf_rd_addr(c_rd_addr), .cdf_rd_data(rd_data),
                    .cdf_wb_we(c_wb_we), .cdf_wb_addr(c_wb_addr), .cdf_wb_data(c_wb_data),
                    .req_valid(c_req_valid), .req_ready(req_ready && !sel_t), .req_kind(c_req_kind), .req_n(c_req_n), .req_cdf(c_req_cdf), .req_cnt(c_req_cnt),
                    .resp_valid(resp_valid && !sel_t), .resp_sym, .resp_cdf, .resp_cnt);
    sym_seq u_sq_t (.clk, .rst, .go(tb_go), .addr(tb_addr), .n(tb_n), .kind(tb_kind), .busy(t_busy), .done(tb_done), .sym(tb_sym),
                    .cdf_rd_en(t_rd_en), .cdf_rd_addr(t_rd_addr), .cdf_rd_data(rd_data),
                    .cdf_wb_we(t_wb_we), .cdf_wb_addr(t_wb_addr), .cdf_wb_data(t_wb_data),
                    .req_valid(t_req_valid), .req_ready(req_ready && sel_t), .req_kind(t_req_kind), .req_n(t_req_n), .req_cdf(t_req_cdf), .req_cnt(t_req_cnt),
                    .resp_valid(resp_valid && sel_t), .resp_sym, .resp_cdf, .resp_cnt);

    always_comb begin
        rd_en   = sel_t ? t_rd_en   : c_rd_en;
        rd_addr = sel_t ? t_rd_addr : c_rd_addr;
        wb_we   = sel_t ? t_wb_we   : c_wb_we;
        wb_addr = sel_t ? t_wb_addr : c_wb_addr;
        wb_data = sel_t ? t_wb_data : c_wb_data;
        req_valid = sel_t ? t_req_valid : c_req_valid;
        req_kind  = sel_t ? t_req_kind  : c_req_kind;
        req_n     = sel_t ? t_req_n     : c_req_n;
        req_cdf   = sel_t ? t_req_cdf   : c_req_cdf;
        req_cnt   = sel_t ? t_req_cnt   : c_req_cnt;
    end

    coef_rd u_coef (.clk, .rst, .cfg_tx, .cfg_ptype, .start_a, .az_ctx, .done_a, .all_zero,
                    .start_b, .tx_type, .dcs_ctx, .done_b, .eob_o, .cul_level, .dc_category, .nonconformant,
                    .q_addr, .q_data,
                    .sq_go(c_go), .sq_addr(c_addr), .sq_n(c_n), .sq_kind(c_kind), .sq_done(c_done), .sq_sym(c_sym));
endmodule
