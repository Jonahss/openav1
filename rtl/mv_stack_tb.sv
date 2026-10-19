// Testbench top for mv_stack: the stack with its two memories (mv_mem for the frame's motion info, mf_band for the
// motion field), both written by the testbench; the stack's array outputs flattened (tb/test_mv_stack.py).
module mv_stack_tb
  import syn_pkg::*;
#(
    parameter int ML2R = 6,
    parameter int ML2C = 7,
    parameter int MFL2R = 5,
    parameter int MFL2C = 6
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  ihdr_t       ihdr,
    // mv_mem writes
    input  logic        mvm_we,
    input  logic [MIW-1:0] mvm_r, mvm_c,
    input  logic [5:0]  mvm_bw4, mvm_bh4,
    input  mv_ent_t     mvm_data,
    output logic        mvm_busy,
    input  logic        mvm_clr,
    input  logic [MIW-1:0] clr_row,
    input  logic [5:0]  clr_n,
    // motion field writes
    input  logic        mf_we,
    input  logic [2:0]  mf_wref,
    input  logic [MIW-2:0] mf_wy8, mf_wx8,
    input  logic        mf_wvalid,
    input  logic signed [15:0] mf_wrow, mf_wcol,
    // the stack
    input  logic        start,
    input  logic [MIW-1:0] r, c,
    input  logic [4:0]  bs,
    input  logic [3:0]  ref0, ref1,
    input  logic        is_comp,
    output logic        busy, done,
    output logic signed [17:0] pred_row, pred_col,
    output logic [3:0]  n_found,
    output logic [71:0] st0_row_f, st0_col_f, st1_row_f, st1_col_f,     // entry k at [18k +: 18]
    output logic [35:0] gmv_row_f, gmv_col_f,
    output logic [2:0]  new_mv_ctx, ref_mv_ctx,
    output logic        zero_mv_ctx,
    output logic [5:0]  drl_ctx_f
);
    logic [MIW-1:0] rd_row, rd_col; mv_ent_t rd_ent; logic rd_written;
    logic [2:0] mf_ref; logic [MIW-2:0] mf_y8, mf_x8; logic mf_valid; logic signed [15:0] mf_row, mf_col;
    mv_mem #(.ML2R(ML2R), .ML2C(ML2C)) u_mvm (.clk, .rst, .we(mvm_we), .w_r(mvm_r), .w_c(mvm_c), .w_bw4(mvm_bw4), .w_bh4(mvm_bh4),
                                              .w_data(mvm_data), .busy(mvm_busy), .clr(mvm_clr), .clr_row, .clr_n,
                                              .rd_row, .rd_col, .rd_ent, .rd_written);
    mf_band #(.MFL2R(MFL2R), .MFL2C(MFL2C)) u_mf (.clk, .we(mf_we), .w_ref(mf_wref), .w_y8(mf_wy8), .w_x8(mf_wx8), .w_valid(mf_wvalid),
                                                  .w_row(mf_wrow), .w_col(mf_wcol), .rd_ref(mf_ref), .rd_y8(mf_y8), .rd_x8(mf_x8),
                                                  .rd_valid(mf_valid), .rd_row(mf_row), .rd_col(mf_col));
    logic signed [17:0] st0_row [4], st0_col [4], st1_row [4], st1_col [4], gmv_row [2], gmv_col [2];
    logic [1:0] drl_ctx [3];
    mv_stack u_mvs (.clk, .rst, .hdr, .ihdr, .start, .r, .c, .bs, .ref0, .ref1, .is_comp, .busy, .done, .pred_row, .pred_col,
                    .n_found, .st0_row, .st0_col, .st1_row, .st1_col, .gmv_row, .gmv_col, .new_mv_ctx, .ref_mv_ctx, .zero_mv_ctx, .drl_ctx,
                    .rd_row, .rd_col, .rd_ent, .rd_written, .mf_ref, .mf_y8, .mf_x8, .mf_valid, .mf_row, .mf_col);
    always_comb begin
        for (int k = 0; k < 4; k++) begin
            st0_row_f[18*k +: 18] = st0_row[k]; st0_col_f[18*k +: 18] = st0_col[k];
            st1_row_f[18*k +: 18] = st1_row[k]; st1_col_f[18*k +: 18] = st1_col[k];
        end
        for (int k = 0; k < 2; k++) begin gmv_row_f[18*k +: 18] = gmv_row[k]; gmv_col_f[18*k +: 18] = gmv_col[k]; end
        for (int k = 0; k < 3; k++) drl_ctx_f[2*k +: 2] = drl_ctx[k];
    end
endmodule
