// openav1 — AV1 arithmetic (symbol) decoder core.
//
// Implements the spec's symbol decoder (AV1 spec section 8.2: init_symbol,
// read_symbol, read_bool via read_symbol, CDF adaptation). One decoded symbol per
// accepted request, result registered the following cycle.
//
// Conventions (chosen to match dav1d, which is spec-equivalent and is our golden trace):
//   * CDFs are stored INVERTED: icdf[i] = 32768 - cdf[i].  The spec itself notes that
//     implementations may prefer this; it removes a subtraction from the hot loop.
//   * req_n is N-1 (the number of symbols minus one), 1..15.
//   * The bit window holds the stream bits INVERTED, so the spec's
//     SymbolValue = (2^15-1) ^ data  is simply the raw window contents, and the
//     zero padding past the end of the tile appears as ones.
//
// CDF storage lives OUTSIDE this module: the requester supplies the CDF row with the
// request and receives the adapted row back with the response. This keeps the core
// small and lets the same block serve an FPGA (CDFs in BRAM) or a tiny chip (CDFs fed
// from off-chip).
module msac (
    input  logic         clk,
    input  logic         rst,

    // ---- tile byte stream (inverted internally) ----
    input  logic [7:0]   in_data,
    input  logic         in_valid,
    output logic         in_ready,
    input  logic         in_eos,       // hold high once the tile's bytes are exhausted

    // ---- control ----
    input  logic         init,         // pulse: start decoding a new tile
    input  logic         cdf_update_en,// !disable_cdf_update

    // ---- symbol request ----
    input  logic         req_valid,
    output logic         req_ready,
    input  logic [1:0]   req_kind,     // 0 = adaptive symbol, 1 = bool with fixed prob (icdf[0]), 2 = equiprobable bool
    input  logic [3:0]   req_n,        // N-1
    input  logic [239:0] req_cdf,      // 15 x 16-bit inverted CDF entries, entry k at [16k +: 16]
    input  logic [5:0]   req_cnt,      // adaptation counter (0..32)

    // ---- response, one cycle after the request is accepted ----
    output logic         resp_valid,
    output logic [3:0]   resp_sym,
    output logic [15:0]  resp_rng,     // SymbolRange after renormalisation (matches dav1d's s->rng)
    output logic [239:0] resp_cdf,
    output logic [5:0]   resp_cnt
);
    localparam int EC_PROB_SHIFT = 6;
    localparam int EC_MIN_PROB   = 4;

    typedef enum logic [1:0] { S_IDLE, S_INIT, S_READY } state_t;
    state_t state;

    // Decoder state
    logic [15:0] rng;          // SymbolRange, 32768..65535 between symbols
    logic [15:0] val;          // SymbolValue (inverted-data domain), always < rng
    logic [31:0] win;          // inverted stream bits, MSB first; bits beyond wcnt are 1 (padding)
    logic [5:0]  wcnt;         // number of real (unconsumed) stream bits in win
    logic        eos;          // sticky: no more bytes for this tile

    // ------------------------------------------------------------------
    // Byte refill (happens before consumption within the same cycle)
    // ------------------------------------------------------------------
    assign in_ready = (state != S_IDLE) && !eos && (wcnt <= 6'd24);

    logic [31:0] win_r;        // window after refill
    logic [5:0]  wcnt_r;
    always_comb begin
        win_r  = win;
        wcnt_r = wcnt;
        if (in_valid && in_ready) begin
            win_r[31 - wcnt -: 8] = ~in_data;
            wcnt_r = wcnt + 6'd8;
        end
    end

    // ------------------------------------------------------------------
    // Request decode (combinational on registered state + request inputs)
    // ------------------------------------------------------------------
    logic [3:0]  n_eff;                // N-1 actually used
    logic [15:0] icdf [15];
    logic [5:0]  cnt_eff;
    logic        do_update;

    always_comb begin
        for (int k = 0; k < 15; k++) icdf[k] = req_cdf[16*k +: 16];
        n_eff     = req_n;
        cnt_eff   = req_cnt;
        do_update = cdf_update_en;
        case (req_kind)
            2'd1: begin n_eff = 4'd1; do_update = 1'b0; end                   // fixed-probability bool, f = icdf[0]
            2'd2: begin n_eff = 4'd1; do_update = 1'b0; icdf[0] = 16'd16384; end // equiprobable bool
            default: ;
        endcase
    end

    // cur[k] for every candidate symbol, spec:
    //   cur = ((SymbolRange >> 8) * (f >> EC_PROB_SHIFT)) >> (7 - EC_PROB_SHIFT) + EC_MIN_PROB * (N - symbol - 1)
    // with f = icdf[k] (already inverted).  For k >= n_eff, cur = 0 so the search always terminates.
    logic [7:0]  r8;
    logic [15:0] cur  [16];
    logic [16:0] prod [16];
    int          n_int;
    always_comb begin
        r8    = rng[15:8];
        n_int = {28'd0, n_eff};
        for (int k = 0; k < 16; k++) begin
            prod[k] = (k < 15) ? 17'(r8) * 17'(icdf[k < 15 ? k : 0] >> EC_PROB_SHIFT) : 17'd0;
            cur[k]  = (k < n_int) ? 16'(prod[k] >> (7 - EC_PROB_SHIFT)) + 16'(EC_MIN_PROB * (n_int - k))
                                  : 16'd0;
        end
    end

    // symbol = first k with SymbolValue >= cur[k]  (spec loop: continue while SymbolValue < cur)
    logic [3:0]  sym;
    logic [15:0] cur_sym, cur_prev;
    always_comb begin
        sym = 4'd15;
        for (int k = 15; k >= 0; k--) if (!(val < cur[k])) sym = 4'(k);
        cur_sym  = cur[sym];
        cur_prev = (sym == 4'd0) ? rng : cur[sym - 4'd1];
    end

    // Renormalise: bits = 15 - FloorLog2(newRange) = leading zeros of the 16-bit range.
    logic [15:0] rng_new, val_new;
    logic [4:0]  bits;
    always_comb begin
        rng_new = cur_prev - cur_sym;
        val_new = val - cur_sym;
        bits = 5'd0;
        for (int b = 0; b < 16; b++) if (rng_new[b]) bits = 5'(15 - b);   // last assignment = highest set bit
    end

    // Window bookkeeping after consuming bits: consumed positions refill with 1s (= zero padding, inverted).
    // Two consumers: init_symbol takes exactly 15 bits, read_symbol takes `bits`.
    logic [31:0] win_after15, win_afterb;
    logic [5:0]  wcnt_after15, wcnt_afterb;
    logic [15:0] newbits;                       // the `bits` stream bits appended to SymbolValue
    always_comb begin
        win_after15  = (win_r << 15) | 32'h0000_7FFF;
        wcnt_after15 = (wcnt_r < 6'd15) ? 6'd0 : wcnt_r - 6'd15;
        win_afterb   = (win_r << bits) | ~(32'hFFFF_FFFF << bits);
        wcnt_afterb  = (wcnt_r < 6'(bits)) ? 6'd0 : wcnt_r - 6'(bits);
        newbits      = (bits == 5'd0) ? 16'd0 : 16'(win_r >> (6'd32 - 6'(bits)));
    end

    // CDF adaptation (spec 8.2.6, in inverted form):
    //   rate = 3 + (cnt > 15) + (cnt > 31) + Min(FloorLog2(N), 2)  ==  4 + (cnt >> 4) + (N-1 > 2)
    logic [3:0]  rate;
    logic [15:0] icdf_upd [15];
    logic [5:0]  cnt_upd;
    always_comb begin
        rate = 4'd4 + 4'(cnt_eff >> 4) + 4'(n_eff > 4'd2);
        for (int i = 0; i < 15; i++) begin
            if (!do_update || i >= n_int)      icdf_upd[i] = icdf[i];
            else if (i < {28'd0, sym})                  icdf_upd[i] = icdf[i] + ((16'd32768 - icdf[i]) >> rate);
            else                                     icdf_upd[i] = icdf[i] - (icdf[i] >> rate);
        end
        cnt_upd = do_update ? (cnt_eff + 6'((cnt_eff < 6'd32) ? 1 : 0)) : cnt_eff;
    end

    // ------------------------------------------------------------------
    // Handshake
    // ------------------------------------------------------------------
    // A symbol may consume up to 15 bits; require them present unless the tile is exhausted.
    assign req_ready = (state == S_READY) && (wcnt >= 6'd15 || eos);
    wire   accept    = req_valid && req_ready;
    wire   init_go   = (state == S_INIT) && (wcnt_r >= 6'd15 || eos || in_eos);

    // ------------------------------------------------------------------
    // State update
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        resp_valid <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
            eos   <= 1'b0;
            wcnt  <= 6'd0;
            win   <= 32'hFFFF_FFFF;
            rng   <= 16'h8000;
            val   <= 16'd0;
        end else if (init) begin
            state <= S_INIT;
            eos   <= 1'b0;
            wcnt  <= 6'd0;
            win   <= 32'hFFFF_FFFF;
            rng   <= 16'h8000;
            val   <= 16'd0;
        end else begin
            if (in_eos && state != S_IDLE) eos <= 1'b1;
            win  <= win_r;
            wcnt <= wcnt_r;
            case (state)
                S_INIT: if (init_go) begin
                    // init_symbol: SymbolValue = (2^15-1) ^ first 15 bits  ==  top 15 inverted bits
                    val   <= {1'b0, win_r[31:17]};
                    win   <= win_after15;
                    wcnt  <= wcnt_after15;
                    state <= S_READY;
                end
                S_READY: if (accept) begin
                    rng   <= rng_new << bits;
                    val   <= (val_new << bits) | newbits;
                    win   <= win_afterb;
                    wcnt  <= wcnt_afterb;
                    resp_valid <= 1'b1;
                    resp_sym   <= sym;
                    resp_rng   <= rng_new << bits;
                    resp_cnt   <= cnt_upd;
                    for (int i = 0; i < 15; i++) resp_cdf[16*i +: 16] <= icdf_upd[i];
                end
                default: ;
            endcase
        end
    end
endmodule
