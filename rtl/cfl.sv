// openav1 — chroma-from-luma prediction (AV1 spec 7.11.5), one chroma pixel per clock per pass.
//
// Load the reconstructed luma region covering the chroma block (up to 32x32 luma samples) and the
// DC-predicted chroma block, set the parameters and pulse start. Pass 1 builds the subsampled luma
// array L (3 fractional bits) and its sum; pass 2 emits Clip1(dc + Round2Signed(alpha * (L - avg), 6)).
//
// luma_avail_w / luma_avail_h: number of valid luma columns / rows in the buffer (MaxLumaW - lumaX0 etc.),
// used for the spec's Min(lumaX, MaxLumaW - (1 << subX)) clamp at the frame edge.
module cfl #(
    parameter int PW = 12
) (
    input  logic          clk,
    input  logic          rst,

    input  logic          luma_we,          // luma[y][x] at addr = y*32 + x
    input  logic [9:0]    luma_addr,
    input  logic [PW-1:0] luma_data,
    input  logic          dc_we,            // dc[i][j] at addr = i*32 + j
    input  logic [9:0]    dc_addr,
    input  logic [PW-1:0] dc_data,

    input  logic          start,
    input  logic [2:0]    log2w,            // chroma block, 2..5
    input  logic [2:0]    log2h,
    input  logic          sub_x,
    input  logic          sub_y,
    input  logic signed [5:0] alpha,        // CflAlpha: -16..16
    input  logic [3:0]    bit_depth,
    input  logic [6:0]    luma_avail_w,     // 1..64
    input  logic [6:0]    luma_avail_h,
    output logic          busy,
    output logic          done,

    output logic          out_valid,
    output logic [4:0]    out_x,
    output logic [4:0]    out_y,
    output logic [PW-1:0] out_pix
);
    logic [PW-1:0]   luma [1024];
    logic [PW-1:0]   dc   [1024];
    logic [PW+2:0]   Lbuf [1024];           // L values: up to 4 samples summed then << up to 3 -> PW+3 bits

    typedef enum logic [1:0] { S_IDLE, S_L, S_OUT, S_DONE } state_t;
    state_t state;

    logic [2:0]  l_log2w, l_log2h;
    logic [5:0]  w, h;
    logic        l_sx, l_sy;
    logic signed [5:0] l_alpha;
    logic [3:0]  l_bd;
    logic [6:0]  l_aw, l_ah;
    logic [4:0]  px, py;
    logic [PW+13:0] lsum;                    // up to 1024 * 2^(PW+3)
    logic [PW+2:0]  avg;
    logic [3:0]     lsz;                     // log2w + log2h (4 bits: up to 10)
    assign lsz = 4'(l_log2w) + 4'(l_log2h);

    // pass 1 datapath: L for (py, px)
    logic [PW+2:0] l_val;
    always_comb begin
        int lx, ly, t, cx, cy, mxx, mxy;
        t = 0; cx = 0; cy = 0; l_val = '0;
        lx = int'(px) << l_sx; ly = int'(py) << l_sy;
        mxx = int'(l_aw) - (1 << l_sx); mxy = int'(l_ah) - (1 << l_sy);
        if (lx > mxx) lx = mxx;
        if (ly > mxy) ly = mxy;
        for (int dy = 0; dy < 2; dy++)
            for (int dx = 0; dx < 2; dx++)
                if (dy <= int'(l_sy) && dx <= int'(l_sx)) begin
                    cx = lx + dx; cy = ly + dy;
                    t += int'(luma[(cy & 31) * 32 + (cx & 31)]);
                end
        l_val = (PW+3)'(t << (3 - int'(l_sx) - int'(l_sy)));
    end

    // pass 2 datapath
    logic [PW-1:0] o_val;
    always_comb begin
        int d, sc, r;
        d = int'(Lbuf[int'(py) * 32 + int'(px)]) - int'(avg);
        sc = int'(l_alpha) * d;
        if (sc >= 0) r = (sc + 32) >> 6; else r = -((-sc + 32) >> 6);      // Round2Signed(sc, 6)
        r = int'(dc[int'(py) * 32 + int'(px)]) + r;
        if (r < 0) r = 0;
        if (r > (1 << int'(l_bd)) - 1) r = (1 << int'(l_bd)) - 1;
        o_val = PW'(r);
    end

    always_ff @(posedge clk) begin
        done <= 1'b0;
        out_valid <= 1'b0;
        if (rst) begin
            state <= S_IDLE; busy <= 1'b0;
        end else begin
            case (state)
            S_IDLE: begin
                if (luma_we) luma[luma_addr] <= luma_data;
                if (dc_we)   dc[dc_addr]     <= dc_data;
                if (start) begin
                    l_log2w <= log2w; l_log2h <= log2h;
                    w <= 6'd1 << log2w; h <= 6'd1 << log2h;
                    l_sx <= sub_x; l_sy <= sub_y; l_alpha <= alpha; l_bd <= bit_depth;
                    l_aw <= luma_avail_w; l_ah <= luma_avail_h;
                    px <= 0; py <= 0; lsum <= '0;
                    busy <= 1'b1; state <= S_L;
                end
            end
            S_L: begin
                Lbuf[int'(py) * 32 + int'(px)] <= l_val;
                lsum <= lsum + (PW+14)'(l_val);
                if (px + 6'd1 == w) begin
                    px <= 0;
                    if (py + 6'd1 == h) begin
                        // avg = Round2(sum + this L, log2w + log2h)
                        avg <= (PW+3)'((lsum + (PW+14)'(l_val) + ((PW+14)'(1) << (lsz - 4'd1))) >> lsz);
                        py <= 0; state <= S_OUT;
                    end else py <= py + 5'd1;
                end else px <= px + 5'd1;
            end
            S_OUT: begin
                out_valid <= 1'b1; out_x <= px; out_y <= py; out_pix <= o_val;
                if (px + 6'd1 == w) begin
                    px <= 0;
                    if (py + 6'd1 == h) state <= S_DONE;
                    else py <= py + 5'd1;
                end else px <= px + 5'd1;
            end
            S_DONE: begin
                done <= 1'b1; busy <= 1'b0; state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
