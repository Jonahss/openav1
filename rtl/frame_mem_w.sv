// Write-port adapter: a stage's single-sample write (we1 at x1) or 4-wide write (we4 at x4, aligned) -> one
// frame_mem write port. The two never fire in the same cycle (the caller's FSM is in one state at a time).
module frame_mem_w #(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int PW = 12
) (
    input  logic          we1,
    input  logic [1:0]    plane1,
    input  logic [FBX-1:0] x1,
    input  logic [FBY-1:0] y1,
    input  logic [PW-1:0] d1,
    input  logic          we4,
    input  logic [1:0]    plane4,
    input  logic [FBX-1:0] x4,
    input  logic [FBY-1:0] y4,
    input  logic [4*PW-1:0] d4,
    output logic          we,
    output logic [3:0]    be,
    output logic [1:0]    plane,
    output logic [FBX-1:0] x,
    output logic [FBY-1:0] y,
    output logic [4*PW-1:0] data
);
    always_comb begin
        we = we1 || we4;
        be = we4 ? 4'hF : (4'b0001 << x1[1:0]);
        plane = we4 ? plane4 : plane1;
        x = we4 ? x4 : x1;
        y = we4 ? y4 : y1;
        data = we4 ? d4 : {4{d1}};
    end
endmodule
