# Optional placement (U200_XDC=fpga/u200/lr_pblock.xdc in build.sh): keep the loop-restoration block and the fb0 mirror
# (the frame-memory copy LR reads) in one SLR. The SB128 build (u200-fm2s100-2018ffe, 2026-11-01) spread its 539 UltraRAMs
# over all three SLRs and missed by 0.294 ns on 697 endpoints, every one of them the same family: fb0-mirror URAM DOUT
# (clk2x, multicycle 2) -> the 32:1 select mux -> u_lr line-memory DIN, 8.5 ns of which 7.5 ns was routing across
# SLR2 -> SLR1 -> SLR0 (the URAM in SLR2, the mux in SLR1, the line BRAM in SLR0), plus 0.9 ns of clock skew between
# the trees and 0.4 ns of inter-SLR compensation. SLR2 has the room (84k of 394k LUTs, 240 of 320 URAMs used).
create_pblock pb_lr
add_cells_to_pblock [get_pblocks pb_lr] [get_cells -quiet {sys_i/host/inst/u/u_dec/u_lr sys_i/host/inst/u/u_dec/g_fb0_uram.g_mirror.u_fbm}]
resize_pblock [get_pblocks pb_lr] -add {SLR2}
