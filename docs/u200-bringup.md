# Alveo U200 bring-up: the decoder behind PCIe

Status (2026-10-23): **flow works, first bitstream built, no card yet** (arrives the week of 2026-10-26 with the
P620's RAM). Build 1 (`u200a-4fd1001`, 100 MHz, 1080p window, intra core): synthesis 8 min, implementation 65 min
on atium, routed with no errors, 186.7k LUT in all (decoder 106.8k, 54 URAM, 286 DSP; XDMA, DDR4 controller and
smartconnects the rest), 198.5 BRAM; every board pin as the board file says, PCIe block `PCIE40E4_X1Y2`. Two
findings: the MMCM VCO came out at 1600.6 MHz (DRC AVAL-46: the controller's "100 MHz" is 100.04, since DDR4-2400
runs the 300 MHz input at 300.12), fixed by capping the VCO search at 1550; and the decoder clock missed setup by
0.224 ns (100 endpoints, the transform engine's `l_log2h -> DSP -> itx_row` path, 19-21 levels, 72 % routing),
hold met. Build 2 (100 MHz, `Performance_ExtraTimingOpt`, VCO 1400): the decoder closes (clk +0.151 ns,
clk2x +0.456) and the only misses are inside XDMA, 44 endpoints at -0.047 ns on the PCIe core's 500 MHz clock,
zero logic levels, 1.9 ns of routing from the 512-bit interface registers into the PCIE40E4: placement crowding
at the hard block, being tried with `U200_PBLOCK=1` (XDMA kept in the PCIe clock regions). Build 3 (80 MHz,
default strategy, VCO 800): **everything met** (WNS +0.031 ns, decoder clock +1.6 ns, pulse widths clean),
185.8k LUT, routed with no errors: **the day-one bitstream**, `~/openav1-synth/u200-u200c80-4fd1001/sys_wrapper.bit`
on atium, the same 80 MHz decoder clock the Kria runs at. 100 MHz and up is a follow-up once the host path is
proven on the card. Everything below the host wrapper is the KV260 design unchanged: the same
`dec_host` register file, the same host protocol (`tb/host_regs.py`), the same driver (`tools/kv260/decode.py`)
with one new backend. Day one on the card is: program, see the PCIe function, read the ID register, decode the
1080p clip bit-exact through the window build at 100 MHz. The inter decoder comes after that (below).

## Why the U200

The inter decoder is ~230k LUT (`FRAME_URAM = 0`, FBX 10 / FBY 9, 2 transform lanes; whole-decoder netlist replay
bit-exact, gate rounds 7-10) against the K26's 117k; the K26 only ever holds the intra build (`fw-t80c`, 110k LUT
at `INTER = 0` after 4fd1001). The XCU200 (VU9P class: 1.18M LUT, 960 UltraRAM, 6,840 DSP, three SLRs) holds the
inter decoder five times over, removes the 80 MHz ceiling (-2 speed grade, 150-200 MHz plausible) and brings
4 x 16 GB DDR4 for reference pictures. The card is the active (fan-cooled) variant; the board is a VU9P in the
`xcu200-fsgd2104-2-e` package. Vivado ML Standard covers it (UG973 2025.2: Alveo in the free edition); the Virtex
UltraScale+ / Alveo device support was added to atium's install on 2026-10-22.

## What is in the bitstream (`fpga/u200/`)

`build.sh` -> `build.tcl` make a block design `sys` for `xcu200-fsgd2104-2-e` with the official board part
`xilinx.com:au200:part0:1.3` (AMD's `au200_board_files_20200616.zip`, not in the Vivado board store; unpacked
at `~/vivado-dl/boards/au200` on atium, `U200_BOARD_REPO`). The board interfaces give every pin: the IP
constraints of XDMA, the reference-clock buffer and the DDR4 controller carry the LOCs, `build.tcl` prints them
at the end. They were cross-checked against corundum's `fpga_au200.xdc` (same card, open source): PCIe refclk
AM11/AM10 (MGTREFCLK0 of quad 226), PERST BD21, 300 MHz system clocks AY37/AY38 (bank 0) and AW20/AW19 (bank 1),
DDR4 C0 `adr[0]` AT36, C1 `ba[0]` AU24, LED0 BC21.

- **`xdma`** (`xilinx.com:ip:xdma:4.2`): PCIe Gen3 x16 endpoint + DMA, GTY quad 227 group (board preset), 512-bit
  AXI at 250 MHz (`axi_aclk`), one H2C and one C2H channel, an AXI-Lite master for BAR0 (1 MB), no interrupts
  (`usr_irq_req` tied low; the driver polls). Device ID `A710`, class 04:00 (multimedia video), so it is not
  mistaken for a stock XDMA board. `refbuf` (`util_ds_buf`, IBUFDSGTE) splits the 100 MHz reference into
  `sys_clk_gt` / `sys_clk`; `pcie_perstn` is the endpoint's `sys_rst_n`.
- **`ddr`** (`xilinx.com:ip:ddr4:2.2`): DDR4 bank `U200_DDR` (default 1: the bank whose pins and clock sit in the
  PCIe SLR, SLR1), board preset: Micron MTA18ASF2G72PZ-2G3 RDIMM, 72 bits with ECC, DDR4-2400 (3.75 ns, UI clock
  300 MHz), 512-bit AXI, 34-bit address (16 GB). Its 100 MHz `addn_ui_clkout1` is the decoder MMCM's input (the
  role `pl_clk0` plays on the Kria). The ECC controller insists on its AXI-Lite control port being connected: it
  is the second register target (0x1_0000, PG150 ECC status / fault injection).
- **`clkgen`** (`clkgen_v.v`, written by `build.tcl`): the KV260's hand-instantiated `MMCME4_ADV`, same divider
  search, same exactly-2:1 `clk` / `clk2x` from one VCO (100 MHz in -> VCO 1600 -> /16 and /8 at `U200_MHZ=100`).
  The clock wizard is still not used (it does not honour the ratio; `docs/kv260-bringup.md`).
- **`host`** (`dec_host` through the KV260's Verilog shell `fpga/kv260/dec_host_v.v`) and **`pg`** (`phase_gen`),
  in the `clk` domain. Default geometry = the KV260 1080p window build: FBX = FBY = 11, `FRAME_WIN = 3`,
  `FRAME_URAM = 1`, `INTER = 0`, CDEF / LR as null stubs (`U200_CORE=1`). Reset: `rst_dec` (`proc_sys_reset`) from
  XDMA's `axi_aresetn` and the MMCM lock, so a PCIe-side reset of the card resets the decoder.
- **`jtag`** (`jtag_axi`): a JTAG-to-AXI master on the register bus, so the Vivado hardware manager can read the
  ID register and drive a frame before the PCIe driver works (bring-up without the host path; also the way to
  look at `STATUS` when the host path is what is broken).
- **`sc_regs`** / **`sc_mem`** (`smartconnect`): registers = {XDMA AXI-Lite, JTAG} at `axi_aclk` ->
  {`dec_host` at `clk`, DDR4 ECC control at `ui_clk`}; memory = {XDMA 512-bit at `axi_aclk`, `dec_host`'s 64-bit
  `m_axi` (picture flush, and `ref_fetch`'s reads when `INTER = 1`) at `clk`} -> the DDR4 bank at `ui_clk`. The
  smartconnects do the width and clock conversions.

Address maps:

| who | what | where |
|---|---|---|
| host BAR0 (`/dev/xdma0_user`), JTAG master | `dec_host` registers (`rtl/dec_host.sv` header; same as 0xA000_0000 on the KV260) | 0x0000 .. 0x0FFF |
| host BAR0, JTAG master | DDR4 controller ECC registers (PG150) | 0x1_0000 .. 0x1_FFFF |
| host DMA (`/dev/xdma0_h2c_0`, `_c2h_0`), decoder `m_axi` | the 16 GB DDR4 bank | 0x0_0000_0000 .. 0x3_FFFF_FFFF |

Constraints: `fpga/u200/sys.xdc` (configuration bank settings per UG1289: `CFGBVS GND`, `CONFIG_VOLTAGE 1.8`,
compressed bitstream; the clk <-> clk2x multicycles as on the KV260) and the KV260's `pre_opt.tcl` as the
`opt_design` pre-hook (the URAM -> clk2x multicycle and the phase generator's alignment path; the cells have the
same names in this block design). Build output: `~/openav1-synth/u200-<tag>/sys_wrapper.bit` + `impl_*.rpt`.

```
ssh atium; source /opt/Xilinx/2025.2.1/Vivado/settings64.sh
cd ~/openav1-wt-u200 && U200_MHZ=100 fpga/u200/build.sh <tag>        # ~2-3 h on atium
```

## Host software

`tools/kv260/decode.py --xdma 0 --window stream.ivf --ref ref.yuv` drives the card: the register file through
the XDMA user BAR (`tools/kv260/xdma.py` `XdmaRegs`, and `libhostregs.so` through `HostLib(0, dev="/dev/xdma0_user")`:
`hl_open_dev` maps the device node the way `hl_open` maps `/dev/mem`), the picture DMA target in the card's DDR4
(`XdmaStage`: `--dma-base` is a card address, default 0x1000_0000; the host reads the finished picture with
`pread` on the C2H channel, loads reference pictures with `pwrite` on H2C). Every register access is one 32-bit
PCIe transaction (the one-store rule holds here too: `ctypes.c_uint32` on the mapping, never `struct.pack_into`).
The directory name is historical; the driver is the same for both boards.

Driver: AMD `dma_ip_drivers` (XDMA/linux-kernel), built on atium 2026-10-23 for kernel 5.15.0-198 and installed
(`/lib/modules/5.15.0-198-generic/xdma/xdma.ko`, version 2025.2.0; `modprobe xdma`). The tools (`reg_rw`,
`dma_to_device`, `dma_from_device`) are built in `~/dma_ip_drivers/XDMA/linux-kernel/tools`. The nodes come up
root-only; `chmod 666 /dev/xdma0_*` (or a udev rule) for the driver as `jonahss`.

Expected costs: an AXI-Lite write over PCIe is posted (~0.1-0.3 us from C), a read is a round trip (~1 us); the
tile bytes and the 1,269 CDF default rows go through the same register path as on the Kria, so the host-side cost
per frame is of the order of the KV260's with `libhostregs.so` (tens of ms), and the picture comes back at DMA
speed instead of 7 us a word. A stream-to-DDR path (the decoder fetching tile bytes itself) is the obvious next
host-side step once the register path is proven; it is not needed to decode.

## Day one checklist

1. Card in the P620: a PCIe x16 slot (Gen3 is what the card does; the P620's slots are Gen4), the 8-pin PCIe
   auxiliary lead (the card draws up to 225 W; the fan variant has its own fan), the slim micro-USB cable on the
   bracket for JTAG (a normal plug does not fit past the shroud). `lspci -d 10ee:` after power-up shows whatever is
   in the configuration flash (a used card carries the XRT shell or nothing); either way our bitstream goes in
   over JTAG first.
2. `hw_server -s tcp::3121 &` on atium, `fpga/u200/program.sh ~/openav1-synth/u200-<tag>/sys_wrapper.bit`:
   programs the FPGA over JTAG, removes the stale PCIe function, rescans. If the function does not appear (the
   root port never trained because the slot was empty of a design at power-up), reboot the host with the card
   programmed: the FPGA keeps its configuration across a warm reboot as long as power stays on.
3. `lspci -d 10ee: -vv`: expect device `a710`, `LnkSta: Speed 8GT/s, Width x16`, BAR0 1 MB. Then `modprobe xdma`,
   `ls /dev/xdma0_*`, and the first register read: `reg_rw /dev/xdma0_user 0 w` must print `0xa71d0001` (the ID),
   offset 4 the CFG word (FBX 11, FBY 11, lanes 2, URAM 1). Through JTAG instead: in the hardware manager,
   `create_hw_axi_txn rd [get_hw_axis hw_axi_1] -type read -address 0 -len 2; run_hw_axi rd`.
4. DDR4: `reg_rw /dev/xdma0_user 0x10000 w` and the ECC status words (PG150) say whether calibration passed
   (`init_calib_complete` is also an ILA-free check: the smartconnect holds any DMA to the bank until it does, so a
   hung `dma_to_device` means no calibration). Then `dma_to_device` / `dma_from_device` a megabyte at 0x1000_0000
   and compare.
5. The decoder: `tools/kv260/decode.py --xdma 0 --window streams/demo/testsrc2_1080p_20f_core.ivf --ref refout/...`
   (20 of 20 frames identical on the KV260 at 80 MHz; here 100 MHz). Then the 120-frame foreman clip, then the
   same register transcript the KV260 replays (`tools/kv260/play.py` needs an `--xdma` switch: todo when the card
   is here).
6. Programme the configuration flash later (`write_cfgmem` -> `.mcs`, QSPI x4 per the bitstream settings in
   `sys.xdc`) so the card comes up with the decoder at power-on and the PCIe function enumerates at boot.

## After the intra build: the inter decoder on the card

Decided and built 2026-10-24 (docs/picture-memory.md, step A): every frame buffer is an UltraRAM **ring of superblock
rows** (fb0 + its mirror fb0m for the filter stages, SR's fb0s, fb1, fb2: `FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3`),
the final picture is flushed row by row to DDR and the references are read back from those flushed pictures by
`ref_fetch`. The whole-picture-in-URAM and DDR-line-buffer routes that stood here are superseded: the rings are the
throughput-grade memory (the line-buffer version is step C, for silicon). Synthesis at 1080p geometry
(FBX = FBY = 11): 207.5k LUT, 255 URAM, 204 BRAM, 820 DSP for `dec_host` alone. The register map and host software
gained the inter protocol (`tb/host_regs.py`: `seg_map_seq`, `motion_field_seq`, `ref_seq`,
`HostDecoder.decode_frames_inter`; `tools/kv260/decode.py --inter`), which `tb/test_dec_host_inter.py` verified
against the model on the generated inter streams and the Argon streams test47 (multi-tile rows, super-res), test6641,
test648 and test8854 (super-res + CDEF + LR).

Inter day one on the card, after the intra checklist above:

1. `U200_CORE=0 U200_INTER=1 U200_WIN=5 U200_WIN1=3 U200_WIN2=3 U200_MHZ=100 U200_STRATEGY=Performance_ExtraTimingOpt
   U200_PBLOCK=1 fpga/u200/build.sh <tag>` (the inter build; the 100 MHz closure of the intra build is the hope, not
   a given: ~290k LUT with XDMA and the MIG).
2. `python3 tools/kv260/decode.py --xdma 0 --inter streams/inter/inter2a_8bit.ivf --ref refout/inter2a_8bit.yuv`: the
   model decodes alongside (seconds a frame: the sequence state comes from it, as in simulation), the pictures land
   in a pool of 8 frame buffers in the card's DDR (`--pool`), each frame's buffer becomes its reference slots'
   address. Then test47 and the Argon super-res streams, then `--show` for the panel... the U200 has no panel: the
   pictures come back over c2h for the comparison, the display stays the Kria's.
3. Real-time inter needs the hardware side of the frame state (motion-field projection, CDF and segmentation
   save / load engines) instead of the host writing the bands: the throughput phase's work, sized by the census.

## Lessons so far (flow only; the card is not here)

- The AMD Vivado board store (`xhub`) has no `au200`; the board files are a separate zip on the U200 download page
  (`amdOpenDownload?filename=au200_board_files_20200616.zip`, no login). `vcu1525` is in the store with the same
  DIMM preset but a different part (`xcvu9p-fsgd2104-2L-e`): a cross-reference, not a substitute.
- An ECC-enabled DDR4 controller refuses `validate_bd_design` until `C0_DDR4_S_AXI_CTRL` is connected
  (`[BD 41-1380]`); a `proc_sys_reset`'s `C_EXT_RESET_HIGH` is read-only in a block design (inferred from the
  connected reset's polarity).
- `fpga/kv260/dec_host_v.v` had no `INTER` parameter while `build.tcl` set `CONFIG.INTER` on it (since 4fd1001):
  fixed here; the K26 window build from main now really builds `INTER = 0`.


## The inter build (RING memory, 2026-10-24)

`U200_CORE=0 U200_INTER=1 U200_WIN=5 U200_WIN1=3 U200_WIN2=3 fpga/u200/build.sh <tag>`: the whole decoder with every
frame buffer on UltraRAM rings (`docs/picture-memory.md`, step A: fb0 + its mirror fb0m, SR's fb0s, fb1, fb2),
CDEF / super-res / LR under the frame sequencer, the final picture flushed row by row to DDR and the references
read from those flushed pictures by `ref_fetch`. Host protocol per frame (tb/test_dec_host_inter.py, the
software's reference): reset; for inter frames the bands (`seg_map_seq`, `motion_field_seq`) and the slot table
+ REF_SLOT / REF_CFG (`ref_seq`, pointing at earlier frames' flush targets); DMA planes + AUTO at a fresh flush
target, LH, CH (CDEF on), CTRL pipe + tile columns, START frame; the tiles with the inherited CDF rows; poll
frame_done; poll DMA flush idle. A frame's flush target becomes its reference slots' address.
