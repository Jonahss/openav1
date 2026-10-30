# Code tour: one AV1 stream, in at the host, out as pictures

Written 2026-10-30 for a first read of the code. Follow the bytes. Line numbers are from main at dae2461; `grep -n` the
names if they have moved. The demo that runs this whole path on the Alveo U200 is `fpga/u200/stream_to_video.sh`.

## 0. The two decoders you will see

- **The model** (`tb/`): Python written from the AV1 specification, process by process, with the spec's names.
  `tb/obu_parser.py` (OBUs, sequence and frame headers), `tb/tile_model.py` (`SymbolDecoder` = the arithmetic decoder of
  spec 8.2, `TileDecoder.decode_tile` = the block syntax of chapter 5/6), `tb/recon_model.py` (`FrameRecon`: prediction,
  transforms, reconstruction), `tb/lf_model.py`, `tb/cdef_model.py`, `tb/superres_model.py`, `tb/lr_model.py` (the
  in-loop filters), `tb/seq_model.py` (what carries from frame to frame). It is slow and it is the oracle: every RTL block
  is fuzzed against it, and on the card it still runs alongside for inter streams (section 6).
- **The RTL** (`rtl/`): the same processes in SystemVerilog. `rtl/dec_top.sv` is the decoder; `rtl/dec_host.sv` wraps it in
  a register file and FIFOs for a host; `fpga/u200/` and `fpga/kv260/` put that behind PCIe or the Kria's ARM cores.

## 1. The stream file -> OBUs -> headers (host, Python)

`tools/kv260/decode.py main()` (line 156) reads the file; `feed_ivf` (`tb/obu_parser.py:147`) splits the IVF container
into temporal units; `obu_parser.Decoder` parses each OBU: the sequence header (`SequenceHeader`, line 106: profile, bit
depth, subsampling), each frame header (`ParsedFrameHeader`, line 110: size, references, quantizers, filter parameters,
tile layout) and the tile groups (the compressed tile bytes, kept as `dec.tiles[i] = (tile header, bytes)`).

## 2. The host protocol: registers (host, Python + C)

`tb/host_regs.py` is the one protocol used by the simulation testbench, the Kria driver and the U200 driver.

- The register map is the comment block at the top of `rtl/dec_host.sv` (lines 7-66). Python mirrors it at
  `tb/host_regs.py:173-184` (`R_ID`, `R_CTRL`, `R_START`, `R_STATUS`, `R_DATA4` = push four tile bytes, `R_DEF_ADDR`/`R_DEF`
  = CDF rows, `R_HDR`/`R_RH`/`R_LH`/`R_CH`/`R_IH` = the header structs the blocks read, `R_DMA_*` = the picture flush,
  `R_SLOT`/`R_REF_*` = the reference slot table, `R_MF_*`/`R_SP_*` = the motion-field and segmentation bands).
- `HostDecoder` (line 304) owns a session. `write_seq` (363) sends (address, value) pairs; through `libhostregs.so`
  (`tools/kv260/hostlib.c`) each is one 32-bit store on the mapped BAR, ~0.1 us, versus ~7 us from Python.
- `run_tile` (422) / `run_tile_row` (472): write the frame's headers, the CDF rows (`default_rows_seq`, 379, or the rows
  inherited from an earlier frame), the engine mask and each engine's columns, then push the tile bytes four at a time
  into the FIFO (`R_DATA4`, budgeted by the free-word count at `0x070 + 4e`), pulse `START.tile`, and poll `STATUS`.
- On the U200 the bus is PCIe: `tools/kv260/xdma.py` `XdmaRegs.rd/wr` (26/29) are 32-bit accesses on `/dev/xdma0_user`;
  `XdmaStage.read/write` (43/54) move picture bytes with `/dev/xdma0_c2h_0` / `_h2c_0` (a `pread` at offset A is a DMA
  from card address A).

## 3. The FIFO and the tile engines (RTL)

`rtl/dec_host.sv` (module at line 68): the AXI-Lite register file, one byte FIFO per tile engine (`g_fifo`), the header
registers that become `hdr_t`/`rec_hdr_t`/`lf_hdr_t`/`cdef_hdr_t`, and the DMA engine that flushes finished rows to DDR.
It instantiates `dec_top`.

`rtl/dec_top.sv` (module at line 5) is the pipeline. `tile_engines` (`u_ts`, line 135) holds TE copies of
`rtl/tile_syntax.sv`: `msac` (the arithmetic decoder, spec 8.2, literal), `sym_seq`/`sym_path` (which symbol comes next),
`cdf_store` (the adaptive probability tables), `blk_syntax` (line 202 of tile_syntax.sv: partition, modes, MVs, the MV
candidate stack `mv_stack`, palette), `coef_rd` (coefficients). The output of this stage is a stream of blocks: mode
info into `mi_store` (`u_mi`, 185) and coefficients plus prediction parameters into reconstruction.

## 4. Reconstruction (RTL)

`rtl/recon_top.sv` (module at line 15): `ipred` (1144, intra prediction, all 13 modes + CfL at 1156), `ref_fetch` (353,
reference blocks from the frame memory, scaled references) and `mc_filt` (360, the 8-tap subpel filters, OBMC, warped
motion via `warp_setup`, compounds and masks), `itx2d` (1137, the level-folded inverse transforms; docs/itx-level-engine.md),
then add-and-clip into the picture memory. Everything is sized by the superblock-row window, not the picture
(docs/picture-memory.md).

## 5. In-loop filters and the picture memory (RTL)

Under the frame sequencer in `dec_top.sv`: `lf_top` (200, deblocking), `cdef_top` (207), `sr_top` (229, super-resolution),
`lr_top` (216, loop restoration). The picture memory is `frame_mem_uram` rings: `u_fb` + `u_fbm` (273/281: reconstruction
and its mirror, 5 superblock rows), `u_fb1` (333, after CDEF, 3 rows), `u_fb2` (350, after restoration, 3 rows). As the
last filter finishes a row, the DMA engine in `dec_host.sv` writes it to DDR (`R_DMA_*`), and for inter streams that DDR
picture is the reference the next frames fetch (`R_SLOT` table).

## 6. Back to the host: pictures, references, and what the host still carries

`HostDecoder.decode_frames` (658, intra / window path) and `decode_frames_inter` (736, inter path) drive the frame loop.
After each frame the host reads the picture back (`XdmaStage.read`, `unpack_dma` at 272) and `decode.py` writes it
(`write_planes` for raw, or the `.y4m` at line ~314) and compares it with the reference if one was given.

The honest part: in the inter path the Python model decodes every tile first (`decode_frames_inter`, comment at 741,
loop at ~812), because the card needs, per frame, the CDF rows inherited from an earlier frame, the motion-field bands
and the segmentation map, and that frame-to-frame state machine (`tb/seq_model.py`) lives on the host today. The card
does every pixel; the host hands it each frame's context. Moving that state onto the card is the next block. The intra
path (`--window`, `MODE=--window` in the demo script) runs without the model: 10 fps for 1080p intra on the U200.

## 7. Where the trust comes from

`docs/` and `tools/`: `tb/test_*.py` fuzz each block against the model (cocotb); `tools/nightly.sh` runs the ladder
(generated streams, the aomenc corpus vs dav1d, Argon conformance, the host path); `tools/gen_replay.py` + atium's
`gate_block.sh` replay recorded port vectors through the Vivado netlist; `fpga/u200/stream_to_video.sh --ref x.yuv`
compares the card's pictures with dav1d's byte for byte.
