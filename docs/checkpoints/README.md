# Checkpoints

A checkpoint is a card image that decodes the reference clip faster than the one before it. Each row of
`checkpoints.csv` records one:

- **clip**: `in_to_tree-aom.ivf`, 1080p, 548 coded frames, 500 shown, single tile (libaom). Always the same clip.
- **fps_unchecked**: the whole clip through the card, end to end with the host, nothing compared. This is the headline
  number from C10 on.
- **fps_checked**: the same run with every output picture compared against dav1d's (`--ref`). `identical` is the proof
  that goes with the speed. Rows before C10 only have this kind.
- **image / commit / clock_mhz**: the bitstream (`u200-<tag>-<commit>`), the RTL commit and the decoder clock.
- **lut_pct .. dsp_pct**: share of the xcu200's LUTs, registers, block RAM, UltraRAM and DSP blocks used by the whole
  design (decoder + PCIe/DDR4 shell), from the routed build's utilization report. Empty where the build is gone.
- **layout**: the placement picture of the routed build, coloured by decoder block. C01 to C09 were drawn on
  2026-11-19 from the builds that were still on the build machine (C01 and C02 got their image, commit and usage
  figures then too, from the journal's record of which image each measurement ran on).

**The 4K line** (`checkpoints_4k.csv`, rows K01 on, since 2026-11-22). The goal is 4K at 60 frames a second, and the
1080p clip cannot show progress there: the 4K images run at 80 MHz and read lower on it than the last 1080p
checkpoint. So the YouTube 2160p clip (`G6aI1-vYHxM`, 248 coded frames, 240 shown, four tile columns) has a table of
its own with the same columns, measured the same way (unchecked for the speed, a checked pass for `identical`), and a
second line in the chart on the same axis. A row is added when a kept image beats the last one on that clip. When the
4K line has caught up with the 1080p one, the chart goes back to one number.

`fps.png` plots the unchecked speed from C10 on and the checked speed before it (the dagger on C10 marks the change of
measurement). It is drawn from the table: `python3 tools/checkpoints/chart.py docs/checkpoints` (SVG, and a PNG when a
headless Chromium is found). The `label` column places the few annotations: `dx;dy;anchor;text` with `|` between lines.

To add one: measure with `pf_card2.sh` (checked pass, then unchecked), read the five utilization rows, dump the
placement (`tools/checkpoints/dump_place.tcl` on the routed checkpoint, about five minutes) and render it
(`tools/checkpoints/render_place.py`), append the row, redraw the chart.
