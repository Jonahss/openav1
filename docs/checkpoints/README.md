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
- **layout**: the placement picture of the routed build, coloured by decoder block (from C10 on).

`fps.png` plots the unchecked speed from C10 on and the checked speed before it (the dagger on C10 marks the change of
measurement). It is drawn from the table: `python3 tools/checkpoints/chart.py docs/checkpoints` (SVG, and a PNG when a
headless Chromium is found). The `label` column places the few annotations: `dx;dy;anchor;text` with `|` between lines.

To add one: measure with `pf_card2.sh` (checked pass, then unchecked), read the five utilization rows, dump the
placement (`tools/checkpoints/dump_place.tcl` on the routed checkpoint, about five minutes) and render it
(`tools/checkpoints/render_place.py`), append the row, redraw the chart.
