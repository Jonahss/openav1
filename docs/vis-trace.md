# Decode-progress trace (`+vis=<file>` on dec_top)

One event per line, `cyc` = clock cycles since the frame's first tile started.

- `T cyc plane x y w h` — reconstruction wrote a transform block of `plane` (0 Y, 1 U, 2 V) at plane
  coordinates (x, y), size w x h pixels, to the frame buffer.
- `F cyc stage y0 y1` — a filter stage finished luma rows y0..y1: `L` deblocking, `C` CDEF, `S` super-resolution,
  `R` loop restoration (stripe).
- `E cyc` — frame finished (the last stage's last row).

`tools/vis_render.py` replays a trace onto the decoded picture (`tools/demo.sh` dumps it) as a video, at the
simulation's own pace (`--seconds <wall seconds>`) or as a short cut (`--seconds 30`).
