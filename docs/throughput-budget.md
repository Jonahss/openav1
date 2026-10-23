# Throughput budget (first census, 2026-10-23)

Question: how far is the decoder from real-time, stage by stage? Answer from numbers already on disk: the
lr-stage Argon inter batches (`~/openav1-wt-lr/logs/lr*_summary.txt` on atium: 325 profile-0 streams through the
whole pipeline, `tb/test_dec_inter.py` at `TS_STAGE=lr`, every frame identical to dav1d). The testbench runs the
stages back to back (tiles, then deblocking, then CDEF, then loop restoration) and counts cycles per stage;
`pixels` is luma samples (W x H per frame).

| | total | per luma pixel |
|---|---|---|
| streams / frames (inter) / pixels | 325 / 3,624 (2,162) / 188.8 M | |
| all stages | 695.6 M cycles | **3.69** |
| tile stage (entropy + syntax + prediction + transforms + reconstruction) | 540.9 M | **2.87** |
| deblocking (`lf_top`) | 150.3 M | **0.80** |
| CDEF | 4.4 M | 0.02 (off in most of these streams) |
| loop restoration | 244 | ~0 (ditto) |

Per stream the total ranges from 1.61 to 10.55 cycles per pixel (median 4.29, 90th percentile 6.39): Argon
streams are synthetic stress content (tiny blocks, every tool on), so these are pessimistic for natural video,
whose big flat blocks cost far less syntax and prediction work per pixel.

## What real time needs

Luma pixels per second: 1080p60 = 124 M, 4K60 = 498 M. With the stages pipelined (the frame sequencer already
overlaps deblocking with the tiles, `pipe_en`), the throughput is set by the slowest stage, so each stage must
stay under:

| clock | 1080p60 | 4K60 |
|---|---|---|
| 80 MHz (K26 today) | 0.64 c/px | 0.16 |
| 200 MHz (U200 target) | 1.61 c/px | 0.40 |
| 400 MHz | 3.2 c/px | 0.80 |

So at 200 MHz the tile stage needs to drop from 2.9 (average) / 6.4 (p90) to 1.6 cycles per pixel: **2x on
average, 4x on the hard streams**, with deblocking already inside the budget at 0.80 and CDEF / LR to be measured
on streams that use them. 4K60 is a further 4x: 7-16x over today. The earlier "10-50x" figure was too high for
1080p60 and about right for 4K60 on stress content.

## Where the tile cycles go (to measure next)

The tile stage is one chain: `tile_syntax` (one symbol per cycle, the entropy decoder) feeding `recon_top`
(prediction one block at a time, 2 transform lanes, reconstruction 4 samples wide) through a block FIFO, with the
syntax side stalling when the FIFO is full (`perf_syn_stall`) and the reconstruction side idling when it is empty
(`perf_rec_idle`). The split between the two decides the first move:

- If reconstruction is the bottleneck (syntax stalls): wider reconstruction (8 or 16 samples a cycle through
  prediction and the residual add), 4 transform lanes (`ITX_LANES = 4`, the throughput setting the K26 could not
  afford), and the edge-gather / filter paths pipelined deeper.
- If syntax is the bottleneck (reconstruction idles): the entropy decoder at one symbol per cycle is the floor of
  this architecture (a coefficient is several symbols); the levers are the two-stage msac already in place at a
  higher clock, skipping the per-coefficient context work for zero runs, and eventually two tile columns decoded
  in parallel (the spec's tile independence, which a 1080p stream need not provide: encoders choose).
- Deblocking at 0.80 c/px is 16-25 cycles per filtered sample on a 1-sample datapath; it meets 1080p60 at 200 MHz
  as is and needs ~2x for 4K60 (two edges in flight, or 4 samples a cycle).

Next: rerun a slice of the batch with the two perf counters and the recon FSM census (`test_dec_top.py` prints
`recon tx-FSM cycles` per state) to attribute the 2.87, on natural clips (foreman, the 1080p testsrc) as well as
Argon; then the memory system, which has to carry the 4x wider datapaths (docs/frame-memory.md, the DDR version).
