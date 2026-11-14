Frames for `rtl/cdef_stream.sv` from inter frames (`tb/cdef_vec.py`): the deblocked luma plane, skip flags, cdef_idx,
CDEF parameters and `cdef_model`'s luma plane, written by the `dec_inter_lr` suite run with `TS_CDEF_VEC_OUT=<dir>`
on 2026-11-13 (branch filter-chain 67ddd4c). Six frames of Argon test6491 and test47 chosen for their unfiltered
blocks: whole 8x8 blocks skipped, 64x64 units without a cdef_idx, and blocks where the fourth skip flag decides.
Replayed by the `cdef_stream_inter` suite. To regenerate:
`TS_CDEF_VEC_OUT=/tmp/v NIGHTLY_ONLY='^dec_inter_lr$' tools/nightly.sh x`, then pick.
