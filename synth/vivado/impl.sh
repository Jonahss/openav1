#!/usr/bin/env bash
# Place and route a synthesised checkpoint (from run.sh, out-of-context) on the Kria K26, for timing and
# routability numbers. The design must have no black boxes: synthesise with -n for the parked stages and
# -f pr for the frame-memory stand-in, e.g.
#   synth/vivado/run.sh -n cdef_top,lr_top -f pr 10 9 core-pr && synth/vivado/impl.sh dec_top-core-pr
# Reports land next to the checkpoint: impl_timing.rpt, impl_util.rpt, impl_route_status.rpt, post_route.dcp;
# the log is impl.log. Needs Vivado on PATH.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
out=${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/$1
[ -f "$out/post_synth.dcp" ] || { echo "no $out/post_synth.dcp" >&2; exit 1; }
export IMPL_OUT=$out
mkdir -p "$out/work/impl"
cd "$out/work/impl"
exec vivado -mode batch -nojournal -log "$out/impl.log" -source "$here/impl.tcl"
