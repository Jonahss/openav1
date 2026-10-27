#!/usr/bin/env bash
# ila_capture.sh <ltx> <csv> -- <decode command...>: arm the card's ILA (fpga/u200/ila_capture.tcl, env ILA_* as there),
# run the decode command once the ILA is armed, then collect the capture as CSV. Needs hw_server on ILA_URL
# (default localhost:3121). Example:
#   fpga/u200/ila_capture.sh ~/openav1-synth/u200-ila50-48c9e48/sys_wrapper.ltx /tmp/test47.csv -- \
#       python tools/kv260/decode.py streams/argon/profile0_core/streams/test47.obu --xdma 0 --inter --frames 2
set -u
ltx=$1; csv=$2; shift 2; [ "${1:-}" = "--" ] && shift
here=$(cd "$(dirname "$0")" && pwd)
log=${csv%.csv}.ila.log
ILA_LTX=$ltx ILA_CSV=$csv vivado -mode batch -nojournal -nolog -source "$here/ila_capture.tcl" > "$log" 2>&1 &
vpid=$!
until grep -qE "^(ARMED|ERROR|TIMEOUT|no xcu200|no ILA|trigger glob)" "$log" 2>/dev/null; do kill -0 $vpid 2>/dev/null || break; sleep 1; done
if ! grep -q "^ARMED" "$log"; then echo "ILA not armed:"; grep -E "PROBES|TRIGGER|ERROR|no |matched" "$log" | head; wait $vpid; exit 1; fi
echo "ILA armed; running: $*"
"$@"
wait $vpid
grep -E "TRIGGER|CAPTURED|TIMEOUT|ERROR" "$log" | head
