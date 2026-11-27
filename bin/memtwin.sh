#!/usr/bin/env bash
# memtwin.sh: the memory twin (K60-108): the card's own DDR4 controller with Xilinx's DDR4 model in xsim, with or
# without the image's memory interconnect and rf_ddr_bridge, fed by a traffic player. See tools/memtwin/README.md.
#
#   memtwin.sh setup <image dir>             make the twin in ~/memtwin from an image build (~/openav1-synth/u200-<tag>):
#                                            the controller's example design from the build's own configuration, the
#                                            interconnect's simulation netlist and stub, the synthetic patterns
#   memtwin.sh direct <pattern> ...          the player on the controller's AXI port (controller clock)
#   memtwin.sh sc     <pattern> ...          the player behind the image's interconnect, as reference fetch is on the card
#   memtwin.sh bridge <pattern> ...          the player behind rf_ddr_bridge; a second player on the interconnect (XDMA's place)
#   memtwin.sh check                         the bridge's data test: both players check every beat, then a mutant bridge
#                                            that misroutes one burst, which the check must catch
#   memtwin.sh mksc <name> [<if>.<PARAM>=<v> ...]   the interconnect made again with changed interface parameters
#   memtwin.sh scx <name> <pattern> ...      the player behind such an interconnect
#   memtwin.sh trace <start> <clocks>        handshakes per decoder clock on the interconnect's S02 entry path (after `sc` or `scx`)
#
# Results: the player's RESULT lines, also left in ~/memtwin/<mode>.log. Environment: TWIN_OUT, TWIN_DEC_PS, TWIN_ARGS
# (see run.tcl). One run at a time: every mode compiles into the same simulation project.
set -euo pipefail
here=$(cd "$(dirname "$0")/.." && pwd); src=$here/tools/memtwin; T=${MEMTWIN_DIR:-$HOME/memtwin}
source ~/.profile; source /opt/Xilinx/2025.2.1/Vivado/settings64.sh >/dev/null
xpr=$T/ex/sys_ddr_0_ex/sys_ddr_0_ex.xpr
viv() { name=$1; shift; "$HOME/ramq/bin/ramq" run - --base-gb 16 --per-worker-gb 0 --max-width 1 --name twin-$name -- vivado -mode batch -nojournal -nolog "$@"; }
show() { grep -E "^PATTERN |TWIN[B]*_STUCK|TWIN.* BAD|RESULT|^ERROR|rf_ddr_bridge:|TWIN_DONE" "$1" | cut -c1-220 || true; }
run() { mode=$1; net=$2; shift 2; cd $T; viv $mode -source $src/run.tcl -tclargs $xpr $src $T/pat $net "$@" > $T/$mode.log 2>&1 || true; show $T/$mode.log; }
verb=${1:-}; [ $# -gt 0 ] && shift
case "$verb" in
  setup)
    img=${1:?image dir}; mkdir -p $T; cd $T
    xci=$(find $img/proj -name sys_ddr_0.xci | head -1); scd=$(dirname "$(find $img/proj/u200.gen -name sys_sc_mem_0_stub.v | head -1)")
    viv setup -source $src/mk_example.tcl -tclargs $T $xci $HOME/vivado-dl/boards > $T/setup.log 2>&1
    grep -q EXAMPLE_DONE $T/setup.log || { echo "no example design, see $T/setup.log" >&2; exit 1; }
    cp $scd/sys_sc_mem_0_sim_netlist.v $scd/sys_sc_mem_0_stub.v $T/
    python3 $src/synth_patterns.py $T/pat; python3 $src/bridge_patterns.py $T/pat
    echo "twin in $T from $img";;
  direct) python3 $src/patch_top.py $T/ex/sys_ddr_0_ex/imports; run direct - "$@";;
  sc)     python3 $src/patch_top.py $T/ex/sys_ddr_0_ex/imports sc $T/sys_sc_mem_0_stub.v; run sc $T/sys_sc_mem_0_sim_netlist.v "$@";;
  bridge) python3 $src/patch_top.py $T/ex/sys_ddr_0_ex/imports bridge $T/sys_sc_mem_0_stub.v; run bridge $T/sys_sc_mem_0_sim_netlist.v "$@";;
  check)
    python3 $src/bridge_patterns.py $T/pat
    echo "== the bridge as it is"; TWIN_ARGS="CHECK=1 CHECKB=1" "$0" bridge chk
    good=$(grep -c "read check: 0 bad beats" $T/bridge.log || true)
    # the mutant: one burst's beats sent the other way; made in a copy of the source directory, never in the repo
    m=$T/mutant_src; rm -rf $m; cp -r $src $m; cp $here/fpga/u200/rf_ddr_bridge.sv $m/
    python3 - $m/rf_ddr_bridge.sv <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
old = "assign df_wr = m_rvalid && m_rid[0];"
assert s.count(old) == 1
s = s.replace(old, "reg [9:0] mut_n = 0; always @(posedge uclk) if (m_rvalid && m_rlast) mut_n <= mut_n + 1;\n    assign df_wr = m_rvalid && (m_rid[0] ^ (mut_n == 10'd700));   // MUTANT")
open(p, "w").write(s)
PY
    echo "== the mutant bridge"; cd $T
    TWIN_ARGS="CHECK=1 CHECKB=1" viv mutant -source $src/run.tcl -tclargs $xpr $m $T/pat $T/sys_sc_mem_0_sim_netlist.v chk > $T/mutant.log 2>&1 || true; show $T/mutant.log
    bad=$(grep -c "BAD beat" $T/mutant.log || true); rm -rf $m
    echo "bridge: $good of 2 players read back clean; mutant: $bad bad beats reported"
    [ "$good" = 2 ] && [ "$bad" -gt 0 ] && echo "MEMTWIN_CHECK PASS" || { echo "MEMTWIN_CHECK FAIL"; exit 1; };;
  mksc)   n=${1:?name}; shift; cd $T; viv mksc -source $src/mk_sc.tcl -tclargs $T/sc $n $T/sys_sc_mem_0_stub.v "$@" > $T/mksc_$n.log 2>&1; grep -E "^OVERRIDE|MK_SC_DONE|^ERROR" $T/mksc_$n.log | cut -c1-200;;
  scx)    n=${1:?name}; shift; python3 $src/patch_top.py $T/ex/sys_ddr_0_ex/imports sc $T/sys_sc_mem_0_stub.v ${n}_wrapper; run scx_$n $T/sc/$n/files.f "$@";;
  trace)  cd $T; viv trace -source $src/trace.tcl -tclargs $xpr $T/pat short "${1:?start, e.g. 50us}" "${2:?clocks}" > $T/trace.log 2>&1 || true; grep -E "^SCOPE|^STAGE|^TRACE|^T |^ERROR" $T/trace.log | cut -c1-400;;
  *) sed -n 2,22p "$0"; exit 2;;
esac
