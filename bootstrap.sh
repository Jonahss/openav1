#!/usr/bin/env bash
# openav1 bootstrap: idempotent setup of the simulation / verification environment on Debian or Ubuntu,
# x86_64 or arm64 (developed on a Raspberry Pi 5 and an Ubuntu 22.04 desktop). Re-run freely.
#
#   0. system packages (apt.sh): compilers, meson/ninja/nasm, verilator, yosys, aomenc/aomdec, pdftotext
#   1. python venv with cocotb 2.x (the testbenches), pytest, numpy
#   2. OSS CAD Suite (YosysHQ nightly): yosys with the slang frontend, used for full-SystemVerilog synthesis
#      estimates (`yosys -p "plugin -i slang; read_slang ..."`); simulation uses the apt verilator
#   3. dav1d (pinned) with the openav1 golden-trace hooks applied by tools/dav1d_trace_patch.py, built
#      asm-free into prefix-trace/ (the trace hooks sit in the C paths; --threads 1 when tracing).
#      Its output is byte-identical to an untraced build, so it also serves as the reference decoder.
#   4. the AV1 specification text (tb/av1_tables.py is generated from it by tools/gen_spec_tables.py)
#
# Everything lands inside the repository directory (toolchain/, venv/, third_party/, prefix-trace/: all
# ignored by git). Afterwards: `source env.sh`, then e.g. `tools/regress.sh` or `python tb/runner.py dec_top`.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
mkdir -p logs third_party toolchain
exec > >(tee -a "$ROOT/logs/bootstrap.log") 2>&1
echo "=== bootstrap start $(date -Is) on $(uname -m)"
J=$(nproc)

case "$(uname -m)" in
  x86_64)  OSS_ARCH=x64;   LIBDIR=x86_64-linux-gnu ;;
  aarch64) OSS_ARCH=arm64; LIBDIR=aarch64-linux-gnu ;;
  *) echo "unsupported architecture $(uname -m)"; exit 1 ;;
esac
OSS_DATE="2026-09-27"                       # YosysHQ/oss-cad-suite-build release tag
DAV1D_REV="bf5a879"                         # tools/dav1d_trace_patch.py is written against this revision

# 0. system packages
if ! grep -q APT_DONE logs/apt.log 2>/dev/null; then
  ./apt.sh > logs/apt.log 2>&1 || { echo "apt.sh failed, see logs/apt.log"; exit 1; }
fi
echo "--- apt ok; verilator $(verilator --version | awk '{print $2}'); yosys $(yosys -V | awk '{print $2}'); aomenc $(aomenc --help 2>&1 | grep -oE 'v[0-9.]+' | head -1)"

# 1. python venv + cocotb
[ -x venv/bin/python ] || python3 -m venv venv
venv/bin/pip install -q --upgrade pip
venv/bin/pip install -q "cocotb>=2.0" pytest numpy
echo "--- cocotb $(venv/bin/python -c 'import cocotb; print(cocotb.__version__)')"

# 2. OSS CAD Suite (yosys + slang for synthesis)
if [ ! -x toolchain/oss-cad-suite/bin/yosys ]; then
  url="https://github.com/YosysHQ/oss-cad-suite-build/releases/download/${OSS_DATE}/oss-cad-suite-linux-${OSS_ARCH}-${OSS_DATE//-/}.tgz"
  echo "--- fetching $url"
  curl -L --retry 3 -o toolchain/oss-cad-suite.tgz "$url"
  tar -xzf toolchain/oss-cad-suite.tgz -C toolchain
  rm -f toolchain/oss-cad-suite.tgz
fi
echo "--- oss-cad-suite yosys: $(toolchain/oss-cad-suite/bin/yosys -V)"

# 3. dav1d with the trace hooks, asm-free, into prefix-trace/
if [ ! -d third_party/dav1d ]; then
  git clone -q https://code.videolan.org/videolan/dav1d.git third_party/dav1d
  git -C third_party/dav1d checkout -q "$DAV1D_REV"
fi
if [ ! -f third_party/dav1d/src/trace.c ]; then
  venv/bin/python tools/dav1d_trace_patch.py third_party/dav1d
fi
if [ ! -x prefix-trace/bin/dav1d ]; then
  ( cd third_party/dav1d
    [ -d build-trace ] || meson setup build-trace --prefix="$ROOT/prefix-trace" --buildtype=release \
        -Denable_asm=false -Denable_tools=true -Denable_tests=false > /dev/null
    ninja -C build-trace -j"$J" > /dev/null
    ninja -C build-trace install > /dev/null )
fi
echo "--- dav1d (trace build): $(LD_LIBRARY_PATH=prefix-trace/lib/$LIBDIR:prefix-trace/lib prefix-trace/bin/dav1d --version 2>&1 | head -1) @ $DAV1D_REV"

# 4. the specification text (for the table generator; the checked-in tb/av1_tables.py already contains the result)
if [ ! -f third_party/spec/av1-spec.txt ]; then
  mkdir -p third_party/spec
  curl -L --retry 3 -o third_party/spec/av1-spec.pdf "https://aomediacodec.github.io/av1-spec/av1-spec.pdf"
  pdftotext -layout third_party/spec/av1-spec.pdf third_party/spec/av1-spec.txt
fi
echo "--- spec text: $(wc -l < third_party/spec/av1-spec.txt) lines"

echo "=== bootstrap done $(date -Is). Next: source env.sh"
