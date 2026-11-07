# openav1 environment: `source env.sh` from anywhere. Repo-relative, works on x86_64 and arm64.
OPENAV1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OPENAV1_ROOT
case "$(uname -m)" in
  x86_64)  _openav1_lib=x86_64-linux-gnu ;;
  aarch64) _openav1_lib=aarch64-linux-gnu ;;
  *)       _openav1_lib=. ;;
esac
# system verilator/yosys (apt) win; the OSS CAD Suite (yosys + slang, used for synthesis) is appended
export PATH="$OPENAV1_ROOT/prefix-trace/bin:$PATH:$OPENAV1_ROOT/toolchain/oss-cad-suite/bin"
export LD_LIBRARY_PATH="$OPENAV1_ROOT/prefix-trace/lib/$_openav1_lib:$OPENAV1_ROOT/prefix-trace/lib:${LD_LIBRARY_PATH:-}"
export DAV1D_TRACE_BIN="$OPENAV1_ROOT/prefix-trace/bin/dav1d"
export OPENAV1_YOSYS="$OPENAV1_ROOT/toolchain/oss-cad-suite/bin/yosys"
if [ -f "$OPENAV1_ROOT/venv/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "$OPENAV1_ROOT/venv/bin/activate"
fi
unset _openav1_lib

# Verilator model builds through ccache when it is installed (OBJCACHE is what Verilator's generated Makefile honours):
# a repeated build of the same RTL + parameters in any worktree compiles nothing. 2026-11-07 probe on atium: 61 s -> 39 s
# for the frame-8 inter twin including the run (the compile itself ~25 s -> ~3 s).
if command -v ccache >/dev/null 2>&1; then export OBJCACHE=ccache; export CCACHE_DIR="${CCACHE_DIR:-$HOME/.ccache-verilator}"; fi
