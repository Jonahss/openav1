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
