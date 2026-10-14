#!/usr/bin/env bash
# Build tools/kv260/hostlib.so for the board (aarch64). The Kria image has no compiler; build on any aarch64 Linux
# (a Raspberry Pi does) and copy the .so next to decode.py: it uses only open / mmap / clock_gettime, so glibc
# version differences do not matter.
set -euo pipefail
cd "$(dirname "$0")"
gcc -O2 -Wall -shared -fPIC -o hostlib.so hostlib.c
echo "built $(pwd)/hostlib.so for $(uname -m)"
