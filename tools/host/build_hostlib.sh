#!/usr/bin/env bash
# Build tools/host/libhostregs.so (the host driver's register traffic in C, tools/host/hostlib.c) for this machine.
set -euo pipefail
cd "$(dirname "$0")"
gcc -O2 -Wall -shared -fPIC -o libhostregs.so hostlib.c
echo "built $(pwd)/libhostregs.so for $(uname -m)"
