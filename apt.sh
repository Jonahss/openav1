#!/usr/bin/env bash
# System packages for openav1 on Debian 12/13 or Ubuntu 22.04+ (x86_64 or arm64). Needs sudo.
# bootstrap.sh calls this once; re-running is harmless.
set -euo pipefail
sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
  build-essential git cmake meson ninja-build nasm pkg-config \
  python3-pip python3-venv python3-dev ccache curl wget unzip zstd \
  perl help2man flex bison autoconf libfl-dev zlib1g-dev liblz4-dev \
  jq bc poppler-utils \
  verilator yosys aom-tools
echo APT_DONE
