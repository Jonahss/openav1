#!/bin/bash
# Differential fuzz: generate random-syntax intra streams with tools/gen_stream.py, decode with dav1d and
# with tools/decode.py, compare byte for byte. Usage: tools/fuzz_streams.sh <first_seed> <count> [extra gen flags]
# Streams that differ (or crash either decoder) are kept in streams/fuzz_fail/.
cd "$(dirname "$0")/.."
first=${1:-1}; count=${2:-20}; shift 2 2>/dev/null
mkdir -p streams/fuzz streams/fuzz_fail
S=${TMPDIR:-/tmp}/openav1_fuzz_$$; mkdir -p "$S"
pass=0; fail=0
for seed in $(seq "$first" $((first + count - 1))); do
  # vary format/bit depth/flags by seed unless caller passed explicit flags
  if [ $# -eq 0 ]; then
    case $((seed % 6)) in 0) fmt=420;; 1) fmt=444;; 2) fmt=422;; 3) fmt=mono;; 4) fmt=420;; 5) fmt=420;; esac
    case $((seed % 5)) in 0|1|2) bd=8;; 3) bd=10;; 4) bd=12;; esac
    [ "$fmt" = 422 ] && bd=10   # 4:2:2 needs profile 2 (which our writer only emits for 12-bit or 4:2:2)
    flags="--fmt $fmt --bd $bd"
    [ $((seed % 7)) -eq 0 ] && flags="$flags --sb128"
    [ $((seed % 4)) -eq 0 ] && flags="$flags --tiles"
    [ $((seed % 9)) -eq 0 ] && flags="$flags --screen"
    [ $((seed % 11)) -eq 0 ] && flags="$flags --lossless"
    case $((seed % 3)) in 0) dim="--w 128 --h 96";; 1) dim="--w 200 --h 136";; 2) dim="--w 72 --h 40";; esac
    flags="$flags $dim"
  else
    flags="$*"
  fi
  st=streams/fuzz/f$seed.ivf
  info=$(timeout 900 python3 tools/gen_stream.py "$st" --seed "$seed" $flags 2>&1) || { echo "FAIL seed $seed: gen error: $(echo "$info" | tail -1)"; fail=$((fail+1)); cp "$st" streams/fuzz_fail/ 2>/dev/null; continue; }
  if ! timeout 60 prefix-trace/bin/dav1d -q -i "$st" -o "$S/ref.yuv" --threads 1 >/dev/null 2>"$S/dav1d.err"; then
    echo "FAIL seed $seed [$flags]: dav1d rejected: $(tail -1 "$S/dav1d.err")"; fail=$((fail+1)); cp "$st" streams/fuzz_fail/; continue
  fi
  if ! timeout 1800 python3 tools/decode.py "$st" -o "$S/ours.yuv" >/dev/null 2>"$S/ours.err"; then
    echo "FAIL seed $seed [$flags]: decode.py error: $(tail -1 "$S/ours.err")"; fail=$((fail+1)); cp "$st" streams/fuzz_fail/; continue
  fi
  if cmp -s "$S/ref.yuv" "$S/ours.yuv"; then
    echo "PASS seed $seed [$flags] ${info#*: }"; pass=$((pass+1))
  else
    echo "FAIL seed $seed [$flags]: output DIFFERS ($(cmp "$S/ref.yuv" "$S/ours.yuv" | head -1))"; fail=$((fail+1)); cp "$st" streams/fuzz_fail/
  fi
done
rm -rf "$S"
echo "fuzz: $pass pass, $fail fail"
[ $fail -eq 0 ]
