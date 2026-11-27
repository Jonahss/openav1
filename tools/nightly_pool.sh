# tools/nightly_pool.sh: the nightly's pool of simulator processes, sourced by tools/nightly.sh (2026-11-27, WF-18).
# A pool of PAR tokens; a process takes one before it starts and puts it back when it ends, so at most PAR run at once.
# A waiting process with the longest expected run goes first, ties in the order they asked: the night ends with its
# last long process. Until 2026-11-27 the pool was a pipe of PAR bytes, which hands tokens out in the order processes
# ask (file order), and the cron night of dacf14a (2026-11-27) spent 7,738 s where the same process times, replayed
# longest first, need 6,806 s (~/fork/run9/timeline-dacf14a.txt on atium: dec_inter's 17 shards, last in the file,
# waited up to 4,527 s and ran the last 40 minutes on 4 to 11 tokens).
# Expected seconds come from the times files of earlier nights in logs/ (nightly-<tag>-times.txt: "suite index seconds"
# per process, index "-" for a run suite, "b" for a runsh build, 0.. for a shard; a later file overrides an earlier
# one). A process with no entry is expected to be long (it starts first); with no times files at all the pool hands
# out tokens in ask order, as the pipe did.
#   pool_init PAR TIMES_OUT   tok_get SECONDS   tok_put   expect SUITE INDEX   expect_build SUITE   record SUITE INDEX SECONDS
POOL_UNKNOWN=9999999
declare -A pool_expect=()
pool_init() {
  pool_par=$1; pool_times=$2
  : > "$pool_times"
  local f s i sec
  for f in $(ls -tr logs/nightly-*-times.txt 2>/dev/null); do
    [[ "$f" -ef "$pool_times" ]] && continue
    while read -r s i sec; do [[ "$sec" =~ ^[0-9]+$ ]] && pool_expect["$s $i"]=$sec; done < "$f"
  done
  [[ $pool_par -gt 1 ]] || return 0
  pool=$(mktemp -d "${TMPDIR:-/tmp}/nightly-pool.XXXXXX")
  echo "$pool_par" > "$pool/free"
  trap 'rm -rf "$pool"' EXIT     # the main shell only: a ( ) subshell does not keep the trap
}
expect() { echo "${pool_expect["$1 $2"]:-$POOL_UNKNOWN}"; }
expect_build() {  # a build is expected to take its own time plus its suite's longest shard: they run one after the other
  local b=${pool_expect["$1 b"]:-} m=0 k
  [[ -n "$b" ]] || { echo $POOL_UNKNOWN; return; }
  for k in "${!pool_expect[@]}"; do [[ "$k" == "$1 "[0-9]* && ${pool_expect[$k]} -gt $m ]] && m=${pool_expect[$k]}; done
  echo $(( b + m ))
}
record() { echo "$1 $2 $3" >> "$pool_times"; }   # one short line, appended in one write
tok_get() {  # SECONDS expected: wait for a token; the waiter expecting the most seconds takes the next one
  [[ $pool_par -gt 1 ]] || return 0
  local p=$1; (( p > POOL_UNKNOWN )) && p=$POOL_UNKNOWN
  local me; me=$(printf 'w.%07d.%s' $(( POOL_UNKNOWN - p )) "$(date +%s%N)").$BASHPID   # this process's pid: outside $( )
  : > "$pool/$me"
  local got="" w first pid free LC_ALL=C   # the waiters' names sort by expected seconds, then by ask time
  while :; do
    {
      flock 8
      w=( "$pool"/w.* ); first=${w[0]##*/}
      pid=${first##*.}
      if [[ "$first" != "$me" ]] && ! kill -0 "$pid" 2>/dev/null; then rm -f "$pool/$first"   # a waiter that died
      else
        free=$(< "$pool/free")
        if [[ "$first" == "$me" && $free -gt 0 ]]; then echo $(( free - 1 )) > "$pool/free"; rm -f "$pool/$me"; got=1; fi
      fi
    } 8> "$pool/lock"
    [[ -n "$got" ]] && return 0
    sleep 1
  done
}
tok_put() {
  [[ $pool_par -gt 1 ]] || return 0
  { flock 8; echo $(( $(< "$pool/free") + 1 )) > "$pool/free"; } 8> "$pool/lock"
}
