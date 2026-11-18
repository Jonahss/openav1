#!/usr/bin/env bash
# Nightly openav1 regression on origin/main (cron 17 8 = 08:17 UTC = 00:17 PST / 01:17 PDT; suites 12 at a time since 2026-11-11). Added 2026-10-26 after a forgotten hand launch.
# Skips if a nightly is already running. Result line appended to ~/logs/nightly-cron.log.
set -u
pgrep -f "tools/nightly.sh main-" >/dev/null && { echo "$(date -Is) skip: a main nightly is already running" >> ~/logs/nightly-cron.log; exit 0; }
cd ~/openav1-wt-main || exit 1
git fetch -q origin && git checkout -q --detach origin/main || { echo "$(date -Is) checkout failed" >> ~/logs/nightly-cron.log; exit 1; }
h=$(git rev-parse --short HEAD)
$(dirname "$0")/nightly_run.sh ~/openav1-wt-main main-$h      # opens and closes its ticket on the Builds board (2026-11-17)
rc=$?   # captured before the $(date) substitution: the old inline $? was always 0 (2026-10-30)
echo "$(date -Is) main-$h rc=$rc $(grep -c ^PASS logs/nightly-main-$h.txt) pass $(grep -c ^FAIL logs/nightly-main-$h.txt) fail" >> ~/logs/nightly-cron.log
