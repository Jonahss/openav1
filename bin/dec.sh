#!/usr/bin/env bash
# dec.sh <stream.ivf|.obu> [more decode.py options]: a stream through the U200 with every output picture on the 7-inch
# panel (decode.py --show), with the host software of the checkout this script lives in. Needs the card programmed
# (~/measure1080/card_image.txt) and the caller in group video. The fps scoreboard (card_measure.sh) runs without --show.
W=$(cd "$(dirname "$0")/.." && pwd); cd $W && source env.sh >/dev/null 2>&1
export PYTHONPATH=$W/tools:$W/tb:$W/tools/host
bash tools/host/build_hostlib.sh >/dev/null || exit 1
exec python tools/host/decode.py "$1" --show "${@:2}"
