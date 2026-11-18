#!/usr/bin/env bash
# dec.sh <stream.ivf|.obu> [more decode.py options]  (2026-11-10, Claw): a stream through the U200 with every output picture on the
# 7-inch panel (decode.py --xdma 0 --inter --card-load --pool 12 --show, from ~/openav1-wt-main). Needs the card programmed
# (~/measure1080/card_image.txt) and the caller in group video. The fps scoreboard (card_measure.sh) runs without --show.
W=~/openav1-wt-main; cd $W && source env.sh >/dev/null 2>&1
export PYTHONPATH=$W/tools:$W/tb:$W/tools/host
[ -f tools/host/libhostregs.so ] || bash tools/host/build_hostlib.sh >/dev/null
exec python tools/host/decode.py "$1" --xdma 0 --inter --card-load --pool 12 --frames 100000 --show "${@:2}"
