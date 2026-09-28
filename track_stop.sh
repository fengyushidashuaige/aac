#!/bin/bash
B=/home/amov/guttermap
if [ -f $B/run/track.pid ]; then
  kill $(cat $B/run/track.pid) 2>/dev/null && echo "  stopped track ($(cat $B/run/track.pid))"
  rm -f $B/run/track.pid
else
  echo "  (没在跑)"
fi
