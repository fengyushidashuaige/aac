#!/bin/bash
DIR=$(cat /home/amov/guttermap/run/flight_dir 2>/dev/null)
PGID=$(cat /home/amov/guttermap/run/flight_pgid 2>/dev/null)
if [ -n "$PGID" ]; then kill -INT -"$PGID" 2>/dev/null && echo "  SIGINT -> 进程组 $PGID"; fi
[ -f /home/amov/guttermap/run/flight_events.pid ] && kill $(cat /home/amov/guttermap/run/flight_events.pid) 2>/dev/null
for i in $(seq 1 10); do sleep 2; [ -f "$DIR/flight.bag" ] && { echo "  ✓ 已收尾 (${i}×2s)"; break; }; done
echo "[rec] 录制已停: $DIR"; ls -lh "$DIR" 2>/dev/null
