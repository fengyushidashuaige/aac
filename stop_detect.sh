#!/bin/bash
B=/home/amov/guttermap
for n in gutter_udp gutter_map; do
  f=$B/run/$n.pid
  if [ -f "$f" ]; then kill $(cat "$f") 2>/dev/null && echo "  stopped $n ($(cat $f))"; rm -f "$f"; fi
done
sleep 2; echo "[detect] 已停"
