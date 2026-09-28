#!/bin/bash
# 备份本次飞行的节点日志(伺服/跟踪/检测) —— 这些文件一重启节点就被覆盖!
# 用法: bash ~/guttermap/save_logs.sh <名字>     例: save_logs.sh servo1
B=/home/amov/guttermap
N=${1:-log}
D=$B/run/archive
mkdir -p $D
for f in servo track udp; do
  [ -f $B/run/$f.log ] && cp -f $B/run/$f.log $D/${f}_$N.log
done
FD=$(cat $B/run/flight_dir 2>/dev/null)
echo "  本次录包: ${FD:-（无）}"
echo "  已备份:"
ls -la --time-style=+%H:%M $D 2>/dev/null | grep "_$N.log" | awk '{printf "    %s  %s\n", $5, $7}'
echo "  目录: $D"
echo
echo "  分析命令:"
echo "    伺服: python3 $B/servo_report.py $D/servo_$N.log"
echo "    跟踪: grep -o 'cam=[+-][0-9]*' $D/track_$N.log | sed 's/cam=//' | sort -n | awk '{v[NR]=\$1} END{print \"cam 中位\", v[int(NR/2)], \"mm\"}'"
