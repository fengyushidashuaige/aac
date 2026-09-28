#!/bin/bash
# 停止 检测 + 伺服  (不动 ROS/FAST-LIO/mavros)
B=/home/amov/guttermap

echo "=== 停止检测 + 伺服 ==="
for n in servo gutter_udp; do
  f=$B/run/$n.pid
  if [ -f "$f" ]; then
    pid=$(cat "$f")
    if kill "$pid" 2>/dev/null; then
      echo "  已停 $n (pid=$pid)"
    else
      echo "  $n pid=$pid 早就不在了"
    fi
    rm -f "$f"
  else
    echo "  $n 没在跑"
  fi
done

sleep 2

# 兜底: 按名字再扫一遍(防 pid 文件丢了)
for pat in "servo_track.py" "gutter_udp.py"; do
  p=$(pgrep -f "$pat" 2>/dev/null)
  if [ -n "$p" ]; then
    echo "  兜底清理 $pat: $p"
    pkill -f "$pat" 2>/dev/null
  fi
done

echo "[servo] 完成 (ROS/雷达/mavros 未动)"
