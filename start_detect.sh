#!/bin/bash
# 起 "相机检测 + 世界系投影"。前置: roscore + livox + FAST-LIO 需已在跑
B=/home/amov/guttermap
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.144.60:11311
# --- 白平衡: 【本段已删, 2026-09-18】
#     原先在这里设 auto=0 temp=2800, 但程序内部的 cam_init() 启动时会设回
#     white_balance_temperature_auto=1(自动白平衡), 所以这段【从来没生效过】。
#     cam_init 里那句才是实际起作用的。旧注释说"AWB 坏了会品红偏色"是换镜头前的
#     历史问题 —— 现用镜头带 IR-CUT, AWB 工作正常(预览无偏色), 故保持自动白平衡。
#     若将来又要改白平衡, 改 gutter_udp.py 的 cam_init(), 不要改这里。

if ! (timeout 3 rostopic list >/dev/null 2>&1); then echo "!! ROS master 不通, 先起 ~/p600_fastlio/start.sh"; exit 1; fi
if ! (timeout 4 rostopic list 2>/dev/null | grep -q '^/Odometry'); then echo "!! /Odometry 不在, 先起 FAST-LIO"; exit 1; fi
if [ -f $B/run/gutter_udp.pid ] && kill -0 $(cat $B/run/gutter_udp.pid) 2>/dev/null; then
  echo "[detect] 已在跑 pid=$(cat $B/run/gutter_udp.pid)"; else
  cd /home/amov/gutter_uav
  setsid nohup python3 -u $B/gutter_udp.py > $B/run/udp.log 2>&1 < /dev/null &
  echo $! > $B/run/gutter_udp.pid; echo "[detect] 检测 pid=$(cat $B/run/gutter_udp.pid)"
fi
if [ -f $B/run/gutter_map.pid ] && kill -0 $(cat $B/run/gutter_map.pid) 2>/dev/null; then
  echo "[detect] 投影已在跑 pid=$(cat $B/run/gutter_map.pid)"; else
  setsid nohup python2 -u $B/gutter_map.py > $B/run/map.log 2>&1 < /dev/null &
  echo $! > $B/run/gutter_map.pid; echo "[detect] 投影 pid=$(cat $B/run/gutter_map.pid)"
fi
sleep 3
echo "[detect] 预览 http://192.168.144.60:8080/frame.jpg"
echo "[detect] 日志 $B/run/udp.log  $B/run/map.log"
