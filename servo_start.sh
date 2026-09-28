#!/bin/bash
# 视觉伺服循迹 (半自动) 启动
B=/home/amov/guttermap
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.144.60:11311
if ! (timeout 3 rostopic list >/dev/null 2>&1); then
  echo "!! ROS master 不通, 先跑 ~/p600_fastlio/start.sh"; exit 1
fi
echo "[servo] 检查检测进程 (要它发 UDP 偏移)..."
if pgrep -f gutter_udp >/dev/null; then echo "  ✓ gutter_udp 在跑"; else
  echo "  ✗ gutter_udp 没跑, 先: bash ~/guttermap/start_detect.sh"; fi
setsid nohup python2 -u $B/servo_track.py > $B/run/servo.log 2>&1 < /dev/null &
echo $! > $B/run/servo.pid
sleep 4
echo "[servo] pid=$(cat $B/run/servo.pid)  日志 $B/run/servo.log"
echo "  使能: rosservice call /servo_track/set_enable true"
echo "  停止: kill \$(cat $B/run/servo.pid)"
