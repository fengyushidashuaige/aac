#!/bin/bash
B=/home/amov/guttermap
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.144.60:11311
if ! (timeout 3 rostopic list >/dev/null 2>&1); then
  echo "!! ROS master 不通, 先跑 ~/p600_fastlio/start.sh"; exit 1; fi
setsid nohup python2 -u $B/source_monitor.py > $B/run/monitor.log 2>&1 < /dev/null &
echo $! > $B/run/monitor.pid
sleep 3
echo "[monitor] pid=$(cat $B/run/monitor.pid)"
echo "  看状态:  tail -f $B/run/monitor.log"
echo "  停止:    kill \$(cat $B/run/monitor.pid)"
