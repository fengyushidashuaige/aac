#!/bin/bash
# 轨迹分析。用法: bash /tmp/analyze.sh [bag路径]  (不给则用最近一次)
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.144.60:11311
BAG=${1:-}
if [ -z "$BAG" ]; then
  BAG=$(ls -t /home/amov/logs/*/flight.bag 2>/dev/null | head -1)
  [ -z "$BAG" ] && BAG=$(ls -t /home/amov/logs/*/flight.bag.active 2>/dev/null | head -1)
fi
[ -z "$BAG" ] && { echo "!! 找不到 bag"; exit 1; }
echo "分析: $BAG"
python2 -u /home/amov/guttermap/analyze_flight.py "$BAG"
