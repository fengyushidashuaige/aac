#!/bin/bash
# 飞行记录: rosbag + 事件日志。用法: bash /tmp/flight_record.sh [会话名]
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.144.60:11311
RB="ros""bag"                      # 拼接, 避免任何自匹配
NAME=${1:-flight_$(date +%Y%m%d_%H%M%S)}
DIR=/home/amov/logs/$NAME
mkdir -p "$DIR"
echo "$DIR" > /home/amov/guttermap/run/flight_dir

setsid nohup $RB record --duration=2400 -O "$DIR/flight.bag" \
  /Odometry /tf /mavros/state /mavros/local_position/pose \
  > "$DIR/rosbag.log" 2>&1 < /dev/null &
sleep 6
PGID=$(ps -eo pgid,cmd | awk -v k="$RB" -v d="$DIR" '$0 ~ k && index($0,d)>0 {print $1}' | head -1)
echo "$PGID" > /home/amov/guttermap/run/flight_pgid

setsid nohup python2 -u /home/amov/guttermap/flight_events.py > "$DIR/events.log" 2>&1 < /dev/null &
echo $! > /home/amov/guttermap/run/flight_events.pid

echo "[rec] 会话   $DIR"
echo "[rec] rosbag PGID=${PGID:-未找到}  events pid=$(cat /home/amov/guttermap/run/flight_events.pid)"
ls -l "$DIR"
echo "[rec] 停止: bash /tmp/flight_stop.sh"
