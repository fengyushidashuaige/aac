#!/bin/bash
# 伺服分档调试助手 (改完自动重启 + 使能)
# 用法: bash ~/guttermap/servo_tune.sh <yaw|center|forward> [use_psi 0/1] [yaw_sign] [vy_sign] [kp_ang]
#   yaw     : 只发偏航, vy/vx=0   —— 手拧验方向
#   center  : 偏航 + 横移, 不前进  —— 手横移验方向
#   forward : 全开                —— 才可能飞
# 例: bash servo_tune.sh yaw          # 第一档
#     bash servo_tune.sh center       # 第二档
#     bash servo_tune.sh forward      # 第三档
#     bash servo_tune.sh yaw 0 1 -1      # 关掉 psi, 用原始 ang
#     bash servo_tune.sh center 0 0 -1   # 关掉航向环(kp_ang=0), 只控横向
B=/home/amov/guttermap
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.144.60:11311

STAGE=${1:-yaw}
PSI=${2:-1}
YS=${3:-1.0}
VS=${4:--1.0}
KA=${5:-}
KAARG=""
[ -n "$KA" ] && KAARG="_kp_ang:=$KA"
CR=${6:-}
[ -n "$CR" ] && KAARG="$KAARG _cruise:=$CR"
[ -n "$KA" ] && KAARG="_kp_ang:=$KA"

case "$STAGE" in
  yaw)     A="_yaw_only:=true" ;;
  center)  A="_center_only:=true" ;;
  forward) A="_forward:=true" ;;
  *) echo "档位只能是 yaw | center | forward"; exit 1 ;;
esac
if [ "$PSI" = "0" ]; then A="$A _use_psi:=false"; fi

if ! (timeout 3 rostopic list >/dev/null 2>&1); then
  echo "!! ROS 不通, 先 bash ~/p600_fastlio/start.sh"; exit 1
fi
if ! pgrep -f gutter_udp >/dev/null; then
  echo "!! gutter_udp 没跑, 先 bash ~/guttermap/start_detect.sh"; exit 1
fi
if [ -f $B/run/servo.pid ] && kill -0 $(cat $B/run/servo.pid) 2>/dev/null; then
  kill $(cat $B/run/servo.pid) 2>/dev/null; sleep 2
fi
setsid nohup python2 -u $B/servo_track.py $A _yaw_sign:=$YS _vy_sign:=$VS $KAARG \
  > $B/run/servo.log 2>&1 < /dev/null &
echo $! > $B/run/servo.pid
sleep 4
echo "[servo] 档位=$STAGE  use_psi=$PSI  yaw_sign=$YS  vy_sign=$VS  pid=$(cat $B/run/servo.pid)"
rosservice call /servo_track/set_enable true 2>&1 | tail -1
echo
echo "  看数据:  tail -f $B/run/servo.log"
echo "  停:      kill \$(cat $B/run/servo.pid)"
echo
echo "  日志格式: off=偏差mm  ang=原始倾角  psi=标定后航向  conf  vx vy yaw"
