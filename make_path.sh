#!/bin/bash
# 一键出路径: ★推荐★ 直接用飞行轨迹拟合直线 —— 不碰相机偏差, 避开 100~350mm 系统误差
# 用法: bash ~/guttermap/make_path.sh <飞行名>
B=/home/amov/guttermap
N=${1:-}
if [ -z "$N" ]; then echo "用法: bash ~/guttermap/make_path.sh <飞行名>  (如 map3)"; exit 1; fi
BAG=/home/amov/logs/$N/flight.bag
if [ ! -f "$BAG" ]; then
  echo "!! 找不到 $BAG"; echo "   先: bash ~/guttermap/flight_record.sh $N  ...飞... bash ~/guttermap/flight_stop.sh"; exit 1
fi
source /opt/ros/melodic/setup.bash
echo "== 从 $BAG 出路径 =="
python2 $B/path_from_traj.py "$BAG" "$B/path_${N}_traj.csv" "$B/run/traj_${N}.png" 120
echo
echo "  ★ 路径: $B/path_${N}_traj.csv"
echo "     图  : $B/run/traj_${N}.png   (灰=轨迹, 绿=路径, 红圈=剔掉的段)"
echo
echo "  跟飞: bash $B/track_start.sh $B/path_${N}_traj.csv 0.15"
echo "        对准(cte<200mm,herr<15deg) -> 切 OFFBOARD -> rosservice call /gutter_track/set_enable true"
