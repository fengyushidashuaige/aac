#!/bin/bash
# 一键出路径: 从某次 flight_record 的 bag + 最新的 detect.csv 生成 polyline
#   -> ~/guttermap/path_<flight>.csv   (可直接喂给 track_start.sh)
#   -> ~/guttermap/run/map_<flight>.png
# 用法: bash ~/guttermap/auto_path.sh [flight名] [detect.csv]
B=/home/amov/guttermap
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.144.60:11311

FLIGHT=${1:-$(cat $B/run/flight_dir 2>/dev/null)}
FLIGHT=$(basename "$FLIGHT")
BAG=${2:-/home/amov/logs/$FLIGHT/flight.bag}
if [ ! -f "$BAG" ]; then
  echo "!! 找不到 bag: $BAG"; echo "   用法: bash auto_path.sh <flight名>"; exit 1
fi
# detect.csv: 优先用户指定, 否则用最新的
if [ -n "${3:-}" ]; then CSV="$3"; else
  CSV=$(ls -t /home/amov/logs/guttermap_*/detect.csv 2>/dev/null | head -1)
fi
if [ ! -f "$CSV" ]; then echo "!! 找不到 detect.csv"; exit 1; fi

echo "== bag  : $BAG"
echo "== 检测 : $CSV"
echo
python2 $B/build_polyline.py "$BAG" "$CSV" || exit 1

SDIR=$(dirname "$CSV")
if [ -f "$SDIR/polyline.csv" ]; then
  # ★ 路径清洗: 去掉"掉头折返"那段(会被纯追踪照飞 -> 把飞机带出槽)
  python3 $B/clean_path.py "$SDIR/polyline.csv" "$B/path_${FLIGHT}.csv" 15 9
  cp -f "$SDIR/polyline.csv" "$B/polyline_${FLIGHT}_raw.csv"
  python2 $B/make_map.py "$BAG" "$SDIR/polyline.csv" "$B/run/map_${FLIGHT}.png" | tail -1
  echo
  echo "== 路径: $B/path_${FLIGHT}.csv"
  echo "== 图  : $B/run/map_${FLIGHT}.png  (红线=槽线, 蓝线=飞行轨迹, 应基本重合)"
  echo
  echo "== 起飞前先量一下路径偏差(相机看槽 vs 路径): 跑"
  echo "     bash $B/track_start.sh $B/path_${FLIGHT}.csv 0.15"
  echo "   然后 tail -f $B/run/track.log 看 cte/fix/cam"
else
  echo "!! 没生成 polyline.csv"; exit 1
fi
