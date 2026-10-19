# 水槽无人机 — 当前可飞版本快照（2026-10-19）

板子：`192.168.144.60`，用户 `amov/amov`，ROS Melodic（Python2）+ Python3 检测/TRT。

代码提交基线：`279fe28`（2026-09-24 收工状态，水槽无人机雷达路径跟踪）。

## 这套系统由哪几块组成

| 目录 | 作用 | 语言 |
|---|---|---|
| `/home/amov/guttermap` | 启动脚本、循迹节点、路径工具、预览 | 主要是 Python2 (ROS) |
| `/home/amov/gutter_uav` | 检测器包 + TRT 模型（`models/gutter_seg_320_v2.engine`） | Python3 |
| `/home/amov/p600_fastlio` | 定位一键启动（FAST-LIO2 + MAVROS + 位姿桥） | Shell/Python |
| `/home/amov/fastlio_ws/src/FAST_LIO` | 雷达惯导里程计（本地有改动，见 patch） | C++ |
| `/home/amov/fastlio_ws/src/livox_ros_driver2` | MID-360S 驱动（本地有改动） | C++ |

数据流：

```
MID-360S → FAST-LIO2 → /Odometry ─┐
下视相机 → gutter_udp(TRT) → UDP ─┼→ gutter_track.py → /mavros/setpoint_velocity/cmd_vel → PX4
                                  └→ preview :8080
```

## 起飞前启动顺序（重要）

```bash
# 0. 检查雷达网口（应看到 192.168.1.50/24）
ip -br addr show eth1
ping -c2 192.168.1.144

# 1. 定位：roscore + Mid360 + FAST-LIO + mavros + 位姿桥（约 40 秒）
bash ~/p600_fastlio/start.sh

# 2. 检测 + 投影（需要 /Odometry 已在跑）
bash ~/guttermap/start_detect.sh

# 3. 预览
#    http://192.168.144.60:8080
```

完整飞行流程见 `FLIGHT_RUNBOOK.md`，要点：

```bash
# 示教（手动 POSCTL，贴槽一次，不要折返）
bash ~/guttermap/flight_record.sh teach1
bash ~/guttermap/flight_stop.sh
bash ~/guttermap/save_logs.sh teach1
bash ~/guttermap/make_path.sh teach1

# 跟踪
bash ~/guttermap/flight_record.sh track1
bash ~/guttermap/track_start.sh ~/guttermap/path_teach1_traj.csv 0.08
# 等日志提示 OK 后：切 OFFBOARD，然后
rosservice call /gutter_track/set_enable true

# 停止
rosservice call /gutter_track/set_enable false
bash ~/guttermap/track_stop.sh
bash ~/guttermap/flight_stop.sh
bash ~/guttermap/save_logs.sh track1
```

## 快照内容

- `gutter_uav_src/` — 检测器源码（config/config_jetson/detector/main/gutter_seg/infer_trt 等）
- `models/gutter_seg_320_v2.engine` — 当前使用的 TensorRT 引擎（SHA256 见 `models/engine.sha256`）
- `p600_fastlio/` — 定位启动脚本与位姿桥
- `fastlio_patches/` — FAST-LIO / livox 驱动的本地改动、实际使用的 launch 与配置
- `fastlio_patches/fast_lio_local.patch` — 重建方法：上游 `hku-mars/FAST_LIO` + 应用此 patch

## 未包含

- VINS-Fusion 相关（`~/VINS-Fusion-gpu`、`~/vins_config`、`~/vins_to_mavros*.py`）：当前可飞路线用的是 FAST-LIO，VINS 是另一条实验线。
- `~/fastlio_ws` 的编译产物（build/devel，约 523MB）：按 patch 重建即可。
- 飞控固件与参数：见 `D:\jetson tiao\fc_backup`（P600 参数存 SD 卡，只写差异参数）。

## 重建定位环境

```bash
cd ~/fastlio_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO && git checkout <当时的上游提交> && git apply /path/to/fast_lio_local.patch
# livox_ros_driver2 同理应用 livox_local.patch
cd ~/fastlio_ws && catkin_make
```
