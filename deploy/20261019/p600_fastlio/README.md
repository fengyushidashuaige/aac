# P600 FAST-LIO2 视觉定位

Livox MID-360 激光雷达 + FAST-LIO2 + MAVROS → PX4 视觉定位

## 一键用法

```bash
bash ~/p600_fastlio/start.sh     # 启动
bash ~/p600_fastlio/stop.sh      # 停止
bash ~/p600_fastlio/stop.sh --all   # 停止并关掉 roscore
bash ~/p600_fastlio/stop.sh --vins  # 顺带停掉 VINS 链路
```

日志在 `logs/`：`livox.log` `fastlio.log` `mavros.log` `bridge.log` `roscore.log`

## 链路

```
Livox MID-360 (192.168.1.144)
   │ USB 转网口 RTL8152, 点云 56300→56301, IMU 56400→56401
   ▼
eth1 = 192.168.1.50/24
   ▼
① livox_ros_driver2_node ──► /livox/lidar (PointCloud2),  /livox/imu (Imu)
   ▼
② fastlio_mapping (FAST-LIO2) ──► /Odometry (frame_id=camera_init)
   │                                /cloud_registered, /path
   ▼
③ fastlio_to_mavros.py ──► /mavros/vision_pose/pose   (默认透传)
   ▼
④ mavros_node (fcu_url=/dev/ttyTHS1:921600) ──► PX4 EKF2
```

## 坐标系约定(重要)

FAST-LIO 的 `camera_init` 世界系 = **开机瞬间的 IMU 机体系**。
源码依据 `IMU_Processing.hpp:198`，`//state_inout.rot = Eye3d;` 是注释掉的，只设了 `grav`，所以 `rot` 保持单位阵。

=> **开机时飞机必须摆平、机头朝向一致**，此时输出姿态就是真实姿态，直接透传即可。
=> 若开机时飞机歪着，会有一个固定的假 roll/pitch 喂给飞控。

> 这与 VINS 不同：VINS 世界系是**相机初始朝向**，恒有一个固定假 roll，所以 `vins_to_mavros*.py` 必须做修正。
> FAST-LIO 不需要。桥接脚本仍保留 `auto_cal` / `fixed` 两种模式备用：
> `python fastlio_to_mavros.py _mode:=auto_cal` （开机采样 2.5s 静止姿态作基准）

## 配置文件位置

| 内容 | 文件 |
|---|---|
| 雷达 IP / 主机 IP | `~/fastlio_ws/src/livox_ros_driver2/config/MID360_config.json` |
| 话题映射 / 外参 | `~/fastlio_ws/src/FAST_LIO/config/mid360.yaml` |
| 飞控串口 | 本目录 `start.sh` 里的 `FCU_URL=/dev/ttyTHS1:921600` |

## 前提 / 依赖

- ROS Melodic，`~/fastlio_ws` 已编译（`fast_lio` + `livox_ros_driver2`）
- `mavros` 已安装（apt）
- `eth1` = 192.168.1.50/24（NetworkManager 连接 `lidar-eth1`，已持久化）
- 雷达 192.168.1.144 可达（`ping 192.168.1.144`）

## 故障排查

| 现象 | 处理 |
|---|---|
| `start.sh` 报"雷达无数据" | 看 `logs/livox.log`；**给雷达断电重启**（雷达卡在旧推流状态时固件不接受新连接）|
| `rostopic hz /livox/lidar` 无输出 | `ss -ulnp \| grep 5630` 看数据端口有没有绑上；没绑上=驱动没握上手 |
| **数据端口 56301/56401 一直没绑** | **确认雷达型号! 这台是 MID-360S(dev_type=35), 必须用 `msg_MID360s.launch`**。用成 MID360 的 launch 会静默不建通道(日志只说 `Handle detection data, dev_type:35` 然后什么都不做) |
| `/Odometry` 有但 `/mavros/vision_pose/pose` 无 | 看 `logs/bridge.log`；确认 `rostopic echo -n1 /mavros/state` 的 connected=true |
| 飞控收到固定假 roll/pitch | 开机时飞机没摆平；或改用 `_mode:=auto_cal` |
| 定位数据跳变 | 检查是否有 VINS 桥接同时在发 `/mavros/vision_pose/pose` |

## 注意

- **不要让 VINS 和 FAST-LIO 同时发 `/mavros/vision_pose/pose`**，会互相覆盖
- `mid360.yaml` 的 `pcd_save_en` 已改为 `false`（原 `true` 会把所有帧攒内存，长时间飞会 OOM）
- 修改过的文件都留了 `.orig` 备份
