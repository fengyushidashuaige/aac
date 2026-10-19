# 水槽视觉跟线 — 项目记录（勿随意改飞控栈）

> 更新：2026-07-21  
> **本仓库基线硬件：RK3588 + CubeOrange（ArduPilot GUIDED）**  
> 可飞 git 标记：`5442a83`（其后 master 含录像会话等，以 GitHub 为准）  
> 板子：`192.168.137.12` firefly/firefly；预览 `http://192.168.137.12:8080`  
> **约定：未明确要求时不要改板上/仓库飞行代码。**  
> **非本基线：** Amov Jetson + PX4 OFFBOARD = 后续移植，勿直接当当前可飞说明。

---

## 1. 当前 3588 + CubeOrange 怎么控无人机（结论）

**外环：视觉误差 → 双通道 PID → 机体速度指令**  
**内环：ArduPilot / CubeOrange（姿态/高度/油门）自己管**

| 环节 | 做法 |
|------|------|
| 感知 | USB 相机 → YOLO-seg RKNN（`gutter_seg_320_fp.rknn`）→ 槽中心线 → `offset_mm`、`angle_deg` |
| 控制律 | `controller.FlightController.update()`：横向 PID（mm→vy）、偏航 PID（deg→yaw_rate）；前进 vx 按对齐/巡航锁 |
| 下发 | MAVLink `SET_POSITION_TARGET_LOCAL_NED`，`MAV_FRAME_BODY_NED` |
| 有效量 | **vx, vy, yaw_rate**；**vz=0**（不做高度 PID） |
| 模式 | 手动切 **GUIDED**（`auto_guided=False`）；丢检超时可回 LOITER |
| 串口 | `/dev/ttyS8` @115200；S7 曾用于 NTRIP 注入 |

关键代码：

```text
offset_mm  → lateral_pid → vy   (roll_correction)
angle_deg  → yaw_pid(-angle) → yaw_rate
对齐/巡航锁 → vx (forward_speed)
→ set_position_target_local_ned(... BODY_NED, vx, vy, 0, yaw_rate)
```

**不是**：直接打电机 / 纯姿态角环 / 位置点跟踪（waypoint）。  
**是**：**视觉伺服 + PID 速度外环 + 飞控内环**。

当前量级（`config.py`）：`lat_kp≈0.12`，`yaw_kp≈0.045`，`max_vy≈0.10`，`max_yaw≈0.09`，`vx≈0.06`。

---

## 2. 视觉控无人机常见方式（对比）

| 方式 | 3588 是否在用 | 特点 | 区别 |
|------|---------------|------|------|
| **A. 速度指令 + PID（当前）** | ✅ | 误差→vy/yaw_rate/vx | 跟线直观；依赖飞控 GUIDED 速度跟踪；高度靠飞控 |
| **B. 姿态/RC override** | ❌（旧路可有） | 视觉→滚转/俯仰 PWM | 更“硬”、和手动抢通道；调试难 |
| **C. 位置/轨迹点** | ❌ | 视觉估相对位置→目标点 | 适合定点；跟连续线延迟大 |
| **D. 端到端学习控** | ❌ | 图→直接动作 | 难落地、难安全 |
| **E. 只提示不控** | 监控用 | 人飞 / MP | 最安全 |

本项目选 **A**：槽是连续线，要横向+航向同时修，速度环最合适。

---

## 3. 已做事项（本阶段）

### 硬件/链路
- CubeOrange + Here3（DroneCAN）；RTK：千寻 NTRIP→`GPS_RTCM_DATA` 可注入，但 Here3 难 FIXED
- `GPS_AUTO_CONFIG=2` 会卡 0 星 → 保持 **0**
- 掉高：程序 **不管高度**；诱因含丢检切 LOITER、大偏航、测距若参与定高时跳动

### 软件修复（进 `5442a83`）
1. **Yaw 反转**：角度跳变 >20° 不再死守旧角（`angle_jump_streak`）；连续 2 帧大跳跟新角并清 yaw 惯性；控制角限幅 ±35°
2. **GUIDED 被 UNKNOWN 清掉**：模式/解锁粘滞，无 HEARTBEAT 帧不清控
3. **激光高**：预览「气压高/激光高」；日志 `baro=` `rng=`；读 `DISTANCE_SENSOR`/`RANGEFINDER`
4. **日志持久化**：`/home/firefly/logs/gutter_flight_*.log`（不用 `/tmp`）

### 未改（核对过）
- 偏航符号仍 `yaw_pid.update(-angle_deg)`
- 仍只发 vx/vy/yaw_rate，vz=0，BODY_NED

### 检测侧（相对更早 8fe69cb）
- 320 模型 + 半分辨率 mask + PCA 中线（识别抖仍是平滑主矛盾）

---

## 4. 板上常用路径

| 项 | 路径 |
|----|------|
| 代码 | `/home/firefly/{main,controller,detector,config,preview}.py` |
| 模型 | `/home/firefly/models/gutter_seg_320_fp.rknn` |
| 启动 | `python3 -u main.py --no-preview --camera-id 23` |
| 日志 | `/home/firefly/logs/gutter_flight_latest.log` |
| 源仓库 | `D:\work\水槽无人机` @ `5442a83` |
| 工作区杂项 | `D:\3588调`（部署脚本多，飞行真源在水槽无人机） |

---

## 5. 明天试飞检查清单

1. 确认进程与日志目录非 `/tmp`
2. 预览看激光高是否乱跳（对照掉高）
3. 手动 GUIDED + 解锁后看 CTRL：`Angle` 与 `yaw` 应为**反号**
4. 后半段若再反转，拉 `/home/firefly/logs/` 算 wrong_rate

---

## 6. 代码怎么写的（极简）

```
main.py 循环:
  frame ← camera
  result ← detector.detect(frame)          # offset_mm, angle_deg
  cmd ← controller.update(...) 或 idle_decay()
  telem ← controller.read_telemetry()      # baro, rng_alt, mode
  controller.check_mode_transition(...)
  controller.send_command(cmd)             # 仅 GUIDED+armed
  写预览 + print CTRL/LOST（含 baro/rng）
```

`detector`：RKNN letterbox → mask → 中线/角度。  
`controller`：滤波/死区/PID/slew/巡航锁 → MAVLink 速度。  
`preview`：Flask MJPEG + `/api/telemetry`。
