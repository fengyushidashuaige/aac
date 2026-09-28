# 水槽无人机 外场飞行手册

板子 `192.168.144.60`（amov/amov）。**顺序不能变**：上电 → 检查 → 起定位 → 起检测
→ 示教 → 出路径 → 追踪。

---

## 阶段 0 · 上电检查（最容易挂的一步）

雷达走 **USB 转网口**（RTL8152），断电后经常不枚举。**先查这个，别急着起飞：**

```bash
ip -br addr show eth1        # 必须看到 192.168.1.50/24
lsusb | grep -i 0bda         # 必须看到 0bda:8152 Realtek
ping -c2 192.168.1.144       # 必须 0% loss
```

不通 → **重插 Jetson 那端的 USB 转网口**；插在集线器/扩展坞上就换直连 USB 口。
（`nmcli` 里的 `lidar-eth1` 配置是持久的，不用管，插上自动配。）

> 典型误判：ping 报 `From 192.168.1.243 Destination Host Unreachable`
> —— 那是从 WiFi(wlan0) 发出去的。说明 eth1 没起来，不是雷达坏。

---

## 阶段 1 · 起定位 + 检测

```bash
bash ~/p600_fastlio/start.sh      # 约 40s
bash ~/guttermap/start_detect.sh
```

`start.sh` 结尾**必须看到三行有数**：雷达数据 / FAST-LIO 点云 / 发给飞控的定位。

- 预览：`http://192.168.144.60:8080/frame.jpg`
- 飞控应为 `connected: True`（`rostopic echo -n1 /mavros/state`）

---

## 阶段 2 · 示教（手动 POSCTL）

```bash
bash ~/guttermap/flight_record.sh teach2
```

→ 手动起飞（**POSCTL，绝对不要 OFFBOARD**），看预览把槽保持在画面中央，
**沿槽单程慢飞一趟**，落地停桨。

```bash
bash ~/guttermap/flight_stop.sh
bash ~/guttermap/save_logs.sh teach2      # ★ 必须紧跟，别插别的命令
bash ~/guttermap/make_path.sh teach2
```

**放行判据**（看 `make_path.sh` 的输出）：

| 指标 | 要求 | 昨天 teach1 的实测 |
|---|---|---|
| 内点占比 | ≥ 85% | 92% (1326/1447) |
| 内点残差 中位 | ≤ 60mm | 51.6mm |
| 折返比 | ≈ 1.00 | 1.00 |

**折返比要自己算**（脚本不打印）：`路径长度 ÷ 起点到终点直线距离`。
例：长 4.49m，起 (0.00,-0.04)、终 (4.49,-0.23) → 4.49 / 4.494 = **1.00** ✓。
明显大于 1（如 1.1+）= 示教时来回走了，路径脏，**重飞**。

---

## 阶段 3 · 追踪（★ 此阶段绝不能重启 FAST-LIO / 断电）

```bash
bash ~/guttermap/flight_record.sh track2
bash ~/guttermap/track_start.sh ~/guttermap/path_teach2_traj.csv 0.08
```

→ 手动起飞（POSCTL），盯着日志：

```bash
tail -f ~/guttermap/run/track.log
```

**达标（cte < 200mm 且 herr < 15deg）后**，两步：

1. 切 **OFFBOARD**
2. `rosservice call /gutter_track/set_enable true`

→ 飞完落地，**立刻**：

```bash
rosservice call /gutter_track/set_enable false
bash ~/guttermap/track_stop.sh
bash ~/guttermap/flight_stop.sh
bash ~/guttermap/save_logs.sh track2      # ★ 必须立刻
```

## 急停

```bash
rosservice call /gutter_track/set_enable false     # 停跟踪，飞机定点悬停
```

---

## 三条铁律

1. **示教之后到追踪结束，绝不能重启 FAST-LIO / 断电** —— 世界系一丢，路径作废，
   这趟白飞。（`make_path.sh` 是离线读 bag，不碰 FAST-LIO，安全。）
2. **每次飞完立刻 `save_logs.sh`** —— `start_detect.sh` / `track_start.sh` /
   `servo_start.sh` 都用 `>` 截断 `run/` 下的日志，晚一步永久丢失。
   （servo4 的原始日志就是这么丢的，只剩汇总数字。）
3. **bag 有 40 分钟上限**（`--duration=2400`）。所以**示教和追踪分开录**
   （`teach2` / `track2`）—— 也顺带解决"追踪段没录上"的老问题。

## 速度

- `0.15` = 旧值；今天降到 **0.08**。
- 控制律已与速度解耦（`pursuit.py`）：降速只是变慢，横向/偏航增益不变。
  `v_cruise=0.15` 时新旧逐位一致，所以这补丁只在传 0.08 时才有差别。
- 还原：`cp ~/guttermap/pursuit.py.bak_pre_speedinv ~/guttermap/pursuit.py`

## 备份文件（板上）

`pursuit.py.bak_pre_speedinv`、`detector.py.bak_pre_validgate` / `.bak_pre_oversize`、
`gutter_udp.py.bak_pre_valid`、`servo_track.py.bak_pre_valid` / `.bak_pre_kd`、
`gutter_track.py.bak_pre_*`

日志归档在 `~/guttermap/run/archive/`；航迹图在 `~/guttermap/run/traj_<名>.png`。
