"""
飞行控制器 — PID控制器 + 飞控通信 (MAVLink / RC Override)

将检测到的中心线偏移转换为飞行修正指令。

横向修正策略:
  偏移量(px) → PID → 横滚角修正 → 飞控

航向修正策略:
  中心线角度(rad) → PID → 偏航角速率修正 → 飞控
"""

import time
import struct
from collections import deque
from typing import Optional, Tuple
from dataclasses import dataclass

import numpy as np

from config import Config, ControlConfig, default_config


@dataclass
class ControlCommand:
    """控制指令 (GUIDED模式速度控制)"""
    roll_correction: float = 0.0     # 横向速度 vy (m/s), 正值=右
    yaw_correction: float = 0.0      # 偏航角速度 (rad/s)
    forward_speed: float = 0.0       # 前进速度 vx (m/s)
    forward_align: float = 0.0       # 对齐系数 0~1 (决定前进开多大)
    throttle_adjust: float = 0.0     # 垂直速度 (通常为0)
    active: bool = False
    timestamp: float = 0.0


class PID:
    """标准PID控制器 (带积分限幅和微分滤波)"""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = 30.0,
                 integral_limit: float = 10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._prev_output: float = 0.0
        self._initialized: bool = False

    def update(self, error: float, dt: float) -> float:
        """
        PID一步更新

        Args:
            error: 当前误差 (设定值 - 实际值)
            dt: 时间步长 (秒)

        Returns:
            控制量
        """
        if not self._initialized:
            self._prev_error = error
            self._initialized = True
            # 首帧给 P，避免 reset 后整整一拍输出为 0
            p0 = self.kp * error
            return max(-self.output_limit, min(self.output_limit, p0))

        # 限制dt范围: 防除零 + 防长时间延迟后微分爆炸
        dt = max(1e-6, min(dt, 0.1))

        # 比例项
        p = self.kp * error

        # 积分项 (带限幅，防积分饱和 windup)
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        # 微分项 (使用误差微分，带低通滤波避免噪声放大)
        d_error = (error - self._prev_error) / dt
        d = self.kd * d_error

        # 综合
        output = p + i + d
        output = max(-self.output_limit, min(self.output_limit, output))

        self._prev_error = error
        self._prev_output = output

        return output

    def reset(self):
        """重置积分和状态"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False


class FlightController:
    """
    飞行控制器

    输入: 横向偏移(mm) + 航向偏差(度)
    输出: 横滚/偏航修正指令

    使用方式:
        fc = FlightController(config)
        cmd = fc.update(offset_mm=25.0, angle_deg=-3.0, dt=0.033)
        fc.send_command(cmd)  # 发送到飞控
    """

    def __init__(self, config: Config = None):
        self.config = config or default_config
        cc = self.config.control

        # --- PID控制器 ---
        self.lateral_pid = PID(
            kp=cc.lateral_kp,
            ki=cc.lateral_ki,
            kd=cc.lateral_kd,
            output_limit=cc.max_lateral_speed,
        )

        self.yaw_pid = PID(
            kp=cc.yaw_kp,
            ki=cc.yaw_ki,
            kd=cc.yaw_kd,
            output_limit=cc.max_yaw_rate,
        )

        # --- 时间 ---
        self._last_update_time = time.perf_counter()

        # --- MAVLink (可选) ---
        self._mav_conn = None
        if cc.use_mavlink:
            self._init_mavlink()

        # --- GUIDED模式状态 ---
        self._guided_active: bool = False
        self._loss_time: float = 0.0
        self._fc_armed: bool = False
        self._last_fc_mode: str = "UNKNOWN"
        self._phase: int = 0  # 0=deadzone, 1=yaw, 2=lateral

        # --- 运行统计 ---
        self.update_count: int = 0

        # --- 死区：略小于吹叶区，近中心精修 ---
        self.dead_zone_mm: float = float(getattr(cc, "dead_zone_mm", 12.0))
        self.dead_zone_deg: float = float(getattr(cc, "dead_zone_deg", 3.0))
        self._prev_offset_mm: float = 0.0
        self._have_prev_offset: bool = False

        # --- yaw/vy/vx 指令斜坡 ---
        self._yaw_cmd: float = 0.0
        self._vy_cmd: float = 0.0
        self._vx_cmd: float = 0.0
        self._filt_angle: float = 0.0
        self._have_angle: bool = False
        self._angle_jump_streak: int = 0  # 连续大跳变帧数
        self._filt_offset: float = 0.0
        self._have_offset: bool = False
        self._offset_jump_streak: int = 0
        # 偏移速度估计（mm/s，滑窗线性拟合）— 用于阻尼/预测/锁心判据
        # 逐点差分会被检测白噪声淹没（30Hz 下原始差分 σ≈1000mm/s），
        # 滑窗斜率对 ~0.9s 做最小二乘，白噪声只剩 ~10mm/s
        self._filt_off_dot: float = 0.0
        self._have_off_dot: bool = False
        self._off_buf: deque = deque()

        # --- 巡航锁：对中后匀速前进，偏差不大就不掉速 ---
        self._cruise_locked: bool = False
        self._cruise_ok_frames: int = 0

    # ═══════════════════════════════════════════════════════════
    # 主接口
    # ═══════════════════════════════════════════════════════════

    def update(self, offset_mm: float, angle_deg: float,
               confidence: float = 1.0, held: bool = False) -> ControlCommand:
        """
        寻锁中心 + 匀速前进:
          - 偏了就轻轻横移/偏航对中
          - 对中后进入巡航锁：恒定 vx，只做小修正
          - held 时弱修正、减速
        """
        now = time.perf_counter()
        dt = now - self._last_update_time
        self._last_update_time = now
        self.update_count += 1

        cmd = ControlCommand(timestamp=now)

        # 2026-09-12: 0.3 -> config.min_accept_conf (0.6)
        # 依据: 真水槽 conf~0.87, 吹风筒误检 conf~0.35
        if confidence < float(getattr(self.config.control, "min_accept_conf", 0.6)):
            self._cruise_locked = False
            self._cruise_ok_frames = 0
            return cmd

        cc = self.config.control
        # 角度滤波：大跳变不再死守旧角（会死锁成反方向偏航），改为快速跟上新角
        if self._have_angle:
            d_ang = angle_deg - self._filt_angle
            # [-180,180] 最短差（角度在 ±90 门控内，通常不必，但防跳变）
            if d_ang > 180.0:
                d_ang -= 360.0
            elif d_ang < -180.0:
                d_ang += 360.0
            ad = abs(d_ang)
            if ad > 20.0:
                self._angle_jump_streak += 1
                if self._angle_jump_streak >= 2:
                    # 连续两帧都大跳 → 认定几何已变，重置跟新角，并清 yaw 惯性
                    self._filt_angle = angle_deg
                    self._yaw_cmd *= 0.3
                    self.yaw_pid.reset()
                    self._angle_jump_streak = 0
                else:
                    # 单帧可能噪声：半跟新角，避免完全锁死旧方向
                    self._filt_angle = 0.45 * self._filt_angle + 0.55 * angle_deg
                angle_deg = self._filt_angle
            else:
                self._angle_jump_streak = 0
                self._filt_angle = 0.72 * self._filt_angle + 0.28 * angle_deg
                angle_deg = self._filt_angle
        else:
            self._filt_angle = angle_deg
            self._have_angle = True
            self._angle_jump_streak = 0

        # 横向偏移滤波：抑制检测噪声导致的 vy 抖动（精准贴中关键）
        if self._have_offset:
            d_off = offset_mm - self._filt_offset
            if abs(d_off) > 120.0:
                self._offset_jump_streak += 1
                if self._offset_jump_streak >= 2:
                    # 连续两帧都大跳 → 几何已变，快速跟上新值；
                    # 不再压 _vy_cmd（持续大偏移是真实位置，压死会导致对不进中心）
                    self._filt_offset = 0.85 * self._filt_offset + 0.15 * offset_mm
                    self._filt_off_dot = 0.0
                    self._have_off_dot = False
                    self._off_buf.clear()
                    self._offset_jump_streak = 0
                else:
                    self._filt_offset = 0.50 * self._filt_offset + 0.50 * offset_mm
            else:
                self._offset_jump_streak = 0
                # 近中心更信滤波，远距更快跟随
                a = 0.55 if abs(self._filt_offset) < 60.0 else 0.35
                self._filt_offset = a * self._filt_offset + (1.0 - a) * offset_mm
            offset_mm = self._filt_offset
        else:
            self._filt_offset = offset_mm
            self._have_offset = True
            self._offset_jump_streak = 0

        # 偏移速度估计（mm/s）：滑窗最小二乘斜率（抗白噪声）
        win_s = 2.5 * float(getattr(cc, "offset_deriv_tau_s", 0.35))
        self._off_buf.append((now, float(offset_mm)))
        while self._off_buf and now - self._off_buf[0][0] > win_s:
            self._off_buf.popleft()
        if len(self._off_buf) >= 5:
            ts = np.array([t for t, _ in self._off_buf])
            os_ = np.array([o for _, o in self._off_buf])
            if ts[-1] - ts[0] > 1e-3:
                try:
                    self._filt_off_dot = float(np.polyfit(ts - ts[0], os_, 1)[0])
                except Exception:
                    pass
            self._have_off_dot = True

        # 控制用偏移限幅：垃圾检测按边界值纠，避免打满乱打
        max_ctrl_off = float(getattr(cc, "max_ctrl_offset_mm", 500.0))
        offset_mm = max(-max_ctrl_off, min(max_ctrl_off, float(offset_mm)))
        # 控制用角度限幅，避免 ±50°+ 噪声把 yaw 打满单边
        angle_deg = max(-35.0, min(35.0, float(angle_deg)))

        abs_angle = abs(angle_deg)
        abs_offset = abs(offset_mm)
        off_dot = float(self._filt_off_dot)

        # ── 平滑 PD + 延迟预测（替代分段 bang-bang）──
        # vy_des = kp·(off + L·off_dot) + kd·off_dot
        #   kp·off:     比例纠偏，大偏移自然给满速
        #   kd·off_dot: 接近速度阻尼，快到中心自动收油，不冲过再回摆
        #   L·off_dot:  预测补偿检测→执行链路延迟
        kp = float(getattr(cc, "offset_kp_s", 0.0008))
        kd = float(getattr(cc, "offset_kd_s", 0.0003))
        lat_pred = float(getattr(cc, "offset_pred_latency_s", 0.25))
        vmax = float(cc.max_lateral_speed)
        vy_prop = kp * (offset_mm + lat_pred * off_dot)
        vy_damp = kd * off_dot
        if abs_offset <= float(self.dead_zone_mm):
            # 噪声门: 近中心不追检测白噪声（σ~25mm 会把噪声推成真实位移），
            # 只留阻尼项，让接近速度自然停下来
            vy_full = vy_damp
        else:
            vy_full = vy_prop + vy_damp
        vy_full = max(-vmax, min(vmax, vy_full))

        yaw_only = abs_angle > float(getattr(cc, "yaw_only_gate_deg", 20.0))
        if yaw_only:
            # 斜角下图像偏移失真：横移只留小比例，偏航为主（快速摆正，不再空转）
            vy_gate_scale = float(getattr(cc, "vy_in_yaw_gate_scale", 0.25))
            vy_target = vy_gate_scale * vy_full
            self._phase = 1
        else:
            vy_target = vy_full
            self._phase = 2

        yaw_target = -float(getattr(cc, "yaw_kp_deg", 0.010)) * angle_deg
        yaw_target = max(-float(cc.max_yaw_rate), min(float(cc.max_yaw_rate), yaw_target))

        # 角度小死区软衰减：不硬切，只弱化，消除贴边抖振
        if abs_angle <= float(self.dead_zone_deg):
            yaw_target *= 0.4

        if held:
            yaw_target = 0.0
            vy_target = 0.0
            self._cruise_locked = False
            self._cruise_ok_frames = 0

        # 巡航锁：对中且稳定停留 → 锁方向匀速走；锁内保留小增益主动对中
        # 锁心不带 od 门限：连续 8 帧停在带内本身已隐含“接近速度慢”
        lock_ang = float(getattr(cc, "cruise_lock_angle_deg", 10.0))
        lock_off = float(getattr(cc, "cruise_lock_offset_mm", 55.0))
        unlock_ang = float(getattr(cc, "cruise_unlock_angle_deg", 18.0))
        unlock_off = float(getattr(cc, "cruise_unlock_offset_mm", 120.0))
        unlock_od = float(getattr(cc, "cruise_unlock_od_mm_s", 55.0))
        need_ok = int(getattr(cc, "cruise_lock_frames", 10))
        if (not held) and abs_angle <= lock_ang and abs_offset <= lock_off:
            self._cruise_ok_frames += 1
            if self._cruise_ok_frames >= need_ok:
                self._cruise_locked = True
        else:
            self._cruise_ok_frames = 0
        if self._cruise_locked and (
                abs_angle > unlock_ang or abs_offset > unlock_off
                or abs(off_dot) > unlock_od):
            self._cruise_locked = False

        if self._cruise_locked and not held:
            # 作业巡航：小幅增益持续对中（不再开环飘）
            cscale = float(getattr(cc, "cruise_corr_scale", 0.35))
            vy_target *= cscale
            yaw_target *= cscale
            self._phase = 4

        yaw_rate = self._slew(
            self._yaw_cmd, yaw_target, getattr(cc, "yaw_slew_rate", 0.12), dt)
        lat_up = float(getattr(cc, "lateral_slew_rate", 0.15))
        lat_down = float(getattr(cc, "lateral_slew_rate_down", 0.60))
        lat_rate = lat_down if abs(vy_target) < abs(self._vy_cmd) - 1e-6 else lat_up
        lateral_speed = self._slew(self._vy_cmd, vy_target, lat_rate, dt)
        self._yaw_cmd = yaw_rate
        self._vy_cmd = lateral_speed

        vx_cruise = float(getattr(cc, "forward_speed", 0.0) or 0.0)
        vx_cruise = min(vx_cruise, float(getattr(cc, "max_forward_speed", vx_cruise)))

        if held:
            align = 0.0
            vx_des = 0.0
            self._cruise_locked = False
            self._cruise_ok_frames = 0
        elif self._cruise_locked:
            align = 1.0
            vx_des = vx_cruise  # 匀速
        else:
            # 未锁：按对齐比例小步前进（2026-08-13 回搬旧版 firefly，避免偏了完全停住）
            ang_gate = float(getattr(cc, "forward_align_angle_deg", 12.0))
            off_gate = float(getattr(cc, "forward_align_offset_mm", 55.0))
            min_scale = float(getattr(cc, "forward_min_scale", 0.35))
            align = 1.0
            if abs_angle > self.dead_zone_deg:
                align *= max(0.0, 1.0 - (abs_angle - self.dead_zone_deg) /
                             max(ang_gate - self.dead_zone_deg, 1.0))
            if abs_offset > self.dead_zone_mm:
                align *= max(0.0, 1.0 - (abs_offset - self.dead_zone_mm) /
                             max(off_gate - self.dead_zone_mm, 1.0))
            align = max(min_scale, min(1.0, align))
            vx_des = vx_cruise * align

        self._vx_cmd = self._slew(
            self._vx_cmd, vx_des, getattr(cc, "forward_slew_rate", 0.08), dt)

        cmd.yaw_correction = yaw_rate
        cmd.roll_correction = lateral_speed
        cmd.forward_speed = self._vx_cmd
        cmd.forward_align = align
        cmd.active = True

        if confidence < 0.8:
            # 2026-08-13 晚: 低光 conf=0.50 时 62.5% 折扣太狠(指令缩水漂移) → 下限 0.55→0.75
            scale = max(0.75, confidence / 0.8)
            cmd.roll_correction *= scale
            cmd.yaw_correction *= scale
            if not self._cruise_locked:
                cmd.forward_speed *= scale
                self._vx_cmd = cmd.forward_speed
            self._yaw_cmd = cmd.yaw_correction
            self._vy_cmd = cmd.roll_correction

        return cmd

    def idle_decay(self) -> ControlCommand:
        """丢检时把速度指令平滑收到 0，避免飞控继续顶着旧速度。"""
        now = time.perf_counter()
        dt = now - self._last_update_time
        self._last_update_time = now
        cc = self.config.control
        self._yaw_cmd = self._slew(self._yaw_cmd, 0.0, getattr(cc, "yaw_slew_rate", 0.2), dt)
        self._vy_cmd = self._slew(
            self._vy_cmd, 0.0, getattr(cc, "lateral_slew_rate", 0.25), dt)
        self._vx_cmd = self._slew(
            self._vx_cmd, 0.0, getattr(cc, "forward_slew_rate", 0.08), dt)
        self.lateral_pid.reset()
        self.yaw_pid.reset()
        self._phase = 0
        self._cruise_locked = False
        self._cruise_ok_frames = 0
        self._angle_jump_streak = 0
        self._offset_jump_streak = 0
        self._have_prev_offset = False
        self._have_offset = False
        self._filt_off_dot = 0.0
        self._have_off_dot = False
        self._off_buf.clear()
        return ControlCommand(
            roll_correction=self._vy_cmd,
            yaw_correction=self._yaw_cmd,
            forward_speed=self._vx_cmd,
            forward_align=0.0,
            active=True,
            timestamp=now,
        )

    @staticmethod
    def _slew(current: float, target: float, rate: float, dt: float) -> float:
        """限制指令变化率，rate 单位为 单位/秒。"""
        if rate <= 0:
            return target
        dt = max(1e-6, min(dt, 0.1))
        step = rate * dt
        delta = target - current
        if delta > step:
            return current + step
        if delta < -step:
            return current - step
        return target

    def reset(self):
        """重置PID状态 (切换飞行模式时调用)"""
        self.lateral_pid.reset()
        self.yaw_pid.reset()
        self._yaw_cmd = 0.0
        self._vy_cmd = 0.0
        self._vx_cmd = 0.0
        self._filt_angle = 0.0
        self._have_angle = False
        self._angle_jump_streak = 0
        self._filt_offset = 0.0
        self._have_offset = False
        self._offset_jump_streak = 0
        self._phase = 0
        self._cruise_locked = False
        self._cruise_ok_frames = 0
        self._have_prev_offset = False
        self._prev_offset_mm = 0.0
        self._filt_off_dot = 0.0
        self._have_off_dot = False
        self._off_buf.clear()
        self._last_update_time = time.perf_counter()

    # ═══════════════════════════════════════════════════════════
    # 死区设置
    # ═══════════════════════════════════════════════════════════

    def set_dead_zones(self, offset_mm: float, angle_deg: float):
        """设置控制死区"""
        self.dead_zone_mm = offset_mm
        self.dead_zone_deg = angle_deg

    # ═══════════════════════════════════════════════════════════
    # MAVLink 通信 (与ArduPilot/PX4飞控交互)
    # ═══════════════════════════════════════════════════════════

    def _init_mavlink(self):
        """初始化MAVLink连接并请求遥测数据流"""
        try:
            from pymavlink import mavutil
            cc = self.config.control
            self._mav_conn = mavutil.mavlink_connection(
                cc.mavlink_connection,
                baud=cc.mavlink_baud,
            )
            print(f"[MAVLink] 已连接到 {cc.mavlink_connection}")

            # 等待飞控 HEARTBEAT（跳过 GCS/伴飞等非载具心跳）
            print("[MAVLink] 等待飞控心跳...")
            msg = None
            deadline = time.time() + 5.0
            while time.time() < deadline:
                m = self._mav_conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
                if m is not None and self._is_vehicle_heartbeat(m):
                    msg = m
                    break
            if msg is None:
                print("[MAVLink] 超时: 未收到飞控心跳")
                self._mav_conn.close()
                self._mav_conn = None
                return

            self._mav_conn.target_system = msg.get_srcSystem()
            self._mav_conn.target_component = msg.get_srcComponent()
            if self._fc_is_px4():
                mode = self._px4_mode_from_heartbeat(msg)
            else:
                mode = mavutil.mode_string_v10(msg)
            print(f"[MAVLink] 飞控已连接 sysid={self._mav_conn.target_system} mode={mode}")

            # 请求遥测数据流
            self._request_telemetry_streams()

        except ImportError:
            print("[MAVLink] pymavlink未安装, 将使用RC override模式")
            self._mav_conn = None
        except Exception as e:
            print(f"[MAVLink] 连接失败: {e}")
            import traceback
            traceback.print_exc()
            if self._mav_conn is not None:
                self._mav_conn.close()
            self._mav_conn = None

    def _request_telemetry_streams(self):
        """请求飞控发送遥测数据流 (SYS_STATUS, BATTERY_STATUS, GPS等)"""
        if self._mav_conn is None:
            return
        try:
            from pymavlink import mavutil

            # 使用REQUEST_DATA_STREAM请求数据流 (兼容性好)
            # MAV_DATA_STREAM_EXTENDED_STATUS = 2 (包含SYS_STATUS, BATTERY_STATUS, GPS等)
            # MAV_DATA_STREAM_EXTRA1 = 6 (包含GPS)
            # MAV_DATA_STREAM_RAW_SENSORS = 3 (包含电压电流)
            streams = [
                (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),  # 2Hz
                (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 4),      # 测距/IMU
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 2),           # GPS 2Hz
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 4),           # VFR_HUD
                (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2),         # 位置 2Hz
            ]

            for stream_id, rate in streams:
                self._mav_conn.mav.request_data_stream_send(
                    self._mav_conn.target_system,
                    self._mav_conn.target_component,
                    stream_id,
                    rate,
                    1,  # 1=start, 0=stop
                )

            # 单独提高 DISTANCE_SENSOR 频率，便于看激光跳动
            try:
                mid = mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR
                self._mav_conn.mav.command_long_send(
                    self._mav_conn.target_system,
                    self._mav_conn.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mid,
                    100000,  # 10 Hz (us)
                    0, 0, 0, 0, 0,
                )
            except Exception:
                pass

            print(f"[MAVLink] 已请求 {len(streams)} 个遥测数据流 (+DISTANCE_SENSOR)")
        except Exception as e:
            print(f"[MAVLink] 请求数据流失败: {e}")

    def send_mavlink(self, cmd: ControlCommand):
        """
        速度指令: SET_POSITION_TARGET_LOCAL_NED (BODY_NED)。
        ArduPilot=GUIDED 激活后发；PX4=OFFBOARD 激活后发（需持续流）。
        """
        if self._mav_conn is None:
            return
        if not self._fc_armed:
            return  # 安全: 未解锁不发送任何指令
        if not self._guided_active:
            # PX4要求切入OFFBOARD前已经有持续的>2Hz设定点流。
            # 在POSITION等其他模式下只发送零速度；PX4不会执行，但会据此允许
            # 操作者稍后手动切入OFFBOARD。真实视觉指令仍只在OFFBOARD内发送。
            if self._fc_is_px4():
                self._send_body_velocity_raw(0.0, 0.0, 0.0)
            return
        if not cmd.active:
            return

        try:
            from pymavlink import mavutil
            cc = self.config.control

            # type_mask: 告诉飞控哪些字段有效 (bit=1 表示忽略该字段)
            mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
            # 不忽略的: vx, vy, vz, yaw_rate (我们只控制速度和偏航速率)

            self._mav_conn.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self._mav_conn.target_system,
                self._mav_conn.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                mask,
                0, 0, 0,           # x, y, z 位置 (忽略)
                float(getattr(cmd, "forward_speed", cc.forward_speed) or 0.0),  # vx
                cmd.roll_correction, # vy: 横向修正 (正值=右)
                0,                  # vz: 垂直 (0=保持)
                0, 0, 0,            # afx, afy, afz (忽略)
                0,                  # yaw (忽略)
                cmd.yaw_correction, # yaw_rate
            )
        except Exception as e:
            print(f"[MAV] 速度指令发送失败: {e}")

    def send_command(self, cmd: ControlCommand):
        """发送控制指令到飞控"""
        if self.config.control.use_mavlink:
            self.send_mavlink(cmd)

    def _fc_is_px4(self) -> bool:
        return str(getattr(self.config.control, "fc_stack", "ardupilot")).lower() == "px4"

    @staticmethod
    def _is_vehicle_heartbeat(msg) -> bool:
        """只认飞控载具心跳，忽略 GCS / 伴飞 / 云台等。"""
        try:
            from pymavlink import mavutil
            mtype = int(getattr(msg, "type", -1))
            ap = int(getattr(msg, "autopilot", -1))
            skip = {
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_TYPE_GIMBAL,
                mavutil.mavlink.MAV_TYPE_ADSB,
                mavutil.mavlink.MAV_TYPE_CAMERA,
            }
            for name in ("MAV_TYPE_BATTERY", "MAV_TYPE_CHASSIS", "MAV_TYPE_WINCH"):
                if hasattr(mavutil.mavlink, name):
                    skip.add(getattr(mavutil.mavlink, name))
            if mtype in skip:
                return False
            if ap in (
                mavutil.mavlink.MAV_AUTOPILOT_PX4,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            ):
                return True
            # 未知自驾栈：只要不是 INVALID 且类型不像地面站，仍接受
            if ap != mavutil.mavlink.MAV_AUTOPILOT_INVALID and mtype not in skip:
                return True
            return False
        except Exception:
            return True

    @staticmethod
    def _px4_mode_from_heartbeat(msg) -> str:
        """
        pymavlink 的 mode_string_v10 对 PX4 常返回 Mode(0xXXXX)/UNKNOWN。
        按 PX4 custom_mode 主模式解码。
        """
        # PX4 custom main modes (px4_custom_mode.main_mode)
        main_names = {
            1: "MANUAL",
            2: "ALTCTL",
            3: "POSCTL",
            4: "AUTO",
            5: "ACRO",
            6: "OFFBOARD",
            7: "STABILIZED",
            8: "RATTITUDE",
        }
        try:
            custom = int(getattr(msg, "custom_mode", 0) or 0)
            main_mode = (custom >> 16) & 0xFF
            sub_mode = (custom >> 24) & 0xFF
            name = main_names.get(main_mode)
            if name == "AUTO":
                auto_sub = {
                    1: "AUTO.READY",
                    2: "AUTO.TAKEOFF",
                    3: "AUTO.LOITER",
                    4: "AUTO.MISSION",
                    5: "AUTO.RTL",
                    6: "AUTO.LAND",
                    8: "AUTO.FOLLOW",
                    9: "AUTO.PRECLAND",
                }
                return auto_sub.get(sub_mode, "AUTO")
            if name:
                return name
        except Exception:
            pass
        try:
            from pymavlink import mavutil
            s = str(mavutil.mode_string_v10(msg) or "")
            if s and "Mode(" not in s and s.upper() != "UNKNOWN":
                return s
        except Exception:
            pass
        return "UNKNOWN"

    def _send_body_velocity_raw(self, vx: float, vy: float, yaw_rate: float):
        """不检查 armed/guided，用于 PX4 切 OFFBOARD 前灌设定点。"""
        if self._mav_conn is None:
            return
        try:
            from pymavlink import mavutil
            mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
            self._mav_conn.mav.set_position_target_local_ned_send(
                0,
                self._mav_conn.target_system,
                self._mav_conn.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                mask,
                0, 0, 0,
                float(vx), float(vy), 0.0,
                0, 0, 0,
                0, float(yaw_rate),
            )
        except Exception as e:
            print(f"[MAV] setpoint 失败: {e}")

    def _set_mode_name(self, mode_name: str) -> bool:
        if self._mav_conn is None:
            return False
        try:
            mapping = self._mav_conn.mode_mapping()
            if not mapping or mode_name not in mapping:
                # 尝试常见别名
                aliases = {
                    "OFFBOARD": ["OFFBOARD"],
                    "AUTO.LOITER": ["AUTO.LOITER", "LOITER"],
                    "POSITION": ["POSCTL", "POSITION"],
                    "GUIDED": ["GUIDED"],
                    "LOITER": ["LOITER", "AUTO.LOITER"],
                }
                found = None
                for a in aliases.get(mode_name, [mode_name]):
                    if mapping and a in mapping:
                        found = a
                        break
                if found is None:
                    print(f"[MAV] 模式映射无 {mode_name}, available={list(mapping.keys()) if mapping else None}")
                    return False
                mode_name = found
            mode_id = mapping[mode_name]
            self._mav_conn.set_mode(mode_id)
            return True
        except Exception as e:
            print(f"[MAV] set_mode({mode_name}) 失败: {e}")
            return False

    def _switch_to_guided(self):
        """切入外环控制模式：ArduPilot=GUIDED，PX4=OFFBOARD。"""
        if self._mav_conn is None or self._guided_active:
            return
        try:
            if self._fc_is_px4():
                # PX4 要求先持续发设定点再切 OFFBOARD
                for _ in range(8):
                    self._send_body_velocity_raw(0.0, 0.0, 0.0)
                    time.sleep(0.05)
                ok = self._set_mode_name("OFFBOARD")
                if not ok:
                    return
                self._guided_active = True
                self._loss_time = 0.0
                print("[OFFBOARD] 已切换到 PX4 OFFBOARD")
            else:
                from pymavlink import mavutil
                # ArduPilot custom_mode: 4=GUIDED
                self._mav_conn.mav.command_long_send(
                    self._mav_conn.target_system,
                    self._mav_conn.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    4,  # ArduPilot GUIDED = 4
                    0, 0, 0, 0, 0,
                )
                self._guided_active = True
                self._loss_time = 0.0
                print("[GUIDED] 已切换到GUIDED模式")
        except Exception as e:
            print(f"[MODE] 切换失败: {e}")

    def _switch_to_loiter(self):
        """安全回退：ArduPilot=LOITER，PX4=POSITION。"""
        if self._mav_conn is None or not self._guided_active:
            return
        try:
            if self._fc_is_px4():
                if self._set_mode_name("POSITION"):
                    self._guided_active = False
                    print("[SAFETY] 不安全/丢线，回退 POSITION 悬停")
            else:
                from pymavlink import mavutil
                self._mav_conn.mav.command_long_send(
                    self._mav_conn.target_system,
                    self._mav_conn.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    5,  # ArduPilot LOITER = 5
                    0, 0, 0, 0, 0,
                )
                self._guided_active = False
                print("[SAFETY] 不安全/丢线，回退 LOITER 悬停")
        except Exception as e:
            print(f"[MODE] 回退失败: {e}")

    def check_mode_transition(self, fc_mode: str, fc_armed: bool, line_detected: bool,
                               frame_dt: float = 0.033):
        """
        每次主循环调用，检查是否需要切换模式。

        ArduPilot: LOITER→GUIDED；不安全/丢线→LOITER
        PX4: HOLD/POSCTL/LOITER→OFFBOARD；不安全(含hold)/丢线→POSITION 悬停

        line_detected: 仅真水槽跟踪为 True；hold/丢检应传 False。
        """
        if self._mav_conn is None:
            return

        # telem 可能只有 SYS_STATUS 等、没有 HEARTBEAT：不能把 mode/armed 当成 UNKNOWN/False
        mode_raw = fc_mode.upper() if isinstance(fc_mode, str) else str(fc_mode).upper()
        if mode_raw in ("UNKNOWN", "N/A", "", "NONE"):
            mode_upper = self._last_fc_mode.upper()
        else:
            self._last_fc_mode = str(fc_mode)
            mode_upper = mode_raw
            self._fc_armed = bool(fc_armed)

        px4 = self._fc_is_px4()
        ctrl_mode = "OFFBOARD" if px4 else "GUIDED"
        idle_hint = ("LOITER", "AUTO.LOITER", "HOLD", "POSCTL", "POSITION") if px4 else ("LOITER",)

        if not self._guided_active:
            in_idle = any(h in mode_upper for h in idle_hint)
            if self._fc_armed and in_idle and line_detected:
                if self.config.control.auto_guided:
                    self._switch_to_guided()
            elif self._fc_armed and ctrl_mode in mode_upper:
                self._guided_active = True
                self._loss_time = 0.0
                print(f"[{ctrl_mode}] 检测到已在{ctrl_mode}模式，激活控制")

        if self._guided_active:
            # 只在拿到明确非控制模式时才清状态；UNKNOWN 不打断控速
            if mode_upper not in ("UNKNOWN", "N/A", "", "NONE") and ctrl_mode not in mode_upper:
                print(f"[{ctrl_mode}] 警告: 飞控实际模式={mode_upper}，重置状态")
                self._guided_active = False
                self._loss_time = 0.0
                return

            # 不安全(hold/丢检)计时 → 超时回 POSITION/LOITER 悬停
            if not line_detected:
                self._loss_time += frame_dt
                if self._loss_time > self.config.control.safety_loss_timeout:
                    self._switch_to_loiter()
            else:
                self._loss_time = 0.0

            # PX4: 即使空闲也尽量保持设定点流（idle_decay 已 active）
            if px4 and self._fc_armed:
                pass  # 主循环 send_command 负责

    def close(self):
        """关闭MAVLink连接"""
        if self._mav_conn is not None:
            self._mav_conn.close()
            self._mav_conn = None

    # ═══════════════════════════════════════════════════════════
    # MAVLink 遥测读取
    # ═══════════════════════════════════════════════════════════

    def read_telemetry(self) -> dict:
        """
        非阻塞读取MAVLink消息，返回最新遥测数据

        返回 dict 包含:
          fc_connected, fc_mode, fc_armed,
          battery_voltage, battery_current, battery_remaining,
          heartbeat_age,
          altitude, groundspeed, heading,
          gps_fix, gps_lat, gps_lon, satellites
        """
        result = {}
        if self._mav_conn is None:
            return result

        try:
            # 每帧限制处理数量；921600波特率下消息持续到达时，
            # “一直读到空”可能永远不退出并饿死相机/检测主循环。
            max_messages = 100
            message_count = 0
            msg = self._mav_conn.recv_match(blocking=False)
            while msg is not None and message_count < max_messages:
                message_count += 1
                t = msg.get_type()

                if t == 'HEARTBEAT':
                    from pymavlink import mavutil
                    # 忽略 GCS/伴飞心跳，避免把模式刷成 UNKNOWN、解锁状态被清掉
                    src = msg.get_srcSystem()
                    tgt = int(getattr(self._mav_conn, "target_system", 0) or 0)
                    accept_hb = self._is_vehicle_heartbeat(msg) and (
                        not tgt or not src or src == tgt
                    )
                    if accept_hb:
                        if self._fc_is_px4():
                            result['fc_mode'] = self._px4_mode_from_heartbeat(msg)
                        else:
                            mode_obj = mavutil.mode_string_v10(msg)
                            result['fc_mode'] = str(mode_obj) if mode_obj else 'UNKNOWN'
                        result['fc_armed'] = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )
                        result['fc_connected'] = True
                        result['heartbeat_age'] = 0.0
                        # 手动切到 OFFBOARD 时立即激活，避免一直发零速
                        if (self._fc_is_px4()
                                and result['fc_armed']
                                and 'OFFBOARD' in str(result['fc_mode']).upper()
                                and not self._guided_active):
                            self._guided_active = True
                            self._loss_time = 0.0
                            print("[OFFBOARD] 检测到已在OFFBOARD，激活视觉控制")

                elif t == 'SYS_STATUS':
                    # 电池电压 (mV → V), 电流 (10mA → A)
                    result['battery_voltage'] = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else 0.0
                    result['battery_current'] = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
                    result['battery_remaining'] = msg.battery_remaining if msg.battery_remaining != -1 else 0

                elif t == 'BATTERY_STATUS':
                    # 更详细的电池信息 (优先于SYS_STATUS)
                    if 'battery_voltage' not in result:
                        result['battery_voltage'] = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0.0
                    result['battery_remaining'] = msg.battery_remaining if msg.battery_remaining != -1 else result.get('battery_remaining', 0)

                elif t == 'GLOBAL_POSITION_INT':
                    result['altitude'] = msg.relative_alt / 1000.0  # mm → m
                    result['heading'] = msg.hdg // 100  # centidegrees → degrees

                elif t == 'DISTANCE_SENSOR':
                    # current_distance: cm；无效时常为 0 或超出量程
                    dist_cm = float(getattr(msg, 'current_distance', 0) or 0)
                    if dist_cm > 1.0:
                        result['rng_alt'] = dist_cm / 100.0
                    q = getattr(msg, 'signal_quality', None)
                    if q is None:
                        q = getattr(msg, 'covariance', -1)
                    try:
                        qi = int(q)
                        # covariance 有时是 cm^2；signal_quality 0-100
                        if 0 <= qi <= 100:
                            result['rng_quality'] = qi
                    except Exception:
                        pass

                elif t == 'RANGEFINDER':
                    d = float(getattr(msg, 'distance', 0) or 0)
                    if d > 0.01:
                        result['rng_alt'] = d

                elif t == 'GPS_RAW_INT':
                    result['gps_fix'] = msg.fix_type
                    result['gps_lat'] = msg.lat / 1e7
                    result['gps_lon'] = msg.lon / 1e7
                    result['satellites'] = msg.satellites_visible

                elif t == 'VFR_HUD':
                    result['groundspeed'] = msg.groundspeed
                    if 'heading' not in result:
                        result['heading'] = int(msg.heading)
                    # 气压/融合高度补充（无 GLOBAL 时）
                    if 'altitude' not in result and getattr(msg, 'alt', None) is not None:
                        result['altitude'] = float(msg.alt)

                msg = self._mav_conn.recv_match(blocking=False)

        except Exception as e:
            import traceback
            print(f"[Telemetry] 读取异常: {e}")
            traceback.print_exc()

        return result

    def update_heartbeat_age(self, dt: float):
        """累加心跳年龄 (在主循环中每帧调用)"""
        if self._mav_conn is not None:
            # heartbeat_age由外部SharedState维护
            pass
