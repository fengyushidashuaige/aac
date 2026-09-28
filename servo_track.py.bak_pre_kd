#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
视觉伺服循迹节点 (半自动) —— 跟着画面中心偏差飞

人负责: 起飞 / 高度 / 降落 / 应急接管
系统负责: 横向对齐 + 航向对齐 (让水槽保持在画面中心)

★ 不占串口: 只发 ROS 话题 -> mavros -> 飞控
★ 不需要路径, 不需要世界系对齐 (偏差是相对机体的)
"""
import math, socket, json, time, threading, os
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool, SetBoolResponse

UDP_PORT = 45679

KP_OFF    = 0.0004      # 每 mm 偏移 -> m/s 横移
KP_ANG    = 0.012       # 每度角   -> rad/s 偏航
MAX_VY    = 0.06
MAX_YAW   = 0.12
CRUISE    = 0.05
MIN_CONF  = 0.40     # 2026-09-21: 0.50 -> 0.40
                     # 换了微调后的 v2 模型, 它的 conf 整体比旧模型低
                     # (旧模型最高分均值 0.78, 新模型约 0.5)。门限停在 0.50 会让
                     # 大量帧卡在边缘反复横跳 => 伺服一顿一顿。留余量降到 0.40。
                     # 注意: 旧模型时代的 0.50 是基于"真槽0.87/误检0.35", 那套
                     # 数据已经过时了, 要用新的实测分布重定。
                     # === 2026-09-20: 0.7 -> 0.50
                     # 依据 20260920_151629 实测飞行数据(3100帧):
                     #   conf 中位 0.51; 而且【槽越接近画面中心, conf 越低】——
                     #   中心附近(|off|<20mm) 中位 0.50, 偏 100~500mm 反而 0.68~0.72。
                     #   门限 0.7 会在"快对准"时把帧全拒掉, 形成
                     #   接近→conf掉→被拒→漂开→conf升→再修 的振荡 => 锁不住。
                     #   取 0.50 这个天然分界: 放过中心帧(0.50), 仍挡住误检(0.35)。
                     #   !! 不要低于 0.50, 否则会把 0.35 那档误检一起放进来。
OFF_MAX   = 250.0       # 偏差物理上限 mm (相机横向视野); 超了必是坏 mask
ANG_MAX   = 45.0        # 倾角物理上限 deg
KD_OFF    = 0.0001     # 微分增益 (m/s per mm/s), 抑制过冲
DEAD_MM   = 25.0        # 实测噪声 sigma~5mm, 取 ~2sigma (v1 用 30 太大了)
DEAD_DEG  = 1.5
SLEW_VY   = 0.15
SLEW_YAW  = 0.30


class Servo(object):
    def __init__(self):
        self.rate_hz = rospy.get_param('~rate', 20.0)
        self.enable = False
        self.off = self.ang = self.conf = 0.0
        self.pkt_valid = True     # 2026-09-22: geometry validity from detector
        self.detected = False; self.held = False; self.t_pkt = 0.0
        self.vy_cmd = self.yaw_cmd = 0.0
        self.last_t = time.time(); self.last_print = 0.0

        # ---- 分档测试开关 (照老版本 single-line-working 的做法) ----
        self.yaw_only    = bool(rospy.get_param('~yaw_only', False))
        self.center_only = bool(rospy.get_param('~center_only', False))
        self.forward     = bool(rospy.get_param('~forward', False))
        # ---- 符号 (FRD->FLU 迁移可能导致反, 做成参数, 反了就改这里) ----
        self.yaw_sign = float(rospy.get_param('~yaw_sign', +1.0))
        self.vy_sign  = float(rospy.get_param('~vy_sign', -1.0))
        self.use_psi  = bool(rospy.get_param('~use_psi', True))
        self.kp_ang   = float(rospy.get_param('~kp_ang', KP_ANG))
        self.cruise   = float(rospy.get_param('~cruise', CRUISE))
        self.kp_off   = float(rospy.get_param('~kp_off', KP_OFF))
        self.kd_off   = float(rospy.get_param('~kd_off', KD_OFF))
        _lhz = float(rospy.get_param('~log_hz', 10.0))
        self.log_dt = 1.0 / max(_lhz, 1.0)
        self.off_prev = None
        self.off_dot  = 0.0
        self.dead_mm  = float(rospy.get_param('~dead_mm', DEAD_MM))
        self.max_vy   = float(rospy.get_param('~max_vy', MAX_VY))
        self.slew_vy  = float(rospy.get_param('~slew_vy', SLEW_VY))
        rospy.logwarn('[servo] 增益: kp_off=%.5f kd_off=%.5f dead=%.0fmm max_vy=%.3f slew=%.2f cruise=%.3f kp_ang=%.4f' % (
            self.kp_off, self.kd_off, self.dead_mm, self.max_vy, self.slew_vy, self.cruise, self.kp_ang))

        self.hgt = 0.4
        self.psi = None
        self.cal = None
        try:
            _cp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ang_cal.json')
            if os.path.exists(_cp):
                self.cal = json.load(open(_cp))
        except Exception as _e:
            rospy.logwarn('[servo] ang_cal.json 读取失败: %s' % _e)
        rospy.Subscriber('/Odometry', Odometry, self.on_odom, queue_size=10)

        self.pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
                                   TwistStamped, queue_size=1)
        rospy.Service('~set_enable', SetBool, self.on_enable)
        threading.Thread(target=self.udp_loop).start()
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.control)

    def on_enable(self, req):
        self.enable = bool(req.data)
        rospy.logwarn('[servo] enable = %s' % self.enable)
        return SetBoolResponse(success=True, message='enable=%s' % self.enable)

    def on_odom(self, m):
        self.hgt = m.pose.pose.position.z

    def udp_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(('127.0.0.1', UDP_PORT)); s.settimeout(1.0)
        except Exception as e:
            rospy.logerr('[servo] UDP %d 绑定失败: %s' % (UDP_PORT, e)); return
        while not rospy.is_shutdown():
            try:
                d, _ = s.recvfrom(1024)
                j = json.loads(d.decode('utf-8'))
                self.off = float(j.get('offset_mm', 0.0))
                self.ang = float(j.get('angle_deg', 0.0))
                self.conf = float(j.get('conf', 0.0))
                self.detected = bool(j.get('detected', False))
                self.pkt_valid = bool(j.get('valid', True))
                self.held = bool(j.get('held', False))
                self.t_pkt = time.time()
            except socket.timeout:
                pass
            except Exception:
                pass

    @staticmethod
    def _slew(cur, tgt, rate, dt):
        d = tgt - cur; m = rate * dt
        return cur + max(-m, min(m, d))

    def publish(self, vx, vy, yaw):
        t = TwistStamped()
        t.header.stamp = rospy.Time.now(); t.header.frame_id = 'base_link'
        t.twist.linear.x = vx; t.twist.linear.y = vy; t.twist.angular.z = yaw
        self.pub.publish(t)

    def control(self, ev):
        now = time.time(); dt = max(now - self.last_t, 1e-3); self.last_t = now

        if not self.enable:                       # 没使能 -> 零速, 但保持发流
            self.vy_cmd = self.yaw_cmd = 0.0
            self.publish(0.0, 0.0, 0.0)
            return

        fresh = (now - self.t_pkt) < 0.5
        valid = (fresh and self.detected and (not self.held)
                 and self.pkt_valid and self.conf >= MIN_CONF)

        if not valid:                             # 丢检 -> 衰减到停
            self.vy_cmd  = self._slew(self.vy_cmd,  0.0, SLEW_VY * 2, dt)
            self.yaw_cmd = self._slew(self.yaw_cmd, 0.0, SLEW_YAW * 2, dt)
            self.publish(0.0, self.vy_cmd, self.yaw_cmd)
            rospy.logwarn_throttle(2.0, '[servo] 丢检 conf=%.2f held=%s fresh=%s'
                                   % (self.conf, self.held, fresh))
            return

        off, ang = self.off, self.ang
        if abs(off) > OFF_MAX or abs(ang) > ANG_MAX:
            self.publish(0.0, 0.0, 0.0)
            rospy.logwarn_throttle(2.0, '[servo] 超物理上限 off=%.0f ang=%.1f -> 停' % (off, ang))
            return
        # ---- 航向误差: 优先用标定后的 psi(已扣掉偏差项和模型偏置) ----
        self.psi = None
        if self.use_psi and self.cal is not None:
            A = float(self.cal['A']); B = float(self.cal['B']); C = float(self.cal['C'])
            if abs(A) > 1e-6:
                h = max(self.hgt, 0.10)
                d = off / 1000.0
                self.psi = math.degrees(math.atan(
                    (math.tan(math.radians(ang)) - B * (d / h) - C) / A))
        head = self.psi if self.psi is not None else ang
        if abs(head) < DEAD_DEG:
            head = 0.0

        # D 项: 偏差变化率(低通后), 给横向环加阻尼 —— 抑制'打满->过冲->反向'的极限环
        _af = 1.0 - math.exp(-dt / 0.5)
        if self.off_prev is None:
            self.off_prev = off
        _d = (off - self.off_prev) / dt
        self.off_prev = off
        self.off_dot += _af * (_d - self.off_dot)
        _vy_pd = self.kp_off * off + self.kd_off * self.off_dot
        vy_t  = 0.0 if abs(off) <= self.dead_mm else _vy_pd * self.vy_sign
        yaw_t = 0.0 if head == 0.0 else self.kp_ang * head * self.yaw_sign
        vy_t  = max(-self.max_vy, min(self.max_vy, vy_t))
        yaw_t = max(-MAX_YAW, min(MAX_YAW, yaw_t))

        self.vy_cmd  = self._slew(self.vy_cmd,  vy_t,  self.slew_vy,  dt)
        self.yaw_cmd = self._slew(self.yaw_cmd, yaw_t, SLEW_YAW, dt)

        if self.yaw_only:
            self.vy_cmd = 0.0; vx = 0.0
        elif self.center_only or not self.forward:
            vx = 0.0
        else:
            # 只用偏差减速(不用角度): v2 模型下 ang 常年偏大, 用它会永远卡在最低速
            vx = self.cruise * max(0.5, 1.0 - min(1.0, abs(off) / 250.0))
        self.publish(vx, self.vy_cmd, self.yaw_cmd)

        if now - self.last_print > self.log_dt:
            self.last_print = now
            _ps = ('%+6.1f' % self.psi) if self.psi is not None else '  --  '
            rospy.loginfo('[servo] off=%+7.1fmm ang=%+6.1f psi=%s conf=%.2f  vx=%.3f vy=%+.3f yaw=%+.3f kpa=%.3f'
                          % (off, ang, _ps, self.conf, vx, self.vy_cmd, self.yaw_cmd, self.kp_ang))


if __name__ == '__main__':
    rospy.init_node('servo_track')
    rospy.logwarn('=== servo_track 视觉伺服启动 (走 mavros, 不占串口) ===')
    Servo()
    rospy.spin()
