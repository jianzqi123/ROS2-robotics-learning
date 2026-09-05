#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮子打滑检测 —— 用 /scan 做独立于轮子的运动测量。

用法:
    # 终端1
    ros2 launch my_robot_description gazebo.launch.py headless:=true
    # 终端2  只监视(可与 nav2 同时跑)
    python3 ~/ros2_ws/src/my_robot_description/scripts/slip_monitor.py
    # 终端2  自检: 自己开向 box1 撞上去, 看检测器是否在该响的时候响
    python3 ~/ros2_ws/src/my_robot_description/scripts/slip_monitor.py --verify

为什么不能比较"指令速度"和"/joint_states 关节速度":
    gz-sim-diff-drive-system 是速度控制器, 轮子不管有没有抓地都会转到
    指令转速。打滑时这两个量恰恰是完全吻合的 —— 拿两个一致的量互相比,
    什么也检测不出来。README 早就记录过同一件事的另一面:
    "odom 内部自洽不能证明任何东西, 因为轮子达到指令转速与是否有牵引力无关"。
    所以判据必须引入一个不经过轮子的运动测量。

判据怎么来的(不是试出来的):
    车平移 d 米, 角度 θ 处的光束打在远处平面上, 距离变化约 -d*cos(θ-θ0)。
    绕整圈对 |变化| 取均值 = (2/pi)*d ≈ 0.637*d。
    实测变化远低于这个预期 => 轮子在转而车没动 => 打滑。

    注意 0.637*d 是下界而非紧预测: 真实房间里光束会扫过物体边缘,
    一点点平移就能让某条光束的距离跳变好几米。实测自由行驶时的比值是
    2.4~11.5(不是 1.0), 而顶住障碍时是 0.00。判据只用"远低于"这一侧,
    所以下界性质正好合适 —— 最近的一次正常读数比值 2.44,
    离 0.30 的门槛还有 8 倍余量。

    下限由传感器决定: URDF 里 <range><resolution>0.01</resolution>,
    低于 1cm 的变化和量化噪声分不开, 所以位移门槛取 0.05m ——
    对应预期扫描变化 3.2cm, 有 3 倍以上的量化余量。

局限(明写在这里, 不要当成通用方案):
    只检测平移打滑。纯旋转时扫描的变化取决于环境的角向结构, 没有
    上面那样干净的解析预期, 所以旋转分量大时本检测器主动弃权。
    被记录过的那次故障(车楔在 table1 底下、odom 漂出房间 9m)正是平移型。
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# 判据常数, 全部有出处(见模块 docstring)
WINDOW_S = 1.0          # 比较当前扫描与多久之前的扫描
MIN_TRANS = 0.05        # 位移门槛 m: 低于此不判定(预期扫描变化仅 3.2cm)
MAX_ROT = 0.05          # 窗口内偏航变化超过此值(rad)则弃权, 见"局限"
TRANS_TO_SCAN = 2.0 / math.pi   # 0.637, 平移 d 对应的平均 |距离变化|
SLIP_RATIO = 0.30       # 实测/预期 低于此判为打滑
CONFIRM_N = 3           # 连续 N 次才报警, 避开单帧抖动

# 自检用: box1 西面在 x=2.5, 车身半长 0.2, 所以顶住时车心应停在 x=2.3。
# 这是个推出来的期望值, 不是阈值 —— 判据用它才能真正验证"车停住了",
# 而不是"车走得比某个随手定的数少"。
BOX1_CONTACT_X = 2.5 - 0.2
CONTACT_TOL = 0.15


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class SlipMonitor(Node):

    def __init__(self, verbose=True):
        super().__init__('slip_monitor')
        self.verbose = verbose
        self.odom = None                 # (t, x, y, yaw)
        self.hist = []                   # [(t, ranges, odom), ...]
        self.streak = 0
        self.events = []                 # 报警记录 (t, d_odom, 实测, 预期)
        self.last_eval = 0.0

        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        self.odom = (t, p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_scan(self, msg):
        if self.odom is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.hist.append((t, list(msg.ranges), self.odom))
        # 只保留窗口两倍长度的历史
        self.hist = [h for h in self.hist if t - h[0] <= WINDOW_S * 2.5]
        if t - self.last_eval >= 0.5:
            self.last_eval = t
            self._evaluate(t)

    def _evaluate(self, now):
        # 找窗口起点: 最接近 now-WINDOW_S 的那一帧
        past = None
        for h in self.hist:
            if now - h[0] >= WINDOW_S:
                past = h
        if past is None:
            return
        t0, r0, o0 = past
        t1, r1, o1 = self.hist[-1]

        d_odom = math.hypot(o1[1] - o0[1], o1[2] - o0[2])
        d_yaw = abs(wrap_pi(o1[3] - o0[3]))

        if d_odom < MIN_TRANS:
            self.streak = 0
            return
        if d_yaw > MAX_ROT:            # 旋转分量太大, 本检测器弃权
            self.streak = 0
            return

        # 两帧都有限的光束才可比 —— inf 表示没打到东西, 参与平均会污染结果
        pairs = [(a, b) for a, b in zip(r0, r1)
                 if math.isfinite(a) and math.isfinite(b)]
        if len(pairs) < 30:
            self.streak = 0
            return
        measured = sum(abs(b - a) for a, b in pairs) / len(pairs)
        expected = TRANS_TO_SCAN * d_odom
        ratio = measured / expected if expected > 0 else 1.0

        if ratio < SLIP_RATIO:
            self.streak += 1
            if self.streak >= CONFIRM_N:
                self.events.append((t1, d_odom, measured, expected))
                if self.verbose:
                    print(f'  [打滑] t={t1:.1f}  odom 位移 {d_odom:.3f} m  '
                          f'扫描变化 {measured * 100:.1f} cm  '
                          f'预期 {expected * 100:.1f} cm  '
                          f'比值 {ratio:.2f}', flush=True)
        else:
            self.streak = 0
            if self.verbose:
                print(f'  正常  odom {d_odom:.3f} m  扫描 {measured * 100:5.1f} cm'
                      f'  预期 {expected * 100:5.1f} cm  比值 {ratio:.2f}',
                      flush=True)

    def drive(self, vx, seconds):
        tw = Twist()
        tw.linear.x = vx
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())


def ground_truth(retries=3):
    """仅用于自检时确认车是真的停住了; 检测器本身不依赖它。取不到返回 None。"""
    import re
    import subprocess
    for _ in range(retries):
        try:
            out = subprocess.run(['gz', 'model', '-m', 'my_robot', '-p'],
                                 capture_output=True, text=True, timeout=25).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        n = re.findall(r'\[\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s*\]', out)
        if len(n) >= 2:
            return (float(n[0][0]), float(n[0][1]))
        time.sleep(1.0)
    return None


def verify(node):
    """
    自检: 从原点直行撞上 box1(占 x[2.5,3.5] y[-0.5,0.5]), 然后继续给指令。
    车身半长 0.2, 所以约 2.3m 处顶住, 之后轮子空转 —— 检测器该在此时报警。
    判据要两头都成立: 撞前不能误报, 撞后必须报。
    """
    print('=' * 66)
    print('自检: 直行撞 box1, 撞前不应报警, 撞后应报警')
    print('=' * 66)
    gt0 = ground_truth()
    print(f'  起点真值 {gt0}')

    print('  --- 阶段1: 自由行驶 6 s (预期: 无报警) ---')
    node.drive(0.2, 6.0)
    free_events = len(node.events)
    gt1 = ground_truth()
    print(f'  阶段1 报警 {free_events} 次, 真值 {gt1}')

    print('  --- 阶段2: 继续直行 14 s 撞上 box1 (预期: 报警) ---')
    node.drive(0.2, 14.0)
    node.drive(0.0, 1.0)
    gt2 = ground_truth()
    stuck_events = len(node.events) - free_events
    print(f'  阶段2 报警 {stuck_events} 次, 真值 {gt2}')

    # 真值拿不到时必须判为"无法确认", 不能让哨兵值混进数值比较 ——
    # 早期版本写成 moved = -1 再判 moved < 0.5, 于是取不到真值反而必过,
    # 那是个不可能失败的检查。
    have_gt = gt1 is not None and gt2 is not None
    moved = math.hypot(gt2[0] - gt1[0], gt2[1] - gt1[1]) if have_gt else None
    print()
    if have_gt:
        # 判据不是"位移小于某个数" —— 车在撞上之前本来就该正常走一段。
        # 真正要确认的是它最后停在了 box1 的接触点上。
        dx = abs(gt2[0] - BOX1_CONTACT_X)
        print(f'  阶段2 真实位移 {moved:.3f} m  (指令位移 0.2*14=2.8 m)')
        print(f'  最终 x={gt2[0]:.4f}, 预期接触点 {BOX1_CONTACT_X:.2f}, '
              f'差 {dx * 1000:.1f} mm')
        stopped = dx < CONTACT_TOL
    else:
        print('  阶段2 真实位移: 取不到 Gazebo 真值 —— 本项无法确认')
        stopped = False
    print(f'  阶段1 误报: {"无" if free_events == 0 else f"{free_events} 次 —— 不合格"}')
    print(f'  阶段2 检出: {"有" if stuck_events > 0 else "无 —— 漏报"}')
    print(f'  停在接触点: {"是" if stopped else ("否" if have_gt else "无法确认")}')
    ok = free_events == 0 and stuck_events > 0 and stopped
    if not ok and free_events == 0 and stuck_events > 0 and not have_gt:
        print('  判定: 无法确认 (检测器行为正确, 但没有独立证据证明车真的停了)')
        return False
    print(f'  判定: {"通过" if ok else "未通过"}')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--verify', action='store_true', help='自己开去撞墙做自检')
    ap.add_argument('--quiet', action='store_true', help='只打印报警')
    args = ap.parse_args()

    rclpy.init()
    node = SlipMonitor(verbose=not args.quiet)
    ok = True
    try:
        if args.verify:
            # 等第一帧扫描和里程计
            end = time.time() + 15
            while node.odom is None and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            ok = verify(node)
        else:
            print('监视中 (Ctrl-C 退出) ...')
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
