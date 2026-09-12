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
    # 终端2  监视 + 检出即中止导航并恢复(需要 nav2 在跑)
    python3 ~/ros2_ws/src/my_robot_description/scripts/slip_monitor.py --abort
    # 终端2  验证上面那条链路
    python3 ~/ros2_ws/src/my_robot_description/scripts/slip_monitor.py --verify-abort

检出之后做三件事(--abort):
    1. 取消 Nav2 目标 —— 继续走只会把 AMCL 一起带偏
    2. 在 /slip_detected 上发 true —— 中止只是停车, 这条消息才是
       "定位现在不可信"那个信息本身
    3. 以打滑开始之前的位姿重新播种 AMCL, 然后倒车 0.3m
       (倒车既脱困, 又提供 AMCL 更新所需的位移: update_min_d 是 0.25m,
        车不动 AMCL 根本不更新, 所以"只播种不动"完全没有效果)

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
import re
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

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

# --abort 用: 确认打滑后取消 Nav2 目标, 冷却期内不重复取消。
# 取消后车会停下, odom 不再前进, 检测器自然不再触发, 所以冷却主要是
# 防止取消请求在动作真正终止前被连发。
ABORT_COOLDOWN_S = 5.0

# 中止后的恢复动作。
# 为什么必须包含运动: AMCL 的 update_min_d 是 0.25m —— 车不动它根本不更新,
# 所以"只把位姿重新播种一下"在停着的车上完全没有效果。倒车既能脱困,
# 又给了 AMCL 重新收敛所需的位移。倒车在这台车上安全, 因为雷达是 360 度的。
BACKUP_DIST = 0.30       # m, 略大于 update_min_d, 保证 AMCL 至少更新一次
BACKUP_SPEED = 0.10      # m/s, 与 DWB 时期的倒车限幅一致
BACKUP_TIMEOUT_S = 8.0

# 取消是异步的: 请求发出后控制器还会再发几百毫秒的 /cmd_vel。
# 如果这时监视器已经开始发倒车指令, 两个发布者同时写同一个话题,
# 谁生效不确定。等一下再动。
ABORT_GRACE_S = 1.5

# 用打滑开始之前的那个 AMCL 位姿去播种, 不能用当前的。
# 这一点是实测撞出来的: 第一版拿当前估计播种, 结果播种后 AMCL 与真值
# 差 0.574m —— 打滑期间 odom 虚进了 0.22m, AMCL 的运动模型跟着漂,
# 拿这个已经错了的估计重新声明一遍, 错误原样保留, 放大方差也救不回来。
# 正确的依据藏在打滑的定义里: 打滑期间车没有移动, 所以打滑开始之前的
# 位姿就是当前的真实位姿 —— 这不是近似, 是精确的。
# 回溯多久: 确认需要 CONFIRM_N=3 次连续判定(每 0.5s 一次) 加上 1s 的
# 比较窗口, 所以打滑大约始于确认前 2.5s, 取 3.0s 留余量。
PRESLIP_LOOKBACK_S = 3.0

# 重新播种时给的不确定度。
# 播种不是"我知道我在哪", 而是"我大概在这附近, 请重新收敛"。
RESEED_XY_VAR = 0.25     # = 0.5m 标准差
RESEED_YAW_VAR = 0.07    # ≈ 15 度标准差

# --verify-abort 用的隐形障碍物。
#   高 0.12m: 光束高度 h(r) = 0.1822 - r*tan(0.337 度) = 0.1822 - 0.00588r,
#   要降到 0.12 需要 r = 10.6m, 而房间对角线才 12.8m 且这条航线只有 2.5m
#   —— 也就是说这个物体在整条路上都在光束之下, /scan 完全看不见它,
#   代价地图上不存在, 规划器会直接穿过去。
#   但它挡得住车: 轮子半径 0.05 爬不上 0.12 的台阶, 底盘前面板占
#   z∈[0.05,0.15], 必然撞上。
# 这正是本项目反复讲的"2D 地图是一个切片"的最坏情况:
# 传感器看不见的障碍物, 只有打滑检测能发现。
LOW_OBSTACLE_POSE = (0.0, 1.3)
LOW_OBSTACLE_SIZE = (0.4, 0.4, 0.12)
ABORT_GOAL = (0.0, 2.5)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class SlipMonitor(Node):

    def __init__(self, verbose=True, abort=False):
        super().__init__('slip_monitor')
        self.verbose = verbose
        self.abort_enabled = abort
        self.odom = None                 # (t, x, y, yaw)
        self.hist = []                   # [(t, ranges, odom), ...]
        self.streak = 0
        self.events = []                 # 报警记录 (t, d_odom, 实测, 预期)
        self.last_eval = 0.0
        self.aborts = 0
        self.last_abort = 0.0
        self.amcl_pose = None            # (x, y, z, w) 位置 + 四元数 z/w
        self.amcl_hist = []              # [(t, pose)], 固定回溯的兜底
        self.slip_start_pose = None      # streak 由 0 变 1 那一帧的 AMCL 位姿
        self.recovery = None             # None | ('backup', 起点 odom, 起始时刻)
        self.reseeds = 0

        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # goal_id 与 stamp 全零 = 取消该动作服务器上的所有目标,
        # 所以不需要持有目标句柄, 监视器可以独立于发目标的那个进程运行。
        self.cancel_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # AMCL 的 /amcl_pose 是 transient_local, 订阅端 QoS 必须匹配
        amcl_qos = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, amcl_qos)
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # 让下游知道"定位现在不可信"。中止只是停车, 这个话题才是那条信息。
        self.status_pub = self.create_publisher(Bool, '/slip_detected', 10)
        # 恢复动作用定时器驱动, 不能写在订阅回调里 —— 在回调里跑阻塞的
        # 行驶循环会把执行器卡住, 连 /scan 都收不到, 检测器自己就瞎了。
        self.create_timer(0.05, self._recovery_tick)

    def _on_amcl(self, msg):
        p = msg.pose.pose
        self.amcl_pose = (p.position.x, p.position.y,
                          p.orientation.z, p.orientation.w)
        now = time.time()
        self.amcl_hist.append((now, self.amcl_pose))
        self.amcl_hist = [h for h in self.amcl_hist if now - h[0] <= 15.0]

    def _preslip_pose(self):
        """
        取打滑开始时的 AMCL 位姿。

        首选 streak 起点那一帧记下的位姿 —— 那是车最后一次被判定为
        正常行驶时的估计, 精确对应"打滑开始的瞬间"。
        取不到时才退回固定回溯; 固定回溯会连打滑之前的真实位移一起丢掉
        (实测丢了 0.24m), 所以只作兜底。
        """
        if self.slip_start_pose is not None:
            return self.slip_start_pose, 'streak 起点'
        if not self.amcl_hist:
            return None, None
        cutoff = time.time() - PRESLIP_LOOKBACK_S
        old = [h for h in self.amcl_hist if h[0] <= cutoff]
        if old:
            return old[-1][1], f'{PRESLIP_LOOKBACK_S:.0f}s 前(兜底)'
        return self.amcl_hist[0][1], '历史最早(兜底, 可能不够旧)'

    def _recovery_tick(self):
        """中止后的恢复: 先倒车脱困并给 AMCL 位移, 再放大不确定度重新播种。"""
        if self.recovery is None or self.odom is None:
            return
        kind, o0, t0 = self.recovery
        if kind == 'wait':
            # 宽限期内什么都不发, 让被取消的目标彻底收场
            if time.time() - t0 < ABORT_GRACE_S:
                return
            self.recovery = ('backup', self.odom, time.time())
            return
        if kind != 'backup' or o0 is None:
            return
        moved = math.hypot(self.odom[1] - o0[1], self.odom[2] - o0[2])
        timeout = time.time() - t0 > BACKUP_TIMEOUT_S
        if moved < BACKUP_DIST and not timeout:
            tw = Twist()
            tw.linear.x = -BACKUP_SPEED
            self.cmd_pub.publish(tw)
            return
        self.cmd_pub.publish(Twist())
        self.recovery = None
        note = ' (超时)' if timeout else ''
        print(f'  [恢复] 已倒车 {moved:.2f} m{note}', flush=True)

    def _reseed_amcl(self):
        pose, src = self._preslip_pose()
        if pose is None:
            print('  [恢复] 收不到 /amcl_pose, 跳过重新播种', flush=True)
            return
        x, y, qz, qw = pose
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        m.pose.covariance[0] = RESEED_XY_VAR
        m.pose.covariance[7] = RESEED_XY_VAR
        m.pose.covariance[35] = RESEED_YAW_VAR
        self.initial_pub.publish(m)
        self.reseeds += 1
        print(f'  [恢复] 以 σ=0.5m 在 ({x:+.2f}, {y:+.2f}) 重新播种 AMCL '
              f'—— 位姿取自 {src}, 因为打滑期间车并没有移动', flush=True)
        # 用过就清掉, 下一次打滑要记新的起点
        self.slip_start_pose = None

    def _abort_nav(self):
        """确认打滑后中止导航。失败不抛异常 —— 监视器不该反过来搞垮系统。"""
        now = time.time()
        if now - self.last_abort < ABORT_COOLDOWN_S:
            return
        self.last_abort = now
        if not self.cancel_cli.service_is_ready():
            if self.verbose:
                print('  [中止] 取消服务不可用, Nav2 在跑吗?', flush=True)
            return
        self.cancel_cli.call_async(CancelGoal.Request())
        self.aborts += 1
        print('  [中止] 已请求取消 Nav2 目标 —— 里程计已不可信, '
              '继续走只会把 AMCL 一起带偏', flush=True)
        # 中止只是停车。这条消息才是"定位现在不可信"那个信息本身,
        # 下游(上层调度、日志、人)据此决定要不要信任当前位姿。
        self.status_pub.publish(Bool(data=True))
        # 先播种再倒车: AMCL 会把倒车位移从修正后的位姿开始积分。
        # 反过来做的话, 倒车这一段就是从错误位姿出发算的。
        self._reseed_amcl()
        if self.recovery is None:
            # 先进 wait 态等控制器停嘴, 到点再转 backup。
            self.recovery = ('wait', None, time.time())

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
            if self.streak == 0:
                # streak 由 0 变 1 的这一帧就是打滑的起点。
                # 此刻的 AMCL 位姿即"车还在正常行驶时的最后一个位姿",
                # 比按固定秒数回溯精确 —— 后者会把打滑前的真实位移一起丢掉。
                self.slip_start_pose = self.amcl_pose
            self.streak += 1
            if self.streak >= CONFIRM_N:
                self.events.append((t1, d_odom, measured, expected))
                if self.verbose:
                    print(f'  [打滑] t={t1:.1f}  odom 位移 {d_odom:.3f} m  '
                          f'扫描变化 {measured * 100:.1f} cm  '
                          f'预期 {expected * 100:.1f} cm  '
                          f'比值 {ratio:.2f}', flush=True)
                if self.abort_enabled:
                    self._abort_nav()
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

    # 前提检查。本测试的接触点 x=2.30 是按"从原点朝 +x 直行撞 box1"推的,
    # 换个起点或朝向, 车会撞上别的东西, 判据就失去意义 ——
    # 曾经紧接着另一个测试跑, 车朝北停着, 结果一路撞上北墙,
    # 检出完全正常却报"未通过", 是个误导性的失败。
    # 前提不满足就明说, 不要产出一个看起来像结论的东西。
    if gt0 is None:
        print('  取不到真值, 无法确认前提')
        return False
    if abs(gt0[0]) > 0.3 or abs(gt0[1]) > 0.3:
        print(f'  前提不满足: 本测试要求车在原点附近, 实际在 {gt0}。')
        print('  请重启仿真后再跑 (bash scripts/kill_stack.sh 然后重新 launch)。')
        return False

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


def _gz(args, timeout=20):
    try:
        return subprocess.run(args, capture_output=True, text=True,
                              timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


def spawn_low_obstacle(name='low_block'):
    """放一个雷达看不见但挡得住车的矮块。尺寸依据见文件顶部常量注释。"""
    sx, sy, sz = LOW_OBSTACLE_SIZE
    sdf = ('<?xml version="1.0"?><sdf version="1.8">'
           f'<model name="{name}"><static>true</static><link name="l">'
           f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size>'
           '</box></geometry></collision>'
           f'<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size>'
           '</box></geometry></visual></link></model></sdf>')
    return 'true' in _gz([
        'gz', 'service', '-s', '/world/my_world/create',
        '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000',
        '--req', f"sdf: '{sdf}', pose: {{position: "
                 f"{{x: {LOW_OBSTACLE_POSE[0]}, y: {LOW_OBSTACLE_POSE[1]}, "
                 f"z: {sz / 2}}}}}"])


def remove_low_obstacle(name='low_block'):
    _gz(['gz', 'service', '-s', '/world/my_world/remove',
         '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', f'name: "{name}", type: MODEL'])


def verify_abort(node):
    """
    验证抢先中止: Nav2 正常导航途中撞上一个雷达看不见的矮块,
    检测器应当检出打滑并取消目标, 而不是任由车继续空转。

    为什么这个场景才算数: 障碍物在 /scan 里不存在, 所以代价地图上也不存在,
    规划器会一路规划穿过去, 局部避障也不会绕。整条 Nav2 链路都认为
    "前方畅通" —— 只有打滑检测能发现车其实已经顶住了。
    这正是"2D 地图是一个切片"的最坏情况。
    """
    print('=' * 66)
    print('自检: Nav2 导航中撞上隐形矮块, 检测器应中止目标')
    print('=' * 66)
    remove_low_obstacle()
    if not node.nav.wait_for_server(timeout_sec=20.0):
        print('  navigate_to_pose 没起来 —— 需要先跑 nav2.launch.py')
        return False

    gt0 = ground_truth()
    print(f'  起点真值 {gt0}')
    if not spawn_low_obstacle():
        print('  矮块投放失败')
        return False
    h, = (LOW_OBSTACLE_SIZE[2],)
    print(f'  已放置 {LOW_OBSTACLE_SIZE[0]}x{LOW_OBSTACLE_SIZE[1]}x{h} m 矮块 '
          f'于 {LOW_OBSTACLE_POSE}  (光束在此高度之上, /scan 看不见)')

    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'map'
    g.pose.pose.position.x = float(ABORT_GOAL[0])
    g.pose.pose.position.y = float(ABORT_GOAL[1])
    g.pose.pose.orientation.w = 1.0
    print(f'  发目标 {ABORT_GOAL} —— 直线穿过矮块所在位置', flush=True)

    send = node.nav.send_goal_async(g)
    rclpy.spin_until_future_complete(node, send, timeout_sec=20.0)
    hd = send.result()
    if hd is None or not hd.accepted:
        print('  目标未被接受')
        remove_low_obstacle()
        return False

    fut = hd.get_result_async()
    end = time.time() + 90.0
    while rclpy.ok() and time.time() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    status = fut.result().status if fut.done() else None
    gt_stuck = ground_truth()

    # 让恢复动作跑完(倒车 + 重新播种)。恢复由定时器驱动, 所以这里
    # 只需要继续 spin。障碍物先不撤 —— 撤了就不是真实场景了。
    print('  等待恢复动作(倒车 + 重新播种) ...', flush=True)
    end = time.time() + 20.0
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.reseeds > 0 and node.recovery is None:
            break
    node.cmd_pub.publish(Twist())
    # 给 AMCL 一点时间消化重新播种
    settle = time.time() + 3.0
    while rclpy.ok() and time.time() < settle:
        rclpy.spin_once(node, timeout_sec=0.1)
    gt1 = ground_truth()
    remove_low_obstacle()

    # 动作状态: 4=SUCCEEDED 5=CANCELED 6=ABORTED
    name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, str(status))
    print()
    print(f'  打滑检出      {len(node.events)} 次')
    print(f'  发出中止请求  {node.aborts} 次')
    print(f'  动作最终状态  {name}')
    print(f'  最终真值      {gt1}')

    # 预期停在矮块南侧: 块中心 y=1.3, 半宽 0.2, 车身半长 0.2 -> y≈0.9
    contact_y = LOW_OBSTACLE_POSE[1] - LOW_OBSTACLE_SIZE[1] / 2 - 0.2
    if gt1:
        dy = abs(gt1[1] - contact_y)
        print(f'  预期接触点 y={contact_y:.2f}, 实测 {gt1[1]:.3f}, '
              f'差 {dy * 1000:.0f} mm')
    else:
        dy = None
        print('  取不到真值, 接触点无法确认')

    # 恢复效果: 车是否退离接触点, 以及重新播种后 AMCL 是否仍与真值一致
    backed = None
    if gt_stuck and gt1:
        backed = math.hypot(gt1[0] - gt_stuck[0], gt1[1] - gt_stuck[1])
        print(f'  卡住位置 ({gt_stuck[0]:+.3f}, {gt_stuck[1]:+.3f}) '
              f'-> 恢复后 ({gt1[0]:+.3f}, {gt1[1]:+.3f}), 退开 {backed:.2f} m')
    amcl_err = None
    if node.amcl_pose and gt1:
        amcl_err = math.hypot(node.amcl_pose[0] - gt1[0],
                              node.amcl_pose[1] - gt1[1])
        print(f'  重新播种后 AMCL 与真值差 {amcl_err:.3f} m')
    print(f'  重新播种次数  {node.reseeds}')

    detected = len(node.events) > 0
    aborted = node.aborts > 0 and status in (5, 6)
    stopped = dy is not None and dy < 0.35
    recovered = (node.reseeds > 0 and backed is not None
                 and backed > BACKUP_DIST * 0.6)
    localized = amcl_err is not None and amcl_err < 0.30
    print()
    print(f'  检出打滑:       {"是" if detected else "否 —— 漏报"}')
    print(f'  目标被中止:     {"是" if aborted else "否"}')
    print(f'  停在矮块处:     {"是" if stopped else ("否" if dy is not None else "无法确认")}')
    print(f'  倒车并重播种:   {"是" if recovered else "否"}')
    print(f'  播种后定位可信: {"是" if localized else ("否" if amcl_err is not None else "无法确认")}')
    ok = detected and aborted and stopped and recovered and localized
    print(f'  判定: {"通过" if ok else "未通过"}')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--verify', action='store_true', help='自己开去撞墙做自检')
    ap.add_argument('--verify-abort', action='store_true',
                    help='验证抢先中止(需要 nav2 在跑)')
    ap.add_argument('--abort', action='store_true',
                    help='检出打滑时取消 Nav2 目标')
    ap.add_argument('--quiet', action='store_true', help='只打印报警')
    args = ap.parse_args()

    rclpy.init()
    # --verify-abort 本身就是在验证中止, 所以隐含开启 --abort
    node = SlipMonitor(verbose=not args.quiet,
                       abort=args.abort or args.verify_abort)
    ok = True
    try:
        if args.verify or args.verify_abort:
            # 等第一帧扫描和里程计
            end = time.time() + 15
            while node.odom is None and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            ok = verify_abort(node) if args.verify_abort else verify(node)
        else:
            mode = '监视 + 检出即中止' if args.abort else '仅监视'
            print(f'{mode} (Ctrl-C 退出) ...')
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
