#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空地瞬移诊断脚本  (对应 HANDOFF.md 第6节 问题1 布置的诊断动作)

用法:
    # 终端1
    source ~/ros2_ws/install/setup.bash
    ros2 launch my_robot_description gazebo.launch.py

    # 终端2 (先确认机器人周围空旷, 远离 box1(3,0)/box2(-2,2)/cylinder1(0,-3))
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/src/my_robot_description/scripts/teleport_test.py

脚本自动跑一套动作序列(前进/后退/原地转/边走边转/急加速急停),
同时监听 /odom, 对相邻两帧做位移突变检测, 结束后打印判定结论。
不需要 teleop_twist_keyboard, 不需要人工按键。

说明: 全程用 odom 消息头里的仿真时间驱动, 所以不受 Gazebo 实时率<1 的影响。
"""

import math
import re
import subprocess
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# 动作序列: (标签, 线速度 m/s, 角速度 rad/s, 持续秒数)
# 急停项是重点: 刹车产生的反向扭矩最容易让底盘俯仰, 是原设计下最容易触发弹飞的时刻
SEQUENCE = [
    ("静止基线",        0.0,  0.0, 3.0),
    ("前进 0.2",        0.2,  0.0, 4.0),
    ("急停",            0.0,  0.0, 1.5),
    ("后退 0.2",       -0.2,  0.0, 3.0),
    ("急停",            0.0,  0.0, 1.5),
    ("原地左转 1.0",    0.0,  1.0, 4.0),
    ("原地右转 1.0",    0.0, -1.0, 4.0),
    ("急停",            0.0,  0.0, 1.5),
    ("边走边转",        0.2,  0.8, 5.0),
    ("急停",            0.0,  0.0, 1.5),
    ("急加速 0.5",      0.5,  0.0, 3.0),
    ("急停 (重点观察)", 0.0,  0.0, 2.0),
]

# 判据: 相邻两帧的等效速度 > |指令线速度| * 3 + 0.5 m/s  → 记为瞬移
# 用"速度"而不是"距离"判定, 这样偶发丢包(dt变大)不会造成误报
SPEED_FACTOR = 3.0
SPEED_MARGIN = 0.5
# 角速度同理
YAW_FACTOR = 3.0
YAW_MARGIN = 1.0


def ground_truth_pose(model='my_robot'):
    """
    从 Gazebo 取真实位姿, 返回 (x, y, yaw) 或 None。

    为什么必须有这一项: /odom 是 DiffDrive 从轮子转角积分出来的。轮子是速度控制,
    不管有没有抓地都会转到目标转速, 所以轮子打滑或悬空时 odom 依然"完美",
    只测 odom 自洽会漏掉真车根本没按指令走的情况。
    """
    try:
        out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                             capture_output=True, text=True, timeout=15).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    # 匹配 "[x y z]" 和紧随其后的 "[roll pitch yaw]"
    nums = re.findall(r'\[\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s*\]', out)
    if len(nums) < 2:
        return None
    xyz, rpy = nums[0], nums[1]
    return float(xyz[0]), float(xyz[1]), float(rpy[2])


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class TeleportTest(Node):

    def __init__(self):
        super().__init__('teleport_test')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 50)

        self.phase = 0
        self.phase_t0 = None      # 当前动作的起始仿真时刻
        self.t0 = None            # 整个测试的起始仿真时刻
        self.prev = None          # (t, x, y, yaw)
        self.odom_start = None    # t0 时刻的 odom 位姿
        # 测试开始前机器人是静止的, 所以这里取的真实位姿等价于 t0 时刻的
        self.gt_start = ground_truth_pose()
        if self.gt_start is None:
            print('注意: 取不到真实位姿(gz 命令不可用?), 本次只做 odom 自洽检查')
        self.events = []          # 检测到的瞬移
        self.path_len = 0.0
        self.n_msgs = 0
        self.done = False

        # 没收到 odom 时的提示
        self.create_timer(5.0, self.check_alive)

        print('等待 /odom ...  (确保 gazebo.launch.py 已经在跑)')

    def check_alive(self):
        if self.n_msgs == 0:
            print('!! 5秒内没收到 /odom, 检查: ros2 topic hz /odom')

    def stop(self):
        self.pub.publish(Twist())

    def on_odom(self, msg):
        if self.done:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.n_msgs += 1

        if self.t0 is None:
            self.t0 = t
            self.phase_t0 = t
            self.prev = (t, p.x, p.y, yaw)
            self.odom_start = (p.x, p.y, yaw)
            self.announce()
            return

        # ---------- 突变检测 ----------
        pt, px, py, pyaw = self.prev
        dt = t - pt
        if dt > 1e-6:
            d = math.hypot(p.x - px, p.y - py)
            v = d / dt
            dyaw = abs(wrap_pi(yaw - pyaw))
            w = dyaw / dt

            _, cmd_v, cmd_w, _ = SEQUENCE[self.phase]
            v_limit = abs(cmd_v) * SPEED_FACTOR + SPEED_MARGIN
            w_limit = abs(cmd_w) * YAW_FACTOR + YAW_MARGIN

            if v > v_limit or w > w_limit:
                self.events.append({
                    't': t - self.t0,
                    'phase': SEQUENCE[self.phase][0],
                    'jump_m': d,
                    'speed': v,
                    'v_limit': v_limit,
                    'yaw_rate': w,
                    'w_limit': w_limit,
                    'dt': dt,
                })
                print(f'  >>> 瞬移! t={t - self.t0:6.2f}s  '
                      f'跳变={d:.3f}m  等效速度={v:.2f}m/s (阈值{v_limit:.2f})  '
                      f'角速度={w:.2f}rad/s (阈值{w_limit:.2f})')
            else:
                self.path_len += d

        self.prev = (t, p.x, p.y, yaw)

        # ---------- 动作推进 ----------
        _, cmd_v, cmd_w, dur = SEQUENCE[self.phase]
        if t - self.phase_t0 >= dur:
            self.phase += 1
            self.phase_t0 = t
            if self.phase >= len(SEQUENCE):
                self.stop()
                self.done = True
                self.report(p, yaw)
                return
            self.announce()

        cmd = Twist()
        cmd.linear.x = SEQUENCE[self.phase][1]
        cmd.angular.z = SEQUENCE[self.phase][2]
        self.pub.publish(cmd)

    def announce(self):
        label, v, w, dur = SEQUENCE[self.phase]
        print(f'[{self.phase + 1}/{len(SEQUENCE)}] {label:<16} '
              f'v={v:+.2f} w={w:+.2f}  {dur}s')

    def report_ground_truth(self, p, yaw):
        """把 odom 累积的位移/转角 和 Gazebo 真实位姿 对比, 检查里程计是否失真。"""
        gt_end = ground_truth_pose()
        if self.gt_start is None or gt_end is None or self.odom_start is None:
            print('真实位姿对比: 不可用(跳过)')
            print()
            return

        gx0, gy0, ga0 = self.gt_start
        gx1, gy1, ga1 = gt_end
        ox0, oy0, oa0 = self.odom_start

        # 必须比"向量"不能比"距离": 距离只有大小, 方向反了照样相等。
        # 各自旋转回自己的起始朝向, 得到起始车体系下的净位移向量, 再逐分量比。
        def to_body(dx, dy, a0):
            c, s = math.cos(-a0), math.sin(-a0)
            return c * dx - s * dy, s * dx + c * dy

        gvx, gvy = to_body(gx1 - gx0, gy1 - gy0, ga0)
        ovx, ovy = to_body(p.x - ox0, p.y - oy0, oa0)
        pos_err = math.hypot(gvx - ovx, gvy - ovy)

        gt_yaw = math.degrees(wrap_pi(ga1 - ga0))
        od_yaw = math.degrees(wrap_pi(yaw - oa0))
        yaw_err = math.degrees(wrap_pi(math.radians(gt_yaw - od_yaw)))

        print('--- odom vs 真实位姿 (轮式里程计失真检查) ---')
        print(f'  净位移向量(起始车体系)  真实 ({gvx:+.3f}, {gvy:+.3f})   '
              f'odom ({ovx:+.3f}, {ovy:+.3f})')
        print(f'  位移向量误差            {pos_err:.3f} m      <- 主判据, 无歧义')
        print(f'  净转角                  真实 {gt_yaw:+7.1f}°  odom {od_yaw:+7.1f}°  '
              f'差 {yaw_err:+.1f}°')
        print('                          (转角是 mod 360° 的, 差值不能单独下结论)')
        print()

        if pos_err < 0.15 and abs(yaw_err) < 10.0:
            print('  → 里程计与真实运动一致, odom 可信, 可以拿去喂 slam_toolbox。')
        else:
            print('  !! 里程计与真实运动不一致 —— 即使瞬移次数为 0 也不能建图。')
            # 方向整体反了 = 轮子转向反了, 是最常见也最容易被漏掉的一种
            if math.hypot(gvx + ovx, gvy + ovy) < pos_err * 0.3:
                print('     真实位移 ≈ odom位移的反向量 —— 两个轮子都在反着转。')
                print('     查 URDF 轮子 <axis>: 关节 origin 的 rpy 会旋转 axis,')
                print('     绕X转+90°时 axis="0 0 1" 实际指向 -Y, 应填 "0 0 -1"。')
            else:
                print('     轮子打滑 / 悬空空转 / wheel_radius 与实际几何不符。')
        print()

    def report(self, p, yaw):
        print()
        print('=' * 62)
        print('空地瞬移测试结果')
        print('=' * 62)
        print(f'odom 消息数     : {self.n_msgs}')
        print(f'累计路径长度    : {self.path_len:.2f} m  (已剔除瞬移跳变)')
        print(f'最终 odom 位置  : x={p.x:.3f}  y={p.y:.3f}  yaw={math.degrees(yaw):.1f}°')
        print(f'检测到的瞬移次数: {len(self.events)}')
        print()
        self.report_ground_truth(p, yaw)

        if not self.events:
            print('结论: 空地无瞬移(问题1的弹飞现象已消除)。')
            print('  → 但是否能进建图主线, 以上面"odom vs 真实位姿"那一节为准:')
            print('    瞬移0次只说明 odom 连续, 不代表 odom 正确。')
        else:
            worst = max(self.events, key=lambda e: e['jump_m'])
            print(f'最大一次跳变: {worst["jump_m"]:.3f} m '
                  f'@ t={worst["t"]:.2f}s  动作="{worst["phase"]}"')
            phases = {}
            for e in self.events:
                phases[e['phase']] = phases.get(e['phase'], 0) + 1
            print('按动作分布:')
            for k, n in sorted(phases.items(), key=lambda kv: -kv[1]):
                print(f'    {k:<16} {n} 次')
            print()
            print('结论: 空地仍有瞬移 → 万向轮不是(唯一)根因。')
            print('  → 按 HANDOFF 第6节分支: 问题在机器人自身物理解算稳定性,')
            print('    下一步查 world 的 <physics> 接触求解参数 / DiffDrive 内部积分。')
            print('  → 把上面"按动作分布"贴给我, 集中在急停/转弯能进一步缩小范围。')
        print('=' * 62)


def main():
    rclpy.init()
    node = TeleportTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n中断, 停车')
    finally:
        node.stop()
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if not node.events else 1


if __name__ == '__main__':
    sys.exit(main())
