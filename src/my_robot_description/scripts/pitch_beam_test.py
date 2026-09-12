#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘俯仰如何改变 /scan 的指向 —— 用一次加速直接量出来。

用法:
    ros2 launch my_robot_description gazebo.launch.py headless:=true
    python3 scripts/pitch_beam_test.py          # 不需要 nav2

起因: 地图上其它实心物体内部都是"未知"(近面之后没被观测到),
      唯独 platform 是"空闲" —— 说明有光束从它上方穿过去了。
      但静止时光束 -0.337° 下倾、从 0.182m 出发,
      在任何距离上都越不过 0.25m 的台阶。

假设: 加速时底盘后仰到后万向轮(俯仰行程 ±1.337°), /scan 的净仰角从
      静止的 -0.337° 变成 +2.337°, 于是 1.52m 之外光束从 0.25m 的台阶
      上方掠过, 把台阶后面的格子标成空闲。

实测结论(假设成立):
                静止          加速
      俯仰角    +1.337°      -1.344°   <- 摆到另一个万向轮, 预测 -1.337°
      前方测距   1.78 m       3.88 m   <- 越过台阶, 打到 4.0m 处的西墙
      预测与实测的俯仰角差 0.007°。

做法: 把车放在 platform 正东 1.9m 处朝它, 然后给一个速度阶跃。
      同时以高速率记录 (a) Gazebo 真值俯仰角 (b) 正前方那条光束的距离。

预期: 静止  俯仰 -1.337°(前倾), 前方测距 ≈ 1.9m (打在台阶东面)
      加速  俯仰 转正(后仰),     前方测距 跳到 ≈ 4.0m (越过台阶打到西墙)
      两者若同时发生, 假设成立。

俯仰角来自 gz topic -e 流(实测 0.1s 内能拿到 3 条), 不用 gz model ——
后者单次要 5 秒, 抓不住 1 秒的加速瞬态。
"""

import math
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

START = (-1.0, 1.0, math.pi)      # 朝西, 正对 platform 东面(x=-2.9)
EXPECT_PLATFORM = 1.9             # 静止时前方应测到的距离
EXPECT_WALL = 4.0                 # 越过台阶后应打到西墙


def set_pose(x, y, yaw):
    req = (f'name: "my_robot", position: {{x: {x}, y: {y}, z: 0.15}}, '
           f'orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}')
    out = subprocess.run(
        ['gz', 'service', '-s', '/world/my_world/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', req],
        capture_output=True, text=True, timeout=20).stdout
    return 'true' in out


class PitchStream:
    """后台流式读取 Gazebo 真值俯仰角。"""

    def __init__(self):
        self.samples = []          # (t, pitch_deg)
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        p = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', '/world/my_world/dynamic_pose/info'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        buf, in_robot = [], False
        try:
            for line in p.stdout:
                if self._stop.is_set():
                    break
                if 'name: "my_robot"' in line:
                    in_robot, buf = True, []
                elif in_robot:
                    buf.append(line)
                    if line.startswith('}'):
                        in_robot = False
                        # 只取 orientation 那个花括号块。
                        # 不能对整段做 findall 再塞进 dict —— position 和
                        # orientation 共用 x/y/z 键名, 会互相覆盖,
                        # 第一版就是这么写的, 结果一条样本都没采到。
                        blk = re.search(r'orientation\s*{([^}]*)}',
                                        ''.join(buf), re.S)
                        if not blk:
                            continue
                        o = dict(re.findall(r'([xyzw]):\s*(-?[\d.e+-]+)',
                                            blk.group(1)))
                        if len(o) == 4:
                            x, y, z, w = (float(o[k]) for k in 'xyzw')
                            sv = max(-1.0, min(1.0, 2*(w*y - z*x)))
                            self.samples.append(
                                (time.time(), math.degrees(math.asin(sv))))
        finally:
            p.kill()

    def stop(self):
        self._stop.set()


class Probe(Node):

    def __init__(self):
        super().__init__('platform_test')
        self.fwd = []              # (t, range)
        self.create_subscription(LaserScan, '/scan', self._scan,
                                 qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def _scan(self, m):
        i = int(round((0.0 - m.angle_min) / m.angle_increment))
        if 0 <= i < len(m.ranges):
            r = m.ranges[i]
            self.fwd.append((time.time(), r if math.isfinite(r) else None))

    def spin(self, secs, vx=None):
        end = time.time() + secs
        tw = Twist()
        if vx is not None:
            tw.linear.x = vx
        while rclpy.ok() and time.time() < end:
            if vx is not None:
                self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
        if vx is not None:
            self.cmd.publish(Twist())


def stats(seq, t0, t1):
    v = [x for t, x in seq if t0 <= t <= t1 and x is not None]
    return (min(v), max(v), sum(v)/len(v), len(v)) if v else (None,)*4


def main():
    rclpy.init()
    n = Probe()

    print('=' * 68)
    print('platform 内部为何被标空闲 —— 加速后仰假设的验证')
    print('=' * 68)
    print(f'  把车放到 {START[:2]} 朝西 ...', flush=True)
    if not set_pose(*START):
        print('  set_pose 失败')
        return 1
    n.spin(4.0)                                  # 等落稳

    ps = PitchStream()
    time.sleep(1.0)

    t_rest0 = time.time()
    n.spin(3.0)
    t_rest1 = time.time()

    print('  给速度阶跃 0 -> 0.22 m/s ...', flush=True)
    t_acc0 = time.time()
    n.spin(2.5, vx=0.22)
    t_acc1 = time.time()

    n.spin(3.0)
    ps.stop()

    rmin, rmax, ravg, rn = stats(n.fwd, t_rest0, t_rest1)
    amin, amax, aavg, an = stats(n.fwd, t_acc0, t_acc1)
    pr = stats(ps.samples, t_rest0, t_rest1)
    pa = stats(ps.samples, t_acc0, t_acc1)

    # 两个量分开打印。第一版把它们塞在同一行且由俯仰的非空判断守着,
    # 结果俯仰解析出问题时, 好好的测距数据也一起被藏掉了,
    # 输出看起来像"两边都没数据" —— 一个误导性的呈现。
    def fmt(v, unit, w=6, p=2):
        return f'{v:>{w}.{p}f}{unit}' if v is not None else f'{"—":>{w}}{unit}'

    print()
    print(f'{"量":<14}{"静止阶段":>22}{"加速阶段":>22}')
    print('-' * 62)
    print(f'{"俯仰角 min":<14}{fmt(pr[0], "°", 8, 3):>22}'
          f'{fmt(pa[0], "°", 8, 3):>22}')
    print(f'{"俯仰角 max":<14}{fmt(pr[1], "°", 8, 3):>22}'
          f'{fmt(pa[1], "°", 8, 3):>22}')
    print(f'{"前方测距 min":<14}{fmt(rmin, " m"):>22}{fmt(amin, " m"):>22}')
    print(f'{"前方测距 max":<14}{fmt(rmax, " m"):>22}{fmt(amax, " m"):>22}')
    print(f'{"样本数":<14}{str(rn or 0) + "/" + str(pr[3] or 0):>22}'
          f'{str(an or 0) + "/" + str(pa[3] or 0):>22}   (测距/俯仰)')
    print('-' * 62)
    # 符号约定必须实测确认, 不能想当然 —— 第一版写反了, 于是判据去查
    # 最大值(前倾侧)而不是最小值(后仰侧), 碰巧也返回"是",
    # 是个查错了量却通过的废判据。
    # 实测: 静止时 +1.337°, 而静止态就是前倾 => 正号 = 前倾。
    print('  俯仰角约定(实测确认): 正 = 前倾(静止态 +1.337°), 负 = 后仰')
    print()

    if None in (rmax, amax) or pa[0] is None:
        print('  数据不足, 无法判定')
        n.destroy_node()
        rclpy.shutdown()
        return 1

    # 后仰 = 俯仰角变负。要查最小值, 不是最大值。
    rest_pitch = pr[1] if pr[1] is not None else 1.337
    pitched_up = pa[0] is not None and pa[0] < rest_pitch - 1.0
    beam_cleared = amax > (EXPECT_PLATFORM + EXPECT_WALL) / 2
    rest_sees = rmax < EXPECT_PLATFORM + 0.4

    print(f'  静止时看得见台阶(测距≈{EXPECT_PLATFORM}m): '
          f'{"是" if rest_sees else "否"}')
    print(f'  加速时出现后仰(俯仰转负):   {"是" if pitched_up else "否"}'
          f'   最小 {pa[0]:+.3f}° (静止 {rest_pitch:+.3f}°)')
    print(f'  加速时光束越过台阶(测距≈{EXPECT_WALL}m): '
          f'{"是" if beam_cleared else "否"}')
    print()
    if rest_sees and pitched_up and beam_cleared:
        print('  结论: 假设成立 —— 加速后仰确实让光束越过 0.25m 台阶,')
        print('        这解释了地图上 platform 内部为何被标成空闲。')
        ok = True
    elif rest_sees and not beam_cleared:
        print('  结论: 假设不成立 —— 加速期间光束始终没有越过台阶,')
        print('        platform 内部被标空闲另有原因。')
        ok = False
    else:
        print('  结论: 前提未满足(静止时就没看到台阶), 本次测量无效。')
        ok = False
    print('=' * 68)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
