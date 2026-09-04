#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nav2 自动化验证 —— 把 README 里两条"没测过"变成测过的。

用法:
    # 终端1 (可以 headless)
    ros2 launch my_robot_description gazebo.launch.py headless:=true
    # 终端2
    ros2 launch my_robot_description nav2.launch.py rviz:=false
    # 终端3
    python3 ~/ros2_ws/src/my_robot_description/scripts/nav2_test.py

    只跑其中一项:  --only passage    /  --only amcl

测试1 多目标穿越窄通道
    此前 Nav2 只被送过一个目标点, 落在空地上, 既没考验规划器绕障,
    也没走过场地里唯一的窄通道。
    通道 = wall_inner_a 西端(x=-4.0) 到西墙内表面(x=-5.0) 的 1.0m 缺口,
    位于 y≈-1.5。机器人外接半径 0.25, 膨胀半径 0.40, 中线离两侧各 0.50 ——
    也就是说通道里只有一条勉强零代价的中线, 这正是 inflation_radius
    必须小于 0.50 的原因。本测试逐段记录终点误差, 并检查轨迹是否真的
    从通道里穿过去了(而不是绕远路从东边 x>-1.0 迂回)。

测试2 AMCL 错误初始位姿
    此前 AMCL 从未被考验过: 车从原点出生, set_initial_pose 把正确答案
    直接给了它, 里程计又准到 0.39%, 粒子滤波器没有任何需要纠正的东西。
    本测试故意给一个偏掉的初始位姿, 然后原地慢转一圈喂给它扫描数据,
    看它能不能收敛回真值。原地旋转是最干净的做法: 不移动就不引入
    里程计误差, 收敛与否只取决于扫描匹配本身。

判据都写在输出里, 不靠"看起来没报错"。
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
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose


# 目标序列。第 1->2 段是重点: 从通道以北直插西南角, 最短路必须穿过
# x∈[-5,-4] y≈-1.5 那个 1.0m 缺口。其余各点用来确认走完还能正常收尾。
GOALS = [
    ('通道以北(西侧)', -4.5, 0.0),
    ('穿窄通道到西南', -4.5, -3.0),
    ('横穿南侧', 2.0, -3.0),
    ('回到原点', 0.0, 0.0),
]

# 窄通道判定窗口
PASSAGE_X = (-5.0, -4.0)
PASSAGE_Y = (-1.9, -1.1)

# 测试2 故意给的偏移量: 0.8m + 25 度。
# 选这个量级的理由: 明显大于 AMCL 的 update_min_d(0.25m), 足以说明
# "它真的被放错了"; 又小于 laser_likelihood_max_dist(2.0m), 使得
# 似然场仍然能提供指向正确方向的梯度 —— 再大就不是收敛测试,
# 而是全局重定位测试, 那需要 global_localization 服务而不是初始位姿。
BAD_OFFSET = (0.8, -0.6, math.radians(25.0))


def ground_truth_pose(model='my_robot'):
    """从 Gazebo 取真实位姿 (x, y, yaw); 取不到返回 None。"""
    try:
        out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                             capture_output=True, text=True, timeout=15).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    nums = re.findall(r'\[\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s*\]', out)
    if len(nums) < 2:
        return None
    return float(nums[0][0]), float(nums[0][1]), float(nums[1][2])


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class Nav2Test(Node):

    def __init__(self):
        super().__init__('nav2_test')
        self.amcl_pose = None

        # AMCL 发的是 transient_local, 订阅端 QoS 必须匹配, 否则收不到
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, qos)
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_amcl(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z),
                         1.0 - 2.0 * (p.orientation.z ** 2))
        self.amcl_pose = (p.position.x, p.position.y, yaw)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    # ------------------------------------------------ 测试1 多目标 + 窄通道

    def send_goal(self, x, y, timeout=180.0):
        """发一个目标点, 阻塞到结束。返回 (成功?, 轨迹采样列表)。"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, []

        result_future = handle.get_result_async()
        track, deadline = [], time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            gt = ground_truth_pose()
            if gt:
                track.append(gt)
            if result_future.done():
                break
        if not result_future.done():
            return False, track
        return result_future.result().status == 4, track   # 4 = SUCCEEDED

    def test_passage(self):
        print('=' * 68)
        print('测试1  多目标导航 + 窄通道')
        print('=' * 68)
        if not self.nav.wait_for_server(timeout_sec=20.0):
            print('  navigate_to_pose 动作服务器没起来 —— nav2 跑了吗?')
            return False

        rows, went_through = [], False
        for label, gx, gy in GOALS:
            print(f'  -> {label} ({gx}, {gy}) ...', flush=True)
            ok, track = self.send_goal(gx, gy)
            gt = ground_truth_pose()
            if gt is None:
                print('     取不到 Gazebo 真值')
                return False
            err = math.hypot(gx - gt[0], gy - gt[1])
            # 轨迹是否真的从通道里穿过
            through = any(PASSAGE_X[0] <= p[0] <= PASSAGE_X[1]
                          and PASSAGE_Y[0] <= p[1] <= PASSAGE_Y[1] for p in track)
            went_through = went_through or through
            rows.append((label, ok, gt, err, through, len(track)))
            print(f'     {"成功" if ok else "失败"}  '
                  f'真值({gt[0]:+.3f}, {gt[1]:+.3f})  误差 {err:.3f} m'
                  f'{"  [穿过窄通道]" if through else ""}')

        print()
        print(f'  {"目标":<18}{"结果":<6}{"终点误差":>10}{"采样":>7}')
        for label, ok, gt, err, through, n in rows:
            print(f'  {label:<18}{"成功" if ok else "失败":<6}{err:>9.3f} m{n:>7}')

        all_ok = all(r[1] for r in rows)
        print()
        print(f'  全部到达: {"是" if all_ok else "否"}')
        print(f'  穿越窄通道: {"是" if went_through else "否 —— 规划器绕了远路"}')
        return all_ok and went_through

    # ------------------------------------------------------- 测试2 AMCL

    def publish_initial(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 协方差要给够, 否则 AMCL 认为这个错误位姿非常可信, 不肯散开粒子
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.initial_pub.publish(msg)

    def amcl_error(self):
        gt = ground_truth_pose()
        if gt is None or self.amcl_pose is None:
            return None
        return (math.hypot(self.amcl_pose[0] - gt[0], self.amcl_pose[1] - gt[1]),
                abs(wrap_pi(self.amcl_pose[2] - gt[2])))

    def test_amcl(self):
        print()
        print('=' * 68)
        print('测试2  AMCL 从错误初始位姿收敛')
        print('=' * 68)
        self.spin_for(3.0)
        gt = ground_truth_pose()
        if gt is None:
            print('  取不到 Gazebo 真值')
            return False

        bx, by, byaw = gt[0] + BAD_OFFSET[0], gt[1] + BAD_OFFSET[1], gt[2] + BAD_OFFSET[2]
        print(f'  真值        ({gt[0]:+.3f}, {gt[1]:+.3f}, {math.degrees(gt[2]):+.1f}°)')
        print(f'  故意给成    ({bx:+.3f}, {by:+.3f}, {math.degrees(byaw):+.1f}°)')
        self.publish_initial(bx, by, byaw)
        self.spin_for(3.0)

        before = self.amcl_error()
        if before is None:
            print('  收不到 /amcl_pose')
            return False
        print(f'  注入后误差  {before[0]:.3f} m, {math.degrees(before[1]):.1f}°')

        # 原地慢转一圈: 不移动就不引入里程计误差, 收敛与否只看扫描匹配。
        # 0.4 rad/s 转 16s ≈ 6.4 rad ≈ 366 度。
        print('  原地旋转 16 s 喂扫描数据 ...', flush=True)
        t_end = time.time() + 16.0
        tw = Twist()
        tw.angular.z = 0.4
        while time.time() < t_end and rclpy.ok():
            self.cmd_pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())
        self.spin_for(3.0)

        after = self.amcl_error()
        if after is None:
            print('  收不到 /amcl_pose')
            return False
        print(f'  旋转后误差  {after[0]:.3f} m, {math.degrees(after[1]):.1f}°')

        # 判据: 位置误差要收敛到 0.15m 以内(等于 xy_goal_tolerance),
        # 且必须比注入的误差明显小 —— 只看"变小了"不够,
        # 粒子滤波器每一步都会抖动, 抖小一点不等于收敛。
        ok = after[0] < 0.15 and after[0] < before[0] * 0.5
        print()
        print(f'  位置误差 {before[0]:.3f} -> {after[0]:.3f} m '
              f'(判据: <0.15 且降幅过半)')
        print(f'  航向误差 {math.degrees(before[1]):.1f}° -> '
              f'{math.degrees(after[1]):.1f}°')
        print(f'  收敛: {"是" if ok else "否"}')
        return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', choices=['passage', 'amcl'])
    args = ap.parse_args()

    rclpy.init()
    node = Nav2Test()
    results = {}
    try:
        if args.only in (None, 'passage'):
            results['多目标 + 窄通道'] = node.test_passage()
        if args.only in (None, 'amcl'):
            results['AMCL 收敛'] = node.test_amcl()
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

    print()
    print('=' * 68)
    for k, v in results.items():
        print(f'  {k:<20}{"通过" if v else "未通过"}')
    print('=' * 68)
    sys.exit(0 if all(results.values()) else 1)


if __name__ == '__main__':
    main()
