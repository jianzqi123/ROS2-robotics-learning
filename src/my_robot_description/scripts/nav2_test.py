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

    只跑其中一项:  --only passage / amcl / recovery / stress

反复跑之前务必确认上一轮的进程真的死透了。同名节点并存时
`ros2 lifecycle get` 会随机应答其中一个, 症状是"参数明明改了却不生效"、
"日志显示 activate 成功但状态查出来是 unconfigured"。
另外别用 `pkill -f "gz sim"` 这类模式清理 —— 它会匹配到你自己那条
包含该字样的 shell 命令, 把执行清理的 shell 一起杀掉,
于是清理只做了一半而你以为做完了。按 PID 杀。

测试3 动态障碍触发恢复行为
    此前四个目标全部一次成功, 所以 spin/backup/wait 从未被调用过。
    本测试等车开进通道口再往通道里塞一个封死缺口的静态障碍 ——
    提前塞会让全局规划器从容绕东端, 那是"重规划"不是"恢复"。
    恢复次数取自动作反馈的 number_of_recoveries, 是 BT 真的调用过
    恢复行为的直接证据, 不靠翻日志猜。

测试4 窄通道压测
    此前通道只被正对着走通过一次。本测试连续穿越多次, 后几段起点偏东
    使车斜切进入, 并逐段量出车心到通道两壁的最小余量再减去外接半径 ——
    这才是"离墙多近"的真实答案, 而不是"到了没到"。

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

ROBOT_RADIUS = 0.25       # 与 nav2_params.yaml 一致
INFLATION = 0.40          # 同上

PASSAGE_NORTH = (-4.5, 0.0)
PASSAGE_SOUTH = (-4.5, -3.0)

# 封路障碍: 横跨整个 1.0m 缺口(宽 1.1m 略有富余), 放在通道正中 y=-1.5。
# 目的是让通道彻底不可通行, 逼 Nav2 要么走恢复行为, 要么绕内墙东端。
BLOCKER_POSE = (-4.5, -1.5)
BLOCKER_SIZE = (1.1, 0.3, 1.0)

# 压测航点。挑选依据: 相邻两点分别位于 wall_inner_a 两侧,
# 所以每一段都必须穿通道; 后几段起点偏东, 使车斜着切入而不是正对着进。
# 已核对全部航点都在自由空间: platform 占 x[-4.1,-2.9] y[0.4,1.6],
# box2 占 x[-2.5,-1.5] y[1.5,2.5], 均不与航点重叠。
STRESS_WAYPOINTS = [
    ('通道以北', -4.5, 0.0),
    ('正对穿越-南', -4.5, -3.0),
    ('正对穿越-北', -4.5, 0.5),
    ('斜切穿越-南', -3.0, -3.0),
    ('斜切穿越-北', -2.5, 0.5),
    ('斜切穿越-南2', -4.5, -3.0),
]

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


def _gz(args, timeout=20):
    try:
        return subprocess.run(args, capture_output=True, text=True,
                              timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


def spawn_blocker(x, y, name='blocker'):
    """往运行中的世界里塞一个静态障碍, 用来在导航途中封路。"""
    sx, sy, sz = BLOCKER_SIZE
    sdf = (
        '<?xml version="1.0"?><sdf version="1.8">'
        f'<model name="{name}"><static>true</static><link name="l">'
        f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size>'
        '</box></geometry></collision>'
        f'<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size>'
        '</box></geometry></visual></link></model></sdf>'
    )
    out = _gz([
        'gz', 'service', '-s', '/world/my_world/create',
        '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000',
        '--req', f"sdf: '{sdf}', pose: {{position: {{x: {x}, y: {y}, "
                 f"z: {sz / 2}}}}}",
    ])
    return 'true' in out


def remove_blocker(name='blocker'):
    _gz([
        'gz', 'service', '-s', '/world/my_world/remove',
        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', f'name: "{name}", type: MODEL',
    ])


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

    def send_goal(self, x, y, timeout=180.0, on_sample=None):
        """
        发一个目标点, 阻塞到结束。返回 (成功?, 轨迹采样, 恢复次数)。

        on_sample(gt) 每采到一个真值就回调一次, 用来在导航途中做事情
        (比如等机器人开到某处再扔障碍物)。
        恢复次数取自动作反馈的 number_of_recoveries —— 这是 BT 是否真的
        调用过 spin/backup/wait 的直接证据, 比翻日志靠谱。
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        self._recoveries = 0

        def _fb(msg):
            self._recoveries = max(self._recoveries,
                                   int(msg.feedback.number_of_recoveries))

        send = self.nav.send_goal_async(goal, feedback_callback=_fb)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, [], 0

        result_future = handle.get_result_async()
        track, deadline = [], time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            gt = ground_truth_pose()
            if gt:
                track.append(gt)
                if on_sample:
                    on_sample(gt)
            if result_future.done():
                break
        if not result_future.done():
            handle.cancel_goal_async()
            return False, track, self._recoveries
        ok = result_future.result().status == 4      # 4 = SUCCEEDED
        return ok, track, self._recoveries

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
            ok, track, _ = self.send_goal(gx, gy)
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

    # -------------------------------------------- 测试3 恢复行为 / 动态障碍

    def test_recovery(self):
        print()
        print('=' * 68)
        print('测试3  动态障碍触发恢复行为')
        print('=' * 68)
        if not self.nav.wait_for_server(timeout_sec=20.0):
            print('  navigate_to_pose 没起来')
            return False

        remove_blocker()
        print('  先回到通道以北 ...', flush=True)
        self.send_goal(*PASSAGE_NORTH)

        # 目标在通道以南。等车真的开进通道口再封路 ——
        # 提前封会让全局规划器不慌不忙地绕东边走, 那是"重规划"不是"恢复"。
        state = {'dropped': False}

        def watch(gt):
            if state['dropped']:
                return
            if gt[0] < -4.0 and gt[1] < -0.6:
                if spawn_blocker(*BLOCKER_POSE):
                    state['dropped'] = True
                    print(f'     [车到 ({gt[0]:+.2f},{gt[1]:+.2f}), 封死通道]',
                          flush=True)

        # 超时给到 420s: 封路后唯一出路是绕内墙东端, 路程约 9.3m,
        # 0.22m/s 下净行驶就要 42s, 而恢复行为每次要花掉 5~15s。
        # 240s 试过, 车推进到 x=-1.45(离东端只差 0.45m)才超时 ——
        # 那是预算不够, 不是走不通, 两者结论天差地别。
        print(f'  -> 通道以南 {PASSAGE_SOUTH} , 途中封路 ...', flush=True)
        ok, track, recoveries = self.send_goal(*PASSAGE_SOUTH, timeout=420.0,
                                               on_sample=watch)
        gt = ground_truth_pose()
        remove_blocker()

        if not state['dropped']:
            print('  障碍物始终没投放 —— 车没走到通道口, 本测试无效')
            return False

        detoured = any(p[0] > -1.0 for p in track)   # 绕到内墙东端以外
        print()
        print(f'  恢复行为触发次数  {recoveries}')
        print(f'  最终到达          {"是" if ok else "否"}'
              f'  真值({gt[0]:+.3f}, {gt[1]:+.3f})' if gt else '')
        print(f'  是否绕行东端      {"是" if detoured else "否"}')
        print()
        # 判据: 通道被封死后, 唯一的合格结局是"察觉到走不通并另找出路"。
        # 恢复行为触发, 或者绕东端成功抵达, 都算; 两者都没有说明它在硬撞。
        ok_overall = (recoveries > 0) or (ok and detoured)
        print(f'  判定: {"通过" if ok_overall else "未通过"}'
              f'  (需要 恢复次数>0 或 绕行东端抵达)')
        return ok_overall

    # ------------------------------------------------ 测试4 窄通道压测

    def test_stress(self):
        print()
        print('=' * 68)
        print('测试4  窄通道反复穿越 + 斜切进入')
        print('=' * 68)
        if not self.nav.wait_for_server(timeout_sec=20.0):
            print('  navigate_to_pose 没起来')
            return False
        remove_blocker()

        rows = []
        for i, (label, gx, gy) in enumerate(STRESS_WAYPOINTS):
            ok, track, rec = self.send_goal(gx, gy, timeout=240.0)
            gt = ground_truth_pose()
            err = math.hypot(gx - gt[0], gy - gt[1]) if gt else float('nan')

            # 只统计真的进了通道的那些段
            inside = [p for p in track
                      if PASSAGE_X[0] <= p[0] <= PASSAGE_X[1]
                      and PASSAGE_Y[0] <= p[1] <= PASSAGE_Y[1]]
            # 通道西侧是西墙内表面 x=-5.0, 东侧是 wall_inner_a 的端头 x=-4.0。
            # 车心到两侧的最小距离再减去外接半径, 就是真实余量。
            clear = (min(min(p[0] - PASSAGE_X[0], PASSAGE_X[1] - p[0])
                         for p in inside) - ROBOT_RADIUS) if inside else None
            rows.append((label, ok, err, bool(inside), clear, rec))
            c = f'{clear * 100:5.1f} cm' if clear is not None else '   —  '
            print(f'  {i + 1}. {label:<16}{"成功" if ok else "失败"}  '
                  f'误差 {err:.3f} m  {"穿越" if inside else "未穿越"}  余量 {c}')

        passes = [r for r in rows if r[3]]
        ok_all = all(r[1] for r in rows)
        print()
        print(f'  穿越次数    {len(passes)}')
        print(f'  全部到达    {"是" if ok_all else "否"}')
        if passes:
            worst = min(r[4] for r in passes)
            print(f'  最小余量    {worst * 100:.1f} cm'
                  f'   (膨胀层留出的理论余量 {(0.5 - INFLATION) * 100:.0f} cm)')
            print(f'  恢复行为    共 {sum(r[5] for r in rows)} 次')
        # 判据: 每段都要到达, 且至少穿越 4 次(含斜切), 且始终没有真正贴墙。
        ok_overall = ok_all and len(passes) >= 4 and min(
            r[4] for r in passes) > 0.0 if passes else False
        print(f'  判定: {"通过" if ok_overall else "未通过"}')
        return ok_overall

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
    ap.add_argument('--only',
                    choices=['passage', 'amcl', 'recovery', 'stress'])
    args = ap.parse_args()

    rclpy.init()
    node = Nav2Test()
    results = {}
    try:
        if args.only in (None, 'passage'):
            results['多目标 + 窄通道'] = node.test_passage()
        if args.only in (None, 'amcl'):
            results['AMCL 收敛'] = node.test_amcl()
        if args.only in (None, 'recovery'):
            results['动态障碍 / 恢复'] = node.test_recovery()
        if args.only in (None, 'stress'):
            results['窄通道压测'] = node.test_stress()
    finally:
        node.cmd_pub.publish(Twist())
        remove_blocker()
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
