#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读 DWB 的 /evaluation, 找出是哪个 critic 在压制某一类轨迹。

用法:
    # 让车处在你想诊断的状态下(例如正在缓慢绕行), 然后:
    python3 scripts/dwb_critic_probe.py [采样秒数]

为什么需要它:
    "车为什么不快点走" 这类问题, 靠改参数试是试不出来的 ——
    本项目为此连续试了四个有理有据的假设(进度检查窗口、是否允许倒车、
    局部代价地图尺寸、目标方向 critic 权重), 全部被实测推翻,
    其中一个方向还完全搞反了。
    而 DWB 本来就会把每条候选轨迹在每个 critic 上的原始分和权重
    发布到 /evaluation。与其猜, 不如直接读分数。

方法:
    每一帧比较两条轨迹 ——
        实际被选中的(总分最低)   vs   该帧里 vx 最大的那条
    逐个 critic 算加权分差。DWB 取总分最低者, 所以
    "最快轨迹得分 - 选中轨迹得分" 为正且最大的那个 critic,
    就是让车不敢提速的原因。

    总分为负表示该轨迹被否决(通常是撞到 inscribed 代价),
    必须单独统计, 混进均值会污染结论。

一次真实的输出(封路后绕行、车在爬行时采到):
    实际选中的 vx 均值 0.000 m/s   <- 车在主动选择纯旋转
    可选最大 vx   均值 0.129 m/s
        PathAlign    选中 0.02  最快 2.79  差 +2.77  <- 唯一的惩罚项
        GoalDist     选中 2.58  最快 1.49  差 -1.09  (反而奖励快轨迹)
        其余 critic  差值全为 0
    结论: PathAlign(权重 32, 前视点 0.1m) 衡量车头前方一点与路径的对齐度,
    车头稍偏, 任何前进都让该点更偏, 而原地旋转能"免费"改善对齐,
    于是弯曲路径上车一直转不敢走。同一套参数在直线走廊里能跑满速,
    对照非常干净。

    注意: 据此把 PathAlign 降到 8.0 后, 机理指标确实改善
    (选中 vx 0.000 -> 0.026, 惩罚 2.77 -> 0.28), 但端到端结果反而更差
    (东进 2.5m -> 1.6m, 恢复 14 -> 20 次) —— 松开一个约束只是让
    下一个(BaseObstacle)顶上来。这个脚本能告诉你谁在挡路,
    但不保证挡路的只有一个。
"""

import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node

from dwb_msgs.msg import LocalPlanEvaluation


class CriticProbe(Node):

    def __init__(self):
        super().__init__('dwb_critic_probe')
        self.frames = 0
        self.total_traj = 0
        self.rejected = 0
        self.chosen_vx = []
        self.best_vx = []
        self.diff = defaultdict(list)
        self.chosen_s = defaultdict(list)
        self.fast_s = defaultdict(list)
        self.create_subscription(LocalPlanEvaluation, '/evaluation',
                                 self._cb, 5)

    def _cb(self, msg):
        ts = msg.twists
        if not ts:
            return
        self.frames += 1
        self.total_traj += len(ts)
        self.rejected += sum(1 for t in ts if t.total < 0)

        valid = [t for t in ts if t.total >= 0]
        if not valid:
            return
        chosen = min(valid, key=lambda t: t.total)
        fastest = max(valid, key=lambda t: t.traj.velocity.x)
        self.chosen_vx.append(chosen.traj.velocity.x)
        self.best_vx.append(fastest.traj.velocity.x)

        cs = {s.name: s.raw_score * s.scale for s in chosen.scores}
        fs = {s.name: s.raw_score * s.scale for s in fastest.scores}
        for k in set(cs) | set(fs):
            self.chosen_s[k].append(cs.get(k, 0.0))
            self.fast_s[k].append(fs.get(k, 0.0))
            self.diff[k].append(fs.get(k, 0.0) - cs.get(k, 0.0))


def mean(a):
    return sum(a) / len(a) if a else 0.0


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = CriticProbe()
    print(f'采样 /evaluation {secs:.0f} s ...', flush=True)
    end = time.time() + secs
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.frames == 0:
        print('没收到 /evaluation。检查: 控制器在跑吗? '
              'FollowPath.publish_evaluation 是 true 吗?')
        rclpy.shutdown()
        return 1

    print()
    print('=' * 70)
    print(f'评估帧数 {node.frames}, 候选轨迹 {node.total_traj}, '
          f'被否决 {node.rejected} '
          f'({100 * node.rejected / node.total_traj:.0f} %)')
    print(f'实际选中的 vx  均值 {mean(node.chosen_vx):.3f} m/s')
    print(f'可选最大 vx    均值 {mean(node.best_vx):.3f} m/s')
    print()
    print(f'{"critic":<16}{"选中轨迹":>12}{"最快轨迹":>12}{"差值":>12}   说明')
    print('-' * 70)
    for name, d in sorted(node.diff.items(), key=lambda kv: -abs(mean(kv[1]))):
        c, f, dd = mean(node.chosen_s[name]), mean(node.fast_s[name]), mean(d)
        note = ''
        if dd > 1.0:
            note = '<= 它惩罚快轨迹'
        elif dd < -1.0:
            note = '(反而奖励快轨迹)'
        print(f'{name:<16}{c:>12.2f}{f:>12.2f}{dd:>12.2f}   {note}')
    print('-' * 70)
    print('差值 = 最快轨迹得分 - 选中轨迹得分。DWB 取总分最低者,')
    print('所以差值为正且最大的那个 critic, 就是让车不敢提速的原因。')
    print('=' * 70)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
