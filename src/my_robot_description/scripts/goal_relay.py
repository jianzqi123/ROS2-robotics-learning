#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 /goal_pose 转成 NavigateToPose 动作调用。

用法(通常由 nav2.launch.py 自动起, 不用手敲):
    python3 scripts/goal_relay.py

为什么需要它:
    RViz 里点目标点有两条路。
    官方那条是 nav2_rviz_plugins 的 GoalTool + "Navigation 2" 面板:
    工具只负责记下一个位姿, 真正调用动作的是面板。
    但那个面板还要连 follow_waypoints(waypoint_follower) 和 smoother_server,
    而本项目刻意不起这两个节点 —— 于是面板加载失败, RViz 每 5 秒刷一次
    "smoother_server service not available / Failed to load plugins"。
    我曾经为了消掉刷屏把面板整个删掉, 结果点目标点毫无反应 ——
    用一个更严重的问题换掉了一个刷屏问题。

    另一条路是 RViz 自带的 rviz_default_plugins/SetGoal 工具:
    它把位姿直接发到 /goal_pose 话题。问题是 Nav2 里没有任何东西订阅
    /goal_pose(实测 `ros2 topic info /goal_pose` 报 Unknown topic),
    所以还需要有人把那个话题接到动作上 —— 就是这个脚本。

    选这条路而不是补两个节点, 是因为本项目的原则是"只起真正需要的节点,
    且每一个都能解释清楚它在做什么"。为了让一个面板加载成功而拉起
    两个用不上的服务器, 方向是反的。
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# 动作状态码 (action_msgs/msg/GoalStatus)
STATUS = {2: '执行中', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


class GoalRelay(Node):

    def __init__(self):
        super().__init__('goal_relay')
        # 注意成员名不能叫 self.handle —— rclpy.node.Node.handle 是只读属性,
        # 赋值会在构造时直接抛 AttributeError, 节点根本起不来。
        self.goal_handle = None
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        print('目标中继已就绪: 在 RViz 里用 "2D Goal Pose" 点一个目标点。',
              flush=True)

    def _on_goal(self, msg):
        if not self.nav.server_is_ready():
            # 不在回调里阻塞等服务器 —— 点了就该立刻有反馈, 哪怕是"还没就绪"
            print('  [中继] navigate_to_pose 还没就绪, 本次点击忽略。'
                  ' Nav2 起来了吗?', flush=True)
            return
        p = msg.pose.position
        yaw = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        print(f'  [中继] 收到目标 ({p.x:+.2f}, {p.y:+.2f}, '
              f'{math.degrees(yaw):+.0f}°), 转发给 Nav2', flush=True)

        goal = NavigateToPose.Goal()
        goal.pose = msg
        # 点新目标就顶掉旧的 —— 和 RViz 面板的行为一致, 也符合直觉
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.nav.send_goal_async(goal).add_done_callback(self._on_accepted)

    def _on_accepted(self, fut):
        h = fut.result()
        if h is None or not h.accepted:
            print('  [中继] 目标被拒绝', flush=True)
            self.goal_handle = None
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        st = fut.result().status
        print(f'  [中继] 导航结束: {STATUS.get(st, st)}', flush=True)
        self.goal_handle = None


def main():
    rclpy.init()
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # launch 在 SIGINT 超时后升级到 SIGTERM, 那时 rclpy 抛的是
        # ExternalShutdownException —— 那是正常收工, 不该吐 traceback。
        if 'Shutdown' not in type(exc).__name__:
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
