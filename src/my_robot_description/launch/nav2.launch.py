"""
Nav2 导航栈: AMCL 定位 + 全局/局部代价地图 + 规划 + 控制。

用法:
    # 终端1 仿真
    ros2 launch my_robot_description gazebo.launch.py
    # 终端2 导航(自带 RViz)
    ros2 launch my_robot_description nav2.launch.py

在 RViz 里用工具栏的 "2D Goal Pose" 点一个目标点即可
(那是 RViz 自带的 SetGoal 工具, 发到 /goal_pose;
 由 scripts/goal_relay.py 转成 NavigateToPose 动作 —— 原因见下)。
初始位姿不用点 —— nav2_params.yaml 里已经设了 set_initial_pose,
因为机器人在 gazebo.launch.py 里固定从原点出生。

为什么不直接用 nav2_bringup 的 navigation_launch.py:
    那个 launch 在 Jazzy 里会拉起 10 个 lifecycle 节点, 其中包含
    route_server(需要一份 graph geojson, launch 里并没有给默认值) 和
    docking_server。lifecycle_manager 要求 node_names 里每一个都激活成功,
    任何一个配不起来, 整个导航栈都停在 inactive —— 而且不会有明显报错,
    表现为"点了目标点没反应"。
    这里只起本项目真正需要的 6 个节点, 每一个都能解释清楚它在做什么。

cmd_vel 的走向:
    nav2_bringup 会把 controller 的 cmd_vel 重映射成 cmd_vel_nav, 再经
    velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel。
    这里不做重映射, controller_server 直接发 /cmd_vel, 正好对上
    gazebo.launch.py 里 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' 的桥接。
    消息类型也对得上: nav2_util::TwistPublisher 的 enable_stamped_cmd_vel
    默认 false, 发的是普通 Twist 而不是 TwistStamped。
    顺带一提, collision_monitor 的默认 base_frame_id 是 base_footprint,
    而本机器人的 TF 树是 odom -> base_link, 没有 base_footprint ——
    真要接那条链, 这里是第一个会静默卡住的地方。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    # 本项目自己的视图。曾经直接用 nav2 自带的 nav2_default_view.rviz,
    # 但那份配置假设官方那套完整节点都在:
    #   - Navigation 2 / Selector / Docking 三个面板要连 smoother_server 等,
    #     而本项目刻意没起它们, 于是 RViz 每 5 秒刷一次
    #     "smoother_server service not available / Failed to load plugins.
    #      Retrying..." —— 刷了三分钟, 真正的报错会被淹没在里面。
    #   - 还订阅着 /mobile_base/sensors/bumper_pointcloud, 那是 TurtleBot 的话题。
    #   - Realsense 相机组也是 TurtleBot 的, 本项目没有相机。
    # 这份是从官方那份裁出来的, 改动仅四处:
    #   去掉三个面板、去掉 bumper 与 Realsense 显示、开启 RobotModel。
    # RobotModel 那处不只是勾选: 它的描述话题原本是 Volatile, 而
    # robot_state_publisher 发 /robot_description 用的是 TRANSIENT_LOCAL
    # (启动时发一次然后锁存) —— Volatile 订阅者晚于那一刻加入就什么都收不到,
    # 即使勾上也是空的。所以同时把 Durability 改成 Transient Local。
    # 2D Goal Pose 工具(nav2_rviz_plugins/GoalTool)在 Tools 段, 不受影响。
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    use_slip_guard = LaunchConfiguration('slip_guard')

    # map_server 的 yaml_filename 必须是绝对路径, 所以在这里覆盖
    # nav2_params.yaml 里那个占位值(参数列表里靠后的字典优先)。
    localization_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),
    ]

    navigation_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),
    ]

    # 两个 lifecycle_manager 分开管: 定位起不来时, 导航侧的报错才不会
    # 淹没真正的原因。autostart=true 表示自动走完 configure->activate。
    lifecycle_managers = [
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),
    ]

    # 打滑守护。默认开启 —— 一个会悄悄把定位搞坏还报告"到达目标"的系统,
    # 比一个偶尔主动停下来的系统危险得多。实测 0 误报(自由行驶 25 次判定),
    # 检出时里程计误差 0.213m, 而不装它的那次故障积累了约 9m 才被发现。
    #
    # 它不是 Node() 而是 ExecuteProcess: 脚本放在 scripts/ 而不是 python 包里,
    # 没有 entry_point, 所以直接按 share 下的绝对路径跑。
    slip_guard = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'slip_monitor.py'),
             '--abort', '--quiet'],
        output='screen',
        condition=IfCondition(use_slip_guard),
    )

    # 目标中继: 把 RViz 标准 SetGoal 工具发的 /goal_pose 转成动作调用。
    # 为什么不用官方的 GoalTool + "Navigation 2" 面板: 那个面板还要连
    # follow_waypoints 和 smoother_server, 而本项目不起这两个节点, 面板
    # 加载不成功就会刷屏。删掉面板则点目标点毫无反应 —— 因为工具只记位姿,
    # 调动作的是面板。Nav2 里没有任何东西订阅 /goal_pose, 所以这一环得自己接。
    # 只在开 RViz 时才需要, 所以跟着 rviz 参数一起开关。
    goal_relay = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'goal_relay.py')],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='要加载的占据栅格 yaml'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否同时启动 RViz'),
        DeclareLaunchArgument(
            'slip_guard', default_value='true',
            description='打滑守护: 检出即取消目标并恢复定位'),
        *localization_nodes,
        *navigation_nodes,
        *lifecycle_managers,
        slip_guard,
        goal_relay,
        rviz,
    ])
