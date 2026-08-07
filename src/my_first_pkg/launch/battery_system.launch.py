from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_pkg',
            executable='battery_publisher',
            name='battery_publisher',
            parameters=[
                {'publish_rate': 2.0},
                {'initial_level': 30}
            ]
        ),
        Node(
            package='my_first_pkg',
            executable='battery_monitor',
            name='battery_monitor'
        ),
    ])
