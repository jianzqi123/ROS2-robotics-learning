import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.subscription = self.create_subscription(
            String,
            'battery_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        level_str = msg.data.split(': ')[1].replace('%', '')
        level = int(level_str)
        if level < 20:
            self.get_logger().warn(f'LOW BATTERY WARNING: {level}%')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
