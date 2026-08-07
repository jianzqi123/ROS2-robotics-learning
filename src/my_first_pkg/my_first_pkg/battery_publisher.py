import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')

        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('initial_level', 100)

        rate = self.get_parameter('publish_rate').value
        self.battery_level = self.get_parameter('initial_level').value

        self.publisher_ = self.create_publisher(String, 'battery_status', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Battery: {self.battery_level}%'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.battery_level -= 1
        if self.battery_level < 0:
            self.battery_level = 100

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



