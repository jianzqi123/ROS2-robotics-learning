import sys
import rclpy
from rclpy.node import Node
from my_first_pkg_interfaces.srv import CheckBattery

class BatteryClient(Node):
    def __init__(self):
        super().__init__('battery_client')
        self.client = self.create_client(CheckBattery, 'check_battery')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, minutes):
        request = CheckBattery.Request()
        request.required_minutes = minutes
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = BatteryClient()

    minutes = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    response = node.send_request(minutes)

    node.get_logger().info(f'Result: can_complete={response.can_complete}, message="{response.message}"')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
