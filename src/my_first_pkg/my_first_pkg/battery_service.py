import rclpy
from rclpy.node import Node
from my_first_pkg_interfaces.srv import CheckBattery

class BatteryService(Node):
    def __init__(self):
        super().__init__('battery_service')
        self.srv = self.create_service(
            CheckBattery,
            'check_battery',
            self.check_battery_callback)
        self.current_battery = 45

    def check_battery_callback(self, request, response):
        minutes_needed = request.required_minutes
        minutes_available = self.current_battery
        if minutes_available >= minutes_needed:
            response.can_complete = True
            response.message = f'OK, battery has {minutes_available} min, task needs {minutes_needed} min'
        else:
            response.can_complete = False
            response.message = f'NOT ENOUGH, battery has {minutes_available} min, task needs {minutes_needed} min'
        self.get_logger().info(f'Request: {minutes_needed} min -> {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BatteryService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
