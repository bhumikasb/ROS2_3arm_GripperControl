import rclpy
from rclpy.node import Node
from wsg50_driver_pkg.srv import SetWidth
import sys

class GripperControlClient(Node):
    def __init__(self):
        super().__init__('gripper_control_client')
        self.cli = self.create_client(SetWidth, 'set_width')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')
        self.req = SetWidth.Request()

    def send_request(self, width_mm):
        self.req.width_mm = float(width_mm)
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(f'Success: {result.message}')
            else:
                self.get_logger().error(f'Failed: {result.message}')
        else:
            self.get_logger().error('Service call failed.')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("Usage: ros2 run wsg50_driver_pkg gripper_control <width_mm>")
        return

    width_mm = sys.argv[1]
    node = GripperControlClient()
    node.send_request(width_mm)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
