# ROS 2 node handling the gripper interface

import rclpy
from rclpy.node import Node
from wsg50_driver_pkg.wsg50_driver.wsg_interface import WSGInterface
from wsg50_driver_pkg.wsg50_driver.constants import *
from wsg50_driver_pkg.wsg50_driver.utils import parse_grip_state, exponential_backoff
from wsg50_driver_pkg.srv import SetWidth, GetStatus, Calibrate, RecoverConnection
import time

class GripperNode(Node):
    def __init__(self):
        super().__init__('wsg50_gripper_node')
        self.declare_parameter('ip')
        self.declare_parameter('port')
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        self.gripper = WSGInterface(ip, port)
        self.get_logger().info(f"Connecting to gripper at {ip}:{port}...")
        self._connect_with_retries()

        self.srv_set_width = self.create_service(SetWidth, 'set_width', self.set_width_cb)
        self.srv_get_status = self.create_service(GetStatus, 'get_status', self.get_status_cb)
        self.srv_calibrate = self.create_service(Calibrate, 'calibrate', self.calibrate_cb)
        self.srv_recover = self.create_service(RecoverConnection, 'recover_connection', self.recover_cb)

    def _connect_with_retries(self, max_retries=5):
        for attempt in range(max_retries):
            try:
                self.gripper.connect()
                self.get_logger().info("Gripper connection successful.")
                return
            except Exception as e:
                delay = exponential_backoff(attempt)
                self.get_logger().warn(f"Connection attempt {attempt + 1} failed: {e}. Retrying in {delay:.2f}s...")
                time.sleep(delay)
        self.get_logger().error("Failed to connect to the gripper after multiple attempts.")

    def set_width_cb(self, request, response):
        response.success = self.gripper.move_to_width(request.target_width)
        return response

    def get_status_cb(self, request, response):
        response.position = self.gripper.get_position()
        response.speed = self.gripper.get_speed()
        response.force = self.gripper.get_force()
        response.state = parse_grip_state(self.gripper.get_state())
        return response

    def calibrate_cb(self, request, response):
        response.success = self.gripper.calibrate()
        response.min_position = getattr(self.gripper, 'min_position', -1.0)
        response.max_position = getattr(self.gripper, 'max_position', -1.0)
        return response

    def recover_cb(self, request, response):
        response.success = self.gripper.reconnect()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

