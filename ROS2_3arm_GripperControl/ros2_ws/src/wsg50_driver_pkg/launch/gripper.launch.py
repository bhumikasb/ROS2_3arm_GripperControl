from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wsg50_driver_pkg'),
        'config',
        'gripper_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='wsg50_driver_pkg',
            executable='gripper_node',
            name='gripper_node',
            output='screen',
            parameters=[config]
        )
    ])