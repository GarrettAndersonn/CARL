from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('carl_teleop'), 'config', 'teleop.yaml')
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy', output='screen'),
        Node(
            package='carl_teleop',
            executable='joy_teleop',
            name='carl_joy_teleop',
            parameters=[config],
            output='screen',
        ),
    ])
