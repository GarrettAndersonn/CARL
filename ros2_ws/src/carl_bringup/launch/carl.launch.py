"""Bring up the core CARL stack: just the bridge.

Teleop is launched separately (keyboard_teleop needs its own terminal, and joy
teleop is optional). Typical workflow:

    Terminal 1:  ros2 launch carl_bringup carl.launch.py
    Terminal 2:  ros2 run   carl_teleop  keyboard_teleop
    Terminal 3:  ros2 topic echo /carl/heartbeat     # (optional)

For gamepad teleop, replace terminal 2 with:
    ros2 launch carl_teleop teleop.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bridge_launch = os.path.join(
        get_package_share_directory('carl_bridge'), 'launch', 'bridge.launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),
    ])
