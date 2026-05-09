"""
Start Livox MID360 driver in Livox CustomMsg mode.

The driver publishes /livox/lidar as livox_ros_driver2/msg/CustomMsg, which
keeps FAST-LIO localization on the higher-quality native Livox input. DDDMR
local obstacle processing subscribes to the same CustomMsg topic directly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    livox_share = get_package_share_directory("livox_ros_driver2")
    user_config_path = os.path.join(livox_share, "config", "MID360_config.json")

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format": 1,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "lvx_file_path": "/home/livox/livox_test.lvx",
            "user_config_path": user_config_path,
            "cmdline_input_bd_code": "livox0000000001",
        }],
    )

    return LaunchDescription([
        livox_driver,
    ])
