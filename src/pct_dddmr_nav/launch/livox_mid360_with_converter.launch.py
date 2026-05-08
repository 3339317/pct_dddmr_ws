"""
Start Livox MID360 driver in CustomMsg mode and publish an additional
PointCloud2 topic for DDDMR local obstacle processing.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("livox_ros_driver2"),
            "/launch_ROS2/msg_MID360_launch.py",
        ])
    )

    converter = Node(
        package="pct_dddmr_nav",
        executable="livox_custom_to_pointcloud2",
        name="livox_custom_to_pointcloud2",
        output="screen",
        parameters=[{
            "input_topic": "/livox/lidar",
            "output_topic": "/livox/lidar_points",
            "frame_id": "livox_frame",
            "include_time": True,
            "include_line": True,
        }],
    )

    return LaunchDescription([
        livox_driver,
        converter,
    ])
