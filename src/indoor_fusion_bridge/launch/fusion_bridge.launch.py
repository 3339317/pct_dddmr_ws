from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="indoor_fusion_bridge",
            executable="fusion_bridge_node",
            name="fusion_bridge_node",
            output="screen",
            parameters=[{"enable_pose_bridge": False}],
        ),
    ])
