from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    web_node = Node(
        package="pct_dddmr_web",
        executable="web_app",
        name="pct_dddmr_web_web_app",
        output="screen",
    )

    return LaunchDescription([
        web_node,
    ])
