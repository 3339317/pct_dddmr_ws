from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    web_node = Node(
        package="indoor_route_nav",
        executable="web_app",
        name="indoor_route_nav_web_app",
        output="screen",
    )

    return LaunchDescription([
        web_node,
    ])
