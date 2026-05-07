from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    web_app = Node(
        package="indoor_route_nav",
        executable="web_app",
        name="indoor_route_nav_web_app",
        output="screen",
    )

    map_preprocessor_config = os.path.join(
        FindPackageShare("indoor_route_nav").find("indoor_route_nav"),
        "config", "map_preprocessor.yaml"
    )
    map_preprocessor = Node(
        package="indoor_route_nav",
        executable="indoor_map_preprocessor_node",
        name="indoor_map_preprocessor",
        output="screen",
        arguments=["--config", map_preprocessor_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch rviz2 for visualization",
        ),
        map_preprocessor,
        web_app,
    ])
