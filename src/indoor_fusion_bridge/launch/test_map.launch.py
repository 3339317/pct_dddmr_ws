from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    web_app = Node(
        package="pct_dddmr_web",
        executable="web_app",
        name="pct_dddmr_web_web_app",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch rviz2 for visualization",
        ),
        web_app,
    ])
