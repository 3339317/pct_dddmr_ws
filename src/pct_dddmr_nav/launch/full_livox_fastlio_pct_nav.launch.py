"""
One-command launch for the concentrated navigation workspace:

- Livox MID360 driver in PointCloud2 mode
- FAST-LIO localization stack
- PCT global planner + DDDMR local navigation + Web UI
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    map_path = LaunchConfiguration("map")
    tomogram_path = LaunchConfiguration("tomogram_path")
    start_livox = LaunchConfiguration("start_livox")
    start_fastlio = LaunchConfiguration("start_fastlio")
    start_nav = LaunchConfiguration("start_nav")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mcl_feature = LaunchConfiguration("use_mcl_feature")
    obstacle_avoidance_enabled = LaunchConfiguration("obstacle_avoidance_enabled")
    obstacle_cloud_topic = LaunchConfiguration("obstacle_cloud_topic")

    pct_share = get_package_share_directory("pct_dddmr_nav")
    fastlio_share = get_package_share_directory("fast_lio_localization")

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pct_share, "launch", "livox_mid360_with_converter.launch.py")
        ),
        condition=IfCondition(start_livox),
    )

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "map": map_path,
            "rviz": use_rviz,
        }.items(),
        condition=IfCondition(start_fastlio),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pct_share, "launch", "pct_dddmr_nav.launch.py")
        ),
        launch_arguments={
            "tomogram_path": tomogram_path,
            "use_rviz": "false",
            "local_lidar_topic": "/livox/lidar",
            "use_mcl_feature": use_mcl_feature,
            "obstacle_avoidance_enabled": obstacle_avoidance_enabled,
            "obstacle_cloud_topic": obstacle_cloud_topic,
        }.items(),
        condition=IfCondition(start_nav),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="", description="PCD map for FAST-LIO localization"),
        DeclareLaunchArgument("tomogram_path", default_value="", description="PCT .pickle tomogram map"),
        DeclareLaunchArgument("start_livox", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_mcl_feature", default_value="false"),
        DeclareLaunchArgument("obstacle_avoidance_enabled", default_value="false"),
        DeclareLaunchArgument("obstacle_cloud_topic", default_value="/segmented_cloud_pure"),
        livox_launch,
        TimerAction(period=2.0, actions=[fastlio_launch]),
        TimerAction(period=5.0, actions=[nav_launch]),
    ])
