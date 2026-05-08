"""
FAST-LIO localization + DDDMR navigation + web UI.

This launch lets FAST-LIO-LOCALIZATION2 own localization and TF, while DDDMR
keeps global planning, local obstacle avoidance, path tracking, and web control.

Example:
  source /home/if/fast_ws/install/setup.bash
  source /home/if/indoor_fusion_ws/install/setup.bash
  ros2 launch indoor_fusion_bridge fastlio_dddmr_nav.launch.py \
      fastlio_map:=/home/if/indoor_fusion_ws/maps/2026_05_07_15_58_12/map.pcd \
      dddmr_map_dir:=/home/if/indoor_fusion_ws/maps/2026_05_07_15_58_12
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map_dir = "/home/if/indoor_fusion_ws/maps/2026_05_07_15_58_12"
    default_fastlio_map = os.path.join(default_map_dir, "map.pcd")

    fastlio_map = LaunchConfiguration("fastlio_map")
    dddmr_map_dir = LaunchConfiguration("dddmr_map_dir")
    dddmr_map_pcd = LaunchConfiguration("dddmr_map_pcd")
    dddmr_ground_pcd = LaunchConfiguration("dddmr_ground_pcd")
    use_rviz = LaunchConfiguration("use_rviz")
    use_fastlio_rviz = LaunchConfiguration("use_fastlio_rviz")
    use_mcl_feature = LaunchConfiguration("use_mcl_feature")

    pkg_fusion_config = FindPackageShare("indoor_fusion_config").find("indoor_fusion_config")
    fusion_config = os.path.join(pkg_fusion_config, "config", "fastlio_fusion_params.yaml")

    fastlio_share = get_package_share_directory("fast_lio_localization")
    fastlio_launch = os.path.join(fastlio_share, "launch", "localization.launch.py")

    fastlio_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "config_file": "mid360.yaml",
            "map": fastlio_map,
            "pcd_map_topic": "/fastlio_map",
            "rviz": use_fastlio_rviz,
        }.items(),
    )

    # FAST-LIO publishes map -> camera_init -> body. DDDMR expects map -> base_link.
    tf_body_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body2baselink",
        arguments=["0", "0", "0", "0", "0", "0", "body", "base_link"],
    )

    tf_base_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="baselink2livox",
        arguments=["0.3", "0", "0.86", "0.0", "0.0", "0", "base_link", "livox_frame"],
    )

    dddmr_map_publisher = Node(
        package="mcl_3dl",
        executable="pcl_publisher",
        name="pcl_publisher",
        output="screen",
        parameters=[{
            "global_frame": "map",
            "map_dir": dddmr_map_pcd,
            "ground_dir": dddmr_ground_pcd,
            "map_down_sample": 0.2,
            "ground_down_sample": 0.3,
        }],
    )

    mcl_feature = Node(
        package="lego_loam_bor",
        executable="mcl_feature",
        output="screen",
        parameters=[fusion_config],
        remappings=[
            ("/lslidar_point_cloud", "/livox/lidar"),
        ],
        condition=IfCondition(use_mcl_feature),
    )

    global_planner = Node(
        package="global_planner",
        executable="global_planner_node",
        output="screen",
        parameters=[fusion_config],
    )

    p2p_move_base = Node(
        package="p2p_move_base",
        executable="p2p_move_base_node",
        output="screen",
        parameters=[fusion_config],
    )

    web_app = Node(
        package="pct_dddmr_web",
        executable="web_app",
        name="pct_dddmr_web_web_app",
        output="screen",
    )

    fusion_bridge = Node(
        package="indoor_fusion_bridge",
        executable="fusion_bridge_node",
        name="fusion_bridge_node",
        output="screen",
        parameters=[{
            "enable_pose_bridge": False,
        }],
    )

    pkg_dddmr_beginner = FindPackageShare("dddmr_beginner_guide").find("dddmr_beginner_guide")
    rviz_config = os.path.join(pkg_dddmr_beginner, "rviz", "airy_tilt45_navigation.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "fastlio_map",
            default_value=default_fastlio_map,
            description="PCD map used by FAST-LIO localization.",
        ),
        DeclareLaunchArgument(
            "dddmr_map_dir",
            default_value=default_map_dir,
            description="Directory containing DDDMR-compatible map.pcd and ground.pcd.",
        ),
        DeclareLaunchArgument(
            "dddmr_map_pcd",
            default_value=PathJoinSubstitution([dddmr_map_dir, "map.pcd"]),
            description="PCD published as /mapcloud for DDDMR static layers.",
        ),
        DeclareLaunchArgument(
            "dddmr_ground_pcd",
            default_value=PathJoinSubstitution([dddmr_map_dir, "ground.pcd"]),
            description="PCD published as /mapground for web target picking and DDDMR ground layer.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch DDDMR RViz visualization.",
        ),
        DeclareLaunchArgument(
            "use_fastlio_rviz",
            default_value="false",
            description="Launch FAST-LIO RViz visualization.",
        ),
        DeclareLaunchArgument(
            "use_mcl_feature",
            default_value="true",
            description="Run DDDMR feature extraction on /livox/lidar for local obstacle input.",
        ),
        fastlio_localization,
        tf_body_to_base_link,
        tf_base_laser,
        dddmr_map_publisher,
        mcl_feature,
        TimerAction(period=2.0, actions=[global_planner]),
        TimerAction(period=3.0, actions=[p2p_move_base]),
        web_app,
        TimerAction(period=4.0, actions=[fusion_bridge]),
        rviz_node,
    ])
