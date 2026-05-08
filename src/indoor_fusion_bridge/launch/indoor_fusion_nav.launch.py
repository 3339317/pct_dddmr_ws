"""
indoor_fusion_nav.launch.py
Unified launch file for the indoor fusion workspace.

Launches dddmr navigation stack + pct_dddmr_web UI + fusion bridge.

Usage:
  ros2 launch indoor_fusion_bridge indoor_fusion_nav.launch.py \
      pose_graph_dir:=/path/to/pose_graph \
      map_pcd:=/path/to/map.pcd
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =========================================================
    # Launch arguments
    # =========================================================
    pose_graph_dir = LaunchConfiguration("pose_graph_dir")
    map_pcd = LaunchConfiguration("map_pcd")
    use_rviz = LaunchConfiguration("use_rviz")
    default_pose_graph_dir = "/home/if/indoor_fusion_ws/maps/2026_05_07_15_58_12"
    default_map_pcd = os.path.join(default_pose_graph_dir, "map.pcd")

    # =========================================================
    # Config file path (fusion_params.yaml)
    # =========================================================
    pkg_fusion_config = FindPackageShare("indoor_fusion_config").find("indoor_fusion_config")
    fusion_config = os.path.join(pkg_fusion_config, "config", "fusion_params.yaml")

    # Alternative: find via share directory (installed)
    # For development, we use the source path directly

    # =========================================================
    # TF publishers
    # =========================================================
    # base_footprint → base_link (地面 → 机器人中心)
    tf_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="footprint2baselink",
        arguments=["0.0", "0", "0.24", "0.0", "0.0", "0", "base_footprint", "base_link"],
    )

    # base_link → livox_frame (adjust x, y, z, roll, pitch, yaw for your robot)
    # Default: Mid-360 mounted 0.3m forward, 0.86m above base_link (1.1m from ground)
    tf_base_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="baselink2livox",
        arguments=["0.3", "0", "0.86", "0.0", "0.0", "0", "base_link", "livox_frame"],
    )

    # =========================================================
    # dddmr nodes
    # =========================================================

    # 1. Feature extraction (lego_loam)
    mcl_feature = Node(
        package="lego_loam_bor",
        executable="mcl_feature",
        output="screen",
        parameters=[fusion_config],
        remappings=[
            ("/lslidar_point_cloud", "/livox/lidar"),
        ],
    )

    # 2. MCL 3D localization
    mcl_3dl = Node(
        package="mcl_3dl",
        executable="mcl_3dl",
        output="screen",
        parameters=[fusion_config, {"pose_graph_dir": pose_graph_dir}],
    )

    # 3. Global planner
    global_planner = Node(
        package="global_planner",
        executable="global_planner_node",
        output="screen",
        parameters=[fusion_config],
    )

    # 4. P2P move base (includes local planner, perception, trajectory generators)
    p2p_move_base = Node(
        package="p2p_move_base",
        executable="p2p_move_base_node",
        output="screen",
        parameters=[fusion_config],
    )

    # =========================================================
    # PCT DDDMR Web - Web App only (no controller/planner)
    # =========================================================
    web_app = Node(
        package="pct_dddmr_web",
        executable="web_app",
        name="pct_dddmr_web_web_app",
        output="screen",
    )

    # =========================================================
    # Fusion Bridge Node
    # =========================================================
    fusion_bridge = Node(
        package="indoor_fusion_bridge",
        executable="fusion_bridge_node",
        name="fusion_bridge_node",
        output="screen",
    )

    # =========================================================
    # Rviz (optional)
    # =========================================================
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
        # Arguments
        DeclareLaunchArgument(
            "pose_graph_dir",
            default_value=default_pose_graph_dir,
            description="Path to dddmr pose_graph directory for sub_maps",
        ),
        DeclareLaunchArgument(
            "map_pcd",
            default_value=default_map_pcd,
            description="Path to PCD map file for web UI (optional)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch rviz2 for visualization",
        ),

        # TF transforms (immediate)
        tf_base_footprint,
        tf_base_laser,

        # dddmr nodes
        mcl_feature,
        mcl_3dl,
        global_planner,
        p2p_move_base,

        # Indoor nav web app
        web_app,

        # Fusion bridge (after all other nodes are up)
        TimerAction(
            period=3.0,
            actions=[fusion_bridge],
        ),

        # Rviz (optional)
        rviz_node,
    ])
