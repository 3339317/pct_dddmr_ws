"""
PCT global planning + DDDMR local obstacle avoidance/path tracking + web UI.

This launch assumes localization and the MID360 driver are already running on
the deployment computer:
  - /localization    nav_msgs/Odometry in map frame
  - /livox/lidar     Livox Mid360 PointCloud2/CustomMsg input used by mcl_feature
  - TF map -> base_link, or equivalent pose TF from your localization stack
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pct_dddmr_nav")
    pct_params = os.path.join(pkg_share, "config", "pct_dddmr_params.yaml")
    dddmr_params = os.path.join(pkg_share, "config", "dddmr_local_params.yaml")
    default_pct_root = os.path.join(pkg_share, "vendor", "pct_planner")
    default_pct_lib = os.path.join(default_pct_root, "planner", "lib")
    pct_ld_paths = [
        default_pct_lib,
        os.path.join(default_pct_lib, "build", "src", "ele_planner"),
        os.path.join(default_pct_lib, "build", "src", "a_star"),
        os.path.join(default_pct_lib, "build", "src", "trajectory_optimization"),
        os.path.join(default_pct_lib, "build", "src", "map_manager"),
        os.path.join(default_pct_lib, "build", "src", "common", "smoothing"),
        os.path.join(default_pct_lib, "3rdparty", "gtsam-4.1.1", "install", "lib"),
        os.path.join(default_pct_lib, "3rdparty", "osqp", "install", "lib"),
    ]

    pct_root = LaunchConfiguration("pct_root")
    tomogram_path = LaunchConfiguration("tomogram_path")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mcl_feature = LaunchConfiguration("use_mcl_feature")
    use_web = LaunchConfiguration("use_web")
    publish_livox_tf = LaunchConfiguration("publish_livox_tf")

    pct_lib = PathJoinSubstitution([pct_root, "planner", "lib"])

    tf_base_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="baselink2livox",
        arguments=["0.3", "0", "0.86", "0.0", "0.0", "0", "base_link", "livox_frame"],
        condition=IfCondition(publish_livox_tf),
    )

    pct_get_plan_server = Node(
        package="pct_dddmr_nav",
        executable="pct_get_plan_server",
        name="pct_get_plan_server",
        output="screen",
        parameters=[
            pct_params,
            {
                "pct_root": pct_root,
                "tomogram_path": tomogram_path,
            },
        ],
    )

    tomogram_map_publisher = Node(
        package="pct_dddmr_nav",
        executable="tomogram_map_publisher",
        name="tomogram_map_publisher",
        output="screen",
        parameters=[
            pct_params,
            {
                "pct_root": pct_root,
                "tomogram_path": tomogram_path,
            },
        ],
    )

    mcl_feature = Node(
        package="lego_loam_bor",
        executable="mcl_feature",
        output="screen",
        parameters=[dddmr_params],
        remappings=[
            ("/lslidar_point_cloud", "/livox/lidar"),
        ],
        condition=IfCondition(use_mcl_feature),
    )

    p2p_move_base = Node(
        package="p2p_move_base",
        executable="p2p_move_base_node",
        output="screen",
        parameters=[dddmr_params],
    )

    web_app = Node(
        package="pct_dddmr_web",
        executable="web_app",
        name="pct_dddmr_web_web_app",
        output="screen",
        arguments=["--tomogram-path", tomogram_path],
        condition=IfCondition(use_web),
    )

    fusion_bridge = Node(
        package="indoor_fusion_bridge",
        executable="fusion_bridge_node",
        name="fusion_bridge_node",
        output="screen",
        parameters=[{"enable_pose_bridge": False}],
        condition=IfCondition(use_web),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("pct_root", default_value=default_pct_root),
        DeclareLaunchArgument(
            "tomogram_path",
            default_value=PathJoinSubstitution([pct_root, "rsc", "tomogram", "building2_9.pickle"]),
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_mcl_feature", default_value="true"),
        DeclareLaunchArgument("use_web", default_value="true"),
        DeclareLaunchArgument("publish_livox_tf", default_value="true"),
        SetEnvironmentVariable(
            "PYTHONPATH",
            [pct_lib, ":", os.environ.get("PYTHONPATH", "")],
        ),
        SetEnvironmentVariable(
            "LD_LIBRARY_PATH",
            [":".join(pct_ld_paths), ":", os.environ.get("LD_LIBRARY_PATH", "")],
        ),
        SetEnvironmentVariable("PCT_TOMOGRAM_PATH", tomogram_path),
        tf_base_laser,
        pct_get_plan_server,
        tomogram_map_publisher,
        mcl_feature,
        TimerAction(period=2.0, actions=[p2p_move_base]),
        web_app,
        TimerAction(period=3.0, actions=[fusion_bridge]),
        rviz_node,
    ])
