"""
PCT global planning + lightweight route tracking + web UI.

This launch assumes localization and the MID360 driver are already running on
the deployment computer:
  - /localization    nav_msgs/Odometry in map frame
  - /livox/lidar     Livox MID360 CustomMsg input used by mcl_feature
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
    use_route_tracker = LaunchConfiguration("use_route_tracker")
    publish_livox_tf = LaunchConfiguration("publish_livox_tf")
    publish_localization_tf = LaunchConfiguration("publish_localization_tf")
    local_lidar_topic = LaunchConfiguration("local_lidar_topic")
    obstacle_avoidance_enabled = LaunchConfiguration("obstacle_avoidance_enabled")
    obstacle_cloud_topic = LaunchConfiguration("obstacle_cloud_topic")

    pct_lib = PathJoinSubstitution([pct_root, "planner", "lib"])

    tf_base_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="baselink2livox",
        arguments=["0.3", "0", "0.86", "0.0", "0.0", "0", "base_link", "livox_frame"],
        condition=IfCondition(publish_livox_tf),
    )

    tf_body_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body2baselink",
        arguments=["0", "0", "0", "0", "0", "0", "body", "base_link"],
        condition=IfCondition(publish_livox_tf),
    )

    localization_tf_bridge = Node(
        package="pct_dddmr_nav",
        executable="localization_to_tf",
        name="localization_to_tf",
        output="screen",
        parameters=[{
            "input_topic": "/localization",
            "parent_frame_override": "map",
            "child_frame_override": "base_link",
            "use_topic_stamp": True,
        }],
        condition=IfCondition(publish_localization_tf),
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
            ("/lslidar_point_cloud", local_lidar_topic),
            ("/odom", "/localization"),
        ],
        condition=IfCondition(use_mcl_feature),
    )

    route_tracker = Node(
        package="indoor_fusion_bridge",
        executable="route_tracker_node",
        name="route_tracker_node",
        output="screen",
        parameters=[{
            "localization_topic": "/localization",
            "route_topic": "/pct_dddmr_web/controller/route",
            "config_topic": "/pct_dddmr_web/controller/config",
            "start_topic": "/pct_dddmr_web/controller/start",
            "stop_topic": "/pct_dddmr_web/controller/stop",
            "clear_topic": "/pct_dddmr_web/controller/clear",
            "cmd_vel_topic": "/cmd_vel",
            "state_topic": "/pct_dddmr_web/controller/state",
            "status_topic": "/pct_dddmr_web/controller/status",
            "done_topic": "/nav/done",
            "control_frequency": 25.0,
            "obstacle_cloud_topic": obstacle_cloud_topic,
            "obstacle_avoidance_enabled": obstacle_avoidance_enabled,
            "obstacle_confirm_frames": 4,
        }],
        condition=IfCondition(use_route_tracker),
    )

    web_app = Node(
        package="pct_dddmr_web",
        executable="web_app",
        name="pct_dddmr_web_web_app",
        output="screen",
        arguments=["--tomogram-path", tomogram_path],
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
        DeclareLaunchArgument("use_mcl_feature", default_value="false"),
        DeclareLaunchArgument("use_web", default_value="true"),
        DeclareLaunchArgument("use_route_tracker", default_value="true"),
        DeclareLaunchArgument("publish_livox_tf", default_value="true"),
        DeclareLaunchArgument("publish_localization_tf", default_value="false"),
        DeclareLaunchArgument("local_lidar_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("obstacle_cloud_topic", default_value="/segmented_cloud_pure"),
        DeclareLaunchArgument("obstacle_avoidance_enabled", default_value="false"),
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
        tf_body_base,
        localization_tf_bridge,
        pct_get_plan_server,
        tomogram_map_publisher,
        mcl_feature,
        TimerAction(period=2.0, actions=[route_tracker]),
        web_app,
        rviz_node,
    ])
