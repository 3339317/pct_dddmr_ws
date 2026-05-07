"""
view_pose_graph.launch.py
Simple launch file to view pose_graph map in rviz2.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pose_graph_dir = LaunchConfiguration("pose_graph_dir")
    
    # TF publishers
    tf_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="baselink2footprint",
        arguments=["0.0", "0", "-0.24", "0.0", "0.0", "0", "base_link", "base_footprint"],
    )
    
    # mcl_3dl node
    mcl_3dl = Node(
        package="mcl_3dl",
        executable="mcl_3dl",
        name="mcl_3dl",
        output="screen",
        parameters=[{"pose_graph_dir": pose_graph_dir}],
    )
    
    # rviz2
    pkg_dddmr_beginner = FindPackageShare("dddmr_beginner_guide").find("dddmr_beginner_guide")
    rviz_config = os.path.join(pkg_dddmr_beginner, "rviz", "airy_tilt45_navigation.rviz")
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "pose_graph_dir",
            default_value="/home/if/pose_graph_test",
            description="Path to pose_graph directory",
        ),
        tf_base_footprint,
        mcl_3dl,
        rviz_node,
    ])