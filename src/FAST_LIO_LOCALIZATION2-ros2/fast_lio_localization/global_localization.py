#!/usr/bin/env python3

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import numpy as np
if not hasattr(np, "float"):
    np.float = float
import tf2_ros
import tf_transformations
import ros2_numpy
from sensor_msgs_py import point_cloud2


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.initialized = False
        self.consecutive_failures = 0
        self.last_fitness = 0.0
        self.max_failures_before_recovery = 3
        self.last_wait_initial_pose_log_time = 0.0
        self.last_scan_cache_time = 0.0

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("publish_debug_clouds", False),
                ("scan_cache_rate", 2.0),
            ],
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.publish_debug_clouds = bool(self.get_parameter("publish_debug_clouds").value)
        self.scan_cache_period = 1.0 / max(0.1, float(self.get_parameter("scan_cache_rate").value))
        self.pub_pc_in_map = None
        self.pub_submap = None
        if self.publish_debug_clouds:
            self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
            self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        self.get_logger().info("Waiting for global map...")
        # global_map_msg = wait_for_message(msg_type = PointCloud2, node = self, topic = "/cloud_pcd")[1]
        # self.initialize_global_map(global_map_msg)
        
        self.initialize_global_map()
        self.get_logger().info("Global map received.")
        
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, 10)

        self.timer_localisation = self.create_timer(1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback)
        # self.timer_global_map = self.create_timer(1/ self.get_parameter("freq_global_map").value, self.global_map_callback)

    def global_map_callback(self):
        # self.get_logger().info(np.array(self.global_map.points).shape)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))
        
    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def msg_to_array(self, pc_msg):
        pc_array = ros2_numpy.numpify(pc_msg)
        return pc_array["xyz"]
    
    def registration_at_scale(self, scan, map, initial, scale):
        result_icp = o3d.pipelines.registration.registration_icp(
        self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale),
        self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale),
        1.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness
            
    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def publish_point_cloud(self, publisher, header, pc):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        points = pc[:, :3].astype(np.float32, copy=False).tolist()
        msg = point_cloud2.create_cloud(header, fields, points)
        msg.header = header
            
        publisher.publish(msg)
        
    def crop_global_map_in_FOV(self, pose_estimation):
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        if self.get_parameter("fov").value > 3.14:
            indices = np.where(
                (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        else:
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)
                & (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        header = self.cur_odom.header
        header.frame_id = "map"
        if self.publish_debug_clouds and self.pub_submap is not None:
            self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        scan_tobe_mapped = copy.copy(self.cur_scan)
        threshold = self.get_parameter("localization_threshold").value

        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)
        transformation, _ = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=1)

        if fitness > threshold:
            self.T_map_to_odom = transformation
            self.consecutive_failures = 0
            self.last_fitness = fitness
            self.publish_odom(transformation)
            return

        self.get_logger().warn(f"ICP fitness {fitness:.3f} < threshold {threshold}, attempting recovery...")
        self.consecutive_failures += 1

        global_map_down = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value * 4.0)
        scan_down = self.voxel_down_sample(scan_tobe_mapped, self.get_parameter("scan_voxel_size").value * 4.0)
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_down, initial=pose_estimation, scale=3)

        if fitness > threshold * 0.6:
            transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_down, initial=transformation, scale=2)
            transformation, fitness = self.registration_at_scale(scan_tobe_mapped, self.global_map, initial=transformation, scale=1)
            if fitness > threshold:
                self.T_map_to_odom = transformation
                self.consecutive_failures = 0
                self.last_fitness = fitness
                self.get_logger().info(f"Recovery successful! fitness={fitness:.3f}, position=({transformation[0,3]:.2f}, {transformation[1,3]:.2f})")
                self.publish_odom(transformation)
                return

        self.last_fitness = fitness
        if self.consecutive_failures > self.max_failures_before_recovery:
            self.get_logger().error(f"Localization lost! {self.consecutive_failures} consecutive failures. Please provide initial pose via /initialpose or Rviz2.")
        else:
            self.get_logger().warn(f"Recovery failed. fitness={fitness:.3f}, failures={self.consecutive_failures}/{self.max_failures_before_recovery}")

    def voxel_down_sample(self, pcd, voxel_size):
        # print(pcd)
        
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        
        except Exception as e:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg
        
    def cb_save_cur_scan(self, msg):
        now = time.monotonic()
        if now - self.last_scan_cache_time < self.scan_cache_period:
            return
        self.last_scan_cache_time = now

        pc = self.msg_to_array(msg)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc)
        if self.publish_debug_clouds and self.pub_pc_in_map is not None:
            self.publish_point_cloud(self.pub_pc_in_map, msg.header, pc)
        
    def initialize_global_map(self): #, pc_msg):
        # self.global_map = o3d.geometry.PointCloud()
        # self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        self.global_map = o3d.io.read_point_cloud(self.get_parameter("pcd_map_path").value)
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        # o3d.io.write_point_cloud("/home/wheelchair2/laksh_ws/pcds/lab_map_with_outside_corridor (with ground pcd)_downsampled.pcd", self.global_map)
        self.get_logger().info("Global map received.")

    def cb_initialize_pose(self, msg):
        T_map_to_base = self.pose_to_mat(msg.pose.pose)
        if self.cur_odom is not None:
            T_camera_init_to_body = self.pose_to_mat(self.cur_odom.pose.pose)
            T_map_to_camera_init = np.matmul(T_map_to_base, self.inverse_se3(T_camera_init_to_body))
        else:
            T_map_to_camera_init = T_map_to_base
        self.initialized = True
        self.consecutive_failures = 0
        self.get_logger().info(f"Initial pose received, triggering relocation at map→camera_init: {T_map_to_camera_init[:3, 3]}")
        
        if self.cur_scan is not None:
            self.global_localization(T_map_to_camera_init)
            
    def publish_odom(self, transform):
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        quat = tf_transformations.quaternion_from_matrix(transform)
        odom_msg.pose.pose = Pose(
            position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
            orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
        )
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        self.pub_map_to_odom.publish(odom_msg)

    def localisation_timer_callback(self):
        if not self.initialized:
            now = time.monotonic()
            if now - self.last_wait_initial_pose_log_time > 10.0:
                self.get_logger().info("Waiting for initial pose...")
                self.last_wait_initial_pose_log_time = now
            return
        
        if self.cur_scan is not None:
            self.global_localization(self.T_map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
