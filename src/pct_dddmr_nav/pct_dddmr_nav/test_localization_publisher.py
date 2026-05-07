#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw_rad):
    return (0.0, 0.0, math.sin(yaw_rad * 0.5), math.cos(yaw_rad * 0.5))


class TestLocalizationPublisher(Node):
    def __init__(self, args):
        super().__init__("test_localization_publisher")
        self.args = args
        self.pub = self.create_publisher(Odometry, args.topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / args.rate, self.publish_pose)
        self.get_logger().info(
            f"Publishing {args.topic} and TF {args.frame_id}->{args.child_frame_id}: "
            f"x={args.x:.2f}, y={args.y:.2f}, z={args.z:.2f}, yaw={args.yaw:.1f}deg, "
            f"rate={args.rate:.1f}Hz"
        )

    def current_pose(self):
        if self.args.circle_radius <= 0.0:
            return self.args.x, self.args.y, self.args.z, math.radians(self.args.yaw)

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        angle = elapsed * self.args.circle_speed
        x = self.args.x + self.args.circle_radius * math.cos(angle)
        y = self.args.y + self.args.circle_radius * math.sin(angle)
        yaw = angle + math.pi * 0.5
        return x, y, self.args.z, yaw

    def publish_pose(self):
        stamp = self.get_clock().now().to_msg()
        x, y, z, yaw = self.current_pose()
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.args.frame_id
        odom.child_frame_id = self.args.child_frame_id
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = self.args.cov_xy
        odom.pose.covariance[7] = self.args.cov_xy
        odom.pose.covariance[14] = self.args.cov_z
        odom.pose.covariance[35] = self.args.cov_yaw
        self.pub.publish(odom)

        if self.args.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.args.frame_id
            tf_msg.child_frame_id = self.args.child_frame_id
            tf_msg.transform.translation.x = float(x)
            tf_msg.transform.translation.y = float(y)
            tf_msg.transform.translation.z = float(z)
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish fake /localization odometry and map->base_link TF for PCT/DDDMR tests."
    )
    parser.add_argument("--topic", default="/localization", help="Odometry topic")
    parser.add_argument("--frame-id", default="map", help="Parent frame")
    parser.add_argument("--child-frame-id", default="base_link", help="Robot frame")
    parser.add_argument("--x", type=float, default=0.0, help="X position in map")
    parser.add_argument("--y", type=float, default=0.0, help="Y position in map")
    parser.add_argument("--z", type=float, default=0.0, help="Z position in map")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle in degrees")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate in Hz")
    parser.add_argument("--no-tf", action="store_false", dest="publish_tf", help="Do not publish TF")
    parser.add_argument("--circle-radius", type=float, default=0.0, help="Move on a circle when > 0")
    parser.add_argument("--circle-speed", type=float, default=0.15, help="Circle angular speed in rad/s")
    parser.add_argument("--cov-xy", type=float, default=0.01, help="X/Y covariance")
    parser.add_argument("--cov-z", type=float, default=0.01, help="Z covariance")
    parser.add_argument("--cov-yaw", type=float, default=0.01, help="Yaw covariance")
    args = parser.parse_args()
    args.rate = max(0.1, float(args.rate))
    args.circle_radius = max(0.0, float(args.circle_radius))
    return args


def main(args=None):
    parsed = parse_args()
    rclpy.init(args=args)
    node = TestLocalizationPublisher(parsed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
