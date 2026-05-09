#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class LocalizationToTf(Node):
    def __init__(self):
        super().__init__("localization_to_tf")

        self.declare_parameter("input_topic", "/localization")
        self.declare_parameter("parent_frame_override", "")
        self.declare_parameter("child_frame_override", "base_link")
        self.declare_parameter("use_topic_stamp", True)

        input_topic = str(self.get_parameter("input_topic").value)
        self.parent_override = str(self.get_parameter("parent_frame_override").value)
        self.child_override = str(self.get_parameter("child_frame_override").value)
        self.use_topic_stamp = bool(self.get_parameter("use_topic_stamp").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, input_topic, self.odom_callback, 10)

        self.get_logger().info(
            f"Publishing TF from {input_topic} with parent='{self.parent_override or '<topic>'}' "
            f"child='{self.child_override or '<topic>'}'"
        )

    def odom_callback(self, msg: Odometry):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp if self.use_topic_stamp else self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_override or msg.header.frame_id or "map"
        tf_msg.child_frame_id = self.child_override or msg.child_frame_id or "base_link"

        pose = msg.pose.pose
        tf_msg.transform.translation.x = pose.position.x
        tf_msg.transform.translation.y = pose.position.y
        tf_msg.transform.translation.z = pose.position.z
        tf_msg.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationToTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
