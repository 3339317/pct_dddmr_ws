#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from livox_ros_driver2.msg import CustomMsg


class LivoxCustomToPointCloud2(Node):
    def __init__(self):
        super().__init__("livox_custom_to_pointcloud2")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_points")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("include_time", True)
        self.declare_parameter("include_line", True)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.sub = self.create_subscription(CustomMsg, input_topic, self.cloud_callback, qos)

        self.get_logger().info(
            f"Converting Livox CustomMsg {input_topic} -> PointCloud2 {output_topic}"
        )

    def cloud_callback(self, msg: CustomMsg):
        frame_id = self.get_parameter("frame_id").value or msg.header.frame_id
        include_time = bool(self.get_parameter("include_time").value)
        include_line = bool(self.get_parameter("include_line").value)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        offset = 16
        if include_time:
            fields.append(PointField(name="time", offset=offset, datatype=PointField.FLOAT32, count=1))
            offset += 4
        if include_line:
            fields.append(PointField(name="ring", offset=offset, datatype=PointField.UINT16, count=1))

        points = []
        for point in msg.points:
            if not (math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)):
                continue

            row = [
                float(point.x),
                float(point.y),
                float(point.z),
                float(point.reflectivity),
            ]
            if include_time:
                row.append(float(point.offset_time) * 1e-9)
            if include_line:
                row.append(int(point.line))
            points.append(row)

        self.pub.publish(point_cloud2.create_cloud(header, fields, points))


def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud2()
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
