import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

from .pct_core import PCT_ROOT_DEFAULT, PCTTomogramPlanner
from .ros_utils import xyz_to_cloud


class TomogramMapPublisher(Node):
    def __init__(self):
        super().__init__("tomogram_map_publisher")

        self.declare_parameter("pct_root", PCT_ROOT_DEFAULT)
        self.declare_parameter("tomogram_path", os.path.join(PCT_ROOT_DEFAULT, "rsc", "tomogram", "building2_9.pickle"))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("mapground_topic", "/mapground")
        self.declare_parameter("mapcloud_topic", "/mapcloud")
        self.declare_parameter("publish_stride", 2)
        self.declare_parameter("ground_max_trav", 45.0)
        self.declare_parameter("obstacle_min_trav", 45.0)
        self.declare_parameter("publish_period", 2.0)

        self.map_frame = self.get_parameter("map_frame").value

        self.planner = PCTTomogramPlanner(pct_root=self.get_parameter("pct_root").value)
        tomogram_path = self.get_parameter("tomogram_path").value
        self.get_logger().info(f"Loading tomogram map: {tomogram_path}")
        self.planner.load(tomogram_path)

        stride = self.get_parameter("publish_stride").value
        self.ground_points = self.planner.traversable_points(
            max_trav=self.get_parameter("ground_max_trav").value,
            stride=stride,
        )
        self.obstacle_points = self.planner.obstacle_points(
            min_trav=self.get_parameter("obstacle_min_trav").value,
            stride=stride,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ground_pub = self.create_publisher(PointCloud2, self.get_parameter("mapground_topic").value, qos)
        self.cloud_pub = self.create_publisher(PointCloud2, self.get_parameter("mapcloud_topic").value, qos)

        self.timer = self.create_timer(
            float(self.get_parameter("publish_period").value),
            self.publish_maps,
        )
        self.publish_maps()
        self.get_logger().info(
            f"Tomogram map publisher ready: ground={len(self.ground_points)} pts, "
            f"cloud={len(self.obstacle_points)} pts"
        )

    def publish_maps(self):
        stamp = self.get_clock().now().to_msg()
        ground_msg = xyz_to_cloud(self.ground_points, frame_id=self.map_frame, stamp=stamp)
        cloud_msg = xyz_to_cloud(self.obstacle_points, frame_id=self.map_frame, stamp=stamp)
        self.ground_pub.publish(ground_msg)
        self.cloud_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TomogramMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
