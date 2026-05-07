import os
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from dddmr_sys_core.action import GetPlan
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from .pct_core import PCT_ROOT_DEFAULT, PCTTomogramPlanner
from .ros_utils import trajectory_to_path


class PCTGetPlanServer(Node):
    def __init__(self):
        super().__init__("pct_get_plan_server")

        self.declare_parameter("pct_root", PCT_ROOT_DEFAULT)
        self.declare_parameter("tomogram_path", os.path.join(PCT_ROOT_DEFAULT, "rsc", "tomogram", "building2_9.pickle"))
        self.declare_parameter("action_name", "get_dwa_plan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("localization_topic", "/localization")
        self.declare_parameter("start_source", "localization")
        self.declare_parameter("default_start_z", 0.0)
        self.declare_parameter("default_goal_z", 0.0)
        self.declare_parameter("pose_z_offset", 0.0)
        self.declare_parameter("goal_z_offset", 0.0)
        self.declare_parameter("use_quintic", True)
        self.declare_parameter("max_heading_rate", 10.0)
        self.declare_parameter("robot_radius_cells", 20)
        self.declare_parameter("safety_margin_cells", 15)
        self.declare_parameter("reference_height", 0.2)
        self.declare_parameter("publish_global_path_topic", "/global_path")
        self.declare_parameter("publish_pct_path_topic", "/pct_path")

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.start_source = self.get_parameter("start_source").value
        self.default_start_z = float(self.get_parameter("default_start_z").value)
        self.default_goal_z = float(self.get_parameter("default_goal_z").value)
        self.pose_z_offset = float(self.get_parameter("pose_z_offset").value)
        self.goal_z_offset = float(self.get_parameter("goal_z_offset").value)

        self.current_odom = None
        self.current_layer = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.planner = PCTTomogramPlanner(
            pct_root=self.get_parameter("pct_root").value,
            use_quintic=self.get_parameter("use_quintic").value,
            max_heading_rate=self.get_parameter("max_heading_rate").value,
            robot_radius_cells=self.get_parameter("robot_radius_cells").value,
            safety_margin_cells=self.get_parameter("safety_margin_cells").value,
            reference_height=self.get_parameter("reference_height").value,
        )

        tomogram_path = self.get_parameter("tomogram_path").value
        self.get_logger().info(f"Loading PCT tomogram: {tomogram_path}")
        self.planner.load(tomogram_path)
        self.get_logger().info(
            f"PCT ready: resolution={self.planner.resolution:.3f}, "
            f"slices={self.planner.n_slice}, map_dim={self.planner.map_dim}"
        )

        qos_transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.global_path_pub = self.create_publisher(
            Path, self.get_parameter("publish_global_path_topic").value, qos_transient
        )
        self.pct_path_pub = self.create_publisher(
            Path, self.get_parameter("publish_pct_path_topic").value, qos_transient
        )

        self.create_subscription(
            Odometry,
            self.get_parameter("localization_topic").value,
            self.localization_callback,
            10,
        )

        action_name = self.get_parameter("action_name").value
        self.action_server = ActionServer(
            self,
            GetPlan,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(f"PCT GetPlan action server ready: {action_name}")

    def localization_callback(self, msg):
        self.current_odom = msg
        pos = msg.pose.pose.position
        try:
            self.current_layer = self.planner.pose_to_layer(
                [pos.x, pos.y],
                pos.z,
                z_offset=self.pose_z_offset,
                prev_layer=self.current_layer,
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"Cannot infer current PCT layer yet: {exc}")

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def get_start_xyz(self, request_start):
        if self.start_source == "request" and request_start.header.frame_id:
            p = request_start.pose.position
            return np.array([p.x, p.y, p.z], dtype=np.float64), None

        if self.start_source == "tf":
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.robot_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                t = transform.transform.translation
                return np.array([t.x, t.y, t.z], dtype=np.float64), None
            except TransformException as exc:
                self.get_logger().warn(f"TF start lookup failed, fallback to localization/request: {exc}")

        if self.current_odom is not None:
            p = self.current_odom.pose.pose.position
            return np.array([p.x, p.y, p.z], dtype=np.float64), self.current_layer

        if request_start.header.frame_id:
            p = request_start.pose.position
            return np.array([p.x, p.y, p.z], dtype=np.float64), None

        raise RuntimeError("No start pose available. Need /localization, TF map->base_link, or request.start.")

    def goal_xyz_from_request(self, request_goal):
        p = request_goal.pose.position
        z = float(p.z)
        if abs(z) < 1e-6:
            z = self.default_goal_z
        return np.array([p.x, p.y, z], dtype=np.float64)

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        result = GetPlan.Result()

        if not request.activate_threading:
            result.path.header.frame_id = self.map_frame
            goal_handle.succeed()
            return result

        start_time = time.monotonic()
        try:
            start_xyz, start_layer = self.get_start_xyz(request.start)
            if abs(start_xyz[2]) < 1e-6:
                start_xyz[2] = self.default_start_z

            goal_xyz = self.goal_xyz_from_request(request.goal)
            goal_layer = self.planner.pose_to_layer(
                goal_xyz[:2],
                goal_xyz[2],
                z_offset=self.goal_z_offset,
            )

            self.get_logger().info(
                "PCT planning: "
                f"start=({start_xyz[0]:.2f}, {start_xyz[1]:.2f}, {start_xyz[2]:.2f}, L{start_layer}) -> "
                f"goal=({goal_xyz[0]:.2f}, {goal_xyz[1]:.2f}, {goal_xyz[2]:.2f}, L{goal_layer})"
            )
            plan_result = self.planner.plan(
                start_xyz,
                goal_xyz,
                start_layer=start_layer,
                goal_layer=goal_layer,
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"PCT planning failed: {exc}")
            goal_handle.abort()
            return result

        if plan_result is None or len(plan_result.xyz) == 0:
            self.get_logger().error("PCT planning failed: empty path")
            goal_handle.abort()
            return result

        stamp = self.get_clock().now().to_msg()
        path_msg = trajectory_to_path(plan_result.xyz, frame_id=self.map_frame, stamp=stamp)
        result.path = path_msg

        elapsed = time.monotonic() - start_time
        result.planning_time = Duration(sec=int(elapsed), nanosec=int((elapsed % 1.0) * 1e9))

        self.global_path_pub.publish(path_msg)
        self.pct_path_pub.publish(path_msg)
        self.get_logger().info(
            f"PCT path ready: points={len(path_msg.poses)}, "
            f"start_layer={plan_result.start_layer}, goal_layer={plan_result.goal_layer}, "
            f"time={elapsed:.3f}s"
        )

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PCTGetPlanServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
