#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import time
import json
import yaml
import threading

import numpy as np
import open3d as o3d

try:
    import cv2
except Exception:
    cv2 = None

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, PointCloud2
from dddmr_sys_core.action import GetPlan

from nav_state import MapManager, RouteManager


# =========================================================
# 核心状态与逻辑
# =========================================================
class AppCore:
    def __init__(self):
        self.lock = threading.RLock()
        self._map = MapManager()
        self._route = RouteManager()

        # =================================================
        # 控制参数
        # =================================================
        self.max_vx = 0.55
        self.max_wz = 0.60
        self.min_vx = 0.20
        self.min_wz = 0.12
        self.arrival_dist = 0.18
        self.arrival_angle = 5.0
        self.alpha = 0.35
        self.lookahead_distance = 0.75
        self.path_yaw_kp = 0.064
        self.final_yaw_kp = 0.035
        self.end_slowdown_distance = 1.10
        self.heading_slow_angle_deg = 28.0
        self.rotate_in_place_angle_deg = 45.0
        self.rotate_exit_angle_deg = 15.0

        # revision，用于前端判断是否需要拉取完整状态
        self.map_revision = 0
        self.route_revision = 0

        # =================================================
        # UI / 状态
        # =================================================
        self.mode = "none"
        self.status_text = "等待操作"

        # =================================================
        # ROS 状态
        # =================================================
        self.current_pose = None
        self.localized = False

        self.current_vx = 0.0
        self.current_wz = 0.0

        # =================================================
        # 导航状态
        # =================================================
        self.is_auto_moving = False
        self.stage = "idle"
        self.nav_progress_idx = 0
        self.lookahead_target = None
        self.route_preview_locked = False
        self.obstacle_distance = 999.0
        self.is_obstacle_paused = False
        self.obstacle_clusters = []
        self.obstacle_boxes = []
        self.local_detour_active = False
        self.local_detour_path = []

        # =================================================
        # 图像
        # =================================================
        self.camera_topic = "/camera/image_raw"
        self.camera_frame_jpeg = None
        self.camera_frame_time = 0.0

        # =================================================
        # 外部话题
        # =================================================
        self.nav_start_topic = "/nav/start"
        self.nav_stop_topic = "/nav/stop"
        self.nav_done_topic = "/nav/done"
        self.external_route_topic = "/pct_dddmr_web/external_route"
        self.external_route_rich_topic = "/pct_dddmr_web/external_route_rich"
        self.nav_clear_topic = "/pct_dddmr_web/clear"

        self.initialpose_frame = "map"
        self.tomogram_path = ""
        self.tomogram_ground_max_trav = 5.0
        self.tomogram_inflation_min_trav = 5.0
        self.tomogram_obstacle_min_trav = 45.0
        self.tomogram_obstacle_stride_multiplier = 1

    # ---- 地图相关属性（委托给 MapManager）----
    @property
    def pcd_path(self):
        return self._map.pcd_path

    @pcd_path.setter
    def pcd_path(self, v):
        self._map.pcd_path = v

    @property
    def points_xyz(self):
        return self._map.points_xyz

    @points_xyz.setter
    def points_xyz(self, v):
        self._map.points_xyz = v

    @property
    def points_rgb(self):
        return self._map.points_rgb

    @points_rgb.setter
    def points_rgb(self, v):
        self._map.points_rgb = v

    @property
    def points_xy(self):
        return self._map.points_xy

    @points_xy.setter
    def points_xy(self, v):
        self._map.points_xy = v

    @property
    def sample_step(self):
        return self._map.sample_step

    @sample_step.setter
    def sample_step(self, v):
        self._map.sample_step = v

    # ---- 路线相关属性（委托给 RouteManager）----
    @property
    def route_polyline(self):
        return self._route.route_polyline

    @route_polyline.setter
    def route_polyline(self, v):
        self._route.route_polyline = v

    @property
    def route_cumlen(self):
        return self._route.route_cumlen

    @route_cumlen.setter
    def route_cumlen(self, v):
        self._route.route_cumlen = v

    @property
    def route_sample_gap(self):
        return self._route.route_sample_gap

    @route_sample_gap.setter
    def route_sample_gap(self, v):
        self._route.route_sample_gap = v

    @property
    def final_target_yaw(self):
        return self._route.final_target_yaw

    @final_target_yaw.setter
    def final_target_yaw(self, v):
        self._route.final_target_yaw = v

    # -----------------------------------------------------
    # revision
    # -----------------------------------------------------
    def _bump_map_rev(self):
        self.map_revision += 1

    def _bump_route_rev(self):
        self.route_revision += 1

    # -----------------------------------------------------
    # 基础工具
    # -----------------------------------------------------
    def set_status(self, text: str):
        with self.lock:
            self.status_text = str(text)

    def normalize_angle_deg(self, ang):
        return (ang + 180.0) % 360.0 - 180.0

    def dist2d(self, ax, ay, bx, by):
        return math.hypot(ax - bx, ay - by)

    def dist3d(self, ax, ay, az, bx, by, bz):
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)

    def point_dist3d(self, p, q):
        return self.dist3d(
            float(p["x"]), float(p["y"]), float(p.get("z", 0.0)),
            float(q["x"]), float(q["y"]), float(q.get("z", 0.0))
        )

    def _normalize_path_point(self, p):
        if isinstance(p, dict):
            return {
                "x": float(p["x"]),
                "y": float(p["y"]),
                "z": float(p.get("z", 0.0)),
            }
        if isinstance(p, (list, tuple, np.ndarray)):
            arr = np.asarray(p).reshape(-1)
            if arr.size >= 2:
                return {
                    "x": float(arr[0]),
                    "y": float(arr[1]),
                    "z": float(arr[2]) if arr.size >= 3 else 0.0,
                }
        raise RuntimeError(f"无效路线点: {p}")

    def _reset_navigation_state_locked(self):
        self.is_auto_moving = False
        self.stage = "idle"
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.lookahead_target = None
        self.nav_progress_idx = 0

    def _handle_navigation_completed_locked(self):
        self._reset_navigation_state_locked()
        self.route_polyline = []
        self._bump_route_rev()
        self.status_text = "路线导航完成"

    # -----------------------------------------------------
    # 图像
    # -----------------------------------------------------
    def update_camera_frame(self, jpeg_bytes: bytes):
        with self.lock:
            self.camera_frame_jpeg = jpeg_bytes
            self.camera_frame_time = time.time()

    def get_camera_frame(self):
        with self.lock:
            return self.camera_frame_jpeg, self.camera_frame_time

    # -----------------------------------------------------
    # 参数导出/导入
    # -----------------------------------------------------
    def get_control_params(self):
        return {
            "max_linear_x": self.max_vx,
            "max_angular_z": self.max_wz,
            "min_linear_x": self.min_vx,
            "min_angular_z": self.min_wz,
            "arrival_distance": self.arrival_dist,
            "arrival_angle_deg": self.arrival_angle,
            "alpha": self.alpha,
            "lookahead_distance": self.lookahead_distance,
            "path_yaw_kp": self.path_yaw_kp,
            "final_yaw_kp": self.final_yaw_kp,
            "end_slowdown_distance": self.end_slowdown_distance,
            "heading_slow_angle_deg": self.heading_slow_angle_deg,
            "rotate_in_place_angle_deg": self.rotate_in_place_angle_deg,
            "rotate_exit_angle_deg": self.rotate_exit_angle_deg,
        }

    def set_control_params(self, d):
        with self.lock:
            self.max_vx = max(0.01, float(d.get("max_linear_x", self.max_vx)))
            self.max_wz = max(0.01, float(d.get("max_angular_z", self.max_wz)))
            self.min_vx = max(0.0, float(d.get("min_linear_x", self.min_vx)))
            self.min_wz = max(0.0, float(d.get("min_angular_z", self.min_wz)))
            self.arrival_dist = max(0.01, float(d.get("arrival_distance", self.arrival_dist)))
            self.arrival_angle = max(0.1, float(d.get("arrival_angle_deg", self.arrival_angle)))
            self.alpha = min(1.0, max(0.001, float(d.get("alpha", self.alpha))))
            self.lookahead_distance = max(0.05, float(d.get("lookahead_distance", self.lookahead_distance)))
            self.path_yaw_kp = max(0.0001, float(d.get("path_yaw_kp", self.path_yaw_kp)))
            self.final_yaw_kp = max(0.0001, float(d.get("final_yaw_kp", self.final_yaw_kp)))
            self.end_slowdown_distance = max(0.05, float(d.get("end_slowdown_distance", self.end_slowdown_distance)))
            self.heading_slow_angle_deg = max(1.0, float(d.get("heading_slow_angle_deg", self.heading_slow_angle_deg)))
            self.rotate_in_place_angle_deg = max(1.0, float(d.get("rotate_in_place_angle_deg", self.rotate_in_place_angle_deg)))
            self.rotate_exit_angle_deg = max(1.0, float(d.get("rotate_exit_angle_deg", self.rotate_exit_angle_deg)))

            if self.min_vx > self.max_vx:
                self.min_vx = self.max_vx
            if self.min_wz > self.max_wz:
                self.min_wz = self.max_wz

    def get_smoothing_params(self):
        return {
            "route_sample_gap": self.route_sample_gap,
        }

    def set_smoothing_params(self, d):
        with self.lock:
            self.route_sample_gap = max(0.02, float(d.get("route_sample_gap", self.route_sample_gap)))

    # -----------------------------------------------------
    # 地图
    # -----------------------------------------------------
    def load_pcd_file(self, path):
        self._map.load_pcd_file(path)
        self._bump_map_rev()
        self.set_status(f"已加载3D地图: {os.path.basename(path)}")

    # -----------------------------------------------------
    # 路线基础
    # -----------------------------------------------------
    def resample_polyline(self, pts, gap=0.35):
        return self._route.resample_polyline(pts, gap)

    def build_route_cumlen(self, poly):
        return self._route.build_route_cumlen(poly)

    def set_route_points(self, pts):
        self._reset_navigation_state_locked()
        self._route.set_route_points(pts)
        self._bump_route_rev()
        self.set_status(f"已设置路线，共 {len(self.route_polyline)} 点，等待开启导航")

    def set_external_route(self, pts, auto_start=False):
        # 为兼容旧接口保留 auto_start 参数，但这里不再自动启动导航
        self.set_route_points(pts)

    def set_external_route_bundle(self, data):
        if not isinstance(data, dict):
            raise RuntimeError("路线包必须是 JSON 对象")

        route = data.get("route", {})
        if isinstance(route, dict):
            points = route.get("polyline", [])
        else:
            points = route

        if not points:
            points = data.get("points") or data.get("route_points") or data.get("route_polyline") or []

        speed_params = data.get("speed_params") or data.get("control_params") or {}
        auto_start = bool(data.get("auto_start", False))

        if speed_params:
            self.set_control_params(speed_params)

        self.set_route_points(points)

        if auto_start:
            try:
                self.start_route_navigation()
                self.set_status(
                    f"已接收外部路线包并启动导航，路线点 {len(self.route_polyline)}"
                )
            except Exception as e:
                self.set_status(f"已接收外部路线包，自动启动失败: {e}")
        else:
            self.set_status(
                f"已接收外部路线包，路线点 {len(self.route_polyline)}，等待开启导航"
            )

    def clear_navigation_data(self):
        self._reset_navigation_state_locked()
        self._route.clear()
        self.set_status("已清除路线和导航状态")

    # 兼容旧接口
    def clear_route(self):
        self.clear_navigation_data()

    def dump_system_params_yaml(self):
        with self.lock:
            data = {
                "control_params": self.get_control_params(),
                "smoothing_params": self.get_smoothing_params(),
                "map_display": {
                    "sample_step": self.sample_step,
                },
                "topics": {
                    "camera_topic": self.camera_topic,
                    "nav_start_topic": self.nav_start_topic,
                    "nav_stop_topic": self.nav_stop_topic,
                    "nav_done_topic": self.nav_done_topic,
                    "initialpose_frame": self.initialpose_frame,
                }
            }
            return yaml.dump(data, allow_unicode=True, sort_keys=False)

    def load_system_params_data(self, data):
        with self.lock:
            self.set_control_params(data.get("control_params", {}))
            self.set_smoothing_params(data.get("smoothing_params", {}))

            map_display = data.get("map_display", {})
            self.sample_step = max(1, int(map_display.get("sample_step", self.sample_step)))
            self.tomogram_path = str(map_display.get("tomogram_path", self.tomogram_path) or "")
            self.tomogram_ground_max_trav = float(map_display.get("ground_max_trav", self.tomogram_ground_max_trav))
            self.tomogram_inflation_min_trav = float(map_display.get("inflation_min_trav", self.tomogram_inflation_min_trav))
            self.tomogram_obstacle_min_trav = float(map_display.get("obstacle_min_trav", self.tomogram_obstacle_min_trav))
            self.tomogram_obstacle_stride_multiplier = max(
                1,
                int(map_display.get("obstacle_stride_multiplier", self.tomogram_obstacle_stride_multiplier)),
            )

            topics = data.get("topics", {})
            self.camera_topic = topics.get("camera_topic", self.camera_topic)
            self.nav_start_topic = topics.get("nav_start_topic", self.nav_start_topic)
            self.nav_stop_topic = topics.get("nav_stop_topic", self.nav_stop_topic)
            self.nav_done_topic = topics.get("nav_done_topic", self.nav_done_topic)
            self.initialpose_frame = topics.get("initialpose_frame", self.initialpose_frame)

            self.set_status("已读取系统参数配置")

    # -----------------------------------------------------
    # 导航
    # -----------------------------------------------------
    def start_route_navigation(self):
        with self.lock:
            if not self.localized:
                raise RuntimeError("尚未定位")
            if len(self.route_polyline) < 2:
                raise RuntimeError("尚未收到路线")

            self.is_auto_moving = True
            self.stage = "path_tracking"
            self.nav_progress_idx = 0
            self.lookahead_target = None

            self.set_status(f"开始导航，路线点数 {len(self.route_polyline)}")
            self.route_preview_locked = False

    def stop_auto_move(self):
        with self.lock:
            self._reset_navigation_state_locked()
            self.route_preview_locked = False
            self.set_status("导航已停止")

    def emergency_stop(self):
        with self.lock:
            self._reset_navigation_state_locked()
            self.set_status("紧急停止")

    def update_pose(self, x, y, yaw_deg, z=0.0):
        with self.lock:
            self.current_pose = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw_deg": float(yaw_deg),
            }
            self.localized = True

    def update_controller_state(self, data):
        if not isinstance(data, dict):
            return
        with self.lock:
            was_moving = self.is_auto_moving
            self.current_vx = float(data.get("current_vx", self.current_vx))
            self.current_wz = float(data.get("current_wz", self.current_wz))
            self.is_auto_moving = bool(data.get("is_auto_moving", self.is_auto_moving))
            self.stage = str(data.get("stage", self.stage))
            self.nav_progress_idx = int(data.get("nav_progress_idx", self.nav_progress_idx))
            self.final_target_yaw = float(data.get("final_target_yaw", self.final_target_yaw))
            target = data.get("lookahead_target", None)
            self.lookahead_target = target if isinstance(target, dict) else None
            self.obstacle_distance = float(data.get("obstacle_distance", self.obstacle_distance))
            self.is_obstacle_paused = bool(data.get("is_obstacle_paused", self.is_obstacle_paused))
            clusters = data.get("obstacle_clusters", [])
            boxes = data.get("obstacle_boxes", [])
            detour_path = data.get("local_detour_path", [])
            self.obstacle_clusters = clusters if isinstance(clusters, list) else []
            self.obstacle_boxes = boxes if isinstance(boxes, list) else []
            self.local_detour_active = bool(data.get("local_detour_active", self.local_detour_active))
            self.local_detour_path = detour_path if isinstance(detour_path, list) else []
            status = data.get("status_text", None)
            if isinstance(status, str) and status:
                self.status_text = status

            route_pts = int(data.get("route_total_points", -1))
            if was_moving and not self.is_auto_moving and route_pts == 0:
                self._handle_navigation_completed_locked()

    # -----------------------------------------------------
    # 状态导出
    # -----------------------------------------------------
    def export_state(self):
        with self.lock:
            return {
                "pcd_path": self.pcd_path,
                "points_xyz": self.points_xyz,
                "points_rgb": self.points_rgb,
                "points_xy": self.points_xy,
                "map_layers": {
                    "ground": self._map.ground_points_xyz,
                    "inflation": self._map.inflation_points_xyz,
                    "obstacle": self._map.obstacle_points_xyz,
                },

                "route_polyline": self.route_polyline,

                "mode": self.mode,
                "status_text": self.status_text,
                "localized": self.localized,
                "current_pose": self.current_pose,
                "current_vx": self.current_vx,
                "current_wz": self.current_wz,

                "nav": {
                    "is_auto_moving": self.is_auto_moving,
                    "stage": self.stage,
                    "nav_progress_idx": self.nav_progress_idx,
                    "route_total_points": len(self.route_polyline),
                    "lookahead_target": self.lookahead_target,
                    "final_target_yaw": self.final_target_yaw,
                    "obstacle_distance": self.obstacle_distance,
                    "is_obstacle_paused": self.is_obstacle_paused,
                    "obstacle_clusters": self.obstacle_clusters,
                    "obstacle_boxes": self.obstacle_boxes,
                    "local_detour_active": self.local_detour_active,
                    "local_detour_path": self.local_detour_path,
                },

                "camera": {
                    "topic": self.camera_topic,
                    "has_frame": self.camera_frame_jpeg is not None,
                    "frame_age": (time.time() - self.camera_frame_time) if self.camera_frame_time > 0 else None,
                },

                "control_params": self.get_control_params(),
                "smoothing_params": self.get_smoothing_params(),

                "topics": {
                    "camera_topic": self.camera_topic,
                    "nav_start_topic": self.nav_start_topic,
                    "nav_stop_topic": self.nav_stop_topic,
                    "nav_done_topic": self.nav_done_topic,
                    "initialpose_frame": self.initialpose_frame,
                },

                "revisions": {
                    "map": self.map_revision,
                    "route": self.route_revision,
                },

                "map_is_3d": True,
            }

    def export_compact_state(self):
        with self.lock:
            return {
                "type": "compact_state",
                "status_text": self.status_text,
                "localized": self.localized,
                "current_pose": dict(self.current_pose) if self.current_pose is not None else None,
                "current_vx": self.current_vx,
                "current_wz": self.current_wz,
                "nav": {
                    "is_auto_moving": self.is_auto_moving,
                    "stage": self.stage,
                    "nav_progress_idx": self.nav_progress_idx,
                    "route_total_points": len(self.route_polyline),
                    "lookahead_target": dict(self.lookahead_target) if self.lookahead_target is not None else None,
                    "final_target_yaw": self.final_target_yaw,
                    "obstacle_distance": self.obstacle_distance,
                    "is_obstacle_paused": self.is_obstacle_paused,
                    "obstacle_clusters": list(self.obstacle_clusters),
                    "obstacle_boxes": list(self.obstacle_boxes),
                    "local_detour_active": self.local_detour_active,
                    "local_detour_path": list(self.local_detour_path),
                },
                "camera": {
                    "topic": self.camera_topic,
                },
                "pcd_path": self.pcd_path,
                "route_polyline": self.route_polyline,
                "route_count": len(self.route_polyline),
                "revisions": {
                    "map": self.map_revision,
                    "route": self.route_revision,
                },
                "topics": {
                    "camera_topic": self.camera_topic,
                    "external_route_topic": self.external_route_topic,
                    "external_route_rich_topic": self.external_route_rich_topic,
                    "nav_start_topic": self.nav_start_topic,
                    "nav_stop_topic": self.nav_stop_topic,
                    "nav_done_topic": self.nav_done_topic,
                    "nav_clear_topic": self.nav_clear_topic,
                    "initialpose_frame": self.initialpose_frame,
                },
            }


# =========================================================
# ROS2 节点
# =========================================================
class WebRosBridgeNode(Node):
    def __init__(self, core: AppCore):
        super().__init__("pct_dddmr_web_web")
        self.core = core
        self.bridge = CvBridge() if CvBridge is not None else None
        self.localization_seq = 0
        self.last_localization_update_time = 0.0
        self.localization_update_period = float(
            self.declare_parameter("localization_update_period", 0.1).value
        )
        self.map_to_odom_seen = False

        # 默认 QoS (RELIABLE) 与 /localization 话题发布者匹配
        # 之前 BEST_EFFORT 与 RELIABLE 发布者不兼容，导致收不到定位数据

        # 订阅 /localization 话题获取当前位置信息
        self.sub_localization = self.create_subscription(
            Odometry, "/localization", self.localization_callback, 10
        )

        self.sub_map_to_odom = self.create_subscription(
            Odometry, "/map_to_odom", self.map_to_odom_callback, 10
        )

        self.sub_nav_start = self.create_subscription(
            Empty, self.core.nav_start_topic, self.nav_start_callback, 10
        )

        self.sub_nav_stop = self.create_subscription(
            Empty, self.core.nav_stop_topic, self.nav_stop_callback, 10
        )

        self.sub_controller_state = self.create_subscription(
            String, "/pct_dddmr_web/controller/state", self.controller_state_callback, 10
        )

        self.plan_action_name = self.declare_parameter("plan_action_name", "get_dwa_plan").value
        self.plan_action_client = ActionClient(self, GetPlan, self.plan_action_name)

        self.sub_nav_done = self.create_subscription(
            String, self.core.nav_done_topic, self.nav_done_callback, 10
        )

        self.sub_camera = None
        if self.bridge is not None and cv2 is not None:
            try:
                self.sub_camera = self.create_subscription(
                    Image, self.core.camera_topic, self.camera_callback, 10
                )
            except Exception as e:
                self.get_logger().warning(f"camera subscription create failed: {e}")

        # 订阅处理后的地面/障碍点云，网页端分层渲染并优先使用地面点云选点。
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_mapground = self.create_subscription(
            PointCloud2, "/mapground", self.mapground_callback, map_qos
        )
        self.sub_mapcloud = self.create_subscription(
            PointCloud2, "/mapcloud", self.mapcloud_callback, map_qos
        )

        # 订阅 dddmr 全局规划路线
        self.sub_global_path = self.create_subscription(
            Path, "/global_path", self.global_path_callback, 10
        )

        self.pub_nav_done = self.create_publisher(String, self.core.nav_done_topic, 10)
        self.pub_controller_route = self.create_publisher(Path, "/pct_dddmr_web/controller/route", 10)
        self.pub_controller_config = self.create_publisher(String, "/pct_dddmr_web/controller/config", 10)
        self.pub_controller_start = self.create_publisher(Empty, "/pct_dddmr_web/controller/start", 10)
        self.pub_controller_stop = self.create_publisher(Empty, "/pct_dddmr_web/controller/stop", 10)
        self.pub_controller_clear = self.create_publisher(Empty, "/pct_dddmr_web/controller/clear", 10)
        self.pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.manual_cmd_vel_topic = self.declare_parameter(
            "manual_cmd_vel_topic", "/pct_dddmr_web/manual_cmd_vel"
        ).value
        self.pub_chassis_cmd_vel = self.create_publisher(Twist, self.manual_cmd_vel_topic, 10)
        self.pub_chassis_mode = self.create_publisher(String, "/chassis/mode", 10)
        self.pub_chassis_sitdown = self.create_publisher(Empty, "/chassis/sitdown", 10)
        self.pub_chassis_emgy_stop = self.create_publisher(Empty, "/chassis/emgy_stop", 10)
        self.pub_chassis_imu_enable = self.create_publisher(String, "/chassis/imu_enable", 10)

        self.get_logger().info("WebRosBridgeNode 启动完成")

    def get_yaw_deg_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    @staticmethod
    def yaw_deg_to_quaternion(yaw_deg):
        yaw = math.radians(yaw_deg)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return {
            "x": 0.0,
            "y": 0.0,
            "z": sy,
            "w": cy,
        }

    def publish_initial_pose(self, x, y, z, yaw_deg):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.core.initialpose_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        q = self.yaw_deg_to_quaternion(float(yaw_deg))
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942,
        ]
        self.pub_initialpose.publish(msg)
        self.get_logger().info(
            f"published /initialpose ({x:.2f}, {y:.2f}, {z:.2f}, yaw={yaw_deg:.1f}°)")

    def publish_chassis_cmd_vel(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vz)
        self.pub_chassis_cmd_vel.publish(msg)

    def publish_chassis_mode(self, mode, enable=True):
        payload = {"mode": mode, "enable": enable}
        if mode not in ("stair",):
            payload.pop("enable", None)
        self.pub_chassis_mode.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f"chassis mode: {mode} (enable={enable})")

    def publish_chassis_sitdown(self):
        self.pub_chassis_sitdown.publish(Empty())
        self.get_logger().info("chassis sitdown")

    def publish_chassis_emgy_stop(self):
        self.pub_chassis_emgy_stop.publish(Empty())
        self.get_logger().info("chassis emergency stop")

    def publish_chassis_imu_enable(self, enable):
        payload = {"enable": bool(enable)}
        self.pub_chassis_imu_enable.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f"chassis IMU enable={enable}")

    def update_core_pose_from_ros_pose(self, pose):
        try:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            yaw = self.get_yaw_deg_from_quaternion(pose.orientation)

            was_localized = self.core.localized
            self.core.update_pose(x, y, yaw, z)
            if not was_localized:
                self.core.set_status("已定位，可以导航")
        except Exception as e:
            self.get_logger().error(f"update_core_pose_from_ros_pose error: {e}")

    def localization_callback(self, msg: Odometry):
        try:
            now = time.monotonic()
            if now - self.last_localization_update_time < self.localization_update_period:
                return
            self.last_localization_update_time = now

            self.update_core_pose_from_ros_pose(msg.pose.pose)
            self.localization_seq += 1
            if self.localization_seq % 50 == 1:
                cp = self.core.current_pose
                self.get_logger().info(
                    f"[localization #{self.localization_seq}] x={cp['x']:.3f} y={cp['y']:.3f} z={cp['z']:.3f} yaw={cp['yaw_deg']:.1f}"
                )
            if self.core.localized and not self.core.status_text.startswith("已定位"):
                self.core.set_status("已定位，可以导航")
        except Exception as e:
            self.get_logger().error(f"localization_callback error: {e}")

    def map_to_odom_callback(self, _msg: Odometry):
        self.map_to_odom_seen = True

    def camera_callback(self, msg: Image):
        if self.bridge is None or cv2 is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                self.core.update_camera_frame(enc.tobytes())
        except Exception as e:
            self.get_logger().warning(f"camera_callback error: {e}")

    def mapground_callback(self, msg: PointCloud2):
        try:
            self.core._map.update_from_pointcloud2(msg, layer="ground")
            if self.core._map.map_source == "tomogram":
                return
            self.core._bump_map_rev()
            pts_count = len(self.core._map.ground_points_xyz)
            if pts_count > 0:
                self.core.set_status(f"已从 /mapground 更新地图 ({pts_count} 点)")
        except Exception as e:
            self.get_logger().warning(f"mapground_callback error: {e}")

    def mapcloud_callback(self, msg: PointCloud2):
        try:
            self.core._map.update_from_pointcloud2(msg, layer="obstacle")
            if self.core._map.map_source == "tomogram":
                return
            self.core._bump_map_rev()
            pts_count = len(self.core._map.obstacle_points_xyz)
            if pts_count > 0:
                self.core.set_status(f"已从 /mapcloud 更新障碍物 ({pts_count} 点)")
        except Exception as e:
            self.get_logger().warning(f"mapcloud_callback error: {e}")

    def global_path_callback(self, msg: Path):
        """接收 dddmr 全局规划器发布的路线并更新到前端显示"""
        try:
            with self.core.lock:
                if self.core.is_auto_moving and self.core.route_preview_locked:
                    return
            pts = []
            for p in msg.poses:
                pts.append({
                    "x": float(p.pose.position.x),
                    "y": float(p.pose.position.y),
                    "z": float(p.pose.position.z),
                })
            if len(pts) >= 2:
                with self.core.lock:
                    self.core._route.set_route_points(pts)
                    self.core._bump_route_rev()
        except Exception as e:
            self.get_logger().warning(f"global_path_callback error: {e}")

    def request_pct_plan(self, x, y, z, yaw_deg=0.0, timeout_sec=10.0):
        if not self.plan_action_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f"PCT规划 action 不可用: {self.plan_action_name}")

        with self.core.lock:
            current_pose = dict(self.core.current_pose or {})
        if not current_pose:
            raise RuntimeError("尚未收到 /localization，无法确定规划起点")

        stamp = self.get_clock().now().to_msg()
        goal_msg = GetPlan.Goal()
        goal_msg.activate_threading = True

        goal_msg.start.header.stamp = stamp
        goal_msg.start.header.frame_id = self.core.initialpose_frame
        goal_msg.start.pose.position.x = float(current_pose.get("x", 0.0))
        goal_msg.start.pose.position.y = float(current_pose.get("y", 0.0))
        goal_msg.start.pose.position.z = float(current_pose.get("z", 0.0))
        goal_msg.start.pose.orientation.w = 1.0

        goal_msg.goal.header.stamp = stamp
        goal_msg.goal.header.frame_id = self.core.initialpose_frame
        goal_msg.goal.pose.position.x = float(x)
        goal_msg.goal.pose.position.y = float(y)
        goal_msg.goal.pose.position.z = float(z)
        q = self.yaw_deg_to_quaternion(float(yaw_deg))
        goal_msg.goal.pose.orientation.x = q["x"]
        goal_msg.goal.pose.orientation.y = q["y"]
        goal_msg.goal.pose.orientation.z = q["z"]
        goal_msg.goal.pose.orientation.w = q["w"]

        done_event = threading.Event()
        result_holder = {"path": None, "error": None}

        def on_result(result_future):
            try:
                wrapped = result_future.result()
                path_msg = wrapped.result.path
                pts = [
                    {
                        "x": float(p.pose.position.x),
                        "y": float(p.pose.position.y),
                        "z": float(p.pose.position.z),
                    }
                    for p in path_msg.poses
                ]
                if len(pts) < 2:
                    raise RuntimeError("PCT规划返回空路径")
                result_holder["path"] = pts
            except Exception as exc:  # pylint: disable=broad-except
                result_holder["error"] = exc
            finally:
                done_event.set()

        def on_goal_response(goal_future):
            try:
                goal_handle = goal_future.result()
                if not goal_handle.accepted:
                    raise RuntimeError("PCT规划目标被拒绝")
                goal_handle.get_result_async().add_done_callback(on_result)
            except Exception as exc:  # pylint: disable=broad-except
                result_holder["error"] = exc
                done_event.set()

        self.plan_action_client.send_goal_async(goal_msg).add_done_callback(on_goal_response)

        if not done_event.wait(timeout=float(timeout_sec)):
            raise RuntimeError("PCT规划超时")
        if result_holder["error"] is not None:
            raise result_holder["error"]

        with self.core.lock:
            self.core._route.set_route_points(result_holder["path"])
            self.core.final_target_yaw = float(yaw_deg)
            self.core.route_preview_locked = bool(self.core.is_auto_moving)
            self.core._bump_route_rev()
            self.core.set_status(f"已规划路径，共 {len(result_holder['path'])} 点，等待开启导航")
        self.publish_controller_route()
        self.publish_controller_config()
        return result_holder["path"]

    def external_route_callback(self, msg: Path):
        try:
            pts = []
            for p in msg.poses:
                pts.append({
                    "x": float(p.pose.position.x),
                    "y": float(p.pose.position.y),
                    "z": float(p.pose.position.z),
                })

            if len(pts) >= 2:
                self.core.set_external_route(pts, auto_start=False)
                self.publish_controller_route()
                self.publish_controller_config()
        except Exception as e:
            self.get_logger().error(f"external_route_callback error: {e}")

    def external_route_rich_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.core.set_external_route_bundle(data)
            self.publish_controller_route()
            self.publish_controller_config()
            if bool(data.get("auto_start", False)):
                self.pub_controller_start.publish(Empty())
        except Exception as e:
            self.core.set_status(f"外部路线包读取失败: {e}")
            self.get_logger().error(f"external_route_rich_callback error: {e}")

    def controller_state_callback(self, msg: String):
        try:
            self.core.update_controller_state(json.loads(msg.data))
        except Exception as e:
            self.get_logger().error(f"controller_state_callback error: {e}")

    def nav_start_callback(self, _msg: Empty):
        try:
            self.core.start_route_navigation()
            self.publish_controller_route()
            self.publish_controller_config()
            self.pub_controller_start.publish(Empty())
        except Exception as e:
            self.core.set_status(f"开启导航失败: {e}")
            self.get_logger().error(f"nav_start_callback error: {e}")

    def nav_stop_callback(self, _msg: Empty):
        try:
            self.stop_controller_navigation()
            self.core.set_status("已通过外部话题停止导航")
        except Exception as e:
            self.core.set_status(f"停止导航失败: {e}")
            self.get_logger().error(f"nav_stop_callback error: {e}")

    def nav_clear_callback(self, _msg: Empty):
        try:
            self.core.clear_navigation_data()
            self.pub_controller_clear.publish(Empty())
        except Exception as e:
            self.get_logger().error(f"nav_clear_callback error: {e}")

    def publish_controller_route(self):
        with self.core.lock:
            route = [dict(p) for p in self.core.route_polyline]
            frame_id = self.core.initialpose_frame
            final_target_yaw = float(self.core.final_target_yaw)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        final_q = self.yaw_deg_to_quaternion(final_target_yaw)
        for i, p in enumerate(route):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(p.get("x", 0.0))
            pose.pose.position.y = float(p.get("y", 0.0))
            pose.pose.position.z = float(p.get("z", 0.0))
            if i == len(route) - 1:
                pose.pose.orientation.x = final_q["x"]
                pose.pose.orientation.y = final_q["y"]
                pose.pose.orientation.z = final_q["z"]
                pose.pose.orientation.w = final_q["w"]
            else:
                pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_controller_route.publish(path)

    def publish_controller_config(self):
        with self.core.lock:
            data = self.core.get_control_params()
            data["final_target_yaw"] = self.core.final_target_yaw
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_controller_config.publish(msg)

    def start_controller_navigation(self):
        self.ensure_global_localization_ready()
        self.core.start_route_navigation()
        self.publish_controller_route()
        self.publish_controller_config()
        self.pub_controller_start.publish(Empty())

    def ensure_global_localization_ready(self, timeout_sec=3.0):
        if self.map_to_odom_seen:
            return

        with self.core.lock:
            pose = dict(self.core.current_pose or {})
        if not pose:
            raise RuntimeError("尚未收到 /localization，无法初始化全局定位")

        self.publish_initial_pose(
            pose.get("x", 0.0),
            pose.get("y", 0.0),
            pose.get("z", 0.0),
            pose.get("yaw_deg", 0.0),
        )
        deadline = time.monotonic() + float(timeout_sec)
        while not self.map_to_odom_seen and time.monotonic() < deadline:
            time.sleep(0.02)

        if not self.map_to_odom_seen:
            raise RuntimeError("等待 /map_to_odom 超时，请先完成 initialpose")

    def stop_controller_navigation(self):
        self.core.stop_auto_move()
        self.pub_controller_stop.publish(Empty())

    def nav_done_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("event") == "nav_done" and data.get("success"):
                self.get_logger().info("收到 nav_done，结束导航任务")
                self.pub_controller_stop.publish(Empty())
                self.pub_controller_clear.publish(Empty())
                with self.core.lock:
                    self.core._handle_navigation_completed_locked()
        except Exception as e:
            self.get_logger().error(f"nav_done_callback error: {e}")

    def emergency_controller_stop(self):
        self.core.emergency_stop()
        self.pub_controller_stop.publish(Empty())

    def clear_controller_navigation(self):
        self.core.clear_navigation_data()
        with self.core.lock:
            self.core.route_preview_locked = False
        self.pub_controller_clear.publish(Empty())


# =========================================================
# ROS 运行器
# =========================================================
class RosRunner:
    def __init__(self, core: AppCore):
        self.core = core
        self.node = None
        self.thread = None
        self.started = False

    def start(self):
        if self.started:
            return
        rclpy.init(args=None)
        self.node = WebRosBridgeNode(self.core)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()
        self.started = True

    def stop(self):
        if not self.started:
            return

        try:
            self.core.emergency_stop()
        except Exception:
            pass

        try:
            if self.node is not None:
                self.node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        self.started = False

    def sync_controller_route(self):
        if not self.started or self.node is None:
            return
        self.node.publish_controller_route()
        self.node.publish_controller_config()

    def sync_controller_config(self):
        if not self.started or self.node is None:
            return
        self.node.publish_controller_config()

    def publish_initial_pose(self, x, y, z, yaw_deg):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_initial_pose(x, y, z, yaw_deg)

    def start_navigation(self):
        if not self.started or self.node is None:
            self.core.start_route_navigation()
            return
        self.node.start_controller_navigation()

    def plan_to_goal(self, x, y, z, yaw_deg=0.0):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        return self.node.request_pct_plan(x, y, z, yaw_deg=yaw_deg)

    def stop_navigation(self):
        if not self.started or self.node is None:
            self.core.stop_auto_move()
            return
        self.node.stop_controller_navigation()

    def emergency_stop_navigation(self):
        if not self.started or self.node is None:
            self.core.emergency_stop()
            return
        self.node.emergency_controller_stop()

    def clear_navigation(self):
        if not self.started or self.node is None:
            self.core.clear_navigation_data()
            return
        self.node.clear_controller_navigation()

    def chassis_cmd_vel(self, vx, vy, vz):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_cmd_vel(vx, vy, vz)

    def chassis_mode(self, mode, enable=True):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_mode(mode, enable)

    def chassis_sitdown(self):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_sitdown()

    def chassis_emgy_stop(self):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_emgy_stop()

    def chassis_imu_enable(self, enable):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_imu_enable(enable)


# =========================================================
# 全局对象
# =========================================================
core = AppCore()
ros_runner = RosRunner(core)
