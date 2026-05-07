import os
import pickle
import struct
import threading
import numpy as np

try:
    import open3d as o3d
except ImportError:
    o3d = None


class MapManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.pcd_path = ""
        self.points_xyz = []
        self.points_rgb = []
        self.points_xy = []
        self.ground_points_xyz = []
        self.obstacle_points_xyz = []
        self.inflation_points_xyz = []
        self.map_source = "none"
        self.sample_step = 1

    def load_pcd_file(self, path):
        if o3d is None:
            raise RuntimeError("open3d 未安装，无法加载点云")
        with self.lock:
            pcd = o3d.io.read_point_cloud(path)
            pts = np.asarray(pcd.points)
            if len(pts) == 0:
                raise RuntimeError("点云为空")

            self.pcd_path = path

            sampled = pts[::self.sample_step].astype(float)
            self.points_xyz = sampled.tolist()
            self.points_xy = sampled[:, :2].tolist()
            self.ground_points_xyz = self.points_xyz
            self.obstacle_points_xyz = []
            self.inflation_points_xyz = []
            self.map_source = "pcd"

            if pcd.has_colors():
                cols = np.asarray(pcd.colors)
                if len(cols) == len(pts):
                    self.points_rgb = cols[::self.sample_step].astype(float).tolist()
                else:
                    self.points_rgb = []
            else:
                self.points_rgb = []

    def _decode_pointcloud2_xyz_rgb(self, msg):
        """从 sensor_msgs/PointCloud2 消息更新地图数据"""
        fields = {f.name: f for f in msg.fields}
        point_step = msg.point_step
        data = msg.data

        has_x = 'x' in fields
        has_y = 'y' in fields
        has_z = 'z' in fields
        if not (has_x and has_y and has_z):
            return [], []

        x_field = fields['x']
        y_field = fields['y']
        z_field = fields['z']

        has_rgb = 'rgb' in fields or 'rgba' in fields
        rgb_field = fields.get('rgb', fields.get('rgba', None))

        num_points = msg.width * msg.height

        xyz_list = []
        rgb_list = []

        for i in range(num_points):
            offset = i * point_step
            if offset + point_step > len(data):
                break

            x = struct.unpack_from('f', data, offset + x_field.offset)[0]
            y = struct.unpack_from('f', data, offset + y_field.offset)[0]
            z = struct.unpack_from('f', data, offset + z_field.offset)[0]

            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                continue

            xyz_list.append([float(x), float(y), float(z)])

            if has_rgb and rgb_field is not None:
                rgb_offset = offset + rgb_field.offset
                if rgb_offset + 4 <= len(data):
                    rgb_int = struct.unpack_from('I', data, rgb_offset)[0]
                    r = ((rgb_int >> 16) & 0xFF) / 255.0
                    g = ((rgb_int >> 8) & 0xFF) / 255.0
                    b = (rgb_int & 0xFF) / 255.0
                    rgb_list.append([r, g, b])

        return xyz_list, rgb_list

    def _tomogram_points(self, elev, mask, resolution, center, offset, stride):
        xs, ys = np.where(mask[::stride, ::stride])
        xs = xs * stride
        ys = ys * stride
        if xs.size == 0:
            return []

        x_map = (xs - offset[0]) * resolution + center[0]
        y_map = (ys - offset[1]) * resolution + center[1]
        z_map = elev[xs, ys]
        return np.stack([x_map, y_map, z_map], axis=1).astype(float).tolist()

    def load_tomogram_file(
        self,
        path,
        sample_step=None,
        ground_max_trav=5.0,
        inflation_min_trav=5.0,
        obstacle_min_trav=45.0,
        obstacle_stride_multiplier=1,
    ):
        with open(os.path.abspath(os.path.expanduser(path)), "rb") as handle:
            data_dict = pickle.load(handle)

        tomogram = np.asarray(data_dict["data"], dtype=np.float32)
        if tomogram.ndim != 4 or tomogram.shape[0] < 5:
            raise RuntimeError(f"PCT tomogram格式不正确: {tomogram.shape}")

        resolution = float(data_dict["resolution"])
        center = np.asarray(data_dict["center"], dtype=np.float64)
        offset = np.array([tomogram.shape[2] // 2, tomogram.shape[3] // 2], dtype=np.int32)
        stride = max(1, int(sample_step if sample_step is not None else self.sample_step))
        obstacle_stride = max(1, stride * int(max(1, obstacle_stride_multiplier)))

        trav_layers = tomogram[0]
        ground_layers = tomogram[3]
        ceiling_layers = tomogram[4]

        ground_points = []
        inflation_points = []
        obstacle_points = []
        for layer in range(tomogram.shape[1]):
            trav = trav_layers[layer]
            ground = ground_layers[layer]
            ceiling = ceiling_layers[layer]
            has_ground = np.isfinite(ground)

            ground_mask = has_ground & (trav < float(ground_max_trav))
            inflation_mask = has_ground & (trav >= float(inflation_min_trav)) & (trav < float(obstacle_min_trav))
            obstacle_mask = has_ground & (trav >= float(obstacle_min_trav))

            ground_points.extend(self._tomogram_points(ground, ground_mask, resolution, center, offset, stride))
            inflation_points.extend(self._tomogram_points(ground, inflation_mask, resolution, center, offset, stride))

            obstacle_elev = np.where(np.isfinite(ceiling), ceiling, ground)
            obstacle_points.extend(
                self._tomogram_points(
                    obstacle_elev,
                    obstacle_mask,
                    resolution,
                    center,
                    offset,
                    obstacle_stride,
                )
            )

        with self.lock:
            self.pcd_path = os.path.abspath(os.path.expanduser(path))
            self.ground_points_xyz = ground_points
            self.inflation_points_xyz = inflation_points
            self.obstacle_points_xyz = obstacle_points
            self.points_xyz = self.ground_points_xyz
            self.points_xy = [[p[0], p[1]] for p in self.points_xyz]
            self.points_rgb = []
            self.map_source = "tomogram"

    def update_from_pointcloud2(self, msg, layer="ground"):
        with self.lock:
            if self.map_source == "tomogram":
                return

            xyz_list, rgb_list = self._decode_pointcloud2_xyz_rgb(msg)

            step = max(1, self.sample_step)
            sampled_xyz = xyz_list[::step]
            if layer == "obstacle":
                self.obstacle_points_xyz = sampled_xyz
            elif layer == "inflation":
                self.inflation_points_xyz = sampled_xyz
            else:
                self.ground_points_xyz = sampled_xyz

            self.points_xyz = self.ground_points_xyz
            self.points_xy = [[p[0], p[1]] for p in self.points_xyz]
            if len(rgb_list) == len(xyz_list):
                self.points_rgb = rgb_list[::step]
            else:
                self.points_rgb = []
            self.pcd_path = (
                f"ros:/mapground ({len(self.ground_points_xyz)} pts), "
                f"/mapcloud ({len(self.obstacle_points_xyz)} pts)"
            )

    def update_sample_step(self, step):
        self.sample_step = max(1, int(step))
        if self.pcd_path and o3d is not None:
            self.load_pcd_file(self.pcd_path)


class RouteManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.route_polyline = []
        self.route_cumlen = []
        self.route_sample_gap = 0.35
        self.final_target_yaw = 0.0

    def resample_polyline(self, pts, gap=None):
        if gap is None:
            gap = self.route_sample_gap
        pts = [_normalize_path_point(p) for p in pts]
        if len(pts) < 2:
            return pts

        pts_arr = np.array(
            [[p["x"], p["y"], p.get("z", 0.0)] for p in pts],
            dtype=float
        )

        lengths = [0.0]
        for i in range(1, len(pts_arr)):
            lengths.append(lengths[-1] + np.linalg.norm(pts_arr[i] - pts_arr[i - 1]))

        total_len = lengths[-1]
        if total_len < 1e-6:
            return pts

        sample_s = np.arange(0.0, total_len, gap)
        if len(sample_s) == 0 or abs(sample_s[-1] - total_len) > 1e-9:
            sample_s = np.append(sample_s, total_len)

        result = []
        j = 0
        for s in sample_s:
            while j < len(lengths) - 2 and lengths[j + 1] < s:
                j += 1

            seg_len = lengths[j + 1] - lengths[j]
            if seg_len < 1e-6:
                p = pts_arr[j]
            else:
                ratio = (s - lengths[j]) / seg_len
                p = pts_arr[j] + ratio * (pts_arr[j + 1] - pts_arr[j])

            result.append({
                "x": float(p[0]),
                "y": float(p[1]),
                "z": float(p[2]),
            })

        return result

    def build_route_cumlen(self, poly):
        if len(poly) == 0:
            return []
        out = [0.0]
        for i in range(1, len(poly)):
            out.append(out[-1] + _point_dist3d(poly[i], poly[i - 1]))
        return out

    def set_route_points(self, pts):
        with self.lock:
            route_points = [_normalize_path_point(p) for p in pts]
            if len(route_points) < 2:
                raise RuntimeError("路线点数不足")

            route_points = self.resample_polyline(route_points)
            self.route_polyline = route_points
            self.route_cumlen = self.build_route_cumlen(self.route_polyline)
            if len(self.route_polyline) >= 2:
                b = self.route_polyline[-1]
                a = self.route_polyline[-2]
                self.final_target_yaw = _angle_deg(b["x"] - a["x"], b["y"] - a["y"])
            else:
                self.final_target_yaw = 0.0

    def clear(self):
        with self.lock:
            self.route_polyline = []
            self.route_cumlen = []
            self.final_target_yaw = 0.0


def _normalize_path_point(p):
    return {
        "x": float(p.get("x", 0.0)),
        "y": float(p.get("y", 0.0)),
        "z": float(p.get("z", 0.0)),
    }


def _point_dist3d(a, b):
    dx = a["x"] - b["x"]
    dy = a["y"] - b["y"]
    dz = a["z"] - b["z"]
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def _angle_deg(dx, dy):
    import math as _math
    return _math.degrees(_math.atan2(dy, dx))
