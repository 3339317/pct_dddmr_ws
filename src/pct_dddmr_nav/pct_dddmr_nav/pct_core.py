import os
import pickle
import sys
from dataclasses import dataclass

import numpy as np


def default_pct_root():
    env_root = os.environ.get("PCT_PLANNER_ROOT")
    if env_root:
        return env_root

    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("pct_dddmr_nav"), "vendor", "pct_planner")
    except Exception:
        return os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "vendor", "pct_planner")
        )


PCT_ROOT_DEFAULT = default_pct_root()


@dataclass
class PCTPlanResult:
    xyz: np.ndarray
    layers: np.ndarray
    start_layer: int
    goal_layer: int


class PCTTomogramPlanner:
    def __init__(
        self,
        pct_root=PCT_ROOT_DEFAULT,
        use_quintic=True,
        max_heading_rate=10.0,
        robot_radius_cells=20,
        safety_margin_cells=15,
        reference_height=0.2,
    ):
        self.pct_root = os.path.abspath(os.path.expanduser(pct_root))
        self.lib_dir = os.path.join(self.pct_root, "planner", "lib")
        if self.lib_dir not in sys.path:
            sys.path.insert(0, self.lib_dir)

        import a_star  # pylint: disable=import-error,import-outside-toplevel
        import ele_planner  # pylint: disable=import-error,import-outside-toplevel
        import traj_opt  # pylint: disable=import-error,import-outside-toplevel

        self.a_star = a_star
        self.ele_planner = ele_planner
        self.traj_opt = traj_opt

        self.use_quintic = bool(use_quintic)
        self.max_heading_rate = float(max_heading_rate)
        self.robot_radius_cells = int(robot_radius_cells)
        self.safety_margin_cells = int(safety_margin_cells)
        self.reference_height = float(reference_height)

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = None
        self.offset = None
        self.trav_raw = None
        self.elev_g_raw = None
        self.elev_c_raw = None
        self.planner = None
        self.start_idx = np.zeros(3, dtype=np.int32)
        self.goal_idx = np.zeros(3, dtype=np.int32)

    def load(self, tomogram_path):
        tomogram_path = os.path.abspath(os.path.expanduser(tomogram_path))
        with open(tomogram_path, "rb") as handle:
            data_dict = pickle.load(handle)

        tomogram = np.asarray(data_dict["data"], dtype=np.float32, order="C")
        if tomogram.ndim != 4 or tomogram.shape[0] != 5:
            raise ValueError(f"Invalid tomogram shape {tomogram.shape}; expected [5, n_slice, dim_x, dim_y]")

        self.resolution = float(data_dict["resolution"])
        self.center = np.asarray(data_dict["center"], dtype=np.float64)
        self.n_slice = int(tomogram.shape[1])
        self.slice_h0 = float(data_dict["slice_h0"])
        self.slice_dh = float(data_dict["slice_dh"])
        self.map_dim = [int(tomogram.shape[2]), int(tomogram.shape[3])]
        self.offset = np.array([self.map_dim[0] // 2, self.map_dim[1] // 2], dtype=np.int32)

        self.trav_raw = np.asarray(tomogram[0], dtype=np.float32)
        trav_gx_raw = np.asarray(tomogram[1], dtype=np.float32)
        trav_gy_raw = np.asarray(tomogram[2], dtype=np.float32)
        self.elev_g_raw = np.asarray(tomogram[3], dtype=np.float32)
        self.elev_c_raw = np.asarray(tomogram[4], dtype=np.float32)

        trav = np.ascontiguousarray(self.trav_raw, dtype=np.float64)
        trav_gx = np.ascontiguousarray(trav_gx_raw, dtype=np.float64)
        trav_gy = np.ascontiguousarray(trav_gy_raw, dtype=np.float64)
        elev_g = np.ascontiguousarray(np.nan_to_num(self.elev_g_raw, nan=-100.0), dtype=np.float64)
        elev_c = np.ascontiguousarray(np.nan_to_num(self.elev_c_raw, nan=1e6), dtype=np.float64)

        self._init_planner(trav, trav_gx, trav_gy, elev_g, elev_c)

    def _init_planner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        gateway_up[:-1] = np.logical_and(diff_t < -8.0, diff_g < 0.1)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        gateway_dn[1:] = np.logical_and(diff_t > 8.0, diff_g < 0.1)

        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = self.ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate,
            use_quintic=self.use_quintic,
        )
        self.planner.init_map(
            self.robot_radius_cells,
            self.safety_margin_cells,
            float(self.resolution),
            int(self.n_slice),
            self.reference_height,
            np.ascontiguousarray(trav.reshape(-1, trav.shape[-1]), dtype=np.float64),
            np.ascontiguousarray(elev_g.reshape(-1, elev_g.shape[-1]), dtype=np.float64),
            np.ascontiguousarray(elev_c.reshape(-1, elev_c.shape[-1]), dtype=np.float64),
            np.ascontiguousarray(gateway.reshape(-1, gateway.shape[-1]), dtype=np.int32),
            np.ascontiguousarray(trav_gy.reshape(-1, trav_gy.shape[-1]), dtype=np.float64),
            np.ascontiguousarray((-trav_gx).reshape(-1, trav_gx.shape[-1]), dtype=np.float64),
        )

    def pos_to_raw_idx(self, pos_xy):
        pos_xy = np.asarray(pos_xy, dtype=np.float64)
        pos_local = pos_xy - self.center
        return np.round(pos_local / self.resolution).astype(np.int32) + self.offset

    def pos_to_planner_idx(self, pos_xy):
        raw_idx = self.pos_to_raw_idx(pos_xy)
        return np.array([raw_idx[1], raw_idx[0]], dtype=np.int32)

    def raw_idx_in_bounds(self, raw_idx):
        ix, iy = int(raw_idx[0]), int(raw_idx[1])
        return 0 <= ix < self.map_dim[0] and 0 <= iy < self.map_dim[1]

    def planner_idx_in_bounds(self, planner_idx):
        px, py = int(planner_idx[0]), int(planner_idx[1])
        return 0 <= px < self.map_dim[1] and 0 <= py < self.map_dim[0]

    def z_to_layer_coarse(self, z_ground):
        layer = int(np.round((float(z_ground) - self.slice_h0) / self.slice_dh))
        return int(np.clip(layer, 0, self.n_slice - 1))

    def pose_to_layer(
        self,
        pos_xy,
        z_map,
        z_offset=0.0,
        prev_layer=None,
        search_radius=2,
        layer_search_radius=2,
        spatial_weight=0.05,
        hysteresis_weight=0.10,
        use_coarse_prior=True,
    ):
        z_ground = float(z_map) - float(z_offset)
        raw_idx = self.pos_to_raw_idx(pos_xy)
        if not self.raw_idx_in_bounds(raw_idx):
            raise ValueError(f"position out of tomogram bounds: pos={pos_xy}, raw_idx={raw_idx}")

        ix0, iy0 = int(raw_idx[0]), int(raw_idx[1])
        if use_coarse_prior:
            coarse_layer = self.z_to_layer_coarse(z_ground)
            candidate_layers = range(
                max(0, coarse_layer - layer_search_radius),
                min(self.n_slice - 1, coarse_layer + layer_search_radius) + 1,
            )
        else:
            coarse_layer = None
            candidate_layers = range(self.n_slice)

        best_score = None
        best_layer = None
        for layer in candidate_layers:
            for dx in range(-search_radius, search_radius + 1):
                for dy in range(-search_radius, search_radius + 1):
                    ix = ix0 + dx
                    iy = iy0 + dy
                    if ix < 0 or ix >= self.map_dim[0] or iy < 0 or iy >= self.map_dim[1]:
                        continue
                    height = self.elev_g_raw[layer, ix, iy]
                    if np.isnan(height):
                        continue
                    score = abs(float(height) - z_ground)
                    score += spatial_weight * float(np.hypot(dx, dy))
                    if prev_layer is not None:
                        score += hysteresis_weight * abs(int(layer) - int(prev_layer))
                    if best_score is None or score < best_score:
                        best_score = score
                        best_layer = int(layer)

        if best_layer is None:
            return coarse_layer if coarse_layer is not None else 0
        return best_layer

    def plan(self, start_xyz, goal_xyz, start_layer=None, goal_layer=None):
        if self.planner is None:
            raise RuntimeError("tomogram is not loaded")

        start_xyz = np.asarray(start_xyz, dtype=np.float64)
        goal_xyz = np.asarray(goal_xyz, dtype=np.float64)
        start_xy = start_xyz[:2]
        goal_xy = goal_xyz[:2]

        if start_layer is None:
            start_layer = self.pose_to_layer(start_xy, start_xyz[2])
        if goal_layer is None:
            goal_layer = self.pose_to_layer(goal_xy, goal_xyz[2])

        start_planner_xy = self.pos_to_planner_idx(start_xy)
        goal_planner_xy = self.pos_to_planner_idx(goal_xy)
        if not self.planner_idx_in_bounds(start_planner_xy):
            raise ValueError(f"start out of bounds: pos={start_xy}, planner_idx={start_planner_xy}")
        if not self.planner_idx_in_bounds(goal_planner_xy):
            raise ValueError(f"goal out of bounds: pos={goal_xy}, planner_idx={goal_planner_xy}")

        self.start_idx[0] = int(start_layer)
        self.start_idx[1:] = start_planner_xy
        self.goal_idx[0] = int(goal_layer)
        self.goal_idx[1:] = goal_planner_xy

        self.planner.plan(self.start_idx, self.goal_idx, True)
        path_finder = self.planner.get_path_finder()
        path = np.asarray(path_finder.get_result_matrix())
        if len(path) == 0:
            return None

        optimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )
        traj_raw = np.asarray(optimizer.get_result_matrix())
        layers = np.asarray(optimizer.get_layers())
        heights = np.asarray(optimizer.get_heights())

        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        xyz = self.traj_grid_to_map(traj_3d)
        return PCTPlanResult(
            xyz=np.asarray(xyz, dtype=np.float32),
            layers=np.asarray(layers, dtype=np.int32),
            start_layer=int(start_layer),
            goal_layer=int(goal_layer),
        )

    def traj_grid_to_map(self, traj_grid):
        offset = np.array([self.map_dim[1] // 2, self.map_dim[0] // 2, 0])
        center = np.array([self.center[1], self.center[0], 0.5])
        traj_grid = (traj_grid - offset) * self.resolution + center
        return np.stack([traj_grid[:, 1], traj_grid[:, 0], traj_grid[:, 2]], axis=1)

    def traversable_points(self, max_trav=45.0, stride=2):
        points = []
        stride = max(1, int(stride))
        for layer in range(self.n_slice):
            trav = self.trav_raw[layer]
            elev = self.elev_g_raw[layer]
            mask = np.isfinite(elev) & (trav < float(max_trav))
            xs, ys = np.where(mask[::stride, ::stride])
            xs = xs * stride
            ys = ys * stride
            if xs.size == 0:
                continue
            x_map = (xs - self.offset[0]) * self.resolution + self.center[0]
            y_map = (ys - self.offset[1]) * self.resolution + self.center[1]
            z_map = elev[xs, ys]
            pts = np.stack([x_map, y_map, z_map], axis=1)
            points.append(pts)
        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.concatenate(points, axis=0).astype(np.float32)

    def obstacle_points(self, min_trav=45.0, stride=2):
        points = []
        stride = max(1, int(stride))
        for layer in range(self.n_slice):
            trav = self.trav_raw[layer]
            elev = np.nan_to_num(self.elev_c_raw[layer], nan=np.nan)
            mask = np.isfinite(elev) & (trav >= float(min_trav))
            xs, ys = np.where(mask[::stride, ::stride])
            xs = xs * stride
            ys = ys * stride
            if xs.size == 0:
                continue
            x_map = (xs - self.offset[0]) * self.resolution + self.center[0]
            y_map = (ys - self.offset[1]) * self.resolution + self.center[1]
            z_map = elev[xs, ys]
            pts = np.stack([x_map, y_map, z_map], axis=1)
            points.append(pts)
        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.concatenate(points, axis=0).astype(np.float32)
