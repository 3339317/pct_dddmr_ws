import os
import pickle
import time
from dataclasses import dataclass

import numpy as np

from .kernels import inflation_kernel, tomography_kernel, trav_kernel


@dataclass
class TomogramConfig:
    resolution: float = 0.10
    ground_h: float = 0.0
    slice_dh: float = 0.5
    kernel_size: int = 7
    interval_min: float = 0.50
    interval_free: float = 0.65
    slope_max: float = 0.40
    step_max: float = 0.17
    standable_ratio: float = 0.20
    cost_barrier: float = 50.0
    safe_margin: float = 0.4
    inflation: float = 0.2
    repeat: int = 1


class TomogramConverter:
    def __init__(self, cfg: TomogramConfig):
        import cupy as cp  # pylint: disable=import-outside-toplevel

        self.cp = cp
        self.cfg = cfg

        self.half_trav_k_size = int(cfg.kernel_size / 2)
        self.step_stand = 1.2 * cfg.resolution * np.tan(cfg.slope_max)
        self.standable_th = int(cfg.standable_ratio * (2 * self.half_trav_k_size + 1) ** 2) - 1
        self.half_inf_k_size = int((cfg.safe_margin + cfg.inflation) / cfg.resolution)

    def init_map_geometry(self, points):
        points_max = np.max(points, axis=0)
        points_min = np.min(points, axis=0)
        points_min[-1] = self.cfg.ground_h

        map_dim_x = int(np.ceil((points_max[0] - points_min[0]) / self.cfg.resolution)) + 4
        map_dim_y = int(np.ceil((points_max[1] - points_min[1]) / self.cfg.resolution)) + 4
        n_slice_init = int(np.ceil((points_max[2] - points_min[2]) / self.cfg.slice_dh))
        n_slice_init = max(1, n_slice_init)

        center = (points_max[:2] + points_min[:2]) / 2
        slice_h0 = points_min[-1] + self.cfg.slice_dh
        return center.astype(np.float32), map_dim_x, map_dim_y, n_slice_init, slice_h0

    def init_buffers(self, n_slice_init, map_dim_x, map_dim_y):
        cp = self.cp
        shape = (n_slice_init, map_dim_x, map_dim_y)
        return {
            "layers_g": cp.zeros(shape, dtype=cp.float32),
            "layers_c": cp.zeros(shape, dtype=cp.float32),
            "grad_mag_sq": cp.zeros(shape, dtype=cp.float32),
            "grad_mag_max": cp.zeros(shape, dtype=cp.float32),
            "trav_cost": cp.zeros(shape, dtype=cp.float32),
            "inflated_cost": cp.zeros(shape, dtype=cp.float32),
        }

    def clear_buffers(self, buffers):
        buffers["layers_g"].fill(-1e6)
        buffers["layers_c"].fill(1e6)
        buffers["grad_mag_sq"].fill(0.0)
        buffers["grad_mag_max"].fill(0.0)
        buffers["trav_cost"].fill(0.0)
        buffers["inflated_cost"].fill(0.0)

    def convert(self, points):
        cp = self.cp
        points = np.asarray(points, dtype=np.float32)
        points = points[~np.isnan(points).any(axis=1)]
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points must have shape [N, 3] or larger")
        points = points[:, :3]

        center, map_dim_x, map_dim_y, n_slice_init, slice_h0 = self.init_map_geometry(points)
        buffers = self.init_buffers(n_slice_init, map_dim_x, map_dim_y)

        tomo_kernel = tomography_kernel(
            self.cfg.resolution,
            map_dim_x,
            map_dim_y,
            n_slice_init,
            slice_h0,
            self.cfg.slice_dh,
        )
        trv_kernel = trav_kernel(
            map_dim_x,
            map_dim_y,
            self.half_trav_k_size,
            self.cfg.interval_min,
            self.cfg.interval_free,
            self.cfg.step_max,
            self.step_stand,
            self.standable_th,
            self.cfg.cost_barrier,
        )
        inf_kernel = inflation_kernel(map_dim_x, map_dim_y, self.half_inf_k_size)

        inf_table = cp.zeros((2 * self.half_inf_k_size + 1, 2 * self.half_inf_k_size + 1), dtype=cp.float32)
        for i in range(inf_table.shape[0]):
            for j in range(inf_table.shape[1]):
                dist = np.sqrt(
                    (self.cfg.resolution * (i - self.half_inf_k_size)) ** 2
                    + (self.cfg.resolution * (j - self.half_inf_k_size)) ** 2
                )
                inf_table[i, j] = np.clip(
                    1 - (dist - self.cfg.inflation) / (self.cfg.safe_margin + self.cfg.resolution),
                    a_min=0.0,
                    a_max=1.0,
                )

        points_gpu = cp.asarray(points)
        center_gpu = cp.asarray(center, dtype=cp.float32)

        timings = {"t_map": 0.0, "t_trav": 0.0, "t_simp": 0.0, "t_all": 0.0}
        repeat = max(1, int(self.cfg.repeat))
        final = None

        for _ in range(repeat):
            start_all = time.time()
            self.clear_buffers(buffers)

            start_gpu = cp.cuda.Event()
            end_gpu = cp.cuda.Event()
            start_gpu.record()
            tomo_kernel(points_gpu, center_gpu, buffers["layers_g"], buffers["layers_c"], size=points_gpu.shape[0])

            layers_g = buffers["layers_g"]
            grad_mag_sq = buffers["grad_mag_sq"]
            grad_mag_max = buffers["grad_mag_max"]
            diff_x_sq = cp.maximum((layers_g[:, 1:-1, :] - layers_g[:, :-2, :]) ** 2, (layers_g[:, 1:-1, :] - layers_g[:, 2:, :]) ** 2)
            diff_y_sq = cp.maximum((layers_g[:, :, 1:-1] - layers_g[:, :, :-2]) ** 2, (layers_g[:, :, 1:-1] - layers_g[:, :, 2:]) ** 2)
            grad_mag_sq[:, 1:-1, 1:-1] = diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :]
            grad_mag_max[:, 1:-1, 1:-1] = cp.maximum(diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :])
            interval = buffers["layers_c"] - layers_g
            end_gpu.record()
            end_gpu.synchronize()
            timings["t_map"] += cp.cuda.get_elapsed_time(start_gpu, end_gpu)

            start_gpu = cp.cuda.Event()
            end_gpu = cp.cuda.Event()
            start_gpu.record()
            trv_kernel(interval, grad_mag_sq, grad_mag_max, buffers["trav_cost"], size=n_slice_init * map_dim_x * map_dim_y)
            inf_kernel(buffers["trav_cost"], inf_table, buffers["inflated_cost"], size=n_slice_init * map_dim_x * map_dim_y)
            end_gpu.record()
            end_gpu.synchronize()
            timings["t_trav"] += cp.cuda.get_elapsed_time(start_gpu, end_gpu)

            start_gpu = cp.cuda.Event()
            end_gpu = cp.cuda.Event()
            start_gpu.record()
            idx_simp = [0]
            if layers_g.shape[0] > 1:
                l_idx, m_idx = 0, 1
                diff_h = layers_g[1:] - layers_g[:-1]
                while m_idx < n_slice_init - 2:
                    mask_l_g = layers_g[m_idx] - layers_g[l_idx] > 0
                    mask_l_t = buffers["inflated_cost"][l_idx] > buffers["inflated_cost"][m_idx]
                    mask_u_g = diff_h[m_idx] > 0
                    mask_t = buffers["inflated_cost"][m_idx] < self.cfg.cost_barrier
                    unique = (mask_l_g | mask_l_t) & mask_u_g & mask_t
                    if cp.any(unique):
                        idx_simp.append(m_idx)
                        l_idx = m_idx
                    m_idx += 1
                idx_simp.append(m_idx)

            trav_grad_x = buffers["inflated_cost"][idx_simp][:, 2:, :] - buffers["inflated_cost"][idx_simp][:, :-2, :]
            trav_grad_y = buffers["inflated_cost"][idx_simp][:, :, 2:] - buffers["inflated_cost"][idx_simp][:, :, :-2]
            end_gpu.record()
            end_gpu.synchronize()
            timings["t_simp"] += cp.cuda.get_elapsed_time(start_gpu, end_gpu)

            layers_t = buffers["inflated_cost"][idx_simp].get()
            layers_g_out = cp.where(layers_g[idx_simp] > -1e6, layers_g[idx_simp], cp.nan).get()
            layers_c_out = cp.where(buffers["layers_c"][idx_simp] < 1e6, buffers["layers_c"][idx_simp], cp.nan).get()
            trav_gx = np.zeros_like(layers_g_out)
            trav_gx[:, 1:-1, :] = trav_grad_x.get()
            trav_gy = np.zeros_like(layers_g_out)
            trav_gy[:, :, 1:-1] = trav_grad_y.get()
            timings["t_all"] += (time.time() - start_all) * 1e3
            final = layers_t, trav_gx, trav_gy, layers_g_out, layers_c_out

        for key in timings:
            timings[key] /= repeat

        tomogram = np.stack(final)
        metadata = {
            "resolution": self.cfg.resolution,
            "center": center,
            "slice_h0": slice_h0,
            "slice_dh": self.cfg.slice_dh,
            "map_dim_x": map_dim_x,
            "map_dim_y": map_dim_y,
            "n_slice_init": n_slice_init,
            "n_slice": int(tomogram.shape[1]),
            "timings_ms": timings,
        }
        return tomogram, metadata


def save_tomogram_pickle(path, tomogram, metadata):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    data_dict = {
        "data": tomogram.astype(np.float16),
        "resolution": metadata["resolution"],
        "center": metadata["center"],
        "slice_h0": metadata["slice_h0"],
        "slice_dh": metadata["slice_dh"],
    }
    with open(path, "wb") as handle:
        pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
