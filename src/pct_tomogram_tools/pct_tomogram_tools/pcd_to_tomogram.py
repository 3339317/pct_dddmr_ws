import argparse
import os

import numpy as np
import rclpy
from rclpy.node import Node

from .converter import TomogramConfig, TomogramConverter, save_tomogram_pickle


SCENE_PRESETS = {
    "building": {
        "resolution": 0.10,
        "ground_h": 0.0,
        "slice_dh": 0.5,
        "kernel_size": 7,
        "interval_min": 0.50,
        "interval_free": 0.65,
        "slope_max": 0.40,
        "step_max": 0.17,
        "standable_ratio": 0.20,
        "cost_barrier": 50.0,
        "safe_margin": 0.4,
        "inflation": 0.2,
    },
    "plaza": {
        "resolution": 0.30,
        "ground_h": 0.0,
        "slice_dh": 0.5,
        "kernel_size": 7,
        "interval_min": 0.30,
        "interval_free": 0.65,
        "slope_max": 1.20,
        "step_max": 0.40,
        "standable_ratio": 0.10,
        "cost_barrier": 30.0,
        "safe_margin": 0.1,
        "inflation": 0.05,
    },
    "spiral": {
        "resolution": 0.20,
        "ground_h": 0.0,
        "slice_dh": 0.5,
        "kernel_size": 7,
        "interval_min": 0.50,
        "interval_free": 0.65,
        "slope_max": 0.40,
        "step_max": 0.30,
        "standable_ratio": 0.40,
        "cost_barrier": 50.0,
        "safe_margin": 1.2,
        "inflation": 0.2,
    },
}


def _arg_or_preset(args, name):
    value = getattr(args, name)
    if value is not None:
        return value
    return SCENE_PRESETS[args.scene_preset][name]


class PcdToTomogramNode(Node):
    def __init__(self, args):
        super().__init__("pcd_to_tomogram")
        self.args = args

    def run(self):
        try:
            import open3d as o3d
        except ImportError as exc:
            raise RuntimeError("open3d is required. Install it before running pcd_to_tomogram.") from exc

        pcd_path = os.path.abspath(os.path.expanduser(self.args.pcd))
        output_path = os.path.abspath(os.path.expanduser(self.args.output))

        self.get_logger().info(f"Loading PCD: {pcd_path}")
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points, dtype=np.float32)
        if points.size == 0:
            raise RuntimeError(f"No points loaded from {pcd_path}")

        self.get_logger().info(f"PCD points: {points.shape[0]}")
        self.get_logger().info(f"Using PCT scene preset: {self.args.scene_preset}")
        cfg = TomogramConfig(
            resolution=_arg_or_preset(self.args, "resolution"),
            ground_h=_arg_or_preset(self.args, "ground_h"),
            slice_dh=_arg_or_preset(self.args, "slice_dh"),
            kernel_size=_arg_or_preset(self.args, "kernel_size"),
            interval_min=_arg_or_preset(self.args, "interval_min"),
            interval_free=_arg_or_preset(self.args, "interval_free"),
            slope_max=_arg_or_preset(self.args, "slope_max"),
            step_max=_arg_or_preset(self.args, "step_max"),
            standable_ratio=_arg_or_preset(self.args, "standable_ratio"),
            cost_barrier=_arg_or_preset(self.args, "cost_barrier"),
            safe_margin=_arg_or_preset(self.args, "safe_margin"),
            inflation=_arg_or_preset(self.args, "inflation"),
            repeat=self.args.repeat,
        )
        self.get_logger().info(f"Tomogram config: {cfg}")

        converter = TomogramConverter(cfg)
        tomogram, metadata = converter.convert(points)
        save_tomogram_pickle(output_path, tomogram, metadata)

        trav = tomogram[0]
        ground = tomogram[3]
        valid = np.isfinite(ground)
        free = valid & (trav < 5.0)
        inflated = valid & (trav >= 5.0) & (trav < cfg.cost_barrier * 0.9)
        blocked = valid & (trav >= cfg.cost_barrier * 0.9)

        self.get_logger().info(f"Tomogram saved: {output_path}")
        self.get_logger().info(
            f"shape={tomogram.shape}, resolution={metadata['resolution']}, "
            f"center={metadata['center']}, slice_h0={metadata['slice_h0']:.3f}, "
            f"slice_dh={metadata['slice_dh']:.3f}"
        )
        self.get_logger().info(
            "cells: "
            f"valid_ground={int(valid.sum())}, "
            f"free(<5)={int(free.sum())}, "
            f"inflated(5~barrier*0.9)={int(inflated.sum())}, "
            f"blocked(>=barrier*0.9)={int(blocked.sum())}"
        )
        self.get_logger().info(f"avg timings ms: {metadata['timings_ms']}")


def build_parser():
    parser = argparse.ArgumentParser(description="Convert a PCD point cloud to a PCT tomogram .pickle map.")
    parser.add_argument("--pcd", required=True, help="Input .pcd file path.")
    parser.add_argument("--output", required=True, help="Output .pickle file path.")
    parser.add_argument(
        "--scene-preset",
        choices=sorted(SCENE_PRESETS.keys()),
        default="plaza",
        help="Use original PCT scene defaults. Override any individual parameter below if needed.",
    )
    parser.add_argument("--resolution", type=float, default=None)
    parser.add_argument("--ground-h", type=float, default=None)
    parser.add_argument("--slice-dh", type=float, default=None)
    parser.add_argument("--kernel-size", type=int, default=None)
    parser.add_argument("--interval-min", type=float, default=None)
    parser.add_argument("--interval-free", type=float, default=None)
    parser.add_argument("--slope-max", type=float, default=None)
    parser.add_argument("--step-max", type=float, default=None)
    parser.add_argument("--standable-ratio", type=float, default=None)
    parser.add_argument("--cost-barrier", type=float, default=None)
    parser.add_argument("--safe-margin", type=float, default=None)
    parser.add_argument("--inflation", type=float, default=None)
    parser.add_argument("--repeat", type=int, default=1, help="Repeat conversion for timing average. Use 1 normally.")
    return parser


def main(args=None):
    rclpy.init(args=args)
    ros_args = rclpy.utilities.remove_ros_args()[1:]
    parsed = build_parser().parse_args(ros_args)
    node = PcdToTomogramNode(parsed)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
