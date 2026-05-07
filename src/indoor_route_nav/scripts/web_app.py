#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import io
import time
import json
import yaml
import socket
import signal
import threading
import subprocess
import sys
import shutil
import asyncio
import math
import argparse

import numpy as np

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

from fastapi import FastAPI, UploadFile, File, Body, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from core_ros import core, ros_runner

_parser = argparse.ArgumentParser(add_help=False)
_parser.add_argument("--tomogram-path", default="")
_args, _unknown = _parser.parse_known_args()
if _args.tomogram_path:
    os.environ["PCT_TOMOGRAM_PATH"] = _args.tomogram_path

APP_PORT = 8000
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
if not os.path.isdir(STATIC_DIR) and get_package_share_directory is not None:
    try:
        STATIC_DIR = os.path.join(get_package_share_directory("indoor_route_nav"), "static")
    except Exception:
        pass
INDEX_FILE = os.path.join(STATIC_DIR, "index.html")
DEFAULT_CONFIG_FILE = os.path.join(BASE_DIR, "config", "default_params.yaml")
if get_package_share_directory is not None:
    try:
        share_dir = get_package_share_directory("indoor_route_nav")
        share_default_config = os.path.join(share_dir, "config", "default_params.yaml")
        if os.path.exists(share_default_config):
            DEFAULT_CONFIG_FILE = share_default_config
    except Exception:
        pass

app = FastAPI(title="ROS2 Web Route Nav 3D")

if os.path.isdir(STATIC_DIR):
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


# =========================================================
# 工具
# =========================================================
def ensure_parent_dir(file_path: str):
    parent = os.path.dirname(os.path.abspath(file_path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent, exist_ok=True)


def build_topics_dict():
    return {
        "camera_topic": getattr(core, "camera_topic", ""),
        "nav_start_topic": getattr(core, "nav_start_topic", ""),
        "nav_stop_topic": getattr(core, "nav_stop_topic", ""),
        "nav_done_topic": getattr(core, "nav_done_topic", ""),
        "initialpose_frame": getattr(core, "initialpose_frame", "map"),
    }


def export_full_state():
    if hasattr(core, "export_state"):
        state = core.export_state()
    else:
        state = {}

    if not isinstance(state, dict):
        state = {}

    if "points_xyz" not in state:
        pts_xy = state.get("points_xy", [])
        pts_xyz = []
        try:
            for p in pts_xy:
                pts_xyz.append([float(p[0]), float(p[1]), 0.0])
        except Exception:
            pts_xyz = []
        state["points_xyz"] = pts_xyz

    if "points_rgb" not in state:
        state["points_rgb"] = []

    if "map_layers" not in state or not isinstance(state["map_layers"], dict):
        state["map_layers"] = {
            "ground": state.get("points_xyz", []),
            "inflation": [],
            "obstacle": [],
        }
    else:
        state["map_layers"].setdefault("ground", state.get("points_xyz", []))
        state["map_layers"].setdefault("inflation", [])
        state["map_layers"].setdefault("obstacle", [])

    if "topics" not in state or not isinstance(state["topics"], dict):
        state["topics"] = {}

    topics = state["topics"]
    for k, v in build_topics_dict().items():
        topics.setdefault(k, v)
    state["topics"] = topics

    if "camera" not in state or not isinstance(state["camera"], dict):
        state["camera"] = {"topic": topics.get("camera_topic", "")}
    else:
        state["camera"].setdefault("topic", topics.get("camera_topic", ""))

    if "nav" not in state or not isinstance(state["nav"], dict):
        state["nav"] = {}

    state.setdefault("map_is_3d", True)

    if "revisions" not in state:
        state["revisions"] = {
            "map": getattr(core, "map_revision", 0),
            "route": getattr(core, "route_revision", 0),
        }

    return state


def build_compact_state():
    if hasattr(core, "export_compact_state"):
        try:
            st = core.export_compact_state()
            if isinstance(st, dict):
                st["topics"] = st.get("topics", build_topics_dict())
                st["type"] = "compact_state"
                if "revisions" not in st:
                    st["revisions"] = {
                        "map": getattr(core, "map_revision", 0),
                        "route": getattr(core, "route_revision", 0),
                    }
                return st
        except Exception as e:
            import traceback
            print(f"[build_compact_state] export_compact_state ERROR: {e}", flush=True)
            traceback.print_exc()

    print("[build_compact_state] FALLBACK path used", flush=True)

    full = export_full_state()
    return {
        "type": "compact_state",
        "status_text": full.get("status_text", "等待操作"),
        "localized": full.get("localized", False),
        "current_pose": full.get("current_pose"),
        "current_vx": full.get("current_vx", 0.0),
        "current_wz": full.get("current_wz", 0.0),
        "nav": full.get("nav", {}),
        "camera": full.get("camera", {}),
        "pcd_path": full.get("pcd_path", ""),
        "route_count": len(full.get("route_polyline", []) or []),
        "route_polyline": full.get("route_polyline", []),
        "topics": full.get("topics", build_topics_dict()),
        "revisions": full.get("revisions", {
            "map": 0, "route": 0
        }),
    }


def yaw_deg_to_quaternion(yaw_deg):
    yaw = math.radians(float(yaw_deg))
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return {"x": 0.0, "y": 0.0, "z": sy, "w": cy}


def apply_route_points(points):
    if hasattr(core, "set_route_points"):
        core.set_route_points(points)
        ros_runner.sync_controller_route()
        return
    if hasattr(core, "set_external_route"):
        core.set_external_route(points)
        ros_runner.sync_controller_route()
        return
    raise RuntimeError("core 不支持设置路线")


def _point_like_to_xyz(item):
    if isinstance(item, dict):
        if "x" in item and "y" in item:
            return {
                "x": float(item["x"]),
                "y": float(item["y"]),
                "z": float(item.get("z", 0.0)),
            }

        if "position" in item and isinstance(item["position"], dict):
            pos = item["position"]
            return {
                "x": float(pos["x"]),
                "y": float(pos["y"]),
                "z": float(pos.get("z", 0.0)),
            }

        if "pose" in item and isinstance(item["pose"], dict):
            pose = item["pose"]
            if "position" in pose and isinstance(pose["position"], dict):
                pos = pose["position"]
                return {
                    "x": float(pos["x"]),
                    "y": float(pos["y"]),
                    "z": float(pos.get("z", 0.0)),
                }
            if "x" in pose and "y" in pose:
                return {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "z": float(pose.get("z", 0.0)),
                }

    if isinstance(item, (list, tuple, np.ndarray)):
        arr = np.asarray(item).reshape(-1)
        if arr.size >= 2:
            return {
                "x": float(arr[0]),
                "y": float(arr[1]),
                "z": float(arr[2]) if arr.size >= 3 else 0.0,
            }

    raise RuntimeError(f"无法解析点: {item}")


def _extract_points_from_ndarray(arr):
    arr = np.asarray(arr)

    if arr.dtype == object:
        if arr.ndim == 0:
            return _extract_points_from_any(arr.item())
        return _extract_points_from_any(arr.tolist())

    if arr.ndim == 2 and arr.shape[1] >= 2:
        pts = []
        for row in arr:
            pts.append({
                "x": float(row[0]),
                "y": float(row[1]),
                "z": float(row[2]) if arr.shape[1] >= 3 else 0.0,
            })
        if len(pts) >= 2:
            return pts

    raise RuntimeError("ndarray中未找到可用路线点")


def _extract_points_from_any(obj):
    search_keys = [
        "points",
        "route_points",
        "route_polyline",
        "path",
        "trajectory",
        "polyline",
        "merged_result",
        "merged_points",
        "result",
    ]

    if isinstance(obj, np.ndarray):
        return _extract_points_from_ndarray(obj)

    if isinstance(obj, dict):
        candidates = []

        if "route" in obj:
            rv = obj["route"]
            if isinstance(rv, dict):
                for k in search_keys:
                    if k in rv:
                        candidates.append(rv[k])
            else:
                candidates.append(rv)

        for k in search_keys:
            if k in obj:
                candidates.append(obj[k])

        if "poses" in obj and isinstance(obj["poses"], list):
            candidates.append(obj["poses"])

        for cand in candidates:
            try:
                pts = _extract_points_from_any(cand)
                if len(pts) >= 2:
                    return pts
            except Exception:
                pass

        raise RuntimeError("文件中未找到可用路线点")

    if isinstance(obj, (list, tuple)):
        if len(obj) == 0:
            raise RuntimeError("路线为空")
        pts = []
        for item in obj:
            pts.append(_point_like_to_xyz(item))
        if len(pts) < 2:
            raise RuntimeError("路线点数不足")
        return pts

    raise RuntimeError("不支持的路线数据格式")


def parse_route_json_bytes(content: bytes):
    data = json.loads(content.decode("utf-8"))
    return _extract_points_from_any(data)


def parse_route_npz_bytes(content: bytes):
    bio = io.BytesIO(content)
    npz = np.load(bio, allow_pickle=True)

    candidate_keys = [
        "points",
        "route_points",
        "route_polyline",
        "path",
        "trajectory",
        "polyline",
        "merged_result",
        "merged_points",
        "result",
        "arr_0",
    ]

    for k in candidate_keys:
        if k in npz.files:
            try:
                pts = _extract_points_from_any(npz[k])
                if len(pts) >= 2:
                    return pts
            except Exception:
                pass

    for k in npz.files:
        try:
            pts = _extract_points_from_any(npz[k])
            if len(pts) >= 2:
                return pts
        except Exception:
            pass

    raise RuntimeError("NPZ中未找到可用路线点")


def parse_structured_upload(content: bytes, filename: str):
    ext = os.path.splitext(filename)[1].lower()
    text = content.decode("utf-8")

    if ext == ".json":
        return json.loads(text)

    try:
        return yaml.safe_load(text) or {}
    except Exception:
        return json.loads(text)


def get_default_system_params_path():
    return os.path.abspath(DEFAULT_CONFIG_FILE)


def load_default_system_params():
    if not os.path.exists(DEFAULT_CONFIG_FILE):
        return
    with open(DEFAULT_CONFIG_FILE, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if isinstance(data, dict):
        core.load_system_params_data(data)


def load_tomogram_map_from_config():
    tomogram_path = os.environ.get("PCT_TOMOGRAM_PATH") or getattr(core, "tomogram_path", "")
    if not tomogram_path:
        core.set_status("未配置PCT地图文件，网页地图将等待备用ROS地图话题")
        return

    if not os.path.exists(os.path.abspath(os.path.expanduser(tomogram_path))):
        core.set_status(f"PCT地图文件不存在: {tomogram_path}")
        return

    core._map.load_tomogram_file(
        tomogram_path,
        sample_step=getattr(core, "sample_step", 3),
        ground_max_trav=getattr(core, "tomogram_ground_max_trav", 5.0),
        inflation_min_trav=getattr(core, "tomogram_inflation_min_trav", 5.0),
        obstacle_min_trav=getattr(core, "tomogram_obstacle_min_trav", 45.0),
        obstacle_stride_multiplier=getattr(core, "tomogram_obstacle_stride_multiplier", 1),
    )
    core._bump_map_rev()
    core.set_status(f"已直接加载PCT地图文件: {tomogram_path}")


def reapply_route_sampling_if_possible():
    if not hasattr(core, "resample_polyline"):
        return
    if not hasattr(core, "build_route_cumlen"):
        return
    if not hasattr(core, "route_polyline"):
        return

    with core.lock:
        route = getattr(core, "route_polyline", []) or []
        if len(route) < 2:
            return

        base = [dict(p) for p in route]
        core.route_polyline = core.resample_polyline(base, core.route_sample_gap)
        core.route_cumlen = core.build_route_cumlen(core.route_polyline)

        if hasattr(core, "_bump_route_rev"):
            core._bump_route_rev()

        core.set_status("已应用路径采样参数")


def estimate_cloud_surface_z_at(x, y, hint_z=None, radius=0.60, z_window=1.20):
    with core.lock:
        raw_points = list(getattr(core, "points_xyz", []) or [])
    if not raw_points:
        return None

    x = float(x)
    y = float(y)
    hint = float(hint_z) if hint_z is not None else None
    radius = max(0.05, float(radius))
    z_window = max(0.05, float(z_window))

    weighted = 0.0
    weight_sum = 0.0
    best = None
    best_score = 1e18
    r2 = radius * radius
    step = max(1, int(len(raw_points) / 120000))

    for item in raw_points[::step]:
        try:
            p = _point_like_to_xyz(item)
        except Exception:
            continue
        dx = float(p["x"]) - x
        dy = float(p["y"]) - y
        d2 = dx * dx + dy * dy
        if d2 > r2:
            continue
        pz = float(p.get("z", 0.0))
        dz = abs(pz - hint) if hint is not None else 0.0
        if hint is not None and dz > z_window:
            continue
        d = math.sqrt(d2)
        w = 1.0 / max(0.03, d)
        weighted += pz * w
        weight_sum += w
        score = d + dz * 0.20
        if score < best_score:
            best = pz
            best_score = score

    if weight_sum > 0.0:
        return float(weighted / weight_sum)
    return best


def project_pose_to_cloud_surface(point, label, radius=0.80, z_window=1.20):
    p = dict(point)
    z = estimate_cloud_surface_z_at(
        p.get("x", 0.0),
        p.get("y", 0.0),
        hint_z=p.get("z", None),
        radius=radius,
        z_window=z_window,
    )
    if z is not None:
        p["z"] = float(z)
        return p, f"{label}高度已投影到脚下点云 z={z:.2f}"
    return p, f"{label}附近未找到可用点云高度，沿用 z={float(p.get('z', 0.0)):.2f}"


# =========================================================
# FastAPI 生命周期
# =========================================================
@app.on_event("startup")
async def startup_event():
    try:
        load_default_system_params()
        load_tomogram_map_from_config()
    except Exception as e:
        core.set_status(f"默认参数读取失败: {e}")

    ros_runner.start()


@app.on_event("shutdown")
async def shutdown_event():
    ros_runner.stop()


# =========================================================
# 页面
# =========================================================
@app.get("/")
async def index():
    if not os.path.exists(INDEX_FILE):
        return Response(
            content="找不到 static/index.html，请检查目录结构。",
            media_type="text/plain; charset=utf-8"
        )
    return FileResponse(
        INDEX_FILE,
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
        },
    )


# =========================================================
# 状态
# =========================================================
@app.get("/api/state")
async def get_state():
    return export_full_state()


@app.get("/api/state/compact")
async def get_state_compact():
    return build_compact_state()


# =========================================================
# 参数
# =========================================================
@app.post("/api/params/control")
async def set_control_params(payload: dict = Body(...)):
    try:
        core.set_control_params(payload)
        ros_runner.sync_controller_config()
        core.set_status("已应用控制参数")
        return {"ok": True}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/params/smoothing")
async def set_smoothing_params(payload: dict = Body(...)):
    try:
        core.set_smoothing_params(payload)
        reapply_route_sampling_if_possible()
        return {"ok": True}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


# =========================================================
# 路线：读取 JSON / NPZ
# =========================================================
@app.post("/api/route/upload-json")
async def route_upload_json(file: UploadFile = File(...)):
    try:
        content = await file.read()
        points = parse_route_json_bytes(content)
        apply_route_points(points)
        return {"ok": True, "message": f"已加载 JSON 路线: {file.filename}", "count": len(points)}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"JSON 路线读取失败: {e}"})


@app.post("/api/route/upload-npz")
async def route_upload_npz(file: UploadFile = File(...)):
    try:
        content = await file.read()
        points = parse_route_npz_bytes(content)
        apply_route_points(points)
        return {"ok": True, "message": f"已加载 NPZ 路线: {file.filename}", "count": len(points)}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"NPZ 路线读取失败: {e}"})


# =========================================================
# 路线：直接设置路线点（手绘路线提交）
# =========================================================
@app.post("/api/route/set-points")
async def route_set_points(payload: dict = Body(...)):
    try:
        points = payload.get("points", [])
        if not isinstance(points, list) or len(points) < 2:
            return JSONResponse({"ok": False, "message": "路线点数不足（至少需要2个点）"})
        apply_route_points(points)
        return {"ok": True, "message": f"已设置手绘路线，共 {len(points)} 点", "count": len(points)}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"设置路线失败: {e}"})


# =========================================================
# 路线：保存路线到文件
# =========================================================
@app.post("/api/route/save-to-path")
async def route_save_to_path(payload: dict = Body(...)):
    try:
        file_path = payload.get("file_path", "route.json")
        ensure_parent_dir(file_path)
        
        if hasattr(core, "export_state"):
            state = core.export_state()
        else:
            state = {}
        
        points = state.get("route_polyline", [])
        if not points or len(points) < 2:
            return JSONResponse({"ok": False, "message": "路线点数不足，无法保存"})
        
        # 转换为列表格式
        export_data = {
            "route_points": [
                {
                    "x": float(p.get("x", 0)),
                    "y": float(p.get("y", 0)),
                    "z": float(p.get("z", 0))
                } for p in points
            ]
        }
        
        with open(file_path, "w") as f:
            json.dump(export_data, f, indent=2)
        
        return {"ok": True, "message": f"已保存路线到: {file_path}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"保存失败: {e}"})


# =========================================================
# 系统参数
# =========================================================
@app.post("/api/config/save-system-params")
async def config_save_system_params():
    try:
        yml = core.dump_system_params_yaml()
        return Response(
            content=yml,
            media_type="application/x-yaml",
            headers={"Content-Disposition": "attachment; filename=system_params.yaml"}
        )
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"系统参数导出失败: {e}"})


@app.post("/api/config/load-system-params")
async def config_load_system_params(file: UploadFile = File(...)):
    try:
        content = await file.read()
        data = yaml.safe_load(content.decode("utf-8")) or {}
        core.load_system_params_data(data)
        return {"ok": True}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"系统参数配置读取失败: {e}"})


@app.get("/api/params")
async def get_system_params():
    try:
        payload = {
            "ok": True,
            "params": core.export_system_params(),
            "default_config_path": get_default_system_params_path(),
        }
        return payload
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/config/save-default-system-params")
async def config_save_default_system_params():
    try:
        file_path = get_default_system_params_path()
        yml = core.dump_system_params_yaml()
        ensure_parent_dir(file_path)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(yml)
        return {"ok": True, "message": f"已保存默认参数到: {file_path}", "file_path": file_path}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"保存默认参数失败: {e}"})


@app.post("/api/config/save-system-params-to-path")
async def config_save_system_params_to_path(payload: dict = Body(...)):
    try:
        file_path = str(payload.get("file_path", "")).strip()
        if not file_path:
            return JSONResponse({"ok": False, "message": "file_path 不能为空"})

        if not (file_path.endswith(".yaml") or file_path.endswith(".yml")):
            return JSONResponse({"ok": False, "message": "文件名必须以 .yaml 或 .yml 结尾"})

        yml = core.dump_system_params_yaml()
        ensure_parent_dir(file_path)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(yml)

        return {"ok": True, "message": f"已保存参数到: {file_path}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"保存失败: {e}"})


# =========================================================
# 导航
# =========================================================
@app.post("/api/nav/set-goal")
async def nav_set_goal(payload: dict = Body(...)):
    """设置导航目标点，dddmr 会自动规划路径并发布到 /global_path"""
    try:
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        z = float(payload.get("z", 0.0))
        yaw_deg = float(payload.get("yaw_deg", 0.0))

        current_pose = getattr(core, "current_pose", None)
        if not current_pose:
            raise RuntimeError("尚未收到 /localization，无法确定规划起点")

        # Web UI still expects a route with at least two points before it
        # starts navigation. DDDMR/PCT will do the real global planning after
        # p2p_move_base receives the last pose as its goal.
        route_points = [
            {
                "x": float(current_pose.get("x", 0.0)),
                "y": float(current_pose.get("y", 0.0)),
                "z": float(current_pose.get("z", 0.0)),
            },
            {"x": x, "y": y, "z": z},
        ]
        apply_route_points(route_points)
        ros_runner.sync_controller_route()

        # 触发导航（fusion bridge 会将路线最后一个点作为目标发给 p2p_move_base）
        ros_runner.start_navigation()

        return {"ok": True, "message": f"已设置导航目标 ({x:.2f}, {y:.2f}, {z:.2f})"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"设置目标失败: {e}"})


@app.post("/api/nav/start")
async def nav_start():
    try:
        ros_runner.start_navigation()
        return {"ok": True, "message": "导航已启动"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/nav/stop")
async def nav_stop():
    try:
        ros_runner.stop_navigation()
        return {"ok": True, "message": "导航已停止"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/nav/emergency")
async def nav_emergency():
    try:
        ros_runner.emergency_stop_navigation()
        return {"ok": True, "message": "已紧急停止"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/nav/clear")
async def nav_clear():
    try:
        ros_runner.clear_navigation()
        return {"ok": True, "message": "已清空当前路线"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/ros/initialpose")
async def ros_initialpose(payload: dict = Body(...)):
    try:
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        z = float(payload.get("z", 0.0))
        yaw_deg = float(payload.get("yaw_deg", 0.0))
        ros_runner.publish_initial_pose(x, y, z, yaw_deg)
        return {"ok": True, "message": f"已发布 /initialpose"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


# 兼容旧接口
@app.post("/api/route/clear")
async def route_clear_compat():
    return await nav_clear()


# =========================================================
# 底盘控制
# =========================================================
@app.post("/api/chassis/cmd_vel")
async def chassis_cmd_vel(payload: dict = Body(...)):
    try:
        vx = float(payload.get("vx", 0.0))
        vy = float(payload.get("vy", 0.0))
        vz = float(payload.get("vz", 0.0))
        ros_runner.chassis_cmd_vel(vx, vy, vz)
        return {"ok": True, "message": f"cmd_vel: vx={vx:.2f} vy={vy:.2f} wz={vz:.2f}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"cmd_vel FAIL: {e}"})


@app.post("/api/chassis/mode")
async def chassis_mode(payload: dict = Body(...)):
    try:
        mode = str(payload.get("mode", "stand"))
        enable = bool(payload.get("enable", True))
        ros_runner.chassis_mode(mode, enable)
        return {"ok": True, "message": f"chassis mode: {mode} enable={enable}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"mode FAIL: {e}"})


@app.post("/api/chassis/sitdown")
async def chassis_sitdown():
    try:
        ros_runner.chassis_sitdown()
        return {"ok": True, "message": "sitdown 已发送"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"sitdown FAIL: {e}"})


@app.post("/api/chassis/emgy_stop")
async def chassis_emgy_stop():
    try:
        ros_runner.chassis_emgy_stop()
        return {"ok": True, "message": "紧急停止已发送"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"emgy_stop FAIL: {e}"})


@app.post("/api/chassis/imu_enable")
async def chassis_imu_enable(payload: dict = Body(...)):
    try:
        enable = bool(payload.get("enable", True))
        ros_runner.chassis_imu_enable(enable)
        return {"ok": True, "message": f"IMU enable={enable}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"imu_enable FAIL: {e}"})


# =========================================================
# ROS 图像 MJPEG
# =========================================================
def mjpeg_generator():
    last_sent_time = 0.0
    while True:
        try:
            if hasattr(core, "get_camera_frame"):
                frame, ts = core.get_camera_frame()
            else:
                frame, ts = None, 0.0
        except Exception:
            frame, ts = None, 0.0

        if frame is not None and ts != last_sent_time:
            last_sent_time = ts
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                frame +
                b"\r\n"
            )
        else:
            time.sleep(0.03)


@app.get("/api/camera/mjpeg")
async def camera_mjpeg():
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


# =========================================================
# 系统
# =========================================================
def delayed_shutdown():
    time.sleep(0.8)

    try:
        core.emergency_stop()
    except Exception:
        pass

    try:
        ros_runner.stop()
    except Exception:
        pass

    try:
        os.kill(os.getpid(), signal.SIGTERM)
    except Exception:
        pass

    time.sleep(0.5)
    sys.exit(0)


@app.post("/api/system/shutdown")
async def system_shutdown():
    threading.Thread(target=delayed_shutdown, daemon=True).start()
    return {"ok": True, "message": "程序即将关闭"}


# =========================================================
# WebSocket：只发轻量状态
# =========================================================
@app.websocket("/ws/status")
async def ws_status(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = build_compact_state()
            await websocket.send_text(json.dumps(data, ensure_ascii=False))
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        print("[WS] client disconnected", flush=True)
        return
    except Exception as e:
        print(f"[WS] error: {e}", flush=True)
        return


# =========================================================
# 启动辅助
# =========================================================
def show_startup_native_popup(local_url: str, lan_url: str):
    text = f"本机访问:  {local_url}\n局域网访问: {lan_url}"

    try:
        if shutil.which("zenity"):
            subprocess.Popen([
                "zenity",
                "--info",
                "--title=程序启动成功",
                "--width=520",
                "--height=220",
                f"--text={text}"
            ])
            return
    except Exception:
        pass

    try:
        if shutil.which("xmessage"):
            subprocess.Popen([
                "xmessage",
                "-center",
                text
            ])
            return
    except Exception:
        pass

    try:
        def _popup():
            import tkinter as tk
            root = tk.Tk()
            root.title("程序启动成功")
            root.geometry("520x180")
            root.resizable(False, False)

            try:
                root.attributes("-topmost", True)
            except Exception:
                pass

            frame = tk.Frame(root, padx=16, pady=16)
            frame.pack(fill="both", expand=True)

            title = tk.Label(frame, text="程序已启动，可通过以下地址访问：", font=("Arial", 12, "bold"))
            title.pack(anchor="w", pady=(0, 12))

            local_var = tk.StringVar(value=local_url)
            lan_var = tk.StringVar(value=lan_url)

            tk.Label(frame, text="本机访问:", anchor="w").pack(fill="x")
            e1 = tk.Entry(frame, textvariable=local_var, font=("Arial", 10))
            e1.pack(fill="x", pady=(2, 8))

            tk.Label(frame, text="局域网访问:", anchor="w").pack(fill="x")
            e2 = tk.Entry(frame, textvariable=lan_var, font=("Arial", 10))
            e2.pack(fill="x", pady=(2, 12))

            btn_frame = tk.Frame(frame)
            btn_frame.pack(fill="x")

            def copy_local():
                root.clipboard_clear()
                root.clipboard_append(local_url)
                root.update()

            def copy_lan():
                root.clipboard_clear()
                root.clipboard_append(lan_url)
                root.update()

            tk.Button(btn_frame, text="复制本机地址", command=copy_local).pack(side="left", padx=(0, 8))
            tk.Button(btn_frame, text="复制局域网地址", command=copy_lan).pack(side="left", padx=(0, 8))
            tk.Button(btn_frame, text="关闭", command=root.destroy).pack(side="right")

            root.mainloop()

        threading.Thread(target=_popup, daemon=True).start()
    except Exception:
        pass


def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


# =========================================================
# 启动
# =========================================================
if __name__ == "__main__":
    ip = get_local_ip()
    local_url = f"http://127.0.0.1:{APP_PORT}"
    lan_url = f"http://{ip}:{APP_PORT}"

    topics = build_topics_dict()

    print("=" * 60)
    print(f"本机访问:  {local_url}")
    print(f"局域网访问: {lan_url}")
    print("=" * 60)
    print(f"图像话题: {topics.get('camera_topic', '-')}")
    print(f"开启导航话题: {topics.get('nav_start_topic', '-')}")
    print(f"停止导航话题: {topics.get('nav_stop_topic', '-')}")
    print(f"导航完成话题: {topics.get('nav_done_topic', '-')}")
    print(f"initialpose frame: {topics.get('initialpose_frame', 'map')}")

    show_startup_native_popup(local_url, lan_url)

    uvicorn.run(app, host="0.0.0.0", port=APP_PORT, reload=False)
