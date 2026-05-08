# pct_dddmr_web

`pct_dddmr_web` 是本项目的 Web 控制台和 ROS2 桥接包，负责浏览器显示 PCT `.pickle` 地图、机器人定位、全局路径，并把网页上的目标点和控制参数同步给 DDDMR 导航链路。

它不再使用原来的 `indoor_route_nav` 包名，部署到已有 `indoor_route_nav` 的机器上也不会产生 colcon 重名冲突。

## 功能

- 通过 FastAPI 提供浏览器页面，默认端口 `8000`。
- 订阅 `/localization` 显示机器人当前位置。
- 订阅 `/mapground`、`/mapcloud` 显示 PCT 地图中的地面、障碍物和膨胀层信息。
- 订阅 `/global_path` 显示 PCT 全局规划结果。
- 发布 `/pct_dddmr_web/controller/route`、`/pct_dddmr_web/controller/config`、`/pct_dddmr_web/controller/start` 给桥接节点。
- 支持高度层筛选，方便多楼层、楼梯、坡道地图选取目标点。

## 启动

通常不需要单独启动本包，主导航 launch 会自动启动：

```bash
ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py \
  tomogram_path:=/path/to/your_map.pickle
```

如果只想测试 Web 页面：

```bash
ros2 launch pct_dddmr_web pct_dddmr_web.launch.py
```

网页地址：

```text
http://127.0.0.1:8000
```

## 主要话题

订阅：

- `/localization`：`nav_msgs/Odometry`，外部定位输出。
- `/mapground`：`sensor_msgs/PointCloud2`，PCT 地面点云。
- `/mapcloud`：`sensor_msgs/PointCloud2`，PCT 障碍物/膨胀层点云。
- `/global_path`：`nav_msgs/Path`，PCT 全局路径。
- `/pct_dddmr_web/controller/state`：`std_msgs/String`，DDDMR 导航状态。

发布：

- `/pct_dddmr_web/controller/route`：`nav_msgs/Path`，网页选择的起点/终点路线请求。
- `/pct_dddmr_web/controller/config`：`std_msgs/String`，网页控制参数 JSON。
- `/pct_dddmr_web/controller/start`：`std_msgs/Empty`，开始导航。
- `/pct_dddmr_web/controller/stop`：`std_msgs/Empty`，停止导航。
- `/pct_dddmr_web/controller/clear`：`std_msgs/Empty`，清空导航。
- `/initialpose`：`geometry_msgs/PoseWithCovarianceStamped`，网页设置初始位姿。

## 与 DDDMR 的关系

Web 端只负责显示、选点、参数同步，不直接接管导航过程中的 `/cmd_vel`。

实际执行流程是：

```text
Web 目标点
  -> /pct_dddmr_web/controller/route
  -> indoor_fusion_bridge
  -> /p2p_move_base action
  -> PCT get_dwa_plan 全局规划
  -> DDDMR local_planner 局部避障和速度输出
```

## 测试定位

本包保留了一个简单的 `/pcl_pose` 测试脚本，主要用于兼容早期页面测试。当前主导航链路推荐使用 `pct_dddmr_nav` 中的 `/localization` 测试脚本：

```bash
ros2 run pct_dddmr_nav test_localization_publisher \
  --x 0 --y 0 --z 0 --yaw 0
```

## 注意事项

- 本包不负责 PCD 到 `.pickle` 的地图转换。
- 本包不包含 FAST-LIO 定位和 MID360 驱动。
- 如果机器人位置不更新，优先检查 `/localization` 和 `map -> base_link` TF。
- 如果高楼层目标点选不中，打开网页左侧的高度层筛选后再选点。
