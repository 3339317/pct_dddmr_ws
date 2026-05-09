# pct_dddmr_web

`pct_dddmr_web` 是当前导航系统的 Web 控制台，负责显示 PCT `.pickle` 地图、机器人定位、全局路径、障碍物状态，并把网页选点和控制参数同步给后端路线追踪器。

## 功能

- 通过 FastAPI 提供浏览器页面，默认端口 `8000`。
- 直接读取 `.pickle` tomogram 地图用于网页显示。
- 订阅 `/localization` 显示机器人当前位置。
- 订阅 `/global_path` 显示 PCT 全局规划结果。
- 订阅 `/pct_dddmr_web/controller/state` 显示路线追踪和避障状态。
- 调用 PCT `get_dwa_plan` action 生成全局路径。
- 发布路线、参数、开始、停止、清空命令给 `route_tracker_node`。
- 支持高度层筛选，方便多楼层、楼梯、坡道地图选取目标点。

## 启动

通常不需要单独启动本包，主导航 launch 会自动启动：

```bash
ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py \
  tomogram_path:=/home/if/pct_dddmr_ws/test.pickle
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

- `/localization`：`nav_msgs/Odometry`，定位输出。
- `/global_path`：`nav_msgs/Path`，PCT 全局路径。
- `/pct_dddmr_web/controller/state`：`std_msgs/String`，路线追踪状态 JSON。
- `/pct_dddmr_web/controller/status`：`std_msgs/String`，后端状态提示。
- `/nav/done`：`std_msgs/String`，导航完成状态。

发布：

- `/pct_dddmr_web/controller/route`：`nav_msgs/Path`，网页选择并规划后的路线。
- `/pct_dddmr_web/controller/config`：`std_msgs/String`，网页控制参数 JSON。
- `/pct_dddmr_web/controller/start`：`std_msgs/Empty`，开始导航。
- `/pct_dddmr_web/controller/stop`：`std_msgs/Empty`，停止导航。
- `/pct_dddmr_web/controller/clear`：`std_msgs/Empty`，清空导航。
- `/initialpose`：`geometry_msgs/PoseWithCovarianceStamped`，网页设置初始位姿。

Action：

- `get_dwa_plan`：PCT 全局规划 action。

## 当前执行流程

```text
Web 选点
  -> get_dwa_plan 生成 PCT 全局路径
  -> /pct_dddmr_web/controller/route
  -> route_tracker_node
  -> /cmd_vel
```

避障链路：

```text
/livox/lidar CustomMsg
  -> mcl_feature
  -> /segmented_cloud_pure
  -> route_tracker_node
```

Web 端不直接发布导航过程中的 `/cmd_vel`。

## 测试定位

当前主导航链路推荐使用 `pct_dddmr_nav` 中的 `/localization` 测试脚本：

```bash
ros2 run pct_dddmr_nav test_localization_publisher \
  --x 0 --y 0 --z 0 --yaw 0
```

## 注意事项

- 本包不负责 PCD 到 `.pickle` 的地图转换。
- 本包不包含 FAST-LIO 定位和 MID360 驱动。
- 旧 `/pcl_pose` 测试入口已经移除，当前统一使用 `/localization`。
- 如果机器人位置不更新，优先检查 `/localization`。
- 如果高楼层目标点选不中，打开网页左侧的高度层筛选后再选点。
