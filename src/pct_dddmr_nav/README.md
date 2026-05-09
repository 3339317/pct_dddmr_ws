# pct_dddmr_nav

`pct_dddmr_nav` 是当前导航系统的主集成包，负责启动 PCT 全局规划、MID360 局部障碍特征、轻量路线追踪器和 Web 控制台。

当前主链路：

- 定位：`fast_lio_localization` 输出 `/localization`。
- 全局规划：PCT `.pickle` tomogram 地图，action 名称 `get_dwa_plan`。
- 路线追踪：`indoor_fusion_bridge/route_tracker_node` 订阅网页路线并发布 `/cmd_vel`。
- 局部避障：`lego_loam_bor/mcl_feature` 直接订阅 Livox `CustomMsg`，输出 `/segmented_cloud_pure` 给路线追踪器。
- 网页显示：`pct_dddmr_web` 直接读取 `.pickle` 地图，并显示定位、路径、障碍物状态。

旧的 `p2p_move_base/local_planner/perception_3d` 控制链和 Livox PointCloud2 转换节点已经从当前工作空间移除。

## 输入要求

运行导航前需要提供：

- `/localization`：`nav_msgs/Odometry`，机器人当前位姿。
- `/livox/lidar`：`livox_ros_driver2/msg/CustomMsg`，给 FAST-LIO 和 `mcl_feature` 使用。
- PCT 转换后的 `.pickle` 地图文件。
- 可选 TF：`map -> base_link`。如果定位节点没有发布，可用 `publish_localization_tf:=true` 由 `/localization` 桥接。

## 构建

```bash
cd /home/if/pct_dddmr_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只改了集成层：

```bash
colcon build --packages-select pct_dddmr_nav pct_dddmr_web indoor_fusion_bridge
source install/setup.bash
```

## 一体化启动

```bash
ros2 launch pct_dddmr_nav full_livox_fastlio_pct_nav.launch.py \
  map:=/home/if/fast_ws/test.pcd \
  tomogram_path:=/home/if/pct_dddmr_ws/test.pickle \
  use_mcl_feature:=true \
  obstacle_avoidance_enabled:=true
```

如果 Livox 或 FAST-LIO 已经单独启动，可以关闭对应模块：

```bash
ros2 launch pct_dddmr_nav full_livox_fastlio_pct_nav.launch.py \
  map:=/home/if/fast_ws/test.pcd \
  tomogram_path:=/home/if/pct_dddmr_ws/test.pickle \
  start_livox:=false
```

只启动 PCT 导航、Web 和路线追踪：

```bash
ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py \
  tomogram_path:=/home/if/pct_dddmr_ws/test.pickle \
  use_mcl_feature:=true \
  obstacle_avoidance_enabled:=true
```

网页默认地址：

```text
http://127.0.0.1:8000
```

## 常用 launch 参数

- `tomogram_path`：PCT `.pickle` 地图路径。
- `use_web`：是否启动网页端，默认 `true`。
- `use_route_tracker`：是否启动轻量路线追踪器，默认 `true`。
- `use_mcl_feature`：是否启动 MID360 局部障碍特征节点，默认 `false`。
- `obstacle_avoidance_enabled`：是否启用路线追踪器避障逻辑，默认 `false`。
- `obstacle_cloud_topic`：避障输入点云，默认 `/segmented_cloud_pure`。
- `publish_livox_tf`：是否发布 `base_link -> livox_frame` 静态 TF，默认 `true`。
- `publish_localization_tf`：是否把 `/localization` 桥接成 `map -> base_link` TF，默认 `false`。
- `local_lidar_topic`：`mcl_feature` 输入雷达话题，默认 `/livox/lidar`。

查看完整参数：

```bash
ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py --show-args
```

## Livox 输入

MID360 驱动保持原生 CustomMsg 输出：

```bash
ros2 launch pct_dddmr_nav livox_mid360_with_converter.launch.py
```

该 launch 只发布：

```text
/livox/lidar  livox_ros_driver2/msg/CustomMsg
```

当前系统不再发布 `/livox/lidar_points`，也不再安装 `livox_custom_to_pointcloud2` 转换节点。

## 网页端使用流程

1. 启动导航 launch。
2. 确认网页左上角显示“已定位，可以导航”。
3. 点击“点击规划”。
4. 第一次点击地图选择目标位置。
5. 第二次点击目标前方一点设置终点朝向。
6. Web 调用 PCT `get_dwa_plan` 生成全局路径。
7. 点击“开始导航”后，`route_tracker_node` 追踪路径并发布 `/cmd_vel`。

网页中的绿色路线来自 PCT 规划结果，路线追踪器不会持续重新规划全局路径。

## 多楼层 / 高度层筛选

多楼层地图投影到网页上时，不同高度会重叠。网页左侧提供高度层筛选：

- 勾选“只显示并选取当前高度层”。
- 手动设置 `Z 最小 / 最大`。
- 点击“跟随当前楼层”，按当前机器人 `z` 高度筛选。
- 点击“上一层 / 下一层”切换高度窗口。
- 点击“全部高度”恢复完整地图显示。

## PCT 规划参数

配置文件：

```text
src/pct_dddmr_nav/config/pct_dddmr_params.yaml
```

关键参数：

```yaml
use_quintic: true
safety_margin_cells: 15
```

- `use_quintic: true`：路径更圆滑，但在障碍物附近可能更贴边。
- `safety_margin_cells`：安全边界格数，值越大越保守，但窄通道更容易规划失败。

## 控制参数联动

网页端控制参数会同步给 `route_tracker_node`：

- `max_linear_x / min_linear_x`：线速度范围。
- `max_angular_z / min_angular_z`：角速度范围。
- `alpha`：速度平滑系数。
- `lookahead_distance`：路径追踪前视距离。
- `arrival_distance / arrival_angle_deg`：到点判定阈值。
- `path_yaw_kp / final_yaw_kp`：路径朝向和最终朝向控制增益。
- `rotate_in_place_angle_deg / rotate_exit_angle_deg`：原地转向进入和退出阈值。

最终速度由 `route_tracker_node` 发布到 `/cmd_vel`。

## 主要话题

输入：

- `/localization`：定位，`nav_msgs/Odometry`。
- `/livox/lidar`：MID360 原生 CustomMsg。
- `/segmented_cloud_pure`：`mcl_feature` 输出的局部障碍点云。

输出：

- `/global_path`：PCT 全局路径，网页显示使用。
- `/pct_path`：PCT 路径调试话题。
- `/cmd_vel`：路线追踪器输出速度。
- `/pct_dddmr_web/controller/state`：导航状态，网页显示使用。
- `/nav/done`：导航完成状态。

Action：

- `get_dwa_plan`：PCT 全局规划 action。

## 测试定位脚本

没有真实定位时，可以发布固定 `/localization` 和 `map -> base_link` TF：

```bash
ros2 run pct_dddmr_nav test_localization_publisher \
  --x 0 --y 0 --z 0 --yaw 0
```

模拟绕圈移动：

```bash
ros2 run pct_dddmr_nav test_localization_publisher \
  --x 0 --y 0 --z 0 --circle-radius 1.0
```

## 常见问题

### 网页提示“路线点数不足”

通常是没有收到 `/localization`，导致无法组成“当前位置 + 目标点”的导航请求。

```bash
ros2 topic echo /localization --once
```

### 没有速度输出

检查路线追踪器是否启动、是否收到路线和开始命令：

```bash
ros2 topic echo /pct_dddmr_web/controller/state
ros2 topic echo /cmd_vel
```

### 避障没有反应

确认启用了 `use_mcl_feature:=true obstacle_avoidance_enabled:=true`，并检查：

```bash
ros2 topic hz /segmented_cloud_pure
```

### 高楼层终点选不中

开启网页左侧“高度层筛选 / 选点辅助”，设置对应 `Z 最小 / 最大` 后再点目标。
