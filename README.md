# PCT-DDDMR Indoor Navigation

一个面向室内移动机器人的 ROS 2 导航工作空间，将 **PCT 点云地图全局规划**、**DDDMR 局部避障/路径追踪** 和 **Web 可视化控制台** 集成到一起。

本项目适合这样的使用场景：

- 已经有 FAST-LIO、LIO-SAM 或其他定位系统，可以发布机器人在地图中的位姿。
- 已经有 MID360 / Livox 等 3D 激光雷达驱动。
- 希望使用点云地图或 PCT `.pickle` tomogram 地图进行全局路径规划。
- 希望保留 DDDMR 的局部避障和路径跟踪能力。
- 希望通过浏览器显示地图、设置目标点、调整导航参数。

## 功能特性

- **PCT 全局路径规划**：使用 PCT `.pickle` tomogram 地图生成全局路径。
- **DDDMR 局部避障**：使用 DDDMR `p2p_move_base`、`local_planner`、`perception_3d` 完成局部避障和速度输出。
- **Web 控制台**：显示处理后的地图、机器人位置、全局路径，并支持点击目标点导航。
- **多楼层选点辅助**：支持按 Z 高度筛选点云，方便在多楼层/楼梯/坡道地图中选择目标点。
- **参数联动**：Web 端速度、角速度、慢加速、到点阈值、角度偏差阈值等参数会同步到 DDDMR。
- **地图转换工具**：提供 `pct_tomogram_tools`，可把 `.pcd` 点云地图转换为 PCT 需要的 `.pickle`。
- **测试定位工具**：提供假的 `/localization` 和 `map -> base_link` TF 发布器，用于无真实定位时测试 Web 与规划链路。

## 系统架构

```text
             converted PCT .pickle map
                        |
                        v
              +--------------------+
              | pct_get_plan_server|
              |  PCT global planner|
              +---------+----------+
                        |
               get_dwa_plan action
                        |
+-------------+         v          +---------------------+
| localization| --> DDDMR p2p ----> | DDDMR local planner |
| /localization|    move_base       | obstacle avoidance  |
+-------------+         |          +----------+----------+
                        |                     |
                        v                     v
                  /global_path             /cmd_vel
                        |
                        v
              +--------------------+
              | Web UI + bridge    |
              | map / goal / params|
              +--------------------+
```

本项目默认 **不包含**：

- 定位系统，例如 FAST-LIO / FAST-LIO-Localization。
- Livox/MID360 雷达驱动。
- 建图流程。

这些系统通常在部署机器上单独运行，本项目只消费它们发布的话题。

## 环境要求

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `colcon`
- PCL / Eigen / TF2 等 ROS 常见依赖

运行导航需要外部提供：

```text
/localization    nav_msgs/Odometry
/livox/lidar     MID360/Livox 点云
map -> base_link TF
```

地图转换额外需要：

```text
CUDA
CuPy
Open3D
```

注意：**PCD 转 `.pickle` 地图转换需要 GPU/CUDA；使用已经转换好的 `.pickle` 导航规划不需要 GPU。**

## 仓库结构

```text
src/
  pct_dddmr_nav/        PCT + DDDMR 导航集成包和主 launch
  pct_tomogram_tools/   PCD -> PCT tomogram pickle 地图转换工具
  indoor_route_nav/     Web 控制台和 ROS Web 桥
  indoor_fusion_bridge/ Web 控制台到 DDDMR action 的桥接节点
  p2p_move_base/        DDDMR 点到点导航
  local_planner/        DDDMR 局部规划器
  perception_3d/        DDDMR 3D 感知/避障
  mpc_critics/          DDDMR 轨迹打分插件
  trajectory_generators/DDDMR 轨迹生成插件
```

详细导航包说明见：

```text
src/pct_dddmr_nav/README.md
```

地图转换工具说明见：

```text
src/pct_tomogram_tools/README.md
```

## 快速开始

### 1. 构建

```bash
mkdir -p ~/pct_dddmr_ws
cd ~/pct_dddmr_ws

# 如果你是从 GitHub clone，一般目录已经包含 src/
# git clone <your-repo-url> .

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. 准备地图

如果已经有转换好的 `.pickle`：

```text
/path/to/your_map.pickle
```

可以直接用于导航。

如果只有 `.pcd` 点云地图，需要先转换：

```bash
ros2 run pct_tomogram_tools pcd_to_tomogram \
  --pcd /path/to/map.pcd \
  --output /path/to/map.pickle
```

更多转换参数见 `src/pct_tomogram_tools/README.md`。

### 3. 启动定位和雷达

在另一个终端启动你的定位系统和 MID360 驱动，确保存在：

```bash
ros2 topic echo /localization --once
ros2 topic echo /livox/lidar --once
ros2 run tf2_ros tf2_echo map base_link
```

### 4. 启动导航

```bash
cd ~/pct_dddmr_ws
source install/setup.bash

ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py \
  tomogram_path:=/path/to/your_map.pickle
```

网页默认地址：

```text
http://127.0.0.1:8000
```

## 无真实定位时测试

发布固定测试位姿：

```bash
ros2 run pct_dddmr_nav test_localization_publisher \
  --x 0 --y 0 --z 0 --yaw 0
```

发布绕圈测试位姿：

```bash
ros2 run pct_dddmr_nav test_localization_publisher \
  --x 0 --y 0 --z 0 --circle-radius 1.0
```

如果 Web 中机器人一直移动，检查是否仍在运行带 `--circle-radius` 的测试脚本。

## Web 使用流程

1. 打开 `http://127.0.0.1:8000`。
2. 确认状态显示"已定位，可以导航"。
3. 点击"点击规划"。
4. 第一次点击地图选择目标位置。
5. 第二次点击目标前方一点设置终点朝向。
6. Web 会发送目标，DDDMR 调用 PCT 全局规划并开始导航。

多楼层地图建议先开启"高度层筛选 / 选点辅助"，避免点到其他楼层。

## 重要参数

PCT 参数文件：

```text
src/pct_dddmr_nav/config/pct_dddmr_params.yaml
```

当前默认：

```yaml
use_quintic: true
safety_margin_cells: 15
```

含义：

- `use_quintic: true`：路径更圆滑，但可能更贴近障碍物。
- `safety_margin_cells`：PCT 全局规划安全边界，越大越保守。

DDDMR 局部规划参数：

```text
src/pct_dddmr_nav/config/dddmr_local_params.yaml
```

Web 参数会动态同步到 DDDMR，包括速度范围、慢加速、到点阈值、角度偏差阈值等。

## 常见问题

### Web 提示"路线点数不足"

通常是没有收到 `/localization`。检查：

```bash
ros2 topic echo /localization --once
```

### 目标点选不到高楼层

开启 Web 左侧"高度层筛选 / 选点辅助"，设置 `Z 最小 / 最大` 后再点目标。

### 路径太贴障碍物

可以尝试增大：

```yaml
safety_margin_cells: 20
```

但值太大会导致窄通道规划失败。

### 使用 `.pickle` 规划需要 GPU 吗？

不需要。GPU 只用于从 PCD 转换 `.pickle` 的地图预处理阶段。

## GitHub 上传注意事项

建议不要上传：

- `build/`
- `install/`
- `log/`
- 大型 `.pcd`
- 大型 `.pickle`
- TensorRT `.engine`
- 本地测试地图

当前 `.gitignore` 已经忽略大部分构建产物和地图文件。

如果你希望开箱即用地包含 PCT 预编译库，需要确认第三方许可和文件体积；否则建议在 README 中说明需要用户自行准备或构建 PCT runtime。

## 第三方代码与许可

本项目集成/改造了多个开源组件，包括 DDDMR、PCT planner、Indoor Route Nav 等。请在发布前确认各子模块和 vendored 第三方库的 LICENSE / NOTICE。

PCT vendored 目录中包含：

```text
src/pct_dddmr_nav/vendor/pct_planner/LICENSE
src/pct_dddmr_nav/vendor/pct_planner/NOTICE
```

## 项目状态

这是一个研究/工程集成型项目，已用于 PCT 点云地图、MID360 局部避障和 Web 目标点导航的联调。不同机器人底盘、雷达安装外参、地图质量、定位稳定性都会影响最终导航效果，部署前请在低速和安全环境中充分测试。
