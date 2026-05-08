# pct_dddmr_nav

`pct_dddmr_nav` 是本仓库的主集成包，负责启动 PCT 全局规划、DDDMR 局部避障/路径追踪、Web 控制台和桥接节点。

当前设计：

- 全局路径规划：PCT `.pickle` tomogram 地图。
- 局部避障与最终 `/cmd_vel`：DDDMR `p2p_move_base` + `local_planner`。
- 网页显示：直接加载 PCT 处理后的 `.pickle` 地图，显示地面、膨胀层、障碍物。
- 定位和雷达驱动：外部提供，本工作空间默认不包含 FAST-LIO 定位和 MID360 驱动。

## 输入要求

运行导航前，外部系统需要提供：

- `/localization`：`nav_msgs/Odometry`，机器人当前位姿。
- TF：`map -> base_link`，通常由定位系统发布。
- `/livox/lidar`：MID360 点云，用于 DDDMR 局部避障。
- PCT 转换后的 `.pickle` 地图文件。

## 构建

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只改了本包：

```bash
colcon build --packages-select pct_dddmr_nav
source install/setup.bash
```

## 启动导航

使用自己的 `.pickle` 地图：

```bash
cd <workspace>
source install/setup.bash

ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py \
  tomogram_path:=/path/to/your_map.pickle
```

如果使用包内示例地图：

```bash
ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py
```

常用 launch 参数：

```bash
ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py --show-args
```

重要参数：

- `tomogram_path`：PCT `.pickle` 地图路径。
- `use_web`：是否启动网页端，默认 `true`。
- `use_mcl_feature`：是否启动 DDDMR 的 MID360 局部避障输入，默认 `true`。
- `publish_livox_tf`：是否发布 `base_link -> livox_frame` 静态 TF，默认 `true`。

网页默认地址：

```text
http://127.0.0.1:8000
```

## 测试定位脚本

没有真实定位时，可以发布一个假的 `/localization` 和 `map -> base_link` TF，用来测试网页显示和 PCT 全局路径规划。

固定机器人位置：

```bash
ros2 run pct_dddmr_nav test_localization_publisher \
  --x 0 --y 0 --z 0 --yaw 0
```

模拟机器人绕圈移动：

```bash
ros2 run pct_dddmr_nav test_localization_publisher \
  --x 0 --y 0 --z 0 --circle-radius 1.0
```

注意：如果机器人在网页上一直动，通常是因为测试脚本带了 `--circle-radius`。停止该脚本，重新用固定位置命令启动即可。

检查 `/localization`：

```bash
ros2 topic echo /localization --once
ros2 topic info /localization -v
```

## 网页端使用流程

1. 启动导航 launch。
2. 确认网页左上角显示“已定位，可以导航”。
3. 在网页中点击“点击规划”。
4. 第一次点击地图选择目标位置。
5. 第二次点击目标前方一点，用来设置终点朝向。
6. 后端会把“当前位置 + 目标点”同步给桥接节点，DDDMR 会调用 PCT 进行真正的全局路径规划。

网页中显示的绿色路线来自 `/global_path`，也就是 PCT 给 DDDMR 的全局路径。

## 多楼层 / 高度层筛选

多楼层点云投影到网页上时，低楼层和高楼层会重叠，容易点到错误高度。网页左侧提供“高度层筛选 / 选点辅助”：

- 勾选“只显示并选取当前高度层”。
- 手动设置 `Z 最小 / 最大`。
- 点击“跟随当前楼层”，按当前机器人 `z` 高度筛选。
- 点击“上一层 / 下一层”切换高度窗口。
- 点击“全部高度”恢复完整地图显示。

开启高度层筛选后，网页选终点会优先在当前高度层内拾取点云落点；如果附近没有点云，会落到当前高度层中间高度。

## PCT 规划参数

当前 PCT 参数已经恢复为原始风格：

```yaml
use_quintic: true
safety_margin_cells: 15
```

配置文件位置：

```text
src/pct_dddmr_nav/config/pct_dddmr_params.yaml
```

说明：

- `use_quintic: true`：路径会更圆滑，但在障碍物附近可能更贴边。
- `safety_margin_cells`：安全边界格数，值越大越保守，但窄通道更可能规划失败。

如果后续需要更安全的路径，建议不要做简单的“逐点吸附到可行驶点”，那会导致锯齿和竖线。更合理的方向是：基于 PCT 栅格路径做折线简化，并对每条线段做碰撞检查。

## DDDMR 控制参数联动

网页端控制参数会同步到 DDDMR：

- `heading_slow_angle_deg`：角度偏差大时，先原地调整朝向。
- `max_linear_x / min_linear_x`：线速度范围。
- `max_angular_z / min_angular_z`：角速度范围。
- `alpha`：慢加速/加速度限制。
- `lookahead_distance`：局部路径跟踪前视距离。
- `arrival_distance / arrival_angle_deg`：到点判定阈值。
- `path_yaw_kp`：路径朝向跟踪权重。

最终速度仍由 DDDMR 局部规划器发布，网页端不直接控制导航过程中的 `/cmd_vel`。

## 主要话题

输入：

- `/localization`：定位，`nav_msgs/Odometry`。
- `/livox/lidar`：MID360 雷达点云。
- `/mapground`：处理后地面点云。
- `/mapcloud`：处理后障碍物点云。

输出：

- `/global_path`：PCT 全局路径，网页显示使用它。
- `/pct_path`：PCT 路径调试话题。
- `/cmd_vel`：DDDMR 输出速度。
- `/pct_dddmr_web/controller/state`：导航状态给网页端。

Action：

- `/p2p_move_base`：DDDMR 点到点导航 action。
- `get_dwa_plan`：PCT 接管的 DDDMR 全局规划 action。

注意：PCT server 已经占用 DDDMR 的 `get_dwa_plan` 全局规划 action，不要同时启动 DDDMR 原始 `global_planner_node`。

## 地图转换

本导航工作空间只使用已经转换好的 `.pickle` 地图。建图和地图转换可以在外部单独完成。

## 常见问题

### 网页提示“路线点数不足”

通常是没有收到 `/localization`，导致无法组成“当前位置 + 目标点”的导航请求。

检查：

```bash
ros2 topic echo /localization --once
```

### 网页中机器人一直移动

检查是否运行了带 `--circle-radius` 的测试定位脚本：

```bash
ps -ef | grep test_localization_publisher
```

固定位置测试请使用：

```bash
ros2 run pct_dddmr_nav test_localization_publisher --x 0 --y 0 --z 0 --yaw 0
```

### 高楼层终点选不中

开启网页左侧“高度层筛选 / 选点辅助”，设置对应 `Z 最小 / 最大` 后再点目标。

### 路径太贴障碍物

可以适当增大：

```yaml
safety_margin_cells: 20
```

但不建议直接使用逐点吸附后处理，容易导致路径锯齿和异常竖线。
