# pct_tomogram_tools

`pct_tomogram_tools` 是把 PCT 原来的点云地图转换功能封装成的 ROS2 命令行工具。

它的作用是把 `.pcd` 点云地图转换成 PCT 全局路径规划需要的 `.pickle` 地图。

## 功能

输入：

```text
map.pcd
```

输出：

```text
map.pickle
```

生成的 `.pickle` 里包含：

```text
data        PCT tomogram 数据，形状为 [5, 层数, x尺寸, y尺寸]
resolution  地图 xy 栅格分辨率，单位 m
center      地图 xy 中心点
slice_h0    第一层高度参考
slice_dh    每层高度间隔
```

这个 `.pickle` 可以直接给 `pct_dddmr_nav` 做全局路径规划。

## 依赖

地图转换使用的是 PCT 原始的 CUDA/CuPy tomography 算法，所以“转换地图”的电脑需要：

```text
CUDA
cupy
open3d
ROS2 Python 环境
```

注意：只是用已经转换好的 `.pickle` 进行导航规划，不需要 GPU。

也就是说：

```text
PCD -> pickle 转换：需要 GPU/CUDA
pickle -> 全局路径规划：不需要 GPU
```

## 编译

在工作空间中执行：

```bash
cd /home/if/pct_dddmr_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select pct_tomogram_tools
source install/setup.bash
```

如果你已经构建过整个工作空间，只需要：

```bash
source /home/if/pct_dddmr_ws/install/setup.bash
```

## 基本用法

```bash
ros2 run pct_tomogram_tools pcd_to_tomogram \
  --pcd /path/to/map.pcd \
  --output /path/to/map.pickle
```

例如：

```bash
ros2 run pct_tomogram_tools pcd_to_tomogram \
  --pcd /home/if/indoor_fusion_ws/maps/2026_05_07_15_58_12/map.pcd \
  --output /home/if/indoor_fusion_ws/maps/2026_05_07_15_58_12/map.pickle \
  --resolution 0.10 \
  --ground-h 0.0 \
  --slice-dh 0.5
```

转换完成后，会输出类似：

```text
Tomogram saved: /path/to/map.pickle
shape=(5, 9, 474, 536)
resolution=0.3
center=[...]
slice_h0=0.5
slice_dh=0.5
```

## 参数说明

基础参数：

```text
--pcd              输入的 .pcd 点云地图路径
--output           输出的 .pickle 地图路径
--resolution       xy 平面栅格分辨率，单位 m
--ground-h         地面基准高度，一般设为 0.0
--slice-dh         高度分层间隔，一般设为 0.5
```

可通行性参数：

```text
--kernel-size       局部地形分析窗口大小
--interval-min      最小可通行净空，小于该值会被认为不可通行
--interval-free     完全自由通行的净空阈值
--slope-max         最大可通行坡度阈值
--step-max          最大可跨越台阶高度
--standable-ratio   周围可站立栅格比例阈值
--cost-barrier      障碍/不可通行区域代价
--safe-margin       障碍安全边界
--inflation         障碍膨胀半径
```

调试/测速参数：

```text
--repeat            重复转换次数，用于统计平均耗时；正常使用设为 1
```

## 推荐参数

室内、楼梯、坡道环境可以先用这一组：

```bash
--resolution 0.10 \
--ground-h 0.0 \
--slice-dh 0.5 \
--kernel-size 7 \
--interval-min 0.50 \
--interval-free 0.65 \
--slope-max 0.40 \
--step-max 0.17 \
--standable-ratio 0.20 \
--cost-barrier 50.0 \
--safe-margin 0.4 \
--inflation 0.2
```

如果地图很大，可以把分辨率调粗一点，例如：

```bash
--resolution 0.20
```

或者：

```bash
--resolution 0.30
```

分辨率越小，地图越精细，但显存和计算量越大。

分辨率越大，转换更快、占用更小，但路径会更粗。

## 导航中使用转换结果

生成 `.pickle` 后，用它启动 PCT + DDDMR 导航：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/if/pct_dddmr_ws/install/setup.bash

ros2 launch pct_dddmr_nav pct_dddmr_nav.launch.py \
  tomogram_path:=/path/to/map.pickle
```

导航时需要外部已经有：

```text
/localization    nav_msgs/Odometry
/livox/lidar     MID360 雷达话题
map -> base_link TF
```

## 常见问题

### 1. 转换时报 CuPy/CUDA 错误

说明 GPU 或 CUDA 环境有问题。检查：

```bash
python3 -c "import cupy; print(cupy.cuda.runtime.getDeviceCount())"
```

如果这里报错，需要重新安装和 CUDA 版本匹配的 `cupy`。

### 2. open3d 找不到

安装 `open3d`：

```bash
pip3 install open3d
```

### 3. 显存不够

可以调大分辨率：

```bash
--resolution 0.20
```

或者：

```bash
--resolution 0.30
```

也可以减小地图范围，只保留需要导航的区域。

### 4. 生成的路径贴墙或离障碍太近

增大：

```bash
--safe-margin
--inflation
```

例如：

```bash
--safe-margin 0.5 --inflation 0.3
```

### 5. 楼梯或台阶过不去

适当调大：

```bash
--step-max
```

例如：

```bash
--step-max 0.25
```

但这个值不能无限增大，否则可能把危险台阶也当成可通行。

## 注意

- 这个工具不会修改 `/home/if/下载/PCT_planner-main` 里的原始 PCT 文件。
- 输出的 `.pickle` 可以直接用于 `pct_dddmr_nav`。
- 地图转换需要 GPU，后续导航规划不需要 GPU。
- 如果换电脑部署，只需要把 `pct_dddmr_ws` 和生成好的 `.pickle` 带过去即可。
