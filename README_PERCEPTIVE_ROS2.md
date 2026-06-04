# 感知 MPC ROS2 迁移说明

这份文档记录当前 workspace 中把 `legged_perceptive` 的核心感知 MPC 能力迁移到 ROS2 Humble `legged_control` 的改动。迁移目标是和原来的非感知 MPC 并存：默认 `LeggedController` 路径不改，新增 perceptive controller / perceptive interface 路径承载地形感知相关逻辑。

## 当前迁移范围

已经接入的能力：

- ROS2 版 convex plane decomposition 消息与节点。
- elevation map / planar terrain 到 MPC 的数据接收。
- 感知版 reference manager，根据平面地形更新 base 高度、pitch 和摆动腿 touchdown / liftoff 高度。
- 感知版 precomputation，缓存每只脚的落足凸多边形约束。
- 足端落足区域约束 `FootPlacementConstraint`。
- 足端 SDF 碰撞约束 `FootCollisionConstraint`。
- 小腿球体近似 SDF 碰撞约束 `SphereSdfConstraint`。

当前保持最小修改策略：

- 原始 `LeggedInterface` 和 `LeggedController` 仍保留。
- 感知逻辑放在 `perceptive/` 子目录。
- 原 MPC 的 task/reference/gait 配置不直接覆盖。
- 通过新增 controller 插件和配置来选择感知 MPC。

## 数据流

整体数据流如下：

```text
RealSense / depth cloud
  -> elevation_mapping
  -> grid_map elevation layer
  -> convex_plane_decomposition_ros
  -> /convex_plane_decomposition_ros/planar_terrain
  -> PlanarTerrainReceiver
  -> PerceptiveLeggedInterface shared terrain / SDF
  -> PerceptiveLeggedReferenceManager
  -> PerceptiveLeggedPrecomputation
  -> OCS2 MPC constraints
```

MPC 中使用的感知数据主要分两类：

- `convex_plane_decomposition::PlanarTerrain`：用于选择脚下支撑平面、构造落足凸区域约束、修正目标 base 姿态。
- `grid_map::SignedDistanceField`：用于足端和小腿球体与地形的 clearance 约束。

如果还没有收到实时地形，`PerceptiveLeggedInterface` 会初始化一个默认 5m x 5m 平地 map，保证感知 controller 可以先启动。

## 依赖

感知迁移涉及这些额外依赖：

- `grid_map`
- `grid_map_sdf`
- `grid_map_filters`
- `elevation_mapping`
- `convex_plane_decomposition`
- `convex_plane_decomposition_msgs`
- `convex_plane_decomposition_ros`
- `grid_map_filters_rsl`
- `ocs2_sphere_approximation`

其中 `grid_map_sdf`、`grid_map_filters` 可以通过 apt 安装：

```bash
sudo apt install ros-humble-grid-map-sdf ros-humble-grid-map-filters
```

`convex_plane_decomposition*` 和 `grid_map_filters_rsl` 当前在 workspace 的 `src/elevation` 下按 ROS2 ament 包构建。

`ocs2_sphere_approximation` 位于：

```text
/workspace/src/ocs2/ocs2_pinocchio/ocs2_sphere_approximation
```

它属于当前 workspace 的 `ocs2` 源码仓库。为了在 ROS2 Humble 下编译，当前做了 TinyXML 兼容修正：`urdf::exportURDF()` 返回 `TiXmlDocument*`，因此源码使用 `TiXmlDocument / TiXmlPrinter`，CMake 通过 `pkg-config` 查找 `tinyxml`。

## 主要改动文件

### elevation 相关

```text
src/elevation/grid_map_filters_rsl/
src/elevation/convex_plane_decomposition/
src/elevation/convex_plane_decomposition_msgs/
src/elevation/convex_plane_decomposition_ros/
```

作用：

- 将原 catkin 包迁移到 `ament_cmake`。
- 提供 ROS2 消息、节点、grid map 转换和 convex region 输出。
- 为 MPC 提供 planar terrain topic。

### legged_interface 感知路径

```text
legged_interface/include/legged_interface/perceptive/
legged_interface/src/perceptive/
legged_interface/CMakeLists.txt
legged_interface/package.xml
```

新增核心类：

```text
PerceptiveLeggedInterface
PerceptiveLeggedReferenceManager
PerceptiveLeggedPrecomputation
ConvexRegionSelector
FootPlacementConstraint
FootCollisionConstraint
SphereSdfConstraint
```

作用：

- 建立感知版 OCS2 problem。
- 接入 terrain / SDF 共享数据。
- 添加落足区域、足端地形碰撞、小腿球体地形碰撞约束。

### legged_controllers 感知路径

```text
legged_controllers/include/legged_controllers/perceptive/
legged_controllers/src/perceptive/
legged_controllers/config/perceptive/
legged_controllers/launch/load_perceptive_controller.launch
legged_controllers/legged_controllers_plugins.xml
legged_controllers/CMakeLists.txt
legged_controllers/package.xml
```

作用：

- 新增感知 controller 插件。
- 订阅 planar terrain topic。
- 将 terrain / SDF 同步模块注册到 MPC solver。

### OCS2 球体近似兼容

```text
src/ocs2/ocs2_pinocchio/ocs2_sphere_approximation/CMakeLists.txt
src/ocs2/ocs2_pinocchio/ocs2_sphere_approximation/src/PinocchioSphereInterface.cpp
```

作用：

- 让 `ocs2_sphere_approximation` 能在当前 ROS2 Humble + urdfdom 环境下编译。
- 支持 `SphereSdfConstraint` 对小腿 collision geometry 做球体近似。

## 构建顺序

推荐先构建 elevation 和 OCS2 依赖：

```bash
source /opt/ros/humble/setup.bash
cd /workspace

colcon build --packages-select \
  grid_map_filters_rsl \
  convex_plane_decomposition_msgs \
  convex_plane_decomposition \
  convex_plane_decomposition_ros \
  ocs2_sphere_approximation \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

然后构建感知 MPC 相关包：

```bash
source /workspace/install/setup.bash

colcon build --packages-select \
  legged_interface \
  legged_controllers \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

当前验证结果：

```text
ocs2_sphere_approximation: pass
legged_interface: pass
legged_controllers: pass
```

构建中可能看到 Boost placeholder 或 PCL policy warning，目前不影响编译。

## 运行检查

启动前先 source：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

检查感知 controller 插件是否安装：

```bash
ros2 pkg prefix legged_controllers
```

检查 planar terrain topic 是否存在：

```bash
ros2 topic list | grep planar_terrain
```

期望 topic：

```text
/convex_plane_decomposition_ros/planar_terrain
```

如果 topic 暂时没有发布，感知 MPC 会使用默认平地初始化，但这只能用于启动和基本编译验证，不能代表真实感知效果。



## 启动 P1 感知仿真

P1 感知仿真入口是：

```bash
ros2 launch legged_gazebo p1_perceptive_sim.launch.py
```

无 GUI 启动：

```bash
ros2 launch legged_gazebo p1_perceptive_sim.launch.py gui:=false
```

这个 launch 会：

- 启动 Gazebo Classic。
- 生成 P1 URDF。
- 加载 `joint_state_broadcaster`。
- 加载 `legged_perceptive_controller`。
- 默认同时启动 `convex_plane_decomposition_ros`。

如果只想先跑感知 MPC controller，不启动平面分割节点：

```bash
ros2 launch legged_gazebo p1_perceptive_sim.launch.py \
  start_convex_plane_decomposition:=false
```

启动后检查 controller：

```bash
ros2 control list_controllers
```

期望看到：

```text
joint_state_broadcaster active
legged_perceptive_controller active
```

检查感知 terrain 输入：

```bash
ros2 topic list | grep planar_terrain
```

期望看到：

```text
/convex_plane_decomposition_ros/planar_terrain
```

注意：`p1_perceptive_sim.launch.py` 只启动平面分割，不负责启动相机和 elevation mapping。真实感知效果还需要另外启动 depth/elevation map 链路，让 `/elevation_mapping/elevation_map_raw` 持续发布。

## 约束说明

### FootPlacementConstraint

作用：限制摆动脚 touchdown 位置落在选中的 convex planar region 内。

输入：

- `PerceptiveLeggedReferenceManager`
- `ConvexRegionSelector`
- end-effector kinematics

### FootCollisionConstraint

作用：限制摆动足端和地形 SDF 之间保持 clearance。

当前 clearance：

```text
0.03 m
```

### SphereSdfConstraint

作用：使用 `ocs2_sphere_approximation` 把小腿 collision geometry 近似成多个球，然后约束球面和地形 SDF 的距离。

当前 collision links：

```text
LF_calf
RF_calf
LH_calf
RH_calf
```

当前 max excess：

```text
0.02 m
```

当前 soft penalty：

```text
RelaxedBarrierPenalty::Config(1e-3, 1e-3)
```

注意：如果换机器人或 URDF link 名不同，需要同步修改 collision link 名称，否则 `PinocchioSphereInterface` 可能无法找到对应 collision geometry。

## 并存方式

当前并存策略是：

```text
原始 MPC:
  legged_interface/
  legged_controllers/src/LeggedController.cpp
  legged_controllers/config/<robot>/

感知 MPC:
  legged_interface/perceptive/
  legged_controllers/src/perceptive/
  legged_controllers/config/perceptive/
```

这样做的好处：

- 原始 controller 可以继续按原配置运行。
- 感知 controller 可以独立调试。
- 如果感知链路有问题，可以回退到原始 MPC，不需要撤销核心代码。

## 后续建议

还需要继续验证的点：

- elevation_mapping 的 frame、resolution、layer 名称是否和 `PlanarTerrainReceiver` 一致。
- `convex_plane_decomposition_ros` 发布频率是否适合 MPC。
- `odom` / `base` / camera frame 的 TF 是否完整。
- 感知 controller 的 launch 是否和现有 `p1_sim.launch.py` 完整串起来。
- Gazebo 中 RealSense depth topic 是否能正常进入 elevation map。
- `SphereSdfConstraint` 对求解速度和可行性的影响。

如果调试中发现 MPC 变慢或 infeasible，建议先临时关闭 `SphereSdfConstraint`，只保留 `FootPlacementConstraint` 和 `FootCollisionConstraint`，确认 terrain 输入稳定后再打开小腿球体 SDF 约束。
