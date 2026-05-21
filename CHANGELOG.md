# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.0] - 2026-05-21

### Added

- 新增 Gazebo Classic ROS2 启动文件 `legged_gazebo/launch/lr_pro_sim.launch.py`，用于生成 `lr_pro` Gazebo URDF、写入控制器参数、spawn 机器人并加载 `joint_state_broadcaster` 与 `legged_controller`。

### Changed

- 将 `lr_pro_description/robot.xacro` 从单体 URDF 重构为模块化 xacro 入口，复用通用 leg、IMU、ros2_control 和 Gazebo 宏生成机器人描述。
- 调整 `lr_pro` 默认控制参数：将 MPC 期望频率改为 `1000 Hz`，将 reference 默认平移速度改为 `0.0`，默认质心高度改为 `0.4`。
- 将 `lr_pro` URDF 关节 effort/velocity 限位对齐 ROS1 P1：HAA/HFE 为 `220/12`，KFE 为 `280/12`。

### Fixed

- 修复 `LeggedController` 读取 IMU 状态依赖 state interface 顺序的问题，改为按 `base_imu/...` 接口名称查找 orientation、angular velocity 和 linear acceleration。

### Removed

- 删除旧的 `legged_robots/lr_pro/lr_pro_description/urdf/lr_p1.urdf` 单体 URDF，`lr_pro` 描述改由模块化 xacro 生成。

### Notes

- 当前主要验证对象是 Unitree Go1 仿真。
- A1 和 Aliengo 的模型与参数仍保留，但需要按机器人逐个重新验证 ROS2 仿真和真机行为。
- `lr_pro`/P1 的上层控制顺序待统一到 OCS2 默认的 `LF, RF, LH, RH`：当前 OCS2 `ModelSettings` 默认 joint/contact 顺序为 `LF, RF, LH, RH`，而 `LeggedController`、`ros2_control.xacro` 和 `task.info` 的部分状态/接口顺序仍按 `LF, LH, RF, RH` 编排，后续应统一并把真实硬件电机顺序留在 hardware interface 内部映射。
- 第一次启动控制器时，OCS2/CppAD 会在 `/tmp/legged_control/...` 生成动态库，配置阶段会比后续启动慢。
- 真机实时调度仍依赖系统权限配置；没有 `SCHED_FIFO` 权限时会有 warning，但不阻止仿真启动。

## [0.0.0] - 2026-05-13

### Added

- 新增 ROS2 Humble 使用说明文档 `README_ROS2.md`，覆盖构建、仿真启动、手柄控制、步态切换、节点职责和控制计算链路。
- 新增 Gazebo Classic ROS2 启动文件 `legged_gazebo/launch/go1_sim.launch.py`，用于自动启动 Gazebo、发布 `robot_description`、spawn Go1，并加载 `joint_state_broadcaster` 与 `legged_controller`。
- 新增 ROS2 手柄控制节点 `legged_controllers/scripts/joy_control.py`。
  - 支持从 `sensor_msgs/msg/Joy` 发布 `/cmd_vel`。
  - 支持通过组合按键发布 OCS2 `ModeSchedule` 以切换 gait。
  - 支持仿真免 deadman 配置和真机 deadman 安全配置。
- 新增手柄相关配置：
  - `legged_controllers/config/joy_sim.yaml`
  - `legged_controllers/config/joy_gait_mappings.yaml`
  - `legged_controllers/launch/joy_control.launch.py`
- 新增 ROS2 `gazebo_ros2_control` 仿真硬件插件 `legged_gazebo/LeggedHWSim`。
- 新增 ROS2 Unitree 硬件接口插件导出，支持 `legged_unitree_hw/UnitreeHW_3_8_0`。
- 新增 `qpoases_vendor` vendor package，用 ROS2 风格替代原 ROS1 命名的 `qpoases_catkin`。
- 新增 `legged_robots/` 机器人资源目录，用于放置 Unitree 描述、硬件接口以及后续机器人平台资源。

### Changed

- 将核心包从 ROS1/catkin 迁移到 ROS2 Humble/ament：
  - `legged_common`
  - `legged_control`
  - `legged_controllers`
  - `legged_estimation`
  - `legged_gazebo`
  - `legged_hw`
  - `legged_interface`
  - `legged_wbc`
- 将控制器迁移到 `controller_interface::ControllerInterface` 生命周期模型。
- 将硬件抽象迁移到 `hardware_interface::SystemInterface`。
- 将 ROS1 自定义 hybrid joint/contact 接口迁移为 ros2_control state/command interface 名称约定。
- 将 ROS1 topic、publisher、subscriber、parameter 和 launch 逻辑迁移到 ROS2 `rclcpp`、Python launch 和 ROS2 参数机制。
- 将 OCS2 ROS 接口适配到 ROS2 版本消息和 reference manager。
- 将 `controllers.yaml` 迁移为 ROS2 controller manager 参数格式。
- 将 Unitree 相关包从 `legged_examples/legged_unitree/` 重组到 `legged_robots/legged_unitree/`。
- 将 `qpoases_catkin` 重命名为 `qpoases_vendor`，并同步更新 `legged_wbc` 的依赖、CMake target 和文档引用。
- 将 Go1 仿真中的 `controller_manager.update_rate` 调整为 `1000`，更接近 ROS1/Gazebo 控制循环行为。
- 将 ros2_control contact sensor 顺序调整为 `LF_FOOT, LH_FOOT, RF_FOOT, RH_FOOT`，与模型/状态估计约定保持一致。
- 调整 Gazebo 仿真 hardware plugin，使其直接发布 `/ground_truth/state`，替代 ROS1 `gazebo_ros_p3d` 插件。

### Fixed

- 修复 Gazebo 启动时 `/spawn_entity` 服务不可用的问题：启动 `gzserver` 时显式启用 `gazebo_ros_factory`。
- 修复 `controller_manager/list_controllers` 长时间不可用的问题：通过 `gazebo_ros2_control` 插件随机器人实体加载 controller manager。
- 修复 `gazebo_ros2_control` 加载后崩溃的问题：完善 ros2_control URDF、controller 参数文件和仿真硬件插件导出。
- 修复 Go1 仿真高速平移更容易摔倒的高风险迁移差异：
  - 恢复 contact 读取的周期时间戳过滤，避免使用非当前控制周期的 Gazebo contact 缓存。
  - 修正 contact state interface 顺序。
  - 将控制器更新频率恢复为 1000 Hz。
- 修复手柄节点退出时可能出现 `ExternalShutdownException` traceback 的问题。
- 修复 ROS2 手柄配置中 topic 名称、deadman、仿真配置和组合键步态发布的若干兼容性问题。

### Removed

- 删除旧 ROS1 control loop 文件：
  - `legged_hw/include/legged_hw/LeggedHWLoop.h`
  - `legged_hw/src/LeggedHWLoop.cpp`
- 删除旧 ROS1 Unitree 入口实现：
  - `legged_unitree_hw/src/legged_unitree_hw.cpp`
- 旧的 `legged_examples/legged_unitree/` ROS1 目录不再作为 ROS2 构建入口，内容已迁移到 `legged_robots/legged_unitree/`。
