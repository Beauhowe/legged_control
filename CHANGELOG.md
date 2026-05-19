# Changelog

本文件记录 `legged_control` 从 ROS1/catkin 迁移到 ROS2 Humble/ament 过程中的主要变更。

格式参考 [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)，版本号当前沿用包内 `0.0.0` 开发版本。日期使用 UTC。

## [Unreleased]

## [v0.0.1] - 2026-05-19 -lr-pro

### Added

- 新增 `legged_robots/lr_pro/lr_pro_description` ROS2 description 包，包含 lr_pro 的 mesh、模块化 xacro、ros2_control 接口和 Gazebo 启动入口。
- 新增 `lr_pro_description/launch/display.launch.py`，支持发布 `robot_description`、启动 Gazebo Classic 并 spawn `lr_pro`。
- 新增 `legged_robots/lr_pro/lr_pro_hw` ROS2 硬件接口包，包含 `lr_pro_hw/LrProHW` 插件导出、默认硬件配置和真机启动入口。
- 新增 `legged_controllers/config/lr_pro` 控制参数目录，供 lr_pro 仿真控制器加载 task/reference/gait 配置。
- 新增 `lr_pro_description/launch/lr_pro_sim.launch.py`，支持在 Gazebo Classic 中 spawn lr_pro 并自动加载 `joint_state_broadcaster` 与 `legged_controller`。

### Changed

- 将 lr_pro 机器人描述改为对标 A1 的顶层组装式 `robot.xacro`，拆分为 `const.xacro`、`leg.xacro`、`imu.xacro`、`ros2_control.xacro`、`gazebo.xacro` 和 `transmission.xacro`。

### Fixed

- 修复 lr_pro Gazebo 中 STL mesh 不显示的问题：将 mesh URI 从 `package://lr_pro_description/meshes` 调整为 Gazebo Classic 更稳定的 `file://$(find lr_pro_description)/meshes`。
- 为 lr_pro 仿真 ros2_control 关节状态添加初始角度，并调整默认站立关节角与 spawn 高度，避免膝关节默认 0 角度导致起立前倾。
- 为 lr_pro 仿真增加暂停启动流程：spawn 后通过 Gazebo `set_model_configuration` 设置初始站立关节角，controller 加载后再 unpause，避免模型从零关节角姿态自由下落。
- 修正 legged_controller 使用的关节顺序，使其跟随 OCS2 `modelSettings.jointNames`，避免 RF/LH 状态和命令顺序错位。
- `lr_pro_hw/LrProHW` 当前为硬件接入骨架：会导出关节、接触和 IMU 接口，并临时将 command 镜像到 state；真实机器人通信仍需接入 LR Pro 的 SDK 或底层协议。

## [v0.0.0] - 2026-05-13 -ros2-migration

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

### Notes

- 当前主要验证对象是 Unitree Go1 仿真。
- A1 和 Aliengo 的模型与参数仍保留，但需要按机器人逐个重新验证 ROS2 仿真和真机行为。
- 第一次启动控制器时，OCS2/CppAD 会在 `/tmp/legged_control/...` 生成动态库，配置阶段会比后续启动慢。
- 真机实时调度仍依赖系统权限配置；没有 `SCHED_FIFO` 权限时会有 warning，但不阻止仿真启动。
