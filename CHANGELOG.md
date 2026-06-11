# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-06-10

### Changed
- `LeggedController` 控制器激活后不再强制启动 MPC 优化，改为进入 **空闲态**：所有关节输出零指令（`kp=kd=ff=0`，完全被动），直到收到步态命令才启动优化。
- `starting()` 去掉激活时的阻塞式初始策略等待，仅设置时钟基准与初始观测，置 `mpcIdle_=true`、`mpcRunning_=false`。
- 新增 `startMpcOptimization()`：**非阻塞**放行后台 MPC 线程（不在实时 `update()` 中阻塞等待策略）；首份策略就绪前 `update()` 持续输出零指令，避免 `evaluatePolicy` 在无策略时崩溃。

### Added
- `setupMpc()` 新增订阅 `legged_robot_mpc_mode_schedule`（与 `GaitReceiver` 同话题），收到**任意**步态命令即置 `motionGaitRequested_`，触发 MPC 优化启动。
- `LeggedController.h` 新增成员 `mpcIdle_`、`motionGaitRequested_`、`motionGaitSubscriber_` 及方法 `startMpcOptimization()`。

### Notes / 迁移说明
- 激活后机器人完全被动（无支撑力矩），应在 **已趴卧或受机械结构支撑** 的状态下激活，否则站立/悬空姿态会瘫倒。
- 启动条件为"收到任意步态命令"，包含 `stance`、`lie_down` 等纯 STANCE 步态。
- 急停逻辑与安全检查不变。
- 仅编译验证（`colcon build --packages-select legged_controllers` 通过）；实机/Gazebo 行为（被动姿态安全性、步态启动平顺性）需现场确认。

## [0.2.0] - 2026-06-02

### Changed
- `p1_sim.launch.py` 与 `go1_sim.launch.py` 启动 Gazebo 前设置 `GAZEBO_MODEL_DATABASE_URI=`，避免离线或模型库访问慢时阻塞 world 加载。
- `empty_world.world` 移除 Gazebo ROS system plugin 声明，改为仅由 `gzserver.launch.py` 的 `init`、`factory`、`force_system` 参数加载，避免 Gazebo 报 `incorrect plugin type`。
### Fixed
- 修复 Gazebo Classic 首次启动访问在线模型库过慢时，`spawn_entity.py` 默认 30 秒等待 `/spawn_entity` 超时退出的问题；P1 与 Go1 仿真 spawn timeout 提高到 180 秒。

## [0.2.0] - 2026-06-01

### Added
- `LeggedController` 启动时打印 `mrtDesiredFrequency` 与对应 `MRT period`，便于核对 `task.info` 与 `controller_manager.update_rate`。
- 当实测控制周期与 `task.info` 中 `mrtDesiredFrequency` 偏差超过 5% 时，输出一次性 WARN，提示原地步态可能出现横向漂移。
- 仿真硬件 `LeggedHWSim` 支持 URDF/xacro 参数 `delay_cycles`（默认 9），命令延迟按 **控制周期数** 计算，随 `update_rate` 缩放。
- `ros2_control.xacro` 与 `p1_sim.launch.py` / `go1_sim.launch.py` 传入 `delay_cycles`；保留旧参数 `delay`（秒）并给出迁移 WARN。
### Changed
- `LeggedController` 中 `currentObservation_.time` 在控制器激活后改为基于 **墙钟** `(now - controllerStartTime_)`，减少仿真/调度抖动下时间轴与 MPC 步态相位错位。
- `LeggedHWSim` 命令缓冲延迟由固定 `0.009 s` 改为 `delay_cycles × period`，降低外环降频后延迟标定失准。
- `controllers.yaml`、`config/p1/task.info`、`legged_control/p1_task.info` 增加注释：`update_rate` 必须与 `mpc.mrtDesiredFrequency` 一致；`go1/task.info` 同步注释说明。
- `empty_world.world` 增加说明：`max_step_size × real_time_update_rate` 决定仿真实时因子，须与外环频率策略一致。

### Fixed
- 缓解因 **标称 MRT 频率与实际 `controller_manager` 周期不一致** 导致的原地 trot 缓慢横向漂移（需配置侧三处对齐，见下方说明）。

### Notes / 迁移说明
- **仿真稳定推荐**：`empty_world.world` 使用 `max_step_size=0.001`、`real_time_update_rate=1000`，且 `controllers.yaml` 的 `update_rate` 与 `task.info` 的 `mrtDesiredFrequency` 均为 **1000**。
- 若将外环改为 500 Hz，须同时将 world 改为 `max_step_size=0.002`、`real_time_update_rate=500`，**不可** 仅改 yaml/task 而 Gazebo 仍为 1000 Hz 物理步进。
- 真机下位机频率 与 `mrtDesiredFrequency` 和 `controllers.yaml` 的 `update_rate`一致。
- 测原地 trot 时，避免与 `legged_target_trajectories_publisher` 同时发布非零 `/cmd_vel`。

## [0.1.1] - 2026-05-28

### Added
- 在 `empty_world.world` 中添加 Gazebo ROS 插件：
  ```xml
  <plugin name="gazebo_ros_init" filename="libgazebo_ros_init.so"/>
  <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>
  <plugin name="gazebo_ros_force_system" filename="libgazebo_ros_force_system.so"/>
  ```

## [0.1.1] - 2026-05-27

### Added
- 新增 `p1_gait_dds_bridge` 节点，订阅 `P1_Gait::gait_command` 并按 `gait_id_mapping` 发布 `/cmd_vel` 与 `/legged_robot_mpc_mode_schedule`。
- 新增 P1 `lie_down` gait 和低高度 `reference_lie_down.info`，用于默认趴下启动和仿真测试趴下姿态。
- 扩展 `p1_gait_dds_bridge`，支持通过 DDS `gait_id` 上升沿发布站立/趴下 OCS2 target trajectory。
- 将 P1 gait bridge 拆为 ROS-only bridge 与 FastDDS-only worker 两个进程，避免 ROS DDS 与下位机 Fast DDS 版本冲突。
- 将 P1 主 DDS 通讯拆为 ROS 进程内 `P1DdsInterface` IPC 客户端与 FastDDS-only `p1_dds_worker`，避免 `ros2_control_node` 内加载下位机 Fast DDS。
- 新增控制器和 P1 硬件层急停订阅，急停时暂停 MPC 推进并清零所有关节命令。
- 新增 P1 真机 launch 内置 `legged_target_trajectories_publisher` 和 gait bridge 启动项，使实机 `/cmd_vel`、步态切换和站立/趴下 target 发布链路与仿真控制链路对齐。

### Changed
- P1 真机 launch 默认使用 `reference_lie_down.info` 启动，便于按“趴下 -> 站立 -> 步态运动”的实机流程控制。
- P1 仿真 launch 默认也切换到 `reference_lie_down.info`，并支持向 `legged_controller` 传入 `emergencyStopTopic`。
- P1 默认电流到关节力矩比例改为 `2.863 Nm/A`，对应当前关节输出端转矩常数。
- 扩展手柄控制脚本，支持急停组合键、站立/趴下姿态 target 发布，以及静止步态下抑制速度命令。

### Fixed
- 控制器急停释放后重置并重新同步 MPC 初始策略，避免恢复控制时继续使用急停前的过期策略。


## [0.1.0] - 2026-05-25

### Added

- 新增 `legged_p1_hw` 真机硬件接口包，通过 Fast DDS 对接 P1 下位机电机状态、IMU 状态和电机命令。
- 新增 P1 DDS 通讯封装 `P1DdsInterface`，将 DDS 回调缓存与 ros2_control `read()`/`write()` 周期解耦。
- 新增 P1 接触力估计器 `P1ContactEstimator`，支持由关节力矩和单腿雅可比估算足端 `Fz`。
- 新增 P1 真机启动文件 `legged_p1_hw/launch/p1.launch.py`，支持配置 DDS domain、话题名、关节映射、命令模式和接触估计参数。
- 新增 P1 硬件接口 README，说明 DDS IDL、数据流、Go1 对齐关系、力矩标定和接触估计方式。

### Changed

- 扩展 P1/通用 ros2_control xacro 参数，支持 `dds_joint_order`、`contact_estimation_method`、`contact_force_threshold`、`current_to_torque_scale` 和 `current_to_torque_offset`。
- P1 关节反馈将下位机电流通过可配置线性标定写入 `jointStates_[i].effort`，为后续真实关节力矩标定预留接口。
- P1 接触状态支持 `current` 阈值模式和 `jacobian` 足端力估计模式。

### Fixed

- 为 P1 DDS 关节映射增加最终重复索引检查，避免 `dds_joint_index_*` 覆盖后多个控制关节映射到同一个 DDS index。

## [0.0.1] - 2026-05-25

### Changed
- 将 legged_description 的 ros2_control xacro 宏与 hardware 名称从 Unitree 专用命名泛化为通用 `legged_ros2_control`/`legged_hw`。


## [0.0.1] - 2026-05-22

### Added

- 新增 P1 ROS2 仿真启动与模型资源，包含 p1 launch、控制配置、URDF/xacro 和 mesh 资源。

### Changed

- 对齐 P1 ROS2 描述与 ROS1 模型参数，包括机身/腿部偏移、膝关节 lateral offset 和默认 p1 URDF 生成结果。
- 调整 P1 mesh 内嵌材质，使 RViz 中机身、hip 和大腿显示为可见的白/深灰材质。

### Fixed

- 修复 P1 后腿 hip visual 未按前后腿方向翻转，导致 RViz 中后腿 hip 看起来缺失的问题。
- 修复 P1 大腿主 box collision 丢失 y 方向镜像偏移的问题，使左右腿碰撞体分别对齐到 `+0.13` 和 `-0.13`。
- 修复 P1 thigh mesh 镜像、材质和生成 URDF 中的若干 ROS1 到 ROS2 移植差异。


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
