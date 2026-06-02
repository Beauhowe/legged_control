# legged_control ROS2 Humble 使用说明

`legged_control` 是一个基于 OCS2、ros2_control 和 Gazebo Classic 的四足机器人控制框架。当前工程已经从 ROS1/catkin 迁移到 ROS2 Humble/ament，保留原有 NMPC + WBC + 状态估计的控制结构，并增加 ROS2 风格的仿真启动、手柄速度控制和组合键步态切换。

当前主要验证对象是 Unitree Go1。A1、Aliengo 的配置仍保留在 `legged_controllers/config` 和 `legged_unitree_description` 中，建议逐个重新验证。

## 主要特性

- ROS2 Humble + ament_cmake 构建。
- ros2_control 控制器插件：`legged/LeggedController`。
- Gazebo Classic 仿真硬件插件：`legged_gazebo/LeggedHWSim`。
- Unitree 硬件接口插件：`legged_unitree_hw/UnitreeHW_3_8_0`。
- OCS2 NMPC 生成最优状态、接触力和关节速度参考。
- WBC 将 NMPC 输出转换为关节力矩和低增益 PD 前馈命令。
- 支持 `/cmd_vel` 和 `/move_base_simple/goal` 输入。
- 支持手柄控制速度，并用组合键切换 gait。
- 支持仿真免 deadman 配置，也支持真机 deadman 安全配置。

## 目录结构

```text
legged_common/                 通用硬件接口名和数据结构
legged_hw/                     ros2_control SystemInterface 基类
legged_gazebo/                 Gazebo ros2_control 仿真硬件插件和仿真 launch
legged_controllers/            legged_controller、目标轨迹发布器、手柄控制
legged_estimation/             状态估计，Kalman filter / topic estimator
legged_interface/              OCS2 legged robot 问题定义、约束、代价
legged_wbc/                    Whole-body controller
legged_robots/legged_unitree/  Unitree 描述与硬件接口
qpoases_vendor/                qpOASES vendor package
```

## 获取依赖源码

本仓库只包含 `legged_control` 本体。当前工程还依赖同级 `src` 目录下的 `pinocchio`、`ocs2`、`fastdds_bridge`、`hpp-fcl` 和 `ocs2_robotic_assets`。其中 `pinocchio`、`ocs2`、`fastdds_bridge` 使用了 P1 部署相关修改，依赖版本由仓库根目录的 `dependencies.repos` 固定。

新机器部署时推荐从工作区 `src` 目录执行：

```bash
mkdir -p ~/p1_ws/src
cd ~/p1_ws/src

git clone -b dev git@github.com:AetherControl/legged_control.git
./legged_control/scripts/import_dependencies.sh
```

脚本会执行：

```bash
vcs import ~/p1_ws/src < legged_control/dependencies.repos
git -C ~/p1_ws/src/pinocchio submodule update --init --recursive
```

如果系统没有 `vcs` 命令，先安装：

```bash
sudo apt install vcstool
```

P1 DDS 相关包还需要 Fast DDS 本地安装路径。默认查找：

```text
~/p1_ws/src/env/fast_dds/local
```

如果 Fast DDS 安装在其他位置，构建前设置：

```bash
export FASTDDS_INSTALL_PREFIX=/path/to/fast_dds/local
```

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/p1_ws

colcon build --base-paths src \
  --packages-up-to legged_gazebo legged_unitree_hw legged_controllers \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_STANDARD=17

source install/setup.bash
```

只改手柄或控制器相关代码时：

```bash
colcon build --base-paths src --packages-select legged_controllers \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## 快速启动仿真

启动 Go1 Gazebo 仿真：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

ros2 launch legged_gazebo go1_sim.launch.py
```

无 GUI 启动：

```bash
ros2 launch legged_gazebo go1_sim.launch.py gui:=false
```

该 launch 会自动完成：

1. 启动 `gzserver` / `gzclient`。
2. 发布 `robot_description`。
3. 通过 `/spawn_entity` 将机器人 spawn 到 Gazebo。
4. 加载 `gazebo_ros2_control`。
5. 加载并激活 `joint_state_broadcaster` 和 `legged_controller`。

检查控制器状态：

```bash
ros2 control list_controllers
```

正常状态应包含：

```text
joint_state_broadcaster active
legged_controller active
```

第一次启动时 OCS2/CppAD 会在 `src/legged_control/legged_control/{robot_type}/` 生成动态库，配置控制器会较慢。后续启动会复用生成结果。若仓库不在 `/workspace/src/legged_control`，可设置环境变量 `LEGGED_CONTROL_REPO` 指向仓库根目录。

## 手柄控制
```bash
ros2 launch legged_controllers joy_control.launch.py robot_type:=go1 joy_device_id:=0
```

手柄控制由 `legged_controllers/scripts/joy_control.py` 实现，启动文件是：

```bash
ros2 launch legged_controllers joy_control.launch.py
```

这个 launch 会启动：

- `joy_node`
- `legged_target_trajectories_publisher`
- `joy_control`

仿真推荐使用免 deadman 配置：

```bash
ros2 launch legged_controllers joy_control.launch.py \
  teleop_config:=/workspace/install/legged_controllers/share/legged_controllers/config/joy_sim.yaml
```

真机或需要安全 deadman 时使用默认配置：

```bash
ros2 launch legged_controllers joy_control.launch.py
```

如果 `/joy` 已经由 Unitree 硬件接口发布，不需要再启动 USB/SDL `joy_node`：

```bash
ros2 launch legged_controllers joy_control.launch.py \
  start_joy_node:=false \
  joy_topic:=/joy
```

如果使用 USB/蓝牙手柄，ROS2 `joy_node` 使用 SDL 设备序号或设备名，不使用 `/dev/input/js0`：

```bash
ros2 launch legged_controllers joy_control.launch.py joy_device_id:=0
```

### 手柄速度映射

配置文件：

- 真机默认：`legged_controllers/config/joy.yaml`
- 仿真默认：`legged_controllers/config/joy_sim.yaml`

默认轴映射：

```yaml
axis 1 -> linear.x
axis 0 -> linear.y
axis 3 -> angular.z
```

真机默认 deadman：

```yaml
deadman_buttons: [4]
```

仿真免 deadman：

```yaml
deadman_buttons: []
```

如果手柄轴编号不同，可以用：

```bash
ros2 topic echo /joy
```

查看 `axes[]` 和 `buttons[]` 后修改 `joy.yaml` 或 `joy_sim.yaml`。

### 步态组合键

配置文件：

```text
legged_controllers/config/joy_gait_mappings.yaml
```

默认组合：

```yaml
mappings:
  - gait: trot
    buttons: [5, 2]
  - gait: stance
    buttons: [5, 3]
```

步态名称必须存在于：

```text
legged_controllers/config/go1/gait.info
```

Go1 默认可用 gait 包括：

- `stance`
- `trot`
- `standing_trot`
- `flying_trot`
- `pace`
- `standing_pace`
- `dynamic_walk`
- `static_walk`
- `amble`
- `lindyhop`
- `skipping`
- `pawup`

## 不使用手柄时的速度控制

启动目标轨迹发布器：

```bash
ros2 run legged_controllers legged_target_trajectories_publisher --ros-args \
  -p referenceFile:=/workspace/install/legged_controllers/share/legged_controllers/config/go1/reference.info \
  -p taskFile:=/workspace/install/legged_controllers/share/legged_controllers/config/go1/task.info
```

发布前进速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

停止：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -1
```

## 主要节点和话题

### `legged_controller`

类型：

```text
controller_interface::ControllerInterface
```

插件名：

```text
legged/LeggedController
```

主要职责：

- 读取 ros2_control state interfaces。
- 运行状态估计。
- 运行 OCS2 MPC / MRT。
- 调用 WBC。
- 写入 hybrid joint command interfaces。

主要订阅：

```text
/legged_robot_mpc_target
/legged_robot_mpc_mode_schedule
```

主要发布：

```text
/legged_robot_mpc_observation
```

关键参数：

```text
urdfFile
taskFile
referenceFile
imuName
```

这些参数在仿真中由 `go1_sim.launch.py` 生成到临时 controller yaml，再通过 Gazebo ros2_control 插件加载。

### `legged_target_trajectories_publisher`

源码：

```text
legged_controllers/src/TargetTrajectoriesPublisher.cpp
legged_controllers/include/legged_controllers/TargetTrajectoriesPublisher.h
```

主要职责：

- 订阅 `/cmd_vel`。
- 订阅 `/move_base_simple/goal`。
- 订阅 `/legged_robot_mpc_observation` 获取当前状态。
- 将速度或目标位姿转换为 OCS2 `TargetTrajectories`。
- 发布 `/legged_robot_mpc_target`。

关键变量：

```text
TARGET_DISPLACEMENT_VELOCITY
TARGET_ROTATION_VELOCITY
COM_HEIGHT
DEFAULT_JOINT_STATE
TIME_TO_TARGET
```

这些变量从 `reference.info` 和 `task.info` 加载。

核心函数：

```text
targetPoseToTargetTrajectories()
goalToTargetTrajectories()
cmdVelToTargetTrajectories()
```

### `joy_control`

源码：

```text
legged_controllers/scripts/joy_control.py
```

主要职责：

- 订阅 `sensor_msgs/msg/Joy`。
- 按 `joy.yaml` 或 `joy_sim.yaml` 生成 `/cmd_vel`。
- 按 `joy_gait_mappings.yaml` 识别组合键。
- 发布 `/legged_robot_mpc_mode_schedule` 切换 gait。

关键参数：

```text
joy_topic
cmd_vel_topic
teleop_config_file
gait_mappings_file
gait_command_file
robot_name
mode_schedule_topic
button_threshold
cooldown_sec
publish_repeats
cmd_vel_deadband_linear
cmd_vel_deadband_angular
```

关键变量：

```text
MODE_NAME_TO_INT
_teleop
_mappings
_gaits
_prev_joy
_last_fire
_last_gait_name
_last_cmd_zero
```

核心函数：

```text
parse_gait_info()
combo_active()
_load_teleop()
_load_mappings()
_publish_gait()
_deadman_ok()
_make_twist()
_joy_cb()
```

### `LeggedHWSim`

源码：

```text
legged_gazebo/src/LeggedHWSim.cpp
legged_gazebo/include/legged_gazebo/LeggedHWSim.h
```

插件名：

```text
legged_gazebo/LeggedHWSim
```

主要职责：

- 作为 `gazebo_ros2_control::GazeboSystemInterface` 接入 Gazebo。
- 导出关节位置、速度、力矩 state interfaces。
- 导出足端接触 state interfaces。
- 导出 IMU state interfaces。
- 接收 hybrid joint commands。
- 在 Gazebo 里计算并施加关节力矩。
- 发布 `/ground_truth/state`。

关键变量：

```text
jointData_
cmdBuffer_
contactStates_
contactLinkNames_
imuData_
delay_
baseLink_
groundTruthPublisher_
```

力矩计算：

```text
effort = kp * (position_desired - position)
       + kd * (velocity_desired - velocity)
       + feedforward
```

### `UnitreeHW`

源码：

```text
legged_robots/legged_unitree/legged_unitree_hw/src/UnitreeHW.cpp
```

插件名：

```text
legged_unitree_hw/UnitreeHW_3_8_0
```

主要职责：

- 接入 Unitree SDK。
- 读取电机、IMU、足端接触和遥控器。
- 发布 `/joy` 和 `/contact`。
- 将 controller 输出写入 Unitree low command。

## 控制计算逻辑

整体数据流：

```text
Joy / cmd_vel / goal
  -> legged_target_trajectories_publisher
  -> /legged_robot_mpc_target
  -> LeggedController
  -> OCS2 MPC
  -> WBC
  -> HybridJointCommand
  -> ros2_control hardware interface
  -> Gazebo / Unitree motors
```

步态数据流：

```text
Joy combo
  -> joy_control
  -> /legged_robot_mpc_mode_schedule
  -> GaitReceiver
  -> GaitSchedule
  -> SwingTrajectoryPlanner
  -> OCS2 constraints and reference manager
```

状态数据流：

```text
Gazebo / Unitree sensors
  -> ros2_control state interfaces
  -> LeggedController::update()
  -> StateEstimateBase / KalmanFilterEstimate
  -> SystemObservation
  -> OCS2 MRT / MPC
```

控制器每个周期大致执行：

1. 从 state interfaces 读关节、IMU、接触。
2. 更新状态估计，得到 `SystemObservation`。
3. 将 observation 送入 MPC/MRT。
4. 获取当前策略下的期望状态和输入。
5. WBC 根据期望状态、输入、接触模式求解关节力矩。
6. 将关节位置、速度、kp、kd、feedforward 写入 command interfaces。
7. 硬件插件把 command 写到 Gazebo 或真实机器人。

## 配置文件说明

### `task.info`

包含 MPC、代价、约束、求解器等任务参数，例如：

```text
mpc.timeHorizon
mpc.mrtDesiredFrequency
sqp.*
```

### `reference.info`

包含目标轨迹生成参数，例如：

```text
comHeight
defaultJointState
targetDisplacementVelocity
targetRotationVelocity
```

### `gait.info`

包含 gait 模板：

```text
modeSequence
switchingTimes
```

其中 `modeSequence` 使用 OCS2 legged robot 的 mode number。`joy_control.py` 内部的 `MODE_NAME_TO_INT` 将 `STANCE`、`LF_RH` 等名字转换为对应整数。

### `controllers.yaml`

ros2_control controller manager 配置：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    legged_controller:
      type: legged/LeggedController
```

当前 Go1 仿真使用 `1000 Hz` 控制器更新频率，尽量贴近原 ROS1/Gazebo 控制循环。降低该频率可能会让高速平移和快速转向更容易失稳。

## 常见问题

### `/controller_manager/list_controllers` 一直不可用

通常说明 Gazebo 中的 `gazebo_ros2_control` 没有成功启动。先看 `gzserver` 日志：

- `/spawn_entity` 是否成功。
- `libgazebo_ros2_control.so` 是否加载。
- `legged_gazebo/LeggedHWSim` 是否能被 pluginlib 找到。

### 第一次启动 `legged_controller` 很慢

OCS2/CppAD 会生成并编译动态库到：

```text
src/legged_control/legged_control/go1/...
```

这是正常现象。后续启动会快很多。

### `SCHED_FIFO Operation not permitted`

这是实时线程权限警告。仿真可以忽略；真机部署时建议配置实时权限。

### Gazebo ALSA/OpenAL 报错

容器或无声卡环境下常见，不影响控制。

### 手柄无输出

检查：

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```

如果 `/joy` 没有数据：

- USB/蓝牙手柄：确认 `start_joy_node:=true` 和 `joy_device_id`。
- Unitree 遥控器：确认 `UnitreeHW` 正在发布 `/joy`，并使用 `start_joy_node:=false joy_topic:=/joy`。

如果 `/cmd_vel` 没有数据：

- 真机配置需要按住 `deadman_buttons`。
- 仿真免 deadman 请使用 `joy_sim.yaml`。

### gait 没有切换

检查：

```bash
ros2 topic echo /legged_robot_mpc_mode_schedule
```

确认组合键编号和 `joy_gait_mappings.yaml` 一致，且 gait 名称存在于当前机器人类型的 `gait.info`。

## 引用

如果在学术工作中使用本项目，请保留原项目引用：

```bibtex
@misc{leggedcontrol,
  title = {{legged_control}: NMPC, WBC, state estimation, and sim2real framework for legged robots based on OCS2 and ros-controls},
  note = {[Online]. Available: \url{https://github.com/qiayuanl/legged_control}},
  author = {Qiayuan Liao and others}
}
```
