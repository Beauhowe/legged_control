# legged_p1_hw

`legged_p1_hw` 是 P1 机器人接入 `legged_control` 的 ROS 2 control 硬件接口包。它负责把控制器的 12 关节指令通过 Fast DDS 发给下位机，并把下位机返回的电机状态和 IMU 状态转换成 `legged_control` 使用的关节状态、IMU 状态和足端接触状态。

## 数据流

控制链路：

1. `legged_controller` 输出 12 个关节的期望位置、速度、PD 增益和前馈力矩。
2. `P1HW::write()` 将控制器命令填入 `P1DdsInterface::Command`。
3. `P1DdsInterface` 通过 pipe 把命令交给 `p1_dds_worker`。
4. `p1_dds_worker` 发布 DDS 话题 `p1_motor_cmd`。
5. 下位机订阅该话题并执行电机控制。

反馈链路：

1. 下位机发布 DDS 话题 `p1_motor_state` 和 `p1_imu`。
2. `p1_dds_worker` 在独立进程的 DDS 回调线程中收到电机状态和 IMU 状态。
3. `P1DdsInterface` 通过 pipe 读取 worker 发来的最新状态缓存。
4. `P1HW::read()` 读取最新状态，更新 `jointStates_`、`base_imu` 状态接口和 `contactStates_`。
4. `legged_controller` 在下一次控制周期中使用这些反馈。

## 代码结构

P1 硬件接口拆成三个主要部分：

| 文件 | 作用 |
| --- | --- |
| `P1HW` | ros2_control 硬件插件，负责 `read()`、`write()`、参数读取、关节映射和状态接口导出 |
| `P1DdsInterface` | ROS 进程内 IPC 客户端，负责和 `p1_dds_worker` 交换电机状态、IMU 和电机命令 |
| `p1_dds_worker` | FastDDS-only 子进程，订阅电机状态/IMU、发布电机命令 |
| `P1ContactEstimator` | 由单腿关节角和关节力矩通过雅可比估算足端力 |
| `p1_gait_dds_bridge` | ROS-only 节点，启动 FastDDS worker 并转换为 `/cmd_vel`、`/legged_robot_mpc_mode_schedule` 和站立/趴下目标轨迹 |
| `p1_gait_dds_worker` | FastDDS-only 子进程，订阅 `P1_Gait::gait_command` 后通过 pipe 交给 bridge |

`P1DdsInterface` 本身不再直接链接 Fast DDS。它启动 `p1_dds_worker` 子进程，并通过 pipe 交换状态和命令；DDS 回调只在 worker 进程中运行。`P1HW::read()` 在 controller manager 的控制周期内读取 IPC 缓存并写入 ros2_control 状态接口。

## DDS 话题和 IDL

默认使用以下 DDS 话题：

| 参数 | 默认值 | 方向 | IDL 类型 |
| --- | --- | --- | --- |
| `dds_state_topic` | `p1_motor_state` | 下位机到上位机 | `Motor_State_12::motor_state` |
| `dds_imu_topic` | `p1_imu` | 下位机到上位机 | `imu_topic_base::IMUData` |
| `dds_command_topic` | `p1_motor_cmd` | 上位机到下位机 | `Motor_Command_12::motor_cmd` |

对应 IDL 文件位于：

- `/workspace/src/fastdds_bridge/dds_topic/motor_12`
- `/workspace/src/fastdds_bridge/dds_topic/imu_topic_base`

电机命令使用 `Motor_Command_12::motor_cmd`，字段顺序为：

```text
kp[12], kd[12], pos[12], vel[12], torque[12], ff_torque[12], mode[12]
```

电机状态使用 `Motor_State_12::motor_state`，其中 P1 当前主要使用：

```text
position[12], speed[12], current[12], isonline[12]
```

## 关节顺序

控制器内部默认关节顺序为：

```text
LF_HAA, LF_HFE, LF_KFE,
LH_HAA, LH_HFE, LH_KFE,
RF_HAA, RF_HFE, RF_KFE,
RH_HAA, RH_HFE, RH_KFE
```

如果下位机 DDS 数组顺序与该顺序一致，不需要额外配置。

如果下位机顺序不同，可以通过 launch 参数 `dds_joint_order` 指定 DDS 数组中的关节顺序。例如：

```bash
ros2 launch legged_p1_hw p1.launch.py \
  dds_joint_order:=RF_HAA,RF_HFE,RF_KFE,RH_HAA,RH_HFE,RH_KFE,LF_HAA,LF_HFE,LF_KFE,LH_HAA,LH_HFE,LH_KFE
```

也可以在 ros2_control 硬件参数中使用单关节映射覆盖：

```xml
<param name="dds_joint_index_LF_HAA">0</param>
<param name="dds_joint_index_LF_HFE">1</param>
<param name="dds_joint_index_LF_KFE">2</param>
```

映射含义是：控制器第 `i` 个关节应该读写 DDS 数组中的第几个元素。

## 电流到关节力矩

`legged_control` 中 `jointStates_[i].effort` 表示关节力矩，不是足端接触力。

P1 下位机当前返回的是 `current[12]`，因此硬件接口预留了电流到关节力矩的线性估算入口：

```text
joint_torque = current * current_to_torque_scale + current_to_torque_offset
```

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `current_to_torque_scale` | `1.0` | 全局电流到力矩比例 |
| `current_to_torque_offset` | `0.0` | 全局力矩偏置 |

也可以按关节配置：

```xml
<param name="current_to_torque_scale_LF_HAA">1.0</param>
<param name="current_to_torque_offset_LF_HAA">0.0</param>
```

或者按关节索引配置：

```xml
<param name="current_to_torque_scale_0">1.0</param>
<param name="current_to_torque_offset_0">0.0</param>
```

实际使用前需要根据 P1 电机的电流单位、减速比、力矩常数和符号方向完成标定。

## 足端接触估计

P1 支持两种接触估计方式。

### current

```text
contact_estimation_method=current
```

该模式直接使用电流绝对值做阈值判断：

```text
contact = abs(current) > contact_threshold
```

这是默认模式，适合在力矩标定完成前先跑通链路。

### jacobian

```text
contact_estimation_method=jacobian
```

该模式先把电流估算成关节力矩，再由 `P1ContactEstimator` 用腿部雅可比估算足端力：

```text
tau = J^T * F
F = solve(J^T, tau)
contact = abs(Fz) > contact_force_threshold
```

该估计是准静态近似，动态运动时会受到惯性、重力、科氏力和电机摩擦影响。要提高准确度，需要先保证关节力矩标定正确，再根据实机数据调整 `contact_force_threshold`。

## 与 Go1 硬件接口的对应关系

P1 的 ros2_control 接口语义和 Go1 的 `legged_unitree_hw/UnitreeHW` 主逻辑保持一致：

| 通用变量 | Go1 来源/去向 | P1 来源/去向 |
| --- | --- | --- |
| `jointStates_[i].position` | `lowState_.motorState[motorIndex].q` | `motor_state.position[ddsIndex]` |
| `jointStates_[i].velocity` | `lowState_.motorState[motorIndex].dq` | `motor_state.speed[ddsIndex]` |
| `jointStates_[i].effort` | `lowState_.motorState[motorIndex].tauEst` | `current_to_torque(current[ddsIndex])` |
| `jointCommands_[i].position_desired` | `lowCmd_.motorCmd[motorIndex].q` | `motor_cmd.pos[ddsIndex]` |
| `jointCommands_[i].velocity_desired` | `lowCmd_.motorCmd[motorIndex].dq` | `motor_cmd.vel[ddsIndex]` |
| `jointCommands_[i].kp` | `lowCmd_.motorCmd[motorIndex].Kp` | `motor_cmd.kp[ddsIndex]` |
| `jointCommands_[i].kd` | `lowCmd_.motorCmd[motorIndex].Kd` | `motor_cmd.kd[ddsIndex]` |
| `jointCommands_[i].feedforward` | `lowCmd_.motorCmd[motorIndex].tau` | `motor_cmd.torque/ff_torque[ddsIndex]` |
| `contactStates_[i]` | `footForce > contact_threshold` | `current` 阈值或 `P1ContactEstimator` 估算 `Fz` 阈值 |

需要注意的差异：

1. Go1 的 `tauEst` 是 SDK 直接给出的关节估计力矩；P1 当前只有电流反馈，所以 `jointStates_[i].effort` 依赖 `current_to_torque_scale` 和 `current_to_torque_offset` 标定。
2. Go1 的接触状态来自足端 `footForce`；P1 的接触状态当前是估算值，不是直接传感器测量值。
3. Go1 在发送命令前调用 SDK 的位置/功率保护；P1 当前假设下位机负责安全保护。若下位机没有完整保护，建议后续在 `P1HW::write()` 前补软件限位和功率/力矩限制。
4. Go1 在每次 `read()` 后会重置部分命令默认值；P1 当前不重置命令，要求控制器按周期完整写入命令。

## DDS Gait Bridge

`p1_gait_dds_bridge` 用于让 P1 通过 Fast DDS 接收步态和速度命令。为避免 ROS 2 Humble 自带 DDS 与下位机 Fast DDS 版本在同一进程内冲突，它拆成两个进程：ROS bridge 只链接 ROS/OCS2，`p1_gait_dds_worker` 只链接下位机 Fast DDS。worker 订阅：

```text
p1_gait_command   P1_Gait::gait_command
```

IDL 字段语义：

```text
stamp       时间戳，目前只透传接收，不参与控制
gait_id     按 gait_id_mapping 选择步态，默认 0=stance, 1=trot, 2=standing_trot, 3=pace, 4=static_walk, 5=lie_down
trigger     false->true 上升沿触发一次步态切换
velocity_x  发布到 /cmd_vel.linear.x
velocity_y  发布到 /cmd_vel.linear.y
yaw_rate    发布到 /cmd_vel.angular.z
```

桥接后的 ROS 2 输出：

| 输出 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 提供机身速度目标 |
| `/legged_robot_mpc_mode_schedule` | `ocs2_msgs/msg/ModeSchedule` | 提供 MPC gait 接触时序 |
| `/legged_robot_mpc_target` | `ocs2_msgs/msg/MpcTargetTrajectories` | 在站立/趴下触发时发布姿态目标 |

`trigger` 只控制 gait 和姿态切换；速度命令每帧都会发布。`gait_id` 到具体步态的映射来自 `gait_id_mapping`，其中每个名字必须存在于 `gait_file`。

默认启动流程是：`reference_file` 使用 `reference_lie_down.info`，控制器先以低高度趴下姿态启动；DDS 发送 `gait_id=0, trigger=true` 时，bridge 发布 `stance` ModeSchedule，并从 `stand_reference_file` 读取 `comHeight/defaultJointState` 发布站立目标；站稳后再发送 `gait_id=1..4` 切到 trot、standing_trot、pace 或 static_walk，并用速度字段控制运动。DDS 发送 `gait_id=5, trigger=true` 时，会发布 `lie_down` ModeSchedule 和低高度目标。

## Launch

基本启动：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch legged_p1_hw p1.launch.py
```

常用参数示例：

```bash
ros2 launch legged_p1_hw p1.launch.py \
  dds_domain:=0 \
  dds_state_topic:=p1_motor_state \
  dds_imu_topic:=p1_imu \
  dds_command_topic:=p1_motor_cmd \
  command_mode:=10 \
  contact_estimation_method:=jacobian \
  contact_force_threshold:=40.0 \
  current_to_torque_scale:=1.0 \
  current_to_torque_offset:=0.0 \
  start_gait_bridge:=true \
  dds_gait_topic:=p1_gait_command \
  gait_id_mapping:=stance,trot,standing_trot,pace,static_walk,lie_down
```

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_type` | `p1` | 使用 P1 URDF/config |
| `task_file` | P1 task.info | 控制器任务配置 |
| `reference_file` | P1 reference_lie_down.info | 控制器启动参考配置，默认低高度趴下 |
| `hardware_plugin` | `legged_p1_hw/P1HW` | ros2_control 硬件插件 |
| `dds_domain` | `0` | Fast DDS domain id |
| `dds_state_topic` | `p1_motor_state` | 电机状态话题 |
| `dds_imu_topic` | `p1_imu` | IMU 话题 |
| `dds_command_topic` | `p1_motor_cmd` | 电机命令话题 |
| `dds_joint_order` | 控制器默认顺序 | DDS 数组关节顺序 |
| `command_mode` | `10` | 写入 `motor_cmd.mode[12]` 的控制模式 |
| `contact_threshold` | `40` | `current` 接触估计模式使用的阈值 |
| `contact_estimation_method` | `current` | `current` 或 `jacobian` |
| `contact_force_threshold` | `40.0` | `jacobian` 接触估计模式使用的足端力阈值 |
| `current_to_torque_scale` | `1.0` | 电流到关节力矩比例 |
| `current_to_torque_offset` | `0.0` | 电流到关节力矩偏置 |
| `start_gait_bridge` | `true` | 是否随 P1 launch 启动 Fast DDS gait bridge |
| `dds_gait_topic` | `p1_gait_command` | DDS gait 指令话题 |
| `gait_file` | P1 gait.info | ModeSchedule 步态配置文件 |
| `gait_id_mapping` | `stance,trot,standing_trot,pace,static_walk,lie_down` | DDS `gait_id` 到 gait 名称的映射 |
| `cmd_vel_topic` | `/cmd_vel` | bridge 发布速度命令的话题 |
| `mode_schedule_topic` | `/legged_robot_mpc_mode_schedule` | bridge 发布 ModeSchedule 的话题 |
| `target_topic` | `/legged_robot_mpc_target` | bridge 发布站立/趴下目标轨迹的话题 |
| `observation_topic` | `/legged_robot_mpc_observation` | bridge 读取当前 MPC observation，用来保留当前 x/y/yaw |
| `stand_reference_file` | P1 reference.info | `stand_gait_id` 触发时读取的站立高度和默认关节角 |
| `lie_down_reference_file` | P1 reference_lie_down.info | `lie_down_gait_id` 触发时读取的趴下高度和默认关节角 |
| `posture_transition_duration` | `2.0` | 站立/趴下目标轨迹持续时间，单位秒 |
| `stand_gait_id` | `0` | 触发站立目标的 DDS gait id |
| `lie_down_gait_id` | `5` | 触发趴下目标的 DDS gait id |

## 构建

Fast DDS 安装路径默认按 `/workspace/src/env/fast_dds/local` 查找，也可以手动指定：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_p1_hw \
  --cmake-args -DFASTDDS_INSTALL_PREFIX=/workspace/src/env/fast_dds/local
```

构建完成后：

```bash
source /workspace/install/setup.bash
ros2 launch legged_p1_hw p1.launch.py
```

## 仿真测试趴下姿态

`lie_down` 在 `gait.info` 中保持四足支撑，真正降低身体高度依赖低高度 reference 文件。可以用下面命令启动 P1 趴下姿态仿真：

```bash
ros2 launch legged_gazebo p1_sim.launch.py \
  reference_file:=/workspace/src/legged_control/legged_controllers/config/p1/reference_lie_down.info
```

该配置使用：

```text
comHeight = 0.26
HAA = 0.0, HFE = 1.35, KFE = -2.55
```

P1 真机 launch 现在默认使用 `reference_lie_down.info` 启动；DDS gait bridge 也会在 `gait_id=0` 和 `gait_id=5` 的上升沿分别发布站立/趴下目标轨迹。仿真中如果只想单独验证趴下初始姿态，可以继续显式传入上面的 `reference_file`。

## 调试建议

1. 先确认 DDS domain 和三个话题名与下位机完全一致。
2. 确认 DDS 数组关节顺序，必要时配置 `dds_joint_order`。
3. 初次上电建议使用较小的 `kp`、`kd` 和前馈力矩，确认方向正确后再提高参数。
4. 标定 `current_to_torque_scale` 和 `current_to_torque_offset` 前，不要把 `jointStates_.effort` 当作真实力矩使用。
5. 使用 `jacobian` 接触估计前，先确认关节位置、关节力矩和腿部几何方向都正确。
6. 如果控制器能启动但机器人不动，优先检查 `mode[12]` 是否符合下位机控制模式定义。
