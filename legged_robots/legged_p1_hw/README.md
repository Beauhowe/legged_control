# legged_p1_hw

`legged_p1_hw` 是 P1 机器人接入 `legged_control` 的 ROS 2 control 硬件接口包。它负责把控制器的 12 关节指令通过 Fast DDS 发给下位机，并把下位机返回的电机状态和 IMU 状态转换成 `legged_control` 使用的关节状态、IMU 状态和足端接触状态。

## 数据流

控制链路：

1. `legged_controller` 输出 12 个关节的期望位置、速度、PD 增益和前馈力矩。
2. `P1HW::write()` 将控制器命令填入 `P1DdsInterface::Command`。
3. `P1DdsInterface` 发布 DDS 话题 `p1_motor_cmd`。
4. 下位机订阅该话题并执行电机控制。

反馈链路：

1. 下位机发布 DDS 话题 `p1_motor_state` 和 `p1_imu`。
2. `P1DdsInterface` 在 DDS 回调线程中保存最新电机状态和 IMU 状态。
3. `P1HW::read()` 读取最新状态，更新 `jointStates_`、`base_imu` 状态接口和 `contactStates_`。
4. `legged_controller` 在下一次控制周期中使用这些反馈。

## 代码结构

P1 硬件接口拆成三个主要部分：

| 文件 | 作用 |
| --- | --- |
| `P1HW` | ros2_control 硬件插件，负责 `read()`、`write()`、参数读取、关节映射和状态接口导出 |
| `P1DdsInterface` | Fast DDS 通讯封装，负责订阅电机状态/IMU、发布电机命令 |
| `P1ContactEstimator` | 由单腿关节角和关节力矩通过雅可比估算足端力 |

`P1DdsInterface` 的 DDS 回调线程只更新内部缓存，不直接访问 `jointStates_`。`P1HW::read()` 在 controller manager 的控制周期内读取缓存并写入 ros2_control 状态接口。

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
  current_to_torque_offset:=0.0
```

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_type` | `p1` | 使用 P1 URDF/config |
| `task_file` | P1 task.info | 控制器任务配置 |
| `reference_file` | P1 reference.info | 控制器参考配置 |
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

## 调试建议

1. 先确认 DDS domain 和三个话题名与下位机完全一致。
2. 确认 DDS 数组关节顺序，必要时配置 `dds_joint_order`。
3. 初次上电建议使用较小的 `kp`、`kd` 和前馈力矩，确认方向正确后再提高参数。
4. 标定 `current_to_torque_scale` 和 `current_to_torque_offset` 前，不要把 `jointStates_.effort` 当作真实力矩使用。
5. 使用 `jacobian` 接触估计前，先确认关节位置、关节力矩和腿部几何方向都正确。
6. 如果控制器能启动但机器人不动，优先检查 `mode[12]` 是否符合下位机控制模式定义。
