# legged_control 项目运行与代码手册

> 本手册按你当前 `README_RUN.md` 推荐的两终端启动流程梳理整个工程的运行路径、数据流以及关键的类 / 函数 / 变量职责。读完之后你应能：
>
> 1. 从一条 `roslaunch` 命令出发，准确指出每个进程、每个话题、每个回调的源码位置；
> 2. 理解一次控制周期内数据是如何在 Gazebo → 估计器 → MPC → WBC → 关节力矩之间流动的；
> 3. 把这套架构迁移到另一种足式机器人（更换 URDF、配置、关节名、足端等）。

---

## 目录

1. [项目分层与目录速览](#1-项目分层与目录速览)
2. [启动脚本逐步解析](#2-启动脚本逐步解析)
3. [核心数据结构与控制周期时序](#3-核心数据结构与控制周期时序)
4. [`LeggedController` 详解](#4-leggedcontroller-详解)
5. [`LeggedInterface` 与最优控制问题装配](#5-leggedinterface-与最优控制问题装配)
6. [步态与目标轨迹的 ROS 数据流](#6-步态与目标轨迹的-ros-数据流)
7. [状态估计：`StateEstimateBase` / `KalmanFilterEstimate`](#7-状态估计stateestimatebase--kalmanfilterestimate)
8. [全身控制：`WbcBase` / `WeightedWbc`](#8-全身控制wbcbase--weightedwbc)
9. [仿真硬件：`LeggedHWSim`](#9-仿真硬件leggedhwsim)
10. [真机硬件路径：`LeggedHW` / `LeggedHWLoop` / `UnitreeHW`](#10-真机硬件路径leggedhw--leggedhwloop--unitreehw)
11. [手柄遥控链路](#11-手柄遥控链路)
12. [配置文件字段对照](#12-配置文件字段对照)
13. [关键 ROS 话题 / 参数 / 服务总表](#13-关键-ros-话题--参数--服务总表)
14. [迁移到其他机器人的修改清单](#14-迁移到其他机器人的修改清单)
15. [调试与常见问题](#15-调试与常见问题)

---

## 1. 项目分层与目录速览

工作区核心是 **`legged_control`**（四足 NMPC + WBC 控制栈），构建在 **`OCS2`**（最优控制 / SQP-MPC 求解器）之上；仿真侧用 **`Gazebo + ros_control`**；真机走 Unitree SDK。

| 层次 | 主要包 | 关键职责 |
|------|--------|----------|
| 描述与仿真 | `legged_unitree_description`、`legged_gazebo` | URDF/xacro、Gazebo 世界、仿真硬件插件 `LeggedHWSim` |
| 机器人接口（OCP） | `legged_interface` | Pinocchio 模型、质心动力学、约束、代价、参考管理 |
| 状态估计 | `legged_estimation` | 关节 + IMU + 触地 → 浮动基状态；可选外部里程计修正 |
| 控制器（ros_control 插件） | `legged_controllers` | 把估计 → MPC → WBC → 关节命令串起来 |
| 全身控制 | `legged_wbc` | QP：力 / 力矩 / 加速度分配 |
| 公共硬件抽象 | `legged_common` | `HybridJointInterface`、`ContactSensorInterface` |
| 真机硬件 | `legged_examples/legged_unitree/legged_unitree_hw`、`legged_hw` | UDP 通信、`controller_manager` 主循环 |
| 上游 ROS 封装 | `ocs2_legged_robot_ros` | 步态键盘节点、`GaitReceiver`、`TargetTrajectoriesRosPublisher` |

补充重要文件：

```
src/legged_control/
├── legged_examples/legged_unitree/legged_unitree_description/launch/empty_world.launch
├── legged_controllers/launch/load_controller.launch
├── legged_controllers/launch/joy_teleop.launch
├── legged_controllers/launch/joy_gait_mux.launch
├── legged_controllers/config/controllers.yaml
├── legged_controllers/config/<robot>/{task.info, reference.info, gait.info}
├── legged_controllers/config/joy.yaml
├── legged_controllers/config/joy_gait_mappings.yaml
├── legged_controllers/scripts/joy_gait_publisher.py
├── legged_controllers/src/{LeggedController.cpp, TargetTrajectoriesPublisher.cpp}
├── legged_interface/src/{LeggedInterface.cpp, SwitchedModelReferenceManager.cpp, ...}
├── legged_estimation/src/{StateEstimateBase.cpp, LinearKalmanFilter.cpp, FromTopicEstimate.cpp}
├── legged_wbc/src/{WbcBase.cpp, WeightedWbc.cpp, HierarchicalWbc.cpp}
├── legged_gazebo/{src/LeggedHWSim.cpp, config/default.yaml}
└── legged_common/scripts/generate_urdf.sh
```

---

## 2. 启动脚本逐步解析

`README_RUN.md` 第 4 节推荐的最小启动顺序是：

```
终端 A: roslaunch legged_unitree_description empty_world.launch
终端 B: roslaunch legged_controllers   load_controller.launch cheater:=false
       (然后用 rqt / rosservice 启动 controllers/legged_controller)
可选 C: roslaunch legged_controllers joy_teleop.launch enable_joy_gait:=true
可选 D: rosrun rviz rviz -d $(rospack find ocs2_legged_robot_ros)/rviz/legged_robot.rviz
```

下面按 launch 文件逐项展开。

### 2.1 终端 A：`empty_world.launch`

文件：`legged_unitree_description/launch/empty_world.launch`

```xml
<arg name="robot_type" default="$(env ROBOT_TYPE)"/>

<param name="legged_robot_description"
       command="$(find xacro)/xacro $(find legged_unitree_description)/urdf/robot.xacro
                robot_type:=$(arg robot_type)"/>

<node name="generate_urdf" pkg="legged_common" type="generate_urdf.sh" output="screen"
      args="$(find legged_unitree_description)/urdf/robot.xacro $(arg robot_type)"/>

<rosparam file="$(find legged_gazebo)/config/default.yaml" command="load"/>
<include file="$(find gazebo_ros)/launch/empty_world.launch">
  <arg name="world_name" value="$(find legged_gazebo)/worlds/empty_world.world"/>
</include>
<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" clear_params="true"
      args="-z 0.5 -param legged_robot_description -urdf -model $(arg robot_type)"/>
```

**做了什么**（按顺序）：

1. **`legged_robot_description` 参数**：把 `urdf/robot.xacro` 用 `xacro` 展开为一份完整 URDF，写到参数服务器。这份用于 Gazebo `spawn_model` 与 RViz `RobotModel`。
2. **`generate_urdf` 节点**：调用 `legged_common/scripts/generate_urdf.sh`：
   ```bash
   rosrun xacro xacro $1 robot_type:=$2 > /tmp/legged_control/$2.urdf
   ```
   将同一个 xacro 落盘为 **`/tmp/legged_control/<robot_type>.urdf`**。后续 `LeggedController` / `LeggedInterface` 通过参数 `/urdfFile` 读取这条路径，给 Pinocchio 使用。
3. **`legged_gazebo/config/default.yaml`** 加载到参数服务器：
   ```yaml
   gazebo:
     delay: 0.009
     imus:
       base_imu:
         frame_id: base_imu
         orientation_covariance_diagonal: [0.0012, 0.0012, 0.0012]
         angular_velocity_covariance:     [0.0004, 0.0004, 0.0004]
         linear_acceleration_covariance:  [0.01,   0.01,   0.01]
     contacts: ["LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"]
   ```
   `LeggedHWSim` 用它注册 IMU 与触地传感器。
4. **`gazebo_ros empty_world.launch`** 启 Gazebo 并加载 `empty_world.world`。
5. **`spawn_urdf`**：在 `z=0.5` 处生成机器人模型，名字为 `$(robot_type)`。Gazebo 在加载 URDF 时通过 `<gazebo>` 标签把 **`legged::LeggedHWSim`** 注册为 `gazebo_ros_control` 的 `RobotHWSim` 实现（见 `LeggedHWSim.cpp` 末尾 `PLUGINLIB_EXPORT_CLASS` 与 `GZ_REGISTER_MODEL_PLUGIN`）。

### 2.2 终端 B：`load_controller.launch`

文件：`legged_controllers/launch/load_controller.launch`

```xml
<arg name="robot_type" default="$(env ROBOT_TYPE)"/>
<arg name="cheater" default="false"/>

<param name="urdfFile"        value="/tmp/legged_control/$(arg robot_type).urdf"/>
<param name="taskFile"        value="$(find legged_controllers)/config/$(arg robot_type)/task.info"/>
<param name="referenceFile"   value="$(find legged_controllers)/config/$(arg robot_type)/reference.info"/>
<param name="gaitCommandFile" value="$(find legged_controllers)/config/$(arg robot_type)/gait.info"/>

<rosparam file="$(find legged_controllers)/config/controllers.yaml" command="load"/>

<node ... name="controller_loader" pkg="controller_manager" type="controller_manager"
      args="load controllers/joint_state_controller controllers/legged_controller [legged_cheater_controller]"/>

<node pkg="ocs2_legged_robot_ros" type="legged_robot_gait_command" name="legged_robot_gait_command" output="screen"/>
<node pkg="legged_controllers"    type="legged_target_trajectories_publisher" name="legged_robot_target" output="screen"/>
```

**做了什么**：

1. **全局参数**：把四份文件路径放到参数服务器上，所有 C++/Python 节点读它们：
   - `/urdfFile` — Pinocchio 用
   - `/taskFile` — MPC/WBC/估计器全部读它
   - `/referenceFile` — 默认关节位姿、目标速度上限、初始 mode schedule
   - `/gaitCommandFile` — 可用步态列表与模板

2. **`controllers.yaml`** 注册插件：
   ```yaml
   controllers:
     joint_state_controller:    { type: joint_state_controller/JointStateController, publish_rate: 100 }
     legged_controller:         { type: legged/LeggedController }
     legged_cheater_controller: { type: legged/LeggedCheaterController }
   ```

3. **`controller_manager` 仅 `load` 不 `start`**。因此你必须在另一个终端用 `rosservice call /controller_manager/switch_controller` 或 `rqt_controller_manager` 启动 `controllers/legged_controller`。这一动作真正进入 `LeggedController::init()` 与 `starting()`。

4. **`legged_robot_gait_command`**：键盘交互节点，读 `/gaitCommandFile`，向 `/legged_robot_mpc_mode_schedule` 发布步态模板。

5. **`legged_target_trajectories_publisher`**：订阅 `/cmd_vel` 与 `/move_base_simple/goal`，配合 `/legged_robot_mpc_observation`，向 `/legged_robot_mpc_target` 发 `ocs2_msgs/mpc_target_trajectories`。

### 2.3 终端 C（可选）：手柄遥控

文件：`legged_controllers/launch/joy_teleop.launch`、`joy_gait_mux.launch`

- 启 `joy_node`，把 `/joy` remap 到 `/legged_robot/joystick`。
- 加载 `joy.yaml`，启 `joy_teleop/joy_teleop.py`，根据摇杆轴向发 `/cmd_vel`（按住 deadman 才发）。
- 若 `enable_joy_gait:=true`，再启 `joy_gait_publisher.py`，根据按键组合发 `/legged_robot_mpc_mode_schedule`。

---

## 3. 核心数据结构与控制周期时序

### 3.1 OCS2 `SystemObservation`

```cpp
struct SystemObservation {
  scalar_t time;     // 累加 ros_control 周期
  vector_t state;    // dim = stateDim：12+12 = 24 (质心动量 6 + 基座位姿 6 + 12 关节角)
  vector_t input;    // dim = inputDim：12+12 = 24 (足底力 12 + 关节速度 12)
  size_t   mode;     // mode number，由 contact flag 编码（见下）
};
```

### 3.2 触地编码

`ocs2_legged_robot/MotionPhaseDefinition.h` 定义 `ModeNumber`：

```
mode = (LF<<3) | (RF<<2) | (LH<<1) | RH   // 取值 0..15
```

`joy_gait_publisher.py` 顶部的 `MODE_NAME_TO_INT` 字典与此一致：`FLY=0`、`STANCE=15`、`LF_RH=9`、`RF_LH=6` 等。

### 3.3 RBD 状态向量布局（估计器输出）

`StateEstimateBase::rbdState_` 长度 `2 * generalizedCoordinatesNum`，按 segment 拆分：

| 段 | 含义 |
|----|------|
| `[0..3)` | 基座 ZYX 欧拉角 |
| `[3..6)` | 基座位置 x,y,z |
| `[6..6+nJ)` | 12 关节位置 |
| `[gCN..gCN+3)` | 基座全局角速度 |
| `[gCN+3..gCN+6)` | 基座全局线速度 |
| `[gCN+6..]` | 12 关节速度 |

其中 `gCN = info.generalizedCoordinatesNum = 18`。

### 3.4 控制周期总览（一拍）

```
┌─────────────────────────────────────────────────────────────────────────┐
│ 仿真：Gazebo 物理 step                                                  │
│   ├─► LeggedHWSim::readSim()                                            │
│   │     - 关节位置/速度  -> JointStateInterface                         │
│   │     - IMU 姿态/角速/线加速 -> ImuSensorInterface                    │
│   │     - 接触布尔 -> ContactSensorInterface                            │
│   │                                                                     │
│   ├─► controllerManager_->update()                                      │
│   │   └─► LeggedController::update(time, period):                       │
│   │         1) updateStateEstimation() -> measuredRbdState_             │
│   │            -> rbdConversions_  -> currentObservation_               │
│   │         2) mpcMrtInterface_->setCurrentObservation(...)             │
│   │         3) mpcMrtInterface_->updatePolicy() (取后台 MPC 结果)       │
│   │         4) evaluatePolicy() -> optimizedState/Input/plannedMode     │
│   │         5) WeightedWbc::update() -> x; torque = x.tail(12)          │
│   │         6) hybridJointHandles_[j].setCommand(pos,vel,0,3,τ)         │
│   │         7) 发布 /legged_robot_mpc_observation                       │
│   │                                                                     │
│   ├─► LeggedHWSim::writeSim()                                           │
│   │     - 取 delay 后的 Hybrid 命令                                     │
│   │     - τ_send = kd*(velDes-vel) + ff                                 │
│   │     - 写到 Gazebo joint effort                                      │
│                                                                         │
│ 与此并行：mpcThread_ 后台以 mpcDesiredFrequency_=100Hz 调用             │
│   mpcMrtInterface_->advanceMpc()                                        │
└─────────────────────────────────────────────────────────────────────────┘
```

外部异步事件：

- `/legged_robot_mpc_mode_schedule` 到达 → `GaitReceiver::mpcModeSequenceCallback` → 下一次 `preSolverRun` 把模板插入 `GaitSchedule`。
- `/legged_robot_mpc_target` 到达 → `RosReferenceManager` 内部 `setTargetTrajectories`。
- `/cmd_vel`、`/move_base_simple/goal` 到达 → `TargetTrajectoriesPublisher` 把它们转成 `mpc_target_trajectories` 再发出。

---

## 4. `LeggedController` 详解

文件：`legged_controllers/include/legged_controllers/LeggedController.h`、`src/LeggedController.cpp`

### 4.1 类层级

```cpp
class LeggedController : public controller_interface::MultiInterfaceController<
                            HybridJointInterface,
                            hardware_interface::ImuSensorInterface,
                            ContactSensorInterface> { ... };

class LeggedCheaterController : public LeggedController {
  void setupStateEstimate(...) override;   // 用 FromTopicStateEstimate（真值/外部）
};
```

通过 `PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)` 注册为 `ros_control` 插件，由 `controllers.yaml` 的 `legged/LeggedController` 加载。

### 4.2 关键成员

```cpp
// OCS2 接口
std::shared_ptr<LeggedInterface> leggedInterface_;
std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

// 仿真/硬件 handle
std::vector<HybridJointHandle>  hybridJointHandles_;  // 12
std::vector<ContactSensorHandle> contactHandles_;     // 4
hardware_interface::ImuSensorHandle imuSensorHandle_; // "base_imu"

// 估计器
SystemObservation currentObservation_;
vector_t          measuredRbdState_;
std::shared_ptr<StateEstimateBase>           stateEstimate_;
std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

// WBC + 安全
std::shared_ptr<WbcBase>       wbc_;
std::shared_ptr<SafetyChecker> safetyChecker_;

// MPC
std::shared_ptr<MPC_BASE>          mpc_;
std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

// 可视化
std::shared_ptr<LeggedRobotVisualizer>          robotVisualizer_;
std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
ros::Publisher observationPublisher_;   // /legged_robot_mpc_observation

// 后台线程
std::thread       mpcThread_;
std::atomic_bool  controllerRunning_, mpcRunning_;
benchmark::RepeatedTimer mpcTimer_, wbcTimer_;
```

### 4.3 `bool init(robot_hw, controller_nh)`

按顺序：

1. 从参数服务器读 `/urdfFile`、`/taskFile`、`/referenceFile`。从 `taskFile.legged_robot_interface.verbose` 读 `verbose`。
2. **`setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose)`**：
   ```cpp
   leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
   leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
   ```
3. **`setupMpc()`**：建 `SqpMpc`、`CentroidalModelRbdConversions`，挂 `GaitReceiver`、设 `RosReferenceManager`，并 advertise `/legged_robot_mpc_observation`。
4. **`setupMrt()`**：建 `MPC_MRT_Interface`，启动 `mpcThread_` 后台循环 `advanceMpc()`。
5. 可视化：建 `PinocchioEndEffectorKinematics`、`LeggedRobotVisualizer`、`LeggedSelfCollisionVisualization`。
6. **从 `RobotHW` 取 handle**：
   ```cpp
   std::vector<std::string> joint_names{
     "LF_HAA","LF_HFE","LF_KFE", "LH_HAA","LH_HFE","LH_KFE",
     "RF_HAA","RF_HFE","RF_KFE", "RH_HAA","RH_HFE","RH_KFE"};
   // ⚠ 关节名硬编码，迁移机器人时要么改 URDF 关节名，要么改这里
   for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF)
     contactHandles_.push_back(contactInterface->getHandle(name));
   imuSensorHandle_ = robot_hw->get<ImuSensorInterface>()->getHandle("base_imu");
   ```
7. **`setupStateEstimate(taskFile, verbose)`**：默认 `KalmanFilterEstimate`，读 `taskFile.kalmanFilter` 一节。`LeggedCheaterController` 重写为 `FromTopicStateEstimate`。
8. **WBC**：`wbc_ = std::make_shared<WeightedWbc>(...)`，`wbc_->loadTasksSetting(taskFile, verbose)`。
9. **SafetyChecker**：构造时只持有 `CentroidalModelInfo` 引用。

### 4.4 `void starting(const ros::Time& time)`

```cpp
currentObservation_.state.setZero(stateDim);
updateStateEstimation(time, ros::Duration(0.002));   // 填一帧测量
currentObservation_.input.setZero(inputDim);
currentObservation_.mode = ModeNumber::STANCE;

TargetTrajectories target({currentObservation_.time},
                          {currentObservation_.state},
                          {currentObservation_.input});

mpcMrtInterface_->setCurrentObservation(currentObservation_);
mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target);

while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
  mpcMrtInterface_->advanceMpc();
  ros::WallRate(mrtDesiredFrequency_).sleep();
}
mpcRunning_ = true;
```

**要点**：第一次 `update()` 前必须有可用 policy；这里用「停在原地的目标」反复 `advanceMpc()` 直到第一份策略产生。

### 4.5 `void update(const ros::Time& time, const ros::Duration& period)`

```cpp
updateStateEstimation(time, period);                  // 写 currentObservation_ & measuredRbdState_

mpcMrtInterface_->setCurrentObservation(currentObservation_);
mpcMrtInterface_->updatePolicy();                     // 拉后台最新策略

vector_t optimizedState, optimizedInput;
size_t   plannedMode = 0;
mpcMrtInterface_->evaluatePolicy(currentObservation_.time,
                                 currentObservation_.state,
                                 optimizedState, optimizedInput, plannedMode);

currentObservation_.input = optimizedInput;

vector_t x = wbc_->update(optimizedState, optimizedInput,
                          measuredRbdState_, plannedMode, period.toSec());
vector_t torque = x.tail(12);

vector_t posDes = centroidal_model::getJointAngles    (optimizedState, info);
vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, info);

if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput))
  stopRequest(time);

for (size_t j = 0; j < info.actuatedDofNum; ++j)
  hybridJointHandles_[j].setCommand(posDes(j), velDes(j), /*kp*/0, /*kd*/3, torque(j));

robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(),
                                              mpcMrtInterface_->getCommand());
selfCollisionVisualization_->update(currentObservation_);
observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
```

**关键设计**：

- WBC 返回 `x = [u̇, F, τ]`，控制器只取最后 12 维力矩，前馈给关节；同时把 MPC 评估出的关节角/速作为 `posDes/velDes` 给硬件抽象层，但 `kp=0, kd=3`，也就是 **力矩主导 + 速度阻尼**，位置环交给 WBC + MPC 处理。
- 安全检查只看基座 roll，超过 `±π/2` 立刻 `stopRequest`。

### 4.6 `void updateStateEstimation(time, period)`

```cpp
for (size_t i=0;i<12;++i) { jointPos[i]=...; jointVel[i]=...; }
for (size_t i=0;i<4;++i)  contactFlag[i] = contactHandles_[i].isContact();
quat = imuSensorHandle_.getOrientation();          // 4 维
angularVel  = imuSensorHandle_.getAngularVelocity();
linearAccel = imuSensorHandle_.getLinearAcceleration();
... // 协方差

stateEstimate_->updateJointStates(jointPos, jointVel);
stateEstimate_->updateContact(contactFlag);
stateEstimate_->updateImu(quat, angularVel, linearAccel, ...);
measuredRbdState_ = stateEstimate_->update(time, period);

currentObservation_.time += period.toSec();
scalar_t yawLast = currentObservation_.state(9);
currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
currentObservation_.mode = stateEstimate_->getMode();   // contact -> mode number
```

`computeCentroidalStateFromRbdModel` 由 OCS2 提供，把 RBD 广义状态转成 24 维质心 OCP 状态。yaw 用 `shortest_angular_distance` unwrap，避免 ±π 跳变。

### 4.7 `setupMpc()` / `setupMrt()` 细节

```cpp
mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(),
                                leggedInterface_->sqpSettings(),
                                leggedInterface_->getOptimalControlProblem(),
                                leggedInterface_->getInitializer());

rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(
    leggedInterface_->getPinocchioInterface(),
    leggedInterface_->getCentroidalModelInfo());

const std::string robotName = "legged_robot";
auto gaitReceiverPtr =
    std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
rosReferenceManagerPtr->subscribe(nh);

mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
```

```cpp
mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());

controllerRunning_ = true;
mpcThread_ = std::thread([&]() {
  while (controllerRunning_) {
    executeAndSleep([&]() {
      if (mpcRunning_) {
        mpcTimer_.startTimer();
        mpcMrtInterface_->advanceMpc();
        mpcTimer_.endTimer();
      }
    }, mpcDesiredFrequency_);
  }
});
setThreadPriority(sqpSettings.threadPriority, mpcThread_);
```

**`SolverSynchronizedModule`**：`GaitReceiver` 不是常规订阅—回调；它继承自该接口，会在 solver 每次开始求解前被回调 `preSolverRun()`，把回调里收到的步态模板「插」进 `GaitSchedule`，从而下一轮 MPC 的 `modifyReferences` 能看到新步态。

---

## 5. `LeggedInterface` 与最优控制问题装配

文件：`legged_interface/{include,src}/LeggedInterface.*`

### 5.1 类签名

```cpp
class LeggedInterface : public ocs2::RobotInterface {
  ModelSettings    modelSettings_;
  mpc::Settings    mpcSettings_;
  sqp::Settings    sqpSettings_;
  ddp::Settings    ddpSettings_;
  ipm::Settings    ipmSettings_;
  rollout::Settings rolloutSettings_;
  const bool useHardFrictionConeConstraint_;

  std::unique_ptr<PinocchioInterface>          pinocchioInterfacePtr_;
  CentroidalModelInfo                          centroidalModelInfo_;
  std::unique_ptr<PinocchioGeometryInterface>  geometryInterfacePtr_;

  std::unique_ptr<OptimalControlProblem>        problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  std::unique_ptr<RolloutBase>     rolloutPtr_;
  std::unique_ptr<Initializer>     initializerPtr_;

  vector_t initialState_;
};
```

### 5.2 构造函数

- 检查 `taskFile / urdfFile / referenceFile` 是否存在。
- 从 `taskFile` 顶层加载子段：
  ```
  model_settings, mpc, ddp, sqp, ipm, rollout
  ```

### 5.3 `setupOptimalControlProblem(...)`

主流程（按顺序）：

```
setupModel()
  ├── createPinocchioInterface(urdfFile, modelSettings_.jointNames)
  └── createCentroidalModelInfo(pinocchio, centroidalModelType, defaultJointState,
                                contactNames3DoF, contactNames6DoF)

initialState_ ← taskFile.initialState

setupReferenceManager()
  ├── SwingTrajectoryPlanner(swing_trajectory_config, numFeet=4)
  └── referenceManagerPtr_ = SwitchedModelReferenceManager(loadGaitSchedule(referenceFile), swingPlanner)

problemPtr_ = std::make_unique<OptimalControlProblem>();
problemPtr_->dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(pinocchio, info, "dynamics", modelSettings_);

problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, info, verbose));

for i in [0..numThreeDofContacts):
  - friction cone（soft 或 hard） : softConstraintPtr / inequalityConstraintPtr
  - footName + "_zeroForce"      : equalityConstraintPtr  (ZeroForceConstraint)
  - footName + "_zeroVelocity"   : equalityConstraintPtr  (ZeroVelocityConstraintCppAd)
  - footName + "_normalVelocity" : equalityConstraintPtr  (NormalVelocityConstraintCppAd)

problemPtr_->stateSoftConstraintPtr->add("selfCollision",
                                         getSelfCollisionConstraint(pinocchio, taskFile, "selfCollision", verbose));

setupPreComputation()
  └── problemPtr_->preComputationPtr = LeggedRobotPreComputation(pinocchio, info, swingPlanner, modelSettings)

rolloutPtr_     = TimeTriggeredRollout(*dynamicsPtr, rolloutSettings_);
initializerPtr_ = LeggedRobotInitializer(info, *referenceManagerPtr_, extendNormalizedNomentum=true);
```

要点：

- **`centroidalModelType=0`**（`taskFile` 第一行）= FullCentroidalDynamics；改成 1 是 SRBD。
- 所有约束循环写在 `numThreeDofContacts` 上，意味着这套接口假设腿数 = `contactNames3DoF.size()`；改足数要在 `task.info.model_settings.contactNames3DoF` 改名，并保证 URDF 中有同名 frame。
- 步态信息来自 `loadGaitSchedule(referenceFile)`：读 `initialModeSchedule` 与 `defaultModeSequenceTemplate`。

### 5.4 `SwitchedModelReferenceManager`

文件：`legged_interface/src/SwitchedModelReferenceManager.cpp`

```cpp
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& m) {
  ReferenceManager::setModeSchedule(m);
  gaitSchedulePtr_->setModeSchedule(m);    // 同步给 GaitSchedule
}

void SwitchedModelReferenceManager::modifyReferences(
        scalar_t initTime, scalar_t finalTime, const vector_t& initState,
        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  const scalar_t terrainHeight = 0.0;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}
```

每次 solver 调用 `preSolverRun` 后 `modifyReferences` 都会：

1. 从 `gaitSchedule_` 截取一段 mode schedule（覆盖 `[t-Δ, t+Δ]`）。
2. 让 `SwingTrajectoryPlanner` 在 `terrainHeight=0` 上重新规划摆动腿轨迹。

---

## 6. 步态与目标轨迹的 ROS 数据流

### 6.1 `GaitReceiver`

文件：`ocs2_legged_robot_ros/src/gait/GaitReceiver.cpp`

```cpp
class GaitReceiver : public SolverSynchronizedModule {
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  ModeSequenceTemplate          receivedGait_;
  std::atomic_bool              gaitUpdated_;
  std::mutex                    receivedGaitMutex_;
  ros::Subscriber               mpcModeSequenceSubscriber_;
};
```

- 构造：订阅 `<robotName>_mpc_mode_schedule`（默认 TCPROS，方便 rospy 发布端通信）。
- `mpcModeSequenceCallback`：把收到的消息转成 `ModeSequenceTemplate`，置 `gaitUpdated_=true`。
- `preSolverRun(initTime, finalTime, ...)`：若 `gaitUpdated_`，则
  ```cpp
  gaitSchedulePtr_->insertModeSequenceTemplate(receivedGait_, finalTime, finalTime - initTime);
  ```

### 6.2 `GaitKeyboardPublisher`（节点 `legged_robot_gait_command`）

文件：`ocs2_legged_robot_ros/src/gait/GaitKeyboardPublisher.cpp`

```cpp
GaitKeyboardPublisher(nh, gaitFile, robotName, verbose) {
  loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);
  modeSequenceTemplatePublisher_ = nh.advertise<ocs2_msgs::mode_schedule>(
      robotName + "_mpc_mode_schedule", 1, /*latch=*/true);
  for (auto& name : gaitList_)
    gaitMap_[name] = loadModeSequenceTemplate(gaitFile, name, verbose);
}

void getKeyboardCommand() {
  cin >> word;
  if (word == "list") printGaitList(...);
  else if (gaitMap_.count(word))
    modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(gaitMap_.at(word)));
}
```

`main()` 死循环调用 `getKeyboardCommand()`。

### 6.3 `TargetTrajectoriesPublisher`

文件：`legged_controllers/{include,src}/TargetTrajectoriesPublisher.{h,cpp}`

`main()` 流程：

```cpp
nh.getParam("/referenceFile", referenceFile);
nh.getParam("/taskFile",      taskFile);

loadData::loadCppDataType(referenceFile, "comHeight",                  COM_HEIGHT);
loadData::loadEigenMatrix (referenceFile, "defaultJointState",         DEFAULT_JOINT_STATE);
loadData::loadCppDataType(referenceFile, "targetRotationVelocity",     TARGET_ROTATION_VELOCITY);
loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
loadData::loadCppDataType(taskFile,      "mpc.timeHorizon",            TIME_TO_TARGET);

TargetTrajectoriesPublisher target(nh, "legged_robot",
                                   &goalToTargetTrajectories,
                                   &cmdVelToTargetTrajectories);
ros::spin();
```

类内部三种订阅：

| 订阅话题 | 用途 |
|----------|------|
| `legged_robot_mpc_observation` | 缓存 `latestObservation_`（带 mutex） |
| `/move_base_simple/goal` | `goalCallback` → `goalToTargetTrajectories` → `publishTargetTrajectories` |
| `/cmd_vel` | `cmdVelCallback` → `cmdVelToTargetTrajectories` → `publishTargetTrajectories` |

两个转换函数：

```cpp
TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& obs) {
  // 取基座 ZYX，旋转 cmd_vel 到 odom 系
  const Eigen::Matrix<scalar_t,3,1> zyx = obs.state.segment<6>(6).tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);
  scalar_t T = TIME_TO_TARGET;     // = mpc.timeHorizon
  vector_t target(6);
  target << currentPose.head(2) + cmdVelRot.head(2)*T,
            COM_HEIGHT,
            currentPose(3) + cmdVel(3)*T, 0, 0;
  // 两段轨迹（now, now+T），都用 target 但首尾速度等于 cmdVelRot
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& obs) {
  // 估算到达时间 = max(|dyaw|/ωmax, |dxyz|/vmax)
}
```

发布器是 `TargetTrajectoriesRosPublisher`，会向 `legged_robot_mpc_target` 发 `ocs2_msgs/mpc_target_trajectories`。

### 6.4 `RosReferenceManager`

文件：`ocs2_ros_interfaces/src/synchronized_module/RosReferenceManager.cpp`

```cpp
void RosReferenceManager::subscribe(ros::NodeHandle& nh) {
  modeScheduleSubscriber_ = nh.subscribe<ocs2_msgs::mode_schedule>(
      topicPrefix_ + "_mode_schedule", 1,
      [this](const auto& msg){ referenceManagerPtr_->setModeSchedule(readModeScheduleMsg(*msg)); });

  targetTrajectoriesSubscriber_ = nh.subscribe<ocs2_msgs::mpc_target_trajectories>(
      topicPrefix_ + "_mpc_target", 1,
      [this](const auto& msg){ referenceManagerPtr_->setTargetTrajectories(readTargetTrajectoriesMsg(*msg)); });
}
```

注意：

- `<prefix>_mode_schedule` 接收 **整段** schedule（不常用，键盘节点发的是 **模板** `<prefix>_mpc_mode_schedule`，由 `GaitReceiver` 接收）。
- `<prefix>_mpc_target` 是 `TargetTrajectoriesPublisher` 默认发布的话题。

### 6.5 端到端数据流图

```
                                    ┌──────────────────────────────────┐
键盘 │ legged_robot_gait_command ──▶│                                  │
手柄 │ joy_gait_publisher.py ──────▶│ /legged_robot_mpc_mode_schedule  │──▶ GaitReceiver
                                    └──────────────────────────────────┘     (插入 GaitSchedule)

cmd_vel ─▶ TargetTrajectoriesPublisher                                    GaitSchedule
goal    ─▶ (in legged_robot_target node)                                     │
            │                                                                ▼
            └─▶ /legged_robot_mpc_target ─▶ RosReferenceManager ─▶ SwitchedModelReferenceManager
                                                                              │
                                                              modifyReferences│ (preSolverRun)
                                                                              ▼
                                                                          SqpMpc solver
                                                                              │
                                                                  advanceMpc()│ (mpcThread_)
                                                                              ▼
                                                                       MPC_MRT_Interface
                                                                              │
                            updatePolicy + evaluatePolicy   ◀─── LeggedController::update
                                                                              │
                                                                              ▼
                                                                       WeightedWbc::update
                                                                              │
                                                            torque, posDes, velDes
                                                                              ▼
                                                                   HybridJointHandle::setCommand
                                                                              ▼
                                                                       LeggedHWSim::writeSim
                                                                              ▼
                                                                          Gazebo 物理
```

---

## 7. 状态估计：`StateEstimateBase` / `KalmanFilterEstimate`

### 7.1 基类公共逻辑

文件：`legged_estimation/{include,src}/StateEstimateBase.*`

```cpp
class StateEstimateBase {
  PinocchioInterface  pinocchioInterface_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  vector3_t zyxOffset_;
  vector_t  rbdState_;                  // 长度 2 * generalizedCoordinatesNum
  contact_flag_t contactFlag_;
  Eigen::Quaternion<scalar_t> quat_;
  vector3_t angularVelLocal_, linearAccelLocal_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

  // 200Hz odom/pose 发布
  RealtimePublisher<nav_msgs::Odometry>                          odomPub_;
  RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>    posePub_;
};
```

提供的写入接口：

- `updateJointStates(jointPos, jointVel)`：写到 `rbdState_.segment(6,12)` 与 `rbdState_.segment(gCN+6,12)`。
- `updateContact(contactFlag)`：直接覆盖 `contactFlag_`。
- `updateImu(quat, ωlocal, alocal, covs...)`：
  1. ZYX 欧拉 = `quatToZyx(quat) - zyxOffset_`。
  2. ωlocal → ZYX 导数 → ωglobal，写入 `rbdState_.segment(0,3)` 与 `rbdState_.segment(gCN,3)`。
- 纯虚 `vector_t update(time, period)`：子类实现，并返回 `rbdState_`。
- `getMode() = stanceLeg2ModeNumber(contactFlag_)`。

### 7.2 `KalmanFilterEstimate`（默认，仿真就用它）

文件：`legged_estimation/{include,src}/LinearKalmanFilter.*`

状态向量（连续时间近似）：

```
x = [ p_base ∈ R^3 ; v_base ∈ R^3 ; p_foot_i ∈ R^3, i=0..3 ]    // 维度 6 + 12 = 18
u = a_base ∈ R^3                                                 // IMU 加速度
y = [ p_foot - p_base ; v_foot - v_base ; foot_z_height ]        // 24+4 = 28
```

构造时建立 `A,B,C,Q,R,P` 初值并 `subscribe("/tracking_camera/odom/sample")` 用于外部修正（可选）。

`update(time, period)` 关键步骤：

```cpp
// 离散化 a_,b_,q_
a_.block(0,3,3,3) = dt*I;
b_.block(0,0,3,3) = 0.5*dt*dt*I;
b_.block(3,0,3,3) = dt*I;
q_.block(0,0,3,3) = (dt/20)*I;
q_.block(3,3,3,3) = (dt*9.81/20)*I;
q_.block(6,6,12,12) = dt*I;

// 由 IMU 当前姿态 + 关节姿态做 forwardKinematics，得到足端位置 eePos、速度 eeVel
pinocchio::forwardKinematics(model,data, qPino, vPino);
auto eePos = eeKinematics_->getPosition(...);
auto eeVel = eeKinematics_->getVelocity(...);

// 按接触状态膨胀 Q/R：未触地时把对应足端噪声放大 100 倍（"high_suspect_number"）
for (i in feet) {
  if (!isContact[i]) {
    q.block(...) *= 100;   // 过程
    r.block(...) *= 100;   // 观测
  }
  ps_.segment(3i,3) = -eePos[i];  ps_[3i+2] += footRadius_;
  vs_.segment(3i,3) = -eeVel[i];
}

// 卡尔曼一步
vector3_t g(0,0,-9.81);
vector3_t accel = R_zyx * a_local + g;
y << ps_, vs_, feetHeights_;
xHat = A*xHat + B*accel;
pm   = A*P*Aᵀ + Q;
ey   = y - C*xHat;
S    = C*pm*Cᵀ + R;
K    = pm*Cᵀ*S^{-1};
xHat += K*ey;
P    = (I - K*C)*pm;
P    = (P+Pᵀ)/2;

if (topicUpdated_) updateFromTopic();   // 用外部里程计校正

updateLinear(xHat.segment<3>(0), xHat.segment<3>(3));   // 写回 rbdState_
publishMsgs(odom);                                       // 200Hz
return rbdState_;
```

噪声参数从 `taskFile.kalmanFilter` 段读：

```
kalmanFilter {
  footRadius                 0.02
  imuProcessNoisePosition    0.02
  imuProcessNoiseVelocity    0.02
  footProcessNoisePosition   0.002
  footSensorNoisePosition    0.005
  footSensorNoiseVelocity    0.1
  footHeightSensorNoise      0.01
}
```

### 7.3 `FromTopicStateEstimate`（`legged_cheater_controller` 用）

文件：`legged_estimation/{include,src}/FromTopic*.*`

订阅外部话题（一般是 Gazebo 真值或仿真插件）作为浮动基状态，跳过 KF。`LeggedCheaterController::setupStateEstimate` 直接覆盖默认实现。

---

## 8. 全身控制：`WbcBase` / `WeightedWbc`

文件：`legged_wbc/{include,src}/{WbcBase, WeightedWbc, HierarchicalWbc, Task, HoQp}.*`

### 8.1 决策变量

```
x = [ q̈ ∈ R^{gCN=18} ; F ∈ R^{3*numFeet=12} ; τ ∈ R^{nJ=12} ]   // 总维度 42
```

`WbcBase` 中：

```cpp
numDecisionVars_ = info_.generalizedCoordinatesNum
                 + 3*info_.numThreeDofContacts
                 + info_.actuatedDofNum;
```

### 8.2 `update(stateDesired, inputDesired, rbdMeasured, mode, period)`（基类）

1. `contactFlag_ = modeNumber2StanceLeg(mode)`，`numContacts_` 统计触地数。
2. `updateMeasured(rbdMeasured)`：
   - 用测量 RBD 拼成 Pinocchio 用的 `qMeasured_, vMeasured_`（基座 [位置, 欧拉]，关节）。
   - `pinocchio::forwardKinematics + computeJointJacobians + updateFramePlacements + crba + nonLinearEffects`。
   - 得到 **`data.M`**（质量矩阵）、**`data.nle`**（含科里奥利+重力）、**`j_`** 与 **`dj_`**（每足 3 行，世界对齐）。
3. `updateDesired(stateDesired, inputDesired)`：在 `pinocchioInterfaceDesired_` 上做同样的 FK，并 `updateCentroidalDynamics`。

### 8.3 七个 Task 构造器

每个 `Task` 是 `{a, b, d, f}`：等式 `a x = b`，不等式 `d x ≤ f`。

| 函数 | 物理含义 | 矩阵形状 |
|------|----------|----------|
| `formulateFloatingBaseEomTask` | `M q̈ - Jᵀ F - Sᵀ τ + nle = 0` | 等式 18 行 |
| `formulateTorqueLimitsTask` | `‖τ‖_∞ ≤ τ_max`（按 HAA/HFE/KFE 单独） | 不等式 24 行 |
| `formulateNoContactMotionTask` | 触地足端 `J_i q̈ + J̇_i v = 0` | 等式 `3*numContacts_` |
| `formulateFrictionConeTask` | 触地：摩擦锥金字塔；摆动：`F_i = 0` | 等式 `3*(nF-nC)` + 不等式 `5*nC` |
| `formulateBaseAccelTask` | 用 MPC 期望质心动量速率 → 基座 6D 加速度 | 等式 6 行 |
| `formulateSwingLegTask` | 摆动腿 PD：`J q̈ + J̇ v = swingKp*(p_d-p) + swingKd*(v_d-v)` | 等式 `3*(nF-nC)` |
| `formulateContactForceTask` | `F = F_des`（MPC 优化的接触力） | 等式 `3*nF` |

### 8.4 `WeightedWbc::update`（默认，仿真就用它）

```cpp
Task constraints  = formulateFloatingBaseEomTask()
                  + formulateTorqueLimitsTask()
                  + formulateFrictionConeTask()
                  + formulateNoContactMotionTask();

Task weighted = formulateSwingLegTask() * weightSwingLeg_
              + formulateBaseAccelTask(...) * weightBaseAccel_
              + formulateContactForceTask(...) * weightContactForce_;

H = weighted.a^T * weighted.a;
g = -weighted.a^T * weighted.b;

// qpOASES：min ½ xᵀH x + gᵀ x  s.t. lbA ≤ A x ≤ ubA
QProblem qp(numDecisionVars, numConstraints);
qp.init(H, g, A, nullptr, nullptr, lbA, ubA, nWsr=20);
return primalSolution;        // 返回长度 numDecisionVars，控制器取 .tail(12) 作为 τ
```

权重 / 参数（来自 `task.info`）：

```
torqueLimitsTask   { (0,0) 33.5; (1,0) 33.5; (2,0) 33.5 }  ; HAA/HFE/KFE
frictionConeTask   { frictionCoefficient 0.3 }
swingLegTask       { kp 350; kd 37 }
weight             { swingLeg 100; baseAccel 1; contactForce 0.01 }
```

### 8.5 `HierarchicalWbc`（可选）

`HoQp` 用「分层 QP」按优先级求解（高优先级零空间内做低优先级最小化）。`LeggedController.cpp` 默认选 `WeightedWbc`，把 `wbc_ = std::make_shared<HierarchicalWbc>(...)` 即可切换。

---

## 9. 仿真硬件：`LeggedHWSim`

文件：`legged_gazebo/{include,src}/LeggedHWSim.*`

继承 `gazebo_ros_control::DefaultRobotHWSim`，注册为 `RobotHWSim` 插件。

### 9.1 数据成员

```cpp
struct HybridJointData       { JointHandle joint_; double posDes_, velDes_, kp_, kd_, ff_; };
struct HybridJointCommand    { ros::Time stamp_; double posDes_, velDes_, kp_, kd_, ff_; };
struct ImuData               { gazebo::physics::LinkPtr linkPtr_; double ori_[4], oriCov_[9], angularVel_[3], angularVelCov_[9], linearAcc_[3], linearAccCov_[9]; };

HybridJointInterface          hybridJointInterface_;
ImuSensorInterface            imuSensorInterface_;
ContactSensorInterface        contactSensorInterface_;

std::vector<HybridJointData>  hybridJointDatas_;
std::deque<HybridJointCommand> cmdBuffer_[joint_name];   // FIFO，模拟通信延迟
std::vector<ImuData>          imuDatas_;
std::unordered_map<std::string,bool> name2contact_;
gazebo::physics::ContactManager* contactManager_;
double delay_;
```

### 9.2 `initSim`

1. 父类初始化（关节状态、效率、位置/速度控制等）。
2. 注册 `HybridJointInterface`：为每个关节建 `HybridJointData`，给 `HybridJointHandle` 绑定 `posDes_/velDes_/kp_/kd_/ff_` 指针。
3. 从参数 `gazebo/imus` 解析 IMU（`parseImu`）：注册 `ImuSensorHandle`，bind 到对应 link。
4. 从参数 `gazebo/delay` 读延迟，`gazebo/contacts` 解析触地（`parseContacts`）：注册 `ContactSensorHandle` 指向布尔表。
5. `contactManager_->SetNeverDropContacts(true)`，否则 Gazebo GUI 不开 contacts 视图时会丢接触。

### 9.3 `readSim`

- **关节**：用 `position(t) - position(t-1) / Δt` 自算速度（避免父类的 biased 速度）；旋转关节用 `shortest_angular_distance` unwrap。
- **IMU**：用 `linkPtr_->WorldPose()` 提取姿态，`RelativeAngularVel`、`RelativeLinearAccel - R^T*g` 给加速度（去重力）。
- **触地**：先全部置 `false`，再遍历 `contactManager_->GetContacts()`，按时间戳 = `time - period` 过滤，命中 `name2contact_` 的 link 名置 `true`。
- **未挂控制器时**把 hybrid 命令对齐当前位置/速度，`kp=kd=ff=0`，避免乱振。

### 9.4 `writeSim`

```cpp
for joint in hybridJointDatas_:
  buffer = cmdBuffer_[joint.name];
  if simReset: buffer.clear();
  while (!buffer.empty() && buffer.back().stamp + delay < time) buffer.pop_back();
  buffer.push_front({stamp=time, posDes, velDes, kp, kd, ff});
  cmd = buffer.back();      // 拿到 delay 后的指令
  τ_effort = kp*(posDes-pos) + kd*(velDes-vel) + ff;
  joint.setCommand(τ_effort);
DefaultRobotHWSim::writeSim(time, period);
```

控制器侧 `setCommand(posDes, velDes, 0, 3, torque)`，意味着仿真上送的力矩为 `3*(velDes-vel) + torque`。

---

## 10. 真机硬件路径：`LeggedHW` / `LeggedHWLoop` / `UnitreeHW`

仿真不需要这部分，但理解后能把控制器无缝迁到真机。

### 10.1 `LeggedHWLoop`

文件：`legged_hw/src/LeggedHWLoop.cpp`

```cpp
LeggedHWLoop::LeggedHWLoop(nh, hardware_interface) {
  controllerManager_ = ControllerManager(hardwareInterface_, nh);
  nhP.getParam("loop_frequency",            loopHz_);
  nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_);
  nhP.getParam("thread_priority",           threadPriority);
  loopThread_ = std::thread([&]{ while (loopRunning_) update(); });
  pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, {threadPriority});
}

void update() {
  hardwareInterface_->read(time, period);
  controllerManager_->update(time, period);
  hardwareInterface_->write(time, period);
  sleep_until(t0 + 1/loopHz_);
}
```

每秒 `loopHz_` 次（如 500/1000Hz）调一次 `read → controllerManager update（含 LeggedController::update） → write`。

### 10.2 `LeggedHW`、`UnitreeHW`

`legged_hw/src/LeggedHW.cpp` 是抽象基类，提供向 ROS 注册 `HybridJointInterface / ImuSensorInterface / ContactSensorInterface` 的样板。

`UnitreeHW`（`legged_unitree_hw`）：

- 通过 UDP 与 Unitree 控制板通信。
- `read`：从 `LowState` 解析关节角/速、IMU、足底力；用阈值判触地。
- `write`：把 `posDes/velDes/kp/kd/ff` 装到 `LowCmd.motorCmd[i]`，UDP 下发。
- 解析机载手柄并发布到 `/joy` 以及 `/contact`（`std_msgs/Int16MultiArray`）。

**重要**：`LeggedController` 在仿真和真机用的是同一个插件类，因为它都通过 ros_control 的 `HybridJointInterface / ImuSensorInterface / ContactSensorInterface` 与硬件层解耦。

---

## 11. 手柄遥控链路

### 11.1 `joy_teleop.launch`

```xml
<arg name="joy_dev"   default="/dev/input/js0"/>
<arg name="joy_topic" default="/legged_robot/joystick"/>
<arg name="teleop_config" default="$(find legged_controllers)/config/joy.yaml"/>
<arg name="enable_joy_gait" default="false"/>

<node pkg="joy" type="joy_node" name="joy_node">
  <param name="dev" value="$(arg joy_dev)"/>
  <remap from="joy" to="$(arg joy_topic)"/>
</node>

<rosparam file="$(arg teleop_config)" command="load"/>
<node pkg="joy_teleop" type="joy_teleop.py" name="joy_teleop">
  <remap from="joy" to="$(arg joy_topic)"/>
</node>

<include file="$(find legged_controllers)/launch/joy_gait_mux.launch">
  <arg name="enable_joy_gait" value="$(arg enable_joy_gait)"/>
  <arg name="joy_topic"       value="$(arg joy_topic)"/>
</include>
```

### 11.2 `joy.yaml`（cmd_vel 映射）

```yaml
teleop:
  walk:
    type: topic
    message_type: geometry_msgs/Twist
    topic_name: /cmd_vel
    deadman_buttons: [4]
    axis_mappings:
      - { axis: 0, target: angular.z, scale: 3.1415 }
      - { axis: 4, target: linear.x,  scale: 1.0    }
      - { axis: 3, target: linear.y,  scale: 0.8    }
```

按住按键 4（LB 等）时摇杆才会发 `/cmd_vel`。

### 11.3 `joy_gait_publisher.py`（手柄切步态）

文件：`legged_controllers/scripts/joy_gait_publisher.py`

类 `JoyGaitPublisher` 关键流程：

1. **加载步态**：从 `/gaitCommandFile` 路径解析 `list { ... }` 与每个步态块的 `modeSequence` / `switchingTimes`，将 mode 名转成 `MODE_NAME_TO_INT` 中的整数 ID。
2. **加载映射**：私有参数 `~mappings`（由 `joy_gait_mux.launch` 内 `<rosparam file="joy_gait_mappings.yaml">` 加载）：
   ```yaml
   mappings:
     - { gait: trot,   buttons: [5, 2] }
     - { gait: stance, buttons: [5, 3] }
   ```
3. **发布器**：`/legged_robot_mpc_mode_schedule`（`ocs2_msgs/mode_schedule`，`latch=true`）。
4. **订阅 Joy**：`/legged_robot/joystick`，每次回调判断每条 mapping 的 buttons 是否 **全部按下** 且这是从 **非全按** 到 **全按** 的上升沿，并通过 `cooldown_sec=0.25` 抑制抖动。
5. **首次发布兜底**：构造时等 `wait_for_subscribers_sec=2.0` 让 `GaitReceiver` 上线；启动后 `Timer(0.5s)` 监测订阅者数量变化，发现 0→N 时自动补发最后一次步态（解决「先开手柄后启控制器」的时序问题）。
6. **发布动作**：连发 `publish_repeats=3` 次（间隔 `publish_repeat_dt=0.03s`）防丢。

---

## 12. 配置文件字段对照

### 12.1 `controllers.yaml`

```yaml
controllers:
  joint_state_controller:    { type: joint_state_controller/JointStateController, publish_rate: 100 }
  legged_controller:         { type: legged/LeggedController }
  legged_cheater_controller: { type: legged/LeggedCheaterController }
```

控制器名直接对应 `PLUGINLIB_EXPORT_CLASS(legged::LeggedController, ...)`。

### 12.2 `legged_gazebo/config/default.yaml`

```yaml
gazebo:
  delay: 0.009                       # 模拟通信延迟
  imus:                              # 注册 IMU sensor handle
    base_imu:
      frame_id: base_imu             # 必须是 URDF 中存在的 link
      orientation_covariance_diagonal: [...]
      angular_velocity_covariance:     [...]
      linear_acceleration_covariance:  [...]
  contacts: ["LF_FOOT","LH_FOOT","RF_FOOT","RH_FOOT"]   # link 名
```

注意 `contacts` 必须与 `task.info.model_settings.contactNames3DoF` 完全一致。

### 12.3 `<robot>/reference.info`

```
targetDisplacementVelocity  0.5      ; 目标平移速度上限（goal 距离 / 时间用）
targetRotationVelocity      1.57     ; 目标旋转速度上限
comHeight                   0.3      ; 站立时 COM 高度
defaultJointState {  ... 12 个关节初值 ... }
initialModeSchedule { ... }          ; 程序启动时 mode schedule
defaultModeSequenceTemplate { ... }  ; 默认步态（一般是 STANCE）
```

`reference.info` 被这些组件读：

- `LeggedInterface::loadGaitSchedule` 读 `initialModeSchedule` 与 `defaultModeSequenceTemplate` 构造 `GaitSchedule`。
- `LeggedInterface::setupModel` 中 `createCentroidalModelInfo(..., loadDefaultJointState(nq-6, referenceFile), ...)` 使用 `defaultJointState`。
- `TargetTrajectoriesPublisher` 在 `main()` 读 `comHeight, defaultJointState, targetRotationVelocity, targetDisplacementVelocity`。

### 12.4 `<robot>/task.info`

被 `LeggedInterface`、`WbcBase`、`WeightedWbc`、`KalmanFilterEstimate`、`TargetTrajectoriesPublisher` 等多处读取。主要段：

| 段 | 用途 | 读取处 |
|----|------|--------|
| `centroidalModelType` | 0=Full、1=SRBD | `setupModel` |
| `legged_robot_interface.verbose` | 打印开关 | `LeggedInterface` 构造、`LeggedController::init` |
| `model_settings` | 关节名、接触帧名、CppAd 缓存路径 | `loadModelSettings` |
| `swing_trajectory_config` | 摆动腿轨迹形状 | `SwingTrajectoryPlanner` |
| `ddp/sqp/ipm/rollout/mpc` | 求解器参数 | 对应 `loadSettings` |
| `initialState` | 24 维起始状态 | `setupOptimalControlProblem` |
| `Q`、`R` | LQR 加权 | `getBaseTrackingCost` |
| `frictionConeSoftConstraint` | 摩擦锥松弛障 | `loadFrictionConeSettings` |
| `selfCollision` | 自碰链对 | `getSelfCollisionConstraint` |
| `torqueLimitsTask` / `frictionConeTask` / `swingLegTask` / `weight` | WBC 参数 | `WbcBase::loadTasksSetting`、`WeightedWbc::loadTasksSetting` |
| `kalmanFilter` | KF 噪声 | `KalmanFilterEstimate::loadSettings` |

`mpc.timeHorizon`（默认 1.0s）还会被 `TargetTrajectoriesPublisher` 用作 `cmdVel` 转目标的时间窗口 `TIME_TO_TARGET`。

### 12.5 `<robot>/gait.info`

```
list { [0] stance; [1] trot; [2] standing_trot; ... }

trot {
  modeSequence    { [0] LF_RH; [1] RF_LH }
  switchingTimes  { [0] 0.0; [1] 0.3; [2] 0.6 }
}
...
```

被 `GaitKeyboardPublisher` 与 `joy_gait_publisher.py` 解析。模式名字符串必须在 `MotionPhaseDefinition.h` 的 `MODE_NAME_TO_INT` 范围内（`STANCE`、`FLY`、`LF`、`RF`、`LH`、`RH` 及其交集）。

---

## 13. 关键 ROS 话题 / 参数 / 服务总表

`<prefix>` 默认 `legged_robot`，硬编码在三处：`LeggedController::setupMpc()`、`TargetTrajectoriesPublisher::main()`、`LeggedRobotGaitCommandNode::main()`。

### 13.1 话题

| 话题 | 方向 | 类型 | 产生 / 消费 |
|------|------|------|-------------|
| `/legged_robot_mpc_observation` | pub by ctrl | `ocs2_msgs/mpc_observation` | `LeggedController::update` → `TargetTrajectoriesPublisher`、可视化 |
| `/legged_robot_mpc_target` | sub by ctrl | `ocs2_msgs/mpc_target_trajectories` | `TargetTrajectoriesPublisher` → `RosReferenceManager` |
| `/legged_robot_mode_schedule` | sub by ctrl | `ocs2_msgs/mode_schedule` | （整段 schedule，一般外部不用） |
| `/legged_robot_mpc_mode_schedule` | sub by ctrl | `ocs2_msgs/mode_schedule` | `legged_robot_gait_command` / `joy_gait_publisher.py` → `GaitReceiver`（**模板**） |
| `/cmd_vel` | sub | `geometry_msgs/Twist` | `joy_teleop.py` → `TargetTrajectoriesPublisher::cmdVelCallback` |
| `/move_base_simple/goal` | sub | `geometry_msgs/PoseStamped` | RViz "2D Nav Goal" → `goalCallback` |
| `/odom` / `/pose` | pub | `nav_msgs/Odometry` etc. | `StateEstimateBase::publishMsgs`（200Hz） |
| `/tracking_camera/odom/sample` | sub | `nav_msgs/Odometry` | `KalmanFilterEstimate::callback`（外部修正） |
| `/legged_robot/joystick` (=`/joy`) | pub by `joy_node` | `sensor_msgs/Joy` | `joy_teleop.py`、`joy_gait_publisher.py` |
| `/joint_states` | pub | `sensor_msgs/JointState` | `joint_state_controller`（100Hz） |
| `/clock` | pub | `rosgraph_msgs/Clock` | Gazebo |

### 13.2 全局参数（`/...`）

| 参数 | 设置者 | 使用者 |
|------|--------|--------|
| `/urdfFile` | `load_controller.launch` | `LeggedController::init`、`LeggedInterface` |
| `/taskFile` | 同上 | `LeggedController`、`LeggedInterface`、`WBC`、`KF`、`TargetTrajPublisher` |
| `/referenceFile` | 同上 | `LeggedInterface`、`TargetTrajPublisher` |
| `/gaitCommandFile` | 同上 | `legged_robot_gait_command`、`joy_gait_publisher.py` |
| `/legged_robot_description` | `empty_world.launch` | Gazebo `spawn_model`、RViz |
| `gazebo/*` | `default.yaml` | `LeggedHWSim::initSim` |

### 13.3 服务

| 服务 | 用途 |
|------|------|
| `/controller_manager/load_controller` | `load_controller.launch` 已自动调用 |
| `/controller_manager/switch_controller` | **手动** 启动 `controllers/legged_controller` |
| `/controller_manager/list_controllers` | 调试 |

---

## 14. 迁移到其他机器人的修改清单

按依赖强度从浅到深。

### 14.1 仅换四足机器人（关节布局相同）

1. **URDF / xacro**：放到 `legged_unitree_description`（或新建 `<your>_description`）下，确保提供：
   - 12 个关节命名为 `LF_HAA, LF_HFE, LF_KFE, LH_*, RF_*, RH_*`（或修改 `LeggedController::init` 中的 `joint_names`）；
   - 四个足端 link 名与 `task.info.contactNames3DoF` 一致（默认 `LF_FOOT, LH_FOOT, RF_FOOT, RH_FOOT`）；
   - 一个 IMU link 名为 `base_imu`（或改 `LeggedController::init` 中 `imuSensorHandle_` 名与 `default.yaml`）；
   - `<gazebo>` 标签加载 `gazebo_ros_control` 插件并指定 `robotHWSim` 为 `legged/LeggedHWSim`。

2. **`legged_controllers/config/<robot>/{task.info, reference.info, gait.info}`**：
   - 复制现有 `a1` 一份；
   - 在 `task.info` 中改 `initialState`、`Q`、`R`、`torqueLimitsTask`、`frictionCoefficient`、`swingLegTask`、`model_settings.modelFolderCppAd`、`kalmanFilter` 等；
   - 在 `reference.info` 中改 `comHeight`、`defaultJointState`、`targetVelocity`；
   - `gait.info` 一般可复用。

3. **`generate_urdf.sh` 的入参**：保证 `roslaunch ... robot_type:=<name>` 可走到 `legged_unitree_description/urdf/robot.xacro` 选到正确 mesh / inertial。

4. **`legged_gazebo/config/default.yaml`**：若 IMU 或足端 link 名变了，需要同步。

### 14.2 关节命名 / 数量与现有不同

- 改 `LeggedController::init` 的 `joint_names` 列表，或重构为从 `taskFile` 读关节名。
- 若是六足、两足，则要改 `legged_interface` 中循环（按 `numThreeDofContacts` 与 `numSixDofContacts`），`SwingTrajectoryPlanner` 构造时的 `numFeet=4` 也要改。
- `joy_gait_publisher.py` 顶部 `MODE_NAME_TO_INT` 是按四足 4 位编码的，若足数变化需要扩展 / 替换。

### 14.3 真机迁移

- 写新的 `LeggedHW` 派生类（参考 `UnitreeHW`），实现 `init/read/write`：
  - `read`：填 `JointStateInterface` 的 `pos/vel/eff`、`ImuSensorInterface` 的 `quat/ω/a/covs`、`ContactSensorInterface` 的布尔；
  - `write`：把 `HybridJointHandle::getPositionDesired/...` 翻成具体协议（CAN、UDP）。
- 配 `legged_hw` 主循环参数 `loop_frequency`、`cycle_time_error_threshold`、`thread_priority`。
- 状态估计若没有 mocap / 视觉，保留 `KalmanFilterEstimate`；否则可写 `FromTopicStateEstimate` 风格的派生订阅外部 odom 直接覆盖。

### 14.4 命名空间 / 多机

- 当前 `robotName="legged_robot"` 三处硬编码。多机时要么 ROS namespace remap，要么把它做成参数。
- 同理 `MPC_MRT_Interface` 的 `mpcDesiredFrequency_` 来自 `mpcSettings_`，可以在 `task.info.mpc.mpcDesiredFrequency` 改。

---

## 15. 调试与常见问题

### 15.1 「先开手柄，后启动控制器」步态不生效

`/legged_robot_mpc_mode_schedule` 是 `GaitReceiver` 在 `LeggedController::setupMpc()` 内订阅的；若控制器还没 start，订阅者为 0，`latch` 也救不了（rospy 的 latch 行为 + TCPROS 在某些版本下不会回放给后接的 C++ 订阅者）。

仓库的处理：

- `joy_gait_publisher.py` 在 `Timer(0.5s)` 监测订阅者数量 `0 → N`，自动**补发**最后一次步态。
- `GaitReceiver` 构造时强制 TCPROS，保证 rospy 端可达。

如果还不生效：

1. `rostopic info /legged_robot_mpc_mode_schedule` 看是否有 Subscribers；
2. `rosrun rqt_controller_manager rqt_controller_manager` 确认 `legged_controller` 处于 running；
3. 在终端 B 用键盘直接输入 `trot` 确认通路（绕开手柄）。

### 15.2 `Initial policy …` 卡住

`starting()` 里 `while !initialPolicyReceived()` 走不下去，常见原因：

- `task.info` 中 `mpcDesiredFrequency` 或 `sqp.nThreads` 不合理；
- `urdfFile` 路径错（先在 `/tmp/legged_control/<robot>.urdf` 检查）；
- CppAd 模型缓存与机器人不匹配，删除 `model_settings.modelFolderCppAd`（默认 `/tmp/legged_control/<robot>`）后重启。

### 15.3 仿真站不起来 / 立刻摔

- 估计器输出是否合理：`rostopic echo /odom`；
- IMU 名是否一致；
- `gazebo/contacts` 与 `task.info.contactNames3DoF` 是否同名；
- WBC 权重 `weight.swingLeg` 太低、`baseAccel` 太高也会站不稳；
- `torqueLimitsTask` 设置过紧（如 `33.5` 对 A1 是正常的，对其它机器要重新查电机手册）。

### 15.4 关键 benchmark 输出

`LeggedController::~LeggedController()` 析构时打印：

```
### MPC Benchmarking
###   Maximum : .. [ms]
###   Average : .. [ms]
### WBC Benchmarking
###   Maximum : .. [ms]
###   Average : .. [ms]
```

如果 MPC Max > 10ms（对应 100Hz），或 WBC Max > 1ms（对应 1kHz），可能就是性能瓶颈，需要：

- 减少 `sqp.sqpIteration`；
- 检查 CppAd 是否启用了缓存（`recompileLibrariesCppAd: false` 后 modelFolderCppAd 要在）；
- 检查实时优先级是否生效。

### 15.5 看 RViz

- `Fixed Frame=odom`（来自估计器发布的 odom 帧）；
- `RobotModel/Description Source = Parameter`、参数名 `legged_robot_description`；
- 加载仓库提供的 `ocs2_legged_robot_ros/rviz/legged_robot.rviz` 已经预设好质心、轨迹、足端可视化。

---

## 附录 A：阅读顺序建议

1. `empty_world.launch` → `generate_urdf.sh` → `default.yaml`：理解仿真侧有哪些传感器名。
2. `load_controller.launch` → `controllers.yaml`：理解三个节点 + 控制器加载。
3. `LeggedController.cpp` 全文，按 `init → starting → update → setupMpc/Mrt` 顺序。
4. `LeggedInterface.cpp` 中 `setupOptimalControlProblem`，对照 `task.info` 字段。
5. `RosReferenceManager.cpp` + `TargetTrajectoriesPublisher.h` + `GaitReceiver.cpp`：理解 ROS 话题协议。
6. `LeggedHWSim.cpp` 的 `readSim / writeSim`：理解 `HybridJointHandle::setCommand` 与 Gazebo 力矩的关系。
7. `WbcBase.cpp` 的 `formulate*Task` + `WeightedWbc.cpp`：理解 QP 物理含义。
8. `LinearKalmanFilter.cpp`：理解触地融合。
9. （真机）`LeggedHWLoop.cpp` + `UnitreeHW.cpp`。

## 附录 B：术语速查

| 术语 | 含义 |
|------|------|
| **OCP** | Optimal Control Problem |
| **MPC** | Model Predictive Control |
| **MRT** | Model Reference Tracking（OCS2 把求解和评估分离的接口） |
| **WBC** | Whole-Body Controller |
| **RBD** | Rigid-Body Dynamics |
| **CRBA** | Composite Rigid Body Algorithm（算质量阵 M） |
| **NLE** | Non-Linear Effects（含科里奥利+重力的 `nle`） |
| **Centroidal Dynamics** | 以质心动量为变量的浮动基简化模型 |
| **SRBD** | Single Rigid Body Dynamics |
| **HoQp** | Hierarchical QP（分层 QP） |
| **ModeSchedule** | 时间 ↔ mode（触地编码）的序列 |
| **ModeSequenceTemplate** | 周期性的 mode 模板 |
