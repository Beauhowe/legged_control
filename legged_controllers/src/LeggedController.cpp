//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <stdexcept>

namespace legged {
controller_interface::CallbackReturn LeggedController::on_init() {
  auto_declare<std::string>("urdfFile", "");
  auto_declare<std::string>("taskFile", "");
  auto_declare<std::string>("referenceFile", "");
  auto_declare<std::string>("imuName", "base_imu");
  auto_declare<std::string>("emergencyStopTopic", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LeggedController::command_interface_configuration() const {
  std::vector<std::string> interfaces;
  interfaces.reserve(jointNames_.size() * HYBRID_JOINT_COMMAND_INTERFACES.size());
  for (const auto& jointName : jointNames_) {
    for (const auto* interfaceName : HYBRID_JOINT_COMMAND_INTERFACES) {
      interfaces.push_back(makeInterfaceName(jointName, interfaceName));
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces};
}

controller_interface::InterfaceConfiguration LeggedController::state_interface_configuration() const {
  std::vector<std::string> interfaces;
  interfaces.reserve(jointNames_.size() * HYBRID_JOINT_STATE_INTERFACES.size() + contactNames_.size() + 10);
  for (const auto& jointName : jointNames_) {
    for (const auto* interfaceName : HYBRID_JOINT_STATE_INTERFACES) {
      interfaces.push_back(makeInterfaceName(jointName, interfaceName));
    }
  }
  for (const auto& contactName : contactNames_) {
    interfaces.push_back(makeContactInterfaceName(contactName));
  }
  interfaces.push_back(imuName_ + "/orientation.x");
  interfaces.push_back(imuName_ + "/orientation.y");
  interfaces.push_back(imuName_ + "/orientation.z");
  interfaces.push_back(imuName_ + "/orientation.w");
  interfaces.push_back(imuName_ + "/angular_velocity.x");
  interfaces.push_back(imuName_ + "/angular_velocity.y");
  interfaces.push_back(imuName_ + "/angular_velocity.z");
  interfaces.push_back(imuName_ + "/linear_acceleration.x");
  interfaces.push_back(imuName_ + "/linear_acceleration.y");
  interfaces.push_back(imuName_ + "/linear_acceleration.z");
  return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces};
}

controller_interface::CallbackReturn LeggedController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  // Guard against re-entry: on_configure must not be called while a previous call is still running
  // (e.g. spawner retrying due to CppAD compile timeout). rosNode_ is set early in this function,
  // so its presence means configure is already in progress or completed.
  if (rosNode_) {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  // Initialize OCS2
  const auto node = get_node();
  const auto urdfFile = node->get_parameter("urdfFile").as_string();
  const auto taskFile = node->get_parameter("taskFile").as_string();
  const auto referenceFile = node->get_parameter("referenceFile").as_string();
  imuName_ = node->get_parameter("imuName").as_string();
  if (urdfFile.empty() || taskFile.empty() || referenceFile.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameters urdfFile, taskFile and referenceFile must be set.");
    return controller_interface::CallbackReturn::ERROR;
  }

  rosNode_ = std::make_shared<rclcpp::Node>(node->get_name() + std::string("_ros"));
  auto emergencyStopTopic = node->get_parameter("emergencyStopTopic").as_string();
  if (!emergencyStopTopic.empty()) {
    if (emergencyStopTopic.front() != '/') {
      emergencyStopTopic = "/" + emergencyStopTopic;
    }
    emergencyStopSubscriber_ = rosNode_->create_subscription<std_msgs::msg::Bool>(
        emergencyStopTopic, rclcpp::QoS(1).reliable().transient_local(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
          emergencyStopActive_.store(msg->data);
          if (!msg->data) {
            emergencyStopLogged_ = false;
          }
        });
    RCLCPP_INFO(rosNode_->get_logger(), "Controller emergency stop topic: %s", emergencyStopTopic.c_str());
  }

  // 固定姿态命令订阅：上游(joy_control / DDS 桥)对 stance/lie_down 发布 Int8 而非 mode_schedule。
  // 1 = STAND, 2 = LIE_DOWN, 0 = 解除(回到运动/空闲)。QoS 与急停一致(latched)，订阅者后启动也能收到最近一次命令。
  postureCommandSubscriber_ = rosNode_->create_subscription<std_msgs::msg::Int8>(
      "/" + std::string("legged_robot") + "_posture_command", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Int8::SharedPtr msg) { postureRequest_.store(msg->data); });
  RCLCPP_INFO(rosNode_->get_logger(), "Controller posture command topic: /legged_robot_posture_command");

  rosExecutor_.add_node(rosNode_);
  rosSpinThread_ = std::thread([this]() { rosExecutor_.spin(); });

  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();

  // 加载固定姿态关节角与位控增益。站立角复用 defaultJointState，趴下角用 lieDownJointState。
  // loadEigenMatrix 不会自动 resize，需按执行器自由度数预设尺寸，否则会抛 "empty matrix"。
  // 这些字段仅 p1 reference.info 有；其它机型缺失时用默认值/退化处理，避免 configure 失败。
  const size_t actuatedDofNum = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;
  standJointState_.setZero(actuatedDofNum);
  lieDownJointState_.setZero(actuatedDofNum);
  postureStartJointState_.setZero(actuatedDofNum);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", standJointState_);
  try {
    loadData::loadEigenMatrix(referenceFile, "lieDownJointState", lieDownJointState_);
  } catch (const std::exception&) {
    lieDownJointState_ = standJointState_;  // 缺失趴下角时退化为站立角，lie_down 等同 stance
    RCLCPP_WARN(rosNode_->get_logger(), "lieDownJointState not found in reference file; lie_down will fall back to stand posture.");
  }
  try {
    loadData::loadCppDataType(referenceFile, "postureKp", postureKp_);
    loadData::loadCppDataType(referenceFile, "postureKd", postureKd_);
  } catch (const std::exception&) {
    RCLCPP_WARN(rosNode_->get_logger(), "postureKp/postureKd not found; using defaults kp=%.1f kd=%.1f.", postureKp_, postureKd_);
  }
  try {
    loadData::loadCppDataType(referenceFile, "postureTransitionDuration", postureTransitionDuration_);
  } catch (const std::exception&) {
    RCLCPP_WARN(rosNode_->get_logger(), "postureTransitionDuration not found; using default %.2fs.", postureTransitionDuration_);
  }
  RCLCPP_INFO(rosNode_->get_logger(), "Posture position control: kp=%.1f kd=%.1f transition=%.2fs", postureKp_, postureKd_,
              postureTransitionDuration_);

  // Visualization
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, rosNode_);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping));

  jointNames_ = {"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                 "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  contactNames_ = leggedInterface_->modelSettings().contactNames3DoF;

  // State estimation
  setupStateEstimate(taskFile, verbose);

  mrtPeriod_ = 1.0 / leggedInterface_->mpcSettings().mrtDesiredFrequency_;
  RCLCPP_INFO(rosNode_->get_logger(), "MRT period %.6f s (mrtDesiredFrequency=%.1f Hz). Match controller_manager update_rate.",
              mrtPeriod_, leggedInterface_->mpcSettings().mrtDesiredFrequency_);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  starting(get_node()->now());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  mpcRunning_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

void LeggedController::starting(const rclcpp::Time& time) {
  controllerStartTime_ = time;
  controllerClockStarted_ = true;
  mrtPeriodMismatchWarned_ = false;

  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, rclcpp::Duration::from_seconds(mrtPeriod_));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  // 不在激活时强制优化。保持空闲（关节零指令），直到收到运动步态命令。
  mpcIdle_.store(true);
  mpcRunning_ = false;
  motionGaitRequested_.store(false);
  // 复位固定姿态状态，避免上次会话遗留(latched 话题)误触发。
  postureActive_ = false;
  activePosture_ = static_cast<int>(PostureCommand::NONE);
  postureRequest_.store(static_cast<int>(PostureCommand::NONE));
  lastHandledMotionSeq_ = motionGaitSeq_.load();
  RCLCPP_INFO(rosNode_->get_logger(),
              "Controller activated in IDLE. MPC will start after a locomotion gait (with swing phase) is received.");
}

void LeggedController::startMpcOptimization(const rclcpp::Time& time) {
  // 非阻塞启动：以当前观测为初值，放行后台 MPC 线程开始优化。
  // 首份策略就绪前 update() 仍输出零指令，不阻塞实时循环。
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  RCLCPP_INFO(rosNode_->get_logger(), "Locomotion gait received. Starting MPC optimization ...");

  mpcRunning_ = true;   // 放行后台 MPC 线程(setupMrt 中创建)开始 advanceMpc
  mpcIdle_.store(false);
}

void LeggedController::resyncMpcAfterEmergencyStop() {
  mpcAdvancePaused_.store(true);

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
  mpcMrtInterface_->reset();
  mpcMrtInterface_->resetMpcNode(target_trajectories);
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  RCLCPP_WARN(rosNode_->get_logger(), "Emergency stop released, resynchronizing MPC at t=%.3f ...", currentObservation_.time);
  rclcpp::Rate rate(leggedInterface_->mpcSettings().mrtDesiredFrequency_);
  const int maxIterations = std::max(1, static_cast<int>(5.0 * leggedInterface_->mpcSettings().mrtDesiredFrequency_));
  int iterations = 0;
  while (!mpcMrtInterface_->initialPolicyReceived() && rclcpp::ok() && iterations < maxIterations) {
    mpcMrtInterface_->advanceMpc();
    ++iterations;
    rate.sleep();
  }

  if (!mpcMrtInterface_->initialPolicyReceived()) {
    RCLCPP_ERROR(rosNode_->get_logger(), "MPC resync timed out after emergency stop (t=%.3f).", currentObservation_.time);
  } else {
    RCLCPP_INFO(rosNode_->get_logger(), "MPC resynchronized after emergency stop (t=%.3f).", currentObservation_.time);
  }

  mpcAdvancePaused_.store(false);
}

controller_interface::return_type LeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  const scalar_t periodSec = period.seconds();
  if (!mrtPeriodMismatchWarned_ && periodSec > 0.0 &&
      std::abs(periodSec - mrtPeriod_) > 0.05 * mrtPeriod_) {
    RCLCPP_WARN(rosNode_->get_logger(),
                "Controller period %.4f s differs from task.info mrtDesiredFrequency (%.4f s, %.0f Hz). "
                "In-place gaits may drift; set controller_manager update_rate to %.0f Hz.",
                periodSec, mrtPeriod_, 1.0 / mrtPeriod_, 1.0 / mrtPeriod_);
    mrtPeriodMismatchWarned_ = true;
  }

  // State Estimate
  updateStateEstimation(time, period);

  const bool emergencyStop = emergencyStopActive_.load();
  mpcAdvancePaused_.store(emergencyStop);

  if (previousEmergencyStopActive_ && !emergencyStop) {
    resyncMpcAfterEmergencyStop();
  }
  previousEmergencyStopActive_ = emergencyStop;

  if (emergencyStop) {
    if (!emergencyStopLogged_) {
      RCLCPP_ERROR(rosNode_->get_logger(), "[Legged Controller] Emergency stop active, zeroing all joint commands.");
      emergencyStopLogged_ = true;
    }
    for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
      setHybridJointCommand(j, 0, 0, 0, 0, 0);
    }
    return controller_interface::return_type::OK;
  }

  // ---- 固定姿态处理（stance / lie_down 绕过 MPC，直接关节位控）----
  // 上游对 stance/lie_down 发布 Int8 姿态命令(1=STAND,2=LIE_DOWN,0=解除)，对运动步态发布 mode_schedule。
  // 每收到一次 mode_schedule，motionGaitSeq_ 自增 —— 以此作为“退出固定姿态、恢复运动”的边沿信号。
  const int postureReq = postureRequest_.load();
  const int currentMotionSeq = motionGaitSeq_.load();
  const bool newLocomotionRequested = (currentMotionSeq != lastHandledMotionSeq_);

  if (newLocomotionRequested) {
    if (postureActive_) {
      postureActive_ = false;
      activePosture_ = static_cast<int>(PostureCommand::NONE);
      postureRequest_.store(static_cast<int>(PostureCommand::NONE));
      RCLCPP_INFO(rosNode_->get_logger(), "Exiting fixed posture, resuming locomotion (MPC).");
      // MPC 曾运行过(非空闲)才需重新同步到当前观测，避免姿态期间策略冻结导致的跳变。
      // 若仍空闲(首个命令就是运动步态前曾处于姿态)，交由下方 mpcIdle_ 分支正常启动 MPC。
      if (!mpcIdle_.load()) {
        resyncMpcAfterEmergencyStop();
      }
    }
    lastHandledMotionSeq_ = currentMotionSeq;
  }

  if (postureReq != static_cast<int>(PostureCommand::NONE) && !newLocomotionRequested) {
    const vector_t& qTarget = (postureReq == static_cast<int>(PostureCommand::LIE_DOWN)) ? lieDownJointState_ : standJointState_;
    if (!postureActive_ || activePosture_ != postureReq) {
      // 进入(或切换)姿态：以"当前实测关节角"为插值起点，重置过渡计时。
      // 关键防弹飞措施：不再一拍把 qDes 跳到目标角，而是在 postureTransitionDuration_ 内平滑插值，
      // 每拍位置误差很小 -> kp*误差 的力矩平顺。
      postureActive_ = true;
      activePosture_ = postureReq;
      postureElapsed_ = 0.0;
      postureStartJointState_ = centroidal_model::getJointAngles(currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
      RCLCPP_INFO(rosNode_->get_logger(), "Entering fixed posture: %s (bypassing MPC, %.2fs ramp).",
                  postureReq == static_cast<int>(PostureCommand::LIE_DOWN) ? "LIE_DOWN" : "STAND", postureTransitionDuration_);
    }
    // 暂停后台 MPC 优化(每周期覆盖上方按急停设置的值)，直接对 12 关节做插值位控。
    mpcAdvancePaused_.store(true);
    postureElapsed_ += periodSec;

    // smoothstep 缓动 s(a) = 3a^2 - 2a^3，首尾速度为 0，避免起止冲击。
    scalar_t alpha = (postureTransitionDuration_ > 1e-6) ? (postureElapsed_ / postureTransitionDuration_) : 1.0;
    alpha = std::min(std::max(alpha, 0.0), 1.0);
    const scalar_t s = alpha * alpha * (3.0 - 2.0 * alpha);
    // 缓动相对时间的导数 ds/dt，用于前馈关节速度，让 kd 顺着运动而非阻碍。
    const scalar_t dsdt = (postureTransitionDuration_ > 1e-6) ? (6.0 * alpha * (1.0 - alpha) / postureTransitionDuration_) : 0.0;

    for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
      const scalar_t delta = qTarget(j) - postureStartJointState_(j);
      const scalar_t posDes = postureStartJointState_(j) + s * delta;
      const scalar_t velDes = dsdt * delta;
      setHybridJointCommand(j, posDes, velDes, postureKp_, postureKd_, 0);
    }
    observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    return controller_interface::return_type::OK;
  }


  if (mpcIdle_.load()) {
    if (motionGaitRequested_.load()) {
      startMpcOptimization(time);
      // startMpcOptimization 已将 mpcIdle_ 置 false，继续执行下方正常路径
    } else {
      for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
        setHybridJointCommand(j, 0, 0, 0, 0, 0);
      }
      observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));
      return controller_interface::return_type::OK;
    }
  }

  // 初始策略未就绪时（MPC 后台线程刚启动但尚未完成首次求解），继续输出零指令。
  if (!mpcMrtInterface_->initialPolicyReceived()) {
    for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
      setHybridJointCommand(j, 0, 0, 0, 0, 0);
    }
    observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    return controller_interface::return_type::OK;
  }

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, periodSec);
  wbcTimer_.endTimer();

  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    RCLCPP_ERROR(rosNode_->get_logger(), "[Legged Controller] Safety check failed, stopping the controller.");
    mpcRunning_ = false;
    return controller_interface::return_type::ERROR;
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    setHybridJointCommand(j, posDes(j), velDes(j), 270, 17, torque(j));
  }

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));
  return controller_interface::return_type::OK;
}

void LeggedController::updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period) {
  vector_t jointPos(jointNames_.size()), jointVel(jointNames_.size());
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < jointNames_.size(); ++i) {
    jointPos(i) = state_interfaces_[i * HYBRID_JOINT_STATE_INTERFACES.size()].get_value();
    jointVel(i) = state_interfaces_[i * HYBRID_JOINT_STATE_INTERFACES.size() + 1].get_value();
  }
  const size_t contactOffset = jointNames_.size() * HYBRID_JOINT_STATE_INTERFACES.size();
  for (size_t i = 0; i < contactFlag.size(); ++i) {
    contactFlag[i] = state_interfaces_[contactOffset + i].get_value() > 0.5;
  }
  quat.x() = getStateInterfaceValue(imuName_ + "/orientation.x");
  quat.y() = getStateInterfaceValue(imuName_ + "/orientation.y");
  quat.z() = getStateInterfaceValue(imuName_ + "/orientation.z");
  quat.w() = getStateInterfaceValue(imuName_ + "/orientation.w");
  angularVel.x() = getStateInterfaceValue(imuName_ + "/angular_velocity.x");
  angularVel.y() = getStateInterfaceValue(imuName_ + "/angular_velocity.y");
  angularVel.z() = getStateInterfaceValue(imuName_ + "/angular_velocity.z");
  linearAccel.x() = getStateInterfaceValue(imuName_ + "/linear_acceleration.x");
  linearAccel.y() = getStateInterfaceValue(imuName_ + "/linear_acceleration.y");
  linearAccel.z() = getStateInterfaceValue(imuName_ + "/linear_acceleration.z");
  orientationCovariance.setZero();
  angularVelCovariance.setZero();
  linearAccelCovariance.setZero();

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  if (controllerClockStarted_) {
    currentObservation_.time = (time - controllerStartTime_).seconds();
  } else {
    currentObservation_.time += period.seconds();
  }
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

double LeggedController::getStateInterfaceValue(const std::string& interfaceName) const {
  for (const auto& stateInterface : state_interfaces_) {
    if (stateInterface.get_name() == interfaceName) {
      return stateInterface.get_value();
    }
  }
  throw std::runtime_error("State interface not found: " + interfaceName);
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  rosExecutor_.cancel();
  if (rosSpinThread_.joinable()) {
    rosSpinThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(rosNode_, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(rosNode_);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = rosNode_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);

  // 监听步态命令：收到任意步态命令(含 stance/lie_down)即请求启动 MPC 优化。
  // 与 GaitReceiver 订阅同一话题。
  motionGaitSubscriber_ = rosNode_->create_subscription<ocs2_msgs::msg::ModeSchedule>(
      robotName + "_mpc_mode_schedule", 1,
      [this](const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr /*msg*/) {
        motionGaitRequested_.store(true);
        motionGaitSeq_.fetch_add(1);
      });
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_ && !mpcAdvancePaused_.load()) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR(rosNode_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
        mpcRunning_ = false;
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(rosNode_, leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(rosNode_, leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

void LeggedController::setHybridJointCommand(size_t jointIndex, scalar_t posDes, scalar_t velDes, scalar_t kp, scalar_t kd, scalar_t ff) {
  const size_t offset = jointIndex * HYBRID_JOINT_COMMAND_INTERFACES.size();
  command_interfaces_[offset + 0].set_value(posDes);
  command_interfaces_[offset + 1].set_value(velDes);
  command_interfaces_[offset + 2].set_value(kp);
  command_interfaces_[offset + 3].set_value(kd);
  command_interfaces_[offset + 4].set_value(ff);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerInterface)
