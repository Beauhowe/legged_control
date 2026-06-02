#include "legged_p1_hw/P1HW.h"

#include "legged_p1_hw/P1DdsInterface.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace legged {
namespace {
constexpr size_t kJointCount = 12;  // P1 四条腿，每条腿 3 个关节。
constexpr size_t kLegCount = 4;
std::string trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\n\r");
  return value.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCommaSeparated(const std::string& value) {
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    result.push_back(trim(item));
  }
  return result;
}
}  // namespace

int P1HW::readIntParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name, int defaultValue) {
  const auto it = hardwareInfo.hardware_parameters.find(name);
  return it == hardwareInfo.hardware_parameters.end() ? defaultValue : std::stoi(it->second);
}

double P1HW::readDoubleParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name, double defaultValue) {
  const auto it = hardwareInfo.hardware_parameters.find(name);
  return it == hardwareInfo.hardware_parameters.end() ? defaultValue : std::stod(it->second);
}

std::string P1HW::readStringParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name,
                                      const std::string& defaultValue) {
  const auto it = hardwareInfo.hardware_parameters.find(name);
  return it == hardwareInfo.hardware_parameters.end() ? defaultValue : it->second;
}

P1HW::~P1HW() {
  emergencyStopExecutor_.cancel();
  if (emergencyStopSpinThread_.joinable()) {
    emergencyStopSpinThread_.join();
  }
}

int P1HW::findJointIndex(const std::string& jointName) const {
  const auto it = std::find(jointNames_.begin(), jointNames_.end(), jointName);
  if (it == jointNames_.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(jointNames_.begin(), it));
}

bool P1HW::loadJointMappingParameters(const hardware_interface::HardwareInfo& hardwareInfo) {
  for (size_t i = 0; i < kJointCount; ++i) {
    jointToDdsIndex_[i] = i;
  }

  const auto ddsJointOrder = readStringParameter(hardwareInfo, "dds_joint_order", "");
  if (!ddsJointOrder.empty()) {
    const auto names = splitCommaSeparated(ddsJointOrder);
    if (names.size() != kJointCount) {
      RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "dds_joint_order must contain 12 joint names, got %zu", names.size());
      return false;
    }

    std::array<bool, kJointCount> seen{};
    for (size_t ddsIndex = 0; ddsIndex < names.size(); ++ddsIndex) {
      const int jointIndex = findJointIndex(names[ddsIndex]);
      if (jointIndex < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Unknown joint in dds_joint_order: %s", names[ddsIndex].c_str());
        return false;
      }
      if (seen[static_cast<size_t>(jointIndex)]) {
        RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Duplicate joint in dds_joint_order: %s", names[ddsIndex].c_str());
        return false;
      }
      seen[static_cast<size_t>(jointIndex)] = true;
      jointToDdsIndex_[static_cast<size_t>(jointIndex)] = ddsIndex;
    }
  }

  for (size_t i = 0; i < kJointCount; ++i) {
    const int mappedByIndex = readIntParameter(hardwareInfo, "dds_joint_index_" + std::to_string(i), static_cast<int>(jointToDdsIndex_[i]));
    const int mappedByName = readIntParameter(hardwareInfo, "dds_joint_index_" + jointNames_[i], mappedByIndex);
    if (mappedByName < 0 || mappedByName >= static_cast<int>(kJointCount)) {
      RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Invalid DDS joint index %d for %s", mappedByName, jointNames_[i].c_str());
      return false;
    }
    jointToDdsIndex_[i] = static_cast<size_t>(mappedByName);
  }

  std::array<bool, kJointCount> used{};
  for (size_t i = 0; i < kJointCount; ++i) {
    const size_t ddsIndex = jointToDdsIndex_[i];
    if (ddsIndex >= kJointCount) {
      RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Invalid final DDS joint index %zu for %s", ddsIndex,
                   jointNames_[i].c_str());
      return false;
    }
    if (used[ddsIndex]) {
      RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Duplicate final DDS joint index %zu, caused by joint %s", ddsIndex,
                   jointNames_[i].c_str());
      return false;
    }
    used[ddsIndex] = true;
  }

  return true;
}

void P1HW::loadTorqueEstimationParameters(const hardware_interface::HardwareInfo& hardwareInfo) {
  const double defaultScale = readDoubleParameter(hardwareInfo, "current_to_torque_scale", 1.0);
  const double defaultOffset = readDoubleParameter(hardwareInfo, "current_to_torque_offset", 0.0);
  currentToTorqueScale_.fill(defaultScale);
  currentToTorqueOffset_.fill(defaultOffset);
  for (size_t i = 0; i < kJointCount; ++i) {
    // 可以按索引覆盖：current_to_torque_scale_0，也可以按关节名覆盖：current_to_torque_scale_LF_HAA。
    currentToTorqueScale_[i] = readDoubleParameter(hardwareInfo, "current_to_torque_scale_" + std::to_string(i), currentToTorqueScale_[i]);
    currentToTorqueOffset_[i] = readDoubleParameter(hardwareInfo, "current_to_torque_offset_" + std::to_string(i), currentToTorqueOffset_[i]);
    currentToTorqueScale_[i] =
        readDoubleParameter(hardwareInfo, "current_to_torque_scale_" + jointNames_[i], currentToTorqueScale_[i]);
    currentToTorqueOffset_[i] =
        readDoubleParameter(hardwareInfo, "current_to_torque_offset_" + jointNames_[i], currentToTorqueOffset_[i]);
  }
}

void P1HW::loadContactEstimationParameters(const hardware_interface::HardwareInfo& hardwareInfo) {
  const auto method = readStringParameter(hardwareInfo, "contact_estimation_method", "current");
  useJacobianContactEstimation_ = method == "jacobian";
  contactForceThreshold_ = readDoubleParameter(hardwareInfo, "contact_force_threshold", static_cast<double>(contactThreshold_));
}

void P1HW::loadCommandProtectionParameters(const hardware_interface::HardwareInfo& hardwareInfo) {
  feedforwardTorqueSlewRate_ = readDoubleParameter(hardwareInfo, "feedforward_torque_slew_rate", feedforwardTorqueSlewRate_);
  maxFeedforwardTorque_ = readDoubleParameter(hardwareInfo, "max_feedforward_torque", maxFeedforwardTorque_);
}

double P1HW::estimateJointTorque(size_t jointIndex, double current) const {
  return current * currentToTorqueScale_[jointIndex] + currentToTorqueOffset_[jointIndex];
}

double P1HW::limitFeedforwardTorque(size_t jointIndex, double desiredTorque, double periodSeconds) {
  if (!std::isfinite(desiredTorque)) {
    desiredTorque = 0.0;
  }
  if (maxFeedforwardTorque_ > 0.0) {
    desiredTorque = std::clamp(desiredTorque, -maxFeedforwardTorque_, maxFeedforwardTorque_);
  }
  if (feedforwardTorqueSlewRate_ <= 0.0 || periodSeconds <= 0.0) {
    lastFeedforwardTorque_[jointIndex] = desiredTorque;
    return desiredTorque;
  }

  const double previousTorque = hasLastFeedforwardTorque_ ? lastFeedforwardTorque_[jointIndex] : 0.0;
  const double maxStep = feedforwardTorqueSlewRate_ * periodSeconds;
  const double limitedTorque = previousTorque + std::clamp(desiredTorque - previousTorque, -maxStep, maxStep);
  lastFeedforwardTorque_[jointIndex] = limitedTorque;
  return limitedTorque;
}

void P1HW::setupEmergencyStop(const hardware_interface::HardwareInfo& hardwareInfo) {
  auto topic = readStringParameter(hardwareInfo, "emergency_stop_topic", "/p1_emergency_stop");
  if (!topic.empty() && topic.front() != '/') {
    topic = "/" + topic;
  }

  emergencyStopNode_ = std::make_shared<rclcpp::Node>("legged_p1_hw_emergency_stop");
  emergencyStopSubscriber_ = emergencyStopNode_->create_subscription<std_msgs::msg::Bool>(
      topic, rclcpp::QoS(1).reliable().transient_local(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
        emergencyStopActive_.store(msg->data);
        if (!msg->data) {
          emergencyStopLogged_ = false;
        }
      });
  emergencyStopExecutor_.add_node(emergencyStopNode_);
  emergencyStopSpinThread_ = std::thread([this]() { emergencyStopExecutor_.spin(); });
  RCLCPP_INFO(rclcpp::get_logger("legged_p1_hw"), "P1 hardware emergency stop topic: %s", topic.c_str());
}

// 初始化 ros2_control 通用接口后，再创建 P1 使用的 DDS 读写通道。
hardware_interface::CallbackReturn P1HW::on_init(const hardware_interface::HardwareInfo& hardwareInfo) {
  if (LeggedHW::on_init(hardwareInfo) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (jointNames_.size() != kJointCount) {
    RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Expected 12 joints, got %zu", jointNames_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto domain = static_cast<uint32_t>(readIntParameter(hardwareInfo, "dds_domain", 0));
  const auto stateTopic = readStringParameter(hardwareInfo, "dds_state_topic", "p1_motor_state");
  const auto imuTopic = readStringParameter(hardwareInfo, "dds_imu_topic", "p1_imu");
  const auto commandTopic = readStringParameter(hardwareInfo, "dds_command_topic", "p1_motor_cmd");

  contactThreshold_ = readIntParameter(hardwareInfo, "contact_threshold", 0);
  commandMode_ = static_cast<uint32_t>(readIntParameter(hardwareInfo, "command_mode", 10));
  if (!loadJointMappingParameters(hardwareInfo)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  loadTorqueEstimationParameters(hardwareInfo);
  loadContactEstimationParameters(hardwareInfo);
  loadCommandProtectionParameters(hardwareInfo);
  setupEmergencyStop(hardwareInfo);
  dds_ = std::make_unique<P1DdsInterface>();
  if (!dds_->init(domain, stateTopic, imuTopic, commandTopic)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// LeggedHW 已导出关节和足端接触，这里补充 base_imu 的状态接口。
std::vector<hardware_interface::StateInterface> P1HW::export_state_interfaces() {
  auto stateInterfaces = LeggedHW::export_state_interfaces();

  stateInterfaces.emplace_back("base_imu", "orientation.x", &imuOrientation_[0]);
  stateInterfaces.emplace_back("base_imu", "orientation.y", &imuOrientation_[1]);
  stateInterfaces.emplace_back("base_imu", "orientation.z", &imuOrientation_[2]);
  stateInterfaces.emplace_back("base_imu", "orientation.w", &imuOrientation_[3]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.x", &imuAngularVelocity_[0]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.y", &imuAngularVelocity_[1]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.z", &imuAngularVelocity_[2]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.x", &imuLinearAcceleration_[0]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.y", &imuLinearAcceleration_[1]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.z", &imuLinearAcceleration_[2]);

  return stateInterfaces;
}

// controller_manager 周期调用：把 DDS 缓存状态同步到 ros2_control state interfaces。
hardware_interface::return_type P1HW::read(const rclcpp::Time&, const rclcpp::Duration&) {
  if (dds_ == nullptr) {
    return hardware_interface::return_type::ERROR;
  }

  P1DdsInterface::MotorState state;
  if (dds_->getLatestMotorState(state)) {
    for (size_t i = 0; i < kJointCount; ++i) {
      const size_t ddsIndex = jointToDdsIndex_[i];
      jointStates_[i].position = state.position[ddsIndex];
      jointStates_[i].velocity = state.speed[ddsIndex];
      // jointStates_[i].effort = estimateJointTorque(i, state.current[ddsIndex]);
      jointStates_[i].effort = state.current[ddsIndex]; // current 实际是力矩
    }

    for (size_t i = 0; i < contactStates_.size() && i < kLegCount; ++i) {
      const size_t firstJoint = i * 3;
      bool online = false;
      for (size_t j = 0; j < 3 && firstJoint + j < kJointCount; ++j) {
        online = online || state.is_online[jointToDdsIndex_[firstJoint + j]];
      }

      bool overThreshold = false;
      if (useJacobianContactEstimation_) {
        const std::array<double, 3> jointPosition{jointStates_[firstJoint].position, jointStates_[firstJoint + 1].position,
                                                  jointStates_[firstJoint + 2].position};
        const std::array<double, 3> jointTorque{jointStates_[firstJoint].effort, jointStates_[firstJoint + 1].effort,
                                                jointStates_[firstJoint + 2].effort};
        const auto footForce = contactEstimator_.estimateFootForce(i, jointPosition, jointTorque);
        overThreshold = std::abs(footForce.z()) > contactForceThreshold_;
      } else {
        for (size_t j = 0; j < 3 && firstJoint + j < kJointCount; ++j) {
          overThreshold = overThreshold || std::abs(state.current[jointToDdsIndex_[firstJoint + j]]) > contactThreshold_;
        }
      }
      contactStates_[i] = online && overThreshold ? 1.0 : 0.0;
    }
  }

  P1DdsInterface::ImuState imu;
  if (dds_->getLatestImuState(imu)) {
    imuOrientation_[0] = imu.orientation_x;
    imuOrientation_[1] = imu.orientation_y;
    imuOrientation_[2] = imu.orientation_z;
    imuOrientation_[3] = imu.orientation_w;
    imuAngularVelocity_[0] = imu.angular_velocity_x;
    imuAngularVelocity_[1] = imu.angular_velocity_y;
    imuAngularVelocity_[2] = imu.angular_velocity_z;
    imuLinearAcceleration_[0] = imu.linear_acceleration_x;
    imuLinearAcceleration_[1] = imu.linear_acceleration_y;
    imuLinearAcceleration_[2] = imu.linear_acceleration_z;
  }

  for (auto& command : jointCommands_) {
    command.feedforward = 0.0;
    command.velocity_desired = 0.0;
    command.kd = 3.0;
  }

  return hardware_interface::return_type::OK;
}

// controller_manager 周期调用：把控制器输出打包成 Motor_Command_12::motor_cmd 发给下位机。
hardware_interface::return_type P1HW::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (dds_ == nullptr) {
    return hardware_interface::return_type::ERROR;
  }

  P1DdsInterface::Command command;
  (void)time;
  const bool emergencyStop = emergencyStopActive_.load();
  if (emergencyStop && !emergencyStopLogged_) {
    RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Emergency stop active, zeroing outgoing motor command.");
    emergencyStopLogged_ = true;
  }
  for (size_t i = 0; i < kJointCount; ++i) {
    const size_t ddsIndex = jointToDdsIndex_[i];
    if (!emergencyStop) {
      command.position_desired[ddsIndex] = static_cast<float>(jointCommands_[i].position_desired);
      command.velocity_desired[ddsIndex] = static_cast<float>(jointCommands_[i].velocity_desired);
      command.kp[ddsIndex] = static_cast<float>(jointCommands_[i].kp);
      command.kd[ddsIndex] = static_cast<float>(jointCommands_[i].kd);
      // const double feedforwardTorque = limitFeedforwardTorque(i, jointCommands_[i].feedforward, period.seconds());
      // command.torque[ddsIndex] = static_cast<float>(feedforwardTorque);
      // command.feedforward_torque[ddsIndex] = static_cast<float>(feedforwardTorque);
      command.feedforward_torque[ddsIndex] = static_cast<float>(jointCommands_[i].feedforward);
      if (jointNames_[i] == "LF_HAA") {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("legged_p1_hw"), *emergencyStopNode_->get_clock(), 1000,
                             "Joint %s (DDS index %zu): pos_des=%.3f vel_des=%.3f kp=%.3f kd=%.3f ff=%.3f",
                             jointNames_[i].c_str(), ddsIndex, command.position_desired[ddsIndex],
                             command.velocity_desired[ddsIndex], command.kp[ddsIndex], command.kd[ddsIndex],
                             command.feedforward_torque[ddsIndex]);
      }
      if (jointNames_[i] == "LF_HFE") {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("legged_p1_hw"), *emergencyStopNode_->get_clock(), 1000,
                             "Joint %s (DDS index %zu): pos_des=%.3f vel_des=%.3f kp=%.3f kd=%.3f ff=%.3f",
                             jointNames_[i].c_str(), ddsIndex, command.position_desired[ddsIndex],
                             command.velocity_desired[ddsIndex], command.kp[ddsIndex], command.kd[ddsIndex],
                             command.feedforward_torque[ddsIndex]);
      }
      if (jointNames_[i] == "LF_KFE") {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("legged_p1_hw"), *emergencyStopNode_->get_clock(), 1000,
                             "Joint %s (DDS index %zu): pos_des=%.3f vel_des=%.3f kp=%.3f kd=%.3f ff=%.3f",
                             jointNames_[i].c_str(), ddsIndex, command.position_desired[ddsIndex],
                             command.velocity_desired[ddsIndex], command.kp[ddsIndex], command.kd[ddsIndex],
                             command.feedforward_torque[ddsIndex]);
      }
    } else {
      lastFeedforwardTorque_[i] = 0.0;
    }
    command.mode[ddsIndex] = commandMode_;
  }
  hasLastFeedforwardTorque_ = true;

  return dds_->writeCommand(command) ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::P1HW, hardware_interface::SystemInterface)
