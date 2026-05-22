
//
// Created by qiayuan on 1/24/22.
//

#include "legged_unitree_hw/UnitreeHW.h"

#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/unitree_joystick.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/joystick.h"
#endif

#include <pluginlib/class_list_macros.hpp>

namespace legged {
namespace {
int readIntParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name, int defaultValue) {
  const auto it = hardwareInfo.hardware_parameters.find(name);
  if (it == hardwareInfo.hardware_parameters.end()) {
    return defaultValue;
  }
  return std::stoi(it->second);
}

std::string readStringParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name,
                                const std::string& defaultValue) {
  const auto it = hardwareInfo.hardware_parameters.find(name);
  return it == hardwareInfo.hardware_parameters.end() ? defaultValue : it->second;
}
}  // namespace

UnitreeHW::~UnitreeHW() {
  executor_.cancel();
  if (spinThread_.joinable()) {
    spinThread_.join();
  }
}

hardware_interface::CallbackReturn UnitreeHW::on_init(const hardware_interface::HardwareInfo& hardwareInfo) {
  if (LeggedHW::on_init(hardwareInfo) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  powerLimit_ = readIntParameter(hardwareInfo, "power_limit", 10);
  contactThreshold_ = readIntParameter(hardwareInfo, "contact_threshold", 20);

  if (!setupJoints() || !setupImu() || !setupContactSensor()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

#ifdef UNITREE_SDK_3_3_1
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
#elif UNITREE_SDK_3_8_0
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007);
#endif

  udp_->InitCmdData(lowCmd_);

  const auto robot_type = readStringParameter(hardwareInfo, "robot_type", "");
#ifdef UNITREE_SDK_3_3_1
  if (robot_type == "a1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  } else if (robot_type == "aliengo") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  }
#elif UNITREE_SDK_3_8_0
  if (robot_type == "go1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Go1);
  }
#endif
  else {
    RCLCPP_FATAL(rclcpp::get_logger("legged_unitree_hw"), "Unknown robot type: %s", robot_type.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("legged_unitree_hw");
  joyPublisher_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
  contactPublisher_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>("/contact", 10);
  executor_.add_node(node_);
  spinThread_ = std::thread([this]() { executor_.spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> UnitreeHW::export_state_interfaces() {
  auto stateInterfaces = LeggedHW::export_state_interfaces();

  stateInterfaces.emplace_back("base_imu", "orientation.x", &imuData_.ori_[0]);
  stateInterfaces.emplace_back("base_imu", "orientation.y", &imuData_.ori_[1]);
  stateInterfaces.emplace_back("base_imu", "orientation.z", &imuData_.ori_[2]);
  stateInterfaces.emplace_back("base_imu", "orientation.w", &imuData_.ori_[3]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.x", &imuData_.angularVel_[0]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.y", &imuData_.angularVel_[1]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.z", &imuData_.angularVel_[2]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.x", &imuData_.linearAcc_[0]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.y", &imuData_.linearAcc_[1]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.z", &imuData_.linearAcc_[2]);

  return stateInterfaces;
}

hardware_interface::return_type UnitreeHW::read(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  udp_->Recv();
  udp_->GetRecv(lowState_);

  for (size_t i = 0; i < jointNames_.size(); ++i) {
    const auto motorIndex = jointToMotorIndex_[i];
    jointStates_[i].position = lowState_.motorState[motorIndex].q;
    jointStates_[i].velocity = lowState_.motorState[motorIndex].dq;
    jointStates_[i].effort = lowState_.motorState[motorIndex].tauEst;
  }

  imuData_.ori_[0] = lowState_.imu.quaternion[1];
  imuData_.ori_[1] = lowState_.imu.quaternion[2];
  imuData_.ori_[2] = lowState_.imu.quaternion[3];
  imuData_.ori_[3] = lowState_.imu.quaternion[0];
  imuData_.angularVel_[0] = lowState_.imu.gyroscope[0];
  imuData_.angularVel_[1] = lowState_.imu.gyroscope[1];
  imuData_.angularVel_[2] = lowState_.imu.gyroscope[2];
  imuData_.linearAcc_[0] = lowState_.imu.accelerometer[0];
  imuData_.linearAcc_[1] = lowState_.imu.accelerometer[1];
  imuData_.linearAcc_[2] = lowState_.imu.accelerometer[2];

  for (size_t i = 0; i < contactSensorNames_.size(); ++i) {
    contactStates_[i] = lowState_.footForce[contactToFootIndex_[i]] > contactThreshold_ ? 1.0 : 0.0;
  }

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  for (auto& command : jointCommands_) {
    command.feedforward = 0.0;
    command.velocity_desired = 0.0;
    command.kd = 3.0;
  }

  updateJoystick(time);
  updateContact(time);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < jointNames_.size(); ++i) {
    const auto motorIndex = jointToMotorIndex_[i];
    lowCmd_.motorCmd[motorIndex].q = static_cast<float>(jointCommands_[i].position_desired);
    lowCmd_.motorCmd[motorIndex].dq = static_cast<float>(jointCommands_[i].velocity_desired);
    lowCmd_.motorCmd[motorIndex].Kp = static_cast<float>(jointCommands_[i].kp);
    lowCmd_.motorCmd[motorIndex].Kd = static_cast<float>(jointCommands_[i].kd);
    lowCmd_.motorCmd[motorIndex].tau = static_cast<float>(jointCommands_[i].feedforward);
  }
  safety_->PositionLimit(lowCmd_);
  safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  udp_->SetSend(lowCmd_);
  udp_->Send();
  return hardware_interface::return_type::OK;
}

bool UnitreeHW::setupJoints() {
  jointToMotorIndex_.clear();
  jointToMotorIndex_.reserve(jointNames_.size());
  for (const auto& jointName : jointNames_) {
    const auto index = getUnitreeMotorIndex(jointName);
    if (index < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("legged_unitree_hw"), "Unsupported Unitree joint name: %s", jointName.c_str());
      return false;
    }
    jointToMotorIndex_.push_back(index);
  }
  return true;
}

bool UnitreeHW::setupImu() {
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

bool UnitreeHW::setupContactSensor() {
  contactToFootIndex_.clear();
  contactToFootIndex_.reserve(contactSensorNames_.size());
  for (const auto& contactName : contactSensorNames_) {
    const auto index = getUnitreeContactIndex(contactName);
    if (index < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("legged_unitree_hw"), "Unsupported Unitree contact sensor name: %s", contactName.c_str());
      return false;
    }
    contactToFootIndex_.push_back(index);
  }
  return true;
}

int UnitreeHW::getUnitreeMotorIndex(const std::string& jointName) const {
  int leg_index = 0;
  int joint_index = 0;
  if (jointName.find("RF") != std::string::npos) {
    leg_index = UNITREE_LEGGED_SDK::FR_;
  } else if (jointName.find("LF") != std::string::npos) {
    leg_index = UNITREE_LEGGED_SDK::FL_;
  } else if (jointName.find("RH") != std::string::npos) {
    leg_index = UNITREE_LEGGED_SDK::RR_;
  } else if (jointName.find("LH") != std::string::npos) {
    leg_index = UNITREE_LEGGED_SDK::RL_;
  } else {
    return -1;
  }

  if (jointName.find("HAA") != std::string::npos) {
    joint_index = 0;
  } else if (jointName.find("HFE") != std::string::npos) {
    joint_index = 1;
  } else if (jointName.find("KFE") != std::string::npos) {
    joint_index = 2;
  } else {
    return -1;
  }

  return leg_index * 3 + joint_index;
}

int UnitreeHW::getUnitreeContactIndex(const std::string& contactName) const {
  if (contactName.find("RF") != std::string::npos) {
    return UNITREE_LEGGED_SDK::FR_;
  }
  if (contactName.find("LF") != std::string::npos) {
    return UNITREE_LEGGED_SDK::FL_;
  }
  if (contactName.find("RH") != std::string::npos) {
    return UNITREE_LEGGED_SDK::RR_;
  }
  if (contactName.find("LH") != std::string::npos) {
    return UNITREE_LEGGED_SDK::RL_;
  }
  return -1;
}

void UnitreeHW::updateJoystick(const rclcpp::Time& time) {
  if ((time - lastJoyPub_).seconds() < 1 / 50.) {
    return;
  }
  lastJoyPub_ = time;
  xRockerBtnDataStruct keyData;
  memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
  sensor_msgs::msg::Joy joyMsg;  // Pack as same as Logitech F710
  joyMsg.axes.push_back(-keyData.lx);
  joyMsg.axes.push_back(keyData.ly);
  joyMsg.axes.push_back(-keyData.rx);
  joyMsg.axes.push_back(keyData.ry);
  joyMsg.buttons.push_back(keyData.btn.components.X);
  joyMsg.buttons.push_back(keyData.btn.components.A);
  joyMsg.buttons.push_back(keyData.btn.components.B);
  joyMsg.buttons.push_back(keyData.btn.components.Y);
  joyMsg.buttons.push_back(keyData.btn.components.L1);
  joyMsg.buttons.push_back(keyData.btn.components.R1);
  joyMsg.buttons.push_back(keyData.btn.components.L2);
  joyMsg.buttons.push_back(keyData.btn.components.R2);
  joyMsg.buttons.push_back(keyData.btn.components.select);
  joyMsg.buttons.push_back(keyData.btn.components.start);
  joyPublisher_->publish(joyMsg);
}

void UnitreeHW::updateContact(const rclcpp::Time& time) {
  if ((time - lastContactPub_).seconds() < 1 / 50.) {
    return;
  }
  lastContactPub_ = time;

  std_msgs::msg::Int16MultiArray contactMsg;
  for (size_t i = 0; i < contactSensorNames_.size(); ++i) {
    contactMsg.data.push_back(lowState_.footForce[contactToFootIndex_[i]]);
  }
  contactPublisher_->publish(contactMsg);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::UnitreeHW, hardware_interface::SystemInterface)
