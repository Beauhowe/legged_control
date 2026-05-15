
//
// Created by qiayuan on 1/24/22.
//

#include "legged_hw/LeggedHW.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>

namespace legged {
hardware_interface::CallbackReturn LeggedHW::on_init(const hardware_interface::HardwareInfo& hardwareInfo) {
  if (hardware_interface::SystemInterface::on_init(hardwareInfo) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!loadUrdf(hardwareInfo.original_xml)) {
    RCLCPP_ERROR(logger_, "Error occurred while setting up URDF");
    return hardware_interface::CallbackReturn::ERROR;
  }

  jointNames_.clear();
  jointNames_.reserve(info_.joints.size());
  for (const auto& joint : info_.joints) {
    jointNames_.push_back(joint.name);
  }
  jointStates_.resize(jointNames_.size());
  jointCommands_.resize(jointNames_.size());

  contactSensorNames_.clear();
  for (const auto& sensor : info_.sensors) {
    for (const auto& stateInterface : sensor.state_interfaces) {
      if (stateInterface.name == HW_IF_CONTACT) {
        contactSensorNames_.push_back(sensor.name);
        break;
      }
    }
  }
  contactStates_.assign(contactSensorNames_.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LeggedHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> stateInterfaces;
  stateInterfaces.reserve(jointStates_.size() * HYBRID_JOINT_STATE_INTERFACES.size() + contactStates_.size());

  for (size_t i = 0; i < jointNames_.size(); ++i) {
    stateInterfaces.emplace_back(jointNames_[i], hardware_interface::HW_IF_POSITION, &jointStates_[i].position);
    stateInterfaces.emplace_back(jointNames_[i], hardware_interface::HW_IF_VELOCITY, &jointStates_[i].velocity);
    stateInterfaces.emplace_back(jointNames_[i], hardware_interface::HW_IF_EFFORT, &jointStates_[i].effort);
  }

  for (size_t i = 0; i < contactSensorNames_.size(); ++i) {
    stateInterfaces.emplace_back(contactSensorNames_[i], HW_IF_CONTACT, &contactStates_[i]);
  }

  return stateInterfaces;
}

std::vector<hardware_interface::CommandInterface> LeggedHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> commandInterfaces;
  commandInterfaces.reserve(jointCommands_.size() * HYBRID_JOINT_COMMAND_INTERFACES.size());

  for (size_t i = 0; i < jointNames_.size(); ++i) {
    commandInterfaces.emplace_back(jointNames_[i], HW_IF_POSITION_DESIRED, &jointCommands_[i].position_desired);
    commandInterfaces.emplace_back(jointNames_[i], HW_IF_VELOCITY_DESIRED, &jointCommands_[i].velocity_desired);
    commandInterfaces.emplace_back(jointNames_[i], HW_IF_KP, &jointCommands_[i].kp);
    commandInterfaces.emplace_back(jointNames_[i], HW_IF_KD, &jointCommands_[i].kd);
    commandInterfaces.emplace_back(jointNames_[i], HW_IF_FEEDFORWARD, &jointCommands_[i].feedforward);
  }

  return commandInterfaces;
}

hardware_interface::return_type LeggedHW::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LeggedHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

bool LeggedHW::loadUrdf(const std::string& urdfString) {
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

}  // namespace legged
