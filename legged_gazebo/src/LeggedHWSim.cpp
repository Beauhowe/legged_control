/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 2/10/21.
//

#include "legged_gazebo/LeggedHWSim.h"

#include <algorithm>

#include <angles/angles.h>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Contact.hh>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
namespace {
double getHardwareParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name, double defaultValue) {
  const auto it = hardwareInfo.hardware_parameters.find(name);
  return it == hardwareInfo.hardware_parameters.end() ? defaultValue : std::stod(it->second);
}

bool hasStateInterface(const hardware_interface::ComponentInfo& component, const std::string& interfaceName) {
  return std::any_of(component.state_interfaces.begin(), component.state_interfaces.end(),
                     [&](const auto& stateInterface) { return stateInterface.name == interfaceName; });
}
}  // namespace

hardware_interface::CallbackReturn LeggedHWSim::on_init(const hardware_interface::HardwareInfo& hardwareInfo) {
  if (hardware_interface::SystemInterface::on_init(hardwareInfo) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  delay_ = getHardwareParameter(hardwareInfo, "delay", 0.009);
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool LeggedHWSim::initSim(rclcpp::Node::SharedPtr& model_nh, gazebo::physics::ModelPtr parent_model,
                          const hardware_interface::HardwareInfo& hardware_info, sdf::ElementPtr /*sdf*/) {
  nh_ = model_nh;
  parentModel_ = std::move(parent_model);
  contactManager_ = parentModel_->GetWorld()->Physics()->GetContactManager();
  contactManager_->SetNeverDropContacts(true);
  baseLink_ = parentModel_->GetLink("base");
  groundTruthPublisher_ = nh_->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/state", 10);

  return setupJoints(hardware_info) && setupImu(hardware_info) && setupContacts(hardware_info);
}

std::vector<hardware_interface::StateInterface> LeggedHWSim::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> stateInterfaces;
  stateInterfaces.reserve(jointData_.size() * HYBRID_JOINT_STATE_INTERFACES.size() + contactStates_.size() + imuData_.size() * 10);

  for (auto& joint : jointData_) {
    stateInterfaces.emplace_back(joint.name_, hardware_interface::HW_IF_POSITION, &joint.state_.position);
    stateInterfaces.emplace_back(joint.name_, hardware_interface::HW_IF_VELOCITY, &joint.state_.velocity);
    stateInterfaces.emplace_back(joint.name_, hardware_interface::HW_IF_EFFORT, &joint.state_.effort);
  }

  for (size_t i = 0; i < contactNames_.size(); ++i) {
    stateInterfaces.emplace_back(contactNames_[i], HW_IF_CONTACT, &contactStates_[i]);
  }

  for (auto& imu : imuData_) {
    stateInterfaces.emplace_back(imu.name_, "orientation.x", &imu.ori_[0]);
    stateInterfaces.emplace_back(imu.name_, "orientation.y", &imu.ori_[1]);
    stateInterfaces.emplace_back(imu.name_, "orientation.z", &imu.ori_[2]);
    stateInterfaces.emplace_back(imu.name_, "orientation.w", &imu.ori_[3]);
    stateInterfaces.emplace_back(imu.name_, "angular_velocity.x", &imu.angularVel_[0]);
    stateInterfaces.emplace_back(imu.name_, "angular_velocity.y", &imu.angularVel_[1]);
    stateInterfaces.emplace_back(imu.name_, "angular_velocity.z", &imu.angularVel_[2]);
    stateInterfaces.emplace_back(imu.name_, "linear_acceleration.x", &imu.linearAcc_[0]);
    stateInterfaces.emplace_back(imu.name_, "linear_acceleration.y", &imu.linearAcc_[1]);
    stateInterfaces.emplace_back(imu.name_, "linear_acceleration.z", &imu.linearAcc_[2]);
  }

  return stateInterfaces;
}

std::vector<hardware_interface::CommandInterface> LeggedHWSim::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> commandInterfaces;
  commandInterfaces.reserve(jointData_.size() * HYBRID_JOINT_COMMAND_INTERFACES.size());

  for (auto& joint : jointData_) {
    commandInterfaces.emplace_back(joint.name_, HW_IF_POSITION_DESIRED, &joint.command_.position_desired);
    commandInterfaces.emplace_back(joint.name_, HW_IF_VELOCITY_DESIRED, &joint.command_.velocity_desired);
    commandInterfaces.emplace_back(joint.name_, HW_IF_KP, &joint.command_.kp);
    commandInterfaces.emplace_back(joint.name_, HW_IF_KD, &joint.command_.kd);
    commandInterfaces.emplace_back(joint.name_, HW_IF_FEEDFORWARD, &joint.command_.feedforward);
  }

  return commandInterfaces;
}

hardware_interface::return_type LeggedHWSim::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
  const double dt = period.seconds();
  for (auto& joint : jointData_) {
    const double position = joint.joint_->Position(0);
    joint.state_.velocity = dt > 0.0 ? (position - joint.state_.position) / dt : 0.0;
    if (time.nanoseconds() == period.nanoseconds()) {
      joint.state_.velocity = 0.0;
    }
    joint.state_.position += angles::shortest_angular_distance(joint.state_.position, position);
    joint.state_.effort = joint.joint_->GetForce(0);
  }

  for (auto& imu : imuData_) {
    const auto pose = imu.linkPtr_->WorldPose();
    imu.ori_[0] = pose.Rot().X();
    imu.ori_[1] = pose.Rot().Y();
    imu.ori_[2] = pose.Rot().Z();
    imu.ori_[3] = pose.Rot().W();

    const auto angularVelocity = imu.linkPtr_->RelativeAngularVel();
    imu.angularVel_[0] = angularVelocity.X();
    imu.angularVel_[1] = angularVelocity.Y();
    imu.angularVel_[2] = angularVelocity.Z();

    const ignition::math::Vector3d gravity{0.0, 0.0, -9.81};
    const auto linearAcceleration = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linearAcc_[0] = linearAcceleration.X();
    imu.linearAcc_[1] = linearAcceleration.Y();
    imu.linearAcc_[2] = linearAcceleration.Z();
  }

  std::fill(contactStates_.begin(), contactStates_.end(), 0.0);
  const auto previousUpdateTime = time - period;
  for (const auto& contact : contactManager_->GetContacts()) {
    const int64_t contactTimeNs = static_cast<int64_t>(contact->time.sec) * 1000000000LL + contact->time.nsec;
    if (contactTimeNs != previousUpdateTime.nanoseconds()) {
      continue;
    }

    const std::string link1 = contact->collision1->GetLink()->GetName();
    const std::string link2 = contact->collision2->GetLink()->GetName();
    for (size_t i = 0; i < contactLinkNames_.size(); ++i) {
      if (link1 == contactLinkNames_[i] || link2 == contactLinkNames_[i]) {
        contactStates_[i] = 1.0;
      }
    }
  }

  for (auto& joint : jointData_) {
    joint.command_.position_desired = joint.state_.position;
    joint.command_.velocity_desired = 0.0;
    joint.command_.kp = 0.0;
    joint.command_.kd = 0.0;
    joint.command_.feedforward = 0.0;
  }

  updateGroundTruth(time);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LeggedHWSim::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
  for (size_t i = 0; i < jointData_.size(); ++i) {
    auto& joint = jointData_[i];
    auto& buffer = cmdBuffer_[i];
    if (time.nanoseconds() == period.nanoseconds()) {
      buffer.clear();
    }

    while (!buffer.empty() && (time - buffer.back().stamp_).seconds() > delay_) {
      buffer.pop_back();
    }
    buffer.push_front(DelayedHybridJointCommand{time, joint.command_});

    const auto& command = buffer.back().command_;
    const double effort = command.kp * (command.position_desired - joint.state_.position) +
                          command.kd * (command.velocity_desired - joint.state_.velocity) + command.feedforward;
    joint.joint_->SetForce(0, effort);
  }

  return hardware_interface::return_type::OK;
}

bool LeggedHWSim::setupJoints(const hardware_interface::HardwareInfo& hardwareInfo) {
  jointData_.clear();
  jointData_.reserve(hardwareInfo.joints.size());
  cmdBuffer_.clear();
  cmdBuffer_.resize(hardwareInfo.joints.size());

  for (const auto& jointInfo : hardwareInfo.joints) {
    auto joint = parentModel_->GetJoint(jointInfo.name);
    if (!joint) {
      RCLCPP_ERROR(logger_, "Gazebo joint '%s' not found.", jointInfo.name.c_str());
      return false;
    }
    jointData_.push_back(SimHybridJointData{jointInfo.name, joint, {}, {}});
  }
  return true;
}

bool LeggedHWSim::setupImu(const hardware_interface::HardwareInfo& hardwareInfo) {
  imuData_.clear();
  for (const auto& sensorInfo : hardwareInfo.sensors) {
    if (!hasStateInterface(sensorInfo, "orientation.x")) {
      continue;
    }
    auto link = parentModel_->GetLink(sensorInfo.name);
    if (!link) {
      RCLCPP_ERROR(logger_, "Gazebo IMU link '%s' not found.", sensorInfo.name.c_str());
      return false;
    }

    ImuData imu{};
    imu.name_ = sensorInfo.name;
    imu.linkPtr_ = link;
    imu.oriCov_[0] = 0.0012;
    imu.oriCov_[4] = 0.0012;
    imu.oriCov_[8] = 0.0012;
    imu.angularVelCov_[0] = 0.0004;
    imu.angularVelCov_[4] = 0.0004;
    imu.angularVelCov_[8] = 0.0004;
    imu.linearAccCov_[0] = 0.01;
    imu.linearAccCov_[4] = 0.01;
    imu.linearAccCov_[8] = 0.01;
    imuData_.push_back(imu);
  }
  return true;
}

bool LeggedHWSim::setupContacts(const hardware_interface::HardwareInfo& hardwareInfo) {
  contactNames_.clear();
  contactLinkNames_.clear();
  for (const auto& sensorInfo : hardwareInfo.sensors) {
    if (!hasStateInterface(sensorInfo, HW_IF_CONTACT)) {
      continue;
    }
    contactNames_.push_back(sensorInfo.name);
    contactLinkNames_.push_back(sensorInfo.name);
  }
  contactStates_.assign(contactNames_.size(), 0.0);
  return true;
}

void LeggedHWSim::updateGroundTruth(const rclcpp::Time& time) {
  if (!baseLink_ || !groundTruthPublisher_) {
    return;
  }

  const auto pose = baseLink_->WorldPose();
  const auto linearVelocity = baseLink_->WorldLinearVel();
  const auto angularVelocity = baseLink_->WorldAngularVel();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base";
  odom.pose.pose.position.x = pose.Pos().X();
  odom.pose.pose.position.y = pose.Pos().Y();
  odom.pose.pose.position.z = pose.Pos().Z();
  odom.pose.pose.orientation.x = pose.Rot().X();
  odom.pose.pose.orientation.y = pose.Rot().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();
  odom.pose.pose.orientation.w = pose.Rot().W();
  odom.twist.twist.linear.x = linearVelocity.X();
  odom.twist.twist.linear.y = linearVelocity.Y();
  odom.twist.twist.linear.z = linearVelocity.Z();
  odom.twist.twist.angular.x = angularVelocity.X();
  odom.twist.twist.angular.y = angularVelocity.Y();
  odom.twist.twist.angular.z = angularVelocity.Z();
  groundTruthPublisher_->publish(odom);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gazebo_ros2_control::GazeboSystemInterface)
