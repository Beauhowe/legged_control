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

#pragma once

#include <deque>
#include <string>
#include <vector>

#include <gazebo/physics/physics.hh>
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace legged {
struct SimHybridJointData {
  std::string name_;
  gazebo::physics::JointPtr joint_;
  HybridJointState state_;
  HybridJointCommand command_;
};

struct DelayedHybridJointCommand {
  rclcpp::Time stamp_;
  HybridJointCommand command_;
};

struct ImuData {
  std::string name_;
  gazebo::physics::LinkPtr linkPtr_;
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class LeggedHWSim : public gazebo_ros2_control::GazeboSystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardwareInfo) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  bool initSim(rclcpp::Node::SharedPtr& model_nh, gazebo::physics::ModelPtr parent_model,
               const hardware_interface::HardwareInfo& hardware_info, sdf::ElementPtr sdf) override;

 private:
  bool setupJoints(const hardware_interface::HardwareInfo& hardwareInfo);
  bool setupImu(const hardware_interface::HardwareInfo& hardwareInfo);
  bool setupContacts(const hardware_interface::HardwareInfo& hardwareInfo);
  void publishJointCommands();
  void updateGroundTruth(const rclcpp::Time& time);

  rclcpp::Logger logger_{rclcpp::get_logger("legged_gazebo")};

  gazebo::physics::ModelPtr parentModel_;
  gazebo::physics::ContactManager* contactManager_{};
  gazebo::physics::LinkPtr baseLink_;

  std::vector<SimHybridJointData> jointData_;
  std::vector<double> contactStates_;
  std::vector<std::string> contactNames_;
  std::vector<std::string> contactLinkNames_;
  std::vector<ImuData> imuData_;
  std::vector<std::deque<DelayedHybridJointCommand>> cmdBuffer_;
  std::vector<double> effortCommand_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr groundTruthPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effortCommandPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedforwardCommandPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr positionDesiredCommandPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocityDesiredCommandPublisher_;

  size_t delayCycles_{9};
};

}  // namespace legged
