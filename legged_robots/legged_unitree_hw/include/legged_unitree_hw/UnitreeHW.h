
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <legged_hw/LeggedHW.h>

#include <array>
#include <thread>
#include <unordered_map>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/safety.h"
#include "unitree_legged_sdk_3_3_1/udp.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/safety.h"
#include "unitree_legged_sdk_3_8_0/udp.h"
#endif

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class UnitreeHW : public LeggedHW {
 public:
  UnitreeHW() = default;
  ~UnitreeHW() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardwareInfo) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  void updateJoystick(const rclcpp::Time& time);

  void updateContact(const rclcpp::Time& time);

 private:
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor();

  int getUnitreeMotorIndex(const std::string& jointName) const;

  int getUnitreeContactIndex(const std::string& contactName) const;

  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState lowState_{};
  UNITREE_LEGGED_SDK::LowCmd lowCmd_{};

  UnitreeMotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  UnitreeImuData imuData_{};
  bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

  int powerLimit_{};
  int contactThreshold_{};

  std::vector<int> jointToMotorIndex_;
  std::vector<int> contactToFootIndex_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinThread_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joyPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr contactPublisher_;
  rclcpp::Time lastJoyPub_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastContactPub_{0, 0, RCL_ROS_TIME};
};

}  // namespace legged
