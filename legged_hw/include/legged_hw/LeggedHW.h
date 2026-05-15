
//
// Created by qiayuan on 6/24/22.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

namespace legged {
class LeggedHW : public hardware_interface::SystemInterface {
 public:
  LeggedHW() = default;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardwareInfo) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  std::vector<std::string> jointNames_;                         // NOLINT(misc-non-private-member-variables-in-classes)
  std::vector<std::string> contactSensorNames_;                 // NOLINT(misc-non-private-member-variables-in-classes)
  std::vector<HybridJointState> jointStates_;                   // NOLINT(misc-non-private-member-variables-in-classes)
  std::vector<HybridJointCommand> jointCommands_;               // NOLINT(misc-non-private-member-variables-in-classes)
  std::vector<double> contactStates_;                           // NOLINT(misc-non-private-member-variables-in-classes)
  std::shared_ptr<urdf::Model> urdfModel_;  // NOLINT(misc-non-private-member-variables-in-classes)

 private:
  bool loadUrdf(const std::string& urdfString);

  rclcpp::Logger logger_{rclcpp::get_logger("legged_hw")};
};

}  // namespace legged
