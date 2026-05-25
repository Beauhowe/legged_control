#pragma once

#include <legged_hw/LeggedHW.h>
#include <legged_p1_hw/P1ContactEstimator.h>
#include <legged_p1_hw/P1DdsInterface.h>
#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace legged {

// P1 真机 ros2_control 硬件插件：负责 ros2_control 接口，DDS 细节交给 P1DdsInterface。
class P1HW : public LeggedHW {
 public:
  P1HW() = default;
  ~P1HW() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardwareInfo) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  static int readIntParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name, int defaultValue);
  static double readDoubleParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name, double defaultValue);
  static std::string readStringParameter(const hardware_interface::HardwareInfo& hardwareInfo, const std::string& name,
                                         const std::string& defaultValue);

  bool loadJointMappingParameters(const hardware_interface::HardwareInfo& hardwareInfo);
  void loadTorqueEstimationParameters(const hardware_interface::HardwareInfo& hardwareInfo);
  void loadContactEstimationParameters(const hardware_interface::HardwareInfo& hardwareInfo);
  int findJointIndex(const std::string& jointName) const;
  double estimateJointTorque(size_t jointIndex, double current) const;

  // ros2_control 的 base_imu state interface 后端存储。
  std::array<double, 4> imuOrientation_{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> imuAngularVelocity_{0.0, 0.0, 0.0};
  std::array<double, 3> imuLinearAcceleration_{0.0, 0.0, 0.0};

  int contactThreshold_{0};
  uint32_t commandMode_{10};
  bool useJacobianContactEstimation_{false};
  double contactForceThreshold_{40.0};
  // jointToDdsIndex_[控制层关节下标] = DDS motor_state/motor_cmd 数组下标。
  std::array<size_t, 12> jointToDdsIndex_{};
  // current_to_torque_scale/offset 用来预留电流到关节力矩的标定接口。
  // 默认 scale=1, offset=0，行为等价于直接把 current 放进 effort。
  std::array<double, 12> currentToTorqueScale_{};
  std::array<double, 12> currentToTorqueOffset_{};
  P1ContactEstimator contactEstimator_;
  std::unique_ptr<P1DdsInterface> dds_;
};

}  // namespace legged
