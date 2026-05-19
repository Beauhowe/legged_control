#pragma once

#include <legged_hw/LeggedHW.h>

#include <array>

namespace legged {

// lr_pro 的 IMU 状态缓存。
// 这些字段会通过 export_state_interfaces() 暴露给 ros2_control，名称需要和 controller 中的 imuName=base_imu 对齐。
struct LrProImuData {
  // 四元数顺序为 x, y, z, w；默认值表示机身姿态为单位四元数。
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
  // 机体系角速度，单位 rad/s。
  std::array<double, 3> angularVelocity{0.0, 0.0, 0.0};
  // 机体系线加速度，单位 m/s^2；占位实现中默认给出重力方向数值。
  std::array<double, 3> linearAcceleration{0.0, 0.0, 9.81};
};

// lr_pro 的 ros2_control 硬件接口。
// 当前类先复用 LeggedHW 的关节/足端接触接口，后续真实上机时主要补 read() 和 write()：
// - read(): 从 lr_pro SDK/底层协议读取关节、IMU、足端接触状态。
// - write(): 将 controller 计算出的 jointCommands_ 下发到电机控制器。
class LrProHW : public LeggedHW {
 public:
  LrProHW() = default;
  ~LrProHW() override = default;

  // 初始化硬件接口参数，并调用 LeggedHW 初始化通用的关节/接触接口。
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardwareInfo) override;

  // 导出状态接口：包含 LeggedHW 的关节/接触状态，以及 lr_pro 自己的 base_imu 状态。
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 从真实机器人读取状态。当前是占位实现，用于先打通 controller manager 链路。
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // 向真实机器人写入控制命令。当前是占位实现，后续在这里接入 lr_pro 电机通信。
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // IMU 状态缓存，必须保持成员生命周期，因为 StateInterface 内部保存的是这些字段的指针。
  LrProImuData imuData_{};
  // 第一次 read() 前没有上一周期命令，因此先把关节状态初始化为 0。
  bool initializedState_{false};
};

}  // namespace legged
