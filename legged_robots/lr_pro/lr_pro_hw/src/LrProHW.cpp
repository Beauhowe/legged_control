#include "lr_pro_hw/LrProHW.h"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace legged {
namespace {
// 防止 NaN/Inf 命令污染状态缓存。真实硬件实现中也建议保留类似保护。
bool isFinite(double value) {
  return std::isfinite(value);
}
}  // namespace

hardware_interface::CallbackReturn LrProHW::on_init(const hardware_interface::HardwareInfo& hardwareInfo) {
  // 先初始化 LeggedHW 中通用的 hybrid joint command/state 和 contact state 存储。
  if (LeggedHW::on_init(hardwareInfo) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 提醒：当前类只是硬件接口骨架，不能直接用于真实机器人闭环控制。
  RCLCPP_WARN(rclcpp::get_logger("lr_pro_hw"),
              "LrProHW is a placeholder hardware interface. Replace read()/write() with the real lr_pro communication layer before "
              "running on hardware.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LrProHW::export_state_interfaces() {
  // LeggedHW 会先导出 12 个关节状态和 4 个足端接触状态。
  auto stateInterfaces = LeggedHW::export_state_interfaces();

  // 额外导出 base_imu，名称需要和 lr_pro.launch.py 中 controller_params 的 imuName 保持一致。
  stateInterfaces.emplace_back("base_imu", "orientation.x", &imuData_.orientation[0]);
  stateInterfaces.emplace_back("base_imu", "orientation.y", &imuData_.orientation[1]);
  stateInterfaces.emplace_back("base_imu", "orientation.z", &imuData_.orientation[2]);
  stateInterfaces.emplace_back("base_imu", "orientation.w", &imuData_.orientation[3]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.x", &imuData_.angularVelocity[0]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.y", &imuData_.angularVelocity[1]);
  stateInterfaces.emplace_back("base_imu", "angular_velocity.z", &imuData_.angularVelocity[2]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.x", &imuData_.linearAcceleration[0]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.y", &imuData_.linearAcceleration[1]);
  stateInterfaces.emplace_back("base_imu", "linear_acceleration.z", &imuData_.linearAcceleration[2]);

  return stateInterfaces;
}

hardware_interface::return_type LrProHW::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // 占位实现：没有实机通信时，将上一周期命令镜像为状态，便于先验证 ros2_control 链路。
  // 接入真实 lr_pro 后，这里应替换为：读取电机编码器/速度/力矩、IMU、足端接触，并填充 jointStates_/imuData_/contactStates_。
  for (size_t i = 0; i < jointStates_.size(); ++i) {
    if (!initializedState_) {
      jointStates_[i].position = 0.0;
      jointStates_[i].velocity = 0.0;
      jointStates_[i].effort = 0.0;
      continue;
    }

    if (isFinite(jointCommands_[i].position_desired)) {
      jointStates_[i].position = jointCommands_[i].position_desired;
    }
    if (isFinite(jointCommands_[i].velocity_desired)) {
      jointStates_[i].velocity = jointCommands_[i].velocity_desired;
    }
    if (isFinite(jointCommands_[i].feedforward)) {
      jointStates_[i].effort = jointCommands_[i].feedforward;
    }
  }

  // 占位实现默认没有接触。真实硬件可由足端力传感器、电流估计或底层状态包更新这些值。
  std::fill(contactStates_.begin(), contactStates_.end(), 0.0);
  initializedState_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LrProHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // TODO: 将 jointCommands_ 转换为 lr_pro 实机协议并发送到底层电机控制器。
  // 常见映射包括：期望位置、期望速度、kp、kd、前馈力矩，以及必要的限幅/急停保护。
  return hardware_interface::return_type::OK;
}

}  // namespace legged

// 将 legged::LrProHW 注册为 pluginlib 插件，供 ros2_control 通过 <plugin>lr_pro_hw/LrProHW</plugin> 加载。
PLUGINLIB_EXPORT_CLASS(legged::LrProHW, hardware_interface::SystemInterface)
