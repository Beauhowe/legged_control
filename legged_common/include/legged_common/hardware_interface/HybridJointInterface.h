//
// Created by qiayuan on 2021/11/5.
//
#pragma once

#include <array>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace legged {

inline constexpr char HW_IF_POSITION_DESIRED[] = "position_desired";
inline constexpr char HW_IF_VELOCITY_DESIRED[] = "velocity_desired";
inline constexpr char HW_IF_KP[] = "kp";
inline constexpr char HW_IF_KD[] = "kd";
inline constexpr char HW_IF_FEEDFORWARD[] = "feedforward";

inline constexpr std::array<const char*, 3> HYBRID_JOINT_STATE_INTERFACES = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT,
};

inline constexpr std::array<const char*, 5> HYBRID_JOINT_COMMAND_INTERFACES = {
    HW_IF_POSITION_DESIRED,
    HW_IF_VELOCITY_DESIRED,
    HW_IF_KP,
    HW_IF_KD,
    HW_IF_FEEDFORWARD,
};

struct HybridJointCommand {
  double position_desired = 0.0;
  double velocity_desired = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double feedforward = 0.0;
};

struct HybridJointState {
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
};

inline std::string makeInterfaceName(const std::string& jointName, const std::string& interfaceName) {
  return jointName + "/" + interfaceName;
}

}  // namespace legged
