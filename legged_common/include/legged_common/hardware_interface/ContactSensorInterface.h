//
// Created by qiayuan on 2021/11/5.
//
#pragma once

#include <string>

namespace legged {

inline constexpr char HW_IF_CONTACT[] = "contact";

struct ContactSensorState {
  bool contact = false;
};

inline std::string makeContactInterfaceName(const std::string& sensorName) {
  return sensorName + "/" + HW_IF_CONTACT;
}

}  // namespace legged
