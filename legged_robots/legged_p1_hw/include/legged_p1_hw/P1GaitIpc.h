#pragma once

#include <cstdint>

namespace legged_p1_hw {

struct P1GaitCommandPacket {
  double stamp{0.0};
  uint8_t gait_id{0};
  uint8_t emergency_stop{0};
  uint8_t trigger{0};
  float velocity_x{0.0F};
  float velocity_y{0.0F};
  float yaw_rate{0.0F};
};

}  // namespace legged_p1_hw
