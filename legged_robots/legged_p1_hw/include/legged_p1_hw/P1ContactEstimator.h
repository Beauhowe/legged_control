#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

namespace legged {

// 根据 P1 单腿关节角和关节力矩，用腿部雅可比估计足端接触力。
class P1ContactEstimator {
 public:
  Eigen::Vector3d estimateFootForce(size_t legIndex, const std::array<double, 3>& jointPosition,
                                    const std::array<double, 3>& jointTorque) const;
};

}  // namespace legged
