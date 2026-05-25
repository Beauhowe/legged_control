#include "legged_p1_hw/P1ContactEstimator.h"

#include <cmath>

#include <Eigen/Dense>

namespace legged {
namespace {
constexpr double kHipThighOffsetX = 0.089;
constexpr double kThighOffsetY = 0.0395;
constexpr double kKneeOffsetY = 0.1323;
constexpr double kThighLength = 0.35;
constexpr double kCalfLength = 0.35;

Eigen::Matrix3d rotationX(double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Eigen::Matrix3d r;
  r << 1.0, 0.0, 0.0,
       0.0, c, -s,
       0.0, s, c;
  return r;
}

Eigen::Matrix3d rotationY(double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Eigen::Matrix3d r;
  r << c, 0.0, s,
       0.0, 1.0, 0.0,
       -s, 0.0, c;
  return r;
}

int legMirror(size_t legIndex) {
  return legIndex < 2 ? 1 : -1;
}

int legFrontHind(size_t legIndex) {
  return (legIndex == 0 || legIndex == 2) ? 1 : -1;
}
}  // namespace

Eigen::Vector3d P1ContactEstimator::estimateFootForce(size_t legIndex, const std::array<double, 3>& jointPosition,
                                                      const std::array<double, 3>& jointTorque) const {
  const double qHaa = jointPosition[0];
  const double qHfe = jointPosition[1];
  const double qKfe = jointPosition[2];
  const int mirror = legMirror(legIndex);
  const int frontHind = legFrontHind(legIndex);

  const Eigen::Vector3d pHaa = Eigen::Vector3d::Zero();
  const Eigen::Vector3d axisHaa = Eigen::Vector3d::UnitX();
  const Eigen::Matrix3d rHaa = rotationX(qHaa);

  const Eigen::Vector3d pHfe = rHaa * Eigen::Vector3d(kHipThighOffsetX * frontHind, kThighOffsetY * mirror, 0.0);
  const Eigen::Vector3d axisHfe = rHaa * Eigen::Vector3d::UnitY();
  const Eigen::Matrix3d rHfe = rHaa * rotationY(qHfe);

  const Eigen::Vector3d pKfe = pHfe + rHfe * Eigen::Vector3d(0.0, kKneeOffsetY * mirror, -kThighLength);
  const Eigen::Vector3d axisKfe = rHfe * Eigen::Vector3d::UnitY();
  const Eigen::Matrix3d rKfe = rHfe * rotationY(qKfe);

  const Eigen::Vector3d pFoot = pKfe + rKfe * Eigen::Vector3d(0.0, 0.0, -kCalfLength);

  Eigen::Matrix3d jacobian;
  jacobian.col(0) = axisHaa.cross(pFoot - pHaa);
  jacobian.col(1) = axisHfe.cross(pFoot - pHfe);
  jacobian.col(2) = axisKfe.cross(pFoot - pKfe);

  Eigen::Vector3d tau;
  tau << jointTorque[0], jointTorque[1], jointTorque[2];

  // tau = J^T * F。这里求最小二乘解，接近奇异位形时也能给出一个连续估计值。
  return jacobian.transpose().completeOrthogonalDecomposition().solve(tau);
}

}  // namespace legged
