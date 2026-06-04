//
// Collision sphere constraint using terrain signed distance field.
//

#include "legged_interface/perceptive/constraint/SphereSdfConstraint.h"

#include "legged_interface/perceptive/PerceptiveLeggedPrecomputation.h"

namespace legged {

SphereSdfConstraint::SphereSdfConstraint(const PinocchioSphereKinematics& sphereKinematics,
                                         std::shared_ptr<grid_map::SignedDistanceField> sdfPtr)
    : StateConstraint(ConstraintOrder::Linear),
      sphereKinematicsPtr_(sphereKinematics.clone()),
      sdfPtr_(std::move(sdfPtr)),
      numConstraints_(sphereKinematicsPtr_->getPinocchioSphereInterface().getNumSpheresInTotal()) {}

SphereSdfConstraint::SphereSdfConstraint(const SphereSdfConstraint& rhs)
    : StateConstraint(rhs),
      sphereKinematicsPtr_(rhs.sphereKinematicsPtr_->clone()),
      sdfPtr_(rhs.sdfPtr_),
      numConstraints_(rhs.numConstraints_) {}

vector_t SphereSdfConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& preComp) const {
  vector_t value(numConstraints_);

  sphereKinematicsPtr_->setPinocchioInterface(cast<PerceptiveLeggedPrecomputation>(preComp).getPinocchioInterface());
  const auto positions = sphereKinematicsPtr_->getPosition(state);
  const auto& radii = sphereKinematicsPtr_->getPinocchioSphereInterface().getSphereRadii();
  for (size_t i = 0; i < numConstraints_; ++i) {
    value(i) = sdfPtr_->getDistanceAt(grid_map::Position3(positions[i])) - radii[i];
  }
  return value;
}

VectorFunctionLinearApproximation SphereSdfConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                              const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numConstraints_, state.size(), 0);
  approx.f = getValue(time, state, preComp);

  const auto positions = sphereKinematicsPtr_->getPosition(state);
  const auto sphereApprox = sphereKinematicsPtr_->getPositionLinearApproximation(state);
  for (size_t i = 0; i < numConstraints_; ++i) {
    approx.dfdx.row(i) = sdfPtr_->getDistanceGradientAt(grid_map::Position3(positions[i])).transpose() * sphereApprox[i].dfdx;
  }
  return approx;
}

}  // namespace legged
