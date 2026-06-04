//
// Foot collision constraint using terrain signed distance field.
//

#include "legged_interface/perceptive/constraint/FootCollisionConstraint.h"

namespace legged {

FootCollisionConstraint::FootCollisionConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                 const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                 std::shared_ptr<grid_map::SignedDistanceField> sdfPtr, size_t contactPointIndex,
                                                 scalar_t clearance)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      sdfPtr_(std::move(sdfPtr)),
      contactPointIndex_(contactPointIndex),
      clearance_(clearance) {}

FootCollisionConstraint::FootCollisionConstraint(const FootCollisionConstraint& rhs)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      sdfPtr_(rhs.sdfPtr_),
      contactPointIndex_(rhs.contactPointIndex_),
      clearance_(rhs.clearance_) {}

bool FootCollisionConstraint::isActive(scalar_t time) const {
  constexpr scalar_t offset = 0.05;
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] &&
         !referenceManagerPtr_->getContactFlags(time + 0.5 * offset)[contactPointIndex_] &&
         !referenceManagerPtr_->getContactFlags(time - offset)[contactPointIndex_];
}

vector_t FootCollisionConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {
  vector_t value(1);
  const grid_map::Position3 position(endEffectorKinematicsPtr_->getPosition(state).front());
  value(0) = sdfPtr_->getDistanceAt(position) - clearance_;
  return value;
}

VectorFunctionLinearApproximation FootCollisionConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                  const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  const grid_map::Position3 position(positionApprox.f);
  approx.f = getValue(time, state, preComp);
  approx.dfdx = sdfPtr_->getDistanceGradientAt(position).transpose() * positionApprox.dfdx;
  return approx;
}

}  // namespace legged
