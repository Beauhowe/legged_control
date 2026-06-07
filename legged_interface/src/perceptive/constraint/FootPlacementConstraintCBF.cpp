#include "legged_interface/perceptive/constraint/FootPlacementConstraintCBF.h"

#include "legged_interface/perceptive/PerceptiveLeggedPrecomputation.h"
#include "legged_interface/perceptive/PerceptiveLeggedReferenceManager.h"

namespace legged {

FootPlacementConstraintCBF::FootPlacementConstraintCBF(const SwitchedModelReferenceManager& referenceManager,
                                                       const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                       size_t contactPointIndex, size_t numVertices, scalar_t cbfLambda)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      numVertices_(numVertices),
      lambda_(cbfLambda) {}

FootPlacementConstraintCBF::FootPlacementConstraintCBF(const FootPlacementConstraintCBF& rhs)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      numVertices_(rhs.numVertices_),
      lambda_(rhs.lambda_) {}

bool FootPlacementConstraintCBF::isActive(scalar_t time) const {
  return dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_).getFootPlacementFlags(time)[contactPointIndex_];
}

vector_t FootPlacementConstraintCBF::getValue(scalar_t, const vector_t& state, const vector_t& input,
                                              const PreComputation& preComp) const {
  const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  const auto velocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();
  const vector_t dotH = param.a * velocity;
  const vector_t h = param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;
  return dotH + lambda_ * h;
}

VectorFunctionLinearApproximation FootPlacementConstraintCBF::getLinearApproximation(scalar_t, const vector_t& state,
                                                                                     const vector_t& input,
                                                                                     const PreComputation& preComp) const {
  const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
  const auto velocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();

  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numVertices_, state.size(), input.size());
  approx.f = param.a * velocity + lambda_ * (param.a * positionApprox.f + param.b);
  approx.dfdx = param.a * velocityApprox.dfdx + lambda_ * param.a * positionApprox.dfdx;
  approx.dfdu = param.a * velocityApprox.dfdu;
  return approx;
}

}  // namespace legged
