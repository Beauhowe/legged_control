#pragma once

#include <legged_interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class FootPlacementConstraintCBF final : public StateInputConstraint {
 public:
  FootPlacementConstraintCBF(const SwitchedModelReferenceManager& referenceManager,
                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                             size_t numVertices, scalar_t cbfLambda);

  ~FootPlacementConstraintCBF() override = default;
  FootPlacementConstraintCBF* clone() const override { return new FootPlacementConstraintCBF(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t) const override { return numVertices_; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  FootPlacementConstraintCBF(const FootPlacementConstraintCBF& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t contactPointIndex_;
  const size_t numVertices_;
  const scalar_t lambda_;
};
}  // namespace legged
