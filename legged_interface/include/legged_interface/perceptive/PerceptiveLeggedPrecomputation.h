//
// Perceptive precomputation cache.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <legged_interface/LeggedRobotPreComputation.h>

#include "legged_interface/perceptive/ConvexRegionSelector.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class PerceptiveLeggedPrecomputation : public LeggedRobotPreComputation {
 public:
  struct FootPlacementParameter {
    matrix_t a;
    vector_t b;
  };

  PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info,
                                 const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                 const ConvexRegionSelector& convexRegionSelector);
  ~PerceptiveLeggedPrecomputation() override = default;

  PerceptiveLeggedPrecomputation* clone() const override { return new PerceptiveLeggedPrecomputation(*this); }
  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<FootPlacementParameter>& getFootPlacementConParameters() const { return footPlacementConParameters_; }

 protected:
  PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs);

 private:
  std::pair<matrix_t, vector_t> getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  const ConvexRegionSelector* convexRegionSelectorPtr_;
  std::vector<FootPlacementParameter> footPlacementConParameters_;
};

}  // namespace legged
