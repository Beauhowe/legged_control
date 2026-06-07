#pragma once

#include <memory>
#include <tuple>

#include "legged_interface/perceptive/ConvexRegionSelector.h"

#include <legged_interface/SwitchedModelReferenceManager.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class PerceptiveLeggedReferenceManager : public SwitchedModelReferenceManager {
 public:
  PerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                   std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                   std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics, scalar_t comHeight);

  const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() { return convexRegionSelectorPtr_; }
  contact_flag_t getFootPlacementFlags(scalar_t time) const;

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;
  virtual void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState, ModeSchedule& modeSchedule);
  scalar_t findMaxHeightAlongLine(const grid_map::GridMap& map, const std::string& layer,
                                  const grid_map::Position& startPoint, const grid_map::Position& endPoint) const;
  bool isPerceptionFreeSchedule(const ModeSchedule& modeSchedule) const;
  void modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex,
                         const std::vector<bool>& contactFlagStocks,
                         std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections);
  std::tuple<scalar_array_t, scalar_array_t, scalar_array_t> getHeights(
      const std::vector<bool>& contactFlagStocks, const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections,
      const grid_map::GridMap& map);

  const CentroidalModelInfo info_;
  feet_array_t<vector3_t> lastLiftoffPos_;
  std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  scalar_t comHeight_;
  bool perceptionReferenceActive_{true};
};
}  // namespace legged
