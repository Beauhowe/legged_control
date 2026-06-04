//
// Perceptive extension point for the legged MPC interface.
//

#include "legged_interface/perceptive/PerceptiveLeggedInterface.h"

#include "legged_interface/perceptive/ConvexRegionSelector.h"
#include "legged_interface/perceptive/PerceptiveLeggedPrecomputation.h"
#include "legged_interface/perceptive/PerceptiveLeggedReferenceManager.h"
#include "legged_interface/perceptive/constraint/FootPlacementConstraint.h"
#include "legged_interface/perceptive/constraint/SphereSdfConstraint.h"
#include "legged_interface/perceptive/constraint/FootCollisionConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

namespace legged {

void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                           const std::string& referenceFile, bool verbose) {
  setupDefaultPlanarTerrain();
  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings().contactNames3DoF[i];
    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

    auto footPlacementConstraint = std::make_unique<FootPlacementConstraint>(*referenceManagerPtr_, *eeKinematicsPtr, i, numVertices_);
    auto placementPenalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config(1e-2, 1e-4));
    problemPtr_->stateSoftConstraintPtr->add(
        footName + "_footPlacement",
        std::make_unique<StateSoftConstraint>(std::move(footPlacementConstraint), std::move(placementPenalty)));

    auto footCollisionConstraint =
        std::make_unique<FootCollisionConstraint>(*referenceManagerPtr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i, 0.03);
    auto collisionPenalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config(1e-2, 1e-3));
    problemPtr_->stateSoftConstraintPtr->add(
        footName + "_footCollision",
        std::make_unique<StateSoftConstraint>(std::move(footCollisionConstraint), std::move(collisionPenalty)));
  }

  const std::vector<std::string> collisionLinks = {"LF_calf", "RF_calf", "LH_calf", "RH_calf"};
  const std::vector<scalar_t> maxExcesses(collisionLinks.size(), 0.02);
  pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(*pinocchioInterfacePtr_, collisionLinks, maxExcesses, 0.6);

  CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo_);
  auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(*pinocchioSphereInterfacePtr_, pinocchioMapping);
  auto sphereSdfConstraint = std::make_unique<SphereSdfConstraint>(*sphereKinematicsPtr, signedDistanceFieldPtr_);
  auto spherePenalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config(1e-3, 1e-3));
  problemPtr_->stateSoftConstraintPtr->add(
      "sdfConstraint", std::make_unique<StateSoftConstraint>(std::move(sphereSdfConstraint), std::move(spherePenalty)));
}

void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                      const std::string& referenceFile, bool verbose) {
  auto swingTrajectoryPlanner =
      std::make_shared<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({modelSettings_.contactNames3DoF}, "ALL_FOOT");
  auto convexRegionSelector =
      std::make_shared<ConvexRegionSelector>(centroidalModelInfo_, planarTerrainPtr_, *eeKinematicsPtr, numVertices_);

  scalar_t comHeight = 0.0;
  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  referenceManagerPtr_.reset(new PerceptiveLeggedReferenceManager(centroidalModelInfo_, loadGaitSchedule(referenceFile, verbose),
                                                                  std::move(swingTrajectoryPlanner), std::move(convexRegionSelector),
                                                                  *eeKinematicsPtr, comHeight));
}

void PerceptiveLeggedInterface::setupPreComputation(const std::string& /*taskFile*/, const std::string& /*urdfFile*/,
                                                    const std::string& /*referenceFile*/, bool /*verbose*/) {
  problemPtr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_,
      *dynamic_cast<PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_).getConvexRegionSelectorPtr());
}

void PerceptiveLeggedInterface::setupDefaultPlanarTerrain() {
  constexpr double width = 5.0;
  constexpr double length = 5.0;
  const std::string elevationLayer = "elevation";

  planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();

  convex_plane_decomposition::PlanarRegion planarRegion;
  planarRegion.transformPlaneToWorld.setIdentity();
  planarRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(-length / 2.0, -width / 2.0, length / 2.0, width / 2.0);

  convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(length / 2.0, width / 2.0));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-length / 2.0, width / 2.0));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-length / 2.0, -width / 2.0));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(length / 2.0, -width / 2.0));
  planarRegion.boundaryWithInset.boundary = boundary;

  convex_plane_decomposition::CgalPolygonWithHoles2d inset;
  inset.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(length / 2.0 - 0.01, width / 2.0 - 0.01));
  inset.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-length / 2.0 + 0.01, width / 2.0 - 0.01));
  inset.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-length / 2.0 + 0.01, -width / 2.0 + 0.01));
  inset.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(length / 2.0 - 0.01, -width / 2.0 + 0.01));
  planarRegion.boundaryWithInset.insets.push_back(inset);

  planarTerrainPtr_->planarRegions.push_back(planarRegion);
  planarTerrainPtr_->gridMap.setFrameId("odom");
  planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(length, width), 0.03);
  planarTerrainPtr_->gridMap.add(elevationLayer, 0.0);
  planarTerrainPtr_->gridMap.add("smooth_planar", 0.0);

  signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>();
  signedDistanceFieldPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, elevationLayer, 0.1);
}

}  // namespace legged
