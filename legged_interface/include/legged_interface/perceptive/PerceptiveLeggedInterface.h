//
// Perceptive extension point for the legged MPC interface.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <legged_interface/LeggedInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class PerceptiveLeggedInterface : public LeggedInterface {
 public:
  using LeggedInterface::LeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

 protected:
  void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                             bool verbose) override;

  void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                           bool verbose) override;

 public:
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }
  std::shared_ptr<grid_map::SignedDistanceField> getSignedDistanceFieldPtr() const { return signedDistanceFieldPtr_; }
  std::shared_ptr<PinocchioSphereInterface> getPinocchioSphereInterfacePtr() const { return pinocchioSphereInterfacePtr_; }
  size_t getNumVertices() const { return numVertices_; }

 protected:
  void setupDefaultPlanarTerrain();

  size_t numVertices_{16};

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr_;
  std::shared_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePtr_;
};

}  // namespace legged
