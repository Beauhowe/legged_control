//
// Receives planar terrain updates for perceptive MPC.
//

#pragma once

#include <atomic>
#include <mutex>
#include <memory>
#include <string>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <rclcpp/rclcpp.hpp>

namespace legged {
using namespace ocs2;

class PlanarTerrainReceiver : public SolverSynchronizedModule {
 public:
  PlanarTerrainReceiver(rclcpp::Node::SharedPtr node, std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                        std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr, const std::string& mapTopic,
                        std::string elevationLayer);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override {}

 private:
  void planarTerrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr subscriber_;
  convex_plane_decomposition::PlanarTerrain planarTerrain_;
  std::string sdfElevationLayer_;

  std::mutex mutex_;
  std::atomic_bool updated_{false};

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
};

}  // namespace legged
