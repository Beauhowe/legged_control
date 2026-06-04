//
// Receives planar terrain updates for perceptive MPC.
//

#include "legged_controllers/perceptive/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>

#include <utility>

namespace legged {

PlanarTerrainReceiver::PlanarTerrainReceiver(rclcpp::Node::SharedPtr node,
                                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                             std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr,
                                             const std::string& mapTopic, std::string elevationLayer)
    : node_(std::move(node)),
      planarTerrainPtr_(std::move(planarTerrainPtr)),
      sdfPtr_(std::move(signedDistanceFieldPtr)),
      sdfElevationLayer_(std::move(elevationLayer)) {
  subscriber_ = node_->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      mapTopic, rclcpp::QoS(1), std::bind(&PlanarTerrainReceiver::planarTerrainCallback, this, std::placeholders::_1));
}

void PlanarTerrainReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/, const vector_t& /*currentState*/,
                                         const ReferenceManagerInterface& /*referenceManager*/) {
  if (updated_.exchange(false)) {
    std::lock_guard<std::mutex> lock(mutex_);
    *planarTerrainPtr_ = planarTerrain_;
    sdfPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, sdfElevationLayer_, 0.1);
  }
}

void PlanarTerrainReceiver::planarTerrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg) {
  convex_plane_decomposition::PlanarTerrain planarTerrain(convex_plane_decomposition::fromMessage(*msg));

  if (!planarTerrain.gridMap.exists(sdfElevationLayer_)) {
    RCLCPP_WARN(node_->get_logger(), "[PlanarTerrainReceiver] Received terrain map without layer `%s`.", sdfElevationLayer_.c_str());
    return;
  }

  auto& elevationData = planarTerrain.gridMap.get(sdfElevationLayer_);
  if (elevationData.hasNaN()) {
    const float inpaint = elevationData.minCoeffOfFinites();
    RCLCPP_WARN(node_->get_logger(), "[PlanarTerrainReceiver] Map contains NaN values. Inpainting with min finite value.");
    elevationData = elevationData.unaryExpr([=](float value) { return std::isfinite(value) ? value : inpaint; });
  }

  std::lock_guard<std::mutex> lock(mutex_);
  planarTerrain_ = std::move(planarTerrain);
  updated_ = true;
}

}  // namespace legged
