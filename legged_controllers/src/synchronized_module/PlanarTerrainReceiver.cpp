#include "legged_controllers/synchronized_module/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace legged {

PlanarTerrainReceiver::PlanarTerrainReceiver(const rclcpp::Node::SharedPtr& node,
                                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                             std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr,
                                             const std::string& mapTopic, std::string sdfElevationLayer)
    : planarTerrainPtr_(std::move(planarTerrainPtr)),
      sdfPtr_(std::move(signedDistanceFieldPtr)),
      sdfElevationLayer_(std::move(sdfElevationLayer)) {
  subscriber_ = node->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      mapTopic, 1, std::bind(&PlanarTerrainReceiver::planarTerrainCallback, this, std::placeholders::_1));
}

void PlanarTerrainReceiver::preSolverRun(scalar_t, scalar_t, const vector_t&, const ReferenceManagerInterface&) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    updated_ = false;
    *planarTerrainPtr_ = planarTerrain_;
    sdfPtr_->~SignedDistanceField();
    new (sdfPtr_.get()) grid_map::SignedDistanceField();
    sdfPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, sdfElevationLayer_, signedDistanceHeightClearance_);
  }
}

void PlanarTerrainReceiver::planarTerrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  updated_ = true;

  planarTerrain_ = convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg));

  auto& elevationData = planarTerrain_.gridMap.get(sdfElevationLayer_);
  if (elevationData.hasNaN()) {
    const float inpaint{elevationData.minCoeffOfFinites()};
    RCLCPP_WARN(rclcpp::get_logger("PlanarTerrainReceiver"),
                "Map contains NaN values. Will apply inpainting with min value.");
    elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
  }
  const float heightMargin{0.1};
  signedDistanceHeightClearance_ = elevationData.maxCoeffOfFinites() + 3 * heightMargin;
}

}  // namespace legged
