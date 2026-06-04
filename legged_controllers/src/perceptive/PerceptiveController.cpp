//
// Perceptive controller entry point.
//

#include <pinocchio/fwd.hpp>

#include "legged_controllers/perceptive/PerceptiveController.h"
#include "legged_controllers/perceptive/PlanarTerrainReceiver.h"

#include <legged_interface/perceptive/PerceptiveLeggedInterface.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

void PerceptiveController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                                const std::string& referenceFile, bool verbose) {
  leggedInterface_ = std::make_shared<PerceptiveLeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void PerceptiveController::setupMpc() {
  LeggedController::setupMpc();

  const auto perceptiveInterface = std::dynamic_pointer_cast<PerceptiveLeggedInterface>(leggedInterface_);
  if (perceptiveInterface == nullptr) {
    RCLCPP_ERROR(rosNode_->get_logger(), "[PerceptiveController] Expected PerceptiveLeggedInterface.");
    return;
  }

  const std::string terrainTopic = "/convex_plane_decomposition_ros/planar_terrain";
  const std::string elevationLayer = "elevation";
  auto planarTerrainReceiver = std::make_shared<PlanarTerrainReceiver>(rosNode_, perceptiveInterface->getPlanarTerrainPtr(),
                                                                       perceptiveInterface->getSignedDistanceFieldPtr(), terrainTopic,
                                                                       elevationLayer);
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
  RCLCPP_INFO(rosNode_->get_logger(), "[PerceptiveController] Subscribed planar terrain: %s", terrainTopic.c_str());
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::PerceptiveController, controller_interface::ControllerInterface)
