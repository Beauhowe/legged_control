//
// Created by qiayuan on 23-1-3.
//

#include <pinocchio/fwd.hpp>

#include "legged_controllers/PerceptiveController.h"
#include "legged_controllers/synchronized_module/PlanarTerrainReceiver.h"

#include <legged_interface/perceptive/PerceptiveLeggedInterface.h>
#include <legged_interface/perceptive/PerceptiveLeggedReferenceManager.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

void PerceptiveController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                                const std::string& referenceFile, bool verbose) {
  leggedInterface_ = std::make_shared<PerceptiveLeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  setupVisualization();
}

void PerceptiveController::setupMpc() {
  LeggedController::setupMpc();

  auto& perceptiveInterface = dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_);
  auto planarTerrainReceiver = std::make_shared<PlanarTerrainReceiver>(
      rosNode_, perceptiveInterface.getPlanarTerrainPtr(), perceptiveInterface.getSignedDistanceFieldPtr(),
      "/convex_plane_decomposition_ros/planar_terrain", "elevation");
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

controller_interface::return_type PerceptiveController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto result = LeggedController::update(time, period);
  footPlacementVisualizationPtr_->update(currentObservation_);
  sphereVisualizationPtr_->update(currentObservation_);
  publishReferenceHeightVisualization(currentObservation_);
  return result;
}

void PerceptiveController::publishReferenceHeightVisualization(const SystemObservation& observation) {
  const scalar_t minPublishTimeDifference = 0.1;
  if (!referenceHeightPublisher_ || observation.time - referenceHeightLastTime_ <= minPublishTimeDifference) {
    return;
  }
  referenceHeightLastTime_ = observation.time;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = rosNode_->get_clock()->now();
  marker.ns = "Fake Reference Height";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.z = fakeReferenceHeight_;
  marker.scale.x = 4.0;
  marker.scale.y = 4.0;
  marker.scale.z = 0.01;
  marker.color.r = 0.1;
  marker.color.g = 0.8;
  marker.color.b = 0.4;
  marker.color.a = 0.18;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  referenceHeightPublisher_->publish(marker);
}

void PerceptiveController::setupVisualization() {
  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(
      *dynamic_cast<PerceptiveLeggedReferenceManager&>(*leggedInterface_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
      leggedInterface_->getCentroidalModelInfo().numThreeDofContacts, rosNode_);

  sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
      *dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPinocchioSphereInterfacePtr(), rosNode_);

  referenceHeightPublisher_ = rosNode_->create_publisher<visualization_msgs::msg::Marker>("fake_reference_height", 1);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::PerceptiveController, controller_interface::ControllerInterface)
