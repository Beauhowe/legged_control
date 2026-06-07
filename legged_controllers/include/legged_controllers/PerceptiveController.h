//
// Created by qiayuan on 23-1-3.
//

#pragma once

#include "legged_controllers/LeggedController.h"
#include "legged_controllers/visualization/FootPlacementVisualization.h"
#include "legged_controllers/visualization/SphereVisualization.h"

#include <visualization_msgs/msg/marker.hpp>

#include <limits>

namespace legged {

class PerceptiveController : public LeggedController {
 protected:
  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;
  void setupMpc() override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  void setupVisualization();
  void publishReferenceHeightVisualization(const SystemObservation& observation);

 private:
  std::shared_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
  std::shared_ptr<SphereVisualization> sphereVisualizationPtr_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr referenceHeightPublisher_;
  scalar_t fakeReferenceHeight_ = 0.58;
  scalar_t referenceHeightLastTime_ = std::numeric_limits<scalar_t>::lowest();
};

}  // namespace legged
