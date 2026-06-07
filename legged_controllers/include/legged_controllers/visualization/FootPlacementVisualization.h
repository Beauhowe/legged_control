//
// Created by qiayuan on 23-1-4.
//

#pragma once

#include <legged_interface/perceptive/ConvexRegionSelector.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace legged {
using namespace ocs2;

class FootPlacementVisualization {
 public:
  FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector, size_t numFoot, const rclcpp::Node::SharedPtr& node,
                             scalar_t maxUpdateFrequency = 20.0);

  void update(const SystemObservation& observation);

 private:
  visualization_msgs::msg::Marker to3dRosMarker(const convex_plane_decomposition::CgalPolygon2d& polygon,
                                                const Eigen::Isometry3d& transformPlaneToWorld,
                                                const std_msgs::msg::Header& header, Color color, float alpha, size_t i);

  scalar_t lineWidth_ = 0.008;
  scalar_t footMarkerDiameter_ = 0.02;
  std::vector<Color> feetColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple};

  const ConvexRegionSelector& convexRegionSelector_;
  size_t numFoot_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;
  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};
}  // namespace legged
