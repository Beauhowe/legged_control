//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <cmath>
#include <memory>
#include <mutex>

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <grid_map_core/GridMap.hpp>

using namespace legged;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
std::mutex TERRAIN_MUTEX;
std::shared_ptr<convex_plane_decomposition::PlanarTerrain> LATEST_TERRAIN;
constexpr auto PLANAR_TERRAIN_TOPIC = "/convex_plane_decomposition_ros/planar_terrain";

bool sampleHeight(const grid_map::GridMap& map, const std::string& layer, scalar_t x, scalar_t y, scalar_t& height) {
  try {
    height = map.atPosition(layer, grid_map::Position(x, y), grid_map::InterpolationMethods::INTER_LINEAR);
    if (std::isfinite(height)) {
      return true;
    }
  } catch (const std::out_of_range&) {
  }

  try {
    height = map.atPosition(layer, grid_map::Position(x, y), grid_map::InterpolationMethods::INTER_NEAREST);
    return std::isfinite(height);
  } catch (const std::out_of_range&) {
    return false;
  }
}

bool sampleTerrain(const grid_map::GridMap& map, const vector_t& pose, scalar_t& height, scalar_t& gradientX, scalar_t& gradientY) {
  const std::string layer = map.exists("smooth_planar") ? "smooth_planar" : "elevation";
  if (!map.exists(layer) || !sampleHeight(map, layer, pose(0), pose(1), height)) {
    return false;
  }

  const scalar_t step = std::max<scalar_t>(map.getResolution(), 0.03);
  scalar_t heightXp = height;
  scalar_t heightXm = height;
  scalar_t heightYp = height;
  scalar_t heightYm = height;
  sampleHeight(map, layer, pose(0) + step, pose(1), heightXp);
  sampleHeight(map, layer, pose(0) - step, pose(1), heightXm);
  sampleHeight(map, layer, pose(0), pose(1) + step, heightYp);
  sampleHeight(map, layer, pose(0), pose(1) - step, heightYm);

  gradientX = (heightXp - heightXm) / (2.0 * step);
  gradientY = (heightYp - heightYm) / (2.0 * step);
  return true;
}

vector_t adaptPoseToTerrain(const vector_t& pose) {
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> terrain;
  {
    std::lock_guard<std::mutex> lock(TERRAIN_MUTEX);
    terrain = LATEST_TERRAIN;
  }
  if (!terrain) {
    return pose;
  }

  scalar_t height = 0.0;
  scalar_t gradientX = 0.0;
  scalar_t gradientY = 0.0;
  if (!sampleTerrain(terrain->gridMap, pose, height, gradientX, gradientY)) {
    return pose;
  }

  vector_t adapted = pose;
  adapted(2) = height + COM_HEIGHT;

  const scalar_t yaw = pose(3);
  const scalar_t slopeHeading = gradientX * std::cos(yaw) + gradientY * std::sin(yaw);
  const scalar_t slopeLateral = -gradientX * std::sin(yaw) + gradientY * std::cos(yaw);
  adapted(4) = -std::atan(slopeHeading);
  adapted(5) = std::atan(slopeLateral);
  return adapted;
}
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = adaptPoseToTerrain(observation.state.segment<6>(6));
  vector_t adaptedTargetPose = adaptPoseToTerrain(targetPose);
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), adaptedTargetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT;
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(robotName + "_target");
  // Get node parameters
  const auto referenceFile = node->declare_parameter<std::string>("referenceFile", "");
  const auto taskFile = node->declare_parameter<std::string>("taskFile", "");

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  auto terrainSub = node->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      PLANAR_TERRAIN_TOPIC, 1, [](const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg) {
        auto terrain = std::make_shared<convex_plane_decomposition::PlanarTerrain>(convex_plane_decomposition::fromMessage(*msg));
        std::lock_guard<std::mutex> lock(TERRAIN_MUTEX);
        LATEST_TERRAIN = std::move(terrain);
      });
  (void)terrainSub;

  TargetTrajectoriesPublisher target_pose_command(node, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
