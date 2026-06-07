#include "legged_interface/perceptive/PerceptiveLeggedReferenceManager.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <grid_map_core/iterators/LineIterator.hpp>

namespace legged {

PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager(CentroidalModelInfo info,
                                                                   std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                   std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                                   std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                   scalar_t comHeight)
    : SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
      info_(std::move(info)),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      comHeight_(comHeight) {}

void PerceptiveLeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  perceptionReferenceActive_ = !isPerceptionFreeSchedule(modeSchedule);
  if (!perceptionReferenceActive_) {
    SwitchedModelReferenceManager::modifyReferences(initTime, finalTime, initState, targetTrajectories, modeSchedule);
    return;
  }

  TargetTrajectories newTargetTrajectories;
  int nodeNum = 11;
  for (size_t i = 0; i < nodeNum; ++i) {
    scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
    vector_t state = targetTrajectories.getDesiredState(time);
    vector_t input = targetTrajectories.getDesiredState(time);

    const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
    vector_t pos = centroidal_model::getBasePose(state, info_).head(3);

    scalar_t step = 0.3;
    grid_map::Vector3 normalVector;
    normalVector(0) = (map.atPosition("smooth_planar", pos + grid_map::Position(-step, 0)) -
                       map.atPosition("smooth_planar", pos + grid_map::Position(step, 0))) /
                      (2 * step);
    normalVector(1) = (map.atPosition("smooth_planar", pos + grid_map::Position(0, -step)) -
                       map.atPosition("smooth_planar", pos + grid_map::Position(0, step))) /
                      (2 * step);
    normalVector(2) = 1;
    normalVector.normalize();
    matrix3_t R;
    scalar_t z = centroidal_model::getBasePose(state, info_)(3);
    R << cos(z), -sin(z), 0,
         sin(z), cos(z), 0,
         0, 0, 1;
    vector_t v = R.transpose() * normalVector;
    centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());

    centroidal_model::getBasePose(state, info_)(2) = map.atPosition("smooth_planar", pos) + comHeight_;

    newTargetTrajectories.timeTrajectory.push_back(time);
    newTargetTrajectories.stateTrajectory.push_back(state);
    newTargetTrajectories.inputTrajectory.push_back(input);
  }
  targetTrajectories = newTargetTrajectories;

  convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);
  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
}

bool PerceptiveLeggedReferenceManager::isPerceptionFreeSchedule(const ModeSchedule& modeSchedule) const {
  if (modeSchedule.modeSequence.empty()) {
    return false;
  }
  for (const auto mode : modeSchedule.modeSequence) {
    if (mode != ModeNumber::STANCE) {
      return false;
    }
  }
  return true;
}

void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                    ModeSchedule& modeSchedule) {
  const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence;

  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);
    auto projections = convexRegionSelectorPtr_->getProjections(leg);
    modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

    scalar_array_t liftOffHeights, touchDownHeights, maxHeights;
    std::tie(liftOffHeights, touchDownHeights, maxHeights) =
        getHeights(contactFlagStocks[leg], projections, convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap);
    liftOffHeightSequence[leg] = liftOffHeights;
    touchDownHeightSequence[leg] = touchDownHeights;
    maxHeightSequence[leg] = maxHeights;
  }
  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);
}

void PerceptiveLeggedReferenceManager::modifyProjections(
    scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex, const std::vector<bool>& contactFlagStocks,
    std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections) {
  if (contactFlagStocks[initIndex]) {
    lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
    lastLiftoffPos_[leg].z() -= 0.02;
    for (int i = initIndex; i < static_cast<int>(projections.size()); ++i) {
      if (!contactFlagStocks[i]) {
        break;
      }
      projections[i].positionInWorld = lastLiftoffPos_[leg];
    }
    for (int i = initIndex; i >= 0; --i) {
      if (!contactFlagStocks[i]) {
        break;
      }
      projections[i].positionInWorld = lastLiftoffPos_[leg];
    }
  }
  if (initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg]) {
    for (int i = initIndex; i >= 0; --i) {
      if (contactFlagStocks[i]) {
        projections[i].positionInWorld = lastLiftoffPos_[leg];
      }
      if (!contactFlagStocks[i] && !contactFlagStocks[i + 1]) {
        break;
      }
    }
  }
}

scalar_t PerceptiveLeggedReferenceManager::findMaxHeightAlongLine(const grid_map::GridMap& map, const std::string& layer,
                                                                          const grid_map::Position& startPoint,
                                                                          const grid_map::Position& endPoint) const {
  if (!map.exists(layer) || !map.isInside(startPoint) || !map.isInside(endPoint)) {
    return std::numeric_limits<scalar_t>::quiet_NaN();
  }

  scalar_t maxHeight = std::max(map.atPosition(layer, startPoint), map.atPosition(layer, endPoint));
  for (grid_map::LineIterator iterator(map, startPoint, endPoint); !iterator.isPastEnd(); ++iterator) {
    const scalar_t height = map.at(layer, *iterator);
    if (std::isfinite(height)) {
      maxHeight = std::max(maxHeight, height);
    }
  }
  return maxHeight;
}

std::tuple<scalar_array_t, scalar_array_t, scalar_array_t> PerceptiveLeggedReferenceManager::getHeights(
    const std::vector<bool>& contactFlagStocks, const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections,
    const grid_map::GridMap& map) {
  scalar_array_t liftOffHeights, touchDownHeights, maxHeights;
  const size_t numPhases = projections.size();

  liftOffHeights.clear();
  liftOffHeights.resize(numPhases);
  touchDownHeights.clear();
  touchDownHeights.resize(numPhases);
  maxHeights.clear();
  maxHeights.resize(numPhases);

  std::vector<vector3_t> liftOffProjections(numPhases, vector3_t::Zero());
  std::vector<vector3_t> touchDownProjections(numPhases, vector3_t::Zero());

  for (size_t i = 1; i < numPhases; ++i) {
    if (!contactFlagStocks[i]) {
      liftOffHeights[i] = contactFlagStocks[i - 1] ? projections[i - 1].positionInWorld.z() : liftOffHeights[i - 1];
      liftOffProjections[i] = contactFlagStocks[i - 1] ? projections[i - 1].positionInWorld : liftOffProjections[i - 1];
    }
  }
  for (int i = numPhases - 2; i >= 0; --i) {
    if (!contactFlagStocks[i]) {
      touchDownHeights[i] = contactFlagStocks[i + 1] ? projections[i + 1].positionInWorld.z() : touchDownHeights[i + 1];
      touchDownProjections[i] = contactFlagStocks[i + 1] ? projections[i + 1].positionInWorld : touchDownProjections[i + 1];
    }
  }

  for (size_t i = 0; i < numPhases; ++i) {
    if (!contactFlagStocks[i]) {
      const scalar_t pathMaxHeight = findMaxHeightAlongLine(map, "elevation", liftOffProjections[i].head(2), touchDownProjections[i].head(2));
      maxHeights[i] = std::isfinite(pathMaxHeight) ? pathMaxHeight : std::max(liftOffHeights[i], touchDownHeights[i]);
    } else {
      maxHeights[i] = liftOffHeights[i];
    }
  }
  return {liftOffHeights, touchDownHeights, maxHeights};
}

contact_flag_t PerceptiveLeggedReferenceManager::getFootPlacementFlags(scalar_t time) const {
  contact_flag_t flag;
  if (!perceptionReferenceActive_) {
    flag.fill(false);
    return flag;
  }
  const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
  for (int i = 0; i < flag.size(); ++i) {
    flag[i] = getContactFlags(time)[i] && time >= finalTime[i];
  }
  return flag;
}

}  // namespace legged
