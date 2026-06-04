//
// Selects convex terrain regions for perceptive foothold planning.
//

#include "legged_interface/perceptive/ConvexRegionSelector.h"

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

namespace legged {

ConvexRegionSelector::ConvexRegionSelector(CentroidalModelInfo info,
                                           std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                           const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t numVertices)
    : info_(std::move(info)),
      numVertices_(numVertices),
      planarTerrainPtr_(std::move(planarTerrainPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()) {}

convex_plane_decomposition::PlanarTerrainProjection ConvexRegionSelector::getProjection(size_t leg, scalar_t time) const {
  if (timeEvents_[leg].empty()) {
    return {};
  }
  const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
  return feetProjections_[leg][index];
}

convex_plane_decomposition::CgalPolygon2d ConvexRegionSelector::getConvexPolygon(size_t leg, scalar_t time) const {
  if (timeEvents_[leg].empty()) {
    return {};
  }
  const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
  return convexPolygons_[leg][index];
}

vector3_t ConvexRegionSelector::getNominalFootholds(size_t leg, scalar_t time) const {
  if (timeEvents_[leg].empty()) {
    return vector3_t::Zero();
  }
  const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
  return nominalFootholds_[leg][index];
}

void ConvexRegionSelector::update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState,
                                  TargetTrajectories& targetTrajectories) {
  planarTerrain_ = *planarTerrainPtr_;
  if (planarTerrain_.planarRegions.empty()) {
    return;
  }

  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto contactFlagStocks = extractContactFlags(modeSequence);
  const size_t numPhases = modeSequence.size();

  feet_array_t<std::vector<int>> startIndices;
  feet_array_t<std::vector<int>> finalIndices;
  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    startIndices[leg] = std::vector<int>(numPhases, 0);
    finalIndices[leg] = std::vector<int>(numPhases, 0);
    for (size_t i = 0; i < numPhases; i++) {
      if (contactFlagStocks[leg][i]) {
        std::tie(startIndices[leg][i], finalIndices[leg][i]) = findIndex(i, contactFlagStocks[leg]);
      }
    }
  }

  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    feetProjections_[leg].assign(numPhases, {});
    convexPolygons_[leg].assign(numPhases, {});
    nominalFootholds_[leg].assign(numPhases, vector3_t::Zero());
    initStandFinalTime_[leg] = 0.0;

    scalar_t lastStandMiddleTime = std::numeric_limits<scalar_t>::quiet_NaN();
    for (size_t i = 0; i < numPhases; ++i) {
      if (!contactFlagStocks[leg][i]) {
        continue;
      }

      const int standStartIndex = startIndices[leg][i];
      const int standFinalIndex = finalIndices[leg][i];
      const scalar_t standStartTime = eventTimes[standStartIndex];
      const scalar_t standFinalTime = eventTimes[standFinalIndex];
      const scalar_t standMiddleTime = standStartTime + (standFinalTime - standStartTime) / 2.0;

      if (!std::isfinite(lastStandMiddleTime) || !numerics::almost_eq(standMiddleTime, lastStandMiddleTime)) {
        vector3_t footPos = getNominalFoothold(leg, standMiddleTime, initState, targetTrajectories);
        auto penaltyFunction = [](const vector3_t&) { return 0.0; };
        const auto projection = getBestPlanarRegionAtPositionInWorld(footPos, planarTerrain_.planarRegions, penaltyFunction);

        if (projection.regionPtr != nullptr) {
          const scalar_t growthFactor = 1.05;
          const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
              projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numVertices_, growthFactor);
          feetProjections_[leg][i] = projection;
          convexPolygons_[leg][i] = convexRegion;
        }
        nominalFootholds_[leg][i] = footPos;
        lastStandMiddleTime = standMiddleTime;
      } else if (i > 0) {
        feetProjections_[leg][i] = feetProjections_[leg][i - 1];
        convexPolygons_[leg][i] = convexPolygons_[leg][i - 1];
        nominalFootholds_[leg][i] = nominalFootholds_[leg][i - 1];
      }

      if (standStartTime < initTime && initTime < standFinalTime) {
        initStandFinalTime_[leg] = standFinalTime;
      }
    }
    timeEvents_[leg] = eventTimes;
  }
}

feet_array_t<std::vector<bool>> ConvexRegionSelector::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const {
  const size_t numPhases = phaseIDsStock.size();
  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < info_.numThreeDofContacts; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

std::pair<int, int> ConvexRegionSelector::findIndex(size_t index, const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();
  if (!contactFlagStock[index]) {
    return {0, 0};
  }

  int startTimesIndex = 0;
  for (int ip = static_cast<int>(index) - 1; ip >= 0; ip--) {
    if (!contactFlagStock[ip]) {
      startTimesIndex = ip;
      break;
    }
  }

  int finalTimesIndex = static_cast<int>(numPhases) - 2;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (!contactFlagStock[ip]) {
      finalTimesIndex = static_cast<int>(ip) - 1;
      break;
    }
  }
  return {startTimesIndex, finalTimesIndex};
}

vector3_t ConvexRegionSelector::getNominalFoothold(size_t leg, scalar_t time, const vector_t& initState,
                                                   TargetTrajectories& targetTrajectories) {
  const scalar_t height = 0.4;
  const vector_t desiredState = targetTrajectories.getDesiredState(time);
  const vector3_t desiredVel = centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
  const vector3_t measuredVel = centroidal_model::getNormalizedMomentum(initState, info_).head(3);

  const auto feedback = (vector3_t() << (std::sqrt(height / 9.81) * (measuredVel - desiredVel)).head(2), 0).finished();
  const vector_t zyx = centroidal_model::getBasePose(desiredState, info_).tail(3);
  const scalar_t offset = tan(-zyx(1)) * height;
  const vector3_t offsetVector(offset * cos(-zyx(1)), 0, offset * sin(-zyx(1)));
  matrix3_t R;
  const scalar_t z = zyx(0);
  R << cos(z), -sin(z), 0,
       sin(z),  cos(z), 0,
       0,       0,      1;
  return endEffectorKinematicsPtr_->getPosition(desiredState)[leg] - R.transpose() * offsetVector + feedback;
}

}  // namespace legged
