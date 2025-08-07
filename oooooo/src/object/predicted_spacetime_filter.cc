#include "object/predicted_spacetime_filter.h"

#include <algorithm>
#include <cstdint>
#include <iterator>

#include "perception.pb.h"
#include "prediction/prediction.h"
#include "prediction/prediction_util.h"

namespace e2e_noa {
namespace planning {

namespace {
constexpr double kConfidentDuration = 1.0;

bool IsConfidentObjectType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_BARRIER:
    case OT_UNKNOWN_STATIC:
      return true;
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

bool IsConfidentPrediction(const prediction::PredictedTrajectory& pred) {
  constexpr double kConfidentTrajectoryProbability = 0.6;
  if (pred.probability() > kConfidentTrajectoryProbability) {
    return true;
  }
  return false;
}

Box2d ComputeConfidentStationaryObjectBox(const PlannerObject& object) {
  constexpr double kShrinkFactor = 0.6;
  auto box = object.bounding_box();
  box.LongitudinalExtendByRatio(kShrinkFactor);
  box.LateralExtendByRatio(kShrinkFactor);
  return box;
}

Box2d ComputeConfidentMovingObjectBox(
    const PlannerObject& object,
    const prediction::PredictedTrajectoryPoint& traj_point) {
  constexpr double kShrinkFactor = 0.6;
  const auto& box = object.bounding_box();
  return Box2d(box.half_length() * kShrinkFactor,
               box.half_width() * kShrinkFactor, traj_point.pos(),
               traj_point.theta());
}

std::vector<PredictedSpacetimeFilter::ObjectCollisionCheckingState>
ExtractConfidentStates(const PlannerObject& object,
                       const prediction::PredictedTrajectory& traj) {
  std::vector<PredictedSpacetimeFilter::ObjectCollisionCheckingState> states;

  if (prediction::IsStationaryTrajectory(traj)) {
    states.push_back({.id = object.id(),
                      .box = ComputeConfidentStationaryObjectBox(object)});
    return states;
  }

  if (traj.points().empty()) return states;

  for (const auto& point : traj.points()) {
    states.push_back({
        .id = object.id(),
        .box = ComputeConfidentMovingObjectBox(object, point),
    });
    if (point.t() > kConfidentDuration) break;
  }
  return states;
}

}  // namespace
PredictedSpacetimeFilter::PredictedSpacetimeFilter(
    absl::Span<const PlannerObject> planner_objects) {
  for (const auto& object : planner_objects) {
    if (!IsConfidentObjectType(object.type())) continue;

    for (const auto& traj : object.prediction().trajectories()) {
      if (!IsConfidentPrediction(traj)) continue;
      auto states = ExtractConfidentStates(object, traj);
      confident_object_states_.reserve(confident_object_states_.size() +
                                       states.size());
      for (auto it = std::make_move_iterator(states.begin()),
                end = std::make_move_iterator(states.end());
           it != end; ++it) {
        confident_object_states_.push_back(*it);
      }
    }
  }

  AABoxKDTreeParams params{
      .max_depth = 4,
      .max_leaf_size = 8,
  };
  kdtree_ = std::make_unique<AABoxKDTree2d<ObjectCollisionCheckingState>>(
      confident_object_states_, params);
}

FilterReason::Type PredictedSpacetimeFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  if (IsConfidentObjectType(object.type())) return FilterReason::NONE;
  if (IsStationaryTrajectory(traj)) return FilterReason::NONE;
  if (object.type() == OT_PEDESTRIAN) return FilterReason::NONE;

  const auto& points = traj.points();
  for (int i = 0, n = points.size(); i < n; ++i) {
    if (points[i].t() > kConfidentDuration) break;
    const auto box = ComputeConfidentMovingObjectBox(object, points[i]);
    auto* nearest = kdtree_->GetNearestObject(box.center());
    if (nearest != nullptr && nearest->box.HasOverlap(box)) {
      return FilterReason::TRAJECTORY_COLLIDES_WITH_CONFIDENT_OTHERS;
    }
  }
  return FilterReason::NONE;
}

}  // namespace planning
}  // namespace e2e_noa
