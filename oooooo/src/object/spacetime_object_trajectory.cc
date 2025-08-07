#include "object/spacetime_object_trajectory.h"

#include <algorithm>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "object/planner_object.h"
#include "prediction/prediction_util.h"

namespace e2e_noa {
namespace planning {

SpacetimeObjectTrajectory::SpacetimeObjectTrajectory(
    const PlannerObject& planner_object, int traj_index,
    double required_lateral_gap)
    : traj_index_(traj_index),
      traj_id_(MakeTrajectoryId(planner_object.id(), traj_index)),
      planner_object_(planner_object),
      required_lateral_gap_(required_lateral_gap) {
  CHECK_GE(traj_index_, 0);
  CHECK_LT(traj_index_, planner_object.num_trajs());
  trajectory_ =
      prediction::PredictedTrajectory(planner_object.traj(traj_index_));
  states_ = ExtractTrajectoryStatePoints(
      trajectory_, planner_object_.pose().pos(), planner_object_.contour(),
      planner_object_.bounding_box());
  is_stationary_ = prediction::IsStationaryTrajectory(trajectory_) ||
                   planner_object_.is_stationary();
}

SpacetimeObjectTrajectory::SpacetimeObjectTrajectory(
    const SpacetimeObjectTrajectory& other) {
  *this = other;
}

SpacetimeObjectTrajectory& SpacetimeObjectTrajectory::operator=(
    const SpacetimeObjectTrajectory& other) {
  if (this != &other) {
    traj_index_ = other.traj_index_;
    traj_id_ = other.traj_id_;
    is_stationary_ = other.is_stationary_;
    planner_object_ = other.planner_object_;
    required_lateral_gap_ = other.required_lateral_gap_;
    trajectory_ = other.trajectory_;

    states_.clear();
    states_.reserve(other.states_.size());
    const auto& traj_points = trajectory_.points();
    for (int i = 0; i < other.states_.size(); ++i) {
      const auto& other_state = other.states_[i];
      states_.push_back(SpacetimeObjectState{.traj_point = &traj_points[i],
                                             .box = other_state.box,
                                             .contour = other_state.contour});
    }
  }
  return *this;
}

absl::StatusOr<SpacetimeObjectTrajectory>
SpacetimeObjectTrajectory::CreateTruncatedCopy(double start_offset,
                                               double horizon) const {
  if (states_.empty()) {
    return absl::FailedPreconditionError(
        absl::StrCat("The trajectory ", traj_id_, " has no states."));
  }
  std::vector<SpacetimeObjectState> truncated_states;
  truncated_states.reserve(states_.size());

  constexpr double kEps = 1e-6;
  const double start_t = states_[0].traj_point->t() + start_offset - kEps;
  for (int i = 0, n = states_.size(); i < n; ++i) {
    if (states_[i].traj_point->t() - start_t > horizon) {
      break;
    }
    if (states_[i].traj_point->t() >= start_t) {
      truncated_states.push_back(states_[i]);
    }
  }

  if (truncated_states.empty()) {
    return absl::NotFoundError(
        absl::StrCat("No trajectory left after truncating ", traj_id_,
                     " with horizon ", horizon));
  }
  return SpacetimeObjectTrajectory(planner_object_, std::move(truncated_states),
                                   traj_index_, required_lateral_gap_);
}

SpacetimeObjectTrajectory::SpacetimeObjectTrajectory(
    const PlannerObject& planner_object,
    std::vector<SpacetimeObjectState> states, int traj_index,
    double required_lateral_gap)
    : traj_index_(traj_index),
      traj_id_(MakeTrajectoryId(planner_object.id(), traj_index)),
      planner_object_(planner_object),
      required_lateral_gap_(required_lateral_gap),
      states_(std::move(states)) {
  CHECK_GE(traj_index_, 0);
  CHECK_LT(traj_index_, planner_object.num_trajs());
  trajectory_ =
      prediction::PredictedTrajectory(planner_object.traj(traj_index_));
  is_stationary_ = prediction::IsStationaryTrajectory(trajectory_) ||
                   planner_object.is_stationary();
}

SpacetimeObjectTrajectory::SpacetimeObjectTrajectory(
    const PlannerObject& planner_object, prediction::PredictedTrajectory traj,
    int traj_index, double required_lateral_gap)
    : traj_index_(traj_index),
      traj_id_(MakeTrajectoryId(planner_object.id(), traj_index)),
      planner_object_(planner_object),
      required_lateral_gap_(required_lateral_gap),
      trajectory_(std::move(traj)) {
  CHECK_GE(traj_index_, 0);
  CHECK_LT(traj_index_, planner_object.num_trajs());
  states_ = ExtractTrajectoryStatePoints(
      trajectory_, planner_object_.pose().pos(), planner_object_.contour(),
      planner_object_.bounding_box());
  is_stationary_ = prediction::IsStationaryTrajectory(trajectory_) ||
                   planner_object.is_stationary();
}

}  // namespace planning
}  // namespace e2e_noa
