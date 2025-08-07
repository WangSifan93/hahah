#include "plan/planner_state.h"

#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "container/strong_int.h"
#include "google/protobuf/message.h"
#include "math/geometry/util.h"
#include "planner_state.pb.h"
#include "smooth_reference_line.pb.h"
#include "traffic_light_info.pb.h"
#include "util/proto_util.h"
#include "util/time_util.h"

namespace e2e_noa::planning {

void PlannerState::Clear() {
  prev_target_lane_path = mapping::LanePath();
  station_anchor = mapping::LanePoint();
  prev_length_along_route = 0.0;
  prev_max_reach_length = 0.0;
  prev_smooth_state = false;
  decision_state.Clear();
  initialization_state.Clear();
  lane_change_state.Clear();
  prev_lane_path_before_lc = mapping::LanePath();
  st_planner_object_trajectories.Clear();

  nudge_object_info = std::nullopt;
  lc_lead_obj_ids.clear();
}

void PlannerState::ClearHMIInfo() {
  stalled_cars.clear();
  in_queue_cars.clear();
}

void PlannerState::FromProto(const PlannerStateProto& proto) {
  header = proto.header();

  previous_trajectory = proto.previous_trajectory();

  lane_change_state = proto.lane_change_state();

  previous_autonomy_state = proto.previous_autonomy_state();

  version = proto.version();

  decision_state = proto.decision_state();
  initialization_state = proto.initialization_state();

  if (proto.has_selected_trajectory_optimizer_state()) {
    selected_trajectory_optimizer_state_proto =
        proto.selected_trajectory_optimizer_state();
  } else {
    selected_trajectory_optimizer_state_proto = std::nullopt;
  }

  st_planner_object_trajectories = proto.st_planner_object_trajectories();

  prev_length_along_route = proto.prev_length_along_route();
  prev_max_reach_length = proto.prev_max_reach_length();
  station_anchor.FromProto(proto.station_anchor());

  prev_smooth_state = proto.prev_smooth_state();

  selector_state.FromProto(proto.selector_state());
}

void PlannerState::ToProto(PlannerStateProto* proto) const {
  proto->Clear();

  *proto->mutable_header() = header;

  *proto->mutable_previous_trajectory() = previous_trajectory;

  *proto->mutable_lane_change_state() = lane_change_state;

  *proto->mutable_previous_autonomy_state() = previous_autonomy_state;

  proto->set_version(version);

  *proto->mutable_decision_state() = decision_state;

  *proto->mutable_initialization_state() = initialization_state;

  prev_lane_path_before_lc.ToProto(proto->mutable_prev_lane_path_before_lc());
  *proto->mutable_st_planner_object_trajectories() =
      st_planner_object_trajectories;

  prev_target_lane_path.ToProto(proto->mutable_prev_target_lane_path());
  proto->set_prev_length_along_route(prev_length_along_route);
  proto->set_prev_max_reach_length(prev_max_reach_length);
  station_anchor.ToProto(proto->mutable_station_anchor());

  proto->set_prev_smooth_state(prev_smooth_state);
  proto->mutable_smooth_result_map()->mutable_lane_id_vec()->Reserve(
      smooth_result_map.smoothed_result_map().size());
  for (const auto& it : smooth_result_map.smoothed_result_map()) {
    auto* lane_id_vec = proto->mutable_smooth_result_map()->add_lane_id_vec();
    for (const auto id : it.first) {
      lane_id_vec->add_lane_id(id);
    }
  }

  preferred_lane_path.ToProto(proto->mutable_preferred_lane_path());

  if (selected_trajectory_optimizer_state_proto.has_value()) {
    *proto->mutable_selected_trajectory_optimizer_state() =
        (*selected_trajectory_optimizer_state_proto);
  } else {
    proto->mutable_selected_trajectory_optimizer_state()->Clear();
  }

  selector_state.ToProto(proto->mutable_selector_state());
}

bool operator==(const PlannerState::PosePoint& lhs,
                const PlannerState::PosePoint& rhs) {
  return lhs.pos.x() == rhs.pos.x() && lhs.pos.y() == rhs.pos.y() &&
         lhs.theta == rhs.theta;
}

bool PlannerState::operator==(const PlannerState& other) const {
  if (

      previous_trajectory != other.previous_trajectory ||

      !ProtoEquals(lane_change_state, other.lane_change_state) ||

      previous_autonomy_state != other.previous_autonomy_state ||

      prev_lane_path_before_lc != other.prev_lane_path_before_lc ||
      !ProtoEquals(initialization_state, other.initialization_state) ||
      !ProtoEquals(st_planner_object_trajectories,
                   other.st_planner_object_trajectories) ||

      prev_target_lane_path != other.prev_target_lane_path ||
      prev_length_along_route != other.prev_length_along_route ||
      station_anchor != other.station_anchor ||
      preferred_lane_path != other.preferred_lane_path ||
      prev_smooth_state != other.prev_smooth_state

  ) {
    return false;
  }

  return true;
}

bool PlannerState::Upgrade() {
  if (previous_trajectory.trajectory_start_timestamp() < 1E-6) {
    previous_trajectory.set_trajectory_start_timestamp(header.timestamp() /
                                                       1E6);
  }
  return true;
}

std::string PlannerState::DebugString() const {
  PlannerStateProto proto;
  ToProto(&proto);
  return proto.DebugString();
}

}  // namespace e2e_noa::planning
