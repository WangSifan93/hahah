#include "descriptor/leading_groups_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "descriptor/descriptor_util.h"
#include "glog/logging.h"
#include "math/util.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/planner_defs.h"
#include "plan/planner_flags.h"
#include "prediction/predicted_trajectory.h"
#include "util/status_macros.h"

namespace e2e_noa::planning {

namespace {

struct LeadingObjectTrajectoryInfo {
  std::string traj_id;
  double front_s, rear_s;
};

bool HasEnteredSlBoundary(const PathSlBoundary& path_boundary,
                          const FrenetBox& fbox, bool lc_left) {
  const auto [boundary_left_l, boundary_right_l] =
      CalcSlBoundaries(path_boundary, fbox);
  const double l_center = path_boundary.QueryReferenceCenterL(fbox.center_s());

  constexpr double kLateralEnterThres = 0.5;
  return lc_left ? fbox.l_min < boundary_left_l - kLateralEnterThres &&
                       fbox.l_max > l_center
                 : fbox.l_max > boundary_right_l + kLateralEnterThres &&
                       fbox.l_min < l_center;
}

bool ShouldConsiderInteraction(const SpacetimeObjectTrajectory& traj,
                               double ego_heading,
                               const FrenetBox& ego_frenet_box,
                               const PathSlBoundary& path_boundary,
                               const PlanPassage& plan_passage, bool lc_left,
                               LeadingObjectTrajectoryInfo* traj_info) {
  const auto& states = traj.states();
  if (std::abs(NormalizeAngle(ego_heading -
                              states.front().traj_point->theta())) > M_PI_2) {
    return false;
  }

  ASSIGN_OR_RETURN(const auto fbox,
                   plan_passage.QueryFrenetBoxAt(states.front().box), false);
  if (fbox.s_min <= ego_frenet_box.s_max) return false;
  if (HasEnteredSlBoundary(path_boundary, fbox, lc_left)) {
    traj_info->traj_id = traj.traj_id().data();
    traj_info->front_s = fbox.s_max;
    traj_info->rear_s = fbox.s_min;
    return true;
  }

  constexpr int kEvalStep = 1;
  constexpr int kEnterSRangeStep = static_cast<int>(1.0 / kTrajectoryTimeStep);
  bool is_valid = false;
  int projected_states = 1, along_path_states = 0;
  double front_s{0}, rear_s{0};
  for (int i = 1; i < states.size(); i += kEvalStep) {
    if (!is_valid && i > kEnterSRangeStep) {
      return false;
    }

    ASSIGN_OR_CONTINUE(const auto fbox,
                       plan_passage.QueryFrenetBoxAt(states[i].box));
    ++projected_states;
    if (!is_valid) {
      is_valid = true;
      front_s = fbox.s_max;
      rear_s = fbox.s_min;
    }
    if (HasEnteredSlBoundary(path_boundary, fbox, lc_left)) {
      ++along_path_states;
    }
  }

  constexpr double kMoveAlongPathPercentageThreshold = 0.5;
  const double on_path_ratio = static_cast<double>(along_path_states) /
                               static_cast<double>(projected_states);
  if (on_path_ratio > kMoveAlongPathPercentageThreshold) {
    traj_info->traj_id = {traj.traj_id().data(), traj.traj_id().size()};
    traj_info->front_s = front_s;
    traj_info->rear_s = rear_s;
    return true;
  }
  return false;
}

}  

std::vector<LeadingGroup> FindMultipleLeadingGroups(
    const PlanPassage& plan_passage, const PathSlBoundary& path_boundary,
    bool lc_left, const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects, double ego_heading,
    const FrenetBox& ego_frenet_box,
    const VehicleGeometryParamsProto& vehicle_geom) {
  std::vector<LeadingGroup> leading_groups;

  const auto& considered_trajectories = st_traj_mgr.trajectories();
  std::vector<LeadingObjectTrajectoryInfo> filtered_trajs;
  for (const auto& traj : considered_trajectories) {
    VLOG(3) << "Consider traj: " << traj.traj_id();
    const auto object_type = traj.planner_object().type();
    LeadingObjectTrajectoryInfo traj_info;
    if (object_type == ObjectType::OT_CYCLIST ||
        object_type == ObjectType::OT_TRICYCLIST)
      continue;
    if (IsLeadingObjectType(object_type) &&
        ShouldConsiderInteraction(traj, ego_heading, ego_frenet_box,
                                  path_boundary, plan_passage, lc_left,
                                  &traj_info)) {
      filtered_trajs.push_back(traj_info);
    }
  }
  std::stable_sort(filtered_trajs.begin(), filtered_trajs.end(),
                   [](const LeadingObjectTrajectoryInfo& traj1,
                      const LeadingObjectTrajectoryInfo& traj2) {
                     return traj1.rear_s < traj2.rear_s;
                   });

  LeadingGroup traj_group;
  const double min_gap = vehicle_geom.length() * 2.0;
  double previous_s = std::numeric_limits<double>::lowest();
  for (const auto& traj : filtered_trajs) {
    if (traj.front_s >= path_boundary.end_s()) break;

    if (!traj_group.empty()) {
      const double current_gap = traj.rear_s - previous_s;
      if (current_gap >= min_gap) {
        traj_group.swap(leading_groups.emplace_back());
      }
    }

    if (!stalled_objects.contains(
            SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
                traj.traj_id))) {
      traj_group.emplace(
          traj.traj_id,
          CreateLeadingObject(
              *st_traj_mgr.FindTrajectoryById(traj.traj_id), plan_passage,
              ConstraintProto::LeadingObjectProto::LANE_CHANGE_TARGET,
              traj_group.empty()));
    }
    previous_s = traj.front_s;

    if (leading_groups.size() ==
        FLAGS_planner_spacetime_search_max_multi_traj_num - 1) {
      break;
    }
  }
  if (!traj_group.empty()) {
    leading_groups.push_back(std::move(traj_group));
  }
  return leading_groups;
}

}  
