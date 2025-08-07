#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_FINDER_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_FINDER_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "common/type_def.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"

namespace e2e_noa::planning {
struct TrafficGap {
  absl::Span<const SpacetimeObjectTrajectory* const> leader_trajectories;
  absl::Span<const SpacetimeObjectTrajectory* const> follower_trajectories;
  double s_start;
  double s_end;
};

struct TrafficGapResult {
  std::optional<std::string> leader_id = std::nullopt;
  std::optional<std::string> follower_id = std::nullopt;
  std::optional<double> dec_gap_target_speed = std::nullopt;
  std::optional<double> dec_gap_target_a = std::nullopt;
  std::optional<double> acc_gap_target_speed = std::nullopt;
  std::optional<double> acc_gap_target_a = std::nullopt;
};

std::vector<TrafficGap> FindCandidateTrafficGapsOnLanePath(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    std::vector<std::string> lc_lead_obj_ids);

absl::StatusOr<TrafficGapResult> EvaluateAndTakeBestTrafficGap(
    absl::Span<const TrafficGap> candidate_gaps,
    const FrenetBox& ego_frenet_box, const FrenetFrame& target_frenet_frame,
    double ego_init_v, double speed_limit, double navi_dist, int lc_num,
    bool no_acc_gap, double leader_obj_s_min, double leader_obj_v,
    ad_e2e::planning::PushDirection push_dir);

}  // namespace e2e_noa::planning

#endif
