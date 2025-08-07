#ifndef ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_H_
#define ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "driving_style.pb.h"
#include "lane_change.pb.h"
#include "math/frenet_frame.h"
#include "object/spacetime_trajectory_manager.h"
#include "planner_status.pb.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {
using LeadingTrajs = std::vector<std::string>;

struct TrajEvalInfo {
  double eval_cost;
  absl::flat_hash_set<std::string> follower_set{};
  absl::flat_hash_set<std::string> leader_set{};
  double follower_max_decel = 0.0;
  double leader_max_decel = 0.0;

  std::string follower_max_id = "";
  std::string leader_max_id = "";

  double follower_risk_factor = 0.0;
  double leader_risk_factor = 0.0;

  std::ostringstream follower_debug_info;
  std::ostringstream leader_debug_info;

  std::string unsafe_object_id = "";
  PlannerStatusProto::PlannerStatusCode status_code = PlannerStatusProto::OK;
  bool enable_pause_counter = false;
};

bool HasEnteredTargetLane(const double center_l, const double half_width);

bool HasEnteredTargetLane(const FrenetBox& obj_box, const double lane_width);

bool HasFullyEnteredTargetLane(const double center_l, const double half_width);

bool HasFullyEnteredTargetLane(const FrenetBox& obj_box,
                               const double half_width);
bool HasFullyCenteredInTargetLane(const FrenetBox& obj_box,
                                  const double half_width);
absl::Status CheckLaneChangeSafety(
    const ApolloTrajectoryPointProto& start_point,
    const std::vector<ApolloTrajectoryPointProto>& ego_traj_pts,
    const LeadingTrajs& leading_trajs, const FrenetFrame& target_frenet_frame,
    double speed_limit, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geom, LaneChangeStyle lc_style,
    const e2e_noa::LaneChangeStage& lc_state,
    const e2e_noa::LaneChangeStage& prev_lc_stage,
    absl::Duration path_look_ahead_duration, TrajEvalInfo* eval_info);
}  // namespace e2e_noa::planning

#endif
