#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "behavior.pb.h"
#include "constraint.pb.h"
#include "maps/lane_path.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/speed_profile.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct TrafficLightDescriptor {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  std::vector<ConstraintProto::SpeedProfileProto> speed_profiles;
  TrafficLightDeciderStateProto traffic_light_decision_state;
};

absl::StatusOr<TrafficLightDescriptor> BuildTrafficLightConstraints(
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    const ad_e2e::planning::TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const TrafficLightDeciderStateProto& tld_state,
    const bool& enable_tl_ok_btn, const bool& override_passable,
    const DecisionConstraintConfigProto& config,
    const Behavior_FunctionId& map_func_id,
    const SpacetimeTrajectoryManager& st_traj_mgr, const int plan_id,
    const bool traffic_light_fun_enable, bool& need_prebrake_for_lka);

}  // namespace planning
}  // namespace e2e_noa

#endif
