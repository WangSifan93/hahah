#ifndef PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_
#define PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "constraint.pb.h"
#include "descriptor/traffic_gap_finder.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "router/plan_passage.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/vt_speed_limit.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct DescriptorInput {
  const SpeedPlanningParamsProto::StBoundaryPreDeciderParamsProto* params =
      nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const absl::flat_hash_set<std::string>* follower_set = nullptr;
  const TrafficGapResult* lane_change_gap = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const DiscretizedPath* path = nullptr;
  const VehicleGeometryParamsProto* vehicle_params = nullptr;
  const PlanPassage* plan_passage = nullptr;
  double current_v = 0.0;
  double current_s = 0.0;
  double max_v = 0.0;
  double time_step = 0.0;
  int trajectory_steps = 0;
  absl::Time plan_time;
  bool run_act_net_speed_decision = false;
};

void MakePreDecisionForStBoundaries(
    const DescriptorInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::optional<VtSpeedLimit>* speed_limit);

}  // namespace planning
}  // namespace e2e_noa

#endif
