#ifndef PLANNER_SPEED_STANDSTILL_DISTANCE_DECIDER_H_
#define PLANNER_SPEED_STANDSTILL_DISTANCE_DECIDER_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "descriptor/constraint_manager.h"
#include "maps/lane_path.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "speed/st_boundary_with_decision.h"
#include "speed_planning_params.pb.h"

namespace e2e_noa {
namespace planning {

struct StandstillDistanceDescriptorInput {
  const SpeedPlanningParamsProto* speed_planning_params = nullptr;
  const absl::flat_hash_set<std::string>* stalled_object_ids = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const mapping::LanePath* lane_path = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  double extra_follow_standstill_for_large_vehicle = 0.0;
};

void DecideStandstillDistanceForStBoundary(
    const StandstillDistanceDescriptorInput& input,
    StBoundaryWithDecision* st_boundary_wd);

}  // namespace planning
}  // namespace e2e_noa

#endif
