#ifndef E2E_NOA_PLANNING_SPEED_SPEED_FINDER_OUTPUT
#define E2E_NOA_PLANNING_SPEED_SPEED_FINDER_OUTPUT

#include <optional>
#include <string>
#include <vector>

#include "descriptor/constraint_manager.h"
#include "object/partial_spacetime_object_trajectory.h"
#include "object/spacetime_object_trajectory.h"
#include "planner.pb.h"
#include "speed/st_boundary_with_decision.h"
#include "speed_planning.pb.h"

namespace e2e_noa::planning {

struct SpeedPlanningOutput {
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  SpeedPlanningDebugProto speed_planning_proto;
  ConstraintManager constraint_mgr;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;
  std::optional<std::string> alerted_front_vehicle;
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  std::map<std::string, ObjectSlInfo> obj_sl_map;
  std::string attention_obj_id = "";
  std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>
      agent_status_history;
};

}  // namespace e2e_noa::planning

#endif
