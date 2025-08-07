#ifndef PLANNER_SPEED_DECIDER_PRE_ST_BOUNDARY_MODIFIER_H_
#define PLANNER_SPEED_DECIDER_PRE_ST_BOUNDARY_MODIFIER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "speed/path_semantic_analyzer.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "vehicle.pb.h"
namespace e2e_noa {
namespace planning {

struct PreStboundaryModifierInput {
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const StGraph* st_graph = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  double current_v = 0.0;
  double current_a = 0.0;
  const DiscretizedPath* path = nullptr;
  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const PlanPassage* plan_passage = nullptr;
  const absl::flat_hash_set<std::string>* follower_set = nullptr;
  const absl::flat_hash_set<std::string>* leader_set = nullptr;
  LaneChangeStage lc_stage;
  std::vector<StBoundaryWithDecision>* st_boundaries_with_decision;
};

void PreModifyStBoundaries(
    const PreStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    OpenLoopSpeedLimit* open_loop_speed_limit);

}  // namespace planning
}  // namespace e2e_noa

#endif
