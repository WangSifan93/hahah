#ifndef PLANNER_SPEED_LON_GAME_MANAGER_H_
#define PLANNER_SPEED_LON_GAME_MANAGER_H_

#include <string_view>
#include <unordered_map>

#include "absl/status/status.h"
#include "async/thread_pool.h"
#include "mcts_data_type.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "planner_params.pb.h"
#include "speed/speed_limit_provider.h"
#include "speed/speed_vector.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

void ProcessMCTSLonGame(
    const StBoundaryWithDecision &boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario, const std::string &traj_id,
    const SpeedPlanningParamsProto::MCTSLonParamsProto *params,
    MCTSInteractiveResult &result);
}
}  // namespace e2e_noa
#endif
