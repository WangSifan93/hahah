#ifndef PLANNER_SPEED_INTERACTIVE_POST_PROCESSING_H_
#define PLANNER_SPEED_INTERACTIVE_POST_PROCESSING_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "async/thread_pool.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "planner_params.pb.h"
#include "speed/decision/post_st_boundary_modifier.h"
#include "speed/interactive_agent_process/game_theory/game_theory_environment.h"
#include "speed/speed_limit_provider.h"
#include "speed/speed_vector.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

void InteractivePostProcessing(
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<gt_result_t>& gt_results,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    double ego_acc_interval = 0.2);

}
}  // namespace e2e_noa
#endif
