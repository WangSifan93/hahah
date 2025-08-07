#ifndef PLANNER_SPEED_MCTS_GAME_MANAGER_H_
#define PLANNER_SPEED_MCTS_GAME_MANAGER_H_

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

void ProcessMCTSGame(
    const std::vector<InteractiveInput> &interactive_input_infos,
    const SpacetimeTrajectoryManager &st_traj_mgr, const DiscretizedPath &path,
    const double av_velocity, const double av_acc, const uint64_t seq_num,
    const SpeedPlanningParamsProto *params, WorkerThreadManager *thread_pool,
    std::vector<MCTSInteractiveResult> &interactive_results);

}
}  // namespace e2e_noa
#endif
