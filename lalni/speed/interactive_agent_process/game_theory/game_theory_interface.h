#ifndef PLANNER_SPEED_GAME_THEORY_INTERFACE_H_
#define PLANNER_SPEED_GAME_THEORY_INTERFACE_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "../interactive_agent.h"
#include "absl/status/status.h"
#include "async/thread_pool.h"
#include "game_theory_environment.h"
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

void GameTheoryEntry(std::vector<gt_result_t>& gt_results,
                     std::vector<InteractiveAgent>& interactive_agents,
                     InteractiveAgent& ego_interactive_info);
}
}  // namespace e2e_noa
#endif
