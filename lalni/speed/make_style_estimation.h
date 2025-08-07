#ifndef PLANNER_SPEED_MAKE_STYLE_ESTIMATION_H_
#define PLANNER_SPEED_MAKE_STYLE_ESTIMATION_H_

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "common/type_def.h"
#include "make_style_estimation.h"
#include "math/double.h"
#include "mcts_game_process/mcts_data_type.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "planner_params.pb.h"
#include "speed/st_boundary_with_decision.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {
absl::Status MakeStyleEstimation(
    const SpeedPlanningParamsProto &speed_finder_params,
    const DiscretizedPath &path, const double av_velocity, const double av_acc,
    const SpacetimeTrajectoryManager &st_traj_mgr, const uint64_t seq_num,
    const absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>
        &agent_status_history,
    std::unordered_map<std::string, double> &obstacle_style_map);

}  // namespace e2e_noa::planning
#endif