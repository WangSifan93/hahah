#ifndef E2E_NOA_PLANNING_SPEED_SPEED_DEBUG
#define E2E_NOA_PLANNING_SPEED_SPEED_DEBUG

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "plan/discretized_path.h"
#include "speed/mcts_game_process/mcts_debug.h"
#include "speed/speed_planning_output.h"
#include "speed/speed_vector.h"

namespace e2e_noa::planning {

void DumpSpeedToDebugFrame(
    const int plan_id, const uint64_t seq_num, const double path_length,
    const int trajectory_steps,
    const std::vector<MCTSInteractiveResult>& mcts_interactive_results,
    const SpeedPlanningParamsProto& speed_planning_params,
    const SpeedPlanningParamsProto& new_speed_planning_params,
    const DiscretizedPath& ego_predict_path,
    const std::map<std::string, prediction::ObjectPrediction>&
        predictions_debug,
    const SpeedVector& preliminary_speed, const SpeedVector& optimized_speed,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpeedPlanningOutput& output);

}

#endif
