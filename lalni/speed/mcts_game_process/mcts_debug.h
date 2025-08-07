
#ifndef PLANNER_SPEED_MCTS_DEBUG_H_
#define PLANNER_SPEED_MCTS_DEBUG_H_
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "common/log_data.h"
#include "mcts_data_type.h"
#include "speed/st_boundary_with_decision.h"

namespace e2e_noa {
namespace planning {
void OnlineMCTSSimpleDebug(
    const std::vector<MCTSInteractiveResult> &interactive_results);

void OnlineMCTSDebug(
    const int plan_id, const uint64_t seq_num,
    const std::vector<MCTSInteractiveResult> &interactive_results);

void OfflineMCTSDebug(
    const int plan_id, const uint64_t seq_num,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const std::vector<MCTSInteractiveResult> &interactive_results);
}  // namespace planning
}  // namespace e2e_noa
#endif
