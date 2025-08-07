#ifndef ONBOARD_PLANNER_ROUTER_PLAN_PASSAGE_BUILDER_H_
#define ONBOARD_PLANNER_ROUTER_PLAN_PASSAGE_BUILDER_H_

#include <optional>

#include "absl/status/statusor.h"
#include "maps/lane_path.h"
#include "maps/lane_point.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"

namespace e2e_noa::planning {

absl::StatusOr<PlanPassage> BuildPlanPassage(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePath& backward_extended_lane_path,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    const mapping::LanePoint& destination, bool all_lanes_virtual = false,
    std::optional<double> override_speed_limit_mps = std::nullopt,
    FrenetFrameType type = FrenetFrameType::kQtfmKdTree,
    ad_e2e::planning::LaneSeqInfoPtr lane_seq_info = nullptr);

absl::StatusOr<PlanPassage> BuildPlanPassageFromLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step_s, bool avoid_loop, double backward_extend_len,
    double required_planning_horizon,
    std::optional<double> override_speed_limit_mps,
    FrenetFrameType type = FrenetFrameType::kQtfmKdTree);

}  

#endif
