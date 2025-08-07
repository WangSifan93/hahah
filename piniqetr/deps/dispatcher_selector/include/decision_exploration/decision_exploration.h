#ifndef PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_
#define PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "autonomy_state.pb.h"
#include "behavior.pb.h"
#include "common/lane_path_info.h"
#include "decision_exploration/decision_exploration_input.h"
#include "decision_exploration/decision_exploration_output.h"
#include "decision_exploration/smooth_reference_line_result.h"
#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/navi/route_navi_info.h"
#include "router/plan_passage.h"
#include "router/route_sections_info.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"
namespace e2e_noa::planning {

absl::StatusOr<std::vector<DecisionExplorationOutput>>
DecisionExplorationPlanTasks(const DecisionExplorationInput& input,
                             const std::vector<LanePathInfo>& target_lp_infos,
                             WorkerThreadManager* thread_pool);

absl::StatusOr<DecisionExplorationOutput> MakeDecisionExplorationOutput(
    const PlannerSemanticMapManager& psmm,
    const std::vector<LanePathInfo>& lp_infos, PlanPassage plan_passage,
    const LanePathInfo& lp_info, const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlannerObjectController& obj_mgr,
    const ObjectHistoryController& obj_history_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const LaneChangeStateProto& prev_lc_state, bool borrow, bool should_smooth,
   Behavior behavior, bool is_miss_navi = false,
    bool is_continuous_lc = false,
    const e2e_noa::DriverAction::LaneChangeCommand lc_cmd_state =
        DriverAction::LC_CMD_NONE,
    int pnp_top1_reason = ad_e2e::planning::LC_REASON_NONE,
    ad_e2e::planning::PushDirection lc_push_dir =
        ad_e2e::planning::PushDirection::Push_None);

bool CheckRoadBoundaryBySInterval(
    const DecisionExplorationInput& input,
    const DecisionExplorationOutput& output,
    const ad_e2e::planning::LaneSequencePtr& current_lane_seq,
    const ad_e2e::planning::LaneSequencePtr& target_lane_seq);

}  

#endif
