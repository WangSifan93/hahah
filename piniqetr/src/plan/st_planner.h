#ifndef ST_PLANNING_ST_PLANNER
#define ST_PLANNING_ST_PLANNER

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "async/thread_pool.h"
#include "autonomy_state.pb.h"
#include "behavior.pb.h"
#include "common/plan_start_point_info.h"
#include "common/planner_status.h"
#include "common/type_def.h"
#include "constraint.pb.h"
#include "decision_exploration/decision_exploration_output.h"
#include "decision_exploration/smooth_reference_line_result.h"
#include "driving_style.pb.h"
#include "ego_history.h"
#include "maps/lane_path.h"
#include "maps/map_def.h"
#include "object/object_history.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_trajectory_manager.h"
#include "perception.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/pnp_util.h"
#include "plan/st_planner_output.h"
#include "planner_object.pb.h"
#include "planner_params.pb.h"
#include "scene_understanding.pb.h"
#include "speed_planning_params.pb.h"
#include "trajectory_initialization.pb.h"
#include "trajectory_point.pb.h"

namespace e2e_noa {
namespace planning {
using PushDirection = ad_e2e::planning::PushDirection;
struct StPlannerInput {
  const ad_e2e::planning::Map* semantic_map_manager = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  int plan_id = 1;
  const VehicleParamsProto* vehicle_params = nullptr;

  const DeciderStateProto* decision_state = nullptr;
  const InitializationStateProto* initialization_state = nullptr;
  const TrajectoryOptimizerStateProto* trajectory_optimizer_state_proto =
      nullptr;
  const SpacetimePlannerObjectTrajectoriesProto*
      st_planner_object_trajectories = nullptr;

  const PlannerObjectController* obj_mgr = nullptr;
  const PlanStartPointInfo* start_point_info = nullptr;
  const StPathPlanStartPointInfo* st_path_start_point_info = nullptr;

  const ad_e2e::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  const std::optional<PNPInfos> pnp_infos = std::nullopt;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;

  bool enable_tl_ok_btn = false;
  bool override_passable = false;

  const RouteTargetInfo* route_target_info = nullptr;
  bool consider_lane_change_gap = true;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;

  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializationConfig* initialization_params = nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const SpeedPlanningParamsProto* speed_planning_params = nullptr;
  const SpacetimeConstraintParamsProto* spacetime_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;

  const SpeedPlanningParamsProto* speed_planning_lc_radical_params = nullptr;
  const SpeedPlanningParamsProto* speed_planning_lc_conservative_params =
      nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_radical_params =
      nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_normal_params =
      nullptr;
  const TrajectoryOptimizerParamsProto*
      trajectory_optimizer_lc_conservative_params = nullptr;
  const SpacetimePlannerObjectTrajectoriesParamsProto*
      spacetime_planner_object_trajectories_params = nullptr;
  const ILQGamesParamsProto* ilq_games_params = nullptr;

  const Behavior* behavior = nullptr;
  bool miss_navi_scenario = false;
  const ObjectHistoryController* obs_history = nullptr;
  const ad_e2e::planning::SpeedState* speed_state = nullptr;
  int cur_navi_lc_num = 0;
  double left_navi_dist = std::numeric_limits<double>::max();
  double left_navi_dist_v2 = std::numeric_limits<double>::max();
  ad_e2e::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2 =
      ad_e2e::planning::V2TurnInfo::V2DetailTurnType::NONE;
  const e2e_noa::LaneChangeStage& prev_lane_change_stage = LCS_NONE;
  const e2e_noa::DriverAction::LaneChangeCommand lc_cmd_state =
      DriverAction::LC_CMD_NONE;
  PushDirection push_dir = PushDirection::Push_None;

  const NudgeObjectInfo* nudge_object_info = nullptr;
  double cur_dist_to_junction = std::numeric_limits<double>::max();
  std::vector<std::string> lc_lead_obj_ids;
  bool is_open_gap = false;
  EgoFrame* curr_ego_frame;
  const EgoHistory* ego_history;
  const uint64_t seq_num = 0;
  std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>
      agent_status_history;
};

PlannerStatus StPlanner(
    const StPlannerInput& input, DecisionExplorationOutput decision_exploration,
    StPlannerOutput* st_output, StPlannerDebug* debug_info,
    WorkerThreadManager* thread_pool,
    std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>&
        agent_status_history);

}  // namespace planning
}  // namespace e2e_noa

#endif
