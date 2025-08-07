#ifndef ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "async/thread_pool.h"
#include "autonomy_state.pb.h"
#include "behavior.pb.h"
#include "common/plan_start_point_info.h"
#include "common/planner_status.h"
#include "constraint.pb.h"
#include "decision_exploration/decision_exploration_output.h"
#include "decision_exploration/smooth_reference_line_result.h"
#include "descriptor/constraint_manager.h"
#include "descriptor/descriptor_output.h"
#include "descriptor/leading_groups_builder.h"
#include "driving_style.pb.h"
#include "maps/lane_path.h"
#include "maps/map_def.h"
#include "object/object_history.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "object/spacetime_trajectory_manager.h"
#include "optimizer.pb.h"
#include "plan/discretized_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/pnp_util.h"
#include "planner_params.pb.h"
#include "scene_understanding.pb.h"
#include "trajectory_initialization.pb.h"
#include "trajectory_point.pb.h"
#include "util/hmi_content_util.h"
namespace e2e_noa::planning {
using PushDirection = ad_e2e::planning::PushDirection;
using LcReason = ad_e2e::planning::LcReason;
struct StPathPlannerInput {
  int plan_id = 0;
  const StPathPlanStartPointInfo* st_path_start_point_info = nullptr;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  const VehicleParamsProto* vehicle_params = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;

  DecisionExplorationOutput decision_exploration;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const std::optional<PNPInfos> pnp_infos = std::nullopt;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;

  const PlanStartPointInfo* start_point_info = nullptr;
  const RouteTargetInfo* route_target_info = nullptr;
  const PlannerObjectController* obj_mgr = nullptr;

  const ad_e2e::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const DeciderStateProto* prev_decision_state = nullptr;

  SpacetimePlannerObjectTrajectories init_st_planner_object_traj;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  Descriptor decision_output;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  const InitializationStateProto* prev_initialization_state = nullptr;
  const TrajectoryOptimizerStateProto* trajectory_optimizer_state_proto =
      nullptr;

  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializationConfig* initialization_params = nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const SpacetimeConstraintParamsProto* spacetime_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;

  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_radical_params =
      nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_normal_params =
      nullptr;
  const TrajectoryOptimizerParamsProto*
      trajectory_optimizer_lc_conservative_params = nullptr;
  const Behavior* behavior = nullptr;
  const ad_e2e::planning::SpeedState* speed_state = nullptr;
  bool miss_navi_scenario = false;
  const ObjectHistoryController* obs_history = nullptr;
  int cur_navi_lc_num = 0;
  double left_navi_dist = 999.0;
  double left_navi_dist_v2 = std::numeric_limits<double>::max();
  const e2e_noa::LaneChangeStage& prev_lane_change_stage = LCS_NONE;
  const DriverAction::LaneChangeCommand lc_cmd_state =
      DriverAction::LC_CMD_NONE;
  PushDirection push_dir = PushDirection::Push_None;
  const NudgeObjectInfo* nudge_object_info = nullptr;
  double cur_dist_to_junction = std::numeric_limits<double>::max();
};

struct StPathPlannerOutput {
  DecisionExplorationOutput decision_exploration;
  DiscretizedPath path;
  std::vector<PathPoint> st_path_points;
  ConstraintManager constraint_manager;
  absl::flat_hash_set<std::string> follower_set;
  absl::flat_hash_set<std::string> leader_set;
  double follower_max_decel = 0.0;
  absl::flat_hash_set<std::string> unsafe_object_ids;
  std::optional<NudgeObjectInfo> nudge_object_info;
  LeadingGroup leading_trajs;
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  InitializationDebugProto initialization_debug_proto;
  TrajectoryOptimizerDebugProto optimizer_debug_proto;
  DeciderStateProto decision_state;
  InitializationStateProto initialization_state;
  TrajectoryOptimizerStateProto trajectory_optimizer_state_proto;
  std::map<std::string, bool> obs_leading;
  PlannerStatusProto::PlannerStatusCode lc_status_code;
  bool is_init_follow_scene = false;
  std::string lc_lead_obj_id = "none";

  SpeedResponseStyle speed_response_style = SPEED_RESPONSE_NORMAL;
};

PlannerStatus RunStPathPlanner(StPathPlannerInput input,
                               StPathPlannerOutput* out,
                               WorkerThreadManager* thread_pool);

}  // namespace e2e_noa::planning

#endif
