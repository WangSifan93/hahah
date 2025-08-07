#ifndef SPACETIME_SEARCH_INPUT
#define SPACETIME_SEARCH_INPUT

#include <memory>
#include <string>
#include <vector>

#include "behavior.pb.h"
#include "common/path_sl_boundary.h"
#include "common/plan_start_point_info.h"
#include "descriptor/constraint_manager.h"
#include "descriptor/leading_groups_builder.h"
#include "decision_exploration/decision_exploration_output.h"
#include "decision_exploration/smooth_reference_line_result.h"
#include "driving_style.pb.h"
#include "trajectory_initialization.pb.h"
#include "maps/map.h"
#include "object/object_history.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner.pb.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "spacetime_search/collision_checker.h"
#include "spacetime_search/spatio_graph/spatio_form_builder.h"
#include "spacetime_search/spatio_graph/spatio_graph.h"
#include "vehicle.pb.h"
namespace e2e_noa::planning {
using PushDirection = ad_e2e::planning::PushDirection;

enum class InitializationSceneType {
  INIT_SCENE_NONE = 0,
  INIT_SCENE_FOLLOW = 1,
  INIT_SCENE_NUDGE = 2,
  INIT_SCENE_BORROW = 3,
  INIT_SCENE_LANE_CHANGE = 4
};

struct SpacetimeSearchInput {
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const ApolloTrajectoryPointProto* start_point = nullptr;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  absl::Time plan_time;
  const PlanPassage* plan_passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
  const InitializationConfig* initialization_params = nullptr;
  const SpacetimeConstraintParamsProto* spacetime_constraint_params = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const SpatioGraph* geom_graph = nullptr;
  const SpatioFormBuilder* form_builder = nullptr;
  const CollisionChecker* collision_checker = nullptr;
  const std::vector<double>* stop_s_vec = nullptr;
  const std::vector<LeadingGroup>* leading_groups = nullptr;
  const ConstraintProto::LeadingObjectProto* blocking_static_traj = nullptr;
  double passage_speed_limit = 10.0;
  const InitializationSceneType init_scene_type;
  bool is_lane_change = false;
  bool eval_safety = false;
  LaneChangeStyle lc_style = LC_STYLE_NORMAL;
  LaneChangeStage lc_state = LCS_PAUSE;
  LaneChangeStage prev_lc_stage = LCS_NONE;
  PushDirection push_dir = PushDirection::Push_None;
  bool borrow_lane = false;
  LaneChangeReason lc_reason = LaneChangeReason::NO_CHANGE;
};

struct InitializationInput {
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const StPathPlanStartPointInfo* path_start_point_info = nullptr;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  const LaneChangeStateProto* lane_change_state = nullptr;
  const LaneChangeStage prev_lc_stage = LaneChangeStage::LCS_NONE;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const PlanPassage* plan_passage = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const InitializationStateProto* prev_initialization_state = nullptr;
  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializationConfig* initialization_params = nullptr;
  const SpacetimeConstraintParamsProto* spacetime_constraint_params = nullptr;
  const VehicleParamsProto* vehicle_params = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
  int plan_id = 0;
  const SceneOutputProto* scene_reasoning = nullptr;
  bool borrow_lane = false;
  const FrenetBox* av_frenet_box = nullptr;

  const PlanStartPointInfo* start_point_info = nullptr;
  const RouteTargetInfo* route_target_info = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  const PlannerObjectController* obj_mgr = nullptr;

  const ad_e2e::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const DeciderStateProto* prev_decision_state = nullptr;
  const ObjectHistoryController* obs_history = nullptr;
  const Behavior* behavior = nullptr;
  const ad_e2e::planning::SpeedState* speed_state = nullptr;
  int cur_lc_num = 0;
  double left_navi_dist = 999.0;
  PushDirection push_dir = PushDirection::Push_None;
  const NudgeObjectInfo* nudge_object_info = nullptr;
  LaneChangeReason lc_reason = LaneChangeReason::NO_CHANGE;
};

struct ReferenceLineSearcherInput {
  const SpatioGraph* spatio_graph = nullptr;
  const PlanPassage* plan_passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const InitializationConfig* initialization_params = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const VehicleDriveParamsProto* vehicle_drive = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
};

}  

#endif
