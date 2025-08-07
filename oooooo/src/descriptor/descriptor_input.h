#ifndef ST_PLANNING_DECISION_DECIDER_INPUT
#define ST_PLANNING_DECISION_DECIDER_INPUT

#include <limits>
#include <string>

#include "behavior.pb.h"
#include "common/path_sl_boundary.h"
#include "common/type_def.h"
#include "decision_exploration/decision_exploration_output.h"
#include "lane_change.pb.h"
#include "maps/map_def.h"
#include "object/object_history.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_trajectory_manager.h"
#include "perception.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "scene_understanding.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {
using PushDirection = ad_e2e::planning::PushDirection;
struct DescriptorInput {
  int plan_id = 0;
  const e2e_noa::VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const SpacetimeConstraintParamsProto* spacetime_constraint_params = nullptr;
  const DecisionConstraintConfigProto* config = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const LaneChangeStateProto* lc_state;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  double target_offset_from_start = 0.0;
  const mapping::LanePath* lane_path_before_lc = nullptr;
  const PlanPassage* passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  bool borrow_lane_boundary = false;
  const PlannerObjectController* obj_mgr = nullptr;
  const ObjectHistoryController* obs_history = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;

  const ad_e2e::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const DeciderStateProto* pre_decision_state = nullptr;

  bool enable_tl_ok_btn = false;
  bool override_passable = false;

  double max_reach_length = std::numeric_limits<double>::max();
  double left_navi_dist_map = std::numeric_limits<double>::max();
  int lc_num = 0;

  absl::Time plan_time;

  const RouteTargetInfo* route_target_info = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  const Behavior* behavior = nullptr;
  const ad_e2e::planning::SpeedState* speed_state = nullptr;
  double cur_dist_to_junction = std::numeric_limits<double>::max();
  std::vector<std::string> lc_lead_obj_ids;
  PushDirection push_dir = PushDirection::Push_None;
  ad_e2e::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2 =
      ad_e2e::planning::V2TurnInfo::V2DetailTurnType::NONE;
};

}  // namespace e2e_noa::planning

#endif
