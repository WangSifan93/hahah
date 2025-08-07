#ifndef ST_PLANNING_SCHEDULER_SCHEDULER_INPUT
#define ST_PLANNING_SCHEDULER_SCHEDULER_INPUT

#include <memory>
#include <string>
#include <vector>

#include "autonomy_state.pb.h"
#include "behavior.pb.h"
#include "common/lane_path_info.h"
#include "decision_exploration/smooth_reference_line_result.h"
#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "object/object_history.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/navi/route_navi_info.h"
#include "router/route_sections_info.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"
namespace e2e_noa::planning {
struct DecisionExplorationInput {
  const PlannerSemanticMapManager* psmm;
  const VehicleGeometryParamsProto* vehicle_geom;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const PlannerObjectController* obj_mgr;
  const ObjectHistoryController* obj_history_mgr;
  const std::vector<LanePathInfo>* lane_path_infos;
  double planning_horizon;
  const mapping::LanePoint* destination;
  bool prev_smooth_state = false;
  const ApolloTrajectoryPointProto* plan_start_point;
  const mapping::LanePoint* station_anchor;
  double start_route_s;
  const SmoothedReferenceLineResultMap* smooth_result_map;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const mapping::LanePath* prev_lane_path_before_lc_from_start = nullptr;
  const LaneChangeStateProto* prev_lc_state = nullptr;
  const RouteNaviInfo* route_navi_info = nullptr;
  std::optional<double> cruising_speed_limit = std::nullopt;
  Vec2d ego_pos;
  bool miss_navi_scenario = false;
  bool continuous_lc_scenario = false;

  const bool is_navi;
  const e2e_noa::DriverAction::LaneChangeCommand lc_cmd_state =
      DriverAction::LC_CMD_NONE;
  const Behavior* behavior = nullptr;
  int pnp_top1_reason = ad_e2e::planning::LC_REASON_NONE;
  ad_e2e::planning::PushDirection lc_push_dir =
      ad_e2e::planning::PushDirection::Push_None;
};

}  

#endif
