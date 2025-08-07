#ifndef AD_E2E_PLANNING_PLAN_PlannerWorld_H
#define AD_E2E_PLANNING_PLAN_PlannerWorld_H
#include <optional>
#include <string>
#include <vector>

#include "absl/types/optional.h"
#include "alc.pb.h"
#include "autonomy_state.pb.h"
#include "behavior.pb.h"
#include "common/plan_start_point_info.h"
#include "common/planner_status.h"
#include "common/plc_internal_result.h"
#include "decision_exploration/target_lane_path_filter.h"
#include "maps/map_def.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_trajectory_manager.h"
#include "perception.pb.h"
#include "plan/planner_state.h"
#include "plan/st_planner_output.h"
#include "planner_params.pb.h"
#include "pnp_info.pb.h"
#include "positioning.pb.h"
#include "router/route_manager_output.h"
#include "scene_understanding.pb.h"
#include "selector/selector_input.h"
#include "selector/selector_state.h"
#include "vehicle.pb.h"
namespace e2e_noa {
namespace planning {

struct PlannerWorldInput {
  const PlannerParamsProto* planner_params = nullptr;
  const VehicleParamsProto* vehicle_params = nullptr;
  absl::Duration min_path_look_ahead_duration;

  const std::optional<PlanStartPointInfo> start_point_info = std::nullopt;
  const std::optional<Vec2d> ego_pos = std::nullopt;
  const mapping::LanePath* prev_target_lane_path = nullptr;
  DriverAction::LaneChangeCommand new_lc_command = DriverAction::LC_CMD_NONE;
  bool auto_model = true;

  const std::optional<RouteSections> route_sections_from_start = std::nullopt;
  const std::optional<RouteSections> route_sections_from_current = std::nullopt;
  const absl::flat_hash_set<mapping::ElementId> avoid_lanes{};
  const std::optional<const SpacetimeTrajectoryManager> st_traj_mgr =
      std::nullopt;
  const std::optional<const PlannerObjectController> object_manager =
      std::nullopt;
  const std::optional<absl::flat_hash_set<std::string>> stalled_objects =
      std::nullopt;
  const std::optional<SceneOutputProto> scene_reasoning = std::nullopt;
  const std::optional<std::vector<ApolloTrajectoryPointProto>>
      time_aligned_prev_traj = std::nullopt;
  const std::optional<ad_e2e::planning::TrafficLightStatusMap>
      traffic_light_status_map = std::nullopt;
  bool consider_lane_change_gap = true;
  const SelectorState* selector_state = nullptr;
  const std::optional<const ObjectsProto> objects_proto = std::nullopt;
  const std::optional<PNPInfos> pnp_infos = std::nullopt;
  bool enable_tl_ok_btn = false;
  bool override_passable = false;
  std::optional<double> cruising_speed_limit = std::nullopt;
  const std::optional<e2e_noa::Behavior> behavior = std::nullopt;
  const PoseProto pose_proto;
};

struct PlannerWorldOutput {
  std::vector<PlannerStatus> st_status_list;
  std::vector<StPlannerOutput> st_planner_output_list;
  std::vector<StPlannerDebug> st_planner_debug_list;

  std::optional<RouteTargetInfo> route_target_info;

  PnpTop1History pnp_top1_history;
  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  DriverAction::LaneChangeCommand input_lc_cmd = DriverAction::LC_CMD_NONE;
  LcFeasibility lc_unable_reason = LcFeasibility::FEASIBILITY_OK;
  std::optional<PlcInternalResult> plc_result = std::nullopt;
  ALCState alc_state = ALCState::ALC_OFF;

  std::optional<NudgeObjectInfo> nudge_object_info;

  int tl_stop_interface = 0;
  ad_e2e::planning::SpeedState speed_state;
  bool if_lc_to_congestion = false;
  bool has_passed_this_junction = false;
  bool if_cancel_lc = false;
  LaneChangeNotice lc_notice = LaneChangeNotice::Notice_None;
  bool if_continuous_lc = false;
  ad_e2e::planning::PushDirection lc_push_dir =
      ad_e2e::planning::PushDirection::Push_None;
  LaneChangeStyle last_lc_style = LC_STYLE_CONSERVATIVE;
  std::optional<FrenetCoordinate> start_point_frenet = std::nullopt;
  std::optional<FrenetCoordinate> ego_pose_frenet = std::nullopt;
  EgoFrame curr_selected_ego_frame;
  TurnSignal turn_type_signal = TURN_SIGNAL_NONE;
  std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>
      agent_status_history;
};

struct PlannerWorld {
  PlannerWorldInput planner_world_input;
  PlannerWorldOutput planner_world_output;

  const PlannerWorldInput& input_ref() const { return planner_world_input; }

  PlannerWorldOutput* output_pt() { return &planner_world_output; }

  const PlannerWorldOutput& output_ref() const { return planner_world_output; }
};

}  // namespace planning
}  // namespace e2e_noa

#endif
