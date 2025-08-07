#ifndef ST_PLANNING_PLANNER_STATE
#define ST_PLANNING_PLANNER_STATE

#include <stdint.h>

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/time/time.h"
#include "alc.pb.h"
#include "autonomy_state.pb.h"
#include "common/planner_status.h"
#include "common/type_def.h"
#include "decision_exploration/smooth_reference_line_result.h"
#include "descriptor/turn_signal_descriptor.h"
#include "driving_style.pb.h"
#include "ego_history.h"
#include "lane_change.pb.h"
#include "lite_common.pb.h"
#include "maps/lane_path.h"
#include "maps/lane_point.h"
#include "math/vec.h"
#include "object/object_history.h"
#include "plan/planner_defs.h"
#include "planner.pb.h"
#include "planner_object.pb.h"
#include "planner_state.pb.h"
#include "router/route_sections.h"
#include "selector/selector_state.h"
#include "speed_planning.pb.h"
#include "trajectory.pb.h"
#include "trajectory_point.pb.h"
#include "util/hmi_content_util.h"

namespace e2e_noa::planning {
enum LaneChangeNotice {
  Notice_None = 0,
  Notice_Left_Waiting = 1,
  Notice_Right_Waiting = 2,
  Notice_Continuous_LC = 3,
  Notice_Miss_Navi = 4,
  Notice_LeTurn_Hard = 5,
  Notice_RiTurn_Hard = 6,
  Notice_Straight_Hard = 7,
  Notice_General_Hard = 8
};
struct PlannerState {
  LiteHeader header;

  PlannerStatusProto_PlannerStatusCode planner_status_code =
      PlannerStatusProto::OK;

  struct PosePoint {
    Vec2d pos = {0.0, 0.0};
    double theta = 0.0;
  };

  PosePoint input_ego_pose;

  TrajectoryProto previous_trajectory;

  LaneChangeStateProto lane_change_state;

  int tl_stop_interface = 0;

  AutonomyStateProto previous_autonomy_state;

  int version = 4;

  void Clear();

  void ClearHMIInfo();

  void FromProto(const PlannerStateProto& proto);

  void ToProto(PlannerStateProto* proto) const;

  bool Upgrade();

  bool operator==(const PlannerState& other) const;

  bool operator!=(const PlannerState& other) const { return !(*this == other); }

  std::string DebugString() const;

  mapping::LanePath prev_lane_path_before_lc;

  DeciderStateProto decision_state;

  InitializationStateProto initialization_state;

  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;

  mapping::LanePath prev_target_lane_path;
  double prev_length_along_route = std::numeric_limits<double>::max();
  double prev_max_reach_length = std::numeric_limits<double>::max();

  mapping::LanePoint station_anchor;

  mapping::LanePath preferred_lane_path;

  RouteSections prev_route_sections;

  bool prev_smooth_state = false;

  SelectorState selector_state;

  std::optional<TrajectoryOptimizerStateProto>
      selected_trajectory_optimizer_state_proto;

  SmoothedReferenceLineResultMap smooth_result_map;

  ObjectHistoryController object_history_manager;
  EgoHistory ego_history;

  std::vector<std::string> stalled_cars;
  std::vector<std::string> in_queue_cars;

  ALCState alc_state = ALCState::ALC_OFF;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  DriverAction::LaneChangeCommand lane_change_command =
      DriverAction::LC_CMD_NONE;
  std::optional<absl::Time> plc_prepare_start_time = std::nullopt;

  TurnSignalResult turn_signal_result;
  TurnSignal last_turn_signal_req = TurnSignal::TURN_SIGNAL_NONE;
  ad_e2e::planning::LaneChangeState output_lc_state =
      ad_e2e::planning::Lane_Keeping;

  DriverAction::LaneChangeCommand last_lc_command = DriverAction::LC_CMD_NONE;
  LaneChangeReason last_lc_reason = LaneChangeReason::NO_CHANGE;
  std::shared_ptr<PlannerSemanticMapManager> planner_semantic_map_manager =
      nullptr;
  int lc_command_number = 0;
  int lc_cancel_delay_number = 0;
  int lc_keeping_delay_number = 0;
  int last_manual_lc_time = 0;
  struct ObjectStopTimeResult {
    double time_duration_since_stop;

    double previous_stop_time_duration;

    double last_move_time_duration;
    double last_time;
  };
  ad_e2e::planning::SpeedState speed_state;
  absl::flat_hash_map<std::string, ObjectStopTimeResult> object_stop_time_map;
  std::optional<NudgeObjectInfo> nudge_object_info = std::nullopt;
  bool has_triggered_lc_notice = false;
  int lc_notice_pub_counter = 0;
  LaneChangeNotice lc_notice = LaneChangeNotice::Notice_None;
  bool if_continuous_lc = false;
  ad_e2e::planning::PushDirection lc_push_dir =
      ad_e2e::planning::PushDirection::Push_None;
  LaneChangeStyle last_lc_style = LC_STYLE_CONSERVATIVE;
  std::vector<std::string> lc_lead_obj_ids;

  double previous_cruising_speed_limit = 0.0;
  bool cruising_speed_limit_is_set = false;
  int auto_drive_counter = 0;
  uint64_t seq_num = 0;
  std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>
      agent_status_history;
};

}  // namespace e2e_noa::planning

#endif
