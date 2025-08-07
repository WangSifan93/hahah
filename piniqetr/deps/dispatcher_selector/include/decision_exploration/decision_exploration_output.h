#ifndef E2E_NOA_PLANNING_SCHEDULER_SCHEDULER_OUTPUT
#define E2E_NOA_PLANNING_SCHEDULER_SCHEDULER_OUTPUT

#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/hash/hash.h"
#include "common/path_sl_boundary.h"
#include "common/type_def.h"
#include "lane_change.pb.h"
#include "math/frenet_frame.h"
#include "object/spacetime_trajectory_manager.h"
#include "router/plan_passage.h"
#include "turn_signal.pb.h"
namespace e2e_noa::planning {

struct DecisionExplorationOutput {
  bool is_fallback = false;
  bool is_expert = false;
  PlanPassage plan_passage;

  PathSlBoundary sl_boundary;

  double target_offset_from_start = 0.0;
  LaneChangeStateProto lane_change_state;
  int pnp_top1_reason = ad_e2e::planning::LC_REASON_NONE;

  mapping::LanePath lane_path_before_lc;
  double length_along_route = std::numeric_limits<double>::max();
  double max_reach_length = std::numeric_limits<double>::max();
  int lc_num = 0;
  double standard_congestion_factor = 0.0;
  double traffic_congestion_factor = 0.0;
  bool should_smooth = false;
  bool borrow_lane = false;
  FrenetBox av_frenet_box_on_plan_passage;

  bool request_help_lane_change_by_route = false;
  bool switch_alternate_route = false;
  TurnSignal planner_turn_signal = TURN_SIGNAL_NONE;
  TurnSignalReason turn_signal_reason = TURN_SIGNAL_OFF;
  bool miss_navi_scenario = false;

  using HashType = std::tuple<mapping::ElementId, bool, bool>;
  HashType Hash() const {
    return std::make_tuple(plan_passage.lane_path().front().lane_id(),
                           borrow_lane, is_fallback);
  }
};

struct RouteTargetInfo {
  int plan_id;
  KdTreeFrenetCoordinate frenet_frame;
  FrenetBox ego_frenet_box;

  PlanPassage plan_passage;
  PathSlBoundary sl_boundary;
  SpacetimeTrajectoryManager st_traj_mgr;
};

}  

#endif
