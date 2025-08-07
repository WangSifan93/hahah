#include "lon_game_manager.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/meta/type_traits.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "async/async_util.h"
#include "common/timer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/fast_math.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/box2d.h"
#include "math/geometry/halfplane.h"
#include "math/geometry/polygon2d.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "mcts_longitudinal.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "plan/planner_defs.h"
#include "plan/speed_profile.h"
#include "plan/trajectory_util.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction_defs.h"
#include "speed/decision/st_boundary_modifier_util.h"
#include "speed/empty_road_speed.h"
#include "speed/overlap_info.h"
#include "speed/speed_point.h"
#include "speed/st_boundary.h"
#include "speed/st_graph_data.h"
#include "speed/st_point.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa::planning {
namespace {
constexpr double kJerkBase = 1.0;
const std::vector<double> kLeaderActionDefault = {-2.0, -1.0, 0.0, 1.0, 2.0};
constexpr double kDefaultTimeLimit = 1.0;
struct MCTSLonSearchParams {
  bool av_is_leader = true;

  std::string av_id = "av";
  double av_velocity = 0.0;
  double av_acc = 0.0;
  double av_remain_dis = 0.0;
  double av_min_v = 0.0;
  double av_max_v = 13.9;

  std::string agent_id = "agent";
  double agent_velocity = 0.0;
  double agent_acc = 0.0;
  double agent_remain_dis = 0.0;
  double agent_min_v = 0.0;
  double agent_max_v = 13.9;

  double ucb_c = 2.0;
  double time_limit = 1.0;
  double time_step = 0.5;
  double simulate_jerk = 1.0;
};

bool UpdateMCTSLonSearchParams(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    MCTSLonSearchParams &params) {
  const auto *st_boundary = st_boundary_with_decision.st_boundary();
  if (nullptr == st_boundary || spacetime_obj.states().empty()) {
    return false;
  }
  const auto &overlap_info = st_boundary->overlap_infos().front();
  const int overlap_av_idx =
      (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;
  CHECK_GT(path.size(), overlap_av_idx);
  const double av_remain_dis = path[overlap_av_idx].s();
  const int overlap_agent_idx = overlap_info.obj_idx;
  CHECK_GT(spacetime_obj.states().size(), overlap_agent_idx);
  const auto *overlap_agent_traj_point =
      spacetime_obj.states()[overlap_agent_idx].traj_point;
  if (nullptr == overlap_agent_traj_point) {
    return false;
  }
  const double agent_remain_dis = overlap_agent_traj_point->s();
  const double start_t = overlap_info.time;
  const double agent_velocity = spacetime_obj.planner_object().pose().v();
  const double agent_acc = spacetime_obj.planner_object().pose().a();
  const double time_step = std::max(start_t * 0.25, 0.4);
  const auto traj_id = st_boundary_with_decision.traj_id();
  if (!traj_id.has_value()) {
    return false;
  }
  params.av_velocity = av_velocity;
  params.av_acc = av_acc;
  params.av_remain_dis = av_remain_dis;

  params.agent_id = traj_id.value();
  params.agent_velocity = agent_velocity;
  params.agent_acc = agent_acc;
  params.agent_remain_dis = agent_remain_dis;

  params.time_step = time_step;
  return true;
}

bool ProcessMCTSLonSearch(const std::vector<double> &leader_actions,
                          const MCTSLonSearchParams &search_params,
                          MCTSLongitudinal &mcts_game_lon,
                          std::vector<MCTSNodeConstPtr> &mcts_result,
                          MCTSDebugInfo &debug_info) {
  mcts_result.clear();

  NodeState node_state;
  if (search_params.av_is_leader) {
    node_state.leader_status =
        AgentStatus(search_params.av_id, 0.0, search_params.av_velocity,
                    search_params.av_acc, 0.0, search_params.av_remain_dis);
    node_state.follower_status = AgentStatus(
        search_params.agent_id, 0.0, search_params.agent_velocity,
        search_params.agent_acc, 0.0, search_params.agent_remain_dis);
    mcts_game_lon.InitOriginalSpeed(search_params.av_velocity,
                                    search_params.agent_velocity);
    mcts_game_lon.InitLeaderModel(
        search_params.av_min_v, search_params.av_max_v, search_params.time_step,
        search_params.simulate_jerk);
    mcts_game_lon.InitFollowerModel(
        search_params.agent_min_v, search_params.agent_max_v,
        search_params.time_step, search_params.simulate_jerk);
  } else {
    node_state.leader_status = AgentStatus(
        search_params.agent_id, 0.0, search_params.agent_velocity,
        search_params.agent_acc, 0.0, search_params.agent_remain_dis);
    node_state.follower_status =
        AgentStatus(search_params.av_id, 0.0, search_params.av_velocity,
                    search_params.av_acc, 0.0, search_params.av_remain_dis);
    mcts_game_lon.InitOriginalSpeed(search_params.agent_velocity,
                                    search_params.av_velocity);
    mcts_game_lon.InitLeaderModel(
        search_params.agent_min_v, search_params.agent_max_v,
        search_params.time_step, search_params.simulate_jerk);
    mcts_game_lon.InitFollowerModel(
        search_params.av_min_v, search_params.av_max_v, search_params.time_step,
        search_params.simulate_jerk);
  }

  MCTSNodePtr root = std::make_shared<MCTSNode>(node_state);
  mcts_game_lon.Search(leader_actions, search_params.ucb_c,
                       search_params.time_limit, search_params.time_step, root,
                       mcts_result, debug_info);
  return !mcts_result.empty();
}

bool ProcessMCTSLonGameInTurnLeft(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario,
    const SpeedPlanningParamsProto::MCTSLonParamsProto *params,
    std::vector<MCTSNodeConstPtr> &mcts_result, MCTSDebugInfo &debug_info) {
  if (nullptr == params) {
    return false;
  }

  MCTSLonSearchParams search_params;
  search_params.ucb_c = 2.0;
  search_params.time_limit = kDefaultTimeLimit;
  search_params.simulate_jerk = kJerkBase;
  if (!UpdateMCTSLonSearchParams(st_boundary_with_decision, spacetime_obj, path,
                                 av_velocity, av_acc, search_params)) {
    return false;
  }

  search_params.av_min_v = 0.0;
  search_params.av_max_v = std::max(9.72, search_params.av_velocity);
  search_params.agent_min_v = 0.0;
  search_params.agent_max_v = std::max(9.72, search_params.agent_velocity);

  search_params.av_is_leader = true;

  if (obj_scenario.relationship == Relationship::Cross ||
      obj_scenario.relationship == Relationship::Merge ||
      obj_scenario.relationship == Relationship::SameDir ||
      obj_scenario.relationship == Relationship::OnComing) {
    auto mcts_game_lon = std::make_unique<MCTSLongitudinal>(params);
    return ProcessMCTSLonSearch(kLeaderActionDefault, search_params,
                                *mcts_game_lon, mcts_result, debug_info);
  }
  return false;
}

bool ProcessMCTSLonGameInTurnRight(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario,
    const SpeedPlanningParamsProto::MCTSLonParamsProto *params,
    std::vector<MCTSNodeConstPtr> &mcts_result, MCTSDebugInfo &debug_info) {
  if (nullptr == params) {
    return false;
  }

  MCTSLonSearchParams search_params;
  search_params.ucb_c = 2.0;
  search_params.time_limit = kDefaultTimeLimit;
  search_params.simulate_jerk = kJerkBase;
  if (!UpdateMCTSLonSearchParams(st_boundary_with_decision, spacetime_obj, path,
                                 av_velocity, av_acc, search_params)) {
    return false;
  }

  search_params.av_min_v = 0.0;
  search_params.av_max_v = std::max(9.72, search_params.av_velocity);
  search_params.agent_min_v = 0.0;
  search_params.agent_max_v = std::max(9.72, search_params.agent_velocity);

  search_params.av_is_leader = true;

  if (obj_scenario.relationship == Relationship::Cross ||
      obj_scenario.relationship == Relationship::Merge ||
      obj_scenario.relationship == Relationship::SameDir ||
      obj_scenario.relationship == Relationship::OnComing) {
    auto mcts_game_lon = std::make_unique<MCTSLongitudinal>(params);
    return ProcessMCTSLonSearch(kLeaderActionDefault, search_params,
                                *mcts_game_lon, mcts_result, debug_info);
  }
  return false;
}

bool ProcessMCTSLonGameInJunctionStraight(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario,
    const SpeedPlanningParamsProto::MCTSLonParamsProto *params,
    std::vector<MCTSNodeConstPtr> &mcts_result, MCTSDebugInfo &debug_info) {
  if (nullptr == params) {
    return false;
  }

  MCTSLonSearchParams search_params;
  search_params.ucb_c = 2.0;
  search_params.time_limit = kDefaultTimeLimit;
  search_params.simulate_jerk = kJerkBase;
  if (!UpdateMCTSLonSearchParams(st_boundary_with_decision, spacetime_obj, path,
                                 av_velocity, av_acc, search_params)) {
    return false;
  }

  search_params.av_min_v = 0.0;
  search_params.av_max_v = std::max(13.9, search_params.av_velocity);
  search_params.agent_min_v = 0.0;
  search_params.agent_max_v = std::max(13.9, search_params.agent_velocity);

  search_params.av_is_leader = true;

  if (obj_scenario.relationship == Relationship::Cross ||
      obj_scenario.relationship == Relationship::Merge ||
      obj_scenario.relationship == Relationship::SameDir ||
      obj_scenario.relationship == Relationship::OnComing) {
    auto mcts_game_lon = std::make_unique<MCTSLongitudinal>(params);
    return ProcessMCTSLonSearch(kLeaderActionDefault, search_params,
                                *mcts_game_lon, mcts_result, debug_info);
  }
  return false;
}

bool ProcessMCTSLonGameInStraight(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario,
    const SpeedPlanningParamsProto::MCTSLonParamsProto *params,
    std::vector<MCTSNodeConstPtr> &mcts_result, MCTSDebugInfo &debug_info) {
  if (nullptr == params) {
    return false;
  }

  MCTSLonSearchParams search_params;
  search_params.ucb_c = 2.0;
  search_params.time_limit = kDefaultTimeLimit;
  search_params.simulate_jerk = kJerkBase;
  if (!UpdateMCTSLonSearchParams(st_boundary_with_decision, spacetime_obj, path,
                                 av_velocity, av_acc, search_params)) {
    return false;
  }

  search_params.av_min_v = 0.0;
  search_params.av_max_v = std::max(13.9, search_params.av_velocity);
  search_params.agent_min_v = 0.0;
  search_params.agent_max_v = std::max(13.9, search_params.agent_velocity);

  search_params.av_is_leader = true;

  if (obj_scenario.relationship == Relationship::Cross ||
      obj_scenario.relationship == Relationship::Merge ||
      obj_scenario.relationship == Relationship::SameDir ||
      obj_scenario.relationship == Relationship::OnComing) {
    auto mcts_game_lon = std::make_unique<MCTSLongitudinal>(params);
    return ProcessMCTSLonSearch(kLeaderActionDefault, search_params,
                                *mcts_game_lon, mcts_result, debug_info);
  }
  return false;
}
}  // namespace

void ProcessMCTSLonGame(
    const StBoundaryWithDecision &boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario, const std::string &traj_id,
    const SpeedPlanningParamsProto::MCTSLonParamsProto *params,
    MCTSInteractiveResult &result) {
  std::vector<MCTSNodeConstPtr> mcts_result;
  MCTSDebugInfo debug_info;
  if (InteractionZone::TurnLeft == obj_scenario.interaction_zone) {
    if (ProcessMCTSLonGameInTurnLeft(boundary_with_decision, spacetime_obj,
                                     path, av_velocity, av_acc, obj_scenario,
                                     params, mcts_result, debug_info)) {
      result.mcts_result = std::move(mcts_result);
      result.traj_id = traj_id;
      result.spacetime_obj = &spacetime_obj;
      result.st_boundary = boundary_with_decision.st_boundary();
      result.debug_info = std::move(debug_info);
      result.scenario_info = "left_turn";
    }
  } else if (InteractionZone::TurnRight == obj_scenario.interaction_zone) {
    if (ProcessMCTSLonGameInTurnRight(boundary_with_decision, spacetime_obj,
                                      path, av_velocity, av_acc, obj_scenario,
                                      params, mcts_result, debug_info)) {
      result.mcts_result = std::move(mcts_result);
      result.traj_id = traj_id;
      result.spacetime_obj = &spacetime_obj;
      result.st_boundary = boundary_with_decision.st_boundary();
      result.debug_info = std::move(debug_info);
      result.scenario_info = "right_turn";
    }
  } else if (InteractionZone::JunctionStraight ==
             obj_scenario.interaction_zone) {
    if (ProcessMCTSLonGameInJunctionStraight(
            boundary_with_decision, spacetime_obj, path, av_velocity, av_acc,
            obj_scenario, params, mcts_result, debug_info)) {
      result.mcts_result = std::move(mcts_result);
      result.traj_id = traj_id;
      result.spacetime_obj = &spacetime_obj;
      result.st_boundary = boundary_with_decision.st_boundary();
      result.debug_info = std::move(debug_info);
      result.scenario_info = "junction_straight";
    }
  } else if (InteractionZone::Straight == obj_scenario.interaction_zone) {
    if (ProcessMCTSLonGameInStraight(boundary_with_decision, spacetime_obj,
                                     path, av_velocity, av_acc, obj_scenario,
                                     params, mcts_result, debug_info)) {
      result.mcts_result = std::move(mcts_result);
      result.traj_id = traj_id;
      result.spacetime_obj = &spacetime_obj;
      result.st_boundary = boundary_with_decision.st_boundary();
      result.debug_info = std::move(debug_info);
      result.scenario_info = "straight";
    }
  }
}
}  // namespace e2e_noa::planning
