#include "vru_game_manager.h"

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
#include "mcts_spatiotemporal.h"
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

namespace e2e_noa {
namespace planning {

constexpr double kJerkBase = 1.0;
const std::vector<double> kLeaderActionDefault = {-2.0, -1.0, 0.0, 1.0, 2.0};
struct MCTSVruSearchParams {
  bool av_is_leader = true;

  std::string av_id = "av";
  double av_velocity = 0.0;
  double av_acc = 0.0;
  double av_remain_dis = 0.0;
  double av_min_v = 0.0;
  double av_max_v = 13.9;
  double av_heading = 0.0;
  double av_x = 0.0;
  double av_y = 0.0;

  std::string agent_id = "agent";
  double agent_velocity = 0.0;
  double agent_acc = 0.0;
  double agent_remain_dis = 0.0;
  double agent_min_v = 0.0;
  double agent_max_v = 6.0;
  double agent_heading = 0.0;
  double agent_x = 0.0;
  double agent_y = 0.0;
  double agent_length = 2.0;
  double agent_width = 1.0;

  double ucb_c = 2.0;
  double time_limit = 2.0;
  double time_step = 0.5;
  double simulate_jerk = 1.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
  double goal_heading = 0.0;
  double target_velocity = 0.0;
  StOverlapMetaProto::OverlapSource overlap_source =
      StOverlapMetaProto::UNKNOWN_SOURCE;
  StBoundaryProto::ObjectType object_type = StBoundaryProto::UNKNOWN_OBJECT;
};

bool UpdateMCTSVruSearchParams(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc, const double v_pedestrian,
    const double v_cyclist, MCTSVruSearchParams &params) {
  const auto *st_boundary = st_boundary_with_decision.st_boundary();
  if (nullptr == st_boundary || spacetime_obj.states().empty() ||
      st_boundary->overlap_infos().empty()) {
    DLOG(INFO)
        << "st_boundary is nullptr or st_boundary->overlap_infos().empty():"
        << " " << st_boundary->overlap_infos().empty() << " "
        << spacetime_obj.states().empty();
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
  const double agent_velocity = spacetime_obj.planner_object().pose().v();
  const double agent_acc = spacetime_obj.planner_object().pose().a();
  const double goal_heading =
      atan2(spacetime_obj.states().back().traj_point->pos().y() -
                spacetime_obj.states().front().traj_point->pos().y(),
            spacetime_obj.states().back().traj_point->pos().x() -
                spacetime_obj.states().front().traj_point->pos().x());
  const auto traj_id = st_boundary_with_decision.traj_id();
  if (!traj_id.has_value()) {
    return false;
  }
  params.av_velocity = av_velocity;
  params.av_acc = av_acc;
  params.av_remain_dis = av_remain_dis;
  params.av_heading = path.front().theta();
  params.av_x = path.front().x();
  params.av_y = path.front().y();

  params.agent_id = traj_id.value();
  params.agent_velocity = agent_velocity;
  params.agent_acc = agent_acc;
  params.agent_remain_dis = agent_remain_dis;
  params.agent_heading = spacetime_obj.states()[0].traj_point->theta();
  params.agent_x = spacetime_obj.planner_object().pose().pos().x();
  params.agent_y = spacetime_obj.planner_object().pose().pos().y();
  params.goal_x = overlap_agent_traj_point->pos().x();
  params.goal_y = overlap_agent_traj_point->pos().y();
  params.goal_heading = goal_heading;
  if (StBoundaryProto::PEDESTRIAN ==
      st_boundary_with_decision.st_boundary()->object_type()) {
    params.target_velocity = v_pedestrian;
  } else if (StBoundaryProto::CYCLIST ==
             st_boundary_with_decision.st_boundary()->object_type()) {
    params.target_velocity = v_cyclist;
  }
  params.agent_length = spacetime_obj.planner_object().bounding_box().length();
  params.agent_width = spacetime_obj.planner_object().bounding_box().width();
  params.overlap_source = st_boundary->overlap_meta()->source();
  params.object_type = st_boundary_with_decision.st_boundary()->object_type();
  return true;
}

bool ProcessMCTSVruSearch(const std::vector<double> &leader_actions,
                          const MCTSVruSearchParams &search_params,
                          MCTSSpatiotemporal &mcts_game_vru,
                          const DiscretizedPath &path,
                          std::vector<MCTSNodeConstPtr> &mcts_result,
                          MCTSDebugInfo &debug_info) {
  mcts_result.clear();
  constexpr double speed_lower = 0.2;
  constexpr double time_upper = 10.0;
  constexpr double remain_distance_lower = 15.0;
  if (search_params.av_velocity < speed_lower ||
      (search_params.av_remain_dis > remain_distance_lower &&
       (search_params.av_remain_dis / search_params.av_velocity) >
           time_upper)) {
    return false;
  }
  NodeState node_state;
  node_state.leader_status =
      AgentStatus(search_params.av_id, search_params.av_x, search_params.av_y,
                  search_params.av_heading, search_params.av_velocity,
                  search_params.av_acc, 0.0, 0.0, 0.0);
  node_state.follower_status = AgentStatus(
      search_params.agent_id, search_params.agent_x, search_params.agent_y,
      search_params.agent_heading, search_params.agent_velocity,
      search_params.agent_acc, 0.0, 0.0, 0.0);
  std::vector<const SpacetimeObjectTrajectory *> other_agents;
  const double theta_diff = std::fabs(
      NormalizeAngle(search_params.av_heading - search_params.agent_heading));
  mcts_game_vru.InitMCTSSpatiotemporal(
      search_params.av_min_v, search_params.av_max_v, search_params.time_step,
      search_params.simulate_jerk, search_params.av_velocity,
      search_params.agent_velocity, &path, search_params.target_velocity,
      search_params.goal_x, search_params.goal_y, search_params.goal_heading,
      search_params.av_x, search_params.av_y, search_params.agent_x,
      search_params.agent_y, search_params.agent_length,
      search_params.agent_width, theta_diff, search_params.overlap_source,
      search_params.object_type, other_agents);
  MCTSNodePtr root = std::make_shared<MCTSNode>(node_state);
  mcts_game_vru.Search(leader_actions, search_params.ucb_c,
                       search_params.time_limit, search_params.time_step, root,
                       mcts_result, debug_info);
  return !mcts_result.empty();
}

bool ProcessMCTSVruGameInTurnLeft(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario,
    const SpeedPlanningParamsProto::MCTSVruParamsProto *params,
    std::vector<MCTSNodeConstPtr> &mcts_result, MCTSDebugInfo &debug_info) {
  if (nullptr == params) {
    return false;
  }

  MCTSVruSearchParams search_params;
  search_params.ucb_c = 2.0;
  search_params.time_limit = 2.0;
  search_params.simulate_jerk = kJerkBase;
  if (!UpdateMCTSVruSearchParams(
          st_boundary_with_decision, spacetime_obj, path, av_velocity, av_acc,
          params->vru_config().v_pedestrian(), params->vru_config().v_cyclist(),
          search_params)) {
    return false;
  }

  search_params.av_min_v = 0.0;
  search_params.av_max_v = std::max(9.72, search_params.av_velocity);
  search_params.agent_min_v = 0.0;
  search_params.agent_max_v = std::max(4.16, search_params.agent_velocity);
  search_params.time_step = std::max(0.5, params->vru_config().delta_t());

  search_params.av_is_leader = true;

  if (obj_scenario.relationship == Relationship::Cross ||
      obj_scenario.relationship == Relationship::Merge ||
      obj_scenario.relationship == Relationship::SameDir ||
      obj_scenario.relationship == Relationship::OnComing) {
    auto mcts_game_vru = std::make_unique<MCTSSpatiotemporal>(params);
    mcts_game_vru->SetInteractionZone(obj_scenario.interaction_zone);
    mcts_game_vru->SetEgoPath(path);
    return ProcessMCTSVruSearch(kLeaderActionDefault, search_params,
                                *mcts_game_vru, path, mcts_result, debug_info);
  }

  return false;
}

bool ProcessMCTSVruGameInTurnRight(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario,
    const SpeedPlanningParamsProto::MCTSVruParamsProto *params,
    std::vector<MCTSNodeConstPtr> &mcts_result, MCTSDebugInfo &debug_info) {
  if (nullptr == params) {
    return false;
  }
  MCTSVruSearchParams search_params;
  search_params.ucb_c = 2.0;
  search_params.time_limit = 2.0;
  search_params.simulate_jerk = kJerkBase;
  if (!UpdateMCTSVruSearchParams(
          st_boundary_with_decision, spacetime_obj, path, av_velocity, av_acc,
          params->vru_config().v_pedestrian(), params->vru_config().v_cyclist(),
          search_params)) {
    return false;
  }

  search_params.av_min_v = 0.0;
  search_params.av_max_v = std::max(9.72, search_params.av_velocity);
  search_params.agent_min_v = 0.0;
  search_params.agent_max_v = std::max(4.16, search_params.agent_velocity);
  search_params.time_step = std::max(0.5, params->vru_config().delta_t());

  search_params.av_is_leader = true;

  if (obj_scenario.relationship == Relationship::Cross ||
      obj_scenario.relationship == Relationship::Merge ||
      obj_scenario.relationship == Relationship::SameDir ||
      obj_scenario.relationship == Relationship::OnComing) {
    auto mcts_game_vru = std::make_unique<MCTSSpatiotemporal>(params);
    mcts_game_vru->SetInteractionZone(obj_scenario.interaction_zone);
    mcts_game_vru->SetEgoPath(path);
    return ProcessMCTSVruSearch(kLeaderActionDefault, search_params,
                                *mcts_game_vru, path, mcts_result, debug_info);
  }
  return false;
}

bool ProcessMCTSVruGameInJunctionStraight(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario,
    const SpeedPlanningParamsProto::MCTSVruParamsProto *params,
    std::vector<MCTSNodeConstPtr> &mcts_result, MCTSDebugInfo &debug_info) {
  if (nullptr == params) {
    return false;
  }
  MCTSVruSearchParams search_params;
  search_params.ucb_c = 2.0;
  search_params.time_limit = 2.0;
  search_params.simulate_jerk = kJerkBase;
  if (!UpdateMCTSVruSearchParams(
          st_boundary_with_decision, spacetime_obj, path, av_velocity, av_acc,
          params->vru_config().v_pedestrian(), params->vru_config().v_cyclist(),
          search_params)) {
    return false;
  }

  search_params.av_min_v = 0.0;
  search_params.av_max_v = std::max(9.72, search_params.av_velocity);
  search_params.agent_min_v = 0.0;
  search_params.agent_max_v = std::max(4.16, search_params.agent_velocity);
  search_params.time_step = std::max(0.5, params->vru_config().delta_t());

  search_params.av_is_leader = true;

  if (obj_scenario.relationship == Relationship::Cross ||
      obj_scenario.relationship == Relationship::Merge ||
      obj_scenario.relationship == Relationship::SameDir ||
      obj_scenario.relationship == Relationship::OnComing) {
    auto mcts_game_vru = std::make_unique<MCTSSpatiotemporal>(params);
    mcts_game_vru->SetInteractionZone(obj_scenario.interaction_zone);
    mcts_game_vru->SetEgoPath(path);
    return ProcessMCTSVruSearch(kLeaderActionDefault, search_params,
                                *mcts_game_vru, path, mcts_result, debug_info);
  }
  return false;
}

bool ProcessMCTSVruGameInStraight(
    const StBoundaryWithDecision &st_boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario,
    const SpeedPlanningParamsProto::MCTSVruParamsProto *params,
    std::vector<MCTSNodeConstPtr> &mcts_result, MCTSDebugInfo &debug_info) {
  if (nullptr == params) {
    return false;
  }
  MCTSVruSearchParams search_params;
  search_params.ucb_c = 2.0;
  search_params.time_limit = 2.0;
  search_params.simulate_jerk = kJerkBase;
  if (!UpdateMCTSVruSearchParams(
          st_boundary_with_decision, spacetime_obj, path, av_velocity, av_acc,
          params->vru_config().v_pedestrian(), params->vru_config().v_cyclist(),
          search_params)) {
    return false;
  }
  search_params.av_min_v = 0.0;
  search_params.av_max_v = std::max(9.72, search_params.av_velocity);
  search_params.agent_min_v = 0.0;
  search_params.agent_max_v = std::max(4.16, search_params.agent_velocity);
  search_params.time_step = std::max(0.5, params->vru_config().delta_t());

  search_params.av_is_leader = true;

  if (obj_scenario.relationship == Relationship::Cross ||
      obj_scenario.relationship == Relationship::Merge ||
      obj_scenario.relationship == Relationship::SameDir ||
      obj_scenario.relationship == Relationship::OnComing) {
    auto mcts_game_vru = std::make_unique<MCTSSpatiotemporal>(params);
    mcts_game_vru->SetInteractionZone(obj_scenario.interaction_zone);
    mcts_game_vru->SetEgoPath(path);
    return ProcessMCTSVruSearch(kLeaderActionDefault, search_params,
                                *mcts_game_vru, path, mcts_result, debug_info);
  }
  return false;
}

void ProcessMCTSVruGame(
    const StBoundaryWithDecision &boundary_with_decision,
    const SpacetimeObjectTrajectory &spacetime_obj, const DiscretizedPath &path,
    const double av_velocity, const double av_acc,
    const ObjectScenarioInfo &obj_scenario, const std::string &traj_id,
    const SpeedPlanningParamsProto::MCTSVruParamsProto *params,
    MCTSInteractiveResult &result) {
  std::vector<MCTSNodeConstPtr> mcts_result;
  MCTSDebugInfo debug_info;
  DLOG(INFO) << "ProcessMCTSVruGame: " << traj_id
             << " interaction_zone: " << int(obj_scenario.interaction_zone);
  if (InteractionZone::TurnLeft == obj_scenario.interaction_zone) {
    if (ProcessMCTSVruGameInTurnLeft(boundary_with_decision, spacetime_obj,
                                     path, av_velocity, av_acc, obj_scenario,
                                     params, mcts_result, debug_info)) {
      result.mcts_result = std::move(mcts_result);
      result.traj_id = traj_id;
      result.spacetime_obj = &spacetime_obj;
      result.st_boundary = boundary_with_decision.st_boundary();
      result.debug_info = std::move(debug_info);
      result.scenario_info = "hrvo_left_turn";
      result.is_hrvo = true;
    }
  } else if (InteractionZone::TurnRight == obj_scenario.interaction_zone) {
    if (ProcessMCTSVruGameInTurnRight(boundary_with_decision, spacetime_obj,
                                      path, av_velocity, av_acc, obj_scenario,
                                      params, mcts_result, debug_info)) {
      result.mcts_result = std::move(mcts_result);
      result.traj_id = traj_id;
      result.spacetime_obj = &spacetime_obj;
      result.st_boundary = boundary_with_decision.st_boundary();
      result.debug_info = std::move(debug_info);
      result.scenario_info = "hrvo_right_turn";
      result.is_hrvo = true;
    }
  } else if (InteractionZone::JunctionStraight ==
             obj_scenario.interaction_zone) {
    if (ProcessMCTSVruGameInJunctionStraight(
            boundary_with_decision, spacetime_obj, path, av_velocity, av_acc,
            obj_scenario, params, mcts_result, debug_info)) {
      result.mcts_result = std::move(mcts_result);
      result.traj_id = traj_id;
      result.spacetime_obj = &spacetime_obj;
      result.st_boundary = boundary_with_decision.st_boundary();
      result.debug_info = std::move(debug_info);
      result.scenario_info = "hrvo_junction_straight";
      result.is_hrvo = true;
    }
  } else if (InteractionZone::Straight == obj_scenario.interaction_zone) {
    if (ProcessMCTSVruGameInStraight(boundary_with_decision, spacetime_obj,
                                     path, av_velocity, av_acc, obj_scenario,
                                     params, mcts_result, debug_info)) {
      result.mcts_result = std::move(mcts_result);
      result.traj_id = traj_id;
      result.spacetime_obj = &spacetime_obj;
      result.st_boundary = boundary_with_decision.st_boundary();
      result.debug_info = std::move(debug_info);
      result.scenario_info = "hrvo_straight";
      result.is_hrvo = true;
    }
  }
}

}  // namespace planning
}  // namespace e2e_noa
