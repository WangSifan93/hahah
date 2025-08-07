#include "make_style_estimation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include "common/type_def.h"
#include "make_style_estimation/style_estimation.h"
#include "math/double.h"
#include "mcts_game_process/mcts_data_type.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "speed/st_boundary_with_decision.h"

namespace e2e_noa::planning {
namespace {
void UpdateObstacleHistMap(
    const PathPoint &current_path_point, const double av_velocity,
    const double av_acc, const SpacetimeTrajectoryManager &st_traj_mgr,
    const absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const int32_t max_history_length,
    std::unordered_map<std::string, std::vector<AgentStatus>>
        &obstacle_hist_map) {
  // update AV history info
  AgentStatus av_status;
  av_status.id = "av";
  av_status.x = current_path_point.x();
  av_status.y = current_path_point.y();
  av_status.v = av_velocity;
  av_status.a = av_acc;
  obstacle_hist_map[av_status.id].push_back(av_status);
  auto &av_hist = obstacle_hist_map[av_status.id];
  if (av_hist.size() > max_history_length) {
    av_hist.erase(av_hist.begin());
  }
  // update obstacle history info
  std::unordered_set<std::string> current_obstacle_ids;
  for (const auto &st_boundary_wd : st_boundaries_with_decision) {
    if (st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }
    if (!st_boundary_wd.st_boundary()->overlap_meta().has_value()) {
      continue;
    }
    if (st_boundary_wd.st_boundary()->source_type() !=
        StBoundarySourceTypeProto::ST_OBJECT) {
      return;
    }
    current_obstacle_ids.insert(st_boundary_wd.id());
    CHECK(st_boundary_wd.traj_id().has_value());
    const auto &traj_id = st_boundary_wd.traj_id();
    // std::string_view traj_id_s(traj_id);
    const auto *traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
    AgentStatus obj_status;
    obj_status.id = st_boundary_wd.id();
    obj_status.x = traj->planner_object().pose().pos().x();
    obj_status.y = traj->planner_object().pose().pos().y();
    obj_status.v = traj->planner_object().pose().v();
    obj_status.a = traj->planner_object().pose().a();
    obstacle_hist_map[obj_status.id].push_back(obj_status);
    if (obstacle_hist_map[obj_status.id].size() > max_history_length) {
      obstacle_hist_map[obj_status.id].erase(
          obstacle_hist_map[obj_status.id].begin());
    }
  }
  // remove obstacles that are not in current_obstacle_ids
  for (auto it = obstacle_hist_map.begin(); it != obstacle_hist_map.end();) {
    if (it->first != av_status.id &&
        current_obstacle_ids.find(it->first) == current_obstacle_ids.end()) {
      it = obstacle_hist_map.erase(it);
    } else {
      ++it;
    }
  }
}
}  // namespace

absl::Status MakeStyleEstimation(
    const SpeedPlanningParamsProto &speed_finder_params,
    const DiscretizedPath &path, const double av_velocity, const double av_acc,
    const SpacetimeTrajectoryManager &st_traj_mgr, const uint64_t seq_num,
    const absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>
        &agent_status_history,
    std::unordered_map<std::string, double> &obstacle_style_map) {
  std::unordered_map<std::string, std::vector<AgentStatus>> obstacle_hist_map;
  // update obstacle and AV history
  for (const auto &agent_hist : agent_status_history) {
    auto id = agent_hist.first;
    const auto &agent_states = agent_hist.second;
    for (const auto &agent_state : agent_states) {
      AgentStatus agent_status;
      agent_status.id = id;
      agent_status.x = agent_state.x;
      agent_status.y = agent_state.y;
      agent_status.v = agent_state.v;
      agent_status.a = agent_state.a;
      obstacle_hist_map[id].push_back(agent_status);
    }
  }
  UpdateObstacleHistMap(
      path.front(), av_velocity, av_acc, st_traj_mgr,
      st_boundaries_with_decision,
      speed_finder_params.style_estimation_params().num_history_points(),
      obstacle_hist_map);
  agent_status_history.clear();
  for (const auto &[id, agent_states] : obstacle_hist_map) {
    for (const auto &agent_state : agent_states) {
      ad_e2e::planning::AgentState agent_status;
      agent_status.id = agent_state.id;
      agent_status.x = agent_state.x;
      agent_status.y = agent_state.y;
      agent_status.v = agent_state.v;
      agent_status.a = agent_state.a;
      agent_status_history[id].push_back(agent_status);
    }
  }
  // make style estimation
  obstacle_style_map.clear();
  for (const auto &st_boundary_wd : st_boundaries_with_decision) {
    const auto &obstacle_id = st_boundary_wd.id();
    if (obstacle_hist_map.find(obstacle_id) != obstacle_hist_map.end()) {
      if (obstacle_hist_map[obstacle_id].size() > 4 &&
          obstacle_hist_map["av"].size() > 4) {
        const auto &first_overlap =
            st_boundary_wd.st_boundary()->overlap_infos().front();
        const auto &overlap_av_path_point = path[first_overlap.av_start_idx];
        const double interact_position_x = overlap_av_path_point.x();
        const double interact_position_y = overlap_av_path_point.y();
        StyleEstimator style_estimator(
            speed_finder_params.style_estimation_params(),
            speed_finder_params.style_estimation_params().num_history_points(),
            interact_position_x, interact_position_y);
        obstacle_style_map[obstacle_id] = style_estimator.EstimateStyle(
            obstacle_hist_map["av"], obstacle_hist_map[obstacle_id]);
      }
    }
  }
  return absl::OkStatus();
}
}  // namespace e2e_noa::planning
