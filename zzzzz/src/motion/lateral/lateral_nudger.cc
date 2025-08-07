/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_nudger.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_nudger.h"

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/conversion.h"

namespace zark {
namespace planning {

std::vector<Nudger> LateralNudger::ConstructNudger(
    const std::vector<std::pair<const Obstacle*, bool>>& lateral_obstacles,
    const LonMPCData& lon_mpc_data, const CorridorInfo& corridor_info) const {
  std::vector<Nudger> nudgers;
  const int n_steps = config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  const double dt = config_.model.dt;
  const int kIdxS = 0;

  if (!lateral_obstacles.empty() && PlanningGflags::enable_use_nudgers_info) {
    auto& first_obstacle_traj = lateral_obstacles.at(0).first->Trajectory();
    bool obs_has_traj = first_obstacle_traj.trajectory_point().size() > 1;
    const auto& obs_sl_boundary_map = corridor_info.GetObsSLBoundayMap();
    ACHECK(obs_has_traj);
    ACHECK(dt > 0.0);

    for (const auto& [obstacle, is_left] : lateral_obstacles) {
      bool has_overlap = false;
      Nudger nudger(n_nodes);
      for (int k = 0; k < n_nodes; k++) {
        const double t = k * dt;
        const Corridor& corridor = corridor_info.GetCorridor();
        if (!obs_sl_boundary_map.empty() &&
            obs_sl_boundary_map.count(obstacle)) {
          const auto& obs_sl_boundary =
              obs_sl_boundary_map.at(obstacle).EvaluateByT(t);
          const double s = lon_mpc_data.x.coeff(kIdxS, k);
          if (NeedUpdateNudger(s, corridor, obs_sl_boundary,
                               is_left)) {
            if (is_left) {
              nudger.l[k] = std::min(obs_sl_boundary.start_l(), nudger.l[k]);
              nudger.v[k] = obstacle->speed();
            } else {
              nudger.l[k] = std::max(obs_sl_boundary.end_l(), nudger.l[k]);
              nudger.v[k] = obstacle->speed();
            }

            has_overlap = true;
          }
        }
      }
      nudger.is_left = is_left;
      if (has_overlap) nudgers.emplace_back(nudger);
    }
  }
  ADEBUG << " nudgers size = " << nudgers.size();
  return nudgers;
}

bool LateralNudger::NeedUpdateNudger(const double& s, const Corridor& corridor,
                                     const SLBoundary& obs_sl_boundary,
                                     bool is_left) const {
  const CorridorPoint corridor_point = corridor.EvaluateByS(s);
  const double corridor_point_l_left = corridor_point.l_left;
  const double corridor_point_l_right = corridor_point.l_right;
  const double obs_sl_boundary_l_end = obs_sl_boundary.end_l();
  const double obs_sl_boundary_l_start = obs_sl_boundary.start_l();
  const double obs_sl_boundary_s_end = obs_sl_boundary.end_s();
  const double obs_sl_boundary_s_start = obs_sl_boundary.start_s();

  const bool is_left_valiation =
      is_left && obs_sl_boundary_l_start < corridor_point_l_left &&
      obs_sl_boundary_l_end > corridor_point_l_left;
  const bool is_right_valiation =
      !is_left && obs_sl_boundary_l_end > corridor_point_l_right &&
      obs_sl_boundary_l_start < corridor_point_l_right;
  if (is_left_valiation || is_right_valiation) {
    if (is_left_valiation) {
      if (corridor_point.s > obs_sl_boundary_s_start &&
          corridor_point.s < obs_sl_boundary_s_end) {
        ADEBUG << "is_left_valiation and has overlap at s = " << s;
        return true;
      }
    } else if (is_right_valiation) {
      if (corridor_point.s > obs_sl_boundary_s_start &&
          corridor_point.s < obs_sl_boundary_s_end) {
        ADEBUG << "is_right_valiation and has overlap at s = " << s;
        return true;
      }
    }
  }
  return false;
}

}  // namespace planning
}  // namespace zark
