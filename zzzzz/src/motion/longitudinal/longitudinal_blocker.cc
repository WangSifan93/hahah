/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_blocker.cc
 **/
#include "apps/planning/src/motion/longitudinal/longitudinal_blocker.h"

#include <limits>

#include "linear_interpolation.h"

namespace zark {
namespace planning {

LongitudinalBlocker::LongitudinalBlocker(
    LongitudinalOptimizerConfig::Model& config) {
  config_ = config;
}

std::vector<Blocker> LongitudinalBlocker::ConstructBlocker(
    const STProposal& st_proposal,
    const IndexedPtrList<std::string, const Obstacle*>& obstacle_map,
    const Corridor& corridor) {
  std::vector<Blocker> blockers;

  for (const auto& st_prop : st_proposal) {
    Blocker blocker(config_.num_steps + 1);
    blocker.is_front = std::get<1>(st_prop);
    blocker.is_filtered = std::get<2>(st_prop);

    const STBoundary* st_boundary = std::get<0>(st_prop);
    const Obstacle* obs = obstacle_map.Find(st_boundary->id());
    if (obs == nullptr) {
      continue;
    }

    blocker.obs = obs;
    const double t_start = st_boundary->lower_points().front().t();
    const double t_end = st_boundary->lower_points().back().t();
    blocker.k_start =
        std::max(0, static_cast<int>(std::ceil(t_start / config_.dt)));
    blocker.k_end = std::min(config_.num_steps,
                             static_cast<int>(std::floor(t_end / config_.dt)));

    for (int k = 0; k < config_.num_steps + 1; ++k) {
      const double t_curr = k * config_.dt;
      if (t_curr >= t_start && t_curr <= t_end) {
        blocker.s[k] = blocker.is_front ? st_boundary->GetLowerSByT(t_curr)
                                        : st_boundary->GetUpperSByT(t_curr);
        const ::common::TrajectoryPoint obs_point = obs->GetPointAtTime(t_curr);
        CorridorPoint corr_point = corridor.EvaluateByS(blocker.s[k]);
        blocker.v[k] = ComputeLonSpeed(obs_point, corr_point);
      }
    }
    if (blocker.k_start < blocker.k_end) {
      blockers.emplace_back(blocker);
    }
  }
  return blockers;
}

double LongitudinalBlocker::ComputeLonSpeed(
    const ::common::TrajectoryPoint& obs_point,
    const CorridorPoint& corr_point) {
  const double theta = obs_point.path_point().theta();
  const double theta_ref = corr_point.theta;
  return std::max(obs_point.v() * cos(theta - theta_ref), 0.0);
}

}  // namespace planning
}  // namespace zark
