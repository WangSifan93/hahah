/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file speed_limit_builder.cc
 **/

#include "apps/planning/src/decision/longitudinal/speed_limit_builder.h"

#include <math.h>

#include <algorithm>

#include "apps/planning/src/config/conf_gflags.h"

namespace zark {
namespace planning {

constexpr double numerical_epsilon = 1e-6;

SpeedLimitBuilder::SpeedLimitBuilder(
    const LongitudinalDeciderConfig::SpeedLimitConfig& config)
    : config_(config),
      a_lat_max_table_(LookupTable(config.a_lat_max_table)),
      t_sliding_window_(config.t_sliding_window) {}

std::unordered_map<std::string, SpeedLimit>
SpeedLimitBuilder::BuildSpeedLimitMap(const LocalRoute& local_route,
                                      const Corridor& corridor,
                                      const double v_ego,
                                      const double goal_cruise_speed) const {
  std::unordered_map<std::string, SpeedLimit> speed_limit_map;

  const double ds_sliding_window = t_sliding_window_ * goal_cruise_speed;
  std::deque<std::pair<double, double>> kappa_window;  // Store (s, kappa) pairs
  for (auto point : corridor) {
    double v_final = goal_cruise_speed;
    speed_limit_map["goal"].AppendSpeedLimit(point.s, goal_cruise_speed);

    const double v_kappa = CalculateCurvatureSpeedLimit(
        point, kappa_window, ds_sliding_window, v_ego);
    v_final = std::min(v_final, v_kappa);
    speed_limit_map["curvature"].AppendSpeedLimit(point.s, v_kappa);

    // TODO crosswalk speed limit
    const double v_crosswalk = config_.v_max;
    v_final = std::min(v_final, v_crosswalk);
    speed_limit_map["crosswalk"].AppendSpeedLimit(point.s, v_crosswalk);

    speed_limit_map["final"].AppendSpeedLimit(point.s, v_final);
  }
  return speed_limit_map;
}

double SpeedLimitBuilder::CalculateCurvatureSpeedLimit(
    const CorridorPoint& point,
    std::deque<std::pair<double, double>>& kappa_window,
    const double ds_sliding_window, const double v) const {
  // Remove elements from the window that are out of the ds_sliding_window
  while (!kappa_window.empty() &&
         kappa_window.front().first < point.s - ds_sliding_window) {
    kappa_window.pop_front();
  }

  // Add the current element to the window and update kappa_max
  kappa_window.push_back(std::make_pair(point.s, std::fabs(point.kappa)));
  double kappa_max = std::numeric_limits<double>::min();
  for (const auto& [s, kappa] : kappa_window) {
    kappa_max = std::max(kappa_max, kappa);
  }

  return std::sqrt(a_lat_max_table_.Evaluate(v) /
                   std::max(kappa_max, numerical_epsilon));
}

}  // namespace planning
}  // namespace zark
