/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file speed_limit_builder.h
 **/

#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/speed_limit.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/common/lookup_table.h"
namespace zark {
namespace planning {

class SpeedLimitBuilder {
 public:
  explicit SpeedLimitBuilder(
      const LongitudinalDeciderConfig::SpeedLimitConfig& config);

  /**
   * @brief Builds a speed limit map
   *
   * @param local_route The local route
   * @param corridor The current corridor
   * @param v_ego The ego speed [m/s]
   * @param goal_cruise_speed The goal cruise speed [m/s]
   * @return std::unordered_map<std::string, SpeedLimit>
   */
  std::unordered_map<std::string, SpeedLimit> BuildSpeedLimitMap(
      const LocalRoute& local_route, const Corridor& corridor,
      const double v_ego, const double goal_cruise_speed) const;

 private:
  /**
   * @brief Calculate the minimum value of  v_κappa by the maximum value
   * within the range of t seconds before the current point.
   *
   * @param kappa_window The deque of kappa values within the range of t
   * seconds
   * @param ds_sliding_window The distance of the sliding window
   * @param point The current point
   * @param v The current velocity
   * @return double
   */
  double CalculateCurvatureSpeedLimit(
      const CorridorPoint& point,
      std::deque<std::pair<double, double>>& kappa_window,
      const double ds_sliding_window, const double v) const;

 private:
  LongitudinalDeciderConfig::SpeedLimitConfig config_;
  const LookupTable a_lat_max_table_;
  const double t_sliding_window_;
};

}  // namespace planning
}  // namespace zark
