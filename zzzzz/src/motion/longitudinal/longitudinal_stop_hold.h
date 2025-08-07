/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_stop_hold.h
 **/

#pragma once

#include <vector>

#include "Eigen/Core"
#include "apps/planning/src/motion/longitudinal/longitudinal_blocker.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_padding.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

class LongitudinalStopHold {
 public:
  LongitudinalStopHold() = default;

  LongitudinalStopHold(LongitudinalOptimizerConfig& config);

  /**
   * @brief Check if it's necessary to stop the vehicle
   *
   * @param x_init the initial state x_init = [s_init; v_init]
   * @param blockers longitudinal blockers with front/rear relations identified
   * @param time_gap_level time gap level from state management
   * @param lon_lookup_tables longitudinal lookup tables
   * @param lon_padding longitudinal paddings
   * @return whether a stop sign is needed
   */
  bool IsStopHold(const Eigen::VectorXd& x_init,
                  const std::vector<Blocker>& blockers,
                  const TimeGapLevel& time_gap_level,
                  const LongitudinalLookupTables& lon_lookup_tables,
                  const LongitudinalPadding& lon_padding);

  /**
   * @brief Generate empty trajectory for stop sign
   *
   * @param u_prev previous acceleration [m/s-2]
   * @param dt_prev time difference with the previous frame [s]
   * @return LonMPCData
   */
  LonMPCData GenerateStopTrajectory(const Eigen::VectorXd& u_prev,
                                    const double& dt_prev);

 private:
  LongitudinalOptimizerConfig config_;
};

}  // namespace planning

}  // namespace zark
