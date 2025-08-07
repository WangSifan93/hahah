/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateal_stop_hold.h
 **/

#pragma once

#include <vector>

#include "Eigen/Core"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/common/math/mpc/utility.h"

namespace zark {
namespace planning {

class LateralStopHold {
 public:
  LateralStopHold() = default;

  LateralStopHold(LateralOptimizerConfig& config);

  /**
   * @brief Generate stop trajectory for stop sign
   *
   * @param u_prev previous front wheel angle [rad]
   * @param dt_prev time difference with the previous frame [s]
   * @return LatMPCData
   */
  LatMPCData GenerateStopTrajectory(const Eigen::VectorXd& u_prev,
                                    const double& dt_prev);

 private:
  LateralOptimizerConfig config_;
};
}  // namespace planning
}  // namespace zark
