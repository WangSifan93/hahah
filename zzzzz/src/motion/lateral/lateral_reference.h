/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_reference.h
 **/

#pragma once

#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/motion/lateral/data_type.h"

#include "Eigen/Core"

namespace zark {
namespace planning {

/**
 * class LateralReference
 * @brief build lateral ref with corridor.
 */
class LateralReference {
 public:
  LateralReference(const LateralOptimizerConfig& config) { config_ = config; }

  ~LateralReference() = default;

  /**
   * @brief This function computes the reference trajectory for the lateral MPC
   * problem.
   *
   * @param tube the lateral tube info.
   * @return l_ref,u_ref
   */
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ConstructReferenceTrajectory(
      const Tube& tube) const;

 private:
  LateralOptimizerConfig config_;
};

}  // namespace planning
}  // namespace zark
