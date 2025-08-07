/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_reference.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_reference.h"

namespace zark {
namespace planning {

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
LateralReference::ConstructReferenceTrajectory(const Tube& tube) const {
  const int n_x = config_.model.num_states;
  const int n_u = config_.model.num_ctrls;
  const int n_steps = config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Zero(n_x, tube.pts.size());
  Eigen::MatrixXd u_ref = Eigen::MatrixXd::Zero(n_u, n_steps);
  for (int k = 0; k < n_nodes; k++) {
    x_ref(kIdxL, k) =
        (tube.pts.at(k).l_left_ref + tube.pts.at(k).l_right_ref) / 2.0;
  }
  return std::make_pair(x_ref, u_ref);
}

}  // namespace planning
}  // namespace zark
