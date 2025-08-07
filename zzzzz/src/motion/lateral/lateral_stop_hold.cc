/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_stop_hold.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_stop_hold.h"

namespace zark {
namespace planning {

LateralStopHold::LateralStopHold(LateralOptimizerConfig& config) {
  config_ = config;
}

LatMPCData LateralStopHold::GenerateStopTrajectory(
    const Eigen::VectorXd& u_prev, const double& dt_prev) {
  LatMPCData mpc_data;
  const int n_steps = config_.model.num_steps;
  const int n_x = config_.model.num_states;
  const int n_u = config_.model.num_ctrls;
  const int n_nodes = config_.model.num_steps + 1;
  const double dt = config_.model.dt;
  const int n_u_dot = config_.model.num_ctrl_rates;

  mpc_data.names.x.emplace_back("l");
  mpc_data.names.x.emplace_back("l_dot");
  mpc_data.names.x.emplace_back("psi_s");
  mpc_data.names.x.emplace_back("psi_s_dot");
  mpc_data.names.u.emplace_back("delta");
  mpc_data.names.u_dot.emplace_back("delta_dot");
  mpc_data.t = util::TVector(dt, n_nodes);
  mpc_data.x_ref = Eigen::MatrixXd::Zero(n_x, n_nodes);
  mpc_data.u_ref = Eigen::MatrixXd::Zero(n_u, n_steps);
  mpc_data.x = Eigen::MatrixXd::Zero(n_x, n_nodes);
  mpc_data.u = Eigen::MatrixXd::Zero(n_u, n_steps);
  mpc_data.u_prev = u_prev;
  mpc_data.u_dot = Eigen::MatrixXd::Zero(n_u_dot, n_steps);
  mpc_data.constraints = MPCData::Constraints(n_x, n_u, n_u_dot);
  mpc_data.slacks = MPCData::Slacks(n_x, n_u, n_u_dot, n_nodes, n_steps);
  mpc_data.dt = dt;
  mpc_data.dt_prev = dt_prev;
  mpc_data.n_x = n_x;
  mpc_data.n_u = n_u;
  mpc_data.n_u_dot = n_u_dot;
  mpc_data.n_steps = n_steps;
  return mpc_data;
}

}  // namespace planning
}  // namespace zark
