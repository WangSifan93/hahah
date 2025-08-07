/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_mpc_problem.cc
 **/

#include "apps/planning/src/common/math/mpc/longitudinal_mpc_problem.h"

#include "lqr/linear_quadratic_regulator.h"

namespace zark {
namespace planning {

LongitudinalMPCProblem::LongitudinalMPCProblem(const int n_steps, const int n_x,
                                               const int n_u, const int n_u_dot,
                                               const Eigen::VectorXd& Q,
                                               const Eigen::VectorXd& R,
                                               const Eigen::VectorXd& R_dot,
                                               const double dt)
    : MPCProblem(n_steps, n_x, n_u, n_u_dot, Q, R, R_dot, dt,
                 {{"s", "v"}, {"a"}, {"j"}, {""}}) {
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = ComputeStateSpaceModel();
  Q_terminal_ = ComputeTerminalCostQ(A_d.back(), B_d.back(), Q, R);
}

std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>,
           std::vector<Eigen::MatrixXd>>
LongitudinalMPCProblem::ComputeStateSpaceModel() {
  Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(2, 2);
  Eigen::VectorXd B_c = Eigen::MatrixXd::Zero(2, 1);
  A_c(0, 1) = 1;
  B_c(1, 0) = 1;

  std::vector<Eigen::MatrixXd> A_d(1);
  std::vector<Eigen::MatrixXd> B_d(1);
  A_d[0] = Eigen::MatrixXd::Identity(n_x_, n_x_) + A_c * dt_;
  B_d[0] = B_c * dt_;

  return std::make_tuple(A_d, B_d, std::vector<Eigen::MatrixXd>());
}

Eigen::MatrixXd LongitudinalMPCProblem::ComputeTerminalCostQ(
    const Eigen::MatrixXd& A_d, const Eigen::MatrixXd& B_d,
    const Eigen::VectorXd& Q, const Eigen::VectorXd& R) {
  constexpr uint max_num_iteration = 150;
  constexpr double tolerance = 0.01;
  return math::SolveDARE(A_d, B_d, Q.asDiagonal(), R.asDiagonal(), tolerance,
                         max_num_iteration);
}

}  // namespace planning
}  // namespace zark
