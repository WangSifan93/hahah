/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_mpc_problem.h
 **/

#pragma once

#include "apps/planning/src/common/math/mpc/mpc_problem.h"

namespace zark {
namespace planning {

/**
 * @brief The class implements methods required to solve the Longitudinal MPC
 */
class LongitudinalMPCProblem : public MPCProblem {
 public:
  explicit LongitudinalMPCProblem(const int n_steps, const int n_x,
                                  const int n_u, const int n_u_dot,
                                  const Eigen::VectorXd& Q,
                                  const Eigen::VectorXd& R,
                                  const Eigen::VectorXd& R_dot,
                                  const double dt);

  /**
   * @brief Compute the state space model matrices for the lateral dynamics
   * control system.
   *
   * This method calculates the state matrices A_d, B_d, and B_d2 based on the
   * vehicle's physical parameters and the control system design. The matrices
   * are used in the state space representation of the dynamic system.
   *
   * @return A tuple containing three vectors of Eigen::MatrixXd:
   *         - First vector: Contains the discretized system matrix A_d.
   *         - Second vector: Contains the input matrix B_d related to the
   * primary control input.
   *         - Third vector: Contains the input matrix B_d2 related to the
   * secondary control input.
   */
  std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>,
             std::vector<Eigen::MatrixXd>>
  ComputeStateSpaceModel() override;

  Eigen::MatrixXd ComputeTerminalCostQ(const Eigen::MatrixXd& A_d,
                                       const Eigen::MatrixXd& B_d,
                                       const Eigen::VectorXd& Q,
                                       const Eigen::VectorXd& R);
};

}  // namespace planning
}  // namespace zark
