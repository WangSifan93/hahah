/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_mpc_problem.h
 **/

#pragma once

#include "apps/planning/src/common/math/mpc/mpc_problem.h"
#include "gtest/gtest.h"
#include "lqr/linear_quadratic_regulator.h"

namespace zark {
namespace planning {

/**
 * @brief The class implements the methods required to solve the lateral MPC
 *
 */
class LateralMPCProblem : public MPCProblem {
 public:
  explicit LateralMPCProblem(const int n_steps, const int n_x, const int n_u,
                             const int n_u_dot, const Eigen::VectorXd& Q,
                             const Eigen::VectorXd& R,
                             const Eigen::VectorXd& R_dot, const double dt,
                             const double w_terminal);

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

  /**
   * @brief Computes the state-space model matrices for lateral vehicle
   * dynamics.
   *
   * This function calculates the discrete-time state-space representation of
   * the lateral dynamics of a vehicle based on the provided physical parameters
   * and the simulation time step. The matrices calculated are commonly used in
   * control systems for vehicle dynamics.
   *
   * @param C_af Cornering stiffness of both front tires [N/rad]
   * @param C_ar Cornering stiffness of both rear tires [N/rad]
   * @param l_f Distance from the vehicle's center of gravity to the front
   * axle[m].
   * @param l_r Distance from the vehicle's center of gravity to the rear
   * axle[m].
   * @param mass Total mass of the vehicle.[kg]
   * @param I_z Yaw moment of inertia of the vehicle[].
   * @param dt Time step for the discretization process[s].
   * @param v Velocity of the vehicle[m/s].
   * @return A tuple containing three Eigen::MatrixXd:
   *         - First Matrix: Discrete-time system matrix A_d.
   *         - Second Matrix: Control input matrix B_d associated with the
   * primary control input.
   *         - Third Matrix: Control input matrix B_d2 associated with the
   * secondary control input.
   */
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> LateralModel(
      double C_af, double C_ar, double l_f, double l_r, double mass, double I_z,
      double dt, double v);

  void SetV(const Eigen::RowVectorXd v) { v_ = v; }

 private:
  Eigen::RowVectorXd v_;  // speeds from the longitudinal trajectory [m/s]
};

}  // namespace planning
}  // namespace zark
