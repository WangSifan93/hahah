/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_mpc_problem.cc
 **/

#include "apps/planning/src/common/math/mpc/lateral_mpc_problem.h"

#include "apps/planning/src/common/configs/vehicle_config_helper.h"

namespace zark {
namespace planning {

LateralMPCProblem::LateralMPCProblem(const int n_steps, const int n_x,
                                     const int n_u, const int n_u_dot,
                                     const Eigen::VectorXd& Q,
                                     const Eigen::VectorXd& R,
                                     const Eigen::VectorXd& R_dot,
                                     const double dt, const double w_terminal)
    : MPCProblem(n_steps, n_x, n_u, n_u_dot, Q, R, R_dot, dt,
                 {{"l", "l_dot", "psi_s", "psi_s_dot"},
                  {"delta"},
                  {"delta_dot"},
                  {"psi_dot_des"}}) {
  Q_terminal_ = Q.asDiagonal() * w_terminal;
}

std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>,
           std::vector<Eigen::MatrixXd>>
LateralMPCProblem::ComputeStateSpaceModel() {
  std::vector<Eigen::MatrixXd> A_d(n_steps_);
  std::vector<Eigen::MatrixXd> B_d(n_steps_);
  std::vector<Eigen::MatrixXd> B_d2(n_steps_);

  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  const double C_alpha_f = vehicle_param.front_wheel_corner_stiffness();
  const double C_alpha_r = vehicle_param.rear_wheel_corner_stiffness();
  const double mass = vehicle_param.mass();
  const double wheel_base = vehicle_param.wheel_base();
  const double l_f = wheel_base - vehicle_param.rear_axle_to_cg();
  const double l_r = vehicle_param.rear_axle_to_cg();
  const double I_z = vehicle_param.moment_of_inertia();
  for (int k = 0; k < n_steps_; ++k) {
    std::tie(A_d[k], B_d[k], B_d2[k]) =
        LateralModel(C_alpha_f, C_alpha_r, l_f, l_r, mass, I_z, dt_, v_[k]);
  }
  return std::make_tuple(A_d, B_d, B_d2);
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
LateralMPCProblem::LateralModel(double C_alpha_f, double C_alpha_r, double l_f,
                                double l_r, double mass, double I_z, double dt,
                                double v) {
  Eigen::MatrixXd A_d_elem, B_d_elem, B_d2_elem;
  A_d_elem.resize(n_x_, n_x_);
  B_d_elem.resize(n_x_, n_u_);
  B_d2_elem.resize(n_x_, n_u_dot_);

  A_d_elem.setZero();
  A_d_elem(0, 0) = 1;
  A_d_elem(0, 1) =
      (dt * mass * v *
       (4 * I_z * v + 2 * C_alpha_f * dt * l_f * l_f +
        2 * C_alpha_r * dt * l_r * l_r - C_alpha_f * dt * dt * l_f * v +
        C_alpha_r * dt * dt * l_r * v)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);
  A_d_elem(0, 2) =
      (dt * dt * v *
       (2 * C_alpha_f * I_z * v + 2 * C_alpha_r * I_z * v +
        C_alpha_f * C_alpha_r * dt * l_f * l_f +
        C_alpha_f * C_alpha_r * dt * l_r * l_r +
        2 * C_alpha_f * C_alpha_r * dt * l_f * l_r)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);
  A_d_elem(0, 3) =
      (I_z * dt * dt * v *
       (2 * C_alpha_r * l_r - 2 * C_alpha_f * l_f + C_alpha_f * dt * v +
        C_alpha_r * dt * v)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);

  A_d_elem(1, 0) = 0;
  A_d_elem(1, 1) =
      -(C_alpha_f * C_alpha_r * dt * dt * l_f * l_f - 4 * I_z * mass * v * v +
        C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
        2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v +
        C_alpha_f * dt * dt * l_f * mass * v * v -
        C_alpha_r * dt * dt * l_r * mass * v * v +
        2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r -
        2 * C_alpha_f * dt * l_f * l_f * mass * v -
        2 * C_alpha_r * dt * l_r * l_r * mass * v) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);
  A_d_elem(1, 2) =
      (2 * dt * v *
       (2 * C_alpha_f * I_z * v + 2 * C_alpha_r * I_z * v +
        C_alpha_f * C_alpha_r * dt * l_f * l_f +
        C_alpha_f * C_alpha_r * dt * l_r * l_r +
        2 * C_alpha_f * C_alpha_r * dt * l_f * l_r)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);
  A_d_elem(1, 3) =
      (2 * I_z * dt * v *
       (2 * C_alpha_r * l_r - 2 * C_alpha_f * l_f + C_alpha_f * dt * v +
        C_alpha_r * dt * v)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);

  A_d_elem(2, 0) = 0;
  A_d_elem(2, 1) =
      -(2 * dt * dt * mass * v * (C_alpha_f * l_f - C_alpha_r * l_r)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);
  A_d_elem(2, 2) =
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v +
       C_alpha_f * dt * dt * l_f * mass * v * v -
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);
  A_d_elem(2, 3) =
      (2 * I_z * dt * v * (C_alpha_f * dt + C_alpha_r * dt + 2 * mass * v)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);

  A_d_elem(3, 0) = 0;
  A_d_elem(3, 1) =
      -(4 * dt * mass * v * (C_alpha_f * l_f - C_alpha_r * l_r)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);
  A_d_elem(3, 2) =
      (4 * dt * mass * v * v * (C_alpha_f * l_f - C_alpha_r * l_r)) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);
  A_d_elem(3, 3) =
      -(C_alpha_f * C_alpha_r * dt * dt * l_f * l_f - 4 * I_z * mass * v * v +
        C_alpha_f * C_alpha_r * dt * dt * l_r * l_r -
        2 * C_alpha_f * I_z * dt * v - 2 * C_alpha_r * I_z * dt * v -
        C_alpha_f * dt * dt * l_f * mass * v * v +
        C_alpha_r * dt * dt * l_r * mass * v * v +
        2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
        2 * C_alpha_f * dt * l_f * l_f * mass * v +
        2 * C_alpha_r * dt * l_r * l_r * mass * v) /
      (4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
       C_alpha_f * C_alpha_r * dt * dt * l_r * l_r +
       2 * C_alpha_f * I_z * dt * v + 2 * C_alpha_r * I_z * dt * v -
       C_alpha_f * dt * dt * l_f * mass * v * v +
       C_alpha_r * dt * dt * l_r * mass * v * v +
       2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
       2 * C_alpha_f * dt * l_f * l_f * mass * v +
       2 * C_alpha_r * dt * l_r * l_r * mass * v);

  B_d_elem.setZero();
  const double denominator_B_d =
      4 * I_z * mass * v * v +
      C_alpha_f * C_alpha_r * dt * dt * (l_f * l_f + l_r * l_r) +
      2 * (C_alpha_f * I_z * dt * v + C_alpha_r * I_z * dt * v) -
      C_alpha_f * dt * dt * l_f * mass * v * v +
      C_alpha_r * dt * dt * l_r * mass * v * v +
      2 * (C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
           C_alpha_f * dt * l_f * l_f * mass * v +
           C_alpha_r * dt * l_r * l_r * mass * v);

  B_d_elem(0, 0) =
      (C_alpha_f * dt * dt * v *
       (4 * I_z * v + 2 * C_alpha_r * dt * l_r * l_r +
        C_alpha_r * dt * dt * l_f * v + C_alpha_r * dt * dt * l_r * v +
        2 * C_alpha_r * dt * l_f * l_r)) /
      (4 * denominator_B_d);

  B_d_elem(1, 0) =
      (C_alpha_f * dt * v *
       (4 * I_z * v + 2 * C_alpha_r * dt * l_r * l_r +
        C_alpha_r * dt * dt * l_f * v + C_alpha_r * dt * dt * l_r * v +
        2 * C_alpha_r * dt * l_f * l_r)) /
      (2 * denominator_B_d);
  B_d_elem(2, 0) =
      (C_alpha_f * dt * dt * v *
       (C_alpha_r * dt * l_f + C_alpha_r * dt * l_r + 2 * l_f * mass * v)) /
      (2 * denominator_B_d);
  B_d_elem(3, 0) =
      (C_alpha_f * dt * v *
       (C_alpha_r * dt * l_f + C_alpha_r * dt * l_r + 2 * l_f * mass * v)) /
      denominator_B_d;

  const double denominator_B_d2 =
      4 * I_z * mass * v * v +
      C_alpha_f * C_alpha_r * dt * dt * (l_f * l_f + l_r * l_r) +
      2 * (C_alpha_f + C_alpha_r) * I_z * dt * v -
      (C_alpha_f * dt * dt * l_f + C_alpha_r * dt * dt * l_r) * mass * v * v +
      2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
      2 * (C_alpha_f * dt * l_f * l_f + C_alpha_r * dt * l_r * l_r) * mass * v;

  B_d2_elem.setZero();
  B_d2_elem(0, 0) =
      -(dt * dt * v *
        (4 * C_alpha_f * I_z * l_f - 4 * C_alpha_r * I_z * l_r +
         4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
         C_alpha_f * C_alpha_r * dt * dt * l_r * l_r -
         C_alpha_f * dt * dt * l_f * mass * v * v +
         C_alpha_r * dt * dt * l_r * mass * v * v +
         2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
         2 * C_alpha_f * dt * l_f * l_f * mass * v +
         2 * C_alpha_r * dt * l_r * l_r * mass * v)) /
      (4 * denominator_B_d2);

  B_d2_elem(1, 0) =
      -(dt * v *
        (4 * C_alpha_f * I_z * l_f - 4 * C_alpha_r * I_z * l_r +
         4 * I_z * mass * v * v + C_alpha_f * C_alpha_r * dt * dt * l_f * l_f +
         C_alpha_f * C_alpha_r * dt * dt * l_r * l_r -
         C_alpha_f * dt * dt * l_f * mass * v * v +
         C_alpha_r * dt * dt * l_r * mass * v * v +
         2 * C_alpha_f * C_alpha_r * dt * dt * l_f * l_r +
         2 * C_alpha_f * dt * l_f * l_f * mass * v +
         2 * C_alpha_r * dt * l_r * l_r * mass * v)) /
      (2 * denominator_B_d2);

  B_d2_elem(2, 0) = -(dt * dt *
                      (C_alpha_f * C_alpha_r * dt * l_f * l_f +
                       C_alpha_f * C_alpha_r * dt * l_r * l_r +
                       2 * C_alpha_f * l_f * l_f * mass * v +
                       2 * C_alpha_r * l_r * l_r * mass * v +
                       2 * C_alpha_f * C_alpha_r * dt * l_f * l_r -
                       C_alpha_f * dt * l_f * mass * v * v +
                       C_alpha_r * dt * l_r * mass * v * v)) /
                    (2 * denominator_B_d2);

  B_d2_elem(3, 0) = -(dt * (C_alpha_f * C_alpha_r * dt * l_f * l_f +
                            C_alpha_f * C_alpha_r * dt * l_r * l_r +
                            2 * C_alpha_f * l_f * l_f * mass * v +
                            2 * C_alpha_r * l_r * l_r * mass * v +
                            2 * C_alpha_f * C_alpha_r * dt * l_f * l_r -
                            C_alpha_f * dt * l_f * mass * v * v +
                            C_alpha_r * dt * l_r * mass * v * v)) /
                    denominator_B_d2;

  return std::make_tuple(A_d_elem, B_d_elem, B_d2_elem);
}

}  // namespace planning
}  // namespace zark
