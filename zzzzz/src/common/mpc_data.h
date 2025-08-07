

/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file mpc_data.h
 **/

#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "apps/planning/src/common/speed_limit.h"
#include "apps/planning/src/motion/lateral/data_type.h"

namespace zark {
namespace planning {

/**
 * @brief This struct defines the data in a general MPC problem.
 *
 */
struct MPCData {
  /**
   * @brief This struct defines the names of states, controls, and control
   * rates.
   *
   */
  struct Names {
    std::vector<std::string> x;
    std::vector<std::string> u;
    std::vector<std::string> u_dot;
    std::vector<std::string> u_2;
  };

  /**
   * @brief This struct defines the inequality constraints.
   *
   */
  struct Constraints {
    Constraints() = default;
    explicit Constraints(const int n_x, const int n_u, const int n_u_dot)
        : soft_min(n_x, n_u, n_u_dot),
          soft_max(n_x, n_u, n_u_dot),
          stiff_min(n_x, n_u, n_u_dot),
          stiff_max(n_x, n_u, n_u_dot),
          hard_min(n_x, n_u, n_u_dot),
          hard_max(n_x, n_u, n_u_dot) {}
    struct Constraint {
      Constraint() = default;
      explicit Constraint(const int n_x, const int n_u, const int n_u_dot)
          : x(n_x), u(n_u), u_dot(n_u_dot) {}

      struct ConstraintInfo {
        ConstraintInfo() = default;
        explicit ConstraintInfo(const int n) : idx_enabled(n, false) {}

        Eigen::MatrixXd constraint;     // row: state/control, col: time steps
        Eigen::VectorXd weight;         // state/control weights
        std::vector<bool> idx_enabled;  // enabled indices
        int n_enabled_row = 0;          // number of enabled rows
        int n_enabled_col = 0;          // number of enabled columns
      };

      ConstraintInfo x;      // state
      ConstraintInfo u;      // control
      ConstraintInfo u_dot;  // control rate
      int n_ineq;            // number of constraints
    };

    Constraint soft_min;
    Constraint soft_max;
    Constraint stiff_min;
    Constraint stiff_max;
    Constraint hard_min;
    Constraint hard_max;
    int n_ineq_hard = 0;  // number of hard constraints
    int n_ineq_soft = 0;  // number of soft constraints
    int n_ineq = 0;       // number of constraints
  };

  /**
   * @brief This struct defines the slack variables for all soft/stiff
   * inequality constraints.
   *
   */
  struct Slacks {
    Slacks() = default;
    explicit Slacks(const int n_x, const int n_u, const int n_u_dot,
                    const int n_nodes, const int n_steps)
        : soft_min(n_x, n_u, n_u_dot, n_nodes, n_steps),
          soft_max(n_x, n_u, n_u_dot, n_nodes, n_steps),
          stiff_min(n_x, n_u, n_u_dot, n_nodes, n_steps),
          stiff_max(n_x, n_u, n_u_dot, n_nodes, n_steps) {}
    struct Slack {
      Slack() = default;
      explicit Slack(const int n_x, const int n_u, const int n_u_dot,
                     const int n_nodes, const int n_steps)
          : x(Eigen::MatrixXd::Constant(n_x, n_nodes, std::nan(""))),
            u(Eigen::MatrixXd::Constant(n_u, n_steps, std::nan(""))),
            u_dot(Eigen::MatrixXd::Constant(n_u_dot, n_steps, std::nan(""))) {}
      Eigen::MatrixXd x;      // states
      Eigen::MatrixXd u;      // controls
      Eigen::MatrixXd u_dot;  // control rates
    };

    Slack soft_min;
    Slack soft_max;
    Slack stiff_min;
    Slack stiff_max;
  };

  Names names;
  Eigen::RowVectorXd t;    // time trajectory [s]
  Eigen::MatrixXd x_ref;   // reference state trajectory
  Eigen::MatrixXd u_ref;   // reference control trajectory
  Eigen::MatrixXd x;       // state trajectory
  Eigen::MatrixXd u;       // control trajectory
  Eigen::VectorXd u_prev;  // control input from the previous planning cycle
  Eigen::MatrixXd u_dot;   // control rate trajectory
  Constraints constraints;
  Slacks slacks;
  double dt;       // sampling time [s]
  double dt_prev;  // time elapsed since the previous planning cycle [s]
  int n_x;         // number of states
  int n_u;         // number of controls
  int n_u_dot;     // number of control rates
  int n_steps;     // number of time steps
};

/**
 * @brief This struct defines the data specifically for the longitudinal MPC
 * problem.
 *
 */
struct LonMPCData : MPCData {
  std::vector<SpeedLimit> speed_limit_set;
  bool is_stop_hold;
};

/**
 * @brief This struct defines the data specifically for the lateral MPC problem.
 *
 */
struct LatMPCData : MPCData {
  Eigen::MatrixXd u_2;
  Tube tube;
};

}  // namespace planning
}  // namespace zark
