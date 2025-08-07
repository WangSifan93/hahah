/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_constraint.h
 **/

#pragma once

#include "Eigen/Core"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/motion/lateral/data_type.h"

namespace zark {
namespace planning {

/**
 * class LateralConstraint
 * @brief This class builds the lateral MPC constraints.
 */

class LateralConstraint {
  /**
   * @brief This struct defines the lateral MPC constraints.
   *
   */
 public:
  struct Constraints {
    struct Constraint {
      Eigen::RowVectorXd l_min;
      Eigen::RowVectorXd l_max;
      Eigen::RowVectorXd l_dot_min;
      Eigen::RowVectorXd l_dot_max;
      Eigen::RowVectorXd psi_min;
      Eigen::RowVectorXd psi_max;
      Eigen::RowVectorXd psi_dot_min;
      Eigen::RowVectorXd psi_dot_max;
      Eigen::RowVectorXd delta_min;
      Eigen::RowVectorXd delta_max;
      Eigen::RowVectorXd delta_dot_min;
      Eigen::RowVectorXd delta_dot_max;
    };
    Constraint hard;
    Constraint stiff;
    Constraint soft;
  };

 public:
  LateralConstraint(const LateralOptimizerConfig& config) { config_ = config; }

  ~LateralConstraint() = default;

  /**
   * @brief This function assigns the lateral constraint.
   *
   * @param tube the lateral tube
   * @return lat mpc constraints
   */
  MPCData::Constraints AssignConstraints(const Tube& tube) const;

 private:
  /**
   * @brief This function converts all lateral constraints to MPC
   * constraints.
   *
   * @param constraints the set of lateral constraints
   * @return MPCConstraints the set of MPC constraints
   */
  MPCData::Constraints ConvertToMPCConstraints(Constraints& constraints) const;

  /**
   * @brief This function updates the indices inside the MPCConstraints struct.
   *
   * @param constraints the set of lateral constraints
   * @param mpc_constraints the set of MPC constraints
   */
  void UpdateMPCConstraintIndices(Constraints& constraints,
                                  MPCData::Constraints& mpc_constraints) const;

  /**
   * @brief This function updates the sizes inside the MPCConstraints struct.
   *
   * @param mpc_constraints the set of MPC constraints
   */
  void UpdateMPCConstraintSizes(MPCData::Constraints& mpc_constraints) const;

  /**
   * @brief This function updates the slack weights inside the MPC constraints.
   *
   * @param mpc_constraints the set of MPC constraints
   * @param cost the cost of lateral MPC config
   */
  void UpdateMPCConstraintWeights(const LateralOptimizerConfig::Cost& cost,
                                  MPCData::Constraints& mpc_constraints) const;

  /**
   * @brief This function converts a lateral constraint of interest to an MPC
   * constraint.
   *
   * @param constraints the set of lateral constraints
   * @param mpc_constraints the set of MPC constraints
   */
  void UpdateMPCConstraints(const Constraints& constraints,
                            MPCData::Constraints& mpc_constraints) const;

 private:
  FRIEND_TEST(LateralConstraintTest, TestConvertToMPCConstraints);
  FRIEND_TEST(LateralConstraintTest, TestUpdateMPCConstraintIndices);
  FRIEND_TEST(LateralConstraintTest, TestUpdateMPCConstraintSizes);
  FRIEND_TEST(LateralConstraintTest, TestUpdateMPCConstraintWeights);
  FRIEND_TEST(LateralConstraintTest, TestUpdateMPCConstraints);
  FRIEND_TEST(LateralMPCProblemTest, TestLaterSolve);

 private:
  LateralOptimizerConfig config_;
};

}  // namespace planning
}  // namespace zark
