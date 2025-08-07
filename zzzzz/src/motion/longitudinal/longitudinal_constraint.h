/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_constraint.h
 **/

#pragma once

#include <Eigen/Core>
#include <vector>

#include "apps/planning/src/motion/longitudinal/data_type.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_blocker.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_padding.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

/**
 * @brief This class builds the longitudinal MPC constraints.
 *
 */
class LongitudinalConstraint {
 public:
  /**
   * @brief This struct defines the longitudinal MPC constraints.
   *
   */
  struct Constraints {
    struct Constraint {
      Eigen::RowVectorXd s_min;
      Eigen::RowVectorXd s_max;
      Eigen::RowVectorXd v_min;
      Eigen::RowVectorXd v_max;
      Eigen::RowVectorXd a_min;
      Eigen::RowVectorXd a_max;
      Eigen::RowVectorXd j_min;
      Eigen::RowVectorXd j_max;
    };
    Constraint hard;
    Constraint stiff;
    Constraint soft;
  };

 public:
  explicit LongitudinalConstraint(const LongitudinalOptimizerConfig& config);
  ~LongitudinalConstraint() = default;

  /**
   * @brief This function design the Constraints (hard, soft, stiff)
   *
   * @param x_init the initial state x_init = [s_init; v_init]
   * @param x_ref the reference trajectory x_ref = [s_ref; v_ref]
   * @param blockers blockers in the ST graph with front/rear relations defined
   * @param speed_limit the speed limit based s-v gragh
   * @param time_gap_level time gap level from state management
   * @param lon_lookup_tables LongitudinalLookupTables
   * @param lon_padding stiff and soft padding for front and rear blockers
   * @return MPCData::Constraints the set of MPC constraints
   */
  MPCData::Constraints DesignConstraints(
      const Eigen::VectorXd& x_init, const Eigen::MatrixXd x_ref,
      const std::vector<Blocker>& blockers, const SpeedLimit& speed_limit,
      const TimeGapLevel& time_gap_level,
      const LongitudinalLookupTables& lon_lookup_tables,
      const LongitudinalPadding& lon_padding);

 private:
  FRIEND_TEST(LongitudinalConstraintTest, TestDesignConstraints);
  FRIEND_TEST(LongitudinalConstraintTest, TestUpdateMPCConstraintIndices);
  FRIEND_TEST(LongitudinalConstraintTest, TestUpdateMPCConstraintSizes);
  FRIEND_TEST(LongitudinalConstraintTest, TestUpdateMPCConstraintWeights);
  FRIEND_TEST(LongitudinalConstraintTest, TestUpdateMPCConstraints);

 private:
  /**
   * @brief This function converts all longitudinal constraints to MPC
   * constraints.
   *
   * @param constraints the set of longitudinal constraints
   * @return MPCData::Constraints the set of MPC constraints
   */
  MPCData::Constraints ConvertToMPCConstraints(const Constraints& constraints);

  /**
   * @brief This function updates the indices inside the MPCData::Constraints
   * struct.
   *
   * @param constraints the set of longitudinal constraints
   * @param mpc_constraints the set of MPC constraints
   */
  void UpdateMPCConstraintIndices(const Constraints& constraints,
                                  MPCData::Constraints& mpc_constraints);

  /**
   * @brief This function updates the sizes inside the MPCData::Constraints
   * struct.
   *
   * @param mpc_constraints the set of MPC constraints
   */
  void UpdateMPCConstraintSizes(MPCData::Constraints& mpc_constraints);

  /**
   * @brief This function updates the slack weights inside the MPC constraints.
   *
   * @param cost_config proto config for long MPC costs
   * @param mpc_constraints the set of MPC constraints
   */
  void UpdateMPCConstraintWeights(
      const LongitudinalOptimizerConfig::Cost& cost_config,
      MPCData::Constraints& mpc_constraints);

  /**
   * @brief This function converts a longitudinal constraint of interest to an
   * MPC constraint.
   *
   * @param constraints the set of longitudinal constraints
   * @param mpc_constraints the set of MPC constraints
   */
  void UpdateMPCConstraints(const Constraints& constraints,
                            MPCData::Constraints& mpc_constraints);

 private:
  LongitudinalOptimizerConfig config_;
};

}  // namespace planning
}  // namespace zark
