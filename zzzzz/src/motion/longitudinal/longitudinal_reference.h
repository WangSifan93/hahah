/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_reference.h
 **/

#pragma once
#include <vector>

#include "Eigen/Core"
#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/common/speed_limit.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_blocker.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_padding.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

class LongitudinalReference {
 public:
  LongitudinalReference() = default;

  LongitudinalReference(const LongitudinalOptimizerConfig& config);

  /**
   * @brief deseign the reference trajectory for the longitudinal MPC
   * @param x_init the initial state x_init = [s_init; v_init]
   * @param speed_limit the speed limit curve on s-v graph
   * @param blockers longitudinal blockers with front/rear relations identified
   * @param time_gap_level time gap level from state management
   * @param lon_lookup_tables longitudinal lookup tables
   * @param lon_padding longitudinal paddings
   * @param speed_limit_set the set of speed limit curves for all time steps
   * @return the reference trajectory x_ref = [s_ref; v_ref] and u_ref = [a_ref]
   */
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> DesignReferenceTrajectory(
      const Eigen::VectorXd& x_init, const SpeedLimit& speed_limit,
      const std::vector<Blocker>& blockers, const TimeGapLevel& time_gap_level,
      const LongitudinalLookupTables& lon_lookup_tables,
      const LongitudinalPadding& lon_padding,
      std::vector<SpeedLimit>& speed_limit_set);

 private:
  /**
   * @brief design speed limit curve for obstacles
   * @param x_init the initial state x_init = [s_init; v_init]
   * @param speed_limit the speed limit curve on s-v graph
   * @param blockers longitudinal blockers with front/rear relations identified
   * @param sv_follow_pairs the critical obstacle sv pairs for all time steps
   * @param time_gap_level time gap level from state management
   * @param lon_lookup_tables longitudinal lookup tables
   * @param lon_padding longitudinal paddings
   * @return smoothed speed limit
   */
  std::vector<SpeedLimit> DesignSpeedLimitCurveForObstacles(
      const Eigen::VectorXd& x_init, const SpeedLimit& speed_limit_map_smooth,
      const std::vector<Blocker>& blockers,
      std::vector<std::vector<std::pair<double, double>>>& sv_follow_pairs,
      const TimeGapLevel& time_gap_level,
      const LongitudinalLookupTables& lon_lookup_tables,
      const LongitudinalPadding& lon_padding);

  /**
   * @brief get lower speed limit curve for obstacles
   * @param s_follow follow s [m]
   * @param v_follow follow v [m/s]
   * @param speed_limit speed_limit the speed limit curve on s-v graph
   */
  void LowerSpeedLimitCurveForObstacle(const double s_follow,
                                       const double v_follow,
                                       SpeedLimit& speed_limit);

  /**
   * @brief forward simulate the reference trajectory
   * @param x_init the initial state x_init = [s_init; v_init]
   * @param speed_limit_set the set of speed limit curves for all time steps
   * @param sv_follow_pairs the critical obstacle sv pairs for all time steps
   * @param a_accel_table a pre-defined acceleration lookup table
   * @param a_min_soft_table a pre-defined deceleration lookup table
   * @return the simulated reference trajectory
   */
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ForwardSimulateTrajectory(
      const Eigen::VectorXd& x_init, std::vector<SpeedLimit> speed_limit_set,
      const std::vector<std::vector<std::pair<double, double>>>&
          sv_follow_pairs,
      const LookupTable& a_accel_table, const LookupTable& a_min_soft_table);

  /**
   * @brief forward simulate the reference trajectory with Version 2
   * @param x_init the initial state x_init = [s_init; v_init]
   * @param speed_limit_map_smooth the set of speed limit curves for map limits
   * @param blockers longitudinal blockers with front/rear relations identified
   * @return the simulated reference trajectory
   */
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ForwardSimulateTrajectoryV2(
      const Eigen::VectorXd& x_init, const SpeedLimit& speed_limit_map_smooth,
      const std::vector<Blocker>& blockers, const LookupTable& a_accel_table,
      const LookupTable& a_min_soft_table, const TimeGapLevel& time_gap_level,
      const LongitudinalLookupTables& lon_lookup_tables,
      const LongitudinalPadding& lon_padding);

  /**
   * @brief compute max decel for reference traj
   * @param x_init the initial state x_init = [s_init; v_init]
   * @param sv_follow_pairs the critical obstacle sv pairs for all time steps
   * @return a_ref_min
   */
  double ComputeMaxDecelForRefTraj(
      const Eigen::VectorXd& x_init,
      const std::vector<std::vector<std::pair<double, double>>>&
          sv_follow_pairs);

  /**
   * @brief smooth the original speed limit curve by an accel table and a
   * constant decel
   * @param speed_limit the original speed_limit curve
   * @param a_accel_table a pre-defined acceleration lookup table
   * @param a_decel a pre-defined constant deceleration [m/s^2]
   * @return SpeedLimit the smoothed speed limit curve
   */
  SpeedLimit SmoothSpeedLimitCurve(const SpeedLimit& speed_limit,
                                   const LookupTable& a_accel_table,
                                   const double& a_decel);

 private:
  LongitudinalOptimizerConfig config_;

 private:
  FRIEND_TEST(LongitudinalReferenceTest, TestDesignReferenceTrajectory);
  FRIEND_TEST(LongitudinalReferenceTest, TestDesignSpeedLimitCurveForObstacles);
  FRIEND_TEST(LongitudinalReferenceTest, TestLowerSpeedLimitCurveForObstacle);
  FRIEND_TEST(LongitudinalReferenceTest, TestForwardSimulateTrajectory);
  FRIEND_TEST(LongitudinalReferenceTest, TestForwardSimulateTrajectoryV2);
  FRIEND_TEST(LongitudinalReferenceTest, TestComputeMaxDecelForRefTraj);
  FRIEND_TEST(LongitudinalReferenceTest, TestSmoothSpeedLimitCurve);
};

}  // namespace planning
}  // namespace zark
