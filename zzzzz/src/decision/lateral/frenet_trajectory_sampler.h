/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file frenet_trajectory_sampler.h
 **/

#pragma once

#include <queue>
#include <vector>

#include "gtest/gtest_prod.h"
#include "pnc_point.h"
#include "apps/planning/src/common/math/curve1d/quartic_polynomial_curve1d.h"
#include "apps/planning/src/common/math/curve1d/quintic_polynomial_curve1d.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

/**
 * class FrenetTrajectorySampler
 * @brief Trajectory sampling in the frenet frame.
 */
class FrenetTrajectorySampler {
 public:
  static constexpr int kNumStates = 3;

  FrenetTrajectorySampler() = default;

  FrenetTrajectorySampler(LateralDeciderConfig::FrenetTrajectorySampler config);

  /**
   * @brief Trajectory compute in the frenet frame.
   *
   * @param s_init Trajectory s init state <s,v,a>.
   * @param l_init Trajectory target state <l,dl,ddl>.
   * @param s_target Trajectory s target state <s,v,a>.
   * @param l_target Trajectory l target state <l,dl,ddl>.
   * @return std::vector<::common::FrenetPoint>>
   * A list of trajectory points.
   */
  std::vector<::common::FrenetPoint> ComputeOptimalTrajectory(
      const std::array<double, kNumStates> s_init,
      const std::array<double, kNumStates> l_init,
      const std::array<double, kNumStates> s_target,
      const std::array<double, kNumStates> l_target);

  LateralDeciderConfig::FrenetTrajectorySampler SampleConfig() {
    return config_;
  }

 private:
  /**
   * @brief Trajectory evaluate.
   *
   * @param lon_trajectory Longitudinal trajectory.
   * @param lat_trajectory Lateral trajectory.
   * @return Trajectroy total cost.
   */
  double EvaluateTrajectoryCost(const Curve1d& lon_trajectory,
                                const Curve1d& lat_trajectory);

  double EvaluateTrajectoryCost(const PolynomialCurve1d& lon_trajectory,
                                const PolynomialCurve1d& lat_trajectory);

  /**
   * @brief Calculate longitudinal trajectroy cost.
   *
   * @param trajectory1d Longitudinal/Lateral trajectory.
   * @param cost_parameter Longitudinal/Lateral cost parameter.
   * @return Trajectroy cost.
   */
  double CalcTrajCost(
      const Curve1d& trajectory1d,
      LateralDeciderConfig::FrenetTrajectorySampler::Cost cost_parameter);

  /**
   * @brief Trajectory feasibility check.
   *
   * @param trajectory1d Longitudinal/Lateral trajectory.
   * @return Trajectroy feasible flag.
   */
  bool IsTrajectoryFeasible(const Curve1d& trajectory1d);

  /**
   * @brief Calculate max overshoot cost.
   *
   * @param trajectory1d Lateral trajectory.
   * @param config_cost Lateral cost parameter.
   * @return Trajectroy overshoot cost.
   */
  double CalcMaxTrajOvershoot(
      const PolynomialCurve1d& trajectory1d,
      const LateralDeciderConfig::FrenetTrajectorySampler::Cost& config_cost);

 private:
  static constexpr int kIdxS = 0;
  static constexpr int kIdxV = 1;
  static constexpr int kIdxA = 2;
  static constexpr int kIdxJ = 3;

  LateralDeciderConfig::FrenetTrajectorySampler config_;

  FRIEND_TEST(FrenetTrajectorySamplerTest, TestEvaluateTrajectoryCost);
  FRIEND_TEST(FrenetTrajectorySamplerTest, TestCalcTrajCost);
  FRIEND_TEST(FrenetTrajectorySamplerTest, TestIsTrajectoryFeasible);
  FRIEND_TEST(FrenetTrajectorySamplerTest, TestSampleTrajectoryCostConvergence);
};

}  // namespace planning
}  // namespace zark
