/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_optimizer.h
 **/

#pragma once

#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/math/mpc/longitudinal_mpc_problem.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_constraint.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_reference.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_stop_hold.h"
#include "apps/planning/src/tasks/task.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

class LongitudinalOptimizer : public Task {
 public:
  explicit LongitudinalOptimizer(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  ::common::Status Execute(Frame* frame);

 private:
  /**
   * @brief Process each proposal by running the longitudinal optimizer
   *
   * @param planning_start_point start point of the plan
   * @param proposal  proposal
   * @param state_management State Management Signal
   * @return common::Status solver status
   */
  ::common::Status Process(
      const ::common::TrajectoryPoint& planning_start_point, Proposal& proposal,
      const std::shared_ptr<zark::ads_fct::FCT_Outputs> state_management);

  /**
   * @brief  Get init state according to start point
   *
   * @param corridor_info corridor info
   * @param planning_start_point planning start point
   * @return std::tuple<Eigen::VectorXd, Eigen::VectorXd> init state
   */
  std::tuple<Eigen::VectorXd, Eigen::VectorXd> ConstructEgoInitialState(
      const CorridorInfo* corridor_info,
      const ::common::TrajectoryPoint& planning_start_point);

  /**
   * @brief Get the Cost Function Matrix object
   *
   * @param config LongitudinalOptimizerConfig
   * @return std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
   */
  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
  GetCostFunctionMatrix(const LongitudinalOptimizerConfig& config);

 private:
  std::shared_ptr<LongitudinalBlocker> lon_blocker_;
  std::shared_ptr<LongitudinalReference> lon_ref_;
  std::shared_ptr<LongitudinalConstraint> lon_con_;
  std::shared_ptr<LongitudinalPadding> lon_padding_;
  std::shared_ptr<LongitudinalLookupTables> lon_table_;
  std::shared_ptr<LongitudinalStopHold> lon_stop_hold_;
  LongitudinalOptimizerConfig config_;
  std::shared_ptr<LongitudinalMPCProblem> lon_mpc_problem_;

 private:
  FRIEND_TEST(LongitudinalOptimizerTest, TestExecute);
  FRIEND_TEST(LongitudinalOptimizerTest, TestProcess);
  FRIEND_TEST(LongitudinalOptimizerTest, TestConstructEgoInitialState);
  FRIEND_TEST(LongitudinalOptimizerTest, TestGetCostFunctionMatrix);
};

}  // namespace planning
}  // namespace zark
