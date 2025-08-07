/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_optimizer.h
 **/

#pragma once

#include <string>

#include "apps/planning/src/common/conversion.h"
#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/math/mpc/lateral_mpc_problem.h"
#include "cartesian_frenet_conversion.h"
#include "math_utils.h"
#include "point_factory.h"
#include "apps/planning/src/motion/lateral/data_type.h"
#include "apps/planning/src/motion/lateral/lateral_constraint.h"
#include "apps/planning/src/motion/lateral/lateral_nudger.h"
#include "apps/planning/src/motion/lateral/lateral_reference.h"
#include "apps/planning/src/motion/lateral/lateral_tube.h"
#include "apps/planning/src/motion/lateral/lateral_stop_hold.h"
#include "apps/planning/src/motion/lateral/lateral_mpc_data_mocker.h"
#include "apps/planning/src/tasks/task.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

class LateralOptimizer : public Task {
 public:
  explicit LateralOptimizer(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  ~LateralOptimizer() = default;

  ::common::Status Execute(Frame* frame);

 private:
  /**
   * @brief Process each proposal by running the longitudinal optimizer
   *
   * @param proposal  proposal
   * @param planning_start_point start point of the plan
   * @param cur_local_route current local route
   * @return common::Status solver status
   */
  ::common::Status Process(
      const ::common::TrajectoryPoint& planning_start_point,
      const LocalRoute& cur_local_route, Proposal& proposal);

  /**
   * @brief convert the trajetory from lateral mpc data and lon mpc data
   * @param proposal proposal
   * @return trajetory
   */
  DiscretizedTrajectory ConvertToTrajectory(Proposal& proposal) const;

  /**
   * @brief Construct the initial state for the ego vehicle.
   *
   * This function constructs the initial state for the ego vehicle based on the
   * provided corridor information and the planning start point.
   *
   * @param planning_start_point The planning start point containing initial
   * @param corridor_info Pointer to the corridor information.
   * state information.
   * @return A pair of Eigen::VectorXd representing the initial state for the
   * ego vehicle. The first element represents the initial state vector, and the
   * second element represents the initial control input vector.
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> ConstructEgoInitialState(
      const ::common::TrajectoryPoint& planning_start_point,
      const CorridorInfo& corridor_info) const;

  /**
   * @brief Compute the control input matrix u_2.
   *
   * This function computes the control input matrix u_2 based on the provided
   * longitudinal MPC data, tube information, and corridor information.
   *
   * @param lon_mpc_data The longitudinal MPC data containing information about
   * the longitudinal control.
   * @param tube The tube containing trajectory points.
   * @param corridor_info Pointer to the corridor information.
   * @return An Eigen::MatrixXd representing the control input matrix u_2.
   */
  Eigen::MatrixXd ComputeU2(const LonMPCData& lon_mpc_data, const Tube& tube,
                            const CorridorInfo& corridor_info);

  /**
   * @brief Get the Cost Function Matrix object
   *
   * @param config LateralOptimizerConfig
   * @return std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
   */
  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
  GetCostFunctionMatrix(const LateralOptimizerConfig& config);

 private:
  std::shared_ptr<LateralNudger> lat_nudger_;
  std::shared_ptr<LateralTube> lat_tube_;
  std::shared_ptr<LateralReference> lat_ref_;
  std::shared_ptr<LateralConstraint> lat_con_;
  std::shared_ptr<LateralPadding> lat_padding_;
  std::shared_ptr<LateralLookupTables> lat_table_;
  std::shared_ptr<LateralMPCProblem> lat_mpc_problem_;
  std::shared_ptr<LateralStopHold> lat_stop_hold_;
  std::shared_ptr<LateralMPCDataMocker> lat_mpc_data_mocker_;

  LateralOptimizerConfig config_;

  Eigen::VectorXd Q_;
  Eigen::VectorXd R_;
  Eigen::VectorXd R_dot_;

 private:
  FRIEND_TEST(LateralOptimizerTest, TestConvertToTrajectory);
  FRIEND_TEST(LateralOptimizerTest, TestProcess);
  FRIEND_TEST(LateralOptimizerTest, TestProcessStopTrajectory);
  FRIEND_TEST(LateralOptimizerTest, TestConstructEgoInitialState);
  FRIEND_TEST(LateralOptimizerTest, TestGetCostFunctionMatrix);
  FRIEND_TEST(LateralOptimizerTest, TestComputeU2);
};

}  // namespace planning
}  // namespace zark
