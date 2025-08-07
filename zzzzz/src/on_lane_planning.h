/******************************************************************************
 * Copyright 2023 The zpilot Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/planner/on_lane_planner_dispatcher.h"
#include "apps/planning/src/planning_base.h"
#include "apps/planning/src/reference_line_provider/local_route_provider.h"
#include "apps/planning/src/mission/force_lane_change_decision.h"
#include "gtest/gtest.h"

/**
 * @namespace zark::planning
 * @brief zark::planning
 */
namespace zark {
namespace planning {

// using namespace common::VehicleState;
/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class OnLanePlanning : public PlanningBase {
 public:
  explicit OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector), scenario_manager_(injector) {
    planner_dispatcher_ = std::make_unique<OnLanePlannerDispatcher>();
  }
  virtual ~OnLanePlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  Status Init(const PlanningConfig& config) override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * prediction msgs.
   * @param local_view input information
   * @param trajectory_pb output trajectory
   */
  void RunOnce(const LocalView& local_view,
               ADCTrajectory& trajectory_pb) override;

  /**
   * @brief main plan process function
   * @param current_time_stamp start time stamp [Unix timestamp].
   * @param planning_start_point planning start point
   * @param trajectory output trajectory
   * @param need_replan need replan flag
   * @return planning Status
   */
  Status Plan(const double current_time_stamp,
              const ::common::TrajectoryPoint& planning_start_point,
              ADCTrajectory& trajectory, const bool need_replan) override;

 private:
  /**
   * @brief init frame function
   * @param sequence_num sequence number.
   * @param planning_start_point planning start point
   * @param vehicle_state vehicle state
   * @return Init Frame Status
   */
  Status InitFrame(const uint32_t sequence_num,
                   const ::common::TrajectoryPoint& planning_start_point,
                   const common::VehicleState& vehicle_state);

  /**
   * @brief check planning config is exit
   * @param config planning config.
   * @return exit state
   */
  bool CheckPlanningConfig(const PlanningConfig& config);

  /**
   * @brief Set ego vehicle parameters.
   * @param trajectory output trajectory.
   */
  void SetEgoVehicleParam(ADCTrajectory& trajectory);

  /**
   * @brief compute the trajectory point from vehicle state when replan is
   * needed
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @param vehicle_state vehicle state
   * @param vehicle_param vehicle parameters
   * @return trajectory point
   */

  ::common::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      const double& dt_prev, const common::VehicleState& vehicle_state,
      const common::VehicleParam& vehicle_param) const;

  /**
   * @brief compute the planning start point when lon replan is needed
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @param traj_prev previous published trajectory
   * @param vehicle_state vehicle state
   * @param vehicle_param vehicle parameters
   * @return trajectory point
   */
  ::common::TrajectoryPoint ComputePlanningStartPointForACC(
      const double dt_prev, const PublishableTrajectory& traj_prev,
      const common::VehicleState& vehicle_state,
      const common::VehicleParam& vehicle_param) const;

  /**
   * @brief compute the planning start point when replan is NOT needed
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @param traj_prev previous published trajectory
   * @param vehicle_state vehicle state
   * @return trajectory point
   */
  ::common::TrajectoryPoint ComputePlanningStartPoint(
      const double dt_prev, const PublishableTrajectory& traj_prev,
      const common::VehicleState& vehicle_state) const;

  /**
   * @brief check is need replan
   * @param replan_reason replan reason.
   * @return is need replan
   */
  bool CheckNeedReplan(std::string& replan_reason) const;

  /**
   * @brief check if need latreplan
   * @param replan_reason replan reason.
   * @return is need replan
   */
  bool CheckNeedLatReplan(std::string& replan_reason) const;

  /**
   * @brief Retrieves local routes
   *
   * Retrieves local routes from the local route provider and saves them to
   * the provided list of local routes.
   * @param[in] planning_start_point Start point for acc local routes
   * @param[in] is_acc_mode The flag indicates if it is acc mode
   * @param[out] local_routes Reference to the list of local routes to be
   * populated.
   */
  void GetLocalRoutes(
      const ::common::TrajectoryPoint& planning_start_point,
      const bool is_acc_mode,
      std::list<zark::planning::LocalRoute>& local_routes) const {
    if (!local_route_provider_->GetLocalRoutes(planning_start_point, is_acc_mode,
                                               local_routes)) {
      const std::string msg = "Failed to get local route.";
      AERROR << msg;
    }
  }

  /**
   * @brief Update the auto mode of the vehicle
   *
   */
  void UpdateAutoMode();

  /**
   * @brief Check if the vehicle in auto mode
   *
   * @return true
   * @return false
   */
  const bool IsInAutoMode() const { return auto_mode_ != AutoMode::MANUAL; }

  /**
   * @brief run noa_decision for routing lane change
   */
  void NoaDecisionProcess();

  /**
   * @brief Transform trajectory point to vehicle body frame
   *
   * @param traj
   * @return DiscretizedTrajectory
   */
  DiscretizedTrajectory ConvertTrajectoryToBodyFrame(
      const DiscretizedTrajectory& traj,
      const common::VehicleState& vehicle_state);

 private:
  std::unique_ptr<LocalRouteProvider> local_route_provider_;
  std::unique_ptr<ForceLaneChangeDecision> noa_decision_;
  scenario::ScenarioManager scenario_manager_;
  PolynomialFit poly_fit_;
  NavigationInfo refline_navigation_info_;
  ads_decision::DEC_Outputs refline_fitted_result_;

  AutoMode auto_mode_ = AutoMode::MANUAL;
  AutoMode auto_mode_prev_ = AutoMode::MANUAL;
  bool is_in_ies_mode_ = false;

 private:
  FRIEND_TEST(OnLanePlanningTest, TestComputeTrajectoryPointFromVehicleState);
  FRIEND_TEST(OnLanePlanningTest, TestComputePlanningStartPoint);
};

}  // namespace planning
}  // namespace zark
