/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file frenet_trajectory_map_builder.h
 **/

#pragma once

#include <unordered_map>
#include <memory>
#include <vector>

#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/common/mission.h"
#include "apps/planning/src/common/hysteresis.h"
#include "apps/common/basic_msgs/pnc_point.h"
#include "apps/planning/src/decision/lateral/frenet_trajectory_sampler.h"
#include "apps/planning/src/common/dependency_injector.h"
#include "apps/common/math/cartesian_frenet_conversion.h"
#include "apps/planning/src/common/frame.h"

using common::CartesianPoint;
using common::FrenetPoint;
using common::TrajectoryPoint;

namespace zark {
namespace planning {

using FrenetTrajectory = std::vector<FrenetPoint>;
using CartesianTrajectory = std::vector<CartesianPoint>;
using TrajectoryMap =
    std::unordered_map<CorridorInfo::Type,
                       std::pair<CartesianTrajectory, double>>;

class FrenentTrajectoryMapBuilder {
 public:
  FrenentTrajectoryMapBuilder(const LateralDeciderConfig &config);

  /**
   * @brief generate trajectory information map for corridor create.
   * @param mission Mission.
   * @param corridor_start_point Corridor start point.
   * @param planning_start_point Planning start point.
   * @param obstacles Obstacles in the frame.
   * @param current_local_route Current local route.
   * @param target_local_route Target local route.
   * @param v_map Velocity map.
   * @param corridor_info_prev Last corridor information.
   * @return trajectory_map Trajectory map.
   */
  TrajectoryMap GenerateTrajectoryMap(
      const Mission &mission, const CorridorPoint &corridor_start_point,
      const TrajectoryPoint &planning_start_point,
      const LocalRoute &current_local_route,
      const LocalRoute &target_local_route,
      const std::vector<const Obstacle *> &obstacles, const double v_map,
      const CorridorInfo *const corridor_info_prev);

  /**
   * @brief generate trajectory information for corridor create.
   *
   * @param current_local_route Current local route.
   * @param target_local_route Target local route.
   * @param corroidor_start_point Corridor start point.
   * @param v_map Velocity map.
   * @param type Corridor type.
   * @param trajectory_map Trajectory map.
   * @param traj_frenet frenet trajectory
   * @return CartesianTrajectory.
   */
  std::pair<CartesianTrajectory, double> CreateTrajectoryInfo(
      const LocalRoute &current_local_route,
      const LocalRoute &target_local_route,
      const CorridorPoint &corroidor_start_point, const double v_map,
      const CorridorInfo::Type &type, FrenetTrajectory *traj_frenet = nullptr);

  /**
   * @brief add trajectory information to trajectory map.
   *
   * @param init_frenet_point Init frenet point.
   * @param v_target Target velocity.
   * @param l_target Target l.
   * @param type corridor type.
   */
  FrenetTrajectory GenerateTrajectory(const FrenetPoint &init_frenet_point,
                                      const double v_target,
                                      const double l_target,
                                      const CorridorInfo::Type type);

  /**
   * @brief Generate trajectory information for corridor create.
   *
   * @param corroidor_start_point Corridor start point.
   * @param local_route Local route.
   * @param v_target Target velocity.
   * @return Nudge FrenetTrajectories.
   */
  std::vector<FrenetTrajectory> GenerateNudgeTrajectories(
      const CorridorPoint &corroidor_start_point, const LocalRoute &local_route,
      const double v_target);

  /**
   * @brief Filter trajectorys for nudge .
   *
   * @param planning_start_point Planning start point.
   * @param lane_keep_trajectory Lane keep trajectory.
   * @param nudge_trajectories Nudge trajectorys.
   * @param local_route Current local route.
   * @param obstacles Obstacles in the environment.
   * @param corridor_info_prev Last corridor information.
   * @return Filter nudge trajectory.
   */
  void FilterNudgeTrajectories(
      const TrajectoryPoint &planning_start_point,
      const FrenetTrajectory &lane_keep_trajectory,
      const LocalRoute &local_route,
      const std::vector<const Obstacle *> &obstacles,
      std::vector<FrenetTrajectory> &nudge_trajectories,
      const CorridorInfo *const corridor_info_prev);

  /**
   * @brief Fileter nudge trajectories by current lane boundary and road
   * boundary.
   *
   * @param local_route Local route.
   * @param nudge_trajectories nudge corridor trajectories.
   */
  void FilterTrajectoriesByBoundaries(
      const LocalRoute &local_route,
      std::vector<FrenetTrajectory> &nudge_trajectories);

  /**
   * @brief Fileter nudge trajectories by lon progress.
   *
   * @param planning_start_point Planning start point.
   * @param local_route Local route.
   * @param obstacles Obstacles in the environment.
   * @param lane_keep_travel_time Lane keep progress time [s].
   * @param nudge_obstacle_velocity Need nudge obstacle velocity [m/s].
   * @param nudge_trajectories Nudge corridor trajectories.
   * @param corridor_info_prev Last corridor information.
   */
  void FilterTrajectoriesByProgress(
      const TrajectoryPoint &planning_start_point,
      const LocalRoute &local_route,
      const std::vector<const Obstacle *> &obstacles,
      const double lane_keep_travel_time, const double nudge_obstacle_velocity,
      std::vector<FrenetTrajectory> &nudge_trajectories,
      const CorridorInfo *const corridor_info_prev);

  /**
   * @brief Sort nudge trajectories by similarity with the latest corridor.
   *
   * @param planning_start_point Planning start point.
   * @param nudge_trajectories Nudge trajectories
   * @param corridor_info_prev Last corridor information.
   */
  void SortTrajectoriesBySimilarity(
      const TrajectoryPoint &planning_start_point,
      std::vector<FrenetTrajectory> &nudge_trajectories,
      const CorridorInfo *const corridor_info_prev);

  /**
   * @brief Calculate similarity between nudge trajectory and the latest
   * corridor.
   *
   * @param planning_start_point Planning start point.
   * @param traj_frenet Trajectory in the frenet coordinate system.
   * @param last_corridor_info Latest corridor information.
   * @return double
   */
  double CaculateNudgeTrajectorySimilarity(
      const TrajectoryPoint &planning_start_point,
      const FrenetTrajectory &traj_frenet,
      const CorridorInfo *const corridor_info_prev);

  /**
   * @brief Find closest front obstacle.
   *
   * @param local_route Local route.
   * @param obstacles Obstacles in the frame.
   * @param type Corridor type.
   * @return  Region obstacles.
   */
  const std::vector<const Obstacle *> GetObstacles(
      const LocalRoute &local_route,
      const std::vector<const Obstacle *> &obstacles,
      const CorridorInfo::Type type, const bool is_left = true);

  /**
   * @brief Convert frenet trajectory to cartesian trajectory.
   * @param local_route Local route
   * @param traj_frenet Frenet trajectory
   * @return Cartesian trajectory
   *
   */
  CartesianTrajectory ConvertFrenetTrajToCartesianTraj(
      const LocalRoute &local_route, const FrenetTrajectory &traj_frenet);

  /**
   * @brief Calculate travelable distance of corridor.
   *
   * @param traj_frenet Frenet trajectory.
   * @param obstacle Front closest obstacle.
   * @param local_route Local route.
   * @param obstacles Obstacles in the frame.
   * @param ego_obstacle_min_delta_l min delta l between ego and obstacle.
   * @return Travelable time [s].
   */
  double CalcTrajectoryTravelableTime(
      const FrenetTrajectory &traj_frenet, const LocalRoute &local_route,
      const std::vector<const Obstacle *> &obstacles,
      double &ego_obstacle_min_delta_l, double *v_overlap_obs = nullptr);

  /**
   * @brief Calcualte target velocity based on map speed limit and ego
   * velocity.
   *
   * @param local_route Local route.
   * @param v_map Map limit velocity.
   * @return Double Target velocity.
   */
  double CalculateVTarget(const LocalRoute &local_route, const double v_map,
                          const CorridorInfo::Type &type);

  /**
   * @brief Check if nudge condition ready.
   * @param v_ego Ego velocity
   * @param v_obstacle Obstacle velocity
   * @return Ready result
   */
  bool CheckNudgeConditionReady(const double v_ego, const double v_obstacle);

  /**
   * @brief Determine initial and terminal state of frenet trajectory sampling.
   *
   * @param init_frenet_point Init frenet point.
   * @param v_target Target velocity.
   * @param l_target Target l.
   * @param type Corridor type.
   * @param s_init Initial s, sdot and sdotdot.
   * @param l_init Initial l, ldot and ldotdot.
   * @param s_terminal Terminal s, sdot and sdotdot.
   * @param l_terminal Terminal l, ldot and ldotdot.
   */
  void DetermineInitAndTerminalState(
      const FrenetPoint &init_frenet_point, const double v_target,
      const double l_target, const CorridorInfo::Type type,
      std::array<double, FrenetTrajectorySampler::kNumStates> &s_init,
      std::array<double, FrenetTrajectorySampler::kNumStates> &l_init,
      std::array<double, FrenetTrajectorySampler::kNumStates> &s_terminal,
      std::array<double, FrenetTrajectorySampler::kNumStates> &l_terminal);

  /**
   * @brief Caculate speed limit by curvature.
   *
   * @param local_route Local route.
   * @return double
   */
  double CaculateSpeedLimitByCurvature(const LocalRoute &local_route);

  /**
   * @brief Get the Frenet Trajectory Sampler object for test
   *
   * @return std::shared_ptr<FrenetTrajectorySampler>
   */
  std::shared_ptr<FrenetTrajectorySampler> GetFrenetTrajectorySamplerLK() const;

  /**
   * @brief Get the Frenet Trajectory Sampler object for test
   *
   * @return std::shared_ptr<FrenetTrajectorySampler>
   */
  std::shared_ptr<FrenetTrajectorySampler> GetFrenetTrajectorySamplerLC() const;

  /**
   * @brief Get the Frenet Trajectory Sampler Nudge object
   *
   * @return std::shared_ptr<FrenetTrajectorySampler>
   */
  std::shared_ptr<FrenetTrajectorySampler> GetFrenetTrajectorySamplerNudge()
      const;

  /**
   * @brief Update the Ego Realtime Velocity
   *
   * @param v ego realtime velocity
   */
  void SetEgoRealtimeVelocity(const double v) { ego_realtime_velocity_ = v; }

  /**
   * @brief Check if obstacle need ignore
   *
   * @param obstacle Obstacle
   * @param ego_position Ego frenet position
   * @return the result is the obstacle need ignore
   */
  bool CheckNudgeObstacleNeedIgnore(const Obstacle *obstacle,
                                    const FrenetPoint ego_position);

 private:
  static constexpr int kIdxL = 0;
  static constexpr int kIdxLDot = 1;
  static constexpr int kIdxLDDot = 2;
  static constexpr int kIdxS = 0;
  static constexpr int kIdxSDot = 1;
  static constexpr int kIdxSDDot = 2;

  static constexpr double kEpsilon = 1e-6;
  static constexpr double kKmPHToMPS = 3.6;

  LateralDeciderConfig::Corridor config_corridor_;
  std::shared_ptr<FrenetTrajectorySampler> frenet_trajectory_sampler_lc_;
  std::shared_ptr<FrenetTrajectorySampler> frenet_trajectory_sampler_lk_;
  std::shared_ptr<FrenetTrajectorySampler> frenet_trajectory_sampler_nudge_;
  std::shared_ptr<FrenetTrajectorySampler> frenet_trajectory_sampler_staging_;
  std::shared_ptr<hysteresis::Hysteresis> hysteresis_nudge_;
  LookupTable v_target_coeff_table_;
  LookupTable a_lat_max_table_;
  double ego_realtime_velocity_;

  FRIEND_TEST(FrenetTrajectoryMapBuilderTest, TestCalculateVTarget);
  FRIEND_TEST(FrenetTrajectoryMapBuilderTest,
              TestDetermineInitAndTerminalState);
  FRIEND_TEST(FrenetTrajectoryMapBuilderTest, TestCalcTrajectoryTravelableTime);
  FRIEND_TEST(FrenetTrajectoryMapBuilderTest, TestGetObstacles);
  FRIEND_TEST(FrenetTrajectoryMapBuilderTest,
              TestConvertFrenetTrajToCartesianTraj);
};

}  // namespace planning
}  // namespace zark
