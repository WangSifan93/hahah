/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file frenet_trajectory_map_builder_test.cc
 **/

#include "apps/planning/src/tasks/deciders/lateral_decider/lateral_decider.h"
#include "apps/planning/src/config/config_main.h"

#include <fstream>
#include <iostream>
#include <string>

#include "gtest/gtest.h"

namespace zark {
namespace planning {

using namespace zark::planning;
using common::VehicleConfigHelper;

class FrenetTrajectoryMapBuilderTest : public ::testing::Test {
 public:
  FrenetTrajectoryMapBuilderTest() {
    // vehicle
    common::VehicleParam vehicle_param;
    vehicle_param.set_width(2.11);
    vehicle_param.set_length(4.933);
    vehicle_param.set_back_edge_to_center(1.043);
    vehicle_param.set_front_edge_to_center(3.89);
    vehicle_param.set_left_edge_to_center(1.055);
    vehicle_param.set_right_edge_to_center(1.055);
    common::VehicleConfig vehicle_config;
    vehicle_config.set_vehicle_param(vehicle_param);
    VehicleConfigHelper::Init(vehicle_config);
    // task_config
    TaskConfig::task_config_union task_config;
    LateralDeciderConfig lateral_decider_config;
    LateralDeciderConfig::FrenetTrajectorySampler frenet_trajectory_sampler;
    frenet_trajectory_sampler.cost_lon =
        LateralDeciderConfig::FrenetTrajectorySampler::Cost{1.0, 1.0, 1.0};
    frenet_trajectory_sampler.cost_lat =
        LateralDeciderConfig::FrenetTrajectorySampler::Cost{1.0, 1.0, 1.0};
    frenet_trajectory_sampler.dt = 0.1;
    frenet_trajectory_sampler.dT = 0.1;
    frenet_trajectory_sampler.t_total = 10.0;
    LateralDeciderConfig::Corridor corridor;
    std::vector<double> points_x = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
                                    6.0, 7.0, 8.0, 9.0, 10.0};
    std::vector<double> points_y = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                                    0.6, 0.7, 0.8, 0.9, 1.0};
    corridor.v_target_coeff_table.points_x = points_x;
    corridor.v_target_coeff_table.points_y = points_y;
    corridor.v_min = 1.0;
    corridor.t_rear = 2.0;
    corridor.length_max = 20.0;
    corridor.length_min = 13.0;
    corridor.t_total = 10;
    corridor.extra_half_width = 0.8;
    corridor.t_staging = 2.0;
    corridor.obs_collision_buffer = 1.0;
    corridor.t_obs_collision_check = 5.0;
    corridor.sample_dist_min = 2.0;
    lateral_decider_config.corridor = corridor;
    lateral_decider_config.frenet_trajectory_sampler_lc =
        frenet_trajectory_sampler;
    task_config.lateral_decider_config = lateral_decider_config;
    // Set expected value for the speed limit
    LocalRouteConfig local_route_config;
    local_route_config.local_route_speed_limit = 10.0;
    local_route_config.min_local_route_speed_limit = 1.0;
    local_route_config.default_local_route_width = 6.0;
    local_route_config.obs_filter_front_end_s = 200;
    local_route_config.obs_filter_rear_start_s = -50;
    local_route_config.default_lane_width = 3.8;
    TaskConfig config;
    config.set_task_config(task_config);
    frenet_trajectory_map_builder_ =
        std::make_unique<FrenentTrajectoryMapBuilder>(config.task_config().lateral_decider_config);
  }

  ::common::Trajectory generate_trajectory(double x_start, double y_start,
                                           double x_end, double y_end) {
    ::common::Trajectory trajectory;
    const int num_points = 17;
    for (int i = 0; i < num_points; ++i) {
      double t = static_cast<double>(i) / (num_points - 1);
      double x = x_start + t * (x_end - x_start);
      double y = y_start + t * (y_end - y_start);
      ::common::PathPoint path_point;
      path_point.set_x(x);
      path_point.set_y(y);
      double angle = std::atan2(y_end - y_start, x_end - x_start);
      path_point.set_theta(angle);
      double v = 1.0;
      double a = 1.0;
      ::common::TrajectoryPoint trajectory_point;
      trajectory_point.set_path_point(path_point);
      trajectory_point.set_v(v);
      trajectory_point.set_a(a);
      trajectory_point.set_relative_time(1.0 * i);
      trajectory.mutable_trajectory_point()->emplace_back(trajectory_point);
    }
    return trajectory;
  }

 protected:
  std::unique_ptr<FrenentTrajectoryMapBuilder> frenet_trajectory_map_builder_;
};

TEST_F(FrenetTrajectoryMapBuilderTest, TestCalculateVTarget) {
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  double v_target = frenet_trajectory_map_builder_->CalculateVTarget(
      local_route, 1.0, CorridorInfo::Type::LANE_CHANGE);
  EXPECT_DOUBLE_EQ(v_target, 1.0);
}

TEST_F(FrenetTrajectoryMapBuilderTest, TestDetermineInitAndTerminalState) {
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 17; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // init_frenet_point
  std::array<double, FrenetTrajectorySampler::kNumStates> init_frenet_s = {
      2.0, 0.5, 0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> init_frenet_l = {
      0.0, 0.0, 0.0};

  FrenetPoint init_frenet_point;
  init_frenet_point.s = init_frenet_s;
  init_frenet_point.l = init_frenet_l;

  std::array<double, FrenetTrajectorySampler::kNumStates> s_init = {0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_init = {0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> s_terminal = {0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_terminal = {0.0};
  double v_target = 3.0;
  double l_target = 0.5;
  frenet_trajectory_map_builder_->DetermineInitAndTerminalState(
      init_frenet_point, v_target, l_target, CorridorInfo::Type::LANE_KEEP,
      s_init, l_init, s_terminal, l_terminal);
  EXPECT_EQ(s_init[0], init_frenet_point.s[0]);
  EXPECT_EQ(s_init[1], 1.0);
  EXPECT_EQ(s_init[2], init_frenet_point.s[2]);
  EXPECT_EQ(l_init[0], init_frenet_point.l[0]);
  EXPECT_EQ(l_init[1], init_frenet_point.l[1] * s_init[1]);
  EXPECT_EQ(l_init[2], init_frenet_point.l[2] * s_init[1] * s_init[1] +
                           init_frenet_point.l[1] * s_init[2]);
  EXPECT_EQ(s_terminal[1], v_target);
  EXPECT_EQ(l_terminal[0], l_target);
  frenet_trajectory_map_builder_->DetermineInitAndTerminalState(
      init_frenet_point, v_target, l_target, CorridorInfo::Type::STAGING,
      s_init, l_init, s_terminal, l_terminal);
  EXPECT_EQ(s_init[0], init_frenet_point.s[0] + init_frenet_point.s[1] * 2);
  EXPECT_EQ(s_init[1], 1.0);
  EXPECT_EQ(s_init[2], init_frenet_point.s[2]);
  EXPECT_EQ(l_init[0], init_frenet_point.l[0]);
  EXPECT_EQ(l_init[1], 0);
  EXPECT_EQ(l_init[2], 0);
  EXPECT_EQ(s_terminal[1], v_target);
  EXPECT_EQ(l_terminal[0], l_target);
}

TEST_F(FrenetTrajectoryMapBuilderTest, TestCalcTrajectoryTravelableTime) {
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  ::common::Point3D position(0.0, 7.5, 0.0);
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_position(position);
  perception_obstacle.set_theta(M_PI / 2);
  Obstacle obstacle_0("0", perception_obstacle);
  obstacle_0.SetRelativeRegion2LocalRoute(RelativeRegionType::FRONT);
  SLBoundary sl_boundary;
  if (!local_route.GetSLBoundary(obstacle_0.PerceptionBoundingBox(),
                                 sl_boundary)) {
    AERROR << " Failed to get sl boundary for obstacle:" << obstacle_0.Id();
  }
  obstacle_0.SetPerceptionSlBoundary(sl_boundary);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  FrenetPoint frenet_pt;
  frenet_pt.s[kIdxS] = 0.0;
  local_route.SetInitFrenetState(frenet_pt);

  std::vector<::common::FrenetPoint> trajectory;
  for (int i = 0; i < 25; i++) {
    FrenetPoint frenet_point;
    frenet_point.t = 0.4 * i;
    frenet_point.s[0] = -5.0 + 1.0 * i;
    frenet_point.s[1] = 1.0;
    frenet_point.s[2] = 0.0;
    frenet_point.l[0] = 0.0;
    trajectory.emplace_back(frenet_point);
  }
  double ego_obstacle_min_delta_l = 3.0;
  EXPECT_NEAR(frenet_trajectory_map_builder_->CalcTrajectoryTravelableTime(
                  trajectory, local_route, obstacles, ego_obstacle_min_delta_l),
              9.6, 0.01);
}

TEST_F(FrenetTrajectoryMapBuilderTest, TestGetObstacles) {
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  ::common::Point3D position(0.0, 7.5, 0.0);
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_position(position);
  perception_obstacle.set_theta(M_PI / 2);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 7.5, 0.0, 15.0);
  Obstacle obstacle_0("0", perception_obstacle, trajectory_0);
  obstacle_0.SetRelativeRegion2LocalRoute(RelativeRegionType::FRONT);
  SLBoundary sl_boundary;
  if (!local_route.GetSLBoundary(obstacle_0.PerceptionBoundingBox(),
                                 sl_boundary)) {
    AERROR << " Failed to get sl boundary for obstacle:" << obstacle_0.Id();
  }
  obstacle_0.SetPerceptionSlBoundary(sl_boundary);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  ::common::Point3D position_1(0.0, 15.0, 0.0);
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_position(position_1);
  perception_obstacle.set_theta(M_PI / 2);
  ::common::Trajectory trajectory_1 = generate_trajectory(0.0, 15.0, 0.0, 22.5);
  Obstacle obstacle_1("1", perception_obstacle, trajectory_1);
  obstacle_1.SetRelativeRegion2LocalRoute(RelativeRegionType::FRONT);
  if (!local_route.GetSLBoundary(obstacle_1.PerceptionBoundingBox(),
                                 sl_boundary)) {
    AERROR << " Failed to get sl boundary for obstacle:" << obstacle_0.Id();
  }
  obstacle_1.SetPerceptionSlBoundary(sl_boundary);
  obstacles.emplace_back(&obstacle_1);
  FrenetPoint frenet_pt;
  frenet_pt.s[kIdxS] = 0.0;
  local_route.SetInitFrenetState(frenet_pt);
  EXPECT_EQ(frenet_trajectory_map_builder_
                ->GetObstacles(local_route, obstacles, CorridorInfo::Type::LANE_KEEP)
                .size(),
            2);
  EXPECT_EQ(frenet_trajectory_map_builder_
                ->GetObstacles(local_route, obstacles, CorridorInfo::Type::LANE_KEEP)
                .at(0)
                ->Id(),
            "0");
}

TEST_F(FrenetTrajectoryMapBuilderTest, TestConvertFrenetTrajToCartesianTraj) {
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  std::vector<::common::FrenetPoint> trajectory;
  for (int i = 0; i < 25; i++) {
    FrenetPoint frenet_point;
    frenet_point.t = 0.5 * i;
    frenet_point.s[0] = -5.0 + 1.0 * i;
    frenet_point.s[1] = 1.0;
    frenet_point.s[2] = 0.0;
    frenet_point.l[0] = 0.0;
    trajectory.emplace_back(frenet_point);
  }
  std::vector<::common::CartesianPoint> cartesian_traj =
      frenet_trajectory_map_builder_->ConvertFrenetTrajToCartesianTraj(
          local_route, trajectory);
  EXPECT_EQ(cartesian_traj.size(), 25);

  EXPECT_NEAR(cartesian_traj.front().t, 0.0, 0.01);
  EXPECT_NEAR(cartesian_traj.front().x, 0.0, 0.01);
  EXPECT_NEAR(cartesian_traj.front().v, 1.0, 0.01);
  EXPECT_NEAR(cartesian_traj.front().a, 0.0, 0.01);
  EXPECT_NEAR(cartesian_traj.front().y, 0.0, 0.01);
  EXPECT_NEAR(cartesian_traj.front().theta, M_PI / 2, 0.01);

  EXPECT_NEAR(cartesian_traj.back().t, 12.0, 0.01);
  EXPECT_NEAR(cartesian_traj.back().x, 0.0, 0.01);
  EXPECT_NEAR(cartesian_traj.back().v, 1.0, 0.01);
  EXPECT_NEAR(cartesian_traj.back().a, 0.0, 0.01);
  EXPECT_NEAR(cartesian_traj.back().y, 19.0, 0.01);
  EXPECT_NEAR(cartesian_traj.back().theta, M_PI / 2, 0.01);
}

}  // namespace planning
}  // namespace zark
