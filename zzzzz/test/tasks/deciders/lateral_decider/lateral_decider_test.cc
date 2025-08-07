/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_decider_test.cc
 **/

#include "apps/planning/src/tasks/deciders/lateral_decider/lateral_decider.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

using namespace zark::planning;
using common::VehicleConfigHelper;

class LateralDeciderTest : public ::testing::Test {
 public:
  LateralDeciderTest() {
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
    corridor.a_lat_max_table.points_x = std::vector<double>{0.0, 10.0};
    corridor.a_lat_max_table.points_y = std::vector<double>{1.0, 1.0};
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
    std::shared_ptr<DependencyInjector> injector =
        std::make_shared<DependencyInjector>();
    injector->vehicle_state()->set_linear_velocity(5.0);
    lateral_decider_ = std::make_unique<LateralDecider>(config, injector);
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

  void GenerateLocalRoutePoints(
      std::vector<LocalRoutePoint> &local_route_points) {
    for (double i = 0.0; i < 300.0; i = i + 0.5) {
      double x = i;
      double y = 0.0;
      double heading = 0.0;
      double kappa = 0.0;
      double dkappa = 0.0;
      local_route_points.emplace_back(LocalRoutePoint(
          hdmap::MapPathPoint(math::Vec2d(x, y), heading), kappa, dkappa));
    }
  }

 protected:
  std::unique_ptr<LateralDecider> lateral_decider_;
};

TEST_F(LateralDeciderTest, TestComputeCorridorStartPoint) {
  std::shared_ptr<DependencyInjector> injector =
      std::make_shared<DependencyInjector>();
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    CorridorPoint corridor_point;
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-10 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = M_PI / 2;
    corridor_point.kappa = 0.0;
    corridor_point.v = 1.0;
    corridor_point.a = 0.0;
    corridor_points.emplace_back(corridor_point);
  }
  LocalRoute local_route{};
  Mission misssion{};
  CorridorInfo::Type corridor_type;
  const int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle *> obs_map;
  Corridor corridor(corridor_points);
  CorridorInfo corr_info(local_route, misssion, corridor_type, corridor,
                         idx_start_point, obs_map);
  Proposal proposal;
  proposal.SetCorridorInfo(&corr_info);
  auto frame_prev = std::make_unique<Frame>(1);
  frame_prev->MutableProposals()->emplace_back(proposal);
  injector->frame_history()->Add(1, std::move(frame_prev));
  zark::planning::Frame frame(2);
  ::common::TrajectoryPoint planning_start_point;
  ::common::PathPoint path_point;
  path_point.set_x(0);
  path_point.set_y(0.5);
  path_point.set_theta(M_PI / 2);
  path_point.set_kappa(0.0);
  planning_start_point.set_path_point(path_point);
  planning_start_point.set_v(2.0);
  planning_start_point.set_a(0.0);
  frame.set_planning_start_point(planning_start_point);
  lateral_decider_->frame_ = &frame;
  CorridorPoint corroidor_start_point =
      lateral_decider_->ComputeCorridorStartPoint(true);
  EXPECT_NEAR(corroidor_start_point.xy_ref.x(), 0.0, 0.01);
  EXPECT_NEAR(corroidor_start_point.xy_ref.y(), 0.5, 0.01);
  EXPECT_NEAR(corroidor_start_point.v, 2.0, 0.01);
  EXPECT_NEAR(corroidor_start_point.a, 0.0, 0.01);
  EXPECT_NEAR(corroidor_start_point.theta, M_PI / 2, 0.01);
  EXPECT_NEAR(corroidor_start_point.kappa, 0.0, 0.01);

  lateral_decider_->injector_ = injector;
  corroidor_start_point = lateral_decider_->ComputeCorridorStartPoint(false);
  EXPECT_NEAR(corroidor_start_point.xy_ref.x(), 0.0, 0.01);
  EXPECT_NEAR(corroidor_start_point.xy_ref.y(), 0.5, 0.01);
  EXPECT_NEAR(corroidor_start_point.v, 2.0, 0.01);
  EXPECT_NEAR(corroidor_start_point.a, 0.0, 0.01);
  EXPECT_NEAR(corroidor_start_point.theta, M_PI / 2, 0.01);
  EXPECT_NEAR(corroidor_start_point.kappa, 0.0, 0.01);
}

TEST_F(LateralDeciderTest, TestCreateCorridorInfo) {
  // GTEST_SKIP();
  // set front obstacle
  perception::PerceptionObstacle perception_obstacle;
  ::common::Point3D position(0.0, 7.5, 0.0);
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_position(position);
  perception_obstacle.set_theta(M_PI / 2);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 3.5, 0.0, 15.0);
  Obstacle obstacle_0("0", perception_obstacle, trajectory_0);
  obstacle_0.SetRelativeRegion2LocalRoute(RelativeRegionType::FRONT);
  // set rear obstacle
  position.set_y(-5);
  perception_obstacle.set_position(position);
  ::common::Trajectory trajectory_1 = generate_trajectory(0.0, -5.0, 0.0, 10.0);
  Obstacle obstacle_1("1", perception_obstacle, trajectory_1);
  obstacle_1.SetRelativeRegion2LocalRoute(RelativeRegionType::FRONT);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  obstacles.emplace_back(&obstacle_1);
  zark::planning::Frame frame(1);
  frame.AddObstacle(obstacle_0);
  frame.AddObstacle(obstacle_1);
  LocalView local_view;
  local_view.fct_output = std::make_shared<zark::ads_fct::FCT_Outputs>();
  local_view.fct_output->mutable_fct_out_bus_accinfosts()
      ->set_fct_out_v_accactsetspd_sg(1.5 * 3.6);
  frame.set_local_view(local_view);
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 30; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(-10.0 + 1.0 * i);
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

  // planning_start_point
  ::common::TrajectoryPoint planning_start_point;
  ::common::PathPoint path_point;
  path_point.set_x(0);
  path_point.set_y(1.0);
  path_point.set_theta(M_PI / 2);
  path_point.set_kappa(0.0);
  planning_start_point.set_path_point(path_point);
  planning_start_point.set_v(2.0);
  planning_start_point.set_a(0.0);
  common::VehicleState vehicle_state;
  local_route.Init(vehicle_state, planning_start_point);
  local_route.AddObstaclesSLBoundary(obstacles);
  local_route.SetInitFrenetState(init_frenet_point);
  local_route.SetDestinationS(15.0);
  std::list<LocalRoute> &mutable_local_route_info = frame.MutableLocalRoutes();
  mutable_local_route_info.push_back(local_route);
  frame.set_planning_start_point(planning_start_point);
  frame.SetTargetLocalRoute(&local_route);
  Mission mission;
  mission.lc_request = Mission::LaneChangeRequest::LC_LEFT;
  mission.is_lc_ready = true;
  frame.SetMission(mission);
  frame.SetIsReplan(true);
  ::common::Status status = lateral_decider_->Execute(&frame);
  EXPECT_EQ(status.ToString(), "OK");
  EXPECT_EQ(frame.GetCorridorInfos().size(), 2);
  EXPECT_EQ(frame.GetMission().lc_request, Mission::LaneChangeRequest::LC_LEFT);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetType(),
            CorridorInfo::Type::LANE_KEEP);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetCorridor().size(), 84);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetObstacleMap().Items().size(), 3);

  EXPECT_EQ(frame.GetCorridorInfos()[0].GetType(),
            CorridorInfo::Type::LANE_CHANGE);
  EXPECT_EQ(frame.GetCorridorInfos()[0].GetObstacleMap().Items().size(), 3);

  path_point.set_x(0.5);
  planning_start_point.set_path_point(path_point);
  frame.set_planning_start_point(planning_start_point);
  frame.MutableCorridorInfos()->clear();
  status = lateral_decider_->Execute(&frame);
  EXPECT_EQ(status.ToString(), "OK");
  EXPECT_EQ(frame.GetCorridorInfos().size(), 2);
  EXPECT_EQ(frame.GetMission().lc_request, Mission::LaneChangeRequest::LC_LEFT);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetType(),
            CorridorInfo::Type::LANE_KEEP);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetCorridor().size(), 83);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetObstacleMap().Items().size(), 3);

  EXPECT_EQ(frame.GetCorridorInfos()[0].GetType(),
            CorridorInfo::Type::LANE_CHANGE);
  EXPECT_EQ(frame.GetCorridorInfos()[0].GetObstacleMap().Items().size(), 3);
  EXPECT_NEAR(frame.GetCorridorInfos()[1].GetInitFrenetPoint().s[0], 0.0, 0.01);
  EXPECT_NEAR(frame.GetCorridorInfos()[1].GetInitFrenetPoint().s[1], 2.0, 0.01);
  EXPECT_NEAR(frame.GetCorridorInfos()[1].GetInitFrenetPoint().l[0], 0.0, 0.01);
  EXPECT_NEAR(frame.GetCorridorInfos()[1].GetInitFrenetPoint().l[1], 0.0, 0.01);

  std::shared_ptr<DependencyInjector> injector =
      std::make_shared<DependencyInjector>();
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    CorridorPoint corridor_point;
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-10 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = M_PI / 2;
    corridor_point.kappa = 0.0;
    corridor_point.v = 1.0;
    corridor_point.a = 0.0;
    corridor_points.emplace_back(corridor_point);
  }
  CorridorInfo::Type corridor_type;
  const int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle *> obs_map;
  Corridor corridor(corridor_points);
  CorridorInfo corr_info(local_route, mission, corridor_type, corridor,
                         idx_start_point, obs_map);
  Proposal proposal;
  proposal.SetCorridorInfo(&corr_info);
  auto frame_prev = std::make_unique<Frame>(2);
  frame_prev->MutableProposals()->emplace_back(proposal);
  injector->frame_history()->Add(1, std::move(frame_prev));
  lateral_decider_->injector_ = injector;
  frame.MutableCorridorInfos()->clear();
  frame.SetIsReplan(false);
  status = lateral_decider_->Execute(&frame);
  EXPECT_EQ(status.ToString(), "OK");
  EXPECT_EQ(frame.GetCorridorInfos().size(), 2);
  EXPECT_EQ(frame.GetMission().lc_request, Mission::LaneChangeRequest::LC_LEFT);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetType(),
            CorridorInfo::Type::LANE_KEEP);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetCorridor().size(), 84);
  EXPECT_EQ(frame.GetCorridorInfos()[1].GetObstacleMap().Items().size(), 3);

  EXPECT_EQ(frame.GetCorridorInfos()[0].GetType(),
            CorridorInfo::Type::LANE_CHANGE);
  EXPECT_EQ(frame.GetCorridorInfos()[0].GetObstacleMap().Items().size(), 3);
  EXPECT_NEAR(frame.GetCorridorInfos()[1].GetInitFrenetPoint().s[0], 0.0, 0.01);
  EXPECT_NEAR(frame.GetCorridorInfos()[1].GetInitFrenetPoint().s[1], 2.0, 0.01);
  EXPECT_NEAR(frame.GetCorridorInfos()[1].GetInitFrenetPoint().l[0], -0.5,
              0.01);
  EXPECT_NEAR(frame.GetCorridorInfos()[1].GetInitFrenetPoint().l[1], 0.0, 0.01);
}

TEST_F(LateralDeciderTest, TestComputeCorridor) {
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle_0("0", perception_obstacle, trajectory_0);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 17; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // planning_start_point
  ::common::TrajectoryPoint planning_start_point;
  ::common::PathPoint path_point;
  path_point.set_x(0);
  path_point.set_y(2.0);
  path_point.set_theta(M_PI / 2);
  planning_start_point.set_path_point(path_point);
  common::VehicleState vehicle_state;
  local_route.Init(vehicle_state, planning_start_point);
  std::array<double, FrenetTrajectorySampler::kNumStates> s_init = {2.0, 0.5,
                                                                    0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_init = {0.0, 0.0,
                                                                    0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> s_terminal = {
      std::numeric_limits<double>::quiet_NaN(), 3.0, 0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_terminal = {
      0.5, 0.0, 0.0};
  std::vector<::common::FrenetPoint> frenet_sampling_traj =
      lateral_decider_->frenet_trajectory_map_builder_
          ->GetFrenetTrajectorySamplerLC()
          ->ComputeOptimalTrajectory(s_init, l_init, s_terminal, l_terminal);
  std::vector<::common::CartesianPoint> cartesian_traj =
      lateral_decider_->frenet_trajectory_map_builder_
          ->ConvertFrenetTrajToCartesianTraj(local_route, frenet_sampling_traj);
  int idx_start_point;
  FrenetPoint init_frenet_point;
  init_frenet_point.s = s_init;
  init_frenet_point.l = l_init;
  const double v_target = 3.0;
  const int i_start_frenet_traj = 0;
  zark::planning::Frame frame(1);
  LocalView local_view;
  local_view.fct_output = std::make_shared<zark::ads_fct::FCT_Outputs>();
  local_view.fct_output->mutable_fct_out_bus_accinfosts()
      ->set_fct_out_v_accactsetspd_sg(16.0 * 3.6);
  frame.set_local_view(local_view);
  lateral_decider_->frame_ = &frame;
  Corridor corridor = lateral_decider_->ComputeCorridor(
      init_frenet_point, local_route, cartesian_traj, v_target,
      i_start_frenet_traj, idx_start_point);
  EXPECT_NEAR(corridor.front().l, 0, 0.01);
  EXPECT_NEAR(corridor.front().s, -0.7, 0.01);
  EXPECT_NEAR(corridor.back().l, 0, 0.1);
  EXPECT_NEAR(corridor.back().s, 14.02, 0.1);
}

TEST_F(LateralDeciderTest, TestComputeEndCorridorSegment) {
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle_0("0", perception_obstacle, trajectory_0);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 17; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // planning_start_point
  ::common::TrajectoryPoint planning_start_point;
  ::common::PathPoint path_point;
  path_point.set_x(0);
  path_point.set_y(2.0);
  path_point.set_theta(M_PI / 2);
  planning_start_point.set_path_point(path_point);
  common::VehicleState vehicle_state;
  local_route.Init(vehicle_state, planning_start_point);
  double s_start_local_route = 11.3;
  double s_start_corridor = 10.22;
  int num_points_front = 20;
  std::array<double, FrenetTrajectorySampler::kNumStates> s_init = {2.0, 0.5,
                                                                    0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_init = {0.0, 0.0,
                                                                    0.0};
  FrenetPoint init_frenet_point;
  init_frenet_point.s = s_init;
  init_frenet_point.l = l_init;
  std::vector<CorridorPoint> corridor_points;
  lateral_decider_->ComputeEndCorridorSegment(
      local_route, s_start_local_route, s_start_corridor, 0.5, 3.0, 0.1,
      num_points_front, corridor_points);
  EXPECT_EQ(corridor_points.size(), 16);
  EXPECT_EQ(corridor_points.back().theta, M_PI / 2);
  EXPECT_EQ(corridor_points.back().kappa, 0);
  EXPECT_EQ(corridor_points.front().s, s_start_corridor);
  EXPECT_EQ(corridor_points.front().s_ref, s_start_local_route);
}

TEST_F(LateralDeciderTest, TestComputeMiddleCorridorSegment) {
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle_0("0", perception_obstacle, trajectory_0);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 17; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // planning_start_point
  ::common::TrajectoryPoint planning_start_point;
  ::common::PathPoint path_point;
  path_point.set_x(0);
  path_point.set_y(2.0);
  path_point.set_theta(M_PI / 2);
  planning_start_point.set_path_point(path_point);
  common::VehicleState vehicle_state;
  local_route.Init(vehicle_state, planning_start_point);
  std::array<double, FrenetTrajectorySampler::kNumStates> s_init = {2.0, 0.5,
                                                                    0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_init = {0.0, 0.0,
                                                                    0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> s_terminal = {
      std::numeric_limits<double>::quiet_NaN(), 3.0, 0.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_terminal = {
      0.5, 0.0, 0.0};
  std::vector<::common::FrenetPoint> frenet_sampling_traj =
      lateral_decider_->frenet_trajectory_map_builder_
          ->GetFrenetTrajectorySamplerLC()
          ->ComputeOptimalTrajectory(s_init, l_init, s_terminal, l_terminal);
  std::vector<::common::CartesianPoint> cartesian_traj =
      lateral_decider_->frenet_trajectory_map_builder_
          ->ConvertFrenetTrajToCartesianTraj(local_route, frenet_sampling_traj);
  FrenetPoint init_frenet_point;
  init_frenet_point.s = s_init;
  init_frenet_point.l = l_init;
  local_route.SetInitFrenetState(init_frenet_point);
  std::vector<CorridorPoint> corridor_points;
  int i_start_frenet_traj = 0;
  lateral_decider_->ComputeMiddleCorridorSegment(
      init_frenet_point, local_route, cartesian_traj, i_start_frenet_traj,
      corridor_points);
  EXPECT_EQ(corridor_points.back().xy_ref.y(), cartesian_traj.back().y);

  s_init = {3.0, 0.5, 0.0};
  l_init = {0.0, 0.0, 0.0};
  s_terminal = {std::numeric_limits<double>::quiet_NaN(), 3.0, 0.0};
  l_terminal = {0.5, 0.0, 0.0};
  frenet_sampling_traj =
      lateral_decider_->frenet_trajectory_map_builder_
          ->GetFrenetTrajectorySamplerLC()
          ->ComputeOptimalTrajectory(s_init, l_init, s_terminal, l_terminal);
  cartesian_traj =
      lateral_decider_->frenet_trajectory_map_builder_
          ->ConvertFrenetTrajToCartesianTraj(local_route, frenet_sampling_traj);
  corridor_points.clear();
  i_start_frenet_traj = 20;
  lateral_decider_->ComputeMiddleCorridorSegment(
      init_frenet_point, local_route, cartesian_traj, i_start_frenet_traj,
      corridor_points);
  EXPECT_EQ(corridor_points.back().l_ref, frenet_sampling_traj.back().l[0]);
}

TEST_F(LateralDeciderTest, TestCalcLLeftAndRightBasedOnCurb) {
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 17; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  const double s_ref = 1.0;
  double l_ref = 0.5;
  double l_left;
  double l_right;
  lateral_decider_->CalcLLeftAndRightBasedOnCurb(local_route, s_ref, l_ref,
                                                 l_left, l_right);
  EXPECT_NEAR(l_left, 1.855, 0.01);
  EXPECT_NEAR(l_right, -1.855, 0.01);

  lateral_decider_->config_.corridor.extra_half_width = 0.8;
  lateral_decider_->CalcLLeftAndRightBasedOnCurb(local_route, s_ref, l_ref,
                                                 l_left, l_right);
  EXPECT_NEAR(l_left, 1.855, 0.01);
  EXPECT_NEAR(l_right, -1.855, 0.01);

  l_ref = -0.5;
  lateral_decider_->CalcLLeftAndRightBasedOnCurb(local_route, s_ref, l_ref,
                                                 l_left, l_right);
  EXPECT_NEAR(l_left, 1.855, 0.01);
  EXPECT_NEAR(l_right, -1.855, 0.01);
}

TEST_F(LateralDeciderTest, TestFilterObstacles) {
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
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // planning_start_point
  ::common::TrajectoryPoint planning_start_point;
  ::common::PathPoint path_point;
  path_point.set_x(0);
  path_point.set_y(1.0);
  path_point.set_theta(M_PI / 2);
  planning_start_point.set_path_point(path_point);
  planning_start_point.set_v(5.0);
  planning_start_point.set_a(1.0);
  common::VehicleState vehicle_state;
  local_route.Init(vehicle_state, planning_start_point);

  std::array<double, FrenetTrajectorySampler::kNumStates> s_init = {1.0, 0.5,
                                                                    1.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_init = {0.0, 0.0,
                                                                    0.0};
  FrenetPoint init_frenet_point;
  init_frenet_point.s = s_init;
  init_frenet_point.l = l_init;
  Mission::LaneChangeRequest lane_change_request =
      Mission::LaneChangeRequest::LANE_KEEP;
  CorridorInfo::Type type = CorridorInfo::Type::LANE_KEEP;
  zark::planning::Frame frame(1);
  lateral_decider_->frame_ = &frame;
  IndexedPtrList<std::string, const Obstacle *> filtered_obstacles =
      lateral_decider_->FilterObstacles(init_frenet_point, obstacles,
                                        lane_change_request, type);
  EXPECT_EQ(filtered_obstacles.Dict().size(), 1);
  EXPECT_EQ(filtered_obstacles.Dict().begin()->first, "0");
}

TEST_F(LateralDeciderTest, TestLCFilterObstacles) {
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  ::common::Point3D position(0.0, 7.5, 0.0);
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_position(position);
  perception_obstacle.set_theta(M_PI / 2);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 7.5, 0.0, 15.0);
  Obstacle obstacle_0("0", perception_obstacle, trajectory_0);
  obstacle_0.SetRelativeRegion2LocalRoute(RelativeRegionType::REAR);
  obstacle_0.SetDriveStatus(Obstacle::DriveStatus::STATUS_LANE_LEFT_CHANGE);
  Obstacle obstacle_1("1", perception_obstacle, trajectory_0);
  obstacle_1.SetRelativeRegion2LocalRoute(RelativeRegionType::LEFT_REAR);
  Obstacle obstacle_2("2", perception_obstacle, trajectory_0);
  obstacle_2.SetRelativeRegion2LocalRoute(RelativeRegionType::RIGHT_REAR);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  obstacles.emplace_back(&obstacle_1);
  obstacles.emplace_back(&obstacle_2);
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // planning_start_point
  ::common::TrajectoryPoint planning_start_point;
  ::common::PathPoint path_point;
  path_point.set_x(0);
  path_point.set_y(1.0);
  path_point.set_theta(M_PI / 2);
  planning_start_point.set_path_point(path_point);
  planning_start_point.set_v(5.0);
  planning_start_point.set_a(1.0);
  common::VehicleState vehicle_state;
  local_route.Init(vehicle_state, planning_start_point);

  std::array<double, FrenetTrajectorySampler::kNumStates> s_init = {1.0, 0.5,
                                                                    1.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_init = {0.0, 0.0,
                                                                    0.0};
  FrenetPoint init_frenet_point;
  init_frenet_point.s = s_init;
  init_frenet_point.l = l_init;
  Mission::LaneChangeRequest lane_change_request =
      Mission::LaneChangeRequest::LC_LEFT;
  CorridorInfo::Type type = CorridorInfo::Type::LANE_CHANGE;
  zark::planning::Frame frame(1);
  lateral_decider_->frame_ = &frame;
  const auto &filtered_obstacles = lateral_decider_->FilterObstacles(
      init_frenet_point, obstacles, lane_change_request, type);
  EXPECT_EQ(filtered_obstacles.Items().size(), 2);
  EXPECT_EQ(filtered_obstacles.Items()[0]->Id(), "0");
  EXPECT_EQ(filtered_obstacles.Items()[1]->Id(), "1");
}

TEST_F(LateralDeciderTest, TestEmptyLCFilterObstacles) {
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  ::common::Point3D position(0.0, 7.5, 0.0);
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_position(position);
  perception_obstacle.set_theta(M_PI / 2);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 7.5, 0.0, 15.0);
  Obstacle obstacle_0("0", perception_obstacle, trajectory_0);
  obstacle_0.SetRelativeRegion2LocalRoute(RelativeRegionType::REAR);
  obstacle_0.SetDriveStatus(Obstacle::DriveStatus::STATUS_LANE_RIGHT_CHANGE);
  Obstacle obstacle_1("1", perception_obstacle, trajectory_0);
  obstacle_1.SetRelativeRegion2LocalRoute(RelativeRegionType::LEFT_REAR);
  Obstacle obstacle_2("2", perception_obstacle, trajectory_0);
  obstacle_2.SetRelativeRegion2LocalRoute(RelativeRegionType::RIGHT_REAR);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);
  obstacles.emplace_back(&obstacle_1);
  obstacles.emplace_back(&obstacle_2);
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  // planning_start_point
  ::common::TrajectoryPoint planning_start_point;
  ::common::PathPoint path_point;
  path_point.set_x(0);
  path_point.set_y(1.0);
  path_point.set_theta(M_PI / 2);
  planning_start_point.set_path_point(path_point);
  planning_start_point.set_v(5.0);
  planning_start_point.set_a(1.0);
  common::VehicleState vehicle_state;
  local_route.Init(vehicle_state, planning_start_point);

  std::array<double, FrenetTrajectorySampler::kNumStates> s_init = {1.0, 0.5,
                                                                    1.0};
  std::array<double, FrenetTrajectorySampler::kNumStates> l_init = {0.0, 0.0,
                                                                    0.0};
  FrenetPoint init_frenet_point;
  init_frenet_point.s = s_init;
  init_frenet_point.l = l_init;
  Mission::LaneChangeRequest lane_change_request =
      Mission::LaneChangeRequest::LC_RIGHT;
  CorridorInfo::Type type = CorridorInfo::Type::LANE_KEEP;
  zark::planning::Frame frame(1);
  lateral_decider_->frame_ = &frame;
  const auto &filtered_obstacles = lateral_decider_->FilterObstacles(
      init_frenet_point, obstacles, lane_change_request, type);
  EXPECT_EQ(filtered_obstacles.Items().size(), 0);
}

TEST_F(LateralDeciderTest, TestAssignCorridorPointType) {
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 17; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  CorridorPoint corridor_point;
  corridor_point.s_ref = 0.0;
  corridor_point.l_left = 2.0;
  corridor_point.l_right = -2.0;
  lateral_decider_->AssignCorridorPointType(local_route, corridor_point);

  EXPECT_EQ(corridor_point.type_left, CorridorPoint::Type::UNKNOWN);
  EXPECT_EQ(corridor_point.type_right, CorridorPoint::Type::UNKNOWN);

  corridor_point.l_left = 1.0;
  corridor_point.l_right = -1.0;
  lateral_decider_->AssignCorridorPointType(local_route, corridor_point);

  EXPECT_EQ(corridor_point.type_left, CorridorPoint::Type::CURB);
  EXPECT_EQ(corridor_point.type_right, CorridorPoint::Type::CURB);
}

}  // namespace planning
}  // namespace zark
