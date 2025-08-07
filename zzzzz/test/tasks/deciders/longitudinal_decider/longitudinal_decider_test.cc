/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_decider_test.cc
 **/

#include "apps/planning/src/tasks/deciders/longitudinal_decider/longitudinal_decider.h"

#include <string>

#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/tasks/deciders/lateral_decider/lateral_decider.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

using namespace zark::planning;
using common::VehicleConfigHelper;

class LongitudinalDeciderTest : public ::testing::Test {
 public:
  void SetUp() override {
    // using ego width fully block judgement
    PlanningGflags::use_corridor_boundary_blocking_judgement = false;
    // vehicle
    common::VehicleParam vehicle_param;
    vehicle_param.set_width(2.11);
    vehicle_param.set_length(4.933);
    vehicle_param.set_back_edge_to_center(1.043);
    vehicle_param.set_front_edge_to_center(3.89);
    vehicle_param.set_left_edge_to_center(1.055);
    vehicle_param.set_right_edge_to_center(1.055);
    vehicle_param.set_rear_axle_to_cg(1.3066);
    common::VehicleConfig vehicle_config;
    vehicle_config.set_vehicle_param(vehicle_param);
    VehicleConfigHelper::Init(vehicle_config);
    // task_config
    TaskConfig::task_config_union task_config;
    LongitudinalDeciderConfig longitudinal_decider_config;
    LongitudinalDeciderConfig::STTopologyConfig st_topology_config;
    st_topology_config.alpha = 0.33;
    LongitudinalDeciderConfig::STProposalConfig st_proposal_config;
    st_proposal_config.max_num_proposals = 2;
    st_proposal_config.a_min_stiff = -6.0;
    st_proposal_config.w_front = 1.0;
    st_proposal_config.w_rear = 1.0;
    st_proposal_config.cost_max = 20.0;
    st_proposal_config.t_start_blocking_max = 4.0;
    LongitudinalDeciderConfig::SpeedLimitConfig speed_limit_builder;
    speed_limit_builder.a_lat_max_table.points_x.push_back(15.0);
    speed_limit_builder.a_lat_max_table.points_x.push_back(1.0);
    speed_limit_builder.a_lat_max_table.points_x.push_back(20.0);
    speed_limit_builder.a_lat_max_table.points_y.push_back(1.0);
    speed_limit_builder.a_lat_max_table.points_y.push_back(25.0);
    speed_limit_builder.a_lat_max_table.points_y.push_back(1.0);
    speed_limit_builder.v_max = 10.0;
    speed_limit_builder.t_sliding_window = 3.0;
    LongitudinalDeciderConfig::STGraphConfig st_graph_config;
    st_graph_config.lateral_buffer = 0.5;
    st_graph_config.safety_l_buffer = 0.1;
    st_graph_config.use_obs_sl_for_st_graph = false;
    st_graph_config.t_plan = 8.0;
    st_graph_config.fully_blocking_buffer = 0.5;
    st_graph_config.st_boundary_counter_min = -1;
    st_graph_config.delta_t_oncoming_obs = 1.0;

    longitudinal_decider_config.st_proposal_config = st_proposal_config;
    longitudinal_decider_config.st_topology_config = st_topology_config;
    longitudinal_decider_config.speed_limit_config = speed_limit_builder;
    longitudinal_decider_config.st_graph_config = st_graph_config;
    task_config.longitudinal_decider_config = longitudinal_decider_config;

    LateralDeciderConfig lateral_decider_config;
    LateralDeciderConfig::FrenetTrajectorySampler frenet_trajectory_sampler;
    frenet_trajectory_sampler.cost_lon =
        LateralDeciderConfig::FrenetTrajectorySampler::Cost{1.0, 1.0, 1.0};
    frenet_trajectory_sampler.cost_lat =
        LateralDeciderConfig::FrenetTrajectorySampler::Cost{1.0, 1.0, 1.0};
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
    corridor.extra_half_width = 1.05;
    lateral_decider_config.corridor = corridor;
    lateral_decider_config.frenet_trajectory_sampler_lc =
        frenet_trajectory_sampler;
    task_config.lateral_decider_config = lateral_decider_config;

    TaskConfig config;
    config.set_task_config(task_config);
    std::shared_ptr<DependencyInjector> injector =
        std::make_shared<DependencyInjector>();
    injector->vehicle_state()->set_linear_velocity(5.0);
    longitudinal_decider_ =
        std::make_unique<LongitudinalDecider>(config, injector);
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
      trajectory_point.set_relative_time(0.0 + 0.5 * i);
      trajectory.mutable_trajectory_point()->emplace_back(trajectory_point);
    }
    return trajectory;
  }

 protected:
  std::unique_ptr<LongitudinalDecider> longitudinal_decider_;
  std::unique_ptr<LateralDecider> lateral_decider_;
  std::unique_ptr<Frame> frame_;
};

TEST_F(LongitudinalDeciderTest, TestRecordSTTopology) {
  LocalRouteConfig local_route_config;
  local_route_config.local_route_speed_limit =
      10.0;  // Set expected value for the speed limit
  local_route_config.min_local_route_speed_limit = 1.0;
  local_route_config.default_local_route_width = 6.0;
  local_route_config.obs_filter_front_end_s = 200;
  local_route_config.obs_filter_rear_start_s = -50;

  // set front obstacle
  perception::PerceptionObstacle perception_obstacle;
  ::common::Point3D position(0.0, 7.5, 0.0);
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_position(position);
  perception_obstacle.set_theta(M_PI / 2);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 7.5, 0.0, 15.0);
  Obstacle obstacle_0("5", perception_obstacle, trajectory_0);
  obstacle_0.SetRelativeRegion2LocalRoute(RelativeRegionType::FRONT);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);

  LocalView local_view;
  local_view.fct_output = std::make_shared<zark::ads_fct::FCT_Outputs>();
  local_view.fct_output->mutable_fct_out_bus_accinfosts()
      ->set_fct_out_v_accactsetspd_sg(2.0 * 3.6);

  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, local_route_config);

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
  GaussianInfo gaussian_info;
  frame_ = std::make_unique<Frame>(1, local_view, planning_start_point,
                                   vehicle_state);
  frame_->AddObstacle(obstacle_0);
  local_route.AddObstaclesSLBoundary(obstacles);

  std::list<LocalRoute> &mutable_local_route_info =
      frame_->MutableLocalRoutes();
  mutable_local_route_info.push_back(local_route);
  frame_->set_planning_start_point(planning_start_point);
  frame_->SetTargetLocalRoute(&local_route);
  frame_->SetIsReplan(true);
  ::common::Status status = lateral_decider_->Execute(frame_.get());
  longitudinal_decider_->Execute(frame_.get());
  EXPECT_EQ(frame_->GetCorridorInfos()[0].GetSTTopology().layers.size(), 1);
  EXPECT_EQ(frame_->GetCorridorInfos()[0].GetSTTopology().layers[0].size(), 1);
  EXPECT_EQ(frame_->GetCorridorInfos()[0].GetSTTopology().layers[0][0], "5");
  EXPECT_EQ(frame_->GetProposals().size(), 1);
  EXPECT_EQ(frame_->GetProposals()[0].GetSTProposal().size(), 1);
  EXPECT_EQ(std::get<0>(frame_->GetProposals()[0].GetSTProposal()[0])->id(),
            "5");
}

TEST_F(LongitudinalDeciderTest, TestInitSTBoundaryCounterMaps) {
  longitudinal_decider_->InitSTBoundaryCounterMaps();
  EXPECT_EQ(longitudinal_decider_->st_boundary_counter_maps_.size(), 4);
  EXPECT_EQ(longitudinal_decider_
                ->st_boundary_counter_maps_[CorridorInfo::Type::LANE_KEEP]
                .size(),
            0);
}

TEST_F(LongitudinalDeciderTest, TestUpdateSTBoundaryCounterMaps) {
  LocalRouteConfig local_route_config;
  local_route_config.local_route_speed_limit =
      10.0;  // Set expected value for the speed limit
  local_route_config.min_local_route_speed_limit = 1.0;
  local_route_config.default_local_route_width = 6.0;
  local_route_config.obs_filter_front_end_s = 200;
  local_route_config.obs_filter_rear_start_s = -50;

  // set front obstacle
  perception::PerceptionObstacle perception_obstacle;
  ::common::Point3D position(0.0, 7.5, 0.0);
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_position(position);
  perception_obstacle.set_theta(M_PI / 2);
  ::common::Trajectory trajectory_0 = generate_trajectory(0.0, 7.5, 0.0, 15.0);
  Obstacle obstacle_0("5", perception_obstacle, trajectory_0);
  obstacle_0.SetRelativeRegion2LocalRoute(RelativeRegionType::FRONT);
  std::vector<const Obstacle *> obstacles;
  obstacles.emplace_back(&obstacle_0);

  LocalView local_view;
  local_view.fct_output = std::make_shared<zark::ads_fct::FCT_Outputs>();
  local_view.fct_output->mutable_fct_out_bus_accinfosts()
      ->set_fct_out_v_accactsetspd_sg(2.0 * 3.6);

  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 36; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, local_route_config);

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
  GaussianInfo gaussian_info;
  frame_ = std::make_unique<Frame>(1, local_view, planning_start_point,
                                   vehicle_state);
  frame_->AddObstacle(obstacle_0);
  local_route.AddObstaclesSLBoundary(obstacles);

  std::list<LocalRoute> &mutable_local_route_info =
      frame_->MutableLocalRoutes();
  mutable_local_route_info.push_back(local_route);
  frame_->set_planning_start_point(planning_start_point);
  frame_->SetTargetLocalRoute(&local_route);
  frame_->SetIsReplan(true);
  ::common::Status status = lateral_decider_->Execute(frame_.get());
  longitudinal_decider_->Execute(frame_.get());
  longitudinal_decider_->UpdateSTBoundaryCounterMaps(
      frame_->GetCorridorInfos());
  EXPECT_EQ(longitudinal_decider_->st_boundary_counter_maps_.size(), 4);
  EXPECT_EQ(longitudinal_decider_
                ->st_boundary_counter_maps_[CorridorInfo::Type::LANE_KEEP]
                .size(),
            1);
  EXPECT_EQ(longitudinal_decider_
                ->st_boundary_counter_maps_[CorridorInfo::Type::LANE_CHANGE]
                .size(),
            0);
}

}  // namespace planning
}  // namespace zark
