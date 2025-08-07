/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_optimizer_test.cc
 **/

#include "apps/planning/src/tasks/optimizers/longitudinal_optimizer/longitudinal_optimizer.h"

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"
#include "point_factory.h"

using ::util::PointFactory;

namespace zark {
namespace planning {

::common::Trajectory generate_trajectory(double x_start, double y_start,
                                         double x_end, double y_end) {
  ::common::Trajectory trajectory;
  const int num_points = 5;
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    double x = x_start + t * (x_end - x_start);
    double y = y_start + t * (y_end - y_start);
    ::common::PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    double v = 3.0;
    double a = 1.0;
    ::common::TrajectoryPoint trajectory_point;
    trajectory_point.set_path_point(path_point);
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(0.0 + 1.0 * i);
    trajectory.mutable_trajectory_point()->emplace_back(trajectory_point);
  }
  return trajectory;
}

STBoundary CreatObject(double v, double s_init, double t_init, double t_end,
                       double length) {
  STBoundary obj;
  std::vector<STPoint> lower_points, upper_points;
  const int num = 50;
  for (size_t i = 0; i <= num; i++) {
    double delta_t_used = double(i) / (double)num * (t_end - t_init);
    STPoint lower_point(s_init + delta_t_used * v, t_init + delta_t_used);
    STPoint upper_point(s_init + delta_t_used * v + length,
                        t_init + delta_t_used);
    lower_points.push_back(lower_point);
    upper_points.push_back(upper_point);
  }
  obj = STBoundary::CreateInstanceAccurate(lower_points, upper_points);
  return obj;
}

class LongitudinalOptimizerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    LocalView local_view;
    local_view.fct_output = std::make_shared<zark::ads_fct::FCT_Outputs>();
    local_view.fct_output->mutable_fct_out_bus_accinfosts()
        ->set_fct_out_sts_accdistmdistlvl_edsl(
            static_cast<zark::ads_common::eDSL_DrvrSetLvl>(3));
    local_view.fct_output->mutable_fct_out_bus_accinfosts()
        ->set_fct_out_is_accsgrsdslreq_bl(0);
    ::common::PathPoint start_point(1.0, 2.0, 3.0, 0.0, 5.0, 6.0, 7.0, 8.0, "1",
                                    1.0, 2.0);
    GaussianInfo gaussian_info;
    ::common::TrajectoryPoint planning_start_point(start_point, 2.0, 2.5, 3.0,
                                                   0.0, 0.0, gaussian_info);
    common::VehicleState vehicle_state;
    frame_ = std::make_shared<Frame>(2, local_view, planning_start_point,
                                     vehicle_state);
    frame_prev_ = std::make_unique<Frame>(1, local_view, planning_start_point,
                                          vehicle_state);

    zark::planning::PlanningConfig config;
    PlanningGflags::vehicle_config_path =
        "/zark/apps/planning/config/json/vehicle_conf.json";
    std::string lon_config_file =
        "/zark/apps/planning/test/config/json/"
        "longitudinal_optimizer_config_test.json";
    Config lon_config(lon_config_file);
    std::ifstream jdata(lon_config_file);
    longitudinal_optimizer_config =
        lon_config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
    TaskConfig task_config;
    TaskConfig::task_config_union union_task_config;
    union_task_config.longitudinal_optimizer_config =
        longitudinal_optimizer_config;
    task_config.set_task_config(union_task_config);
    config.add_default_task_config(task_config);

    ADCTrajectory traj;
    ::common::Header header;
    header.set_timestamp_sec(100.0);
    traj.set_header(header);
    frame_->set_current_frame_planned_trajectory(traj);
    header.set_timestamp_sec(99.0);
    traj.set_header(header);
    frame_prev_->set_current_frame_planned_trajectory(traj);
    std::shared_ptr<DependencyInjector> injector =
        std::make_shared<DependencyInjector>();
    injector->frame_history()->Add(1, std::move(frame_prev_));

    LongitudinalLookupTables lon_lookup_tables(longitudinal_optimizer_config);
    lon_lookup_tables_ = std::make_shared<LongitudinalLookupTables>(
        longitudinal_optimizer_config);
    lon_opt_ = std::make_shared<LongitudinalOptimizer>(
        config.default_task_config().back(), injector);
  }

 protected:
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::shared_ptr<LongitudinalLookupTables> lon_lookup_tables_;
  std::shared_ptr<LongitudinalOptimizer> lon_opt_;
  std::shared_ptr<Frame> frame_;
  std::unique_ptr<Frame> frame_prev_;
};

TEST_F(LongitudinalOptimizerTest, TestExecute) {
  ::common::Status status = lon_opt_->Execute(frame_.get());
  bool is_string_same = status.ToString() == "OK";
  EXPECT_EQ(is_string_same, 1);
}

TEST_F(LongitudinalOptimizerTest, TestProcess) {
  LocalRoute local_route{};
  Mission misssion{};
  CorridorInfo::Type corridor_type;
  CorridorPoint corr_pt_1, corr_pt_2, corr_pt_3, corr_pt_4, corr_pt_5;
  corr_pt_1.s = 0.0;
  corr_pt_1.l = 0.0;
  corr_pt_2.s = 6.0;
  corr_pt_2.l = 0.0;
  corr_pt_3.s = 12.0;
  corr_pt_3.l = 0.0;
  corr_pt_4.s = 18.0;
  corr_pt_4.l = 0.0;
  corr_pt_5.s = 24.0;
  corr_pt_5.l = 0.0;
  Corridor corridor_points;
  corridor_points.emplace_back(corr_pt_1);
  corridor_points.emplace_back(corr_pt_2);
  corridor_points.emplace_back(corr_pt_3);
  corridor_points.emplace_back(corr_pt_4);
  corridor_points.emplace_back(corr_pt_5);

  SpeedLimit speed_limit_map_smooth;
  speed_limit_map_smooth.AppendSpeedLimit(-5.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(10.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(20.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(30.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(40.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(50.0, 8.0);
  std::unordered_map<std::string, SpeedLimit> speed_limit_map;
  speed_limit_map.emplace("final", speed_limit_map_smooth);
  // Obstacle map
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 1.0, 5.0, 10.0);
  Obstacle obstacle("1", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obs_map;
  obs_map.Add("1", &obstacle);
  const int idx_start_point = 0;
  CorridorInfo corr_info(local_route, misssion, corridor_type, corridor_points,
                         idx_start_point, obs_map);
  corr_info.SetSpeedLimitMap(speed_limit_map);
  Proposal proposal;
  proposal.SetCorridorInfo(&corr_info);
  std::vector<STBoundary> st_graph;
  STBoundary st_boundary;
  st_boundary = CreatObject(3.0, 0.0, 0.0, 4.0, 5.0);
  st_boundary.set_id("1");
  st_graph.push_back(st_boundary);
  STProposal st_proposal;
  st_proposal.emplace_back(std::make_tuple(&st_boundary, true, true));
  Eigen::VectorXd x_init;
  Eigen::VectorXd u_prev;
  std::tie(x_init, u_prev) = lon_opt_->ConstructEgoInitialState(
      &corr_info, frame_->PlanningStartPoint());

  DiscretizedTrajectory dis_trajectory;
  ::common::TrajectoryPoint trajectory_point;
  double ego_v = 2.0;
  for (int i = 0; i < 50; ++i) {
    trajectory_point.mutable_path_point()->set_x(0.0 + i * ego_v * 0.1);
    trajectory_point.mutable_path_point()->set_y(0.0);
    trajectory_point.mutable_path_point()->set_theta(0.0);
    trajectory_point.set_v(ego_v);
    trajectory_point.set_relative_time(0.0 + i * 0.1);
    dis_trajectory.AppendTrajectoryPoint(trajectory_point);
  }
  proposal.SetSTProposal(st_proposal);
  proposal.SetTrajectory(dis_trajectory);
  ::common::Status status = lon_opt_->Process(
      frame_->PlanningStartPoint(), proposal, frame_->local_view().fct_output);
  bool is_string_same = status.ToString() == "OK";
  EXPECT_EQ(is_string_same, 1);
}

TEST_F(LongitudinalOptimizerTest, TestConstructEgoInitialState) {
  LocalRoute local_route{};
  Mission misssion{};
  CorridorInfo::Type corridor_type;
  CorridorPoint corr_pt_1, corr_pt_2, corr_pt_3, corr_pt_4, corr_pt_5;
  corr_pt_1.s = 0.0;
  corr_pt_1.l = 0.0;
  corr_pt_1.theta = 0.0;
  corr_pt_2.s = 6.0;
  corr_pt_2.l = 0.0;
  corr_pt_2.theta = 0.0;
  corr_pt_3.s = 12.0;
  corr_pt_3.l = 0.0;
  corr_pt_3.theta = 0.0;
  corr_pt_4.s = 18.0;
  corr_pt_4.l = 0.0;
  corr_pt_4.theta = 0.0;
  corr_pt_5.s = 24.0;
  corr_pt_5.l = 0.0;
  corr_pt_5.theta = 0.0;
  Corridor corridor_points;
  corridor_points.emplace_back(corr_pt_1);
  corridor_points.emplace_back(corr_pt_2);
  corridor_points.emplace_back(corr_pt_3);
  corridor_points.emplace_back(corr_pt_4);
  corridor_points.emplace_back(corr_pt_5);

  SpeedLimit speed_limit_map_smooth;
  speed_limit_map_smooth.AppendSpeedLimit(-2.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(10.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(20.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(30.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(40.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(50.0, 8.0);
  std::unordered_map<std::string, SpeedLimit> speed_limit_map;
  speed_limit_map.emplace("final", speed_limit_map_smooth);
  // Obstacle map
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 1.0, 5.0, 10.0);
  Obstacle obstacle("1", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obs_map;
  obs_map.Add("1", &obstacle);
  const int idx_start_point = 0;
  CorridorInfo corr_info(local_route, misssion, corridor_type, corridor_points,
                         idx_start_point, obs_map);
  corr_info.SetSpeedLimitMap(speed_limit_map);

  Eigen::VectorXd x_init;
  Eigen::VectorXd u_prev;
  std::tie(x_init, u_prev) = lon_opt_->ConstructEgoInitialState(
      &corr_info, frame_->PlanningStartPoint());
  const double kTol = 1e-2;
  EXPECT_DOUBLE_EQ(x_init(0), 0.0);
  EXPECT_NEAR(x_init(1), 2.0, kTol);
  EXPECT_NEAR(u_prev(0), 2.5, kTol);
}

TEST_F(LongitudinalOptimizerTest, TestGetCostFunctionMatrix) {
  Eigen::VectorXd Q;
  Eigen::VectorXd R;
  Eigen::VectorXd R_dot;
  std::tie(Q, R, R_dot) =
      lon_opt_->GetCostFunctionMatrix(longitudinal_optimizer_config);
  EXPECT_DOUBLE_EQ(Q(0), 0.1);
  EXPECT_DOUBLE_EQ(Q(1), 1.0);
  EXPECT_DOUBLE_EQ(R(0), 1.0);
  EXPECT_DOUBLE_EQ(R_dot(0), 1.0);
}

}  // namespace planning
}  // namespace zark
