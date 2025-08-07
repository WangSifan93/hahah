/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_optimizer_test.cc
 **/

#include "apps/planning/src/tasks/optimizers/lateral_optimizer/lateral_optimizer.h"

#include "apps/planning/src/config/config_main.h"
#include "apps/planning/src/motion/lateral/lateral_constraint.h"
#include "gtest/gtest.h"

using ::util::PointFactory;

namespace zark {
namespace planning {

namespace {
const int n_steps = 40;  // same to lateral_optimizer_config_test
const int n_nodes = n_steps + 1;
const double kLMax = 100.0;
const double kTol = 1.0e-3;
const int n_x = 4;
const int n_u = 1;
const int n_u_dot = 1;
const double dt = 0.2;
}  // namespace

class LateralOptimizerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string lat_config_file =
        "/zark/apps/planning/test/config/json/"
        "lateral_optimizer_config_test.json";
    Config lat_config(lat_config_file);
    std::ifstream jdata(lat_config_file);
    lateral_optimizer_config =
        lat_config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
    TaskConfig task_config;
    TaskConfig::task_config_union union_task_config;
    union_task_config.lateral_optimizer_config = lateral_optimizer_config;
    task_config.set_task_config(union_task_config);
    zark::planning::PlanningConfig config;
    config.add_default_task_config(task_config);

    LocalView local_view;
    ::common::PathPoint start_point(1.0, 2.0, 3.0, 0.0, 5.0, 6.0, 7.0, 8.0, "1",
                                    1.0, 2.0);
    GaussianInfo gaussian_info;
    ::common::TrajectoryPoint planning_start_point(start_point, 2.0, 2.5, 3.0,
                                                   0.0, 0.0, gaussian_info);
    planning_start_point.set_dt_prev(0.1);
    common::VehicleState vehicle_state;
    PlanningGflags::vehicle_config_path =
        "/zark/apps/planning/config/json/vehicle_conf.json";
    frame_prev_ = std::make_unique<Frame>(1, local_view, planning_start_point,
                                          vehicle_state);
    frame_ = std::make_shared<Frame>(2, local_view, planning_start_point,
                                     vehicle_state);
    std::shared_ptr<DependencyInjector> injector =
        std::make_shared<DependencyInjector>();
    injector->frame_history()->Add(1, std::move(frame_prev_));
    lat_opt_ = std::make_shared<LateralOptimizer>(
        config.default_task_config().back(), injector);
  }

 protected:
  LateralOptimizerConfig lateral_optimizer_config;
  std::shared_ptr<LateralOptimizer> lat_opt_;
  std::shared_ptr<Frame> frame_;
  std::unique_ptr<Frame> frame_prev_;
};

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

::common::Trajectory generate_lat_mpc_trajectory(double x_start, double y_start,
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
    trajectory_point.set_relative_time(0.0 + 0.1 * i);
    trajectory.mutable_trajectory_point()->emplace_back(trajectory_point);
  }
  return trajectory;
}

CorridorInfo InitializeCorridorInfo() {
  static LocalRoute local_route{};
  local_route = LocalRoute();
  static Mission misssion{};
  misssion = Mission();
  CorridorInfo::Type corridor_type;

  std::vector<CorridorPoint> corridor_points;
  const double speed = 3.0;
  for (int k = 0; k < n_nodes; k++) {
    CorridorPoint corridor_pt;
    corridor_pt.s = k * dt * speed;
    corridor_pt.l = 0.5;
    corridor_pt.s_ref = k * dt * speed;
    corridor_pt.l_ref = 0.0;
    corridor_pt.l_left = 2.5;
    corridor_pt.l_right = -2.5;
    corridor_pt.theta = 0.0;
    corridor_pt.kappa = 0.01;
    corridor_points.emplace_back(corridor_pt);
  }
  Corridor corridor(corridor_points);
  const int idx_start_point = 0;

  // Obstacle map
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_length(1.0);
  ::common::Trajectory trajectory =
      generate_lat_mpc_trajectory(0.0, 1.0, 5.0, 10.0);
  Obstacle obstacle("1", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obs_map;
  obs_map.Add("1", &obstacle);
  CorridorInfo corr_info(local_route, misssion, corridor_type, corridor,
                         idx_start_point, obs_map);
  std::vector<std::pair<const Obstacle*, bool>> lateral_obstacles;
  lateral_obstacles.emplace_back(std::make_pair(&obstacle, false));
  corr_info.SetLateralObstacles(lateral_obstacles);
  ::common::FrenetPoint init_frenet_point;
  init_frenet_point.s[0] = 0.0;
  init_frenet_point.s[1] = 2.0;
  init_frenet_point.l[0] = 0.5;
  init_frenet_point.l[1] = 1.0;
  corr_info.SetInitFrenetPoint(init_frenet_point);

  return corr_info;
}

void InitializeLonMPCData(LonMPCData& lon_mpc_data) {
  lon_mpc_data.x.resize(2, n_nodes);
  lon_mpc_data.t.resize(1, n_nodes);
  lon_mpc_data.u.resize(1, n_steps);
  lon_mpc_data.u_dot.resize(1, n_steps);
  lon_mpc_data.x.row(0) = Eigen::RowVectorXd::Ones(n_nodes);
  lon_mpc_data.x.row(1) = Eigen::RowVectorXd::Ones(n_nodes);
  lon_mpc_data.u.row(0) = Eigen::RowVectorXd::Zero(n_steps);
  lon_mpc_data.u_dot.row(0) = Eigen::RowVectorXd::Zero(n_steps);
  for (int k = 0; k < n_nodes; k++) {
    lon_mpc_data.t[k] = dt * k;
  }
  lon_mpc_data.is_stop_hold = false;
}

void InitializeLatMPCData(LatMPCData& lat_mpc_data) {
  lat_mpc_data.x.resize(n_x, n_nodes);
  lat_mpc_data.t.resize(1, n_nodes);
  lat_mpc_data.u.resize(n_u, n_steps);
  lat_mpc_data.u_dot.resize(n_u_dot, n_steps);
  lat_mpc_data.x.row(0) = Eigen::RowVectorXd::Ones(n_nodes);
  lat_mpc_data.x.row(1) = Eigen::RowVectorXd::Zero(n_nodes);
  lat_mpc_data.x.row(2) = Eigen::RowVectorXd::Zero(n_nodes);
  lat_mpc_data.x.row(3) = Eigen::RowVectorXd::Zero(n_nodes);
  lat_mpc_data.u.row(0) = Eigen::RowVectorXd::Zero(n_steps);
  lat_mpc_data.u_dot.row(0) = Eigen::RowVectorXd::Zero(n_steps);
  for (int k = 0; k < n_nodes; k++) {
    lat_mpc_data.t[k] = dt * k;
  }
}

void InitializeTube(Tube& tube) {
  tube.pts.reserve(n_nodes);
  const double time_step = 0.1;
  const double speed = 3.0;
  for (int k = 0; k < n_nodes; k++) {
    Tube::TubePoint tube_point;
    tube_point.t = time_step * k;
    tube_point.s = speed * time_step * k;
    tube_point.l_left_soft = 2.0;
    tube_point.l_right_soft = -3.0;
    tube_point.l_left_stiff = 2.0;
    tube_point.l_right_stiff = -3.0;
    tube_point.l_left_hard = 2.0;
    tube_point.l_right_hard = -3.0;
    tube.pts.emplace_back(tube_point);
  }
}

::common::TrajectoryPoint InitializePlanningStartPoint() {
  ::common::PathPoint start_point(1.0, 2.0, 3.0, M_PI_2, 5.0, 6.0, 7.0, 8.0,
                                  "1", 1.0, 2.0);
  GaussianInfo gaussian_info;
  // steer 1.0
  ::common::TrajectoryPoint planning_start_point(start_point, 2.0, 2.5, 3.0,
                                                 0.0, 1.0, gaussian_info);
  planning_start_point.set_omega(0.1);
  return planning_start_point;
}

TEST_F(LateralOptimizerTest, TestExecute) {
  ::common::Status status = lat_opt_->Execute(frame_.get());
  bool is_string_same = status.ToString() == "OK";
  EXPECT_EQ(is_string_same, 1);
}

TEST_F(LateralOptimizerTest, TestProcessStopTrajectory) {
  Proposal proposal;
  LonMPCData lon_mpc_data;
  lon_mpc_data.x.resize(2, n_nodes);
  lon_mpc_data.t.resize(1, n_nodes);
  lon_mpc_data.u.resize(1, n_steps);
  lon_mpc_data.u_dot.resize(1, n_steps);
  lon_mpc_data.x.row(0) = Eigen::RowVectorXd::Ones(n_nodes);
  lon_mpc_data.x.row(1) = Eigen::RowVectorXd::Ones(n_nodes);
  lon_mpc_data.u.row(0) = Eigen::RowVectorXd::Zero(n_steps);
  lon_mpc_data.u_dot.row(0) = Eigen::RowVectorXd::Zero(n_steps);

  for (int k = 0; k < n_nodes; k++) {
    lon_mpc_data.t[k] = dt * k;
  }
  lon_mpc_data.is_stop_hold = true;
  proposal.SetLonMPCData(lon_mpc_data);
  CorridorInfo corridor_info = InitializeCorridorInfo();
  ::common::TrajectoryPoint planning_start_point =
      InitializePlanningStartPoint();

  proposal.SetCorridorInfo(&corridor_info);
  static LocalRoute local_route{};
  local_route = LocalRoute();
  ::common::Status status =
      lat_opt_->Process(planning_start_point, local_route, proposal);
  LatMPCData lat_mpc_data = proposal.GetLatMPCData();
  bool is_string_same = status.ToString() == "OK";
  EXPECT_EQ(is_string_same, 1);
  for (int k = 0; k < n_nodes; k++) {
    EXPECT_DOUBLE_EQ(lat_mpc_data.x(0, k), 0.0);
    EXPECT_DOUBLE_EQ(lat_mpc_data.x(1, k), 0.0);
    EXPECT_DOUBLE_EQ(lat_mpc_data.x(2, k), 0.0);
    EXPECT_DOUBLE_EQ(lat_mpc_data.x(3, k), 0.0);
  }

  for (int k = 0; k < n_steps; k++) {
    EXPECT_DOUBLE_EQ(lat_mpc_data.u(0, k), 0.0);
    EXPECT_DOUBLE_EQ(lat_mpc_data.u_dot(0, k), 0.0);
  }
}

TEST_F(LateralOptimizerTest, TestProcess) {
  LonMPCData lon_mpc_data;
  InitializeLonMPCData(lon_mpc_data);
  CorridorInfo corridor_info = InitializeCorridorInfo();
  ::common::TrajectoryPoint planning_start_point =
      InitializePlanningStartPoint();

  Proposal proposal;
  proposal.SetCorridorInfo(&corridor_info);
  proposal.SetLonMPCData(lon_mpc_data);
  static LocalRoute local_route{};
  local_route = LocalRoute();
  
  ::common::Status status = lat_opt_->Process(planning_start_point, local_route, proposal);
  bool is_string_same = status.ToString() == "OK";
  EXPECT_EQ(is_string_same, 1);
}

TEST_F(LateralOptimizerTest, TestConstructEgoInitialState) {
  CorridorInfo corridor_info = InitializeCorridorInfo();
  ::common::TrajectoryPoint planning_start_point =
      InitializePlanningStartPoint();
  Eigen::VectorXd x_init;
  Eigen::VectorXd u_prev;
  std::tie(x_init, u_prev) =
      lat_opt_->ConstructEgoInitialState(planning_start_point, corridor_info);
  EXPECT_DOUBLE_EQ(u_prev(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(x_init(0), 0.5);
  EXPECT_DOUBLE_EQ(x_init(1), 2.0);
  EXPECT_DOUBLE_EQ(x_init(2), M_PI_2);
  EXPECT_DOUBLE_EQ(x_init(3), 0.08);
}

TEST_F(LateralOptimizerTest, TestComputeU2) {
  LonMPCData lon_mpc_data;
  Tube tube;
  InitializeLonMPCData(lon_mpc_data);
  CorridorInfo corridor_info = InitializeCorridorInfo();
  InitializeTube(tube);
  Eigen::MatrixXd u_2 = lat_opt_->ComputeU2(lon_mpc_data, tube, corridor_info);
  for (int k = 0; k < u_2.cols(); k++) {
    EXPECT_DOUBLE_EQ(u_2(0, k), 0.01);
  }
}

TEST_F(LateralOptimizerTest, TestGetCostFunctionMatrix) {
  Eigen::VectorXd Q;
  Eigen::VectorXd R;
  Eigen::VectorXd R_dot;
  std::tie(Q, R, R_dot) =
      lat_opt_->GetCostFunctionMatrix(lateral_optimizer_config);
  EXPECT_DOUBLE_EQ(Q(0), 1.0);
  EXPECT_DOUBLE_EQ(Q(1), 0.0);
  EXPECT_DOUBLE_EQ(Q(2), 1.0);
  EXPECT_DOUBLE_EQ(Q(3), 0.0);
  EXPECT_DOUBLE_EQ(R(0), 1.0);
  EXPECT_DOUBLE_EQ(R_dot(0), 1.0);
}

TEST_F(LateralOptimizerTest, TestConvertToTrajectory) {
  CorridorInfo corridor_info = InitializeCorridorInfo();

  LonMPCData lon_mpc_data;
  InitializeLonMPCData(lon_mpc_data);

  LatMPCData lat_mpc_data;
  InitializeLatMPCData(lat_mpc_data);

  Proposal proposal;
  proposal.SetLonMPCData(lon_mpc_data);
  proposal.SetLatMPCData(lat_mpc_data);
  proposal.SetCorridorInfo(&corridor_info);

  DiscretizedTrajectory traj = lat_opt_->ConvertToTrajectory(proposal);

  std::vector<double> s_results, l_results, v_results, psi_results, a_results,
      steer_results, steer_rate_results, da_results, omega_results,
      kappa_results;
  s_results.assign(n_nodes, 0.0);
  l_results.assign(n_nodes, 1.0);
  v_results.assign(n_nodes, 1.0);
  psi_results.assign(n_nodes, 0.0);
  omega_results.assign(n_nodes, 0.01);
  kappa_results.assign(n_nodes, 0.01);
  a_results.assign(n_steps, 0.0);
  a_results.emplace_back(a_results[n_steps - 1]);
  steer_results.assign(n_steps, 0.0);
  steer_results.emplace_back(steer_results[n_steps - 1]);
  da_results.assign(n_steps - 1, 0.0);
  da_results.emplace_back(da_results[n_steps - 2]);
  steer_rate_results.assign(n_steps - 1, 0.0);
  steer_rate_results.emplace_back(steer_rate_results[n_steps - 2]);

  for (int k = 0; k < n_steps - 1; ++k) {
    EXPECT_NEAR(traj[k].da(), da_results[k], kTol);
    EXPECT_NEAR(traj[k].steer_rate(), steer_rate_results[k], kTol);
  }
  for (int k = n_steps - 1; k < n_nodes; ++k) {
    EXPECT_NEAR(traj[k].da(), da_results[n_steps - 2], kTol);
    EXPECT_NEAR(traj[k].steer_rate(), steer_rate_results[n_steps - 2], kTol);
  }

  for (int k = 0; k < n_steps; ++k) {
    EXPECT_NEAR(traj[k].a(), a_results[k], kTol);
    EXPECT_NEAR(traj[k].steer(), steer_results[k], kTol);
  }
  for (int k = n_steps; k < n_nodes; ++k) {
    EXPECT_NEAR(traj[k].a(), a_results[n_steps - 1], kTol);
    EXPECT_NEAR(traj[k].steer(), steer_results[n_steps - 1], kTol);
  }

  for (int k = 0; k < n_nodes; ++k) {
    EXPECT_NEAR(traj[k].path_point().s(), s_results[k], kTol);
    EXPECT_NEAR(traj[k].omega(), omega_results[k], kTol);
    EXPECT_NEAR(traj[k].path_point().kappa(), kappa_results[k], kTol);
  }
}

}  // namespace planning
}  // namespace zark
