#include "apps/planning/src/decision/longitudinal/st_graph_builder.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/decision/longitudinal/obs_sl_boundary_builder.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

using namespace zark::planning;
using ::common::Status;
using ::math::Box2d;

class STGraphBuilderTest : public ::testing::Test {
 public:
  STGraphBuilderTest() {
    // using ego width fully block judgement
    PlanningGflags::use_corridor_boundary_blocking_judgement = false;
    // vehicle_param
    common::VehicleConfig vehicle_config;
    common::VehicleParam vehicle_param;
    vehicle_param.set_width(2.11);
    vehicle_param.set_length(4.933);
    vehicle_param.set_back_edge_to_center(1.043);
    vehicle_param.set_front_edge_to_center(3.89);
    vehicle_param.set_left_edge_to_center(1.055);
    vehicle_param.set_right_edge_to_center(1.055);
    vehicle_param.set_rear_axle_to_cg(1.0);
    vehicle_config.set_vehicle_param(vehicle_param);
    common::VehicleConfigHelper::Init(vehicle_config);
    LongitudinalDeciderConfig::STGraphConfig config;
    config.lateral_buffer = 0.5;
    config.safety_l_buffer = 0.1;
    config.use_obs_sl_for_st_graph = false;
    config.t_plan = 8.0;
    config.fully_blocking_buffer = 0.5;
    config.st_boundary_counter_min = 3;
    st_graph_builder_ = std::make_unique<STGraphBuilder>(config);
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

  ::common::Trajectory generate_prediction_const_trajectory(double x_start,
                                                            double y_start,
                                                            double x_end,
                                                            double y_end) {
    ::common::Trajectory trajectory;
    const int num_points = 86;
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
      double a = 0.0;
      ::common::TrajectoryPoint trajectory_point;
      trajectory_point.set_path_point(path_point);
      trajectory_point.set_v(v);
      trajectory_point.set_a(a);
      trajectory_point.set_relative_time(0.0 + 0.1 * i);
      trajectory.mutable_trajectory_point()->emplace_back(trajectory_point);
    }
    return trajectory;
  }

 protected:
  std::unique_ptr<STGraphBuilder> st_graph_builder_;
};

TEST_F(STGraphBuilderTest, TestMapObstaclesToSTBoundaries) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);

  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_id(0);
  PlanningGflags::enable_reduce_obs_traj_sample_t = false;
  ::common::Trajectory trajectory = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  obstacle_map.Add("0", &obstacle);
  ObsSLBoundaryBuilder obs_sl_boundary_builder;
  std::unordered_map<const Obstacle*, SLTrajectory> obs_sl_boundary_map;
  obs_sl_boundary_map = obs_sl_boundary_builder.BuildObsSLBoundaryMap(
      corridor, obstacle_map.Items());
  std::unordered_map<std::string, int> st_boundary_counter_map;
  st_boundary_counter_map.clear();
  st_boundary_counter_map["1"] = 2;
  std::vector<STBoundary> st_graph;
  std::vector<std::pair<const Obstacle*, bool>> obstacle_close;
  Status status = st_graph_builder_->BuildSTGraph(
      corridor, obstacle_map, obs_sl_boundary_map, st_graph, obstacle_close,
      st_boundary_counter_map);
  EXPECT_EQ(status.ToString(), "OK");
  EXPECT_EQ(st_graph.size(), 0);
  EXPECT_EQ(obstacle_close.size(), 0);
  EXPECT_EQ(st_boundary_counter_map.size(), 1);
  EXPECT_EQ(st_boundary_counter_map["0"], 1);

  for (int i = 0; i < st_graph_builder_->config_.st_boundary_counter_min; ++i) {
    status = st_graph_builder_->BuildSTGraph(
        corridor, obstacle_map, obs_sl_boundary_map, st_graph, obstacle_close,
        st_boundary_counter_map);
    EXPECT_EQ(st_boundary_counter_map["0"], i + 2);
  }
  EXPECT_EQ(status.ToString(), "OK");
  EXPECT_EQ(st_graph.size(), 1);
  EXPECT_EQ(obstacle_close.size(), 0);
  EXPECT_EQ(st_graph[0].id(), "0");
  EXPECT_DOUBLE_EQ(st_graph[0].min_t(), 0);
  EXPECT_DOUBLE_EQ(st_graph[0].min_s(), -2);
  EXPECT_DOUBLE_EQ(st_graph[0].max_t(), 8);
  EXPECT_DOUBLE_EQ(st_graph[0].max_s(), 19);
  EXPECT_EQ(st_boundary_counter_map.size(), 1);
}

TEST_F(STGraphBuilderTest, TestComputeObstacleSTBoundaryByObsOverlap) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);

  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  obstacle_map.Add("0", &obstacle);
  ObsSLBoundaryBuilder obs_sl_boundary_builder;
  std::unordered_map<const Obstacle*, SLTrajectory> obs_sl_boundary_map;
  obs_sl_boundary_map = obs_sl_boundary_builder.BuildObsSLBoundaryMap(
      corridor, obstacle_map.Items());
  const SLTrajectory sl_boundaries = obs_sl_boundary_map.at(&obstacle);
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  bool is_lateral = false;
  bool is_left = false;
  st_graph_builder_->config_.use_obs_sl_for_st_graph = false;
  bool status = st_graph_builder_->ComputeObstacleSTBoundaryByObsOverlap(
      obstacle, corridor, sl_boundaries, lower_points, upper_points, is_lateral,
      is_left);
  EXPECT_EQ(status, 1);
  EXPECT_EQ(is_lateral, false);
  EXPECT_EQ(is_left, false);
  EXPECT_EQ(lower_points.size(), upper_points.size());
  EXPECT_NEAR(lower_points.back().s(), 10, 0.1);
  EXPECT_NEAR(lower_points.back().t(), 8, 0.1);
  EXPECT_NEAR(lower_points.front().s(), -2, 0.1);
  EXPECT_NEAR(lower_points.front().t(), 0, 0.1);
}
TEST_F(STGraphBuilderTest, TestComputeObstacleSTBoundaryByObsSL) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_id(1);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  obstacle_map.Add("0", &obstacle);
  ObsSLBoundaryBuilder obs_sl_boundary_builder;
  std::unordered_map<const Obstacle*, SLTrajectory> obs_sl_boundary_map;
  obs_sl_boundary_map = obs_sl_boundary_builder.BuildObsSLBoundaryMap(
      corridor, obstacle_map.Items());
  const SLTrajectory sl_boundaries = obs_sl_boundary_map.at(&obstacle);
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  bool is_lateral = false;
  bool is_left = false;
  st_graph_builder_->config_.use_obs_sl_for_st_graph = true;
  bool status = st_graph_builder_->ComputeObstacleSTBoundaryByObsSL(
      obstacle, corridor, sl_boundaries, lower_points, upper_points, is_lateral,
      is_left);
  EXPECT_EQ(status, true);
  EXPECT_EQ(is_lateral, false);
  EXPECT_EQ(is_left, false);
  EXPECT_EQ(lower_points.size(), upper_points.size());
  EXPECT_NEAR(lower_points.back().s(), 9.65, 0.1);
  EXPECT_NEAR(lower_points.back().t(), 8, 0.1);
  EXPECT_NEAR(lower_points.front().s(), -2.35, 0.1);
  EXPECT_NEAR(lower_points.front().t(), 0, 0.1);
}
TEST_F(STGraphBuilderTest, TestGetOverlappingS) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);

  const Box2d& obs_box =
      obstacle.GetBoundingBox(trajectory.trajectory_point().front());
  std::pair<double, double> overlapping_s;
  bool is_overlapping = st_graph_builder_->GetOverlappingS(
      corridor, obs_box, st_graph_builder_->config_.safety_l_buffer,
      &overlapping_s);
  EXPECT_EQ(is_overlapping, true);
}
TEST_F(STGraphBuilderTest, TestGetSBoundingPathPointIndex) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);

  const Box2d& obs_box =
      obstacle.GetBoundingBox(trajectory.trajectory_point().front());
  int pt_idx = st_graph_builder_->GetSBoundingPathPointIndex(
      corridor, obs_box, 2.5, true, 0, static_cast<int>(corridor.size()) - 2);
  EXPECT_EQ(pt_idx, 3);
}
TEST_F(STGraphBuilderTest, TestIsPathPointAwayFromObstacle) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);

  const Box2d& obs_box =
      obstacle.GetBoundingBox(trajectory.trajectory_point().front());
  bool status_before = st_graph_builder_->IsPathPointAwayFromObstacle(
      corridor[3], corridor[4], obs_box, 2.5, true);
  EXPECT_EQ(status_before, true);
  bool status_after = st_graph_builder_->IsPathPointAwayFromObstacle(
      corridor[3], corridor[4], obs_box, 2.5, false);
  EXPECT_EQ(status_after, false);
}
TEST_F(STGraphBuilderTest, TestIsADCOverlappingWithObstacle) {
  // set corridor
  CorridorPoint corridor_point;
  ::math::Vec2d xy_point;
  corridor_point.s = 3.0;
  corridor_point.l = 0.0;
  xy_point.set_x(0.0);
  xy_point.set_y(3.0);
  corridor_point.xy_ref = xy_point;
  corridor_point.theta = 0.0;
  corridor_point.l_left = 1.75;
  corridor_point.l_right = -1.75;
  corridor_point.theta = M_PI / 2;
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 3.0, 0.0, 15.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);

  const Box2d& obs_box =
      obstacle.GetBoundingBox(trajectory.trajectory_point().front());
  bool status = st_graph_builder_->IsADCOverlappingWithObstacle(
      corridor_point, obs_box, st_graph_builder_->config_.safety_l_buffer);
  EXPECT_EQ(status, true);
}
TEST_F(STGraphBuilderTest, TestDetermineLeftOrRightObstacle) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  bool is_lateral = false;
  bool is_left = false;
  SLBoundary sl_boundary;
  sl_boundary.set_start_s(3.0);
  sl_boundary.set_end_s(5.0);
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.0);
  sl_boundary.set_t(0.0);
  st_graph_builder_->DetermineLeftOrRightObstacle(corridor, sl_boundary,
                                                  is_lateral, is_left);
  EXPECT_EQ(is_lateral, true);
  EXPECT_EQ(is_left, false);
}
TEST_F(STGraphBuilderTest, TestGetLeftAndRightLbyS) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  double lateral_left = 0.0;
  st_graph_builder_->GetLeftAndRightLbyS(corridor, 5.0, true, &lateral_left);
  EXPECT_NEAR(lateral_left, 1.75, 0.1);
}

TEST_F(STGraphBuilderTest, TestIsFullyBlocking) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  double start_l = -0.5;
  double end_l = 0.5;
  bool is_fully_blocking =
      st_graph_builder_->IsFullyBlocking(corridor, 0.0, start_l, end_l);
  EXPECT_EQ(is_fully_blocking, true);

  start_l = -1.8;
  end_l = -0.9;
  is_fully_blocking =
      st_graph_builder_->IsFullyBlocking(corridor, 0.0, start_l, end_l);
  EXPECT_EQ(is_fully_blocking, false);

  start_l = -1.8;
  end_l = -0.6;
  is_fully_blocking =
      st_graph_builder_->IsFullyBlocking(corridor, 0.0, start_l, end_l);
  EXPECT_EQ(is_fully_blocking, true);

  start_l = 0.9;
  end_l = 1.5;
  is_fully_blocking =
      st_graph_builder_->IsFullyBlocking(corridor, 0.0, start_l, end_l);
  EXPECT_EQ(is_fully_blocking, false);
}

TEST_F(STGraphBuilderTest, TestMapObstaclesToSTBoundariesPlanB) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = 0.0;
    corridor_point.l_left = 1.75;
    corridor_point.l_right = -1.75;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);

  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_id(0);
  perception_obstacle.set_velocity(::common::Point3D(1.0, 0.0, 0.0));
  PlanningGflags::enable_reduce_obs_traj_sample_t = true;
  st_graph_builder_->config_.use_obs_sl_for_st_graph = true;
  ::common::Trajectory trajectory =
      generate_prediction_const_trajectory(0.0, 0.0, 0.0, 0.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  obstacle_map.Add("0", &obstacle);
  std::unordered_map<std::string, int> st_boundary_counter_map;
  st_boundary_counter_map["0"] = 3;
  ObsSLBoundaryBuilder obs_sl_boundary_builder;
  std::unordered_map<const Obstacle*, SLTrajectory> obs_sl_boundary_map;
  obs_sl_boundary_map = obs_sl_boundary_builder.BuildObsSLBoundaryMap(
      corridor, obstacle_map.Items());

  std::vector<STBoundary> st_graph;
  std::vector<std::pair<const Obstacle*, bool>> obstacle_close;
  Status status = st_graph_builder_->BuildSTGraph(
      corridor, obstacle_map, obs_sl_boundary_map, st_graph, obstacle_close,
      st_boundary_counter_map);

  EXPECT_EQ(status.ToString(), "OK");
  EXPECT_EQ(st_graph.size(), 0);

  // TODO(zhaofu) add real static obstacle info to test
  if (!st_graph.empty()) {
    EXPECT_EQ(st_graph.size(), 1);
    EXPECT_EQ(obstacle_close.size(), 1);
    EXPECT_EQ(st_graph[0].id(), "0");
    EXPECT_NEAR(st_graph[0].min_t(), 0, 0.01);
    EXPECT_NEAR(st_graph[0].min_s(), -3.945, 0.1);
    EXPECT_EQ(obstacle_close.size(), 0);
    EXPECT_EQ(st_graph[0].id(), "0");
    EXPECT_NEAR(st_graph[0].min_t(), 0, 0.01);
    EXPECT_NEAR(st_graph[0].min_s(), -3.945, 0.1);
    EXPECT_NEAR(st_graph[0].max_t(), 0.1, 0.01);
    EXPECT_NEAR(st_graph[0].max_s(), 3.098, 0.1);
  }
}

TEST_F(STGraphBuilderTest, TestUpdateSTBoundaryCounterMap) {
  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.11);
  perception_obstacle.set_length(4.933);
  perception_obstacle.set_id(0);
  perception_obstacle.set_velocity(::common::Point3D(1.0, 0.0, 0.0));
  PlanningGflags::enable_reduce_obs_traj_sample_t = true;
  st_graph_builder_->config_.use_obs_sl_for_st_graph = true;
  ::common::Trajectory trajectory =
      generate_prediction_const_trajectory(0.0, 0.0, 0.0, 0.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  obstacle_map.Add("0", &obstacle);
  std::unordered_map<std::string, int> st_boundary_counter_map;
  st_boundary_counter_map["0"] = 2;
  st_boundary_counter_map["1"] = 1;
  st_graph_builder_->UpdateSTBoundaryCounterMap(obstacle_map,
                                                st_boundary_counter_map);

  EXPECT_EQ(st_boundary_counter_map.size(), 1);
  EXPECT_EQ(st_boundary_counter_map["0"], 2);
}

}  // namespace planning
}  // namespace zark
