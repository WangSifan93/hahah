#include "decision/longitudinal/obs_sl_boundary_builder.h"

#include <fstream>
#include <iostream>
#include <string>

#include "common/corridor.h"
#include "common/corridor_info.h"
#include "gtest/gtest.h"

using namespace zark::planning;

::common::Trajectory generate_trajectory(double x_start, double y_start,
                                         double x_end, double y_end) {
  ::common::Trajectory trajectory;
  const int num_points = 9;
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    double x = x_start + t * (x_end - x_start);
    double y = y_start + t * (y_end - y_start);
    ::common::PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    double v = 1.0;
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

::common::Trajectory generate_trajectory() {
  ::common::Trajectory trajectory;
  const int num_points = 9;
  double radius = 10.0;
  for (int i = 1; i <= num_points; ++i) {
    double angle = 1.0 * i / radius;
    double x = radius - radius * std::cos(angle);
    double y = radius * std::sin(angle);
    ::common::PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    path_point.set_theta(M_PI / 2 - angle);
    double v = 1.0;
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

TEST(ObsSLBoundaryBuilderTest, basic_test) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 15; i++) {
    corridor_point.s = -5.0 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(5.0);
    xy_point.set_y(-5.0 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);

  // set obstacle
  perception::PerceptionObstacle perception_obstacle_0;
  perception_obstacle_0.set_width(2.0);
  perception_obstacle_0.set_length(1.0);
  ::common::Trajectory trajectory_0 = generate_trajectory(5.0, -4.0, 5.0, 6.0);
  ::common::Trajectory trajectory_1 = generate_trajectory(2.0, 0.0, 7.0, 3.0);
  ::common::Trajectory trajectory_2 = generate_trajectory(7.0, -4.0, 7.0, 6.0);
  ::common::Trajectory trajectory_3 = generate_trajectory(2.0, -4.0, 2.0, 6.0);
  Obstacle obstacle_0("0", perception_obstacle_0, trajectory_0);
  Obstacle obstacle_1("1", perception_obstacle_0, trajectory_1);
  Obstacle obstacle_2("2", perception_obstacle_0, trajectory_2);
  Obstacle obstacle_3("3", perception_obstacle_0, trajectory_3);
  std::vector<const Obstacle*> obstacles;
  obstacles.emplace_back(&obstacle_0);
  obstacles.emplace_back(&obstacle_1);
  obstacles.emplace_back(&obstacle_2);
  obstacles.emplace_back(&obstacle_3);

  ObsSLBoundaryBuilder obs_sl_boundary_builder;
  std::unordered_map<const Obstacle*, SLTrajectory>
      obs_sl_boundary_map;
  obs_sl_boundary_map =
      obs_sl_boundary_builder.BuildObsSLBoundaryMap(corridor, obstacles);
  for (const auto& entry : obs_sl_boundary_map) {
    const Obstacle* obstacle_ptr = entry.first;  //  const Obstacle ptr
    const SLTrajectory& sl_boundaries = entry.second;  // SLBoundary
    EXPECT_EQ(sl_boundaries.size(),
              obstacle_ptr->Trajectory().trajectory_point().size());
  }
  EXPECT_EQ(obs_sl_boundary_map.size(), obstacles.size());
}

TEST(ObsSLBoundaryBuilderTest, basic_test_curve) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  double radius = 10.0;
  for (int i = 0; i < 12; i++) {
    corridor_point.s = 0.0 + 1.0 * i;
    corridor_point.l = 0.0;
    double angle = corridor_point.s / radius;
    double x = radius - radius * std::cos(angle);
    double y = radius * std::sin(angle);
    xy_point.set_x(x);
    xy_point.set_y(y);
    corridor_point.xy_ref = xy_point;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  // set obstacle
  perception::PerceptionObstacle perception_obstacle_0;
  perception_obstacle_0.set_width(1.0);
  perception_obstacle_0.set_length(1.0);
  ::common::Trajectory trajectory_0 = generate_trajectory();
  ::common::Trajectory trajectory_1 = generate_trajectory(3.5, 0.5, 3.5, 7.5);
  Obstacle obstacle_0("0", perception_obstacle_0, trajectory_0);
  Obstacle obstacle_1("1", perception_obstacle_0, trajectory_1);
  std::vector<const Obstacle*> obstacles;
  obstacles.emplace_back(&obstacle_0);
  obstacles.emplace_back(&obstacle_1);
  ObsSLBoundaryBuilder obs_sl_boundary_builder;
  std::unordered_map<const Obstacle*, SLTrajectory>
      obs_sl_boundary_map;
  obs_sl_boundary_map =
      obs_sl_boundary_builder.BuildObsSLBoundaryMap(corridor, obstacles);
  for (const auto& entry : obs_sl_boundary_map) {
    const Obstacle* obstacle_ptr = entry.first;  //  const Obstacle ptr
    const SLTrajectory& sl_boundaries = entry.second;  // SLBoundary
    EXPECT_EQ(sl_boundaries.size(),
              obstacle_ptr->Trajectory().trajectory_point().size());
  }
  EXPECT_EQ(obs_sl_boundary_map.size(), obstacle_map.size());
}
