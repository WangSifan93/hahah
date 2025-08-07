#include <fstream>
#include <iostream>
#include <string>

#include "common/corridor.h"
#include "polygon2d.h"
#include "decision/longitudinal/obs_sl_boundary_builder.h"
#include "gtest/gtest.h"

using ::math::Polygon2d;
using namespace zpilot::planning;

Trajectory generate_trajectory(double x_start, double y_start, double x_end,
                               double y_end) {
  ::common::Trajectory trajectory;
  const int num_points = 17;
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

Trajectory generate_trajectory() {
  ::common::Trajectory trajectory;
  const int num_points = 16;
  double radius = 20.0;
  for (int i = 2; i < num_points; ++i) {
    double angle = 0.5 * i / radius;
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
    trajectory_point.set_relative_time(0.0 + 0.5 * i);
    trajectory.mutable_trajectory_point()->emplace_back(trajectory_point);
  }
  return trajectory;
}

TEST(ObsSLBoundaryBuilderTest, basic_test) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(-10 + 1.0 * i);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = M_PI / 2;
    corridor_points.emplace_back(corridor_point);
  }

  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_length(1.0);
  ::common::Trajectory trajectory = generate_trajectory(0.0, -3.0, 0.0, 6.0);
  Obstacle obstacle("0", perception_obstacle, trajectory);

  common::SLPoint sl_point;
  std::vector<SLPoint> sl_points;
  const auto& obs_trajectory = obstacle.Trajectory();

  // Corridor init
  Corridor corridor_points_test(corridor_points);

  for (const auto& obs_traj_pt : obs_trajectory.trajectory_point()) {
    Polygon2d cur_polygon = obstacle.GetPolygonAtPoint(obs_traj_pt);
    for (const auto& p : cur_polygon.points()) {
      ::math::Vec2d* sl2xy_point = new ::math::Vec2d();
      corridor_points_test.XYToSL(p, sl_point);
      sl_points.emplace_back(sl_point);
      corridor_points_test.SLToXY(sl_point, sl2xy_point);
      EXPECT_DOUBLE_EQ(p.x(), sl2xy_point->x());
      EXPECT_DOUBLE_EQ(p.y(), sl2xy_point->y());
    }
  }
  EXPECT_NEAR(sl_points[0].s(), 1, 0.01);
  EXPECT_NEAR(sl_points[0].l(), 0.5, 0.01);
  EXPECT_NEAR(sl_points[4].s(), 1.5625, 0.01);
  EXPECT_NEAR(sl_points[4].l(), 0.5, 0.01);
  EXPECT_NEAR(sl_points[8].s(), 2.125, 0.01);
  EXPECT_NEAR(sl_points[8].l(), 0.5, 0.01);
  EXPECT_NEAR(sl_points[12].s(), 2.6875, 0.01);
  EXPECT_NEAR(sl_points[12].l(), 0.5, 0.01);
}

TEST(ObsSLBoundaryBuilderTest, basic_test_curve) {
  // set corridor
  CorridorPoint corridor_point;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  double radius = 20.0;
  for (int i = 0; i < 25; i++) {
    corridor_point.s = 0.0 + 0.5 * i;
    corridor_point.l = 0.0;
    double angle = corridor_point.s / radius;
    double x = radius - radius * std::cos(angle);
    double y = radius * std::sin(angle);
    xy_point.set_x(x);
    xy_point.set_y(y);
    corridor_point.xy_ref = xy_point;
    corridor_point.theta = M_PI / 2 - angle;
    corridor_points.emplace_back(corridor_point);
  }

  // set obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_length(1.0);
  ::common::Trajectory trajectory = generate_trajectory();
  Obstacle obstacle("0", perception_obstacle, trajectory);

  common::SLPoint sl_point;
  std::vector<SLPoint> sl_points;
  const auto& obs_trajectory = obstacle.Trajectory();

  // Corridor init
  Corridor corridor_points_test(corridor_points);

  for (const auto& obs_traj_pt : obs_trajectory.trajectory_point()) {
    Polygon2d cur_polygon = obstacle.GetPolygonAtPoint(obs_traj_pt);
    for (const auto& p : cur_polygon.points()) {
      ::math::Vec2d* sl2xy_point = new ::math::Vec2d();
      corridor_points_test.XYToSL(p, sl_point);
      sl_points.emplace_back(sl_point);
      corridor_points_test.SLToXY(sl_point, sl2xy_point);
      EXPECT_NEAR(p.x(), sl2xy_point->x(), 0.02);
      EXPECT_NEAR(p.y(), sl2xy_point->y(), 0.02);
    }
  }
  EXPECT_NEAR(sl_points[0].s(), 0.513, 0.01);
  EXPECT_NEAR(sl_points[0].l(), 1.0, 0.01);
  EXPECT_NEAR(sl_points[4].s(), 1.013, 0.01);
  EXPECT_NEAR(sl_points[4].l(), 1.0, 0.01);
  EXPECT_NEAR(sl_points[8].s(), 1.513, 0.01);
  EXPECT_NEAR(sl_points[8].l(), 1.0, 0.01);
  EXPECT_NEAR(sl_points[12].s(), 2.013, 0.01);
  EXPECT_NEAR(sl_points[12].l(), 1.0, 0.01);
}

TEST(CorridorTest, EvaluateByS) {
  CorridorPoint corridor_point_1, corridor_point_2;
  corridor_point_1.s = 1.0;
  corridor_point_1.l = 1.0;
  corridor_point_1.l_left = 2.5;
  corridor_point_1.l_right = -3.0;
  corridor_point_1.kappa = 0.01;
  corridor_point_1.theta = 0;
  corridor_point_1.type_left = CorridorPoint::Type::CURB;
  corridor_point_1.type_right = CorridorPoint::Type::DASHED_WHITE;

  corridor_point_2.s = 2.0;
  corridor_point_2.l = 2.0;
  corridor_point_2.l_left = 3.0;
  corridor_point_2.l_right = -2.5;
  corridor_point_2.kappa = 0.02;
  corridor_point_2.theta = M_PI / 2.0;
  corridor_point_2.type_left = CorridorPoint::Type::UNKNOWN;
  corridor_point_2.type_right = CorridorPoint::Type::CURB;

  Corridor corridor_points;
  corridor_points.emplace_back(corridor_point_1);
  corridor_points.emplace_back(corridor_point_2);
  double s = 0.0;

  s = 0.5;
  CorridorPoint evaluate_point_1 = corridor_points.EvaluateByS(s);
  EXPECT_DOUBLE_EQ(evaluate_point_1.l, 1.0);
  EXPECT_DOUBLE_EQ(evaluate_point_1.l_left, 2.5);
  EXPECT_DOUBLE_EQ(evaluate_point_1.l_right, -3.0);
  EXPECT_DOUBLE_EQ(evaluate_point_1.theta, 0.0);
  EXPECT_DOUBLE_EQ(evaluate_point_1.kappa, 0.01);
  EXPECT_EQ(evaluate_point_1.type_left, corridor_point_1.type_left);
  EXPECT_EQ(evaluate_point_1.type_right, corridor_point_1.type_right);

  s = 1.5;
  CorridorPoint evaluate_point_2 = corridor_points.EvaluateByS(s);
  EXPECT_DOUBLE_EQ(evaluate_point_2.l, 1.5);
  EXPECT_DOUBLE_EQ(evaluate_point_2.l_left, 2.75);
  EXPECT_DOUBLE_EQ(evaluate_point_2.l_right, -2.5);
  EXPECT_DOUBLE_EQ(evaluate_point_2.theta, M_PI / 4);
  EXPECT_DOUBLE_EQ(evaluate_point_2.kappa, 0.015);
  EXPECT_EQ(evaluate_point_2.type_left, corridor_point_2.type_left);
  EXPECT_EQ(evaluate_point_2.type_right, corridor_point_2.type_right);

  s = 2.5;
  CorridorPoint evaluate_point_3 = corridor_points.EvaluateByS(s);
  EXPECT_DOUBLE_EQ(evaluate_point_3.l, 2.0);
  EXPECT_DOUBLE_EQ(evaluate_point_3.l_left, 3.0);
  EXPECT_DOUBLE_EQ(evaluate_point_3.l_right, -2.5);
  EXPECT_DOUBLE_EQ(evaluate_point_3.theta, M_PI / 2.0);
  EXPECT_DOUBLE_EQ(evaluate_point_3.kappa, 0.02);
  EXPECT_EQ(evaluate_point_3.type_left, corridor_point_2.type_left);
  EXPECT_EQ(evaluate_point_3.type_right, corridor_point_2.type_right);
}
