#include "apps/planning/src/common/frame.h"

#include "apps/common/basic_msgs/geometry.h"
#include "apps/common/basic_msgs/pnc_point.h"
#include "apps/planning/src/config/config_main.h"

#include <fstream>
#include <iostream>
#include <string>

#include "gtest/gtest.h"

namespace zark {
namespace planning {

void SetConfigFile(LocalRouteConfig &local_route_config) {
  common::VehicleConfig vehicle_config;
  common::VehicleParam vehicle_param_;
  vehicle_param_.set_width(2.11);
  vehicle_param_.set_length(4.933);
  vehicle_param_.set_steer_ratio(16);
  vehicle_param_.set_max_steer_angle(8.20304748437);
  vehicle_param_.set_max_steer_angle_rate(8.55211);
  vehicle_config.set_vehicle_param(vehicle_param_);
  common::VehicleConfigHelper::Init(vehicle_config);
  std::string reference_line_config_file(
      "/zark/apps/planning/test/config/json/frame_conf_test.json");
  zark::planning::Config config_instance{reference_line_config_file};
  config_instance.SetLocalRouteConfig(local_route_config);
}

void SetPerceptionObstacleInfo(
    perception::PerceptionObstacle &perception_obstacle, const PathPoint point,
    const double length, const double width, const Point3D &velocity) {
  perception_obstacle.mutable_position()->set_x(point.x());
  perception_obstacle.mutable_position()->set_y(point.y());
  perception_obstacle.mutable_position()->set_z(0.0);
  perception_obstacle.set_theta(point.theta());
  perception_obstacle.set_length(length);
  perception_obstacle.set_width(width);
  perception_obstacle.set_velocity(velocity);
  perception_obstacle.set_type(perception::Type::VEHICLE);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  const double dx = length / 2.0;
  const double dy = width / 2.0;
  std::vector<::common::Point3D> polygon_points;
  ::common::Point3D polygon_point_1, polygon_point_2, polygon_point_3,
      polygon_point_4;
  polygon_point_1.set_x(point.x() + dx * cos(point.theta()) -
                        dy * sin(point.theta()));
  polygon_point_1.set_y(point.y() + dx * sin(point.theta()) +
                        dy * cos(point.theta()));
  polygon_point_2.set_x(point.x() + dx * cos(point.theta()) +
                        dy * sin(point.theta()));
  polygon_point_2.set_y(point.y() + dx * sin(point.theta()) -
                        dy * cos(point.theta()));
  polygon_point_3.set_x(point.x() - dx * cos(point.theta()) -
                        dy * sin(point.theta()));
  polygon_point_3.set_y(point.y() - dx * sin(point.theta()) +
                        dy * cos(point.theta()));
  polygon_point_4.set_x(point.x() - dx * cos(point.theta()) +
                        dy * sin(point.theta()));
  polygon_point_4.set_y(point.y() - dx * sin(point.theta()) -
                        dy * cos(point.theta()));
  polygon_points.emplace_back(polygon_point_1);
  polygon_points.emplace_back(polygon_point_2);
  polygon_points.emplace_back(polygon_point_3);
  polygon_points.emplace_back(polygon_point_4);
  perception_obstacle.set_polygon_point(polygon_points);
}

::common::Trajectory GenerateObstacleTrajectory(const double v,
                                                const PathPoint current_point) {
  ::common::Trajectory trajectory;
  for (double t = 0.0; t < 8.0; t = t + 0.1) {
    ::common::TrajectoryPoint *point = trajectory.add_trajectory_point();
    point->set_a(0.0);
    point->set_da(0.0);
    point->set_steer(0.0);
    point->set_relative_time(t);
    point->set_v(v);

    ::common::PathPoint path_point;
    double dx = t * v;
    double dy = 0.0;
    path_point.set_x(current_point.x() + dx);
    path_point.set_y(current_point.y() + dy);
    path_point.set_z(0.0);
    path_point.set_kappa(0.0);
    path_point.set_dkappa(0.0);
    path_point.set_ddkappa(0.0);
    path_point.set_theta(current_point.theta());
    path_point.set_s(v * t);

    point->set_path_point(path_point);
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

TEST(FrameTest, TestCurrentLaneObstacleFilter) {
  LocalRouteConfig local_route_config;
  SetConfigFile(local_route_config);
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(local_route_points);

  LocalRoute local_route(local_route_points, local_route_config);
  //   common::VehicleState vehicle_state;
  ::common::TrajectoryPoint start_point;
  start_point.mutable_path_point()->set_x(50.1);
  start_point.mutable_path_point()->set_y(0.01);
  start_point.mutable_path_point()->set_theta(0.0);

  std::vector<Obstacle *> obstacles;
  perception::PerceptionObstacle front_perception_obstacle;
  PathPoint front_perception_point;
  front_perception_point.set_x(100.0);
  front_perception_point.set_y(0.2);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle(
      "01", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle);

  front_perception_point.set_x(150.0);
  front_perception_point.set_y(0.2);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle2(
      "02", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle2);

  front_perception_point.set_x(200.0);
  front_perception_point.set_y(0.2);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle3(
      "03", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle3);

  front_perception_point.set_x(10.0);
  front_perception_point.set_y(0.2);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle4(
      "04", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle4);

  front_perception_point.set_x(-20.0);
  front_perception_point.set_y(0.2);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle5(
      "05", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle5);
  uint32_t sequence_num = 1;
  LocalView local_view;
  common::VehicleState vehicle_state;
  Frame frame(sequence_num, local_view, start_point, vehicle_state);
  for (auto &ptr : obstacles) {
    frame.AddObstacleRegionType(*ptr, local_route);
    frame.AddObstacle(*ptr);
  }
  EXPECT_EQ(frame.obstacles().size(), 5);
  EXPECT_TRUE(frame.Find("01")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::FRONT);
  EXPECT_TRUE(frame.Find("02")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::FRONT);
  EXPECT_TRUE(frame.Find("03")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::FRONT);
  EXPECT_TRUE(frame.Find("04")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::REAR);
}

TEST(FrameTest, TestLeftLaneObstacleFilter) {
  LocalRouteConfig local_route_config;
  SetConfigFile(local_route_config);
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(local_route_points);

  LocalRoute local_route(local_route_points, local_route_config);
  common::VehicleState vehicle_state;
  ::common::TrajectoryPoint start_point;
  start_point.mutable_path_point()->set_x(50.1);
  start_point.mutable_path_point()->set_y(0.01);
  start_point.mutable_path_point()->set_theta(0.0);

  std::vector<Obstacle *> obstacles;
  perception::PerceptionObstacle front_perception_obstacle;
  PathPoint front_perception_point;
  front_perception_point.set_x(100.0);
  front_perception_point.set_y(4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle(
      "01", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle);

  front_perception_point.set_x(200.0);
  front_perception_point.set_y(4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle2(
      "02", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle2);

  front_perception_point.set_x(300.0);
  front_perception_point.set_y(4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle3(
      "03", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle3);

  front_perception_point.set_x(50.0);
  front_perception_point.set_y(4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle4(
      "04", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle4);

  front_perception_point.set_x(25.0);
  front_perception_point.set_y(4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle5(
      "05", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle5);

  front_perception_point.set_x(-20.0);
  front_perception_point.set_y(4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle6(
      "06", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle6);

  uint32_t sequence_num = 1;
  LocalView local_view;
  Frame frame(sequence_num, local_view, start_point, vehicle_state);
  for (auto &ptr : obstacles) {
    frame.AddObstacleRegionType(*ptr, local_route);
    frame.AddObstacle(*ptr);
  }

  EXPECT_EQ(frame.obstacles().size(), 6);
  EXPECT_TRUE(frame.Find("01")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::LEFT_FRONT);
  EXPECT_TRUE(frame.Find("02")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::LEFT_FRONT);
  EXPECT_TRUE(frame.Find("04")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::LEFT);
  EXPECT_TRUE(frame.Find("05")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::LEFT_REAR);
}

TEST(FrameTest, TestRightLaneObstacleFilter) {
  LocalRouteConfig local_route_config;
  SetConfigFile(local_route_config);
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(local_route_points);

  LocalRoute local_route(local_route_points, local_route_config);
  common::VehicleState vehicle_state;
  ::common::TrajectoryPoint start_point;
  start_point.mutable_path_point()->set_x(50.1);
  start_point.mutable_path_point()->set_y(0.01);
  start_point.mutable_path_point()->set_theta(0.0);

  std::vector<Obstacle *> obstacles;
  perception::PerceptionObstacle front_perception_obstacle;
  PathPoint front_perception_point;
  front_perception_point.set_x(100.0);
  front_perception_point.set_y(-4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle(
      "01", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle);

  front_perception_point.set_x(200.0);
  front_perception_point.set_y(-4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle2(
      "02", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle2);

  front_perception_point.set_x(300.0);
  front_perception_point.set_y(-4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle3(
      "03", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle3);

  front_perception_point.set_x(50.0);
  front_perception_point.set_y(-4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle4(
      "04", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle4);

  front_perception_point.set_x(25.0);
  front_perception_point.set_y(-4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle5(
      "05", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle5);

  front_perception_point.set_x(-15.0);
  front_perception_point.set_y(-4.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle6(
      "06", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle6);

  uint32_t sequence_num = 1;
  LocalView local_view;
  Frame frame(sequence_num, local_view, start_point, vehicle_state);
  for (auto &ptr : obstacles) {
    frame.AddObstacleRegionType(*ptr, local_route);
    frame.AddObstacle(*ptr);
  }
  EXPECT_EQ(frame.obstacles().size(), 6);
  EXPECT_TRUE(frame.Find("01")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::RIGHT_FRONT);
  EXPECT_TRUE(frame.Find("02")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::RIGHT_FRONT);
  EXPECT_TRUE(frame.Find("04")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::RIGHT);
  EXPECT_TRUE(frame.Find("05")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::RIGHT_REAR);
}

TEST(FrameTest, TestLeftLeftLaneObstacleFilter) {
  LocalRouteConfig local_route_config;
  SetConfigFile(local_route_config);
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(local_route_points);

  LocalRoute local_route(local_route_points, local_route_config);
  common::VehicleState vehicle_state;
  ::common::TrajectoryPoint start_point;
  start_point.mutable_path_point()->set_x(50.1);
  start_point.mutable_path_point()->set_y(0.01);
  start_point.mutable_path_point()->set_theta(0.0);

  std::vector<Obstacle *> obstacles;
  perception::PerceptionObstacle front_perception_obstacle;
  PathPoint front_perception_point;
  front_perception_point.set_x(100.0);
  front_perception_point.set_y(8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle(
      "01", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle);

  front_perception_point.set_x(200.0);
  front_perception_point.set_y(8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle2(
      "02", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle2);

  front_perception_point.set_x(300.0);
  front_perception_point.set_y(8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle3(
      "03", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle3);

  front_perception_point.set_x(50.0);
  front_perception_point.set_y(8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle4(
      "04", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle4);

  front_perception_point.set_x(25.0);
  front_perception_point.set_y(8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle5(
      "05", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle5);

  uint32_t sequence_num = 1;
  LocalView local_view;
  Frame frame(sequence_num, local_view, start_point, vehicle_state);
  for (auto &ptr : obstacles) {
    frame.AddObstacleRegionType(*ptr, local_route);
    frame.AddObstacle(*ptr);
  }
  EXPECT_EQ(frame.obstacles().size(), 5);
  EXPECT_TRUE(frame.Find("01")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::LEFT_LEFT_FRONT);
  EXPECT_TRUE(frame.Find("02")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::LEFT_LEFT_FRONT);
  EXPECT_TRUE(frame.Find("04")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::LEFT_LEFT_FRONT);
}

TEST(FrameTest, TestRightRightLaneObstacleFilter) {
  LocalRouteConfig local_route_config;
  SetConfigFile(local_route_config);
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(local_route_points);

  LocalRoute local_route(local_route_points, local_route_config);
  common::VehicleState vehicle_state;
  ::common::TrajectoryPoint start_point;
  start_point.mutable_path_point()->set_x(50.1);
  start_point.mutable_path_point()->set_y(0.01);
  start_point.mutable_path_point()->set_theta(0.0);

  std::vector<Obstacle *> obstacles;
  perception::PerceptionObstacle front_perception_obstacle;
  PathPoint front_perception_point;
  front_perception_point.set_x(100.0);
  front_perception_point.set_y(-8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle(
      "01", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle);

  front_perception_point.set_x(200.0);
  front_perception_point.set_y(-8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle2(
      "02", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle2);

  front_perception_point.set_x(300.0);
  front_perception_point.set_y(-8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle3(
      "03", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle3);

  front_perception_point.set_x(50.0);
  front_perception_point.set_y(-8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle4(
      "04", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle4);

  front_perception_point.set_x(25.0);
  front_perception_point.set_y(-8.0);
  front_perception_point.set_z(0.0);
  front_perception_point.set_theta(0.0);
  SetPerceptionObstacleInfo(front_perception_obstacle, front_perception_point,
                            5.2, 2.4, ::common::Point3D(20.0, 0.0, 0.0));
  Obstacle front_obstacle5(
      "05", front_perception_obstacle,
      GenerateObstacleTrajectory(20.0, front_perception_point));
  obstacles.emplace_back(&front_obstacle5);

  uint32_t sequence_num = 1;
  LocalView local_view;
  Frame frame(sequence_num, local_view, start_point, vehicle_state);
  for (auto &ptr : obstacles) {
    frame.AddObstacleRegionType(*ptr, local_route);
    frame.AddObstacle(*ptr);
  }
  EXPECT_EQ(frame.obstacles().size(), 5);
  EXPECT_TRUE(frame.Find("01")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::RIGHT_RIGHT_FRONT);
  EXPECT_TRUE(frame.Find("02")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::RIGHT_RIGHT_FRONT);
  EXPECT_TRUE(frame.Find("03")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::IDLE_TYPE);
  EXPECT_TRUE(frame.Find("04")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::RIGHT_RIGHT_FRONT);
  EXPECT_TRUE(frame.Find("05")->RelativeRegionToLocalRoute() ==
              RelativeRegionType::IDLE_TYPE);
}

}  // namespace planning
}  // namespace zark
