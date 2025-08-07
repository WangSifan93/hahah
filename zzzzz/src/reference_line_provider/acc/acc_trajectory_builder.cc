/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file acc_trajectory_builder.cc
 **/

#include "acc_trajectory_builder.h"

namespace zark {
namespace planning {

ACCTrajectoryBuilder::ACCTrajectoryBuilder(
    const LocalRouteConfig &local_route_config)
    : local_route_config_(local_route_config) {}

void ACCTrajectoryBuilder::GenerateLocalRoutePoints(
    std::vector<LocalRoutePoint> &local_route_points, const double x,
    const double y, const double theta, const double kappa, const double steer,
    const double v, const double t_total) {
  // To to simplify the function to generate the route points
  double wheel_base =
      common::VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  double time_interval = 0.1;      // Time interval between each point
  double steer_threshold = 0.001;  // Steering threshold
  double R = std::numeric_limits<double>::infinity();
  double x_c = 0;
  double y_c = 0;
  double theta_now = theta;
  double virtual_v = v;

  // Avoid the zero speed
  if (virtual_v < 2) virtual_v = 2;

  if (std::abs(steer) >= steer_threshold) {
    R = wheel_base / tan(steer);
    // Calculate the parameters of the circle
    x_c = x - R * sin(theta);
    y_c = y + R * cos(theta);
  }

  ADEBUG << "The Parameters of initial point is " << "x " << x << " y "
         << " steer " << steer << " theta " << theta << " v " << v;
  ADEBUG << "The parameters of a circle is x_c " << x_c << ", y_c " << y_c
         << ", R " << R;

  double history_time = -3;
  double theta_new =
      UpdateHeading(theta_now, steer, virtual_v, wheel_base, history_time);
  ADEBUG << "Heading angle change of backward trajectory is theta_new: "
         << theta_new;
  theta_now = theta_new;

  // Define x_new and y_new to calculate the new coordinates on the circular or
  // straight path
  double x_new = x;
  double y_new = y;

  for (double t = history_time; t <= t_total; t += time_interval) {
    // Update the new coordinates on the circular or straight path
    if (std::abs(steer) < steer_threshold) {
      x_new = x + virtual_v * cos(theta_now) * t;
      y_new = y + virtual_v * sin(theta_now) * t;
    } else {
      x_new = x_c + R * sin(theta_now);
      y_new = y_c - R * cos(theta_now);
    }
    // Update the theta value based on the the parameters of car and the steer
    // direction
    double theta_new =
        UpdateHeading(theta_now, steer, virtual_v, wheel_base, time_interval);
    theta_now = theta_new;

    // Add the new point to the route points vector
    local_route_points.emplace_back(LocalRoutePoint(
        hdmap::MapPathPoint(::math::Vec2d(x_new, y_new), theta_now), kappa,
        0.0));  // With constant curvature, dkappa = 0
    if (fmod(history_time, 10 * time_interval) == 0) {
      ADEBUG << "Forward estimated heading angle change is theta_new: "
             << theta_new;
      ADEBUG << "Coordinates at time t " << t << " x_new " << x_new << " y_new "
             << y_new;
    }
  }
}

LocalRoute ACCTrajectoryBuilder::GenerateLocalRouteForACC(
    const ::common::TrajectoryPoint &planning_start_point) {
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(
      local_route_points, planning_start_point.path_point().x(),
      planning_start_point.path_point().y(),
      planning_start_point.path_point().theta(),
      planning_start_point.path_point().kappa(), planning_start_point.steer(),
      planning_start_point.v(),
      local_route_config_.acc_route_config.acc_trajectory_builder_config()
          .t_total());

  AINFO << " local_route_points.size() " << local_route_points.size();
  common::VehicleState vehicle_state;

  LocalRoute local_route(local_route_points, local_route_config_);
  local_route.Init(vehicle_state, planning_start_point);
  hdmap::RouteSegments lanes;
  lanes.SetSegmentType(hdmap::RouteSegments::SegmentType::CurrentSegment);
  local_route.SetLanes(lanes);
  return local_route;
}

double ACCTrajectoryBuilder::UpdateHeading(const double theta,
                                           const double steer, const double v,
                                           const double wheelbase,
                                           const double deltaTime) {
  double heading_change_rate;
  if (std::abs(steer) > 0.001) {
    heading_change_rate = (v / wheelbase) * std::tan(steer);
  } else {
    heading_change_rate = 0;
  }
  ADEBUG << "heading_change_rate " << heading_change_rate;
  double newHeading = theta;
  newHeading = theta + heading_change_rate * deltaTime;
  return newHeading;
}

}  // namespace planning
}  // namespace zark
