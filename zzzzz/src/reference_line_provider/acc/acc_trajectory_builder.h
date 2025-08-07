/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file acc_trajectory_builder.h
 **/

#pragma once

#include <unordered_map>
#include <memory>
#include <vector>
#include <cmath>

#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/planning_msgs/acc_route_config.h"

namespace zark {
namespace planning {

class ACCTrajectoryBuilder {
 public:
  explicit ACCTrajectoryBuilder(const LocalRouteConfig &config);

  LocalRoute GenerateLocalRouteForACC(
      const ::common::TrajectoryPoint &planning_start_point);

 private:
  void GenerateLocalRoutePoints(std::vector<LocalRoutePoint> &local_route,
                                const double x, const double y,
                                const double theta, const double kappa,
                                const double steer, const double v,
                                const double t_total);

  double UpdateHeading(const double theta, const double steer, const double v,
                       const double wheelbase, const double deltaTime);

  LocalRouteConfig local_route_config_;
};

}  // namespace planning
}  // namespace zark
