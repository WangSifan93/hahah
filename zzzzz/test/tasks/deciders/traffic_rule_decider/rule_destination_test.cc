/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file rule_destination_test.cc
 **/

#include "gtest/gtest.h"
#define private public
#include "apps/planning/src/tasks/deciders/traffic_rule_decider/rule_destination.h"

namespace zark {
namespace planning {

void GenerateLocalRoutePoints(
    std::vector<LocalRoutePoint>& local_route_points) {
  for (double i = 0.0; i < 300.0; i = i + 0.5) {
    double x = i;
    double y = 0.0;
    double heading = 0.0;
    double kappa = 0.0;
    double dkappa = 0.0;
    local_route_points.emplace_back(LocalRoutePoint(
        hdmap::MapPathPoint(::math::Vec2d(x, y), heading), kappa, dkappa));
  }
}

}  // namespace planning
}  // namespace zark
