/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file local_route_test.cc
 **/

#include "apps/planning/src/common/conversion.h"
#include "apps/planning/src/common/local_route/local_route.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

void GenerateLocalRoutePoints(
    std::vector<LocalRoutePoint> &local_route_points) {
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

TEST(LocalRouteTest, TestCalculateCenterBox) {
  common::VehicleParam param;
  param.set_length(4.2);
  param.set_width(2.0);
  param.set_rear_axle_to_cg(1.7);
  param.set_back_edge_to_center(0.5);
  ::math::Box2d box =
      common::CalculateCenterBoxFromCGPose(1.0, 0.0, 0.0, param);
  auto corners = box.GetAllCorners();
  ::math::Vec2d center_point(0.9, 0.0);
  ::math::Vec2d corner_point1(3.0, -1.0);
  EXPECT_TRUE(box.center().DistanceTo(center_point) < 0.001);
  EXPECT_TRUE(corners.begin()->DistanceTo(corner_point1) < 0.001);
}

TEST(LocalRouteTest, TestGetEndDistanceS) {
  std::vector<LocalRoutePoint> points;
  GenerateLocalRoutePoints(points);
  auto local_route = LocalRoute(points, LocalRouteConfig());
  EXPECT_DOUBLE_EQ(local_route.GetEndS(), 299.5);
  local_route.SetDestinationS(50.0);
  EXPECT_DOUBLE_EQ(local_route.GetEndS(), 299.5);
}

}  // namespace planning
}  // namespace zark
