#include "decision/longitudinal/speed_limit_builder.h"

#include <gtest/gtest.h>

#include "common/local_route/local_route.h"
#include "common/local_route/local_route_point.h"
#include "vec2d.h"
#include "common/speed/st_boundary.h"
#include "common/speed_limit.h"
#include "reference_line_provider/pnc_map/path.h"
#include "planning_msgs/task_config.h"

using namespace zark::planning;

TEST(SpeedLimitBuilderTest, basic_test) {
  CorridorPoint corridor_point_1;
  CorridorPoint corridor_point_2;
  corridor_point_1.s = 1.0;
  corridor_point_1.kappa = 0.01;
  corridor_point_2.s = 2.0;
  corridor_point_2.kappa = 0.02;
  Corridor corridor;
  corridor.emplace_back(corridor_point_1);
  corridor.emplace_back(corridor_point_2);

  LongitudinalDeciderConfig::SpeedLimitConfig config;
  config.a_lat_max = 1.0;
  config.v_max = 10.0;

  ::math::Vec2d point_1(0.0, 2.0);
  ::math::Vec2d point_2(1.0, 5.0);
  hdmap::MapPathPoint map_path_point_1(point_1, 0.0);
  hdmap::MapPathPoint map_path_point_2(point_2, 0.0);
  LocalRoutePoint local_route_point_1(map_path_point_1, 0.1, 0.1);
  LocalRoutePoint local_route_point_2(map_path_point_2, 0.1, 0.1);
  std::vector<LocalRoutePoint> local_route_points;
  local_route_points.emplace_back(local_route_point_1);
  local_route_points.emplace_back(local_route_point_2);
  LocalRoute local_route(local_route_points, LocalRouteConfig());

  SpeedLimitBuilder speed_limit_builder_test = SpeedLimitBuilder(config);
  // TODO
  // std::unordered_map<std::string, SpeedLimit> speed_limit_map =
  //     speed_limit_builder_test.BuildSpeedLimitMap(&local_route,
  //                                                 corridor);
  // double speed_limit = speed_limit_map["final"].GetSpeedLimitByS(1.0);
  // EXPECT_DOUBLE_EQ(speed_limit, 10.0);
}
