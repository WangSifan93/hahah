/******************************************************************************
 * Copyright 2024 The Zark Authors. All Rights Reserved.
 *
 *****************************************************************************/

#include "apps/planning/src/common/util/util.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {
namespace util {

TEST(UtilTest, TestIsVehicleStateValid) {
  zark::planning::common::VehicleState vehicle_state;
  vehicle_state.set_x(0.0);
  vehicle_state.set_y(1.0);
  vehicle_state.set_z(0.0);
  EXPECT_TRUE(util::IsVehicleStateValid(vehicle_state));
}

}  // namespace util
}  // namespace planning
}  // namespace zark