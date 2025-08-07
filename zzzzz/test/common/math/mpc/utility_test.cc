/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file utility_test.cc
 **/

#include "apps/planning/src/common/math/mpc/utility.h"

#include <iostream>

#include "gtest/gtest.h"

namespace zark {
namespace planning {

TEST(MPCUtilityTest, TestTVector) {
  const double dt = 0.1;
  const int n = 4;
  Eigen::RowVectorXd t = util::TVector(dt, n);
  EXPECT_DOUBLE_EQ(t(0), 0.0);
  EXPECT_DOUBLE_EQ(t(1), 0.1);
  EXPECT_DOUBLE_EQ(t(2), 0.2);
  EXPECT_DOUBLE_EQ(t(3), 0.3);
}

TEST(MPCUtilityTest, TestConvertToTimeString) {
  const double dt = 0.1;
  const std::string expected_str = "t=0.1s";
  EXPECT_EQ(util::ConvertToTimeString(dt), expected_str);
}

}  // namespace planning
}  // namespace zark
