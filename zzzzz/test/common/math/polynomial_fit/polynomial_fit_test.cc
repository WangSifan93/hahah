/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file lane_change_evaluator_test.cc
 **/

#include "apps/planning/src/common/math/polynomial_fit/polynomial_fit.h"
#include "gtest/gtest.h"
#include <math.h>

namespace zark {
namespace planning {

TEST(TrajPolynomialTest, TestTrajPointsPolyFit) {
  // prepare
  PolynomialFit p_fit{};
  VehicleState state{};
  state.set_x(0.0);
  state.set_y(0.0);
  state.set_heading(M_PI_2);
  const auto fit_order{3};
  zark::planning::DiscretizedTrajectory traj_points{};
  for (auto i = 0; i < fit_order + 1; i++) {
    ::common::TrajectoryPoint tp{};
    ::common::PathPoint pp{};
    pp.set_x(0);
    pp.set_y(-i * 1.0);
    pp.set_s(i * 1.0);
    tp.set_path_point(pp);
    tp.set_relative_time(0);
    traj_points.push_back(tp);
  }

  // when
  const auto& res_fit = p_fit.TrajPointsPolyFit(traj_points, state);

  // then
  EXPECT_EQ(2, res_fit.lat_coefficients.size());
  EXPECT_EQ(1, res_fit.lon_coefficients.size());
  EXPECT_NEAR(3, res_fit.length, 1e-10);
  EXPECT_TRUE(res_fit.is_valid);
}

TEST(TrajPolynomialTest, TestLocalCoordinatesToVehicleBody) {
  // prepare
  PolynomialFit p_fit{};
  VehicleState state{};
  state.set_x(0.0);
  state.set_y(0.0);
  state.set_heading(M_PI_2);
  const auto local_x{1.0};
  const auto local_y{0.0};
  auto body_x{0.0};
  auto body_y{0.0};
  // when
  p_fit.LocalCoordinatesToVehicleBody(state, local_x, local_y, body_x, body_y);

  // then
  EXPECT_NEAR(body_x, 0.0, 1e-10);
  EXPECT_NEAR(body_y, -1.0, 1e-10);
}

TEST(TrajPolynomialTest, TestLatPolyFit) {
  // prepare
  std::vector<::math::Vec2d> tmp_points{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  const auto order = 3;
  PolynomialFit::TrajPolynomial traj_poly_fit{};
  PolynomialFit polu_fit{};

  // when
  polu_fit.LatPolyFit(tmp_points, order, traj_poly_fit);

  // then
  EXPECT_EQ(traj_poly_fit.lat_coefficients.size(), 4);
  for (const auto val : traj_poly_fit.lat_coefficients) {
    EXPECT_NEAR(0.0, val, 1e-10);
  }
}

TEST(TrajPolynomialTest, TestLonPolyFit) {
  // prepare
  std::vector<double> s{0, 0, 0, 0};
  std::vector<double> t{0, 1, 2, 3};
  const auto order{3};
  PolynomialFit::TrajPolynomial traj_poly_fit{};
  PolynomialFit polu_fit{};

  // when
  polu_fit.LonPolyFit(s, t, order, traj_poly_fit);

  // then
  EXPECT_EQ(traj_poly_fit.lon_coefficients.size(), 4);
  for (const auto val : traj_poly_fit.lon_coefficients) {
    EXPECT_NEAR(0.0, val, 1e-10);
  }
}

}  // namespace planning
}  // namespace zark
