/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file quartic_polynomial_curve1d_test.cc
 **/

#include "apps/planning/src/common/math/curve1d/quartic_polynomial_curve1d.h"
#include "apps/planning/src/common/math/curve1d/quintic_polynomial_curve1d.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

TEST(QuarticPolynomialCurve1dTest, Evaluate) {
  {
    double x0 = 0.0;
    double dx0 = 0.0;
    double ddx0 = 0.0;
    double dx1 = 10.0;
    double ddx1 = 1.0;
    double param = 8.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(2, 0.0), 1e-8);
  }
  {
    double x0 = 0.0;
    double dx0 = 0.0;
    double ddx0 = 0.0;
    double dx1 = 5.0;
    double ddx1 = 1.0;
    double param = 3.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(2, 0.0), 1e-8);
  }

  {
    double x0 = 1.0;
    double dx0 = 2.0;
    double ddx0 = 3.0;
    double dx1 = 5.0;
    double ddx1 = 1.0;
    double param = 3.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(x0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(dx0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(ddx0, curve.Evaluate(2, 0.0), 1e-8);
  }
}

TEST(QuarticPolynomialCurve1dTest, DerivedFromQuinticCurve) {
  QuinticPolynomialCurve1d quintic_curve(1, 2, 3, 2, 1, 2, 5);
  QuarticPolynomialCurve1d quartic_curve;
  quartic_curve.DerivedFromQuinticCurve(quintic_curve);
  for (double value = 0.0; value < 5.1; value += 1) {
    EXPECT_NEAR(quartic_curve.Evaluate(0, value),
                quintic_curve.Evaluate(1, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(1, value),
                quintic_curve.Evaluate(2, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(2, value),
                quintic_curve.Evaluate(3, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(3, value),
                quintic_curve.Evaluate(4, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(4, value),
                quintic_curve.Evaluate(5, value), 1e-8);
  }
}

TEST(QuarticPolynomialCurve1dTest, FitWithEndPointFirstOrder) {
  QuarticPolynomialCurve1d quartic_curve(1, 2, 4, 2, 1, 3);
  quartic_curve.FitWithEndPointFirstOrder(2, 3, 2, 1, 2, 5);
  EXPECT_NEAR(quartic_curve.Evaluate(0, 0), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(1, 0), 3, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(2, 0), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(0, 5), 1, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(1, 5), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.ParamLength(), 5, 1e-8);
}

TEST(QuarticPolynomialCurve1dTest, FitWithEndPointSecondOrder) {
  QuarticPolynomialCurve1d quartic_curve(1, 2, 4, 2, 1, 3);
  quartic_curve.FitWithEndPointSecondOrder(2, 7, 2, 6, 2, 8);
  EXPECT_NEAR(quartic_curve.Evaluate(0, 0), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(1, 0), 7, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(0, 8), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(1, 8), 6, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(2, 8), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.ParamLength(), 8, 1e-8);
}
}  // namespace planning
}  // namespace zark
