#include "apps/planning/src/planning_msgs/sl_boundary.h"

#include "gtest/gtest.h"
namespace zark {
namespace planning {

TEST(SLBoundaryTest, TestEvaluateByT) {
  SLTrajectory sl_trajectory;
  SLBoundary sl_boundary_0, sl_boundary_1;
  sl_boundary_0.set_start_s(-5.0);
  sl_boundary_0.set_end_s(0.0);
  sl_boundary_0.set_start_l(0.0);
  sl_boundary_0.set_end_l(0.5);
  sl_boundary_0.set_t(0.0);
  sl_trajectory.emplace_back(sl_boundary_0);

  sl_boundary_1.set_start_s(0.0);
  sl_boundary_1.set_end_s(5.0);
  sl_boundary_1.set_start_l(0.5);
  sl_boundary_1.set_end_l(1.0);
  sl_boundary_1.set_t(1.0);
  sl_trajectory.emplace_back(sl_boundary_1);
  const double t = 0.5;
  const SLBoundary sl_boundary = sl_trajectory.EvaluateByT(t);

  EXPECT_NEAR(sl_boundary.start_s(), -2.5, 0.1);
  EXPECT_NEAR(sl_boundary.end_s(), 2.5, 0.1);
  EXPECT_NEAR(sl_boundary.start_l(), 0.25, 0.1);
  EXPECT_NEAR(sl_boundary.end_l(), 0.75, 0.1);
}

}  // namespace planning
}  // namespace zark
