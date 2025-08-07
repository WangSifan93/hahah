
#include "apps/planning/src/common/math/mpc/longitudinal_mpc_problem.h"

#include "gtest/gtest.h"

using namespace std;
namespace zark {
namespace planning {

TEST(LongitudinalMPCProblemTest, TestComputeStateSpaceModel) {
  const int n_x = 2;
  const double dt = 0.1;
  Eigen::VectorXd Q, R, R_dot;
  LongitudinalMPCProblem lon_mpc_problem(4, n_x, 1, 1, Q, R, R_dot, dt);
  std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>,
             std::vector<Eigen::MatrixXd>>
      lon_mpc_model = lon_mpc_problem.ComputeStateSpaceModel();

  Eigen::MatrixXd expected_A_d(n_x, n_x);
  expected_A_d(0, 0) = 1;
  expected_A_d(0, 1) = 0.1;
  expected_A_d(1, 0) = 0;
  expected_A_d(1, 1) = 1;
  Eigen::MatrixXd expected_B_d(n_x, 1);
  expected_B_d(0, 0) = 0;
  expected_B_d(1, 0) = 0.1;
  EXPECT_DOUBLE_EQ(expected_A_d(0, 0), std::get<0>(lon_mpc_model).at(0)(0, 0));
  EXPECT_DOUBLE_EQ(expected_A_d(0, 1), std::get<0>(lon_mpc_model).at(0)(0, 1));
  EXPECT_DOUBLE_EQ(expected_A_d(1, 0), std::get<0>(lon_mpc_model).at(0)(1, 0));
  EXPECT_DOUBLE_EQ(expected_A_d(1, 1), std::get<0>(lon_mpc_model).at(0)(1, 1));
  EXPECT_DOUBLE_EQ(expected_B_d(0, 0), std::get<1>(lon_mpc_model).at(0)(0, 0));
  EXPECT_DOUBLE_EQ(expected_B_d(1, 0), std::get<1>(lon_mpc_model).at(0)(1, 0));
  EXPECT_DOUBLE_EQ(0, std::get<2>(lon_mpc_model).size());
}

}  // namespace planning
}  // namespace zark
