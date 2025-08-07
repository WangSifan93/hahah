/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_mpc_problem_test.cc
 **/

#include "apps/planning/src/common/math/mpc/lateral_mpc_problem.h"
#include "apps/planning/src/motion/lateral/lateral_constraint.h"
#include "apps/planning/src/config/config_main.h"

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/math/mpc/mpc_problem.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

namespace {
const double C_alpha_f = 4016;
const double C_alpha_r = 4016;
const double mass = 1971.5;
const double wheel_base = 2.8448;
const double rear_axle_to_cg = 1.3066;
const double l_f = wheel_base - rear_axle_to_cg;
const double l_r = rear_axle_to_cg;
const double I_z = 5156;
const double dt = 0.1;
const double w_terminal = 10.0;
const double v = 1.0;

const int n_steps = 40;
const int n_nodes = n_steps + 1;
const double kLMax = 100.0;
const double kTol = 1.0e-3;
const int n_x = 4;
const int n_u = 1;
const int n_u_dot = 1;

Eigen::RowVectorXd psi_dot_des = Eigen::RowVectorXd::Zero(n_steps + 1);
Eigen::RowVectorXd lon_speed = Eigen::RowVectorXd::Ones(n_steps + 1);
Eigen::RowVectorXd lon_speed_zero = Eigen::RowVectorXd::Zero(n_steps + 1);
Eigen::VectorXd Q = Eigen::VectorXd::Ones(n_x);
Eigen::VectorXd R = Eigen::VectorXd::Ones(n_u);
Eigen::VectorXd R_dot = Eigen::VectorXd::Ones(n_u);
}  // namespace

class LateralMPCProblemTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    lat_mpc_ = std::unique_ptr<LateralMPCProblem>(new LateralMPCProblem(
        n_steps, n_x, n_u, n_u_dot, Q, R, R_dot, dt, w_terminal));
    lat_mpc_->SetV(lon_speed);
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "lateral_optimizer_config_test.json";
    Config config{config_file};
    std::ifstream jdata(config_file);
    lateral_config_ =
        config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
    lat_con_ = std::unique_ptr<LateralConstraint>(
        new LateralConstraint(lateral_config_));

    common::VehicleConfig vehicle_config;
    vehicle_param_.set_front_wheel_corner_stiffness(C_alpha_f);
    vehicle_param_.set_rear_wheel_corner_stiffness(C_alpha_r);
    vehicle_param_.set_mass(mass);
    vehicle_param_.set_rear_axle_to_cg(l_r);
    vehicle_param_.set_moment_of_inertia(I_z);
    vehicle_param_.set_wheel_base(wheel_base);
    vehicle_config.set_vehicle_param(vehicle_param_);
    common::VehicleConfigHelper::Init(vehicle_config);
  }

  std::unique_ptr<LateralMPCProblem> lat_mpc_;
  common::VehicleParam vehicle_param_;
  LateralOptimizerConfig lateral_config_;
  std::unique_ptr<LateralConstraint> lat_con_;
};

void ConstructXandURef(const Eigen::VectorXd &x_init, const double dt,
                       Eigen::MatrixXd &x_ref, Eigen::MatrixXd &u_ref) {
  const int n_steps = 40;
  const int n_nodes = n_steps + 1;
  x_ref.resize(4, n_nodes);
  x_ref = Eigen::MatrixXd::Zero(4, n_nodes);
  u_ref.resize(1, n_steps);
  u_ref = Eigen::MatrixXd::Zero(1, n_steps);
}


TEST_F(LateralMPCProblemTest, TestLaterSolve) {
  Eigen::VectorXd Q(4);
  Q << 1.0, 0.0, 1.0, 0.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  double dt = 0.2;
  Eigen::VectorXd x_init(4);
  x_init << 0.5, 0.0, 0.0, 0.0;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  Eigen::MatrixXd u_2 = Eigen::MatrixXd::Ones(1, 40);

  ConstructXandURef(x_init, dt, x_ref, u_ref);
  LateralConstraint::Constraints constraints;
  MPCData::Constraints mpc_constraint;
  mpc_constraint = lat_con_->ConvertToMPCConstraints(constraints);
  MPCData mpc_data;
  lat_mpc_->Solve(x_init, x_ref, u_ref, u_2, mpc_constraint, u_prev, dt_prev,
                  mpc_data);

  EXPECT_EQ(mpc_data.x.rows(), n_x);
  EXPECT_EQ(mpc_data.x.cols(), n_nodes);
  EXPECT_EQ(mpc_data.u.rows(), n_u);
  EXPECT_EQ(mpc_data.u.cols(), n_steps);
  EXPECT_EQ(mpc_data.u_dot.rows(), n_u);
  EXPECT_EQ(mpc_data.u_dot.cols(), n_steps);
  EXPECT_EQ(mpc_data.slacks.soft_min.x.rows(), n_x);
  EXPECT_EQ(mpc_data.slacks.soft_min.u.rows(), n_u);
  EXPECT_EQ(mpc_data.slacks.soft_min.u_dot.rows(), n_u_dot);
  EXPECT_EQ(mpc_data.slacks.soft_max.x.rows(), n_x);
  EXPECT_EQ(mpc_data.slacks.soft_max.u.rows(), n_u);
  EXPECT_EQ(mpc_data.slacks.soft_max.u.cols(), n_steps);
  EXPECT_EQ(mpc_data.slacks.soft_max.u_dot.rows(), n_u);
  EXPECT_EQ(mpc_data.slacks.soft_max.u_dot.cols(), n_steps);
  EXPECT_EQ(mpc_data.slacks.stiff_min.x.rows(), n_x);
  EXPECT_EQ(mpc_data.slacks.stiff_min.x.cols(), n_nodes);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u.rows(), n_u);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u.cols(), n_steps);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u_dot.rows(), n_u);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u_dot.cols(), n_steps);
  EXPECT_EQ(mpc_data.slacks.stiff_max.x.rows(), n_x);
  EXPECT_EQ(mpc_data.slacks.stiff_max.x.cols(), n_nodes);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u.rows(), n_u);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u.cols(), n_steps);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u_dot.rows(), n_u);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u_dot.cols(), n_steps);
}

TEST_F(LateralMPCProblemTest, TESTComputeStateSpaceModel) {
  std::vector<Eigen::MatrixXd> A_d_expected, B_d_expected, B_d2_expected;
  std::vector<Eigen::MatrixXd> A_d_expected_zero, B_d_expected_zero,
      B_d2_expected_zero;

  A_d_expected.resize(n_steps);
  B_d_expected.resize(n_steps);
  B_d2_expected.resize(n_steps);
  A_d_expected_zero.resize(n_steps);
  B_d_expected_zero.resize(n_steps);
  B_d2_expected_zero.resize(n_steps);

  std::tie(A_d_expected, B_d_expected, B_d2_expected) =
      lat_mpc_->ComputeStateSpaceModel();
  std::tie(A_d_expected_zero, B_d_expected_zero, B_d2_expected_zero) =
      lat_mpc_->ComputeStateSpaceModel();

  EXPECT_EQ(A_d_expected.size(), n_steps);
  EXPECT_EQ(B_d_expected.size(), n_steps);
  EXPECT_EQ(B_d2_expected.size(), n_steps);
  EXPECT_EQ(A_d_expected[0].rows(), n_x);
  EXPECT_EQ(A_d_expected[0].cols(), n_x);
  EXPECT_EQ(A_d_expected[1].rows(), n_x);
  EXPECT_EQ(A_d_expected[1].cols(), n_x);
  EXPECT_EQ(B_d_expected[0].rows(), n_x);
  EXPECT_EQ(B_d_expected[0].cols(), n_u);
  EXPECT_EQ(B_d2_expected[0].rows(), n_x);
  EXPECT_EQ(B_d2_expected[0].cols(), n_u);

  // set speed zero
  EXPECT_EQ(A_d_expected_zero.size(), n_steps);
  EXPECT_EQ(B_d_expected_zero.size(), n_steps);
  EXPECT_EQ(B_d2_expected_zero.size(), n_steps);
  EXPECT_EQ(A_d_expected_zero[0].rows(), n_x);
  EXPECT_EQ(A_d_expected_zero[0].cols(), n_x);
  EXPECT_EQ(A_d_expected_zero[1].rows(), n_x);
  EXPECT_EQ(A_d_expected_zero[1].cols(), n_x);
  EXPECT_EQ(B_d_expected_zero[0].rows(), n_x);
  EXPECT_EQ(B_d_expected_zero[0].cols(), n_u);
  EXPECT_EQ(B_d2_expected_zero[0].rows(), n_x);
  EXPECT_EQ(B_d2_expected_zero[0].cols(), n_u);
}

TEST_F(LateralMPCProblemTest, TESTLateralModel) {
  for (int k = 0; k < 1; ++k) {
    Eigen::MatrixXd A_d_expected_elem, B_d_expected_elem, B_d2_expected_elem;
    Eigen::MatrixXd A_d_expected_elem_zero, B_d_expected_elem_zero,
        B_d2_expected_elem_zero;
    A_d_expected_elem.resize(n_x, n_x);
    B_d_expected_elem.resize(n_x, n_u);
    B_d2_expected_elem.resize(n_x, n_u_dot);
    A_d_expected_elem_zero.resize(n_x, n_x);
    B_d_expected_elem_zero.resize(n_x, n_u);
    B_d2_expected_elem_zero.resize(n_x, n_u_dot);

    std::tie(A_d_expected_elem, B_d_expected_elem, B_d2_expected_elem) =
        lat_mpc_->LateralModel(C_alpha_f, C_alpha_r, l_f, l_r, mass, I_z, dt,
                               lon_speed[k]);
    std::tie(A_d_expected_elem_zero, B_d_expected_elem_zero,
             B_d2_expected_elem_zero) =
        lat_mpc_->LateralModel(C_alpha_f, C_alpha_r, l_f, l_r, mass, I_z, dt,
                               lon_speed_zero[k]);

    EXPECT_NEAR(A_d_expected_elem(0, 0), 1, kTol);
    EXPECT_NEAR(A_d_expected_elem(0, 1), 0.0831, kTol);
    EXPECT_NEAR(A_d_expected_elem(0, 2), 0.0169, kTol);
    EXPECT_NEAR(A_d_expected_elem(0, 3), -0.0010, kTol);

    EXPECT_NEAR(A_d_expected_elem(1, 0), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem(1, 1), 0.661683641436677, kTol);
    EXPECT_NEAR(A_d_expected_elem(1, 2), 0.3383, kTol);
    EXPECT_NEAR(A_d_expected_elem(1, 3), -0.0192, kTol);

    EXPECT_NEAR(A_d_expected_elem(2, 0), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem(2, 1), -0.0006, kTol);
    EXPECT_NEAR(A_d_expected_elem(2, 2), 1.000646065989419, kTol);
    EXPECT_NEAR(A_d_expected_elem(2, 3), 0.0864, kTol);

    EXPECT_NEAR(A_d_expected_elem(3, 0), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem(3, 1), -0.0129, kTol);
    EXPECT_NEAR(A_d_expected_elem(3, 2), 0.0129, kTol);
    EXPECT_NEAR(A_d_expected_elem(3, 3), 0.727405090818950, kTol);

    EXPECT_NEAR(B_d_expected_elem(0, 0), 0.0042, kTol);
    EXPECT_NEAR(B_d_expected_elem(1, 0), 0.0840, kTol);
    EXPECT_NEAR(B_d_expected_elem(2, 0), 0.0025, kTol);
    EXPECT_NEAR(B_d_expected_elem(3, 0), 0.0510, kTol);

    EXPECT_NEAR(B_d2_expected_elem(0, 0), -0.0030, kTol);
    EXPECT_NEAR(B_d2_expected_elem(1, 0), -0.0596, kTol);
    EXPECT_NEAR(B_d2_expected_elem(2, 0), -0.0068, kTol);
    EXPECT_NEAR(B_d2_expected_elem(3, 0), -0.1363, kTol);

    // set speed zero
    EXPECT_NEAR(A_d_expected_elem_zero(0, 0), 1, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(0, 1), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(0, 2), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(0, 3), 0.0, kTol);

    EXPECT_NEAR(A_d_expected_elem_zero(1, 0), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(1, 1), -1, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(1, 2), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(1, 3), 0.0, kTol);

    EXPECT_NEAR(A_d_expected_elem_zero(2, 0), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(2, 1), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(2, 2), 1, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(2, 3), 0.0, kTol);

    EXPECT_NEAR(A_d_expected_elem_zero(3, 0), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(3, 1), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(3, 2), 0.0, kTol);
    EXPECT_NEAR(A_d_expected_elem_zero(3, 3), -1, kTol);

    EXPECT_NEAR(B_d_expected_elem_zero(0, 0), 0.0, kTol);
    EXPECT_NEAR(B_d_expected_elem_zero(1, 0), 0.0, kTol);
    EXPECT_NEAR(B_d_expected_elem_zero(2, 0), 0.0, kTol);
    EXPECT_NEAR(B_d_expected_elem_zero(3, 0), 0.0, kTol);

    EXPECT_NEAR(B_d2_expected_elem_zero(0, 0), 0.0, kTol);
    EXPECT_NEAR(B_d2_expected_elem_zero(1, 0), 0.0, kTol);
    EXPECT_NEAR(B_d2_expected_elem_zero(2, 0), -0.05, kTol);
    EXPECT_NEAR(B_d2_expected_elem_zero(3, 0), -1, kTol);
  }
}

}  // namespace planning
}  // namespace zark
