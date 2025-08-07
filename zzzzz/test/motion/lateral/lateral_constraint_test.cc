#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/motion/lateral/data_type.h"
#include "apps/planning/src/motion/lateral/lateral_tube.h"
#include "apps/planning/src/motion/lateral/lateral_constraint.h"
#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"
#include <iostream>

namespace zark {
namespace planning {

namespace {
const int n_steps = 40;  // same to lateral_optimizer_config_test
const int n_nodes = n_steps + 1;
const double kLMax = 100.0;
const double kTol = 1.0e-3;
const int n_x = 4;
const int n_u = 1;
const int n_u_dot = 1;
}  // namespace

void InitializeConstraints(LateralConstraint::Constraints& constraints,
                           int n_nodes,
                           const common::VehicleParam& vehicle_param) {
  auto& stiff = constraints.stiff;
  auto& soft = constraints.soft;
  soft.l_max = Eigen::RowVectorXd::Ones(n_nodes) * 2.0;
  stiff.l_max = Eigen::RowVectorXd::Ones(n_nodes) * 3.0;
  soft.l_min = Eigen::RowVectorXd::Ones(n_nodes) * -1.0;
  stiff.l_min = Eigen::RowVectorXd::Ones(n_nodes) * -4.0;

  soft.l_dot_max = Eigen::RowVectorXd::Ones(n_nodes) * 5.0;
  stiff.l_dot_max = Eigen::RowVectorXd::Ones(n_nodes) * 6.0;
  soft.l_dot_min = Eigen::RowVectorXd::Ones(n_nodes) * -7.0;
  stiff.l_dot_min = Eigen::RowVectorXd::Ones(n_nodes) * -8.0;

  const double delta_min =
      -vehicle_param.max_steer_angle() / vehicle_param.steer_ratio();
  const double delta_max =
      vehicle_param.max_steer_angle() / vehicle_param.steer_ratio();
  const double delta_dot_min =
      -vehicle_param.max_steer_angle_rate() / vehicle_param.steer_ratio();
  const double delta_dot_max =
      vehicle_param.max_steer_angle_rate() / vehicle_param.steer_ratio();

  soft.delta_min = Eigen::RowVectorXd::Ones(n_steps) * delta_min;
  soft.delta_max = Eigen::RowVectorXd::Ones(n_steps) * delta_max;
  soft.delta_dot_min = Eigen::RowVectorXd::Ones(n_steps) * delta_dot_min;
  soft.delta_dot_max = Eigen::RowVectorXd::Ones(n_steps) * delta_dot_max;
  stiff.delta_min = Eigen::RowVectorXd::Ones(n_steps) * delta_min;
  stiff.delta_max = Eigen::RowVectorXd::Ones(n_steps) * delta_max;
  stiff.delta_dot_min = Eigen::RowVectorXd::Ones(n_steps) * delta_dot_min;
  stiff.delta_dot_max = Eigen::RowVectorXd::Ones(n_steps) * delta_dot_max;
}

void InitializeTube(Tube& tube, int n_nodes) {
  tube.pts.reserve(n_nodes);
  const double time_step = 0.1;
  const double speed = 3.0;
  for (int k = 0; k < n_nodes; k++) {
    Tube::TubePoint tube_point;
    tube_point.t = time_step * k;
    tube_point.s = speed * time_step * k;
    tube_point.l_left_soft = 2.0;
    tube_point.l_right_soft = -3.0;
    tube_point.l_left_stiff = 2.0;
    tube_point.l_right_stiff = -3.0;
    tube_point.l_left_hard = 2.0;
    tube_point.l_right_hard = -3.0;
    tube.pts.emplace_back(tube_point);
  }
}
class LateralConstraintTest : public ::testing::Test {
 public:
  void SetUp() override {
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
    vehicle_param_.set_width(2.11);
    vehicle_param_.set_length(4.933);
    vehicle_param_.set_steer_ratio(16);
    vehicle_param_.set_max_steer_angle(8.20304748437);
    vehicle_param_.set_max_steer_angle_rate(8.55211);
    vehicle_config.set_vehicle_param(vehicle_param_);
    common::VehicleConfigHelper::Init(vehicle_config);
  }

 protected:
  LateralOptimizerConfig lateral_config_;
  std::unique_ptr<LateralConstraint> lat_con_;
  common::VehicleParam vehicle_param_;
  MPCData::Constraints mpc_constraints_;
};

TEST_F(LateralConstraintTest, TestAssignConstraints) {
  Tube tube;
  LateralConstraint::Constraints constraints;
  InitializeTube(tube, n_nodes);
  mpc_constraints_ = lat_con_->AssignConstraints(tube);
  EXPECT_EQ(mpc_constraints_.soft_min.x.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.x.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_min.x.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.constraint.rows(), 1);

  EXPECT_EQ(mpc_constraints_.soft_min.u.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_min.u.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.constraint.rows(), 1);

  // soft
  EXPECT_EQ(mpc_constraints_.soft_max.x.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.x.constraint.cols(), n_nodes);
  EXPECT_EQ(mpc_constraints_.soft_max.u.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.u.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.soft_max.x.weight.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.u.weight.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.weight.rows(), 1);

  // stiff
  EXPECT_EQ(mpc_constraints_.stiff_max.x.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.constraint.cols(), n_nodes);
  EXPECT_EQ(mpc_constraints_.stiff_max.u.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.u.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.weight.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.u.weight.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.weight.rows(), 1);

  EXPECT_EQ(mpc_constraints_.n_ineq_hard, 0);
  EXPECT_EQ(mpc_constraints_.n_ineq_soft, 484);
  EXPECT_EQ(mpc_constraints_.n_ineq, 484);
}

TEST_F(LateralConstraintTest, TestConvertToMPCConstraints) {
  LateralConstraint::Constraints constraints;
  InitializeConstraints(constraints, n_nodes, vehicle_param_);
  mpc_constraints_ = lat_con_->ConvertToMPCConstraints(constraints);

  // soft
  EXPECT_EQ(mpc_constraints_.soft_max.x.constraint.rows(), 2);
  EXPECT_EQ(mpc_constraints_.soft_max.x.constraint.cols(), n_nodes);
  EXPECT_EQ(mpc_constraints_.soft_max.u.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.u.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.soft_max.x.weight.rows(), 2);
  EXPECT_EQ(mpc_constraints_.soft_max.u.weight.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.weight.rows(), 1);

  EXPECT_EQ(mpc_constraints_.soft_min.x.constraint.rows(), 2);
  EXPECT_EQ(mpc_constraints_.soft_min.x.constraint.cols(), n_nodes);
  EXPECT_EQ(mpc_constraints_.soft_min.u.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_min.u.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.soft_min.u_dot.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_min.u_dot.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.soft_min.x.weight.rows(), 2);
  EXPECT_EQ(mpc_constraints_.soft_min.u.weight.rows(), 1);
  EXPECT_EQ(mpc_constraints_.soft_min.u_dot.weight.rows(), 1);

  EXPECT_EQ(mpc_constraints_.soft_max.x.idx_enabled.size(), n_x);
  EXPECT_EQ(mpc_constraints_.soft_max.u.idx_enabled.size(), n_u);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.idx_enabled.size(), n_u_dot);

  EXPECT_EQ(mpc_constraints_.soft_min.x.idx_enabled.size(), n_x);
  EXPECT_EQ(mpc_constraints_.soft_min.u.idx_enabled.size(), n_u);
  EXPECT_EQ(mpc_constraints_.soft_min.u_dot.idx_enabled.size(), n_u_dot);

  // stiff
  EXPECT_EQ(mpc_constraints_.stiff_max.x.constraint.rows(), 2);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.constraint.cols(), n_nodes);
  EXPECT_EQ(mpc_constraints_.stiff_max.u.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.u.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.weight.rows(), 2);
  EXPECT_EQ(mpc_constraints_.stiff_max.u.weight.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.weight.rows(), 1);

  EXPECT_EQ(mpc_constraints_.stiff_min.x.constraint.rows(), 2);
  EXPECT_EQ(mpc_constraints_.stiff_min.x.constraint.cols(), n_nodes);
  EXPECT_EQ(mpc_constraints_.stiff_min.u.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_min.u.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.stiff_min.u_dot.constraint.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_min.u_dot.constraint.cols(), n_steps);
  EXPECT_EQ(mpc_constraints_.stiff_min.x.weight.rows(), 2);
  EXPECT_EQ(mpc_constraints_.stiff_min.u.weight.rows(), 1);
  EXPECT_EQ(mpc_constraints_.stiff_min.u_dot.weight.rows(), 1);

  EXPECT_EQ(mpc_constraints_.stiff_max.x.idx_enabled.size(), n_x);
  EXPECT_EQ(mpc_constraints_.stiff_max.u.idx_enabled.size(), n_u);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.idx_enabled.size(), n_u_dot);

  EXPECT_EQ(mpc_constraints_.stiff_min.x.idx_enabled.size(), n_x);
  EXPECT_EQ(mpc_constraints_.stiff_min.u.idx_enabled.size(), n_u);
  EXPECT_EQ(mpc_constraints_.stiff_min.u_dot.idx_enabled.size(), n_u_dot);

  EXPECT_EQ(mpc_constraints_.n_ineq_hard, 0);
  EXPECT_EQ(mpc_constraints_.n_ineq_soft, 648);
  EXPECT_EQ(mpc_constraints_.n_ineq, 648);
}

TEST_F(LateralConstraintTest, TestUpdateMPCConstraintIndices) {
  LateralConstraint::Constraints constraints;
  InitializeConstraints(constraints, n_nodes, vehicle_param_);
  lat_con_->UpdateMPCConstraintIndices(constraints, mpc_constraints_);

  // soft constraint
  EXPECT_EQ(mpc_constraints_.soft_max.x.idx_enabled.size(), n_x);
  EXPECT_EQ(mpc_constraints_.soft_max.x.idx_enabled[0], true);
  EXPECT_EQ(mpc_constraints_.soft_max.x.idx_enabled[1], true);
  EXPECT_EQ(mpc_constraints_.soft_max.x.idx_enabled[2], false);
  EXPECT_EQ(mpc_constraints_.soft_max.x.idx_enabled[3], false);

  EXPECT_EQ(mpc_constraints_.soft_min.x.idx_enabled.size(), n_x);
  EXPECT_EQ(mpc_constraints_.soft_min.x.idx_enabled[0], true);
  EXPECT_EQ(mpc_constraints_.soft_min.x.idx_enabled[1], true);

  EXPECT_EQ(mpc_constraints_.soft_max.u.idx_enabled.size(), n_u);
  EXPECT_EQ(mpc_constraints_.soft_max.u.idx_enabled[0], true);

  EXPECT_EQ(mpc_constraints_.soft_min.u.idx_enabled.size(), n_u);
  EXPECT_EQ(mpc_constraints_.soft_min.u.idx_enabled[0], true);

  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.idx_enabled.size(), n_u_dot);
  EXPECT_EQ(mpc_constraints_.soft_max.u_dot.idx_enabled[0], true);

  EXPECT_EQ(mpc_constraints_.soft_min.u_dot.idx_enabled.size(), n_u_dot);
  EXPECT_EQ(mpc_constraints_.soft_min.u_dot.idx_enabled[0], true);

  // stiff constraint
  EXPECT_EQ(mpc_constraints_.stiff_max.x.idx_enabled.size(), n_x);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.idx_enabled[0], true);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.idx_enabled[1], true);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.idx_enabled[2], false);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.idx_enabled[3], false);

  EXPECT_EQ(mpc_constraints_.stiff_min.x.idx_enabled.size(), n_x);
  EXPECT_EQ(mpc_constraints_.stiff_min.x.idx_enabled[0], true);
  EXPECT_EQ(mpc_constraints_.stiff_min.x.idx_enabled[1], true);

  EXPECT_EQ(mpc_constraints_.stiff_max.u.idx_enabled.size(), n_u);
  EXPECT_EQ(mpc_constraints_.stiff_max.u.idx_enabled[0], true);

  EXPECT_EQ(mpc_constraints_.stiff_min.u.idx_enabled.size(), n_u);
  EXPECT_EQ(mpc_constraints_.stiff_min.u.idx_enabled[0], true);

  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.idx_enabled.size(), n_u_dot);
  EXPECT_EQ(mpc_constraints_.stiff_max.u_dot.idx_enabled[0], true);

  EXPECT_EQ(mpc_constraints_.stiff_min.u_dot.idx_enabled.size(), n_u_dot);
  EXPECT_EQ(mpc_constraints_.stiff_min.u_dot.idx_enabled[0], true);
}

TEST_F(LateralConstraintTest, TestUpdateMPCConstraintSizes) {
  LateralConstraint::Constraints constraints;
  InitializeConstraints(constraints, n_nodes, vehicle_param_);
  lat_con_->UpdateMPCConstraintIndices(constraints, mpc_constraints_);
  lat_con_->UpdateMPCConstraintSizes(mpc_constraints_);

  auto CheckConstraintSizesLambda =
      [](const MPCData::Constraints::Constraint& mpc_constraint,
         std::pair<int, int> expected_n_x_info,
         std::pair<int, int> expected_n_u_info,
         std::pair<int, int> expected_n_u_dot_info) {
        double expected_n_rows_x = expected_n_x_info.first;
        double expected_n_cols_x = expected_n_x_info.second;
        double expected_n_rows_u = expected_n_u_info.first;
        double expected_n_cols_u = expected_n_u_info.second;
        double expected_n_rows_u_dot = expected_n_u_dot_info.first;
        double expected_n_cols_u_dot = expected_n_u_dot_info.second;

        auto ValidateConstraintInfo =
            [](int expected_rows, int expected_cols,
               const MPCData::Constraints::Constraint::ConstraintInfo& info) {
              // EXPECT_EQ to check the conditions
              EXPECT_EQ(expected_rows, info.n_enabled_row)
                  << "Row count does not match expected value.";
              EXPECT_EQ(expected_cols, info.n_enabled_col)
                  << "Column count does not match expected value.";
            };

        // validate x constraint
        ValidateConstraintInfo(expected_n_rows_x, expected_n_cols_x,
                               mpc_constraint.x);
        // validate u constraint
        ValidateConstraintInfo(expected_n_rows_u, expected_n_cols_u,
                               mpc_constraint.u);
        // validate u_dot constraint
        ValidateConstraintInfo(expected_n_rows_u_dot, expected_n_cols_u_dot,
                               mpc_constraint.u_dot);
      };

  std::pair<int, int> expected_soft_x_info = std::make_pair(2, n_nodes);
  std::pair<int, int> expected_soft_u_info = std::make_pair(1, n_steps);
  std::pair<int, int> expected_soft_u_dot_info = std::make_pair(1, n_steps);

  std::pair<int, int> expected_stiff_x_info = std::make_pair(2, n_nodes);
  std::pair<int, int> expected_stiff_u_info = std::make_pair(1, n_steps);
  std::pair<int, int> expected_stiff_u_dot_info = std::make_pair(1, n_steps);

  CheckConstraintSizesLambda(mpc_constraints_.soft_min, expected_soft_x_info,
                             expected_soft_u_info, expected_soft_u_dot_info);

  CheckConstraintSizesLambda(mpc_constraints_.soft_max, expected_soft_x_info,
                             expected_soft_u_info, expected_soft_u_dot_info);

  CheckConstraintSizesLambda(mpc_constraints_.stiff_min, expected_stiff_x_info,
                             expected_stiff_u_info, expected_stiff_u_dot_info);

  CheckConstraintSizesLambda(mpc_constraints_.stiff_max, expected_stiff_x_info,
                             expected_stiff_u_info, expected_stiff_u_dot_info);

  EXPECT_EQ(mpc_constraints_.n_ineq_hard, 0);
  EXPECT_EQ(mpc_constraints_.n_ineq_soft, 648);
  EXPECT_EQ(mpc_constraints_.n_ineq, 648);
}

TEST_F(LateralConstraintTest, TestUpdateMPCConstraintWeights) {
  LateralConstraint::Constraints constraints;
  InitializeConstraints(constraints, n_nodes, vehicle_param_);
  lat_con_->UpdateMPCConstraintIndices(constraints, mpc_constraints_);
  lat_con_->UpdateMPCConstraintSizes(mpc_constraints_);
  lat_con_->UpdateMPCConstraintWeights(lateral_config_.cost, mpc_constraints_);

  EXPECT_EQ(mpc_constraints_.soft_min.x.weight[0], 1.0e1);
  EXPECT_EQ(mpc_constraints_.soft_min.x.weight[1], 1.0e1);
  EXPECT_EQ(mpc_constraints_.stiff_min.x.weight[0], 5.0e2);
  EXPECT_EQ(mpc_constraints_.stiff_min.x.weight[1], 5.0e2);

  EXPECT_EQ(mpc_constraints_.soft_max.x.weight[0], 1.0e1);
  EXPECT_EQ(mpc_constraints_.soft_max.x.weight[1], 1.0e1);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.weight[0], 5.0e2);
  EXPECT_EQ(mpc_constraints_.stiff_max.x.weight[1], 5.0e2);
}

TEST_F(LateralConstraintTest, TestUpdateMPCConstraints) {
  LateralConstraint::Constraints constraints;
  InitializeConstraints(constraints, n_nodes, vehicle_param_);
  lat_con_->UpdateMPCConstraintIndices(constraints, mpc_constraints_);
  lat_con_->UpdateMPCConstraintSizes(mpc_constraints_);
  lat_con_->UpdateMPCConstraintWeights(lateral_config_.cost, mpc_constraints_);
  lat_con_->UpdateMPCConstraints(constraints, mpc_constraints_);

  for (int k = 0; k < n_nodes; k++) {
    EXPECT_NEAR(mpc_constraints_.soft_max.x.constraint(0, k), 2.0, kTol);
    EXPECT_NEAR(mpc_constraints_.stiff_max.x.constraint(0, k), 3.0, kTol);
    EXPECT_NEAR(mpc_constraints_.soft_max.x.constraint(1, k), 5.0, kTol);
    EXPECT_NEAR(mpc_constraints_.stiff_max.x.constraint(1, k), 6.0, kTol);

    EXPECT_NEAR(mpc_constraints_.soft_min.x.constraint(0, k), -1.0, kTol);
    EXPECT_NEAR(mpc_constraints_.stiff_min.x.constraint(0, k), -4.0, kTol);
    EXPECT_NEAR(mpc_constraints_.soft_min.x.constraint(1, k), -7.0, kTol);
    EXPECT_NEAR(mpc_constraints_.stiff_min.x.constraint(1, k), -8.0, kTol);
  }

  for (int k = 0; k < n_steps; k++) {
    EXPECT_NEAR(mpc_constraints_.soft_max.u.constraint(0, k),
                vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio(),
                kTol);
    EXPECT_NEAR(mpc_constraints_.stiff_max.u.constraint(0, k),
                vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio(),
                kTol);
    EXPECT_NEAR(
        mpc_constraints_.soft_min.u.constraint(0, k),
        -vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio(), kTol);
    EXPECT_NEAR(
        mpc_constraints_.stiff_min.u.constraint(0, k),
        -vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio(), kTol);

    EXPECT_NEAR(
        mpc_constraints_.soft_max.u_dot.constraint(0, k),
        vehicle_param_.max_steer_angle_rate() / vehicle_param_.steer_ratio(),
        kTol);
    EXPECT_NEAR(
        mpc_constraints_.stiff_max.u_dot.constraint(0, k),
        vehicle_param_.max_steer_angle_rate() / vehicle_param_.steer_ratio(),
        kTol);
    EXPECT_NEAR(
        mpc_constraints_.soft_min.u_dot.constraint(0, k),
        -vehicle_param_.max_steer_angle_rate() / vehicle_param_.steer_ratio(),
        kTol);
    EXPECT_NEAR(
        mpc_constraints_.stiff_min.u_dot.constraint(0, k),
        -vehicle_param_.max_steer_angle_rate() / vehicle_param_.steer_ratio(),
        kTol);
  }
}

}  // namespace planning
}  // namespace zark
