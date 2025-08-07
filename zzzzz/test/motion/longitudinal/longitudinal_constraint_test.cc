#include "apps/planning/src/motion/longitudinal/longitudinal_constraint.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"
#include "gtest/gtest_prod.h"

namespace zark {
namespace planning {
namespace {
const int num_steps = 4;
const int num_nodes = num_steps + 1;
const int num_states = 2;
const int num_ctrls = 1;
const int num_ctrl_rates = 1;

// matlab results
// ineq
const int matlab_n_ineq_soft = 62;
const int matlab_n_ineq_hard = 5;
const int matlab_n_ineq = 67;
// index
std::vector<bool> matlab_soft_min_x_idx_enabled = {1, 0};
std::vector<bool> matlab_soft_min_u_idx_enabled = {1};
std::vector<bool> matlab_soft_min_u_dot_idx_enabled = {1};
std::vector<bool> matlab_soft_max_x_idx_enabled = {1, 1};
std::vector<bool> matlab_soft_max_u_idx_enabled = {1};
std::vector<bool> matlab_soft_max_u_dot_idx_enabled = {1};
std::vector<bool> matlab_stiff_min_x_idx_enabled = {1, 0};
std::vector<bool> matlab_stiff_min_u_idx_enabled = {1};
std::vector<bool> matlab_stiff_min_u_dot_idx_enabled = {1};
std::vector<bool> matlab_stiff_max_x_idx_enabled = {1, 1};
std::vector<bool> matlab_stiff_max_u_idx_enabled = {1};
std::vector<bool> matlab_stiff_max_u_dot_idx_enabled = {1};
std::vector<bool> matlab_hard_min_x_idx_enabled = {0, 1};
std::vector<bool> matlab_hard_min_u_idx_enabled = {0};
std::vector<bool> matlab_hard_min_u_dot_idx_enabled = {0};
std::vector<bool> matlab_hard_max_x_idx_enabled = {0, 0};
std::vector<bool> matlab_hard_max_u_idx_enabled = {0};
std::vector<bool> matlab_hard_max_u_dot_idx_enabled = {0};
// weight
Eigen::VectorXd matlab_soft_min_x_weight =
    (Eigen::VectorXd(1) << 10).finished();
Eigen::VectorXd matlab_soft_min_u_weight =
    (Eigen::VectorXd(1) << 10).finished();
Eigen::VectorXd matlab_soft_min_u_dot_weight =
    (Eigen::VectorXd(1) << 10).finished();
Eigen::VectorXd matlab_soft_max_x_weight =
    (Eigen::VectorXd(2) << 10, 10).finished();
Eigen::VectorXd matlab_soft_max_u_weight =
    (Eigen::VectorXd(1) << 50).finished();
Eigen::VectorXd matlab_soft_max_u_dot_weight =
    (Eigen::VectorXd(1) << 50).finished();
Eigen::VectorXd matlab_stiff_min_x_weight =
    (Eigen::VectorXd(1) << 500).finished();
Eigen::VectorXd matlab_stiff_min_u_weight =
    (Eigen::VectorXd(1) << 500).finished();
Eigen::VectorXd matlab_stiff_min_u_dot_weight =
    (Eigen::VectorXd(1) << 500).finished();
Eigen::VectorXd matlab_stiff_max_x_weight =
    (Eigen::VectorXd(2) << 500, 500).finished();
Eigen::VectorXd matlab_stiff_max_u_weight =
    (Eigen::VectorXd(1) << 500).finished();
Eigen::VectorXd matlab_stiff_max_u_dot_weight =
    (Eigen::VectorXd(1) << 500).finished();
Eigen::VectorXd matlab_hard_min_x_weight;
Eigen::VectorXd matlab_hard_min_u_weight;
Eigen::VectorXd matlab_hard_min_u_dot_weight;
Eigen::VectorXd matlab_hard_max_x_weight;
Eigen::VectorXd matlab_hard_max_u_weight;
Eigen::VectorXd matlab_hard_max_u_dot_weight;
// constraints
Eigen::MatrixXd matlab_soft_min_x_constraint =
    (Eigen::MatrixXd(1, 5) << 0, 0, 0, 0, 0).finished();
Eigen::MatrixXd matlab_soft_min_u_constraint =
    (Eigen::MatrixXd(1, 4) << -2, -2, -2, -2).finished();
Eigen::MatrixXd matlab_soft_min_u_dot_constraint =
    (Eigen::MatrixXd(1, 4) << -3, -3, -3, -3).finished();
Eigen::MatrixXd matlab_soft_max_x_constraint =
    (Eigen::MatrixXd(2, 5) << kInf, 9, 14, 0, 4, 8, 8, 8, 8, 8).finished();
Eigen::MatrixXd matlab_soft_max_u_constraint =
    (Eigen::MatrixXd(1, 4) << 1.6493, 1.6493, 1.6493, 1.6493).finished();
Eigen::MatrixXd matlab_soft_max_u_dot_constraint =
    (Eigen::MatrixXd(1, 4) << 1.5, 1.5, 1.5, 1.5).finished();
Eigen::MatrixXd matlab_stiff_min_x_constraint =
    (Eigen::MatrixXd(1, 5) << 0, 0, 0, 0, 0).finished();
Eigen::MatrixXd matlab_stiff_min_u_constraint =
    (Eigen::MatrixXd(1, 4) << -6, -6, -6, -6).finished();
Eigen::MatrixXd matlab_stiff_min_u_dot_constraint =
    (Eigen::MatrixXd(1, 4) << -6, -6, -6, -6).finished();
Eigen::MatrixXd matlab_stiff_max_x_constraint =
    (Eigen::MatrixXd(2, 5) << kInf, 16, 21, 6, 11, 11, 11, 11, 11, 11)
        .finished();
Eigen::MatrixXd matlab_stiff_max_u_constraint =
    (Eigen::MatrixXd(1, 4) << 2.087, 2.087, 2.087, 2.087).finished();
Eigen::MatrixXd matlab_stiff_max_u_dot_constraint =
    (Eigen::MatrixXd(1, 4) << -6, -6, -6, -6).finished();
Eigen::MatrixXd matlab_hard_min_x_constraint =
    (Eigen::MatrixXd(1, 5) << 0, 0, 0, 0, 0).finished();
Eigen::MatrixXd matlab_hard_min_u_constraint;
Eigen::MatrixXd matlab_hard_min_u_dot_constraint;
Eigen::MatrixXd matlab_hard_max_x_constraint;
Eigen::MatrixXd matlab_hard_max_u_constraint;
Eigen::MatrixXd matlab_hard_max_u_dot_constraint;
}  // namespace

void InitializeConstraints(LongitudinalConstraint::Constraints &constraints) {
  auto &hard = constraints.hard;
  auto &stiff = constraints.stiff;
  auto &soft = constraints.soft;
  soft.s_min = (Eigen::RowVectorXd(5) << 0, 0, 0, 0, 0).finished();
  soft.s_max = (Eigen::RowVectorXd(5) << kInf, 9, 14, 0, 4).finished();
  soft.v_max = (Eigen::RowVectorXd(5) << 8, 8, 8, 8, 8).finished();
  soft.a_min = (Eigen::RowVectorXd(4) << -2, -2, -2, -2).finished();
  soft.a_max =
      (Eigen::RowVectorXd(4) << 1.64926, 1.64926, 1.64926, 1.64926).finished();
  soft.j_min = (Eigen::RowVectorXd(4) << -3, -3, -3, -3).finished();
  soft.j_max = (Eigen::RowVectorXd(4) << 1.5, 1.5, 1.5, 1.5).finished();
  stiff.s_min = (Eigen::RowVectorXd(5) << 0, 0, 0, 0, 0).finished();
  stiff.s_max = (Eigen::RowVectorXd(5) << kInf, 16, 21, 6, 11).finished();
  stiff.v_max = (Eigen::RowVectorXd(5) << 11, 11, 11, 11, 11).finished();
  stiff.a_min = (Eigen::RowVectorXd(4) << -6, -6, -6, -6).finished();
  stiff.a_max =
      (Eigen::RowVectorXd(4) << 2.08704, 2.08704, 2.08704, 2.08704).finished();
  stiff.j_min = (Eigen::RowVectorXd(4) << -6, -6, -6, -6).finished();
  stiff.j_max = (Eigen::RowVectorXd(4) << 5, 5, 5, 5).finished();
  hard.v_min = (Eigen::RowVectorXd(5) << 0, 0, 0, 0, 0).finished();
}

class LongitudinalConstraintTest : public ::testing::Test {
  virtual void SetUp() {
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "longitudinal_optimizer_config_test.json";
    zark::planning::Config config{config_file};
    std::ifstream jdata(config_file);
    longitudinal_optimizer_config =
        config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
    lon_con_ =
        std::make_shared<LongitudinalConstraint>(longitudinal_optimizer_config);
    lon_lookup_tables_ = std::make_shared<LongitudinalLookupTables>(
        longitudinal_optimizer_config);
  }

 protected:
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  std::shared_ptr<LongitudinalLookupTables> lon_lookup_tables_;
  std::shared_ptr<LongitudinalConstraint> lon_con_;
};

TEST_F(LongitudinalConstraintTest, TestDesignConstraints) {
  Eigen::VectorXd x_init(3);
  x_init(0) = 0.0;
  x_init(1) = 5.0;
  Eigen::MatrixXd x_ref(2, 5);
  x_ref << 0, 5, 10, 15, 20, 5, 5, 5, 5, 5;
  Blocker blocker_1(5);
  Blocker blocker_2(5);
  std::vector<Blocker> blockers;
  blocker_1.is_filtered = true;
  blocker_1.is_front = true;
  blocker_1.s[1] = 20.0;
  blocker_1.s[2] = 25.0;
  blocker_1.s[3] = 30.0;
  blocker_1.v[1] = 5.0;
  blocker_1.v[2] = 5.0;
  blocker_1.v[3] = 5.0;
  blocker_2.is_filtered = true;
  blocker_2.is_front = true;
  blocker_2.v[3] = 5.0;
  blocker_2.v[4] = 5.0;
  blocker_2.s[3] = 10.0;
  blocker_2.s[4] = 15.0;
  Obstacle obstacle1, obstacle2;
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_theta(0.0);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_id(1);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  obstacle1 = Obstacle("1_0", perception_obstacle);
  perception_obstacle.set_id(2);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_sub_type(perception::SubType::ST_UNKNOWN);
  obstacle2 = Obstacle("2_0", perception_obstacle);
  blocker_1.obs = &obstacle1;
  blocker_2.obs = &obstacle2;
  blockers.emplace_back(blocker_1);
  blockers.emplace_back(blocker_2);
  SpeedLimit speed_limit;
  speed_limit.AppendSpeedLimit(0.0, 8.0);
  speed_limit.AppendSpeedLimit(10.0, 8.0);
  speed_limit.AppendSpeedLimit(20.0, 8.0);
  speed_limit.AppendSpeedLimit(30.0, 8.0);
  speed_limit.AppendSpeedLimit(40.0, 8.0);
  speed_limit.AppendSpeedLimit(50.0, 8.0);
  LongitudinalPadding lon_padding;

  MPCData::Constraints con_results = lon_con_->DesignConstraints(
      x_init, x_ref, blockers, speed_limit, TimeGapLevel::LEVEL_3,
      *lon_lookup_tables_, lon_padding);
}

TEST_F(LongitudinalConstraintTest, TestUpdateMPCConstraintIndices) {
  LongitudinalConstraint::Constraints constraints;
  InitializeConstraints(constraints);
  MPCData::Constraints mpc_constraints;
  lon_con_->UpdateMPCConstraintIndices(constraints, mpc_constraints);

  CHECK_EQ(matlab_soft_min_x_idx_enabled[0],
           mpc_constraints.soft_min.x.idx_enabled[0]);
  CHECK_EQ(matlab_soft_min_x_idx_enabled[1],
           mpc_constraints.soft_min.x.idx_enabled[1]);
  CHECK_EQ(matlab_soft_min_u_idx_enabled[0],
           mpc_constraints.soft_min.u.idx_enabled[0]);
  CHECK_EQ(matlab_soft_min_u_dot_idx_enabled[0],
           mpc_constraints.soft_min.u_dot.idx_enabled[0]);
  CHECK_EQ(matlab_soft_max_x_idx_enabled[0],
           mpc_constraints.soft_max.x.idx_enabled[0]);
  CHECK_EQ(matlab_soft_max_x_idx_enabled[1],
           mpc_constraints.soft_max.x.idx_enabled[1]);
  CHECK_EQ(matlab_soft_max_u_idx_enabled[0],
           mpc_constraints.soft_max.u.idx_enabled[0]);
  CHECK_EQ(matlab_soft_max_u_dot_idx_enabled[0],
           mpc_constraints.soft_max.u_dot.idx_enabled[0]);
  CHECK_EQ(matlab_stiff_min_x_idx_enabled[0],
           mpc_constraints.stiff_min.x.idx_enabled[0]);
  CHECK_EQ(matlab_stiff_min_x_idx_enabled[1],
           mpc_constraints.stiff_min.x.idx_enabled[1]);
  CHECK_EQ(matlab_stiff_min_u_idx_enabled[0],
           mpc_constraints.stiff_min.u.idx_enabled[0]);
  CHECK_EQ(matlab_stiff_min_u_dot_idx_enabled[0],
           mpc_constraints.stiff_min.u_dot.idx_enabled[0]);
  CHECK_EQ(matlab_stiff_max_x_idx_enabled[0],
           mpc_constraints.stiff_max.x.idx_enabled[0]);
  CHECK_EQ(matlab_stiff_max_x_idx_enabled[1],
           mpc_constraints.stiff_max.x.idx_enabled[1]);
  CHECK_EQ(matlab_stiff_max_u_idx_enabled[0],
           mpc_constraints.stiff_max.u.idx_enabled[0]);
  CHECK_EQ(matlab_stiff_max_u_dot_idx_enabled[0],
           mpc_constraints.stiff_max.u_dot.idx_enabled[0]);
  CHECK_EQ(matlab_hard_min_x_idx_enabled[0],
           mpc_constraints.hard_min.x.idx_enabled[0]);
  CHECK_EQ(matlab_hard_min_x_idx_enabled[1],
           mpc_constraints.hard_min.x.idx_enabled[1]);
  CHECK_EQ(matlab_hard_min_u_idx_enabled[0],
           mpc_constraints.hard_min.u.idx_enabled[0]);
  CHECK_EQ(matlab_hard_min_u_dot_idx_enabled[0],
           mpc_constraints.hard_min.u_dot.idx_enabled[0]);
  CHECK_EQ(matlab_hard_max_x_idx_enabled[0],
           mpc_constraints.hard_max.x.idx_enabled[0]);
  CHECK_EQ(matlab_hard_max_x_idx_enabled[1],
           mpc_constraints.hard_max.x.idx_enabled[1]);
  CHECK_EQ(matlab_hard_max_u_idx_enabled[0],
           mpc_constraints.hard_max.u.idx_enabled[0]);
  CHECK_EQ(matlab_hard_max_u_dot_idx_enabled[0],
           mpc_constraints.hard_max.u_dot.idx_enabled[0]);
}

TEST_F(LongitudinalConstraintTest, TestUpdateMPCConstraintSizes) {
  LongitudinalConstraint::Constraints constraints;
  InitializeConstraints(constraints);
  MPCData::Constraints mpc_constraints;
  lon_con_->UpdateMPCConstraintIndices(constraints, mpc_constraints);
  lon_con_->UpdateMPCConstraintSizes(mpc_constraints);

  CHECK_EQ(matlab_n_ineq_soft, mpc_constraints.n_ineq_soft);
  CHECK_EQ(matlab_n_ineq_hard, mpc_constraints.n_ineq_hard);
  CHECK_EQ(matlab_n_ineq, mpc_constraints.n_ineq);
}

TEST_F(LongitudinalConstraintTest, TestUpdateMPCConstraintWeights) {
  LongitudinalConstraint::Constraints constraints;
  InitializeConstraints(constraints);
  MPCData::Constraints mpc_constraints;
  lon_con_->UpdateMPCConstraintIndices(constraints, mpc_constraints);
  lon_con_->UpdateMPCConstraintSizes(mpc_constraints);
  lon_con_->UpdateMPCConstraintWeights(longitudinal_optimizer_config.cost,
                                       mpc_constraints);

  CHECK_EQ(matlab_soft_min_x_weight(0), mpc_constraints.soft_min.x.weight(0));
  CHECK_EQ(matlab_soft_min_u_weight(0), mpc_constraints.soft_min.u.weight(0));
  CHECK_EQ(matlab_soft_min_u_dot_weight(0),
           mpc_constraints.soft_min.u_dot.weight(0));
  CHECK_EQ(matlab_soft_max_x_weight(0), mpc_constraints.soft_max.x.weight(0));
  CHECK_EQ(matlab_soft_max_u_weight(0), mpc_constraints.soft_max.u.weight(0));
  CHECK_EQ(matlab_soft_max_u_dot_weight(0),
           mpc_constraints.soft_max.u_dot.weight(0));
  CHECK_EQ(matlab_stiff_min_x_weight(0), mpc_constraints.stiff_min.x.weight(0));
  CHECK_EQ(matlab_stiff_min_u_weight(0), mpc_constraints.stiff_min.u.weight(0));
  CHECK_EQ(matlab_stiff_min_u_dot_weight(0),
           mpc_constraints.stiff_min.u_dot.weight(0));
  CHECK_EQ(matlab_stiff_max_x_weight(0), mpc_constraints.stiff_max.x.weight(0));
  CHECK_EQ(matlab_stiff_max_u_weight(0), mpc_constraints.stiff_max.u.weight(0));
  CHECK_EQ(matlab_stiff_max_u_dot_weight(0),
           mpc_constraints.stiff_max.u_dot.weight(0));
  CHECK_EQ(matlab_hard_min_x_weight.size(),
           mpc_constraints.hard_min.x.weight.size());
  CHECK_EQ(matlab_hard_min_u_weight.size(),
           mpc_constraints.hard_min.u.weight.size());
  CHECK_EQ(matlab_hard_min_u_dot_weight.size(),
           mpc_constraints.hard_min.u_dot.weight.size());
  CHECK_EQ(matlab_hard_max_x_weight.size(),
           mpc_constraints.hard_max.x.weight.size());
  CHECK_EQ(matlab_hard_max_u_weight.size(),
           mpc_constraints.hard_max.u.weight.size());
  CHECK_EQ(matlab_hard_max_u_dot_weight.size(),
           mpc_constraints.hard_max.u_dot.weight.size());
}

TEST_F(LongitudinalConstraintTest, TestUpdateMPCConstraints) {
  LongitudinalConstraint::Constraints constraints;
  InitializeConstraints(constraints);
  MPCData::Constraints mpc_constraints;
  lon_con_->UpdateMPCConstraintIndices(constraints, mpc_constraints);
  lon_con_->UpdateMPCConstraintSizes(mpc_constraints);
  lon_con_->UpdateMPCConstraintWeights(longitudinal_optimizer_config.cost,
                                       mpc_constraints);
  lon_con_->UpdateMPCConstraints(constraints, mpc_constraints);

  CHECK_EQ(matlab_soft_min_x_constraint.cols(),
           mpc_constraints.soft_min.x.constraint.cols());
  CHECK_EQ(matlab_soft_min_x_constraint.rows(),
           mpc_constraints.soft_min.x.constraint.rows());
  CHECK_EQ(matlab_soft_min_u_constraint.cols(),
           mpc_constraints.soft_min.u.constraint.cols());
  CHECK_EQ(matlab_soft_min_u_dot_constraint.cols(),
           mpc_constraints.soft_min.u_dot.constraint.cols());
  CHECK_EQ(matlab_soft_max_x_constraint.cols(),
           mpc_constraints.soft_max.x.constraint.cols());
  CHECK_EQ(matlab_soft_max_x_constraint.rows(),
           mpc_constraints.soft_max.x.constraint.rows());
  CHECK_EQ(matlab_soft_max_u_constraint.cols(),
           mpc_constraints.soft_max.u.constraint.cols());
  CHECK_EQ(matlab_soft_max_u_dot_constraint.cols(),
           mpc_constraints.soft_max.u_dot.constraint.cols());
  CHECK_EQ(matlab_stiff_min_x_constraint.cols(),
           mpc_constraints.stiff_min.x.constraint.cols());
  CHECK_EQ(matlab_stiff_min_u_constraint.cols(),
           mpc_constraints.stiff_min.u.constraint.cols());
  CHECK_EQ(matlab_stiff_min_u_dot_constraint.cols(),
           mpc_constraints.stiff_min.u_dot.constraint.cols());
  CHECK_EQ(matlab_stiff_max_x_constraint.cols(),
           mpc_constraints.stiff_max.x.constraint.cols());
  CHECK_EQ(matlab_stiff_max_x_constraint.rows(),
           mpc_constraints.stiff_max.x.constraint.rows());
  CHECK_EQ(matlab_stiff_max_u_constraint.cols(),
           mpc_constraints.stiff_max.u.constraint.cols());
  CHECK_EQ(matlab_stiff_max_u_dot_constraint.cols(),
           mpc_constraints.stiff_max.u_dot.constraint.cols());
  CHECK_EQ(matlab_hard_min_x_constraint.cols(),
           mpc_constraints.hard_min.x.constraint.cols());
  CHECK_EQ(matlab_hard_min_u_constraint.cols(),
           mpc_constraints.hard_min.u.constraint.cols());
  CHECK_EQ(matlab_hard_min_u_dot_constraint.cols(),
           mpc_constraints.hard_min.u_dot.constraint.cols());
  CHECK_EQ(matlab_hard_max_x_constraint.cols(),
           mpc_constraints.hard_max.x.constraint.cols());
  CHECK_EQ(matlab_hard_max_u_constraint.cols(),
           mpc_constraints.hard_max.u.constraint.cols());
  CHECK_EQ(matlab_hard_max_u_dot_constraint.cols(),
           mpc_constraints.hard_max.u_dot.constraint.cols());
}

}  // namespace planning
}  // namespace zark
