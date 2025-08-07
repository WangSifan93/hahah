/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_stop_hold_test.cc
 **/

#include "apps/planning/src/motion/longitudinal/longitudinal_stop_hold.h"

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

class LongitudinalStopHoldTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "longitudinal_optimizer_config_test.json";
    LongitudinalOptimizerConfig longitudinal_optimizer_config;
    zark::planning::Config config{config_file};
    std::ifstream jdata(config_file);
    longitudinal_optimizer_config =
        config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
    LongitudinalLookupTables lon_lookup_tables(longitudinal_optimizer_config);
    lon_stop_hold_ =
        std::make_shared<LongitudinalStopHold>(longitudinal_optimizer_config);
    lon_lookup_tables_ = std::make_shared<LongitudinalLookupTables>(
        longitudinal_optimizer_config);
  }

 protected:
  std::shared_ptr<LongitudinalStopHold> lon_stop_hold_;
  std::shared_ptr<LongitudinalLookupTables> lon_lookup_tables_;
};

TEST_F(LongitudinalStopHoldTest, TestIsStopHold) {
  LongitudinalPadding lon_padding;
  Blocker blocker_1(5);
  Blocker blocker_2(5);
  std::vector<Blocker> blockers;
  blocker_1.is_filtered = true;
  blocker_1.is_front = true;
  blocker_1.s[0] = 15.0;
  blocker_1.s[1] = 20.0;
  blocker_1.s[2] = 25.0;
  blocker_1.s[3] = 30.0;
  blocker_1.v[0] = 0.0;
  blocker_1.v[1] = 5.0;
  blocker_1.v[2] = 5.0;
  blocker_1.v[3] = 5.0;
  blocker_2.is_filtered = true;
  blocker_2.is_front = true;
  blocker_2.v[0] = 0.0;
  blocker_2.v[1] = 0.0;
  blocker_2.s[0] = 10.0;
  blocker_2.s[1] = 15.0;
  Obstacle obstacle1, obstacle2;
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_theta(0.0);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_id(1);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  ::common::Trajectory traj{};
  traj.add_trajectory_point();
  traj.add_trajectory_point();
  obstacle1 = Obstacle("1_0", perception_obstacle, traj);
  perception_obstacle.set_id(2);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_sub_type(perception::SubType::ST_UNKNOWN);
  obstacle2 = Obstacle("2_0", perception_obstacle);
  blocker_1.obs = &obstacle1;
  blocker_2.obs = &obstacle2;
  blockers.emplace_back(blocker_1);
  blockers.emplace_back(blocker_2);

  Eigen::VectorXd x_init(2);
  x_init(0) = 8.0;
  x_init(1) = 0.0;

  bool is_stop =
      lon_stop_hold_->IsStopHold(x_init, blockers, TimeGapLevel::LEVEL_3,
                                 *lon_lookup_tables_, lon_padding);
  EXPECT_EQ(is_stop, true);

  x_init(0) = 0.0;
  x_init(1) = 2.0;
  is_stop = lon_stop_hold_->IsStopHold(x_init, blockers, TimeGapLevel::LEVEL_3,
                                       *lon_lookup_tables_, lon_padding);
  EXPECT_EQ(is_stop, false);

  x_init(0) = -2.0;
  x_init(1) = 2.0;
  is_stop = lon_stop_hold_->IsStopHold(x_init, blockers, TimeGapLevel::LEVEL_3,
                                       *lon_lookup_tables_, lon_padding);
  EXPECT_EQ(is_stop, false);
}

TEST_F(LongitudinalStopHoldTest, TestPublishStopTrajectory) {
  LonMPCData mpc_data;
  Eigen::VectorXd u_prev(1);
  u_prev(0) = 10.0;
  const double dt_prev = 0.15;
  mpc_data.names.x.emplace_back("s");
  mpc_data.names.x.emplace_back("v");
  mpc_data.names.u_dot.emplace_back("j");
  mpc_data.x_ref = Eigen::MatrixXd::Zero(56, 5);
  mpc_data.u_prev = u_prev;
  mpc_data = lon_stop_hold_->GenerateStopTrajectory(u_prev, dt_prev);

  EXPECT_DOUBLE_EQ(mpc_data.u_prev(0), 10.0);
}

}  // namespace planning
}  // namespace zark
