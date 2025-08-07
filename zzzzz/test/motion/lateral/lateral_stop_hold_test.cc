/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_stop_hold_test.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_stop_hold.h"
#include "point_factory.h"
#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {
class LateralStopHoldTest : public ::testing::Test {
 public:
  void SetUp() override {
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "lateral_optimizer_config_test.json";
    Config config{config_file};
    std::ifstream jdata(config_file);
    lateral_config_ =
        config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
    lat_stop_hold_ =
        std::unique_ptr<LateralStopHold>(new LateralStopHold(lateral_config_));
  }

 protected:
  std::shared_ptr<LateralStopHold> lat_stop_hold_;
  LateralOptimizerConfig lateral_config_;
};

::common::TrajectoryPoint InitializePlanningStartPoint() {
  ::common::PathPoint start_point(1.0, 2.0, 3.0, M_PI_2, 5.0, 6.0, 7.0, 8.0,
                                  "1", 1.0, 2.0);
  GaussianInfo gaussian_info;
  ::common::TrajectoryPoint planning_start_point(start_point, 2.0, 2.5, 3.0,
                                                 0.0, 1.0, gaussian_info);
  planning_start_point.set_omega(0.1);
  return planning_start_point;
}

TEST_F(LateralStopHoldTest, TestPublishStopTrajectory) {
  LatMPCData mpc_data;
  const int n_nodes = lateral_config_.model.num_steps + 1;
  ::common::TrajectoryPoint planning_start_point =
      InitializePlanningStartPoint();
  Eigen::VectorXd u_prev(lateral_config_.model.num_ctrl_rates);

  u_prev(0) = planning_start_point.steer();
  const double dt_prev = 0.1;
  mpc_data.names.x.emplace_back("l");
  mpc_data.names.x.emplace_back("l_dot");
  mpc_data.names.x.emplace_back("psi_s");
  mpc_data.names.x.emplace_back("psi_s_dot");
  mpc_data.names.u.emplace_back("delta");
  mpc_data.names.u_dot.emplace_back("delta_dot");
  mpc_data.x_ref =
      Eigen::MatrixXd::Zero(lateral_config_.model.num_states, n_nodes);

  mpc_data.u_prev = u_prev;
  mpc_data = lat_stop_hold_->GenerateStopTrajectory(u_prev, dt_prev);

  EXPECT_DOUBLE_EQ(mpc_data.u_prev(0), 1.0);
  EXPECT_DOUBLE_EQ(mpc_data.dt_prev, 0.1);

  for (int k = 0; k < n_nodes; k++) {
    EXPECT_DOUBLE_EQ(mpc_data.x_ref(0, k), 0.0);
    EXPECT_DOUBLE_EQ(mpc_data.x_ref(1, k), 0.0);
    EXPECT_DOUBLE_EQ(mpc_data.x_ref(2, k), 0.0);
    EXPECT_DOUBLE_EQ(mpc_data.x_ref(3, k), 0.0);
  }
}

}  // namespace planning
}  // namespace zark
