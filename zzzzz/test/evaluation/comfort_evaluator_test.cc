/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file comfort_evaluator_test.cc
 **/

#include "apps/planning/src/evaluation/comfort_evaluator.h"

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;

TEST(ComfortEvaluatorTest, Evaluate) {
  std::string config_file =
      "/zark/apps/planning/test/config/json/evaluation_conf_test.json";
  zark::planning::Config config{config_file};
  EvaluationDeciderConfig evaluation_decider_config;
  std::ifstream jdata(config_file);
  nlohmann::json jnode = nlohmann::json::parse(jdata);
  if (jnode.count("evaluation_decider_config")) {
    evaluation_decider_config = config.SetEvaluationDeciderConfig(
        jnode.at("evaluation_decider_config"));
  }
  ComfortEvaluator comfort_evaluator(evaluation_decider_config.comfort);
  LatMPCData lat_mpc_data;
  LonMPCData lon_mpc_data;
  Proposal proposal{};
  lat_mpc_data.slacks.soft_max.x = Eigen::MatrixXd::Zero(4, 1);
  lat_mpc_data.slacks.soft_max.u = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.soft_max.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.soft_min.x = Eigen::MatrixXd::Zero(4, 1);
  lat_mpc_data.slacks.soft_min.u = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.soft_min.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.soft_max.x = Eigen::MatrixXd::Zero(2, 2);
  lon_mpc_data.slacks.soft_max.u = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.soft_max.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.soft_min.x = Eigen::MatrixXd::Zero(2, 1);
  lon_mpc_data.slacks.soft_min.u = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.soft_min.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.soft_max.x(3, 0) = 2.0;
  lat_mpc_data.slacks.soft_max.u(0, 0) = 1.0;
  lon_mpc_data.slacks.soft_max.x(0, 1) = 2.0;
  lon_mpc_data.slacks.soft_max.x(1, 0) = 3.0;
  lat_mpc_data.n_x = 4;
  lat_mpc_data.n_u = 1;
  lat_mpc_data.n_u_dot = 1;
  lon_mpc_data.n_x = 2;
  lon_mpc_data.n_u = 1;
  lon_mpc_data.n_u_dot = 1;
  lat_mpc_data.names.x = {"l", "l_dot", "psi", "psi_dot"};
  lat_mpc_data.names.u = {"delta"};
  lat_mpc_data.names.u_dot = {"l"};
  lon_mpc_data.names.x = {"s", "u"};
  lon_mpc_data.names.u = {"a"};
  lon_mpc_data.names.u_dot = {"j"};
  proposal.SetLatMPCData(lat_mpc_data);
  proposal.SetLonMPCData(lon_mpc_data);
  auto cost_table = comfort_evaluator.Evaluate(proposal);
  double cost_sum = 0.0;
  for (const auto& cost : cost_table) {
    cost_sum += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum, 6.2);
}