/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file progress_evaluator_test.cc
 **/

#include "apps/planning/src/evaluation/progress_evaluator.h"

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;

class ProgressEvaluatorTest : public ::testing::Test {
 public:
  virtual void SetUp() {
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
    progress_evaluator = ProgressEvaluator(evaluation_decider_config.progress);
    LonMPCData lon_mpc_data1, lon_mpc_data2;
    Proposal proposal1, proposal2;
    lon_mpc_data1.x = Eigen::MatrixXd::Zero(2, 50);
    lon_mpc_data2.x = Eigen::MatrixXd::Zero(2, 50);
    for (std::size_t i = 0; i < 50; ++i) {
      lon_mpc_data1.x(0, i) = static_cast<double>(i);
      lon_mpc_data2.x(0, i) = static_cast<double>(i * 2);
    }
    proposal1.SetLonMPCData(lon_mpc_data1);
    proposal2.SetLonMPCData(lon_mpc_data2);
    proposals.emplace_back(proposal1);
    proposals.emplace_back(proposal2);
  }

 protected:
  ProgressEvaluator progress_evaluator;
  std::vector<Proposal> proposals;
};

TEST_F(ProgressEvaluatorTest, CalculateMaxDistance) {
  bool max_distance = progress_evaluator.CalculateMaxDistance(proposals);
  EXPECT_EQ(max_distance, true);
}

TEST_F(ProgressEvaluatorTest, Evaluate) {
  progress_evaluator.CalculateMaxDistance(proposals);
  LocalRoute local_route1, local_route2;
  Mission mission1, mission2;
  CorridorInfo::Type corridor_type;

  Corridor corridor_points;
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info1(local_route1, mission1, corridor_type,
                              corridor_points, idx_start_point, obstacle_map);
  CorridorInfo corridor_info2(local_route2, mission2, corridor_type,
                              corridor_points, idx_start_point, obstacle_map);
  proposals.at(0).SetCorridorInfo(&corridor_info1);
  proposals.at(1).SetCorridorInfo(&corridor_info2);
  auto cost_table1 = progress_evaluator.Evaluate(proposals.at(0));
  auto cost_table2 = progress_evaluator.Evaluate(proposals.at(1));
  double cost_sum1 = 0.0, cost_sum2 = 0.0;
  for (const auto& cost : cost_table1) {
    cost_sum1 += cost.first;
  }
  for (const auto& cost : cost_table2) {
    cost_sum2 += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum1, 5.0 / 6.0);
  EXPECT_DOUBLE_EQ(cost_sum2, 0.0);
}
