/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file safety_evaluator_test.cc
 **/

#include "apps/planning/src/evaluation/safety_evaluator.h"

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;
using ::math::Box2d;
using ::math::Polygon2d;
using ::math::Vec2d;

class SafetyEvaluatorTest : public ::testing::Test {
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
    safety_evaluator = SafetyEvaluator(evaluation_decider_config.safety);
    zark::planning::PlanningGflags::vehicle_config_path =
        "/zark/apps/planning/test/config/json/vehicle_conf_test.json";

    LatMPCData lat_mpc_data;
    LonMPCData lon_mpc_data;
    lat_mpc_data.slacks.stiff_max.x = Eigen::MatrixXd::Zero(4, 1);
    lat_mpc_data.slacks.stiff_max.u = Eigen::MatrixXd::Zero(1, 1);
    lat_mpc_data.slacks.stiff_max.u_dot = Eigen::MatrixXd::Zero(1, 1);
    lat_mpc_data.slacks.stiff_min.x = Eigen::MatrixXd::Zero(4, 1);
    lat_mpc_data.slacks.stiff_min.u = Eigen::MatrixXd::Zero(1, 1);
    lat_mpc_data.slacks.stiff_min.u_dot = Eigen::MatrixXd::Zero(1, 1);
    lon_mpc_data.slacks.stiff_max.x = Eigen::MatrixXd::Zero(2, 2);
    lon_mpc_data.slacks.stiff_max.u = Eigen::MatrixXd::Zero(1, 1);
    lon_mpc_data.slacks.stiff_max.u_dot = Eigen::MatrixXd::Zero(1, 1);
    lon_mpc_data.slacks.stiff_min.x = Eigen::MatrixXd::Zero(2, 1);
    lon_mpc_data.slacks.stiff_min.u = Eigen::MatrixXd::Zero(1, 1);
    lon_mpc_data.slacks.stiff_min.u_dot = Eigen::MatrixXd::Zero(1, 1);
    lat_mpc_data.slacks.stiff_max.x(3, 0) = 2.0;
    lat_mpc_data.slacks.stiff_max.u(0, 0) = 1.0;
    lon_mpc_data.slacks.stiff_max.x(0, 1) = 2.0;
    lon_mpc_data.slacks.stiff_max.x(1, 0) = 3.0;
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
    DiscretizedTrajectory trajectory;
    ::common::TrajectoryPoint trajectory_point;
    double ego_v = 2.0;
    for (int i = 0; i < 50; ++i) {
      trajectory_point.mutable_path_point()->set_x(0.0 + i * ego_v * 0.1);
      trajectory_point.mutable_path_point()->set_y(0.0);
      trajectory_point.mutable_path_point()->set_theta(0.0);
      trajectory_point.mutable_path_point()->set_s(0.0 + i * ego_v * 0.1);
      trajectory_point.set_v(ego_v);
      trajectory_point.set_relative_time(0.0 + i * 0.1);
      trajectory.AppendTrajectoryPoint(trajectory_point);
    }
    proposal.SetTrajectory(trajectory);
    // corridor info
    static CorridorInfo corridor_info{
        LocalRoute{},
        Mission{},
        CorridorInfo::Type::LANE_CHANGE,
        Corridor{},
        0,
        IndexedPtrList<std::string, const Obstacle*>{}};
    proposal.SetCorridorInfo(&corridor_info);
    // obs1
    perception::PerceptionObstacle perception_obstacle;
    perception_obstacle.mutable_position()->set_x(0.0);
    perception_obstacle.mutable_position()->set_y(3.705);
    perception_obstacle.set_theta(0.0);
    perception_obstacle.set_length(5.0);
    perception_obstacle.set_width(2.0);
    perception_obstacle.set_id(1);
    obstacle1 = Obstacle("static", perception_obstacle);
    // obs2
    ::common::Trajectory obstacle_traj;
    for (int i = 0; i < 50; ++i) {
      trajectory_point.mutable_path_point()->set_x(5.0 + i * ego_v * 0.05);
      trajectory_point.mutable_path_point()->set_y(0.0);
      trajectory_point.mutable_path_point()->set_theta(0.0);
      trajectory_point.set_v(ego_v * 0.5);
      trajectory_point.set_relative_time(0.0 + i * 0.1);
      obstacle_traj.mutable_trajectory_point()->emplace_back(trajectory_point);
    }
    obstacle2 = Obstacle("dynamic", perception_obstacle, obstacle_traj);
    // obs3
    perception_obstacle.mutable_position()->set_x(15.0);
    perception_obstacle.mutable_position()->set_y(0.0);
    obstacle3 = Obstacle("static", perception_obstacle);
  }

 protected:
  SafetyEvaluator safety_evaluator;
  Proposal proposal;
  Obstacle obstacle1, obstacle2, obstacle3;
};

TEST_F(SafetyEvaluatorTest, EvaluateLonLatCollisionCost) {
  std::vector<const Obstacle*> obstacles;
  Costs cost_table1;
  safety_evaluator.EvaluateLonLatCollisionCost(proposal, obstacles,
                                               cost_table1);
  double cost_sum = 0.0;
  for (const auto& cost : cost_table1) {
    cost_sum += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum, 0);
  obstacles.emplace_back(&obstacle1);
  Costs cost_table2;
  safety_evaluator.EvaluateLonLatCollisionCost(proposal, obstacles,
                                               cost_table2);
  cost_sum = 0.0;
  for (const auto& cost : cost_table2) {
    cost_sum += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum, 7.4);
  // obs2
  obstacles.emplace_back(&obstacle2);
  Costs cost_table3;
  safety_evaluator.EvaluateLonLatCollisionCost(proposal, obstacles,
                                               cost_table3);
  cost_sum = 0.0;
  for (const auto& cost : cost_table3) {
    cost_sum += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum, 10);
}

TEST_F(SafetyEvaluatorTest, EvaluateLonLatCollisionCostWithTimeWeight) {
  std::vector<const Obstacle*> obstacles;
  obstacles.emplace_back(&obstacle3);
  Costs cost_table;
  safety_evaluator.EvaluateLonLatCollisionCost(proposal, obstacles, cost_table);
  double cost_sum = 0.0;
  for (const auto& cost : cost_table) {
    cost_sum += cost.first;
  }
  EXPECT_NEAR(cost_sum, 5.3732, 0.00001);
}

TEST_F(SafetyEvaluatorTest, EvaluateSlackCost) {
  Costs cost_table;
  safety_evaluator.EvaluateSlackCost(proposal, cost_table);
  double cost_sum = 0.0;
  for (const auto& cost : cost_table) {
    cost_sum += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum, 6.2);
}

TEST_F(SafetyEvaluatorTest, Evaluate) {
  // Tip: if want to skip one gtest, could use following func at the top of test
  // func. 'GTEST_SKIP() << "Skipping single test";'
  std::vector<const Obstacle*> obstacles;
  obstacles.emplace_back(&obstacle1);
  obstacles.emplace_back(&obstacle2);
  auto cost_table = safety_evaluator.Evaluate(proposal, obstacles);
  double cost_sum = 0.0;
  for (const auto& cost : cost_table) {
    cost_sum += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum, 16.2);
}
