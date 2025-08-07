/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file lane_change_evaluator_test.cc
 **/

#include "apps/planning/src/evaluation/lane_change_evaluator.h"

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

namespace {

LaneChangeEvaluator PrepareLaneChangeEvaluator() {
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
  LaneChangeEvaluator lane_change_evaluator(
      evaluation_decider_config.lane_change);

  return lane_change_evaluator;
}

Proposal PrepareBasicProposal() {
  LatMPCData lat_mpc_data;
  LonMPCData lon_mpc_data;
  Proposal proposal{};
  lat_mpc_data.slacks.soft_max.x = Eigen::MatrixXd::Zero(4, 1);
  lat_mpc_data.slacks.soft_max.u = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.soft_max.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.soft_min.x = Eigen::MatrixXd::Zero(4, 1);
  lat_mpc_data.slacks.soft_min.u = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.soft_min.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.soft_max.x = Eigen::MatrixXd::Zero(2, 1);
  lon_mpc_data.slacks.soft_max.u = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.soft_max.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.soft_min.x = Eigen::MatrixXd::Zero(2, 1);
  lon_mpc_data.slacks.soft_min.u = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.soft_min.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.soft_max.x(3, 0) = 2.0;
  lat_mpc_data.slacks.soft_max.u(0, 0) = 1.0;
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
  lon_mpc_data.dt = 0.1;
  lon_mpc_data.n_steps = 80;
  proposal.SetLatMPCData(lat_mpc_data);
  proposal.SetLonMPCData(lon_mpc_data);

  return proposal;
}

}  // namespace

TEST(LaneChangeEvaluatorTest, Evaluate) {
  // prepare
  auto lane_change_evaluator = PrepareLaneChangeEvaluator();
  auto proposal = PrepareBasicProposal();
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 20; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRoute local_route(local_route_points, LocalRouteConfig());
  Mission mission;
  mission.lc_urgency = 2.0;
  Corridor corridor;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission,
                             CorridorInfo::Type::LANE_KEEP, corridor, 0,
                             obstacle_map);
  proposal.SetCorridorInfo(&corridor_info);

  // when
  Costs lane_change_costs = lane_change_evaluator.Evaluate(
      proposal, CorridorInfo::Type::LANE_CHANGE, 2.0);

  // then
  EXPECT_EQ(2, lane_change_costs.size());
}

TEST(LaneChangeEvaluatorTest, TestEvaluateLaneChangeUrgencyCost) {
  // prepare
  auto lane_change_evaluator = PrepareLaneChangeEvaluator();
  auto proposal = PrepareBasicProposal();
  Mission mission;
  mission.lc_urgency = 2.0;
  const auto t_staging = 2.0;
  const auto corridor_types = std::array<CorridorInfo::Type, 4>{
      CorridorInfo::Type::LANE_KEEP, CorridorInfo::Type::LANE_CHANGE,
      CorridorInfo::Type::STAGING, CorridorInfo::Type::NUDGE};

  for (const auto corridor_type : corridor_types) {
    Costs costs;

    // when
    lane_change_evaluator.EvaluateLaneChangeUrgencyCost(
        mission, proposal.GetLonMPCData(), corridor_type, t_staging, costs);

    // then
    if (corridor_type == CorridorInfo::Type::LANE_KEEP) {
      EXPECT_EQ(1, costs.size());
      EXPECT_DOUBLE_EQ(2.0, costs.at(0).first);
    } else if (corridor_type == CorridorInfo::Type::STAGING) {
      EXPECT_EQ(1, costs.size());
      EXPECT_DOUBLE_EQ(0.5 + 0.5, costs.at(0).first);
    } else {
      EXPECT_EQ(0, costs.size());
    }
  }
}

TEST(LaneChangeEvaluatorTest, TestEvaluateLaneChangeAbortCost) {
  // prepare
  auto lane_change_evaluator = PrepareLaneChangeEvaluator();
  const auto last_corridor_types = std::array<CorridorInfo::Type, 4>{
      CorridorInfo::Type::LANE_KEEP, CorridorInfo::Type::LANE_CHANGE,
      CorridorInfo::Type::STAGING, CorridorInfo::Type::NUDGE};
  const auto cur_corridor_types = last_corridor_types;

  //
  for (const auto last_corridor_type : last_corridor_types) {
    for (const auto cur_corridor_type : cur_corridor_types) {
      Costs costs;
      // when
      lane_change_evaluator.EvaluateLaneChangeAbortCost(
          cur_corridor_type, last_corridor_type, costs);

      // then
      if (last_corridor_type != CorridorInfo::Type::LANE_CHANGE) {
        EXPECT_EQ(0, costs.size());
      } else {
        if (cur_corridor_type == CorridorInfo::Type::LANE_CHANGE) {
          EXPECT_EQ(0, costs.size());

        } else {
          EXPECT_EQ(1, costs.size());
          EXPECT_DOUBLE_EQ(0.5, costs.at(0).first);
        }
      }
    }
  }
}

}  // namespace planning
}  // namespace zark
