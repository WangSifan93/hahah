/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file similarity_evaluator_test.cc
 **/

#include "apps/planning/src/evaluation/similarity_evaluator.h"

#include "point_factory.h"
#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;
using ::util::PointFactory;

TEST(SimilarityEvaluatorTest, Evaluate) {
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
  SimilarityEvaluator similarity_evaluator(
      evaluation_decider_config.similarity);
  ::common::TrajectoryPoint p1;
  ::common::TrajectoryPoint p2;
  ::common::TrajectoryPoint p3;
  ::common::TrajectoryPoint p4;
  p1.set_path_point(PointFactory::ToPathPoint(0.0, 0.0, 0.0, 0.0));
  p2.set_path_point(PointFactory::ToPathPoint(1.0, 0.0, 0.0, 1.0));
  p3.set_path_point(PointFactory::ToPathPoint(2.0, 0.0, 0.0, 2.0));
  p4.set_path_point(PointFactory::ToPathPoint(3.0, 0.0, 0.0, 3.0));
  std::vector<::common::TrajectoryPoint> path_points{p1, p2, p3, p4};
  DiscretizedTrajectory prev_trajectory(path_points);
  Proposal proposal;
  ::common::TrajectoryPoint trajectory_point1, trajectory_point2,
      trajectory_point3, trajectory_point4;
  trajectory_point1.set_path_point(
      PointFactory::ToPathPoint(1.0, 0.0, 0.0, 0.0));
  trajectory_point2.set_path_point(
      PointFactory::ToPathPoint(2.0, 0.0, 0.0, 1.0));
  trajectory_point3.set_path_point(
      PointFactory::ToPathPoint(3.0, 0.0, 0.0, 2.0));
  trajectory_point4.set_path_point(
      PointFactory::ToPathPoint(4.0, 0.0, 0.0, 3.0));
  DiscretizedTrajectory trajectory1({trajectory_point1, trajectory_point2,
                                     trajectory_point3, trajectory_point4});
  proposal.SetTrajectory(trajectory1);
  auto cost_table1 = similarity_evaluator.Evaluate(proposal, prev_trajectory);
  double cost_sum = 0.0;
  for (const auto& cost : cost_table1) {
    cost_sum += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum, 0);
  trajectory_point1.set_path_point(
      PointFactory::ToPathPoint(0.0, 0.0, 0.0, 0.0));
  trajectory_point2.set_path_point(
      PointFactory::ToPathPoint(1.0, 1.0, 0.0, std::sqrt(2.0)));
  trajectory_point3.set_path_point(
      PointFactory::ToPathPoint(2.0, 1.0, 0.0, std::sqrt(2.0) + 1.0));
  trajectory_point4.set_path_point(
      PointFactory::ToPathPoint(3.0, 1.0, 0.0, std::sqrt(2.0) + 2.0));
  DiscretizedTrajectory trajectory2({trajectory_point1, trajectory_point2,
                                     trajectory_point3, trajectory_point4});
  proposal.SetTrajectory(trajectory2);
  auto cost_table2 = similarity_evaluator.Evaluate(proposal, prev_trajectory);
  cost_sum = 0.0;
  for (const auto& cost : cost_table2) {
    cost_sum += cost.first;
  }
  EXPECT_DOUBLE_EQ(cost_sum, 1.0 / 6);
}
