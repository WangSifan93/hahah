/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file evaluation_decider_test.cc
 **/

#include <tuple>
#include <memory>

#include "point_factory.h"
#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"
#include "apps/planning/src/tasks/deciders/evaluation_decider/evaluation_decider.h"

using namespace zark::planning;
using ::util::PointFactory;

STBoundary CreatObject(double v, double s_init, double t_init, double t_end,
                       double length) {
  STBoundary obj;
  std::vector<STPoint> lower_points, upper_points;
  const int num = 50;
  for (size_t i = 0; i <= num; i++) {
    double delta_t_used = double(i) / (double)num * (t_end - t_init);
    STPoint lower_point(s_init + delta_t_used * v, t_init + delta_t_used);
    STPoint upper_point(s_init + delta_t_used * v + length,
                        t_init + delta_t_used);
    lower_points.push_back(lower_point);
    upper_points.push_back(upper_point);
  }
  obj = STBoundary::CreateInstanceAccurate(lower_points, upper_points);
  return obj;
}

std::tuple<std::shared_ptr<EvaluationDecider>, std::shared_ptr<Frame>,
           std::shared_ptr<DependencyInjector>>
PrepareFrame() {
  auto frame_ptr = std::make_shared<Frame>(0);
  Frame &frame = *frame_ptr;
  auto frame_previous = std::make_unique<Frame>(1);
  std::string config_file =
      "/zark/apps/planning/test/config/json/evaluation_conf_test.json";
  zark::planning::Config evaluation_config{config_file};
  EvaluationDeciderConfig evaluation_decider_config;

  std::ifstream jdata(config_file);
  nlohmann::json jnode = nlohmann::json::parse(jdata);
  if (jnode.count("evaluation_decider_config")) {
    evaluation_decider_config = evaluation_config.SetEvaluationDeciderConfig(
        jnode.at("evaluation_decider_config"));
  }
  zark::planning::PlanningGflags::vehicle_config_path =
      "/zark/apps/planning/test/config/json/vehicle_conf_test.json";
  TaskConfig::task_config_union task_config;
  task_config.evaluation_decider_config = evaluation_decider_config;
  LateralDeciderConfig lateral_decider_config;
  LateralDeciderConfig::Corridor lateral_corridor;
  lateral_corridor.t_staging = 2.0;
  lateral_decider_config.corridor = lateral_corridor;
  task_config.lateral_decider_config = lateral_decider_config;
  TaskConfig config;
  config.set_task_config(task_config);

  std::vector<::common::TrajectoryPoint> path_points;
  for (int i = 0; i < 50; ++i) {
    ::common::TrajectoryPoint p;
    p.set_path_point(
        PointFactory::ToPathPoint(0.0 + i * 1.0, 0.0, 0.0, 0.0 + i * 1.0));
    path_points.emplace_back(p);
  }
  DiscretizedTrajectory prev_trajectory(path_points);
  // local_route
  std::vector<LocalRoutePoint> local_route_points;
  LocalRoutePoint local_route_point;
  for (int i = 0; i < 20; i++) {
    local_route_point.set_x(0.0);
    local_route_point.set_y(0.0 + 1.0 * i);
    local_route_point.set_heading(M_PI / 2);
    local_route_points.emplace_back(local_route_point);
  }
  LocalRouteConfig local_route_config;
  LocalRoute local_route(local_route_points, local_route_config);
  static Mission mission;
  mission = Mission();
  mission.lc_urgency = 2.0;
  mission.is_lc_ready = true;
  mission.lc_request = Mission::LaneChangeRequest::LC_LEFT;
  frame_ptr->SetMission(mission);
  CorridorInfo::Type corridor_type = CorridorInfo::Type::LANE_KEEP;
  Corridor corridor;
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle *> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);

  corridor_type = CorridorInfo::Type::STAGING;
  CorridorInfo staging_corridor_info(local_route, mission, corridor_type,
                                     corridor, idx_start_point, obstacle_map);

  std::vector<Proposal> *pre_proposals = frame_previous->MutableProposals();
  corridor_type = CorridorInfo::Type::LANE_CHANGE;
  CorridorInfo pre_corridor_info(local_route, mission, corridor_type, corridor,
                                 idx_start_point, obstacle_map);
  Proposal pre_proposal;
  pre_proposal.SetCorridorInfo(&pre_corridor_info);
  pre_proposal.SetTrajectory(prev_trajectory);
  pre_proposals->emplace_back(pre_proposal);

  std::shared_ptr<DependencyInjector> injector =
      std::make_shared<DependencyInjector>();
  injector->frame_history()->Add(1, std::move(frame_previous));
  auto evaluation_decider_ptr =
      std::make_shared<EvaluationDecider>(config, injector);
  Proposal proposal1, proposal2, lc_proposal;
  Obstacle obstacle1, obstacle2;
  LatMPCData lat_mpc_data;
  LonMPCData lon_mpc_data;
  lat_mpc_data.slacks.stiff_max.x = Eigen::MatrixXd::Zero(4, 1);
  lat_mpc_data.slacks.stiff_max.u = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.stiff_max.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.stiff_min.x = Eigen::MatrixXd::Zero(4, 1);
  lat_mpc_data.slacks.stiff_min.u = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.stiff_min.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.stiff_max.x = Eigen::MatrixXd::Zero(2, 1);
  lon_mpc_data.slacks.stiff_max.u = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.stiff_max.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.stiff_min.x = Eigen::MatrixXd::Zero(2, 1);
  lon_mpc_data.slacks.stiff_min.u = Eigen::MatrixXd::Zero(1, 1);
  lon_mpc_data.slacks.stiff_min.u_dot = Eigen::MatrixXd::Zero(1, 1);
  lat_mpc_data.slacks.stiff_max.x(3, 0) = 2.0;
  lat_mpc_data.slacks.stiff_max.u(0, 0) = 1.0;
  lon_mpc_data.slacks.stiff_max.x(1, 0) = 3.0;
  lat_mpc_data.n_x = 4;
  lat_mpc_data.n_u = 1;
  lat_mpc_data.n_u_dot = 1;
  lon_mpc_data.n_x = 2;
  lon_mpc_data.n_u = 1;
  lon_mpc_data.n_u_dot = 1;
  lat_mpc_data.names.x = {"l", "l_dot", "psi", "psi_dot"};
  lat_mpc_data.names.u = {"delta"};
  lat_mpc_data.names.u_dot = {"delta_dot"};
  lon_mpc_data.names.x = {"s", "v"};
  lon_mpc_data.names.u = {"a"};
  lon_mpc_data.names.u_dot = {"j"};
  lon_mpc_data.x = Eigen::MatrixXd::Zero(2, 50);
  lon_mpc_data.dt = 0.1;
  lon_mpc_data.n_steps = 80;
  for (std::size_t i = 0; i < 50; ++i) {
    lon_mpc_data.x(0, i) = static_cast<double>(i);
  }
  proposal1.SetLatMPCData(lat_mpc_data);
  proposal1.SetLonMPCData(lon_mpc_data);
  proposal1.SetCorridorInfo(&corridor_info);
  lat_mpc_data.n_x = 1;
  lon_mpc_data.n_x = 1;
  proposal2.SetLatMPCData(lat_mpc_data);
  proposal2.SetLonMPCData(lon_mpc_data);
  proposal2.SetCorridorInfo(&staging_corridor_info);

  DiscretizedTrajectory trajectory;
  ::common::TrajectoryPoint trajectory_point;
  double ego_v = 1.0;
  for (int i = 0; i < 50; ++i) {
    trajectory_point.mutable_path_point()->set_x(1.0 + i * ego_v * 0.1);
    trajectory_point.mutable_path_point()->set_y(0.0);
    trajectory_point.mutable_path_point()->set_theta(0.0);
    trajectory_point.set_v(ego_v);
    trajectory_point.set_relative_time(0.0 + i * 0.1);
    trajectory.AppendTrajectoryPoint(trajectory_point);
  }
  proposal1.SetTrajectory(trajectory);
  trajectory.Clear();
  for (int i = 0; i < 50; ++i) {
    trajectory_point.mutable_path_point()->set_x(1.0 + i * ego_v * 0.1);
    trajectory_point.mutable_path_point()->set_y(0.5);
    trajectory_point.mutable_path_point()->set_theta(0.0);
    trajectory_point.set_v(ego_v);
    trajectory_point.set_relative_time(0.0 + i * 0.1);
    trajectory.AppendTrajectoryPoint(trajectory_point);
  }
  proposal2.SetTrajectory(trajectory);
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.mutable_position()->set_x(0.0);
  perception_obstacle.mutable_position()->set_y(3.705);
  perception_obstacle.set_theta(0.0);
  perception_obstacle.set_length(5.0);
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_id(1);
  obstacle1 = Obstacle("static", perception_obstacle);
  ::common::Trajectory obstacle_traj;
  for (int i = 0; i < 50; ++i) {
    trajectory_point.mutable_path_point()->set_x(10.0 + i * ego_v * 0.01);
    trajectory_point.mutable_path_point()->set_y(0.0);
    trajectory_point.mutable_path_point()->set_theta(0.0);
    trajectory_point.set_v(ego_v * 0.5);
    trajectory_point.set_relative_time(0.0 + i * 0.1);
    obstacle_traj.mutable_trajectory_point()->emplace_back(trajectory_point);
  }
  obstacle2 = Obstacle("dynamic", perception_obstacle, obstacle_traj);
  frame.AddObstacle(obstacle1);
  frame.AddObstacle(obstacle2);
  auto proposals = frame.MutableProposals();
  proposals->emplace_back(proposal1);
  proposals->emplace_back(proposal2);

  lc_proposal = proposal1;
  lc_proposal.SetCorridorInfo(&pre_corridor_info);
  STProposal st_proposal;
  auto st_boundary_front = CreatObject(3.0, 10.0, 0.0, 4.0, 5.0);
  st_boundary_front.set_id("1");  
  st_proposal.emplace_back(std::make_tuple(&st_boundary_front, true, true));
  auto st_boundary_back = CreatObject(3.0, 5.0, 0.0, 4.0, 5.0);
  st_boundary_back.set_id("2");  
  st_proposal.emplace_back(std::make_tuple(&st_boundary_back, false, true));
  lc_proposal.SetSTProposal(st_proposal);
  proposals->emplace_back(lc_proposal);

  return std::make_tuple(evaluation_decider_ptr, frame_ptr, injector);
}

TEST(EvaluationDeciderTest, Execute) {
  auto res = PrepareFrame();
  auto &evaluation_decider = *std::get<0>(res);
  auto &frame = *std::get<1>(res);

  evaluation_decider.Execute(&frame);
  const auto &new_proposal1 = frame.GetProposals()[0];
  const auto &new_proposal2 = frame.GetProposals().at(1);

  EXPECT_DOUBLE_EQ(new_proposal1.GetCosts().at("safety").at(0).first, 10.0);
  EXPECT_EQ(new_proposal1.GetCosts().at("safety").at(0).second,
            "relative time: 0.000000s. s/l distance to obtacle dynamic: "
            "1.600000/0.500000");
  EXPECT_DOUBLE_EQ(new_proposal1.GetCosts().at("similarity").at(0).first,
                   0.125);
  EXPECT_EQ(new_proposal1.GetCosts().at("similarity").at(0).second,
            "average difference to previous trajectory is 0.500000m. ");
  EXPECT_DOUBLE_EQ(new_proposal1.GetCosts().at("lane_change").at(0).first, 0.5);
  EXPECT_EQ(new_proposal1.GetCosts().at("lane_change").at(0).second,
            "urgency cost ");
  EXPECT_DOUBLE_EQ(new_proposal1.GetCosts().at("lane_change").at(1).first, 0.5);
  EXPECT_EQ(new_proposal1.GetCosts().at("lane_change").at(1).second,
            "lane change abort cost ");
  EXPECT_DOUBLE_EQ(new_proposal1.GetCosts().at("Total").at(0).first, 11.125);
  EXPECT_EQ(new_proposal1.GetCosts().at("Total").at(0).second, "");
  EXPECT_DOUBLE_EQ(new_proposal2.GetCosts().at("safety").at(0).first, 10.0);
  EXPECT_EQ(new_proposal2.GetCosts().at("safety").at(0).second,
            "relative time: 0.000000s. s/l distance to obtacle dynamic: "
            "1.600000/1.000000");
  EXPECT_DOUBLE_EQ(new_proposal2.GetCosts().at("safety").at(1).first, 2.0);
  EXPECT_EQ(new_proposal2.GetCosts().at("safety").at(1).second,
            "stiff constraint violation in psi_dot");
  EXPECT_DOUBLE_EQ(new_proposal2.GetCosts().at("safety").at(2).first, 1.0);
  EXPECT_EQ(new_proposal2.GetCosts().at("safety").at(2).second,
            "stiff constraint violation in delta");
  EXPECT_DOUBLE_EQ(new_proposal2.GetCosts().at("safety").at(3).first, 3.0);
  EXPECT_EQ(new_proposal2.GetCosts().at("safety").at(3).second,
            "stiff constraint violation in v");
  EXPECT_DOUBLE_EQ(new_proposal2.GetCosts().at("lane_change").at(0).first, 2.0);
  EXPECT_EQ(new_proposal2.GetCosts().at("lane_change").at(0).second,
            "urgency cost ");
  EXPECT_DOUBLE_EQ(new_proposal2.GetCosts().at("lane_change").at(1).first, 0.5);
  EXPECT_EQ(new_proposal2.GetCosts().at("lane_change").at(1).second,
            "lane change abort cost ");
  EXPECT_DOUBLE_EQ(new_proposal2.GetCosts().at("Total").at(0).first, 12.5);
  EXPECT_EQ(new_proposal2.GetCosts().at("Total").at(0).second, "");

  // LCGap Test
  auto lc_gap = frame.GetLCGap();
  EXPECT_TRUE(lc_gap.has_gap);
  EXPECT_EQ(lc_gap.lc_front_hazard_id, 1);
  EXPECT_EQ(lc_gap.lc_back_hazard_id, 2);
}

TEST(EvaluationDeciderTest, ExecuteWhenLastFrameEmptyProposal) {
  // prepare
  auto res = PrepareFrame();
  auto &evaluation_decider = *std::get<0>(res);
  auto &frame = *std::get<1>(res);
  auto inject = std::get<2>(res);
  inject->frame_history()->Clear();

  // when
  evaluation_decider.Execute(&frame);

  // then (not crash)
  const auto &new_proposal1 = frame.GetProposals().front();
  EXPECT_EQ(new_proposal1.GetCosts().size(), 2);
}

TEST(EvaluationDeciderTest, ExecuteWhenProposalMaxDistanceZero) {
  // prepare
  constexpr auto kIdxS = 0u;
  auto res = PrepareFrame();
  auto &evaluation_decider = *std::get<0>(res);
  auto &frame = *std::get<1>(res);
  auto &inject = *std::get<2>(res);
  inject.frame_history()->Clear();
  auto &proposals = *frame.MutableProposals();
  proposals.pop_back();
  auto &proposal = proposals.front();
  auto l_mpc_data = proposal.GetLonMPCData();
  l_mpc_data.x.row(kIdxS).tail(1)(0) = 0;
  proposal.SetLonMPCData(l_mpc_data);

  // when
  const auto st = evaluation_decider.Execute(&frame);

  // then
  EXPECT_TRUE(st.ok());
}
