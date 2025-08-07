/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file proto_convertor_test.cc
 **/

#include "apps/planning/src/common/proto_convertor/proto_convertor.h"

#include "gtest/gtest.h"
#include "gtest/gtest_prod.h"
#include "status.h"

namespace zark {
namespace planning {
namespace common {

class ProtoConvertorTest : public ::testing::Test {
 public:
  ProtoConvertorTest() = default;

  void GenerateLocalRoutePoints(
      std::vector<LocalRoutePoint>& local_route_points) {
    for (double i = 0.0; i < 300.0; i = i + 0.5) {
      double x = i;
      double y = 0.0;
      double heading = 0.0;
      double kappa = 0.0;
      double dkappa = 0.0;
      local_route_points.emplace_back(LocalRoutePoint(
          hdmap::MapPathPoint(::math::Vec2d(x, y), heading), kappa, dkappa));
    }
  }
};

TEST_F(ProtoConvertorTest, TestRecordPlanningDebugInfo) {
  Proposal proposal;
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type = CorridorInfo::Type::LANE_CHANGE;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 155; i++) {
    CorridorPoint corridor_point;
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(1.0 * i);
    corridor_point.xy_ref = xy_point;
    xy_point.set_x(2.0);
    corridor_point.xy_left = xy_point;
    xy_point.set_x(-2.0);
    corridor_point.xy_right = xy_point;
    corridor_point.theta = M_PI / 2;
    corridor_point.kappa = 0.0;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info_proposla(local_route, mission, corridor_type,
                                      corridor, idx_start_point, obstacle_map);
  proposal.SetCorridorInfo(&corridor_info_proposla);
  STProposal st_proposal;
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;
  lower_points.emplace_back(10.0, 0.0);
  lower_points.emplace_back(20.0, 0.5);
  lower_points.emplace_back(30.0, 1.0);
  lower_points.emplace_back(40.0, 1.5);
  lower_points.emplace_back(50.0, 2.0);
  lower_points.emplace_back(60.0, 2.5);
  lower_points.emplace_back(70.0, 3.0);
  lower_points.emplace_back(80.0, 3.5);
  lower_points.emplace_back(90.0, 4.0);
  upper_points.emplace_back(25.0, 0.0);
  upper_points.emplace_back(35.0, 0.5);
  upper_points.emplace_back(45.0, 1.0);
  upper_points.emplace_back(55.0, 1.5);
  upper_points.emplace_back(65.0, 2.0);
  upper_points.emplace_back(75.0, 2.5);
  upper_points.emplace_back(85.0, 3.0);
  upper_points.emplace_back(95.0, 3.5);
  upper_points.emplace_back(105.0, 4.0);
  STBoundary boundary_1 =
      STBoundary::CreateInstanceAccurate(lower_points, upper_points, "0");
  lower_points.clear();
  upper_points.clear();
  lower_points.emplace_back(10.0, 1.0);
  lower_points.emplace_back(20.0, 1.5);
  lower_points.emplace_back(30.0, 2.0);
  upper_points.emplace_back(25.0, 1.0);
  upper_points.emplace_back(35.0, 1.5);
  upper_points.emplace_back(45.0, 2.0);
  STBoundary boundary_2 =
      STBoundary::CreateInstanceAccurate(lower_points, upper_points, "1");
  st_proposal.emplace_back(std::make_tuple(&boundary_1, true, false));
  st_proposal.emplace_back(std::make_tuple(&boundary_2, false, false));
  proposal.SetSTProposal(st_proposal);

  LonMPCData lon_mpc_data;
  const int n_nodes = 41;
  const int n_steps = 40;
  lon_mpc_data.n_steps = n_steps;
  MPCData::Names name;
  name.x = std::vector<std::string>{"s", "v"};
  name.u = std::vector<std::string>{"a"};
  name.u_dot = std::vector<std::string>{"j"};
  lon_mpc_data.names = name;
  Eigen::MatrixXd x = Eigen::MatrixXd::Constant(2, n_nodes, 3.0);
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Constant(2, n_nodes, 5.0);
  Eigen::MatrixXd u = Eigen::MatrixXd::Constant(1, n_steps, 7.0);
  Eigen::MatrixXd u_dot = Eigen::MatrixXd::Constant(1, n_steps, 9.0);
  Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(7);
  Eigen::MatrixXd t = Eigen::MatrixXd::Constant(1, n_nodes, 0.0);
  for (int i = 0; i < n_nodes; i++) {
    t(i) = i * 0.2;
  }
  lon_mpc_data.x = x;
  lon_mpc_data.x_ref = x_ref;
  lon_mpc_data.u = u;
  lon_mpc_data.u_ref = u;
  lon_mpc_data.u_dot = u_dot;
  lon_mpc_data.t = t;
  lon_mpc_data.u_prev = u_prev;
  lon_mpc_data.dt = 0.2;
  lon_mpc_data.dt_prev = 0.15;
  MPCData::Constraints constraint;
  constraint.soft_max.x.constraint =
      Eigen::MatrixXd::Constant(2, n_nodes, 20.0);
  constraint.soft_max.x.idx_enabled = std::vector<bool>(n_nodes, true);
  constraint.soft_min.x.constraint =
      Eigen::MatrixXd::Constant(2, n_nodes, 15.0);
  constraint.soft_min.x.idx_enabled = std::vector<bool>(n_nodes, true);
  constraint.hard_max.x.constraint =
      Eigen::MatrixXd::Constant(2, n_nodes, 40.0);
  constraint.hard_max.x.idx_enabled = std::vector<bool>(n_nodes, true);
  constraint.hard_min.x.constraint = Eigen::MatrixXd::Constant(2, n_nodes, 0.1);
  constraint.hard_min.x.idx_enabled = std::vector<bool>(n_nodes, true);
  constraint.stiff_max.x.constraint =
      Eigen::MatrixXd::Constant(2, n_nodes, 30.0);
  constraint.stiff_max.x.idx_enabled = std::vector<bool>(n_nodes, true);
  constraint.stiff_min.x.constraint =
      Eigen::MatrixXd::Constant(2, n_nodes, 10.0);
  constraint.stiff_min.x.idx_enabled = std::vector<bool>(n_nodes, true);
  constraint.soft_max.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 20.0);
  constraint.soft_max.u.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.soft_min.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 15.0);
  constraint.soft_min.u.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.hard_max.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 40.0);
  constraint.hard_max.u.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.hard_min.u.constraint = Eigen::MatrixXd::Constant(1, n_steps, 0.1);
  constraint.hard_min.u.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.stiff_max.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 30.0);
  constraint.stiff_max.u.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.stiff_min.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 10.0);
  constraint.stiff_min.u.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.soft_max.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 20.0);
  constraint.soft_max.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.soft_min.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 15.0);
  constraint.soft_min.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.hard_max.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 40.0);
  constraint.hard_max.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.hard_min.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 0.1);
  constraint.hard_min.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.stiff_max.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 30.0);
  constraint.stiff_max.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  constraint.stiff_min.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 10.0);
  constraint.stiff_min.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  lon_mpc_data.constraints = constraint;
  MPCData::Slacks slack;
  slack.soft_max.x = constraint.soft_max.x.constraint;
  slack.soft_min.x = constraint.soft_min.x.constraint;
  slack.stiff_max.x = constraint.stiff_max.x.constraint;
  slack.stiff_min.x = constraint.stiff_min.x.constraint;
  slack.soft_max.u = constraint.soft_max.u.constraint;
  slack.soft_min.u = constraint.soft_min.u.constraint;
  slack.stiff_max.u = constraint.stiff_max.u.constraint;
  slack.stiff_min.u = constraint.stiff_min.u.constraint;
  slack.soft_max.u_dot = constraint.soft_max.u_dot.constraint;
  slack.soft_min.u_dot = constraint.soft_min.u_dot.constraint;
  slack.stiff_max.u_dot = constraint.stiff_max.u_dot.constraint;
  slack.stiff_min.u_dot = constraint.stiff_min.u_dot.constraint;
  lon_mpc_data.slacks = slack;
  SpeedLimit speed_limit_1, speed_limit_2;
  for (int i = 0; i < n_nodes; ++i) {
    speed_limit_1.AppendSpeedLimit(0.0 + i * 2.5, 5.0);
    speed_limit_2.AppendSpeedLimit(0.0 + i * 2.5, 10.0);
  }
  std::vector<SpeedLimit> speed_limit_set;
  speed_limit_set.emplace_back(speed_limit_1);
  speed_limit_set.emplace_back(speed_limit_2);
  lon_mpc_data.speed_limit_set = speed_limit_set;
  proposal.SetLonMPCData(lon_mpc_data);

  LatMPCData lat_data;
  name.x = std::vector<std::string>{"l", "l_dot", "psi_s", "psi_s_dot"};
  name.u = std::vector<std::string>{"delta"};
  name.u_dot = std::vector<std::string>{"delta_dot"};
  name.u_2 = std::vector<std::string>{"psi_dot_des"};
  lat_data.names = name;
  x = Eigen::MatrixXd::Zero(4, n_nodes);
  x_ref = Eigen::MatrixXd::Zero(4, n_nodes);
  u = Eigen::MatrixXd::Constant(1, n_steps, 0.0);
  u_dot = Eigen::MatrixXd::Constant(1, n_steps, 0.0);
  u_prev = Eigen::VectorXd::Zero(1);
  t = Eigen::MatrixXd::Constant(1, n_nodes, 0.0);
  for (int i = 0; i < n_nodes; ++i) {
    x(0, i) = 5.0;
    x(1, i) = 10.0;
    x_ref(0, i) = 6.0;
    t(i) = i * 0.2;
  }
  for (int i = 0; i < n_steps; ++i) {
    x(2, i) = 0.5;
    x(3, i) = 1.0;
  }

  lat_data.x = x;
  lat_data.x_ref = x_ref;
  lat_data.u = u;
  lat_data.u_ref = u;
  lat_data.u_dot = u_dot;
  lat_data.t = t;
  lat_data.u_prev = u_prev;
  lat_data.dt = 0.1;
  lat_data.dt_prev = 0.1;
  lat_data.n_steps = n_steps;
  MPCData::Constraints lat_constraint;
  lat_constraint.soft_max.x.constraint =
      Eigen::MatrixXd::Constant(4, n_nodes, 20.0);
  lat_constraint.soft_max.x.idx_enabled = std::vector<bool>(n_nodes, true);
  lat_constraint.soft_min.x.constraint =
      Eigen::MatrixXd::Constant(4, n_nodes, 15.0);
  lat_constraint.soft_min.x.idx_enabled = std::vector<bool>(n_nodes, true);
  lat_constraint.hard_max.x.constraint =
      Eigen::MatrixXd::Constant(4, n_nodes, 40.0);
  lat_constraint.hard_max.x.idx_enabled = std::vector<bool>(n_nodes, true);
  lat_constraint.hard_min.x.constraint =
      Eigen::MatrixXd::Constant(4, n_nodes, 0.1);
  lat_constraint.hard_min.x.idx_enabled = std::vector<bool>(n_nodes, true);
  lat_constraint.stiff_max.x.constraint =
      Eigen::MatrixXd::Constant(4, n_nodes, 30.0);
  lat_constraint.stiff_max.x.idx_enabled = std::vector<bool>(n_nodes, true);
  lat_constraint.stiff_min.x.constraint =
      Eigen::MatrixXd::Constant(4, n_nodes, 10.0);
  lat_constraint.stiff_min.x.idx_enabled = std::vector<bool>(n_nodes, true);
  lat_constraint.soft_max.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 1.5);
  lat_constraint.soft_max.u.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.soft_min.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 0.5);
  lat_constraint.soft_min.u.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.hard_max.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 2.5);
  lat_constraint.hard_max.u.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.hard_min.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 0.1);
  lat_constraint.hard_min.u.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.stiff_max.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 2.0);
  lat_constraint.stiff_max.u.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.stiff_min.u.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 0.3);
  lat_constraint.stiff_min.u.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.soft_max.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 2.0);
  lat_constraint.soft_max.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.soft_min.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 0.5);
  lat_constraint.soft_min.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.hard_max.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 2.5);
  lat_constraint.hard_max.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.hard_min.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 0.1);
  lat_constraint.hard_min.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.stiff_max.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 2.3);
  lat_constraint.stiff_max.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  lat_constraint.stiff_min.u_dot.constraint =
      Eigen::MatrixXd::Constant(1, n_steps, 0.2);
  lat_constraint.stiff_min.u_dot.idx_enabled = std::vector<bool>(n_steps, true);
  lat_data.constraints = lat_constraint;
  slack.soft_max.x = lat_constraint.soft_max.x.constraint;
  slack.soft_min.x = lat_constraint.soft_min.x.constraint;
  slack.stiff_max.x = lat_constraint.stiff_max.x.constraint;
  slack.stiff_min.x = lat_constraint.stiff_min.x.constraint;
  slack.soft_max.u = lat_constraint.soft_max.u.constraint;
  slack.soft_min.u = lat_constraint.soft_min.u.constraint;
  slack.stiff_max.u = lat_constraint.stiff_max.u.constraint;
  slack.stiff_min.u = lat_constraint.stiff_min.u.constraint;
  slack.soft_max.u_dot = lat_constraint.soft_max.u_dot.constraint;
  slack.soft_min.u_dot = lat_constraint.soft_min.u_dot.constraint;
  slack.stiff_max.u_dot = lat_constraint.stiff_max.u_dot.constraint;
  slack.stiff_min.u_dot = lat_constraint.stiff_min.u_dot.constraint;
  lat_data.slacks = slack;
  Tube res_tube;
  for (int k = 0; k < n_nodes; k++) {
    Tube::TubePoint pts;
    ::math::Vec2d xy_left_ref_(0. + k, 0.);
    ::math::Vec2d xy_right_ref_(0. + k, 0.);
    ::math::Vec2d xy_left_soft_(0. + k, 1.);
    ::math::Vec2d xy_right_soft_(0. + k, -1.);
    ::math::Vec2d xy_left_stiff_(0. + k, 1.5);
    ::math::Vec2d xy_right_stiff_(0. + k, -1.5);
    ::math::Vec2d xy_left_hard_(0. + k, 2.);
    ::math::Vec2d xy_right_hard_(0. + k, -2.);
    pts.t = k;
    pts.xy_left_ref = xy_left_ref_;
    pts.xy_right_ref = xy_right_ref_;
    pts.xy_left_soft = xy_left_soft_;
    pts.xy_right_soft = xy_right_soft_;
    pts.xy_left_stiff = xy_left_stiff_;
    pts.xy_right_stiff = xy_right_stiff_;
    pts.xy_left_hard = xy_left_hard_;
    pts.xy_right_hard = xy_right_hard_;
    res_tube.pts.emplace_back(pts);
  }
  lat_data.tube = res_tube;
  proposal.SetLatMPCData(lat_data);

  auto cost_table1 = proposal.GetMutableCosts();
  cost_table1->emplace(
      "safety",
      Costs({std::make_pair(
          10.0,
          "relative time: 4.400000s. min distance to obtacle: 0.000000")}));
  cost_table1->emplace(
      "similarity",
      Costs({std::make_pair(
          0.125, "average difference to previous trajectory is 0.500000m.")}));
  cost_table1->emplace("total", Costs({std::make_pair(10.125, "")}));

  DiscretizedTrajectory trajectory;
  const double init_s = 0.0;
  const double init_v = 10.0;
  const double init_a = -2.0;
  const double delta_t = 0.2;
  for (int i = 0; i < n_nodes; ++i) {
    TrajectoryPoint trajectory_point;
    ::common::PathPoint path_point;
    const double time = delta_t * i;
    trajectory_point.set_relative_time(time);
    trajectory_point.set_v(init_v + init_a * time);
    trajectory_point.set_a(init_a);
    path_point.set_s(init_s + init_v * time + 0.5 * init_a * time * time);
    path_point.set_x(init_s + init_v * time + 0.5 * init_a * time * time);
    path_point.set_y(0.0);
    path_point.set_theta(0.5 * M_PI_2);
    path_point.set_kappa(0.0);
    trajectory_point.set_path_point(path_point);
    trajectory.emplace_back(trajectory_point);
  }
  proposal.SetTrajectory(trajectory);

  std::vector<CorridorInfo> corridor_infos;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);

  std::unordered_map<std::string, SpeedLimit> speed_limit_map;
  SpeedLimit speed_limit_3;
  speed_limit_3.AppendSpeedLimit(149., 1.0);
  speed_limit_3.AppendSpeedLimit(150., 2.0);
  speed_limit_map["map"] = speed_limit_1;
  speed_limit_map["curvature"] = speed_limit_2;
  speed_limit_map["final"] = speed_limit_3;
  corridor_info.SetSpeedLimitMap(speed_limit_map);

  CorridorInfo::STTopology st_topology;
  std::vector<std::string> layer;
  layer.emplace_back("0");
  layer.emplace_back("1");
  st_topology.layers.emplace_back(layer);
  layer.clear();
  layer.emplace_back("2");
  layer.emplace_back("3");
  layer.emplace_back("4");
  st_topology.layers.emplace_back(layer);
  corridor_info.SetSTTopology(st_topology);

  std::vector<CorridorInfo::STProposal> st_proposals;
  CorridorInfo::STProposal cor_st_proposal;
  cor_st_proposal.cost_total = 10.0;
  CorridorInfo::STProposal::STBoundary st_boundary;
  CorridorInfo::STProposal::Cost cost;
  st_boundary.obs_id = "0";
  st_boundary.is_front = true;
  cost.reason = "0, front";
  cost.cost = 1.0;
  cor_st_proposal.st_boundaries.emplace_back(st_boundary);
  cor_st_proposal.costs.emplace_back(cost);
  st_boundary.obs_id = "1";
  st_boundary.is_front = false;
  cost.reason = "1, rear";
  cost.cost = 3.0;
  cor_st_proposal.st_boundaries.emplace_back(st_boundary);
  cor_st_proposal.costs.emplace_back(cost);
  st_proposals.emplace_back(cor_st_proposal);
  cor_st_proposal.cost_total = 12.0;
  cor_st_proposal.st_boundaries.clear();
  cor_st_proposal.costs.clear();
  st_boundary.obs_id = "0";
  st_boundary.is_front = true;
  cost.reason = "0, front";
  cost.cost = 1.0;
  cor_st_proposal.st_boundaries.emplace_back(st_boundary);
  cor_st_proposal.costs.emplace_back(cost);
  st_boundary.obs_id = "1";
  st_boundary.is_front = true;
  cost.reason = "1, front";
  cost.cost = 4.0;
  cor_st_proposal.st_boundaries.emplace_back(st_boundary);
  cor_st_proposal.costs.emplace_back(cost);
  st_proposals.emplace_back(cor_st_proposal);
  corridor_info.SetSTProposals(st_proposals);

  corridor_infos.emplace_back(corridor_info);

  std::vector<Proposal> proposals;
  proposals.emplace_back(proposal);
  proposals.emplace_back(proposal);

  mission.lc_request = Mission::LaneChangeRequest::LC_LEFT;
  mission.lc_type = Mission::LaneChangeType::BLOCKING_LC;
  mission.lc_urgency = 2.0;
  mission.is_take_over = true;
  mission.is_lc_ready = true;
  mission.is_lc_abort = true;

  EnvInfos env_infos;
  env_infos.vehicle_infos.cur_adc_v = 12.3;
  env_infos.vehicle_infos.driver_set_v = 23.4;

  LatencyStats latency_stats;
  TaskStats* p_task_stats = latency_stats.add_task_stats();
  p_task_stats->set_name("PAINFUL_TASK");
  p_task_stats->set_time_ms(1000);

  bool need_replan = true;
  std::string replan_reason = "replan";

  PlanningDebugInfo proposal_proto;
  ProtoConvertor::RecordPlanningDebugInfo(
      corridor_infos, proposals, &corridor_info, mission, latency_stats,
      need_replan, replan_reason, proposal_proto);

  EXPECT_EQ(proposal_proto.proposals().size(), 2);
  EXPECT_EQ(proposal_proto.corridor_infos().size(), 1);
  EXPECT_EQ(proposal_proto.task_latencies_size(), 1);
  EXPECT_TRUE(proposal_proto.is_replan());
  EXPECT_STREQ("replan", proposal_proto.replan_reason().c_str());
}

TEST_F(ProtoConvertorTest, TestRecordPlanningOutput) {
  zark::planning::ADCTrajectory trajectory;
  trajectory.mutable_header()->set_timestamp_sec(1.0e-9);
  trajectory.mutable_header()->set_frame_id("key_frame");
  trajectory.mutable_header()->set_sequence_num(99999);
  trajectory.mutable_header()->mutable_status()->set_error_code(
      ::common::ErrorCode::PLANNING_ERROR);
  trajectory.mutable_header()->mutable_status()->set_msg("status message");
  for (int i = 0; i < 100; i++) {
    ::common::TrajectoryPoint* p = trajectory.add_trajectory_point();
    ::common::PathPoint path_point;
    path_point.set_x(1.0);
    path_point.set_y(2.0);
    path_point.set_z(3.0);
    path_point.set_s(4.0);
    path_point.set_theta(5.0);
    path_point.set_kappa(6.0);
    path_point.set_dkappa(7.0);
    p->set_path_point(path_point);
    p->set_a(8.0);
    p->set_v(9.0);
    p->set_relative_time(10.0);
  }
  PolynomialFit::TrajPolynomial traj_polyfit_out;
  traj_polyfit_out.lon_coefficients = Eigen::VectorXd(6);
  traj_polyfit_out.lon_coefficients << 0, 1, 2, 3, 4, 5;
  traj_polyfit_out.lat_coefficients = Eigen::VectorXd(6);
  traj_polyfit_out.lat_coefficients << 0, -1, -2, -3, -4, -5;
  traj_polyfit_out.length = 6;
  traj_polyfit_out.is_valid = true;
  trajectory.set_traj_polyfit_out(traj_polyfit_out);

  zark::planning::PlanningOutputMsg planning_output_proto;
  ProtoConvertor::RecordPlanningOutput(trajectory, planning_output_proto);

  auto& header = planning_output_proto.header();
  EXPECT_DOUBLE_EQ(header.common_header().timestamp_nano(), 1.0);
  EXPECT_EQ(header.common_header().frame_id(), "key_frame");
  EXPECT_EQ(header.common_header().sequence_num(), 9);
  EXPECT_EQ(header.version(), 1);
  EXPECT_EQ(header.module_name(), "Planning");
  EXPECT_EQ(header.status().error_code(), ::common::ErrorCode::PLANNING_ERROR);
  EXPECT_EQ(header.status().msg(), "status message");
  EXPECT_EQ(planning_output_proto.trajectory_published().size(), 100);
  auto& trajectory_point = planning_output_proto.trajectory_published()[50];
  EXPECT_DOUBLE_EQ(trajectory_point.point().x(), 1.0);
  EXPECT_DOUBLE_EQ(trajectory_point.point().y(), 2.0);
  EXPECT_DOUBLE_EQ(trajectory_point.point().z(), 3.0);
  EXPECT_DOUBLE_EQ(trajectory_point.s(), 4.0);
  EXPECT_DOUBLE_EQ(trajectory_point.v(), 9.0);
  EXPECT_DOUBLE_EQ(trajectory_point.a(), 8.0);
  EXPECT_DOUBLE_EQ(trajectory_point.theta(), 5.0);
  EXPECT_DOUBLE_EQ(trajectory_point.kappa(), 6.0);
  EXPECT_DOUBLE_EQ(trajectory_point.dkappa(), 7.0);
  EXPECT_DOUBLE_EQ(trajectory_point.t(), 10.0);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().is_valid(), true);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().traj_length(), 6);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().lon_fit_coeff().size(),
            6);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().lat_fit_coeff().size(),
            6);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().lon_fit_coeff()[3], 3);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().lat_fit_coeff()[3], -3);
}

TEST_F(ProtoConvertorTest, TestRecordLocalRoute) {
  std::list<LocalRoute> local_routes;
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(local_route_points);
  auto local_route = LocalRoute(local_route_points, LocalRouteConfig());
  local_route.SetIsTargetRoute(false);
  auto local_route_target = LocalRoute(local_route_points, LocalRouteConfig());
  local_route_target.SetIsTargetRoute(true);

  local_routes.emplace_back(std::move(local_route));
  local_routes.emplace_back(std::move(local_route_target));

  zark::reference_line_proto::ReferenceLines ref_lines;

  ProtoConvertor::RecordLocalRoute(local_routes, ref_lines);

  EXPECT_EQ(ref_lines.reference_lines_size(), 2);
  EXPECT_EQ(ref_lines.reference_lines()[0].path_points().size(), 600);
  EXPECT_EQ(ref_lines.reference_lines()[1].path_points().size(), 600);
  EXPECT_EQ(ref_lines.target_line_id(), 2);
}

TEST_F(ProtoConvertorTest, TestRecordLatencyStatsProto) {
  LatencyStats latency_stats;
  auto task_stats = latency_stats.add_task_stats();
  task_stats->set_name("task1");
  task_stats->set_time_ms(0.01);
  PlanningDebugInfo planning_debug_info_proto;

  ProtoConvertor::RecordLatencyStatsProto(latency_stats,
                                          planning_debug_info_proto);

  EXPECT_EQ(planning_debug_info_proto.task_latencies().size(), 1);
  EXPECT_EQ(planning_debug_info_proto.task_latencies()[0].task_name(), "task1");
  EXPECT_EQ(planning_debug_info_proto.task_latencies()[0].latency(), 0.01);
}

TEST_F(ProtoConvertorTest, TestRecordMissionProto) {
  Mission mission;
  mission.lc_request = Mission::LaneChangeRequest::LC_LEFT;
  mission.lc_type = Mission::LaneChangeType::BLOCKING_LC;
  mission.lc_urgency = 2.0;
  mission.is_take_over = true;
  mission.is_lc_ready = true;
  mission.is_lc_abort = true;

  mission.env_infos.vehicle_infos.cur_adc_v = 12.3;
  mission.env_infos.vehicle_infos.driver_set_v = 23.4;

  PlanningDebugInfo planning_debug_info;
  ProtoConvertor::RecordMissionProto(mission, planning_debug_info);

  EXPECT_EQ(planning_debug_info.mission().lc_type(),
            PlanningDebugInfo::Mission::BLOCKING_LC);
  EXPECT_EQ(planning_debug_info.mission().lc_urgency(), 2.0);
  EXPECT_EQ(planning_debug_info.mission().env_infos().cur_adc_v(), 12.3);
}

TEST_F(ProtoConvertorTest, TestRecordCorridorInfosProto) {
  std::vector<CorridorInfo> corridor_infos;
  PlanningDebugInfo planning_debug_info;
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type = CorridorInfo::Type::LANE_CHANGE;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    CorridorPoint corridor_point;
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(1.0 * i);
    corridor_point.xy_ref = xy_point;
    xy_point.set_x(2.0);
    corridor_point.xy_left = xy_point;
    xy_point.set_x(-2.0);
    corridor_point.xy_right = xy_point;
    corridor_point.theta = M_PI / 2;
    corridor_point.kappa = 0.0;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);

  std::unordered_map<std::string, SpeedLimit> speed_limit_map;
  SpeedLimit speed_limit_1;
  speed_limit_1.AppendSpeedLimit(0, 10.0);
  speed_limit_1.AppendSpeedLimit(15.0, 20.0);
  SpeedLimit speed_limit_2;
  speed_limit_2.AppendSpeedLimit(0, 5.0);
  speed_limit_2.AppendSpeedLimit(15.0, 3.0);
  speed_limit_map["map"] = speed_limit_1;
  speed_limit_map["curvature"] = speed_limit_2;
  corridor_info.SetSpeedLimitMap(speed_limit_map);

  CorridorInfo::STTopology st_topology;
  std::vector<std::string> layer;
  layer.emplace_back("0");
  layer.emplace_back("1");
  st_topology.layers.emplace_back(layer);
  layer.clear();
  layer.emplace_back("2");
  layer.emplace_back("3");
  layer.emplace_back("4");
  st_topology.layers.emplace_back(layer);
  corridor_info.SetSTTopology(st_topology);

  std::vector<CorridorInfo::STProposal> st_proposals;
  CorridorInfo::STProposal st_proposal;
  st_proposal.cost_total = 10.0;
  CorridorInfo::STProposal::STBoundary st_boundary;
  CorridorInfo::STProposal::Cost cost;
  st_boundary.obs_id = "0";
  st_boundary.is_front = true;
  cost.reason = "0, front";
  cost.cost = 1.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_boundary.obs_id = "1";
  st_boundary.is_front = false;
  cost.reason = "1, rear";
  cost.cost = 3.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_proposals.emplace_back(st_proposal);
  st_proposal.cost_total = 12.0;
  st_proposal.st_boundaries.clear();
  st_proposal.costs.clear();
  st_boundary.obs_id = "0";
  st_boundary.is_front = true;
  cost.reason = "0, front";
  cost.cost = 1.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_boundary.obs_id = "1";
  st_boundary.is_front = true;
  cost.reason = "1, front";
  cost.cost = 4.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_proposals.emplace_back(st_proposal);
  corridor_info.SetSTProposals(st_proposals);

  corridor_infos.emplace_back(corridor_info);
  corridor_infos.emplace_back(corridor_info);
  corridor_infos.emplace_back(corridor_info);
  corridor_infos.emplace_back(corridor_info);

  ProtoConvertor::RecordCorridorInfosProto(corridor_infos, planning_debug_info);

  EXPECT_EQ(planning_debug_info.corridor_infos().size(), 4);
}

TEST_F(ProtoConvertorTest, TestRecordProposalsProto) {
  std::vector<Proposal> proposals;
  PlanningDebugInfo planning_debug_info;
  Proposal proposal;
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type = CorridorInfo::Type::LANE_KEEP;
  Corridor corridor;
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);
  proposal.SetCorridorInfo(&corridor_info);

  STProposal st_proposal;
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;
  lower_points.emplace_back(10.0, 0.0);
  lower_points.emplace_back(13.0, 10.0);
  upper_points.emplace_back(15.0, 0.0);
  upper_points.emplace_back(18.0, 10.0);
  STBoundary boundary_1 =
      STBoundary::CreateInstanceAccurate(lower_points, upper_points, "0");
  lower_points.clear();
  upper_points.clear();
  lower_points.emplace_back(0.0, 0.0);
  lower_points.emplace_back(0.0, 10.0);
  upper_points.emplace_back(5.0, 0.0);
  upper_points.emplace_back(5.0, 10.0);
  STBoundary boundary_2 =
      STBoundary::CreateInstanceAccurate(lower_points, upper_points, "1");
  st_proposal.emplace_back(std::make_tuple(&boundary_1, true, false));
  st_proposal.emplace_back(std::make_tuple(&boundary_2, false, false));
  proposal.SetSTProposal(st_proposal);

  LonMPCData lon_mpc_data;
  const int step = 10;
  lon_mpc_data.n_steps = step - 1;
  MPCData::Names name;
  name.x = std::vector<std::string>{"s", "v"};
  name.u = std::vector<std::string>{"a"};
  name.u_dot = std::vector<std::string>{"j"};
  lon_mpc_data.names = name;
  Eigen::MatrixXd x = Eigen::MatrixXd::Constant(2, step, 1.0);
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Constant(2, step, 1.0);
  Eigen::MatrixXd u = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::MatrixXd u_dot = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd t = Eigen::MatrixXd::Constant(1, step, 0.0);
  for (int i = 0; i < step; i++) {
    x(0, i) += i * 1.;
    x_ref(0, i) += i * 1.1;
    t(i) = i;
  }
  lon_mpc_data.x = x;
  lon_mpc_data.x_ref = x_ref;
  lon_mpc_data.u = u;
  lon_mpc_data.u_ref = u;
  lon_mpc_data.u_dot = u_dot;
  lon_mpc_data.t = t;
  lon_mpc_data.u_prev = u_prev;
  lon_mpc_data.dt = 0.1;
  lon_mpc_data.dt_prev = 0.1;
  MPCData::Constraints constraint;
  constraint.soft_max.x.constraint = x.row(0);
  constraint.soft_max.x.idx_enabled = {true, false};
  constraint.soft_min.x.constraint = x.row(0);
  constraint.soft_min.x.idx_enabled = {true, false};
  constraint.hard_max.x.constraint = x.row(0);
  constraint.hard_max.x.idx_enabled = {true, false};
  constraint.hard_min.x.constraint = x.row(0);
  constraint.hard_min.x.idx_enabled = {true, false};
  constraint.stiff_max.x.constraint = x.row(1);
  constraint.stiff_max.x.idx_enabled = {false, true};
  constraint.stiff_min.x.constraint = x.row(1);
  constraint.stiff_min.x.idx_enabled = {false, true};
  constraint.soft_max.u.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u.idx_enabled = {false};
  constraint.soft_min.u.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u.idx_enabled = {false};
  constraint.hard_max.u.constraint = Eigen::RowVectorXd();
  constraint.hard_max.u.idx_enabled = {false};
  constraint.hard_min.u.constraint = Eigen::RowVectorXd();
  constraint.hard_min.u.idx_enabled = {false};
  constraint.stiff_max.u.constraint = u;
  constraint.stiff_max.u.idx_enabled = {true};
  constraint.stiff_min.u.constraint = u;
  constraint.stiff_min.u.idx_enabled = {true};
  constraint.soft_max.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u_dot.idx_enabled = {false};
  constraint.soft_min.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u_dot.idx_enabled = {false};
  constraint.hard_max.u_dot.constraint = u_dot;
  constraint.hard_max.u_dot.idx_enabled = {true};
  constraint.hard_min.u_dot.constraint = u_dot;
  constraint.hard_min.u_dot.idx_enabled = {true};
  constraint.stiff_max.u_dot.constraint = u_dot;
  constraint.stiff_max.u_dot.idx_enabled = {true};
  constraint.stiff_min.u_dot.constraint = u_dot;
  constraint.stiff_min.u_dot.idx_enabled = {true};
  lon_mpc_data.constraints = constraint;
  MPCData::Slacks slack;
  slack.soft_max.x = x;
  slack.soft_min.x = x;
  slack.stiff_max.x = x;
  slack.stiff_min.x = x;
  slack.soft_max.u = u;
  slack.soft_min.u = u;
  slack.stiff_max.u = u;
  slack.stiff_min.u = u;
  slack.soft_max.u_dot = u_dot;
  slack.soft_min.u_dot = u_dot;
  slack.stiff_max.u_dot = u_dot;
  slack.stiff_min.u_dot = u_dot;
  lon_mpc_data.slacks = slack;
  SpeedLimit speed_limit;
  speed_limit.AppendSpeedLimit(0.0, 5.7446);
  speed_limit.AppendSpeedLimit(4.0, 5.0);
  speed_limit.AppendSpeedLimit(10.0, 5.0);
  speed_limit.AppendSpeedLimit(20.0, 5.0);
  speed_limit.AppendSpeedLimit(30.0, 5.0);
  speed_limit.AppendSpeedLimit(40.0, 5.0);
  speed_limit.AppendSpeedLimit(50.0, 5.0);
  std::vector<SpeedLimit> speed_limit_set;
  speed_limit_set.emplace_back(speed_limit);
  speed_limit_set.emplace_back(speed_limit);
  lon_mpc_data.speed_limit_set = speed_limit_set;
  proposal.SetLonMPCData(lon_mpc_data);

  LatMPCData data;
  name.x = std::vector<std::string>{"l", "l_dot", "psi_s", "psi_s_dot"};
  name.u = std::vector<std::string>{"delta"};
  name.u_dot = std::vector<std::string>{"delta_dot"};
  name.u_2 = std::vector<std::string>{"psi_dot_des"};
  data.names = name;
  x = Eigen::MatrixXd::Zero(4, step);
  x_ref = Eigen::MatrixXd::Zero(4, step);
  u = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  u_dot = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  u_prev = Eigen::VectorXd::Zero(1);
  t = Eigen::MatrixXd::Constant(1, step, 0.0);
  for (int i = 0; i < step; i++) {
    x(0, i) = 1.;
    x_ref(0, i) = 1.1;
    t(i) = i;
  }
  data.x = x;
  data.x_ref = x_ref;
  data.u = u;
  data.u_ref = u;
  data.u_dot = u_dot;
  data.t = t;
  data.u_prev = u_prev;
  data.dt = 0.1;
  data.dt_prev = 0.1;
  data.n_steps = step - 1;
  constraint.soft_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.soft_max.x.idx_enabled = {true, false, true, false};
  constraint.soft_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.soft_min.x.idx_enabled = {true, false, true, false};
  constraint.hard_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.hard_max.x.idx_enabled = {true, true, false, false};
  constraint.hard_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.hard_min.x.idx_enabled = {true, true, false, false};
  constraint.stiff_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.stiff_max.x.idx_enabled = {true, false, false, true};
  constraint.stiff_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.stiff_min.x.idx_enabled = {true, false, false, true};
  constraint.soft_max.u.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u.idx_enabled = {false};
  constraint.soft_min.u.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u.idx_enabled = {false};
  constraint.hard_max.u.constraint = Eigen::RowVectorXd();
  constraint.hard_max.u.idx_enabled = {false};
  constraint.hard_min.u.constraint = Eigen::RowVectorXd();
  constraint.hard_min.u.idx_enabled = {false};
  constraint.stiff_max.u.constraint = u;
  constraint.stiff_max.u.idx_enabled = {true};
  constraint.stiff_min.u.constraint = u;
  constraint.stiff_min.u.idx_enabled = {true};
  constraint.soft_max.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u_dot.idx_enabled = {false};
  constraint.soft_min.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u_dot.idx_enabled = {false};
  constraint.hard_max.u_dot.constraint = u_dot;
  constraint.hard_max.u_dot.idx_enabled = {true};
  constraint.hard_min.u_dot.constraint = u_dot;
  constraint.hard_min.u_dot.idx_enabled = {true};
  constraint.stiff_max.u_dot.constraint = u_dot;
  constraint.stiff_max.u_dot.idx_enabled = {true};
  constraint.stiff_min.u_dot.constraint = u_dot;
  constraint.stiff_min.u_dot.idx_enabled = {true};
  data.constraints = constraint;
  slack.soft_max.x = x;
  slack.soft_min.x = x;
  slack.stiff_max.x = x;
  slack.stiff_min.x = x;
  slack.soft_max.u = u;
  slack.soft_min.u = u;
  slack.stiff_max.u = u;
  slack.stiff_min.u = u;
  slack.soft_max.u_dot = u_dot;
  slack.soft_min.u_dot = u_dot;
  slack.stiff_max.u_dot = u_dot;
  slack.stiff_min.u_dot = u_dot;
  data.slacks = slack;
  Tube res_tube;
  for (int k = 0; k < step; k++) {
    Tube::TubePoint pts;
    ::math::Vec2d xy_left_ref_(0. + k, 0.);
    ::math::Vec2d xy_right_ref_(0. + k, 0.);
    ::math::Vec2d xy_left_soft_(0. + k, 1.);
    ::math::Vec2d xy_right_soft_(0. + k, -1.);
    ::math::Vec2d xy_left_stiff_(0. + k, 1.5);
    ::math::Vec2d xy_right_stiff_(0. + k, -1.5);
    ::math::Vec2d xy_left_hard_(0. + k, 2.);
    ::math::Vec2d xy_right_hard_(0. + k, -2.);
    pts.t = k;
    pts.xy_left_ref = xy_left_ref_;
    pts.xy_right_ref = xy_right_ref_;
    pts.xy_left_soft = xy_left_soft_;
    pts.xy_right_soft = xy_right_soft_;
    pts.xy_left_stiff = xy_left_stiff_;
    pts.xy_right_stiff = xy_right_stiff_;
    pts.xy_left_hard = xy_left_hard_;
    pts.xy_right_hard = xy_right_hard_;
    res_tube.pts.emplace_back(pts);
  }
  data.tube = res_tube;
  proposal.SetLatMPCData(data);

  auto cost_table1 = proposal.GetMutableCosts();
  cost_table1->emplace(
      "safety",
      Costs({std::make_pair(
          10.0,
          "relative time: 4.400000s. min distance to obtacle: 0.000000")}));
  cost_table1->emplace(
      "similarity",
      Costs({std::make_pair(
          0.125, "average difference to previous trajectory is 0.500000m.")}));
  cost_table1->emplace("total", Costs({std::make_pair(10.125, "")}));

  DiscretizedTrajectory trajectory;
  const int num_point = 10;
  const double init_s = 0.0;
  const double init_v = 10.0;
  const double init_a = -2.0;
  const double delta_t = 0.1;
  for (int i = 0; i < num_point; ++i) {
    TrajectoryPoint trajectory_point;
    ::common::PathPoint path_point;
    const double time = delta_t * i;
    trajectory_point.set_relative_time(time);
    trajectory_point.set_v(init_v + init_a * time);
    trajectory_point.set_a(init_a);
    path_point.set_s(init_s + init_v * time + 0.5 * init_a * time * time);
    path_point.set_x(init_s + init_v * time + 0.5 * init_a * time * time);
    path_point.set_y(0.0);
    path_point.set_theta(0.5 * M_PI_2);
    path_point.set_kappa(0.0);
    trajectory_point.set_path_point(path_point);
    trajectory.emplace_back(trajectory_point);
  }
  proposal.SetTrajectory(trajectory);

  proposals.emplace_back(proposal);
  proposals.emplace_back(proposal);
  proposals.emplace_back(proposal);

  ProtoConvertor::RecordProposalsProto(proposals, planning_debug_info);

  EXPECT_EQ(planning_debug_info.proposals().size(), 3);
}

TEST_F(ProtoConvertorTest, TestRecordCorridorInfoProto) {
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type = CorridorInfo::Type::LANE_CHANGE;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    CorridorPoint corridor_point;
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(1.0 * i);
    corridor_point.xy_ref = xy_point;
    xy_point.set_x(2.0);
    corridor_point.xy_left = xy_point;
    xy_point.set_x(-2.0);
    corridor_point.xy_right = xy_point;
    corridor_point.theta = M_PI / 2;
    corridor_point.kappa = 0.0;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);

  std::unordered_map<std::string, SpeedLimit> speed_limit_map;
  SpeedLimit speed_limit_1;
  speed_limit_1.AppendSpeedLimit(0, 10.0);
  speed_limit_1.AppendSpeedLimit(15.0, 20.0);
  SpeedLimit speed_limit_2;
  speed_limit_2.AppendSpeedLimit(0, 5.0);
  speed_limit_2.AppendSpeedLimit(15.0, 3.0);
  speed_limit_map["map"] = speed_limit_1;
  speed_limit_map["curvature"] = speed_limit_2;
  corridor_info.SetSpeedLimitMap(speed_limit_map);

  CorridorInfo::STTopology st_topology;
  std::vector<std::string> layer;
  layer.emplace_back("0");
  layer.emplace_back("1");
  st_topology.layers.emplace_back(layer);
  layer.clear();
  layer.emplace_back("2");
  layer.emplace_back("3");
  layer.emplace_back("4");
  st_topology.layers.emplace_back(layer);
  corridor_info.SetSTTopology(st_topology);

  std::vector<CorridorInfo::STProposal> st_proposals;
  CorridorInfo::STProposal st_proposal;
  st_proposal.cost_total = 10.0;
  CorridorInfo::STProposal::STBoundary st_boundary;
  CorridorInfo::STProposal::Cost cost;
  st_boundary.obs_id = "0";
  st_boundary.is_front = true;
  cost.reason = "0, front";
  cost.cost = 1.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_boundary.obs_id = "1";
  st_boundary.is_front = false;
  cost.reason = "1, rear";
  cost.cost = 3.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_proposals.emplace_back(st_proposal);
  st_proposal.cost_total = 12.0;
  st_proposal.st_boundaries.clear();
  st_proposal.costs.clear();
  st_boundary.obs_id = "0";
  st_boundary.is_front = true;
  cost.reason = "0, front";
  cost.cost = 1.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_boundary.obs_id = "1";
  st_boundary.is_front = true;
  cost.reason = "1, front";
  cost.cost = 4.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_proposals.emplace_back(st_proposal);
  corridor_info.SetSTProposals(st_proposals);

  PlanningDebugInfo_CorridorInfo corridor_info_proto;
  ProtoConvertor::RecordCorridorInfoProto(corridor_info, &corridor_info_proto);

  EXPECT_EQ(corridor_info_proto.type(), "LANE_CHANGE");

  EXPECT_EQ(corridor_info_proto.speed_limit_map().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).name(), "map");
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).name(), "curvature");
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).s().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).v().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).s().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).v().size(), 2);

  EXPECT_EQ(corridor_info_proto.corridor_points().size(), 25);

  EXPECT_EQ(corridor_info_proto.st_topology().layers().size(), 2);
  EXPECT_EQ(corridor_info_proto.st_proposals().size(), 2);
}

TEST_F(ProtoConvertorTest, TestRecordProposalProto) {
  Proposal proposal;
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type = CorridorInfo::Type::LANE_KEEP;
  Corridor corridor;
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);
  proposal.SetCorridorInfo(&corridor_info);

  STProposal st_proposal;
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;
  lower_points.emplace_back(10.0, 0.0);
  lower_points.emplace_back(13.0, 10.0);
  upper_points.emplace_back(15.0, 0.0);
  upper_points.emplace_back(18.0, 10.0);
  STBoundary boundary_1 =
      STBoundary::CreateInstanceAccurate(lower_points, upper_points, "0");
  lower_points.clear();
  upper_points.clear();
  lower_points.emplace_back(0.0, 0.0);
  lower_points.emplace_back(0.0, 10.0);
  upper_points.emplace_back(5.0, 0.0);
  upper_points.emplace_back(5.0, 10.0);
  STBoundary boundary_2 =
      STBoundary::CreateInstanceAccurate(lower_points, upper_points, "1");
  st_proposal.emplace_back(std::make_tuple(&boundary_1, true, false));
  st_proposal.emplace_back(std::make_tuple(&boundary_2, false, false));
  proposal.SetSTProposal(st_proposal);

  LonMPCData lon_mpc_data;
  const int step = 10;
  lon_mpc_data.n_steps = step - 1;
  MPCData::Names name;
  name.x = std::vector<std::string>{"s", "v"};
  name.u = std::vector<std::string>{"a"};
  name.u_dot = std::vector<std::string>{"j"};
  lon_mpc_data.names = name;
  Eigen::MatrixXd x = Eigen::MatrixXd::Constant(2, step, 1.0);
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Constant(2, step, 1.0);
  Eigen::MatrixXd u = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::MatrixXd u_dot = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd t = Eigen::MatrixXd::Constant(1, step, 0.0);
  for (int i = 0; i < step; i++) {
    x(0, i) += i * 1.;
    x_ref(0, i) += i * 1.1;
    t(i) = i;
  }
  lon_mpc_data.x = x;
  lon_mpc_data.x_ref = x_ref;
  lon_mpc_data.u = u;
  lon_mpc_data.u_ref = u;
  lon_mpc_data.u_dot = u_dot;
  lon_mpc_data.t = t;
  lon_mpc_data.u_prev = u_prev;
  lon_mpc_data.dt = 0.1;
  lon_mpc_data.dt_prev = 0.1;
  MPCData::Constraints constraint;
  constraint.soft_max.x.constraint = x.row(0);
  constraint.soft_max.x.idx_enabled = {true, false};
  constraint.soft_min.x.constraint = x.row(0);
  constraint.soft_min.x.idx_enabled = {true, false};
  constraint.hard_max.x.constraint = x.row(0);
  constraint.hard_max.x.idx_enabled = {true, false};
  constraint.hard_min.x.constraint = x.row(0);
  constraint.hard_min.x.idx_enabled = {true, false};
  constraint.stiff_max.x.constraint = x.row(1);
  constraint.stiff_max.x.idx_enabled = {false, true};
  constraint.stiff_min.x.constraint = x.row(1);
  constraint.stiff_min.x.idx_enabled = {false, true};
  constraint.soft_max.u.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u.idx_enabled = {false};
  constraint.soft_min.u.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u.idx_enabled = {false};
  constraint.hard_max.u.constraint = Eigen::RowVectorXd();
  constraint.hard_max.u.idx_enabled = {false};
  constraint.hard_min.u.constraint = Eigen::RowVectorXd();
  constraint.hard_min.u.idx_enabled = {false};
  constraint.stiff_max.u.constraint = u;
  constraint.stiff_max.u.idx_enabled = {true};
  constraint.stiff_min.u.constraint = u;
  constraint.stiff_min.u.idx_enabled = {true};
  constraint.soft_max.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u_dot.idx_enabled = {false};
  constraint.soft_min.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u_dot.idx_enabled = {false};
  constraint.hard_max.u_dot.constraint = u_dot;
  constraint.hard_max.u_dot.idx_enabled = {true};
  constraint.hard_min.u_dot.constraint = u_dot;
  constraint.hard_min.u_dot.idx_enabled = {true};
  constraint.stiff_max.u_dot.constraint = u_dot;
  constraint.stiff_max.u_dot.idx_enabled = {true};
  constraint.stiff_min.u_dot.constraint = u_dot;
  constraint.stiff_min.u_dot.idx_enabled = {true};
  lon_mpc_data.constraints = constraint;
  MPCData::Slacks slack;
  slack.soft_max.x = x;
  slack.soft_min.x = x;
  slack.stiff_max.x = x;
  slack.stiff_min.x = x;
  slack.soft_max.u = u;
  slack.soft_min.u = u;
  slack.stiff_max.u = u;
  slack.stiff_min.u = u;
  slack.soft_max.u_dot = u_dot;
  slack.soft_min.u_dot = u_dot;
  slack.stiff_max.u_dot = u_dot;
  slack.stiff_min.u_dot = u_dot;
  lon_mpc_data.slacks = slack;
  SpeedLimit speed_limit;
  speed_limit.AppendSpeedLimit(0.0, 5.7446);
  speed_limit.AppendSpeedLimit(4.0, 5.0);
  speed_limit.AppendSpeedLimit(10.0, 5.0);
  speed_limit.AppendSpeedLimit(20.0, 5.0);
  speed_limit.AppendSpeedLimit(30.0, 5.0);
  speed_limit.AppendSpeedLimit(40.0, 5.0);
  speed_limit.AppendSpeedLimit(50.0, 5.0);
  std::vector<SpeedLimit> speed_limit_set;
  speed_limit_set.emplace_back(speed_limit);
  speed_limit_set.emplace_back(speed_limit);
  lon_mpc_data.speed_limit_set = speed_limit_set;
  proposal.SetLonMPCData(lon_mpc_data);

  LatMPCData data;
  name.x = std::vector<std::string>{"l", "l_dot", "psi_s", "psi_s_dot"};
  name.u = std::vector<std::string>{"delta"};
  name.u_dot = std::vector<std::string>{"delta_dot"};
  name.u_2 = std::vector<std::string>{"psi_dot_des"};
  data.names = name;
  x = Eigen::MatrixXd::Zero(4, step);
  x_ref = Eigen::MatrixXd::Zero(4, step);
  u = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  u_dot = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::MatrixXd u_2 = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  u_prev = Eigen::VectorXd::Zero(1);
  t = Eigen::MatrixXd::Constant(1, step, 0.0);
  for (int i = 0; i < step; i++) {
    x(0, i) = 1.;
    x_ref(0, i) = 1.1;
    t(i) = i;
  }
  data.x = x;
  data.x_ref = x_ref;
  data.u = u;
  data.u_ref = u;
  data.u_dot = u_dot;
  data.u_2 = u_2;
  data.t = t;
  data.u_prev = u_prev;
  data.dt = 0.1;
  data.dt_prev = 0.1;
  data.n_steps = step - 1;
  constraint.soft_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.soft_max.x.idx_enabled = {true, false, true, false};
  constraint.soft_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.soft_min.x.idx_enabled = {true, false, true, false};
  constraint.hard_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.hard_max.x.idx_enabled = {true, true, false, false};
  constraint.hard_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.hard_min.x.idx_enabled = {true, true, false, false};
  constraint.stiff_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.stiff_max.x.idx_enabled = {true, false, false, true};
  constraint.stiff_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.stiff_min.x.idx_enabled = {true, false, false, true};
  constraint.soft_max.u.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u.idx_enabled = {false};
  constraint.soft_min.u.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u.idx_enabled = {false};
  constraint.hard_max.u.constraint = Eigen::RowVectorXd();
  constraint.hard_max.u.idx_enabled = {false};
  constraint.hard_min.u.constraint = Eigen::RowVectorXd();
  constraint.hard_min.u.idx_enabled = {false};
  constraint.stiff_max.u.constraint = u;
  constraint.stiff_max.u.idx_enabled = {true};
  constraint.stiff_min.u.constraint = u;
  constraint.stiff_min.u.idx_enabled = {true};
  constraint.soft_max.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u_dot.idx_enabled = {false};
  constraint.soft_min.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u_dot.idx_enabled = {false};
  constraint.hard_max.u_dot.constraint = u_dot;
  constraint.hard_max.u_dot.idx_enabled = {true};
  constraint.hard_min.u_dot.constraint = u_dot;
  constraint.hard_min.u_dot.idx_enabled = {true};
  constraint.stiff_max.u_dot.constraint = u_dot;
  constraint.stiff_max.u_dot.idx_enabled = {true};
  constraint.stiff_min.u_dot.constraint = u_dot;
  constraint.stiff_min.u_dot.idx_enabled = {true};
  data.constraints = constraint;
  slack.soft_max.x = x;
  slack.soft_min.x = x;
  slack.stiff_max.x = x;
  slack.stiff_min.x = x;
  slack.soft_max.u = u;
  slack.soft_min.u = u;
  slack.stiff_max.u = u;
  slack.stiff_min.u = u;
  slack.soft_max.u_dot = u_dot;
  slack.soft_min.u_dot = u_dot;
  slack.stiff_max.u_dot = u_dot;
  slack.stiff_min.u_dot = u_dot;
  data.slacks = slack;
  Tube res_tube;
  for (int k = 0; k < step; k++) {
    Tube::TubePoint pts;
    ::math::Vec2d xy_left_ref_(0. + k, 0.);
    ::math::Vec2d xy_right_ref_(0. + k, 0.);
    ::math::Vec2d xy_left_soft_(0. + k, 1.);
    ::math::Vec2d xy_right_soft_(0. + k, -1.);
    ::math::Vec2d xy_left_stiff_(0. + k, 1.5);
    ::math::Vec2d xy_right_stiff_(0. + k, -1.5);
    ::math::Vec2d xy_left_hard_(0. + k, 2.);
    ::math::Vec2d xy_right_hard_(0. + k, -2.);
    pts.t = k;
    pts.xy_left_ref = xy_left_ref_;
    pts.xy_right_ref = xy_right_ref_;
    pts.xy_left_soft = xy_left_soft_;
    pts.xy_right_soft = xy_right_soft_;
    pts.xy_left_stiff = xy_left_stiff_;
    pts.xy_right_stiff = xy_right_stiff_;
    pts.xy_left_hard = xy_left_hard_;
    pts.xy_right_hard = xy_right_hard_;
    res_tube.pts.emplace_back(pts);
  }
  data.tube = res_tube;
  proposal.SetLatMPCData(data);

  auto cost_table1 = proposal.GetMutableCosts();
  cost_table1->emplace(
      "safety",
      Costs({std::make_pair(
          10.0,
          "relative time: 4.400000s. min distance to obtacle: 0.000000")}));
  cost_table1->emplace(
      "similarity",
      Costs({std::make_pair(
          0.125, "average difference to previous trajectory is 0.500000m.")}));
  cost_table1->emplace("Total", Costs({std::make_pair(10.125, "")}));

  DiscretizedTrajectory trajectory;
  const int num_point = 10;
  const double init_s = 0.0;
  const double init_v = 10.0;
  const double init_a = -2.0;
  const double delta_t = 0.1;
  for (int i = 0; i < num_point; ++i) {
    TrajectoryPoint trajectory_point;
    ::common::PathPoint path_point;
    const double time = delta_t * i;
    trajectory_point.set_relative_time(time);
    trajectory_point.set_v(init_v + init_a * time);
    trajectory_point.set_a(init_a);
    path_point.set_s(init_s + init_v * time + 0.5 * init_a * time * time);
    path_point.set_x(init_s + init_v * time + 0.5 * init_a * time * time);
    path_point.set_y(0.0);
    path_point.set_theta(0.5 * M_PI_2);
    path_point.set_kappa(0.0);
    trajectory_point.set_path_point(path_point);
    trajectory.emplace_back(trajectory_point);
  }
  proposal.SetTrajectory(trajectory);

  PlanningDebugInfo_Proposal proposal_proto;
  ProtoConvertor::RecordProposalProto(proposal, &proposal_proto);

  EXPECT_EQ(proposal_proto.corridor_type(), "LANE_KEEP");

  EXPECT_EQ(proposal_proto.st_proposal().st_boundaries().size(), 2);
  EXPECT_EQ(proposal_proto.st_proposal().st_boundaries().at(0).obs_id(), "0");
  EXPECT_EQ(proposal_proto.st_proposal().st_boundaries().at(0).t().size(), 4);
  EXPECT_EQ(proposal_proto.st_proposal().st_boundaries().at(0).s().size(), 4);

  EXPECT_EQ(proposal_proto.lon_mpc().mpc_data().trajectories(0).name(), "s");
  EXPECT_EQ(proposal_proto.lon_mpc().mpc_data().trajectories(1).name(), "v");
  EXPECT_EQ(proposal_proto.lon_mpc().mpc_data().trajectories(2).name(), "a");
  EXPECT_EQ(proposal_proto.lon_mpc().mpc_data().trajectories(3).name(), "j");
  EXPECT_EQ(proposal_proto.lon_mpc().mpc_data().trajectories().size(), 4);
  EXPECT_DOUBLE_EQ(proposal_proto.lon_mpc().mpc_data().dt_prev(), 0.1);
  EXPECT_DOUBLE_EQ(proposal_proto.lon_mpc().mpc_data().u_prev(0), 0.);
  const std::string t0_name = "t=0.0s";
  const std::string t1_name = "t=0.1s";
  EXPECT_EQ(t0_name, proposal_proto.lon_mpc().speed_limit_set().at(0).name());
  EXPECT_EQ(t1_name, proposal_proto.lon_mpc().speed_limit_set().at(1).name());
  EXPECT_EQ(proposal_proto.lon_mpc().speed_limit_set().size(), 2);

  EXPECT_EQ(proposal_proto.lat_mpc().mpc_data().trajectories(0).name(), "l");
  EXPECT_EQ(proposal_proto.lat_mpc().mpc_data().trajectories(1).name(),
            "l_dot");
  EXPECT_EQ(proposal_proto.lat_mpc().mpc_data().trajectories(2).name(),
            "psi_s");
  EXPECT_EQ(proposal_proto.lat_mpc().mpc_data().trajectories(3).name(),
            "psi_s_dot");
  EXPECT_EQ(proposal_proto.lat_mpc().mpc_data().trajectories(4).name(),
            "delta");
  EXPECT_EQ(proposal_proto.lat_mpc().mpc_data().trajectories(5).name(),
            "delta_dot");
  EXPECT_EQ(proposal_proto.lat_mpc().mpc_data().trajectories(6).name(),
            "psi_dot_des");
  EXPECT_EQ(proposal_proto.lat_mpc().mpc_data().trajectories().size(), 7);
  EXPECT_DOUBLE_EQ(proposal_proto.lat_mpc().mpc_data().dt_prev(), 0.1);
  EXPECT_DOUBLE_EQ(proposal_proto.lat_mpc().tubes().t().size(), 10);
  EXPECT_DOUBLE_EQ(proposal_proto.lat_mpc().tubes().ref().xy_left_size(), 10);
  EXPECT_DOUBLE_EQ(proposal_proto.lat_mpc().tubes().ref().xy_right_size(), 10);

  EXPECT_EQ(proposal_proto.evaluation().costs().size(), 3);
  EXPECT_EQ(proposal_proto.evaluation().costs().at(0).cost_type(), "Total");

  EXPECT_EQ(proposal_proto.trajectory().t().size(), num_point);
  EXPECT_EQ(proposal_proto.trajectory().s().size(), num_point);
  EXPECT_EQ(proposal_proto.trajectory().v().size(), num_point);
  EXPECT_EQ(proposal_proto.trajectory().a().size(), num_point);
}

TEST_F(ProtoConvertorTest, TestConvertCorridorTypeToString) {
  CorridorInfo::Type type = CorridorInfo::Type::LANE_KEEP;
  EXPECT_EQ(ProtoConvertor::ConvertCorridorTypeToString(type), "LANE_KEEP");
  type = CorridorInfo::Type::LANE_CHANGE;
  EXPECT_EQ(ProtoConvertor::ConvertCorridorTypeToString(type), "LANE_CHANGE");
  type = CorridorInfo::Type::STAGING;
  EXPECT_EQ(ProtoConvertor::ConvertCorridorTypeToString(type), "STAGING");
  type = CorridorInfo::Type::NUDGE;
  EXPECT_EQ(ProtoConvertor::ConvertCorridorTypeToString(type), "NUDGE");
  type = CorridorInfo::Type::NUDGE;
  EXPECT_EQ(ProtoConvertor::ConvertCorridorTypeToString(type), "NUDGE");
}

TEST_F(ProtoConvertorTest, TestRecordSpeedLimitMapProto) {
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type;
  Corridor corridor;
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);
  std::unordered_map<std::string, SpeedLimit> speed_limit_map;
  SpeedLimit speed_limit_1;
  speed_limit_1.AppendSpeedLimit(0, 10.0);
  speed_limit_1.AppendSpeedLimit(15.0, 20.0);
  SpeedLimit speed_limit_2;
  speed_limit_2.AppendSpeedLimit(0, 5.0);
  speed_limit_2.AppendSpeedLimit(15.0, 3.0);
  speed_limit_map["map"] = speed_limit_1;
  speed_limit_map["curvature"] = speed_limit_2;
  corridor_info.SetSpeedLimitMap(speed_limit_map);
  PlanningDebugInfo_CorridorInfo corridor_info_proto;
  ProtoConvertor::RecordSpeedLimitMapProto(corridor_info, &corridor_info_proto);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).name(), "map");
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).name(), "curvature");
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).s().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).v().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).s().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).v().size(), 2);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).s().at(0), 0.0);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).v().at(0), 10.0);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).s().at(1), 15.0);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(1).v().at(1), 20.0);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).s().at(0), 0.0);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).v().at(0), 5.0);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).s().at(1), 15.0);
  EXPECT_EQ(corridor_info_proto.speed_limit_map().at(0).v().at(1), 3.0);
}

TEST_F(ProtoConvertorTest, TestRecordCorridorPointsProto) {
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type;
  std::vector<CorridorPoint> corridor_points;
  ::math::Vec2d xy_point;
  for (int i = 0; i < 25; i++) {
    CorridorPoint corridor_point;
    corridor_point.s = -5 + 1.0 * i;
    corridor_point.l = 0.0;
    xy_point.set_x(0.0);
    xy_point.set_y(1.0 * i);
    corridor_point.xy_ref = xy_point;
    xy_point.set_x(2.0);
    corridor_point.xy_left = xy_point;
    xy_point.set_x(-2.0);
    corridor_point.xy_right = xy_point;
    corridor_point.theta = M_PI / 2;
    corridor_point.kappa = 0.0;
    corridor_points.emplace_back(corridor_point);
  }
  Corridor corridor(corridor_points);
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);
  PlanningDebugInfo_CorridorInfo corridor_info_proto;
  ProtoConvertor::RecordCorridorPointsProto(corridor_info,
                                            &corridor_info_proto);
  EXPECT_EQ(corridor_info_proto.corridor_points().size(), 25);
  EXPECT_EQ(corridor_info_proto.corridor_points().at(24).xy_ref().x(), 0.0);
  EXPECT_EQ(corridor_info_proto.corridor_points().at(24).xy_ref().y(), 24.0);
  EXPECT_EQ(corridor_info_proto.corridor_points().at(24).xy_left().x(), 2.0);
  EXPECT_EQ(corridor_info_proto.corridor_points().at(24).xy_right().x(), -2.0);
  EXPECT_EQ(corridor_info_proto.corridor_points().at(0).theta(), M_PI / 2);
  EXPECT_EQ(corridor_info_proto.corridor_points().at(0).kappa(), 0.0);
}

TEST_F(ProtoConvertorTest, TestRecordCorridorSTBoundaryCounterMap) {
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type;
  Corridor corridor;
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);
  std::unordered_map<std::string, int> st_boundary_counter_map;
  st_boundary_counter_map["0"] = 2;
  st_boundary_counter_map["1"] = 5;
  corridor_info.SetSTBoundaryCounterMap(st_boundary_counter_map);

  PlanningDebugInfo_CorridorInfo corridor_info_proto;

  ProtoConvertor::RecordCorridorSTBoundaryCounterMap(corridor_info,
                                                     &corridor_info_proto);
  EXPECT_EQ(corridor_info_proto.st_boundary_counter_map().size(), 2);
  EXPECT_EQ(corridor_info_proto.st_boundary_counter_map().at(0).obs_id(), "1");
  EXPECT_EQ(corridor_info_proto.st_boundary_counter_map().at(0).counter(), 5);
  EXPECT_EQ(corridor_info_proto.st_boundary_counter_map().at(1).obs_id(), "0");
  EXPECT_EQ(corridor_info_proto.st_boundary_counter_map().at(1).counter(), 2);
}

TEST_F(ProtoConvertorTest, TestRecordCorridorSTProposalsProto) {
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type;
  Corridor corridor;
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);
  std::vector<CorridorInfo::STProposal> st_proposals;
  CorridorInfo::STProposal st_proposal;
  st_proposal.cost_total = 10.0;
  CorridorInfo::STProposal::STBoundary st_boundary;
  CorridorInfo::STProposal::Cost cost;
  st_boundary.obs_id = "0";
  st_boundary.is_front = true;
  cost.reason = "0, front";
  cost.cost = 1.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_boundary.obs_id = "1";
  st_boundary.is_front = false;
  cost.reason = "1, rear";
  cost.cost = 3.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_proposals.emplace_back(st_proposal);

  st_proposal.cost_total = 12.0;
  st_proposal.st_boundaries.clear();
  st_proposal.costs.clear();
  st_boundary.obs_id = "0";
  st_boundary.is_front = true;
  cost.reason = "0, front";
  cost.cost = 1.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_boundary.obs_id = "1";
  st_boundary.is_front = true;
  cost.reason = "1, front";
  cost.cost = 4.0;
  st_proposal.st_boundaries.emplace_back(st_boundary);
  st_proposal.costs.emplace_back(cost);
  st_proposals.emplace_back(st_proposal);
  corridor_info.SetSTProposals(st_proposals);
  PlanningDebugInfo_CorridorInfo corridor_info_proto;

  ProtoConvertor::RecordCorridorSTProposalsProto(corridor_info,
                                                 &corridor_info_proto);
  EXPECT_EQ(corridor_info_proto.st_proposals().size(), 2);
  EXPECT_EQ(corridor_info_proto.st_proposals().at(0).st_boundaries().size(), 2);
  EXPECT_EQ(corridor_info_proto.st_proposals().at(0).costs().size(), 2);
  EXPECT_EQ(corridor_info_proto.st_proposals().at(0).cost_total(), 10.0);
  EXPECT_EQ(
      corridor_info_proto.st_proposals().at(0).st_boundaries().at(0).obs_id(),
      "0");
  EXPECT_EQ(
      corridor_info_proto.st_proposals().at(0).st_boundaries().at(0).is_front(),
      true);
  EXPECT_EQ(corridor_info_proto.st_proposals().at(0).costs().at(0).reason(),
            "0, front");
  EXPECT_EQ(corridor_info_proto.st_proposals().at(0).costs().at(0).cost(), 1.0);
  EXPECT_EQ(
      corridor_info_proto.st_proposals().at(1).st_boundaries().at(1).obs_id(),
      "1");
  EXPECT_EQ(
      corridor_info_proto.st_proposals().at(1).st_boundaries().at(1).is_front(),
      true);
  EXPECT_EQ(corridor_info_proto.st_proposals().at(1).costs().at(1).reason(),
            "1, front");
  EXPECT_EQ(corridor_info_proto.st_proposals().at(1).costs().at(1).cost(), 4.0);
}

TEST_F(ProtoConvertorTest, TestRecordSTTopologyProto) {
  LocalRoute local_route;
  Mission mission;
  CorridorInfo::Type corridor_type;
  Corridor corridor;
  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);
  CorridorInfo::STTopology st_topology;
  std::vector<std::string> layer;
  layer.emplace_back("0");
  layer.emplace_back("1");
  st_topology.layers.emplace_back(layer);
  layer.clear();
  layer.emplace_back("2");
  layer.emplace_back("3");
  layer.emplace_back("4");
  st_topology.layers.emplace_back(layer);
  corridor_info.SetSTTopology(st_topology);
  PlanningDebugInfo_CorridorInfo_STTopology st_topology_proto;
  ProtoConvertor::RecordSTTopologyProto(corridor_info, st_topology_proto);
  EXPECT_EQ(st_topology_proto.layers().size(), 2);
  EXPECT_EQ(st_topology_proto.layers().at(0).obs_ids().size(), 2);
  EXPECT_EQ(st_topology_proto.layers().at(1).obs_ids().size(), 3);
  EXPECT_EQ(st_topology_proto.layers().at(0).obs_ids().at(0), "0");
  EXPECT_EQ(st_topology_proto.layers().at(0).obs_ids().at(1), "1");
  EXPECT_EQ(st_topology_proto.layers().at(1).obs_ids().at(0), "2");
  EXPECT_EQ(st_topology_proto.layers().at(1).obs_ids().at(1), "3");
  EXPECT_EQ(st_topology_proto.layers().at(1).obs_ids().at(2), "4");
}

TEST_F(ProtoConvertorTest, TestRecordSTProposalProto) {
  Proposal proposal;
  STProposal st_proposal;
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;
  lower_points.emplace_back(10.0, 0.0);
  lower_points.emplace_back(13.0, 10.0);
  upper_points.emplace_back(15.0, 0.0);
  upper_points.emplace_back(18.0, 10.0);
  STBoundary boundary_1 =
      STBoundary::CreateInstanceAccurate(lower_points, upper_points, "0");
  lower_points.clear();
  upper_points.clear();
  lower_points.emplace_back(0.0, 0.0);
  lower_points.emplace_back(0.0, 10.0);
  upper_points.emplace_back(5.0, 0.0);
  upper_points.emplace_back(5.0, 10.0);
  STBoundary boundary_2 =
      STBoundary::CreateInstanceAccurate(lower_points, upper_points, "1");
  st_proposal.emplace_back(std::make_tuple(&boundary_1, true, false));
  st_proposal.emplace_back(std::make_tuple(&boundary_2, false, false));
  proposal.SetSTProposal(st_proposal);
  PlanningDebugInfo_Proposal_STProposal st_proposal_proto;
  ProtoConvertor::RecordSTProposalProto(proposal, st_proposal_proto);
  EXPECT_EQ(st_proposal_proto.st_boundaries().size(), 2);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).obs_id(), "0");
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).t().size(), 4);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).s().size(), 4);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).t().at(0), 0.0);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).t().at(1), 10.0);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).t().at(2), 10.0);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).t().at(3), 0.0);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).s().at(0), 10.0);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).s().at(1), 13.0);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).s().at(2), 18.0);
  EXPECT_EQ(st_proposal_proto.st_boundaries().at(0).s().at(3), 15.0);
}

TEST_F(ProtoConvertorTest, TestRecordEvaluationProto) {
  Proposal proposal1, proposal2;
  std::vector<Proposal> proposals;
  auto cost_table1 = proposal1.GetMutableCosts();
  cost_table1->emplace(
      "safety",
      Costs({std::make_pair(
          10.0,
          "relative time: 4.400000s. min distance to obtacle: 0.000000")}));
  cost_table1->emplace(
      "similarity",
      Costs({std::make_pair(
          0.125, "average difference to previous trajectory is 0.500000m.")}));
  cost_table1->emplace("Total", Costs({std::make_pair(10.125, "")}));
  auto cost_table2 = proposal2.GetMutableCosts();
  cost_table2->emplace(
      "safety",
      Costs({std::make_pair(
                 10.0,
                 "relative time: 4.400000s. min distance to obtacle: 0.000000"),
             std::make_pair(2.0, "stiff constraint violation in psi_dot"),
             std::make_pair(1.0, "stiff constraint violation in delta"),
             std::make_pair(3.0, "stiff constraint violation in v")}));
  cost_table2->emplace("Total", Costs({std::make_pair(16.0, "")}));
  proposals.emplace_back(proposal1);
  proposals.emplace_back(proposal2);
  PlanningDebugInfo_Proposal_Evaluation evaluation_proto;
  ProtoConvertor::RecordEvaluationProto(proposals[0], evaluation_proto);
  EXPECT_EQ(evaluation_proto.costs().at(0).cost_type(), "Total");
  EXPECT_DOUBLE_EQ(evaluation_proto.costs().at(0).cost().at(0), 10.125);
  EXPECT_EQ(evaluation_proto.costs().at(0).reason().at(0), "");
  EXPECT_EQ(evaluation_proto.costs().at(1).cost_type(), "safety");
  EXPECT_DOUBLE_EQ(evaluation_proto.costs().at(1).cost().at(0), 10.0);
  EXPECT_EQ(evaluation_proto.costs().at(1).reason().at(0),
            "relative time: 4.400000s. min distance to obtacle: 0.000000");
  EXPECT_EQ(evaluation_proto.costs().at(2).cost_type(), "similarity");
  EXPECT_DOUBLE_EQ(evaluation_proto.costs().at(2).cost().at(0), 0.125);
  EXPECT_EQ(evaluation_proto.costs().at(2).reason().at(0),
            "average difference to previous trajectory is 0.500000m.");

  evaluation_proto.Clear();
  ProtoConvertor::RecordEvaluationProto(proposals[1], evaluation_proto);
  EXPECT_EQ(evaluation_proto.costs().at(0).cost_type(), "Total");
  EXPECT_DOUBLE_EQ(evaluation_proto.costs().at(0).cost().at(0), 16.0);
  EXPECT_EQ(evaluation_proto.costs().at(0).reason().at(0), "");
  EXPECT_EQ(evaluation_proto.costs().at(1).cost_type(), "safety");
  EXPECT_DOUBLE_EQ(evaluation_proto.costs().at(1).cost().at(0), 10.0);
  EXPECT_EQ(evaluation_proto.costs().at(1).reason().at(0),
            "relative time: 4.400000s. min distance to obtacle: 0.000000");
  EXPECT_DOUBLE_EQ(evaluation_proto.costs().at(1).cost().at(1), 2.0);
  EXPECT_EQ(evaluation_proto.costs().at(1).reason().at(1),
            "stiff constraint violation in psi_dot");
  EXPECT_DOUBLE_EQ(evaluation_proto.costs().at(1).cost().at(2), 1.0);
  EXPECT_EQ(evaluation_proto.costs().at(1).reason().at(2),
            "stiff constraint violation in delta");
  EXPECT_DOUBLE_EQ(evaluation_proto.costs().at(1).cost().at(3), 3.0);
  EXPECT_EQ(evaluation_proto.costs().at(1).reason().at(3),
            "stiff constraint violation in v");
}

TEST_F(ProtoConvertorTest, TestRecordLonMPCDataProto) {
  LonMPCData lon_mpc_data;
  const int step = 10;
  lon_mpc_data.n_steps = step - 1;
  MPCData::Names name;
  name.x = std::vector<std::string>{"s", "v"};
  name.u = std::vector<std::string>{"a"};
  name.u_dot = std::vector<std::string>{"j"};
  lon_mpc_data.names = name;

  Eigen::MatrixXd x = Eigen::MatrixXd::Constant(2, step, 1.0);
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Constant(2, step, 1.0);
  Eigen::MatrixXd u = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::MatrixXd u_dot = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd t = Eigen::MatrixXd::Constant(1, step, 0.0);
  for (int i = 0; i < step; i++) {
    x(0, i) += i * 1.;
    x_ref(0, i) += i * 1.1;
    t(i) = i;
  }
  lon_mpc_data.x = x;
  lon_mpc_data.x_ref = x_ref;
  lon_mpc_data.u = u;
  lon_mpc_data.u_ref = u;
  lon_mpc_data.u_dot = u_dot;
  lon_mpc_data.t = t;
  lon_mpc_data.u_prev = u_prev;
  lon_mpc_data.dt = 0.1;
  lon_mpc_data.dt_prev = 0.1;

  MPCData::Constraints constraint;
  constraint.soft_max.x.constraint = x.row(0);
  constraint.soft_max.x.idx_enabled = {true, false};
  constraint.soft_min.x.constraint = x.row(0);
  constraint.soft_min.x.idx_enabled = {true, false};
  constraint.hard_max.x.constraint = x.row(0);
  constraint.hard_max.x.idx_enabled = {true, false};
  constraint.hard_min.x.constraint = x.row(0);
  constraint.hard_min.x.idx_enabled = {true, false};
  constraint.stiff_max.x.constraint = x.row(1);
  constraint.stiff_max.x.idx_enabled = {false, true};
  constraint.stiff_min.x.constraint = x.row(1);
  constraint.stiff_min.x.idx_enabled = {false, true};
  constraint.soft_max.u.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u.idx_enabled = {false};
  constraint.soft_min.u.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u.idx_enabled = {false};
  constraint.hard_max.u.constraint = Eigen::RowVectorXd();
  constraint.hard_max.u.idx_enabled = {false};
  constraint.hard_min.u.constraint = Eigen::RowVectorXd();
  constraint.hard_min.u.idx_enabled = {false};
  constraint.stiff_max.u.constraint = u;
  constraint.stiff_max.u.idx_enabled = {true};
  constraint.stiff_min.u.constraint = u;
  constraint.stiff_min.u.idx_enabled = {true};
  constraint.soft_max.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u_dot.idx_enabled = {false};
  constraint.soft_min.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u_dot.idx_enabled = {false};
  constraint.hard_max.u_dot.constraint = u_dot;
  constraint.hard_max.u_dot.idx_enabled = {true};
  constraint.hard_min.u_dot.constraint = u_dot;
  constraint.hard_min.u_dot.idx_enabled = {true};
  constraint.stiff_max.u_dot.constraint = u_dot;
  constraint.stiff_max.u_dot.idx_enabled = {true};
  constraint.stiff_min.u_dot.constraint = u_dot;
  constraint.stiff_min.u_dot.idx_enabled = {true};
  lon_mpc_data.constraints = constraint;

  MPCData::Slacks slack;
  slack.soft_max.x = x;
  slack.soft_min.x = x;
  slack.stiff_max.x = x;
  slack.stiff_min.x = x;
  slack.soft_max.u = u;
  slack.soft_min.u = u;
  slack.stiff_max.u = u;
  slack.stiff_min.u = u;
  slack.soft_max.u_dot = u_dot;
  slack.soft_min.u_dot = u_dot;
  slack.stiff_max.u_dot = u_dot;
  slack.stiff_min.u_dot = u_dot;
  lon_mpc_data.slacks = slack;

  SpeedLimit speed_limit;
  speed_limit.AppendSpeedLimit(0.0, 5.7446);
  speed_limit.AppendSpeedLimit(4.0, 5.0);
  speed_limit.AppendSpeedLimit(10.0, 5.0);
  speed_limit.AppendSpeedLimit(20.0, 5.0);
  speed_limit.AppendSpeedLimit(30.0, 5.0);
  speed_limit.AppendSpeedLimit(40.0, 5.0);
  speed_limit.AppendSpeedLimit(50.0, 5.0);
  std::vector<SpeedLimit> speed_limit_set;
  speed_limit_set.emplace_back(speed_limit);
  speed_limit_set.emplace_back(speed_limit);
  lon_mpc_data.speed_limit_set = speed_limit_set;

  Proposal proposal;
  PlanningDebugInfo_Proposal_LonMPC lon_mpc_proto;
  proposal.SetLonMPCData(lon_mpc_data);

  ProtoConvertor::RecordLonMPCDataProto(proposal, lon_mpc_proto);

  EXPECT_EQ(lon_mpc_proto.mpc_data().trajectories(0).name(), "s");
  EXPECT_EQ(lon_mpc_proto.mpc_data().trajectories(1).name(), "v");
  EXPECT_EQ(lon_mpc_proto.mpc_data().trajectories(2).name(), "a");
  EXPECT_EQ(lon_mpc_proto.mpc_data().trajectories(3).name(), "j");
  EXPECT_EQ(lon_mpc_proto.mpc_data().trajectories().size(), 4);
  EXPECT_DOUBLE_EQ(lon_mpc_proto.mpc_data().dt_prev(), 0.1);
  EXPECT_DOUBLE_EQ(lon_mpc_proto.mpc_data().u_prev(0), 0.);

  const std::string t0_name = "t=0.0s";
  const std::string t1_name = "t=0.1s";
  EXPECT_EQ(t0_name, lon_mpc_proto.speed_limit_set().at(0).name());
  EXPECT_EQ(t1_name, lon_mpc_proto.speed_limit_set().at(1).name());
  EXPECT_EQ(lon_mpc_proto.speed_limit_set().size(), 2);
}

TEST_F(ProtoConvertorTest, TestRecordLatMPCDataProto) {
  const int step = 10;

  LatMPCData data;
  MPCData::Names name;
  name.x = std::vector<std::string>{"l", "l_dot", "psi_s", "psi_s_dot"};
  name.u = std::vector<std::string>{"delta"};
  name.u_dot = std::vector<std::string>{"delta_dot"};
  name.u_2 = std::vector<std::string>{"psi_dot_des"};
  data.names = name;

  Eigen::MatrixXd x = Eigen::MatrixXd::Zero(4, step);
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Zero(4, step);
  Eigen::MatrixXd u = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::MatrixXd u_dot = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::MatrixXd u_2 = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd t = Eigen::MatrixXd::Constant(1, step, 0.0);
  for (int i = 0; i < step; i++) {
    x(0, i) = 1.;
    x_ref(0, i) = 1.1;
    t(i) = i;
  }
  data.x = x;
  data.x_ref = x_ref;
  data.u = u;
  data.u_ref = u;
  data.u_dot = u_dot;
  data.u_2 = u_2;
  data.t = t;
  data.u_prev = u_prev;
  data.dt = 0.1;
  data.dt_prev = 0.1;
  data.n_steps = step - 1;

  MPCData::Constraints constraint;
  constraint.soft_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.soft_max.x.idx_enabled = {true, false, true, false};
  constraint.soft_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.soft_min.x.idx_enabled = {true, false, true, false};
  constraint.hard_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.hard_max.x.idx_enabled = {true, true, false, false};
  constraint.hard_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.hard_min.x.idx_enabled = {true, true, false, false};
  constraint.stiff_max.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.stiff_max.x.idx_enabled = {true, false, false, true};
  constraint.stiff_min.x.constraint = Eigen::MatrixXd(2, x.cols());
  constraint.stiff_min.x.idx_enabled = {true, false, false, true};
  constraint.soft_max.u.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u.idx_enabled = {false};
  constraint.soft_min.u.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u.idx_enabled = {false};
  constraint.hard_max.u.constraint = Eigen::RowVectorXd();
  constraint.hard_max.u.idx_enabled = {false};
  constraint.hard_min.u.constraint = Eigen::RowVectorXd();
  constraint.hard_min.u.idx_enabled = {false};
  constraint.stiff_max.u.constraint = u;
  constraint.stiff_max.u.idx_enabled = {true};
  constraint.stiff_min.u.constraint = u;
  constraint.stiff_min.u.idx_enabled = {true};
  constraint.soft_max.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u_dot.idx_enabled = {false};
  constraint.soft_min.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u_dot.idx_enabled = {false};
  constraint.hard_max.u_dot.constraint = u_dot;
  constraint.hard_max.u_dot.idx_enabled = {true};
  constraint.hard_min.u_dot.constraint = u_dot;
  constraint.hard_min.u_dot.idx_enabled = {true};
  constraint.stiff_max.u_dot.constraint = u_dot;
  constraint.stiff_max.u_dot.idx_enabled = {true};
  constraint.stiff_min.u_dot.constraint = u_dot;
  constraint.stiff_min.u_dot.idx_enabled = {true};
  data.constraints = constraint;

  MPCData::Slacks slack;
  slack.soft_max.x = x;
  slack.soft_min.x = x;
  slack.stiff_max.x = x;
  slack.stiff_min.x = x;
  slack.soft_max.u = u;
  slack.soft_min.u = u;
  slack.stiff_max.u = u;
  slack.stiff_min.u = u;
  slack.soft_max.u_dot = u_dot;
  slack.soft_min.u_dot = u_dot;
  slack.stiff_max.u_dot = u_dot;
  slack.stiff_min.u_dot = u_dot;
  data.slacks = slack;

  Tube res_tube;
  for (int k = 0; k < step; k++) {
    Tube::TubePoint pts;
    ::math::Vec2d xy_left_ref_(0. + k, 0.);
    ::math::Vec2d xy_right_ref_(0. + k, 0.);
    ::math::Vec2d xy_left_soft_(0. + k, 1.);
    ::math::Vec2d xy_right_soft_(0. + k, -1.);
    ::math::Vec2d xy_left_stiff_(0. + k, 1.5);
    ::math::Vec2d xy_right_stiff_(0. + k, -1.5);
    ::math::Vec2d xy_left_hard_(0. + k, 2.);
    ::math::Vec2d xy_right_hard_(0. + k, -2.);
    pts.t = k;
    pts.xy_left_ref = xy_left_ref_;
    pts.xy_right_ref = xy_right_ref_;
    pts.xy_left_soft = xy_left_soft_;
    pts.xy_right_soft = xy_right_soft_;
    pts.xy_left_stiff = xy_left_stiff_;
    pts.xy_right_stiff = xy_right_stiff_;
    pts.xy_left_hard = xy_left_hard_;
    pts.xy_right_hard = xy_right_hard_;
    res_tube.pts.emplace_back(pts);
  }
  data.tube = res_tube;

  Proposal proposal;
  proposal.SetLatMPCData(data);
  PlanningDebugInfo_Proposal_LatMPC lat_mpc_proto;
  ProtoConvertor::RecordLatMPCDataProto(proposal, lat_mpc_proto);
  name.x = std::vector<std::string>{"l", "l_dot", "psi_s", "psi_s_dot"};
  name.u = std::vector<std::string>{"delta"};
  name.u_dot = std::vector<std::string>{"delta_dot"};
  name.u_2 = std::vector<std::string>{"psi_dot_des"};
  EXPECT_EQ(lat_mpc_proto.mpc_data().trajectories(0).name(), "l");
  EXPECT_EQ(lat_mpc_proto.mpc_data().trajectories(1).name(), "l_dot");
  EXPECT_EQ(lat_mpc_proto.mpc_data().trajectories(2).name(), "psi_s");
  EXPECT_EQ(lat_mpc_proto.mpc_data().trajectories(3).name(), "psi_s_dot");
  EXPECT_EQ(lat_mpc_proto.mpc_data().trajectories(4).name(), "delta");
  EXPECT_EQ(lat_mpc_proto.mpc_data().trajectories(5).name(), "delta_dot");
  EXPECT_EQ(lat_mpc_proto.mpc_data().trajectories(6).name(), "psi_dot_des");
  EXPECT_EQ(lat_mpc_proto.mpc_data().trajectories().size(), 7);
  EXPECT_DOUBLE_EQ(lat_mpc_proto.mpc_data().dt_prev(), 0.1);
  EXPECT_DOUBLE_EQ(lat_mpc_proto.tubes().t().size(), 10);
  EXPECT_DOUBLE_EQ(lat_mpc_proto.tubes().ref().xy_left_size(), 10);
  EXPECT_DOUBLE_EQ(lat_mpc_proto.tubes().ref().xy_right_size(), 10);
}

TEST_F(ProtoConvertorTest, TestRecordMPCDataProto) {
  const int step = 10;

  MPCData data;
  MPCData::Names name;
  name.x = std::vector<std::string>{"s", "v"};
  name.u = std::vector<std::string>{"a"};
  name.u_dot = std::vector<std::string>{"j"};
  data.names = name;

  Eigen::MatrixXd x = Eigen::MatrixXd::Constant(2, step, 1.0);
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Constant(2, step, 1.0);
  Eigen::MatrixXd u = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::MatrixXd u_dot = Eigen::MatrixXd::Constant(1, step - 1, 0.0);
  Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd t = Eigen::MatrixXd::Constant(1, step, 0.0);
  for (int i = 0; i < step; i++) {
    x(0, i) += i * 1.;
    x_ref(0, i) += i * 1.1;
    t(i) = i;
  }
  data.x = x;
  data.x_ref = x_ref;
  data.u = u;
  data.u_ref = u;
  data.u_dot = u_dot;
  data.t = t;

  MPCData::Constraints constraint;
  constraint.soft_max.x.constraint = x.row(0);
  constraint.soft_max.x.idx_enabled = {true, false};
  constraint.soft_min.x.constraint = x.row(0);
  constraint.soft_min.x.idx_enabled = {true, false};
  constraint.hard_max.x.constraint = x.row(0);
  constraint.hard_max.x.idx_enabled = {true, false};
  constraint.hard_min.x.constraint = x.row(0);
  constraint.hard_min.x.idx_enabled = {true, false};
  constraint.stiff_max.x.constraint = x.row(1);
  constraint.stiff_max.x.idx_enabled = {false, true};
  constraint.stiff_min.x.constraint = x.row(1);
  constraint.stiff_min.x.idx_enabled = {false, true};
  constraint.soft_max.u.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u.idx_enabled = {false};
  constraint.soft_min.u.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u.idx_enabled = {false};
  constraint.hard_max.u.constraint = Eigen::RowVectorXd();
  constraint.hard_max.u.idx_enabled = {false};
  constraint.hard_min.u.constraint = Eigen::RowVectorXd();
  constraint.hard_min.u.idx_enabled = {false};
  constraint.stiff_max.u.constraint = u;
  constraint.stiff_max.u.idx_enabled = {true};
  constraint.stiff_min.u.constraint = u;
  constraint.stiff_min.u.idx_enabled = {true};
  constraint.soft_max.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_max.u_dot.idx_enabled = {false};
  constraint.soft_min.u_dot.constraint = Eigen::RowVectorXd();
  constraint.soft_min.u_dot.idx_enabled = {false};
  constraint.hard_max.u_dot.constraint = u_dot;
  constraint.hard_max.u_dot.idx_enabled = {true};
  constraint.hard_min.u_dot.constraint = u_dot;
  constraint.hard_min.u_dot.idx_enabled = {true};
  constraint.stiff_max.u_dot.constraint = u_dot;
  constraint.stiff_max.u_dot.idx_enabled = {true};
  constraint.stiff_min.u_dot.constraint = u_dot;
  constraint.stiff_min.u_dot.idx_enabled = {true};
  data.constraints = constraint;

  MPCData::Slacks slack;
  slack.soft_max.x = x;
  slack.soft_min.x = x;
  slack.stiff_max.x = x;
  slack.stiff_min.x = x;
  slack.soft_max.u = u;
  slack.soft_min.u = u;
  slack.stiff_max.u = u;
  slack.stiff_min.u = u;
  slack.soft_max.u_dot = u_dot;
  slack.soft_min.u_dot = u_dot;
  slack.stiff_max.u_dot = u_dot;
  slack.stiff_min.u_dot = u_dot;
  data.slacks = slack;

  PlanningDebugInfo planning_debug_info;
  auto lon_mpc_data_proto = planning_debug_info.add_proposals()
                                ->mutable_lon_mpc()
                                ->mutable_mpc_data();
  ProtoConvertor::RecordMPCDataProto(x, u, u_dot, Eigen::MatrixXd(), x_ref, u,
                                     constraint, slack, 1., step - 1, u_prev,
                                     1., name, *lon_mpc_data_proto);
  EXPECT_EQ(lon_mpc_data_proto->trajectories(0).name(), "s");
  EXPECT_EQ(lon_mpc_data_proto->trajectories(1).name(), "v");
  EXPECT_EQ(lon_mpc_data_proto->trajectories(2).name(), "a");
  EXPECT_EQ(lon_mpc_data_proto->trajectories(3).name(), "j");
  EXPECT_DOUBLE_EQ(lon_mpc_data_proto->dt_prev(), 1.);
  EXPECT_DOUBLE_EQ(lon_mpc_data_proto->u_prev(0), 0.);
  for (int i = 0; i < lon_mpc_data_proto->trajectories().size(); i++) {
    if (lon_mpc_data_proto->trajectories(i).name() == "s") {
      for (int j = 0; j < lon_mpc_data_proto->trajectories(i).t().size(); j++) {
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).t(j), (double)j);
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).soln(j), x(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).ref(j), x_ref(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).constraints().soft_min(j),
                  constraint.soft_min.x.constraint(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).constraints().soft_max(j),
                  constraint.soft_max.x.constraint(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i)
                      .constraints()
                      .stiff_min()
                      .size(),
                  0);
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i)
                      .constraints()
                      .stiff_max()
                      .size(),
                  0);
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).constraints().hard_min(j),
                  constraint.hard_min.x.constraint(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).constraints().hard_max(j),
                  constraint.hard_max.x.constraint(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().soft_min(j),
                  slack.soft_min.x(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().soft_max(j),
                  slack.soft_max.x(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().stiff_min(j),
                  slack.stiff_min.x(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().stiff_max(j),
                  slack.stiff_max.x(0, j));
      }
    } else if (lon_mpc_data_proto->trajectories(i).name() == "v") {
      for (int j = 0; j < lon_mpc_data_proto->trajectories(i).t().size(); j++) {
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).t(j), (double)j);
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).soln(j), x(1, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).ref(j), x_ref(1, j));
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().soft_min().size(),
            0);
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().soft_max().size(),
            0);
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().stiff_min(j),
            constraint.stiff_min.x.constraint(0, j));
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().stiff_max(j),
            constraint.stiff_max.x.constraint(0, j));
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().hard_min().size(),
            0);
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().hard_max().size(),
            0);
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().soft_min(j),
                  slack.soft_min.x(1, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().soft_max(j),
                  slack.soft_max.x(1, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().stiff_min(j),
                  slack.stiff_min.x(1, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().stiff_max(j),
                  slack.stiff_max.x(1, j));
      }
    } else if (lon_mpc_data_proto->trajectories(i).name() == "a") {
      for (int j = 0; j < lon_mpc_data_proto->trajectories(i).t().size(); j++) {
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).t(j), (double)j);
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).soln(j), u(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).ref(j), u(0, j));
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().soft_min().size(),
            0);
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().soft_max().size(),
            0);
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().stiff_min(j),
            constraint.stiff_min.u.constraint(0, j));
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().stiff_max(j),
            constraint.stiff_max.u.constraint(0, j));
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().hard_min().size(),
            0);
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().hard_max().size(),
            0);
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().soft_min(j),
                  slack.soft_min.u(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().soft_max(j),
                  slack.soft_max.u(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().stiff_min(j),
                  slack.stiff_min.u(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().stiff_max(j),
                  slack.stiff_max.u(0, j));
      }
    } else if (lon_mpc_data_proto->trajectories(i).name() == "j") {
      for (int j = 0; j < lon_mpc_data_proto->trajectories(i).t().size(); j++) {
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).t(j), (double)j - 1);
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).soln(j), u_dot(0, j));
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().soft_min().size(),
            0);
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().soft_max().size(),
            0);
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().stiff_min(j),
            constraint.stiff_min.u_dot.constraint(0, j));
        EXPECT_EQ(
            lon_mpc_data_proto->trajectories(i).constraints().stiff_max(j),
            constraint.stiff_max.u_dot.constraint(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).constraints().hard_min(j),
                  constraint.hard_min.u_dot.constraint(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).constraints().hard_max(j),
                  constraint.hard_max.u_dot.constraint(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().soft_min(j),
                  slack.soft_min.u_dot(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().soft_max(j),
                  slack.soft_max.u_dot(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().stiff_min(j),
                  slack.stiff_min.u_dot(0, j));
        EXPECT_EQ(lon_mpc_data_proto->trajectories(i).slacks().stiff_max(j),
                  slack.stiff_max.u_dot(0, j));
      }
    }
  }
}

TEST_F(ProtoConvertorTest, TestRecordSpeedLimitSetProto) {
  google::protobuf::RepeatedPtrField<PlanningDebugInfo_SpeedLimit>
      speed_limit_set_proto;

  const double dt = 0.1;
  SpeedLimit speed_limit;
  speed_limit.AppendSpeedLimit(0.0, 5.7446);
  speed_limit.AppendSpeedLimit(4.0, 5.0);
  speed_limit.AppendSpeedLimit(10.0, 5.0);
  speed_limit.AppendSpeedLimit(20.0, 5.0);
  speed_limit.AppendSpeedLimit(30.0, 5.0);
  speed_limit.AppendSpeedLimit(40.0, 5.0);
  speed_limit.AppendSpeedLimit(50.0, 5.0);
  std::vector<SpeedLimit> speed_limit_set;
  speed_limit_set.emplace_back(speed_limit);
  speed_limit_set.emplace_back(speed_limit);
  ProtoConvertor::RecordSpeedLimitSetProto(dt, speed_limit_set,
                                           speed_limit_set_proto);
  const std::string t0_name = "t=0.0s";
  const std::string t1_name = "t=0.1s";
  EXPECT_EQ(t0_name, speed_limit_set_proto.at(0).name());
  EXPECT_EQ(t1_name, speed_limit_set_proto.at(1).name());
  for (int k = 0; k < static_cast<int>(speed_limit_set.size()); k++) {
    for (int i = 0;
         i < static_cast<int>(speed_limit_set[k].speed_limit_points().size());
         ++i) {
      EXPECT_DOUBLE_EQ(speed_limit_set[k].speed_limit_points()[i].first,
                       speed_limit_set_proto[k].s(i));
      EXPECT_DOUBLE_EQ(speed_limit_set[k].speed_limit_points()[i].second,
                       speed_limit_set_proto[k].v(i));
    }
  }
}

TEST_F(ProtoConvertorTest, TestRecordTubesProto) {
  const int step = 10;
  Tube res_tube;

  for (int k = 0; k < step; k++) {
    Tube::TubePoint pts;
    ::math::Vec2d xy_left_ref_(0. + k, 0.);
    ::math::Vec2d xy_right_ref_(0. + k, 0.);
    ::math::Vec2d xy_left_soft_(0. + k, 1.);
    ::math::Vec2d xy_right_soft_(0. + k, -1.);
    ::math::Vec2d xy_left_stiff_(0. + k, 1.5);
    ::math::Vec2d xy_right_stiff_(0. + k, -1.5);
    ::math::Vec2d xy_left_hard_(0. + k, 2.);
    ::math::Vec2d xy_right_hard_(0. + k, -2.);
    pts.t = k;
    pts.xy_left_ref = xy_left_ref_;
    pts.xy_right_ref = xy_right_ref_;
    pts.xy_left_soft = xy_left_soft_;
    pts.xy_right_soft = xy_right_soft_;
    pts.xy_left_stiff = xy_left_stiff_;
    pts.xy_right_stiff = xy_right_stiff_;
    pts.xy_left_hard = xy_left_hard_;
    pts.xy_right_hard = xy_right_hard_;
    res_tube.pts.emplace_back(pts);
  }

  PlanningDebugInfo_Proposal_LatMPC_Tubes tubes_proto;
  ProtoConvertor::RecordTubesProto(res_tube, tubes_proto);

  for (int k = 0; k < static_cast<int>(res_tube.pts.size()); k++) {
    EXPECT_EQ(tubes_proto.t(k), res_tube.pts.at(k).t);
    EXPECT_EQ(tubes_proto.ref().xy_left(k).x(),
              res_tube.pts.at(k).xy_left_ref.x());
    EXPECT_EQ(tubes_proto.ref().xy_left(k).y(),
              res_tube.pts.at(k).xy_left_ref.y());
    EXPECT_EQ(tubes_proto.ref().xy_right(k).x(),
              res_tube.pts.at(k).xy_right_ref.x());
    EXPECT_EQ(tubes_proto.ref().xy_right(k).y(),
              res_tube.pts.at(k).xy_right_ref.y());
    EXPECT_EQ(tubes_proto.soft().xy_left(k).x(),
              res_tube.pts.at(k).xy_left_soft.x());
    EXPECT_EQ(tubes_proto.soft().xy_left(k).y(),
              res_tube.pts.at(k).xy_left_soft.y());
    EXPECT_EQ(tubes_proto.stiff().xy_left(k).x(),
              res_tube.pts.at(k).xy_left_stiff.x());
    EXPECT_EQ(tubes_proto.stiff().xy_left(k).y(),
              res_tube.pts.at(k).xy_left_stiff.y());
    EXPECT_EQ(tubes_proto.hard().xy_left(k).x(),
              res_tube.pts.at(k).xy_left_hard.x());
    EXPECT_EQ(tubes_proto.hard().xy_left(k).y(),
              res_tube.pts.at(k).xy_left_hard.y());
    EXPECT_EQ(tubes_proto.soft().xy_right(k).x(),
              res_tube.pts.at(k).xy_right_soft.x());
    EXPECT_EQ(tubes_proto.soft().xy_right(k).y(),
              res_tube.pts.at(k).xy_right_soft.y());
    EXPECT_EQ(tubes_proto.stiff().xy_right(k).x(),
              res_tube.pts.at(k).xy_right_stiff.x());
    EXPECT_EQ(tubes_proto.stiff().xy_right(k).y(),
              res_tube.pts.at(k).xy_right_stiff.y());
    EXPECT_EQ(tubes_proto.hard().xy_right(k).x(),
              res_tube.pts.at(k).xy_right_hard.x());
    EXPECT_EQ(tubes_proto.hard().xy_right(k).y(),
              res_tube.pts.at(k).xy_right_hard.y());
  }
}

TEST_F(ProtoConvertorTest, TestRecordTrajectoryProto) {
  Proposal proposal;
  DiscretizedTrajectory trajectory;
  const int num_point = 10;
  const double init_s = 0.0;
  const double init_v = 10.0;
  const double init_a = -2.0;
  const double delta_t = 0.1;
  for (int i = 0; i < num_point; ++i) {
    TrajectoryPoint trajectory_point;
    ::common::PathPoint path_point;
    const double time = delta_t * i;
    trajectory_point.set_relative_time(time);
    trajectory_point.set_v(init_v + init_a * time);
    trajectory_point.set_a(init_a);
    path_point.set_s(init_s + init_v * time + 0.5 * init_a * time * time);
    path_point.set_x(init_s + init_v * time + 0.5 * init_a * time * time);
    path_point.set_y(0.0);
    path_point.set_theta(0.5 * M_PI_2);
    path_point.set_kappa(0.0);
    trajectory_point.set_path_point(path_point);
    trajectory.emplace_back(trajectory_point);
  }
  proposal.SetTrajectory(trajectory);
  PlanningDebugInfo_Proposal_Trajectory trajectory_proto;
  ProtoConvertor::RecordTrajectoryProto(proposal, trajectory_proto);
  EXPECT_EQ(trajectory_proto.t().size(), num_point);
  EXPECT_EQ(trajectory_proto.s().size(), num_point);
  EXPECT_EQ(trajectory_proto.v().size(), num_point);
  EXPECT_EQ(trajectory_proto.a().size(), num_point);
  for (int i = 0; i < num_point; ++i) {
    EXPECT_EQ(trajectory_proto.t().at(i), trajectory[i].relative_time());
    EXPECT_EQ(trajectory_proto.v().at(i), trajectory[i].v());
    EXPECT_EQ(trajectory_proto.a().at(i), trajectory[i].a());
    EXPECT_EQ(trajectory_proto.x().at(i), trajectory[i].path_point().x());
    EXPECT_EQ(trajectory_proto.y().at(i), trajectory[i].path_point().y());
    EXPECT_EQ(trajectory_proto.s().at(i), trajectory[i].path_point().s());
    EXPECT_EQ(trajectory_proto.theta().at(i),
              trajectory[i].path_point().theta());
  }
}

TEST_F(ProtoConvertorTest, TestRecordHeader) {
  zark::planning::ADCTrajectory trajectory;
  trajectory.mutable_header()->set_timestamp_sec(1.0e-9);
  trajectory.mutable_header()->set_frame_id("key_frame");
  trajectory.mutable_header()->set_sequence_num(99999);
  trajectory.mutable_header()->mutable_status()->set_error_code(
      ::common::ErrorCode::PLANNING_ERROR);
  trajectory.mutable_header()->mutable_status()->set_msg("status message");

  zark::common::ExternedHeader header;
  ProtoConvertor::RecordHeader(trajectory, &header);

  EXPECT_DOUBLE_EQ(header.common_header().timestamp_nano(), 1.0);
  EXPECT_EQ(header.common_header().frame_id(), "key_frame");
  EXPECT_EQ(header.common_header().sequence_num(), 9);
  EXPECT_EQ(header.version(), 1);
  EXPECT_EQ(header.module_name(), "Planning");
  EXPECT_EQ(header.status().error_code(), ::common::ErrorCode::PLANNING_ERROR);
  EXPECT_EQ(header.status().msg(), "status message");
}

TEST_F(ProtoConvertorTest, TestRecordTrajectoryPublished) {
  zark::planning::ADCTrajectory trajectory;
  for (int i = 0; i < 100; i++) {
    ::common::TrajectoryPoint* p = trajectory.add_trajectory_point();
    ::common::PathPoint path_point;
    path_point.set_x(1.0);
    path_point.set_y(2.0);
    path_point.set_z(3.0);
    path_point.set_s(4.0);
    path_point.set_theta(5.0);
    path_point.set_kappa(6.0);
    path_point.set_dkappa(7.0);
    p->set_path_point(path_point);
    p->set_a(8.0);
    p->set_v(9.0);
    p->set_relative_time(10.0);
  }

  zark::planning::PlanningOutputMsg planning_output_proto;
  ProtoConvertor::RecordTrajectoryPublished(trajectory, planning_output_proto);

  EXPECT_EQ(planning_output_proto.trajectory_published().size(), 100);
  auto& trajectory_point = planning_output_proto.trajectory_published()[50];
  EXPECT_DOUBLE_EQ(trajectory_point.point().x(), 1.0);
  EXPECT_DOUBLE_EQ(trajectory_point.point().y(), 2.0);
  EXPECT_DOUBLE_EQ(trajectory_point.point().z(), 3.0);
  EXPECT_DOUBLE_EQ(trajectory_point.s(), 4.0);
  EXPECT_DOUBLE_EQ(trajectory_point.v(), 9.0);
  EXPECT_DOUBLE_EQ(trajectory_point.a(), 8.0);
  EXPECT_DOUBLE_EQ(trajectory_point.theta(), 5.0);
  EXPECT_DOUBLE_EQ(trajectory_point.kappa(), 6.0);
  EXPECT_DOUBLE_EQ(trajectory_point.dkappa(), 7.0);
  EXPECT_DOUBLE_EQ(trajectory_point.t(), 10.0);
}

TEST_F(ProtoConvertorTest, TestRecordTrajectoryPolyfit) {
  zark::planning::ADCTrajectory trajectory;
  PolynomialFit::TrajPolynomial traj_polyfit_out;
  traj_polyfit_out.lon_coefficients = Eigen::VectorXd(6);
  traj_polyfit_out.lon_coefficients << 0, 1, 2, 3, 4, 5;
  traj_polyfit_out.lat_coefficients = Eigen::VectorXd(6);
  traj_polyfit_out.lat_coefficients << 0, -1, -2, -3, -4, -5;
  traj_polyfit_out.length = 6;
  traj_polyfit_out.is_valid = true;
  trajectory.set_traj_polyfit_out(traj_polyfit_out);
  ::common::TrajectoryPoint trajectory_point;
  trajectory_point.set_a(0.0);
  trajectory_point.set_v(10.0);
  std::vector<::common::TrajectoryPoint> trajectory_points;
  trajectory_points.emplace_back(trajectory_point);
  trajectory.mutable_trajectory_point(trajectory_points);
  zark::planning::PlanningOutputMsg planning_output_proto;
  ProtoConvertor::RecordTrajectoryPolyfit(trajectory, planning_output_proto);

  EXPECT_EQ(planning_output_proto.trajectory_polyfit().is_valid(), true);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().traj_length(), 6);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().lon_fit_coeff().size(),
            6);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().lat_fit_coeff().size(),
            6);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().lon_fit_coeff()[3], 3);
  EXPECT_EQ(planning_output_proto.trajectory_polyfit().lat_fit_coeff()[3], -3);
}

TEST_F(ProtoConvertorTest, TestRecordPlanningStatus) {
  zark::planning::ADCTrajectory trajectory;
  trajectory.set_planning_status(PlanningStatus::NORMAL);
  zark::planning::PlanningOutputMsg planning_output_proto;
  ProtoConvertor::RecordPlanningStatus(trajectory, planning_output_proto);
  EXPECT_EQ(planning_output_proto.planning_status(), PlanningStatus::NORMAL);
}

TEST_F(ProtoConvertorTest, TestRecordLocalRoutePoint) {
  double x = 1.0;
  double y = 2.0;
  double heading = 3.0;
  double kappa = 4.0;
  double dkappa = 5.0;
  auto point = LocalRoutePoint(
      hdmap::MapPathPoint(::math::Vec2d(x, y), heading), kappa, dkappa);
  zark::planning::hdmap::Path map_path;
  int32_t path_point_index = 42;
  zark::reference_line_proto::ReferenceLineInfo out_line;
  ProtoConvertor::RecordLocalRoutePoint(point, map_path, path_point_index,
                                        &out_line);

  EXPECT_EQ(out_line.path_points().size(), 1);
  EXPECT_EQ(out_line.path_points()[0].theta(), heading);
  EXPECT_EQ(out_line.path_points()[0].kappa(), kappa);
  EXPECT_EQ(out_line.path_points()[0].dkappa(), dkappa);
  EXPECT_EQ(out_line.path_points()[0].base_point().x(), x);
  EXPECT_EQ(out_line.path_points()[0].base_point().y(), y);
}

TEST_F(ProtoConvertorTest, TestRecordLaneSegment) {
  zark::planning::hdmap::LaneSegment lane_seg;
  zark::reference_line_proto::ReferenceLineInfo out_line;
  ProtoConvertor::RecordLaneSegment(lane_seg, &out_line);
  EXPECT_EQ(out_line.lane_info().size(), 0);
}

}  // namespace common
}  // namespace planning
}  // namespace zark
