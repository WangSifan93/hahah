#include "apps/planning/src/motion/lateral/lateral_tube.h"

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/perception_obstacle.h"
#include "apps/planning/src/config/config_main.h"
#include "apps/planning/src/motion/lateral/data_type.h"
#include "apps/planning/src/motion/lateral/lateral_padding.h"
#include "apps/planning/src/motion/lateral/lateral_tube.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

class LateralTubeTest : public ::testing::Test {
 public:
  void SetUp() override {
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "lateral_optimizer_config_test.json";
    Config config{config_file};
    std::ifstream jdata(config_file);
    lateral_config_ =
        config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
    lat_tube_ptr_ =
        std::unique_ptr<LateralTube>(new LateralTube(lateral_config_));
    common::VehicleConfig vehicle_config;
    vehicle_param_.set_width(2.11);
    vehicle_config.set_vehicle_param(vehicle_param_);
    common::VehicleConfigHelper::Init(vehicle_config);
  }

 protected:
  LateralOptimizerConfig lateral_config_;
  LateralPadding lateral_padding;
  std::unique_ptr<LateralTube> lat_tube_ptr_;
  common::VehicleParam vehicle_param_;
};

void GenerateLocalRoutePoints(
    std::vector<LocalRoutePoint> &local_route_points) {
  for (double i = 0.0; i < 300.0; i = i + 0.5) {
    double x = i;
    double y = 0.0;
    double heading = 0.0;
    double kappa = 0.0;
    double dkappa = 0.0;
    local_route_points.emplace_back(LocalRoutePoint(
        hdmap::MapPathPoint(math::Vec2d(x, y), heading), kappa, dkappa));
  }
}

void SetConfigFile(LocalRouteConfig &local_route_config) {
  common::VehicleConfig vehicle_config;
  common::VehicleParam vehicle_param_;
  vehicle_param_.set_width(2.11);
  vehicle_param_.set_length(4.933);
  vehicle_param_.set_steer_ratio(16);
  vehicle_param_.set_max_steer_angle(8.20304748437);
  vehicle_param_.set_max_steer_angle_rate(8.55211);
  vehicle_config.set_vehicle_param(vehicle_param_);
  common::VehicleConfigHelper::Init(vehicle_config);
  std::string reference_line_config_file(
      "/zark/apps/planning/test/config/json/frame_conf_test.json");
  zark::planning::Config config_instance{reference_line_config_file};
  config_instance.SetLocalRouteConfig(local_route_config);
}

::common::Trajectory generate_lat_mpc_trajectory(double x_start, double y_start,
                                                 double x_end, double y_end) {
  ::common::Trajectory trajectory;
  const int num_points = 5;
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    double x = x_start + t * (x_end - x_start);
    double y = y_start + t * (y_end - y_start);
    ::common::PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    double v = 3.0;
    double a = 1.0;
    ::common::TrajectoryPoint trajectory_point;
    trajectory_point.set_path_point(path_point);
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(0.0 + 0.1 * i);
    trajectory.mutable_trajectory_point()->emplace_back(trajectory_point);
  }
  return trajectory;
}

CorridorInfo InitializeCorridorInfo() {
  static LocalRoute local_route{};
  local_route = LocalRoute();
  static Mission misssion{};
  misssion = Mission();
  CorridorInfo::Type corridor_type;
  const int n_steps = 40;  // same to lateral_optimizer_config_test
  const int n_nodes = n_steps + 1;
  const double dt = 0.2;

  std::vector<CorridorPoint> corridor_points;
  const double speed = 3.0;
  for (int k = 0; k < n_nodes; k++) {
    CorridorPoint corridor_pt;
    corridor_pt.s = k * dt * speed;
    corridor_pt.l = 0.5;
    corridor_pt.s_ref = k * dt * speed;
    corridor_pt.l_ref = -0.1;
    corridor_pt.l_left = 2.5;
    corridor_pt.l_right = -2.5;
    corridor_pt.theta = 0.0;
    corridor_pt.kappa = 0.01;
    corridor_points.emplace_back(corridor_pt);
  }
  Corridor corridor(corridor_points);
  const int idx_start_point = 0;

  // Obstacle map
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_length(1.0);
  ::common::Trajectory trajectory =
      generate_lat_mpc_trajectory(0.0, 1.0, 5.0, 10.0);
  Obstacle obstacle("1", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obs_map;
  obs_map.Add("1", &obstacle);
  CorridorInfo corr_info(local_route, misssion, corridor_type, corridor,
                         idx_start_point, obs_map);
  std::vector<std::pair<const Obstacle*, bool>> lateral_obstacles;
  lateral_obstacles.emplace_back(std::make_pair(&obstacle, false));
  corr_info.SetLateralObstacles(lateral_obstacles);
  ::common::FrenetPoint init_frenet_point;
  init_frenet_point.s[0] = 0.0;
  init_frenet_point.s[1] = 2.0;
  init_frenet_point.l[0] = 0.5;
  init_frenet_point.l[1] = 1.0;
  corr_info.SetInitFrenetPoint(init_frenet_point);

  return corr_info;
}

void InitializeTube(Tube& tube, const common::VehicleParam& vehicle_param) {
  const int n_steps = 40;
  const int n_nodes = n_steps + 1;
  tube.pts.reserve(n_nodes);
  const double time_step = 0.1;
  const double speed = 3.0;
  for (int k = 0; k < n_nodes; k++) {
    Tube::TubePoint tube_point;
    tube_point.t = time_step * k;
    tube_point.s = speed * time_step * k;
    tube_point.l_left_ref = vehicle_param.width() * 0.5;
    tube_point.l_right_ref = -vehicle_param.width() * 0.5;
    tube_point.l_left_soft = 2.0;
    tube_point.l_right_soft = -3.0;
    tube_point.l_left_stiff = 2.0;
    tube_point.l_right_stiff = -3.0;
    tube_point.l_left_hard = 2.0;
    tube_point.l_right_hard = -3.0;
    tube.pts.emplace_back(tube_point);
  }
  tube.pts[1].l_left_ref = -1.0;
  tube.pts[2].l_left_ref = 0.0;
  tube.pts[5].l_left_ref = -1.0;
  tube.pts[24].l_left_ref = 0.0;
  tube.pts[25].l_left_ref = -1.0;
  tube.pts[26].l_left_ref = -2.0;
  tube.pts[27].l_left_ref = -3.0;
  tube.pts[28].l_left_ref = 0.0;

  tube.pts[2].l_right_ref = 1.0;
  tube.pts[3].l_right_ref = 0.0;
  tube.pts[10].l_right_ref = 1.0;
  tube.pts[15].l_right_ref = 1.0;
}

TEST_F(LateralTubeTest, ConstructTubes) {
  const double speed = 1.0;
  const int n_steps = this->lateral_config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  const double dt = this->lateral_config_.model.dt;

  ::common::TrajectoryPoint planning_start_point;
  planning_start_point.set_v(speed);
  LonMPCData lon_mpc_data;
  std::vector<CorridorPoint> corridor_points;
  lon_mpc_data.t.resize(n_nodes);
  lon_mpc_data.x.resize(1, n_nodes);
  for (int i = 0; i < n_nodes; i++) {
    lon_mpc_data.t[i] = dt * i;
    lon_mpc_data.x(0, i) = speed * lon_mpc_data.t(i);
    CorridorPoint corridor_point;
    corridor_point.s = lon_mpc_data.x(0, i);
    corridor_point.l = 0.0;
    corridor_point.l_left = 1.85;
    corridor_point.l_right = -1.85;
    corridor_point.type_left = CorridorPoint::Type::CURB;
    corridor_point.type_right = CorridorPoint::Type::UNKNOWN;
    corridor_points.push_back(corridor_point);
  }

  LocalRouteConfig local_route_config;
  SetConfigFile(local_route_config);
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(local_route_points);
  LocalRoute local_route(local_route_points, local_route_config);
  
  Mission mission{};
  CorridorInfo::Type corridor_type;
  Corridor corridor(corridor_points);

  int idx_start_point = 0;
  IndexedPtrList<std::string, const Obstacle*> obstacle_map;
  CorridorInfo corridor_info(local_route, mission, corridor_type, corridor,
                             idx_start_point, obstacle_map);

  std::vector<Nudger> nudgers;
  Nudger nudge_1(n_nodes);
  Nudger nudge_2(n_nodes);
  Nudger nudge_3(n_nodes);
  perception::PerceptionObstacle perception_obstacle;
  ::common::Point3D obs_pos(4.0, -1.0, 0);
  perception_obstacle.set_position(obs_pos);
  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(8.0);
  perception_obstacle.set_theta(0.0);
  perception_obstacle.set_sub_type(perception::SubType::ST_PEDESTRIAN);
  const Obstacle obs_("test_obs", perception_obstacle);
  nudge_2.obs = &obs_;
  nudge_2.is_left = false;
  for (int i = 0; i < 1; i++) {
    nudge_2.v[i] = 0.0;
    nudge_2.l[i] = -0.9;
  }

  nudge_3.obs = &obs_;
  nudge_3.is_left = true;
  for (int i = 0; i < 1; i++) {
    nudge_3.v[i] = 0.0;
    nudge_3.l[i] = 0.9;
  }

  nudgers.push_back(nudge_1);
  nudgers.push_back(nudge_2);
  nudgers.push_back(nudge_3);

  LateralLookupTables lateral_tables(lateral_config_);
  STProposal st_proposal;

  STBoundary st_boundary_1, st_boundary_2, st_boundary_3;
  std::vector<STBoundary> st_graph;
  std::vector<STPoint> lower_points, upper_points;
  bool is_front = true;
  bool is_filtered = false;

  st_boundary_1.mutable_lower_points()->emplace_back(STPoint(1.0, 0.1));
  st_boundary_1.mutable_lower_points()->emplace_back(STPoint(3.0, 0.4));
  st_boundary_1.mutable_upper_points()->emplace_back(STPoint(2.0, 0.1));
  st_boundary_1.mutable_upper_points()->emplace_back(STPoint(4.0, 0.4));
  st_proposal.emplace_back(
      std::make_tuple(&st_boundary_1, is_front, is_filtered));

  is_front = true;
  is_filtered = false;
  st_boundary_2.mutable_lower_points()->emplace_back(STPoint(0.9, 0.1));
  st_boundary_2.mutable_lower_points()->emplace_back(STPoint(1.0, 0.4));
  st_boundary_2.mutable_upper_points()->emplace_back(STPoint(2.0, 0.1));
  st_boundary_2.mutable_upper_points()->emplace_back(STPoint(2.0, 0.4));
  st_proposal.emplace_back(
      std::make_tuple(&st_boundary_2, is_front, is_filtered));

  is_front = false;
  is_filtered = false;
  st_boundary_3.mutable_lower_points()->emplace_back(STPoint(0.8, 0.1));
  st_boundary_3.mutable_lower_points()->emplace_back(STPoint(1.0, 0.4));
  st_boundary_3.mutable_upper_points()->emplace_back(STPoint(2.0, 0.1));
  st_boundary_3.mutable_upper_points()->emplace_back(STPoint(2.0, 0.4));
  st_proposal.emplace_back(
      std::make_tuple(&st_boundary_3, is_front, is_filtered));

  Tube res_tube = lat_tube_ptr_->ConstructTubes(
      planning_start_point, nudgers, corridor_info, lon_mpc_data, corridor,
      std::vector<Tube::TubePoint>(), lateral_tables, lateral_padding,
      st_proposal, local_route);
  const double kTol = 1.0e-6;
  for (int k = 0; k < 1; ++k) {
    EXPECT_NEAR(res_tube.pts[k].t, lon_mpc_data.t(k), kTol);
    EXPECT_NEAR(res_tube.pts[k].s, lon_mpc_data.x(0, k), kTol);
    EXPECT_NEAR(res_tube.pts[k].l_left_soft, 0.7, kTol);
    EXPECT_NEAR(res_tube.pts[k].l_right_soft, -0.7, kTol);
    EXPECT_NEAR(res_tube.pts[k].l_left_stiff, 0.8, kTol);
    EXPECT_NEAR(res_tube.pts[k].l_right_stiff, -0.8, kTol);
    EXPECT_NEAR(res_tube.pts[k].l_left_hard, 0.9, kTol);
    EXPECT_NEAR(res_tube.pts[k].l_right_hard, -0.9, kTol);
    EXPECT_NEAR(res_tube.pts[k].l_left_ref, 1.155, kTol);
    EXPECT_NEAR(res_tube.pts[k].l_right_ref, -1.155, kTol);
  }
}

TEST_F(LateralTubeTest, TestSmoothTubeReference) {
  Tube tube;
  InitializeTube(tube, vehicle_param_);
  lat_tube_ptr_->SmoothTubeReference(tube.pts, true);
  lat_tube_ptr_->SmoothTubeReference(tube.pts, false);
  const double kTol = 1.0e-6;

  EXPECT_NEAR(tube.pts[0].l_left_ref, -0.9, kTol);
  EXPECT_NEAR(tube.pts[1].l_left_ref, -1.0, kTol);
  EXPECT_NEAR(tube.pts[2].l_left_ref, -0.9, kTol);
  EXPECT_NEAR(tube.pts[3].l_left_ref, -0.8, kTol);
  EXPECT_NEAR(tube.pts[4].l_left_ref, -0.9, kTol);
  EXPECT_NEAR(tube.pts[5].l_left_ref, -1.0, kTol);
  EXPECT_NEAR(tube.pts[6].l_left_ref, -0.9, kTol);
  EXPECT_NEAR(tube.pts[7].l_left_ref, -1.0, kTol);
  EXPECT_NEAR(tube.pts[8].l_left_ref, -1.1, kTol);
  EXPECT_NEAR(tube.pts[15].l_left_ref, -1.8, kTol);
  EXPECT_NEAR(tube.pts[24].l_left_ref, -2.7, kTol);
  EXPECT_NEAR(tube.pts[25].l_left_ref, -2.8, kTol);
  EXPECT_NEAR(tube.pts[26].l_left_ref, -2.9, kTol);
  EXPECT_NEAR(tube.pts[27].l_left_ref, -3.0, kTol);
  EXPECT_NEAR(tube.pts[28].l_left_ref, -2.9, kTol);
  EXPECT_NEAR(tube.pts[40].l_left_ref, -1.7, kTol);

  EXPECT_NEAR(tube.pts[0].l_right_ref, 0.8, kTol);
  EXPECT_NEAR(tube.pts[1].l_right_ref, 0.9, kTol);
  EXPECT_NEAR(tube.pts[2].l_right_ref, 1.0, kTol);
  EXPECT_NEAR(tube.pts[3].l_right_ref, 0.9, kTol);
  EXPECT_NEAR(tube.pts[4].l_right_ref, 0.8, kTol);
  EXPECT_NEAR(tube.pts[5].l_right_ref, 0.7, kTol);
  EXPECT_NEAR(tube.pts[6].l_right_ref, 0.6, kTol);
  EXPECT_NEAR(tube.pts[7].l_right_ref, 0.7, kTol);
  EXPECT_NEAR(tube.pts[8].l_right_ref, 0.8, kTol);
  EXPECT_NEAR(tube.pts[9].l_right_ref, 0.9, kTol);
  EXPECT_NEAR(tube.pts[10].l_right_ref, 1.0, kTol);
  EXPECT_NEAR(tube.pts[12].l_right_ref, 0.8, kTol);
  EXPECT_NEAR(tube.pts[13].l_right_ref, 0.8, kTol);
  EXPECT_NEAR(tube.pts[15].l_right_ref, 1.0, kTol);
  EXPECT_NEAR(tube.pts[40].l_right_ref, -1.055, kTol);
}

TEST_F(LateralTubeTest, TestEvaluteByS) {
  Tube tube;
  InitializeTube(tube, vehicle_param_);
  const double s_test = 5.0;
  Tube::TubePoint point = lat_tube_ptr_->EvaluateByS(tube.pts, s_test);
  const double kTol = 1.0e-3;
  EXPECT_NEAR(point.l_left_ref, 1.055, kTol);
  EXPECT_NEAR(point.l_right_ref, -1.055, kTol);
}

TEST_F(LateralTubeTest, TestFindBottomLeftPoint) {
  Tube tube;
  InitializeTube(tube, vehicle_param_);
  STProposal st_proposal;
  STBoundary st_boundary_1, st_boundary_2, st_boundary_3;
  std::vector<STBoundary> st_graph;
  std::vector<STPoint> lower_points, upper_points;
  bool is_front = true;
  bool is_filtered = false;

  st_boundary_1.mutable_lower_points()->emplace_back(STPoint(1.0, 0.1));
  st_boundary_1.mutable_lower_points()->emplace_back(STPoint(3.0, 0.4));
  st_boundary_1.mutable_upper_points()->emplace_back(STPoint(2.0, 0.1));
  st_boundary_1.mutable_upper_points()->emplace_back(STPoint(4.0, 0.4));
  st_proposal.emplace_back(
      std::make_tuple(&st_boundary_1, is_front, is_filtered));

  is_front = true;
  is_filtered = false;
  st_boundary_2.mutable_lower_points()->emplace_back(STPoint(0.9, 0.1));
  st_boundary_2.mutable_lower_points()->emplace_back(STPoint(1.0, 0.4));
  st_boundary_2.mutable_upper_points()->emplace_back(STPoint(2.0, 0.1));
  st_boundary_2.mutable_upper_points()->emplace_back(STPoint(2.0, 0.4));
  st_proposal.emplace_back(
      std::make_tuple(&st_boundary_2, is_front, is_filtered));

  is_front = false;
  is_filtered = false;
  st_boundary_3.mutable_lower_points()->emplace_back(STPoint(0.8, 0.1));
  st_boundary_3.mutable_lower_points()->emplace_back(STPoint(1.0, 0.4));
  st_boundary_3.mutable_upper_points()->emplace_back(STPoint(2.0, 0.1));
  st_boundary_3.mutable_upper_points()->emplace_back(STPoint(2.0, 0.4));
  st_proposal.emplace_back(
      std::make_tuple(&st_boundary_3, is_front, is_filtered));

  auto st_point = lat_tube_ptr_->FindBottomLeftPoint(st_proposal);
  const double kTol = 1.0e-3;
  if (st_point != std::nullopt) {
    EXPECT_NEAR(st_point->t(), 0.1, kTol);
    EXPECT_NEAR(st_point->s(), 0.9, kTol);
  }
}

TEST_F(LateralTubeTest, TestGetCorridorAxisLanesWidth) {
  CorridorInfo corridor_info = InitializeCorridorInfo();
  Corridor corridor = corridor_info.GetCorridor();
  LocalRouteConfig local_route_config;
  SetConfigFile(local_route_config);
  std::vector<LocalRoutePoint> local_route_points;
  GenerateLocalRoutePoints(local_route_points);
  LocalRoute local_route(local_route_points, local_route_config);
  double lanes_left_boundary, lanes_right_boundary;
  for (int k = 0; k < static_cast<int>(corridor.size()); k++) {
    lat_tube_ptr_->GetCorridorAxisLanesWidth(
        corridor[k], local_route, lanes_left_boundary, lanes_right_boundary);
    EXPECT_DOUBLE_EQ(lanes_left_boundary, 1.975);
    EXPECT_DOUBLE_EQ(lanes_right_boundary, -1.775);
  }
}

}  // namespace planning
}  // namespace zark
