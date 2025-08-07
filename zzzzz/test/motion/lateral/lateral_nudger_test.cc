#include "apps/planning/src/motion/lateral/lateral_nudger.h"

#include "apps/planning/src/config/config_main.h"
#include "apps/planning/src/motion/lateral/data_type.h"
#include "apps/planning/src/motion/lateral/lateral_tube.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

namespace {
const int n_steps = 40;  // same to lateral_optimizer_config_test
const int n_nodes = n_steps + 1;
const double kLMax = 100.0;
const double kTol = 1.0e-3;
const int n_x = 4;
const int n_u = 1;
const int n_u_dot = 1;
const double dt = 0.2;
const int kIdxS = 0;
}  // namespace

class LateralNudgerTest : public ::testing::Test {
 public:
  void SetUp() override {
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "lateral_optimizer_config_test.json";
    Config config{config_file};
    std::ifstream jdata(config_file);
    lateral_config_ =
        config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
    lat_nudger_ =
        std::unique_ptr<LateralNudger>(new LateralNudger(lateral_config_));
    common::VehicleConfig vehicle_config;
    vehicle_param_.set_width(2.11);
    vehicle_param_.set_length(4.933);
    vehicle_param_.set_steer_ratio(16);
    vehicle_param_.set_max_steer_angle(8.20304748437);
    vehicle_param_.set_max_steer_angle_rate(8.55211);
    vehicle_config.set_vehicle_param(vehicle_param_);
    common::VehicleConfigHelper::Init(vehicle_config);
  }

 protected:
  LateralOptimizerConfig lateral_config_;
  std::unique_ptr<LateralNudger> lat_nudger_;
  common::VehicleParam vehicle_param_;
};

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

CorridorInfo InitializeCorridorInfo(::math::Box2d& box_2d) {
  static LocalRoute local_route{};
  local_route = LocalRoute();
  static Mission misssion{};
  misssion = Mission();
  CorridorInfo::Type corridor_type;

  std::vector<CorridorPoint> corridor_points;
  const double speed = 3.0;
  for (int k = 0; k < n_nodes; k++) {
    CorridorPoint corridor_pt;
    corridor_pt.s = k * dt * speed;
    corridor_pt.l = 0.5;
    corridor_pt.s_ref = k * dt * speed;
    corridor_pt.l_ref = 0.0;
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
  const std::string id = "1";
  Obstacle obstacle(id, perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obs_map;
  obs_map.Add(id, &obstacle);
  for (auto obs_info : obs_map.Dict()) {
    AINFO << "obs_info_Id = " << obs_info.first
          << " id = " << obs_info.second->Id() << std::endl;
  }

  std::vector<std::pair<const Obstacle*, bool>> lateral_obstacles;
  lateral_obstacles.emplace_back(std::make_pair(&obstacle, false));
  lateral_obstacles.emplace_back(std::make_pair(&obstacle, true));
  CorridorInfo corr_info(local_route, misssion, corridor_type, corridor,
                         idx_start_point, obs_map);

  corr_info.SetLateralObstacles(lateral_obstacles);
  std::unordered_map<const Obstacle*, SLTrajectory> obs_sl_boundary_map;
  SLTrajectory sl_boundarys;
  SLBoundary sl_boundary;
  sl_boundary.set_start_s(-1.0);
  sl_boundary.set_end_s(1.0);
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.0);
  sl_boundarys.emplace_back(sl_boundary);
  obs_sl_boundary_map.emplace(std::make_pair(&obstacle, sl_boundarys));
  corr_info.SetObsSLBoundayMap(obs_sl_boundary_map);
  return corr_info;
}

void InitializeLonMPCData(LonMPCData& lon_mpc_data) {
  lon_mpc_data.x.resize(2, n_nodes);
  lon_mpc_data.t.resize(1, n_nodes);
  lon_mpc_data.u.resize(1, n_steps);
  lon_mpc_data.u_dot.resize(1, n_steps);
  lon_mpc_data.x.row(0) = Eigen::RowVectorXd::Ones(n_nodes);
  lon_mpc_data.x.row(1) = Eigen::RowVectorXd::Ones(n_nodes);
  lon_mpc_data.u.row(0) = Eigen::RowVectorXd::Zero(n_steps);
  lon_mpc_data.u_dot.row(0) = Eigen::RowVectorXd::Zero(n_steps);
  for (int k = 0; k < n_nodes; k++) {
    lon_mpc_data.t[k] = dt * k;
  }
}

TEST_F(LateralNudgerTest, ConstructNudger) {
  LonMPCData lon_mpc_data;
  InitializeLonMPCData(lon_mpc_data);
  ::math::Box2d box_2d;
  CorridorInfo corridor_info = InitializeCorridorInfo(box_2d);

  // Obstacle map
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_length(1.0);
  ::common::Trajectory trajectory =
      generate_lat_mpc_trajectory(0.0, 1.0, 5.0, 10.0);
  const std::string id = "1";
  Obstacle obstacle(id, perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obs_map;
  obs_map.Add(id, &obstacle);

  std::vector<std::pair<const Obstacle*, bool>> lateral_obstacles;
  lateral_obstacles.emplace_back(std::make_pair(&obstacle, false));
  lateral_obstacles.emplace_back(std::make_pair(&obstacle, true));

  const std::vector<Nudger> lateral_nudgers = lat_nudger_->ConstructNudger(
      lateral_obstacles, lon_mpc_data, corridor_info);
  for (auto lateral_nudger : lateral_nudgers) {
    for (int k = 0; k < n_nodes; k++) {
      EXPECT_EQ(std::isnan(lateral_nudger.l.at(k)), true);
      EXPECT_EQ(std::isnan(lateral_nudger.v.at(k)), true);
    }
  }
}

}  // namespace planning
}  // namespace zark
