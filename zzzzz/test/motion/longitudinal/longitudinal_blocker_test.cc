#include "apps/planning/src/motion/longitudinal/longitudinal_blocker.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/config/config_main.h"
#include "apps/planning/src/decision/longitudinal/data_type.h"
#include "gtest/gtest.h"

using namespace std;
namespace zark {
namespace planning {

TEST(LongitudinalBlockerTest, TestComputeLonSpeed) {
  const ::common::PathPoint path_point(1.0, 2.0, 0.0, 0.0, 0.01, 0.1, 0.1, 0.1,
                                       "", 0.1, 0.1);
  const ::common::GaussianInfo gaussian_info;
  ::common::TrajectoryPoint obs_point(path_point, 5.0, 0.1, 2.0, 0.1, 0.1,
                                      gaussian_info);
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  zark::planning::Config config{config_file};

  CorridorPoint corr_point;
  corr_point.theta = 0.0;
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalBlocker longitudinal_blocker(longitudinal_optimizer_config.model);
  EXPECT_DOUBLE_EQ(longitudinal_blocker.ComputeLonSpeed(obs_point, corr_point),
                   5.0);
}

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

::common::Trajectory generate_trajectory(double x_start, double y_start,
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
    trajectory_point.set_relative_time(0.0 + 1.0 * i);
    trajectory.mutable_trajectory_point()->emplace_back(trajectory_point);
  }
  return trajectory;
}

TEST(LongitudinalBlockerTest, TestConstructBlocker) {
  const ::common::PathPoint path_point(1.0, 2.0, 0.0, 0.0, 0.01, 0.1, 0.1, 0.1,
                                       "", 0.1, 0.1);
  const ::common::GaussianInfo gaussian_info;
  ::common::TrajectoryPoint obs_point(path_point, 5.0, 0.1, 2.0, 0.1, 0.1,
                                      gaussian_info);
  std::string config_file =
      "/zark/apps/planning/test/config/json/"
      "longitudinal_optimizer_config_test.json";
  LongitudinalOptimizerConfig longitudinal_optimizer_config;
  zark::planning::Config lon_config{config_file};
  std::ifstream jdata(config_file);
  longitudinal_optimizer_config =
      lon_config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
  LongitudinalBlocker lon_blocker(longitudinal_optimizer_config.model);

  // Proposal
  std::vector<STBoundary> st_graph;
  STBoundary st_boundary;
  st_boundary = CreatObject(3.0, 0.0, 0.0, 4.0, 5.0);
  st_boundary.set_id("1");
  st_graph.push_back(st_boundary);
  STProposal st_proposal;
  st_proposal.emplace_back(std::make_tuple(&st_boundary, true, true));
  LocalRoute local_route{};
  Mission misssion{};
  CorridorInfo::Type corridor_type;
  CorridorPoint corr_pt_1, corr_pt_2, corr_pt_3, corr_pt_4, corr_pt_5;
  corr_pt_1.s = 0.0;
  corr_pt_1.l = 0.0;
  corr_pt_2.s = 6.0;
  corr_pt_2.l = 0.0;
  corr_pt_3.s = 12.0;
  corr_pt_3.l = 0.0;
  corr_pt_4.s = 18.0;
  corr_pt_4.l = 0.0;
  corr_pt_5.s = 24.0;
  corr_pt_5.l = 0.0;
  Corridor corridor_points;
  corridor_points.emplace_back(corr_pt_1);
  corridor_points.emplace_back(corr_pt_2);
  corridor_points.emplace_back(corr_pt_3);
  corridor_points.emplace_back(corr_pt_4);
  corridor_points.emplace_back(corr_pt_5);
  // Obstacle map
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_length(1.0);
  ::common::Trajectory trajectory = generate_trajectory(0.0, 1.0, 0.0, 10.0);
  Obstacle obstacle("1", perception_obstacle, trajectory);
  IndexedPtrList<std::string, const Obstacle*> obs_map;
  obs_map.Add("1", &obstacle);
  int idx_start_point = 0;
  CorridorInfo corr_info(local_route, misssion, corridor_type, corridor_points,
                         idx_start_point, obs_map);
  DiscretizedTrajectory dis_trajectory;
  ::common::TrajectoryPoint trajectory_point;
  double ego_v = 2.0;
  for (int i = 0; i < 50; ++i) {
    trajectory_point.mutable_path_point()->set_x(0.0 + i * ego_v * 0.1);
    trajectory_point.mutable_path_point()->set_y(0.0);
    trajectory_point.mutable_path_point()->set_theta(0.0);
    trajectory_point.set_v(ego_v);
    trajectory_point.set_relative_time(0.0 + i * 0.1);
    dis_trajectory.AppendTrajectoryPoint(trajectory_point);
  }
  std::vector<Blocker> blockers =
      lon_blocker.ConstructBlocker(st_proposal, obs_map, corridor_points);
  std::cout << blockers.size() << std::endl;

  EXPECT_EQ(blockers.size(), 1);
  std::vector<std::pair<double, double>> exp_results;
  exp_results.emplace_back(std::make_pair(0.0, 3.0));
  exp_results.emplace_back(std::make_pair(3.0, 3.0));
  exp_results.emplace_back(std::make_pair(6.0, 3.0));
  exp_results.emplace_back(std::make_pair(9.0, 3.0));
  exp_results.emplace_back(std::make_pair(12.0, 3.0));
  for (Blocker blocker : blockers) {
    for (size_t k = 0; k < blocker.s.size(); ++k) {
      EXPECT_DOUBLE_EQ(exp_results[k].first, blocker.s[k]);
      EXPECT_DOUBLE_EQ(exp_results[k].second, blocker.v[k]);
    }
  }
}

}  // namespace planning
}  // namespace zark
