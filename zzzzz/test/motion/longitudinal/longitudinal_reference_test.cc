#include "apps/planning/src/motion/longitudinal/longitudinal_reference.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"
#include "gtest/gtest_prod.h"

const double kTol = 1e-2;

namespace zark {
namespace planning {

class LongitudinalReferenceTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "longitudinal_optimizer_config_test.json";
    LongitudinalOptimizerConfig longitudinal_optimizer_config;
    zark::planning::Config config{config_file};
    std::ifstream jdata(config_file);
    longitudinal_optimizer_config =
        config.SetLongitudinalOptimizerConfig(nlohmann::json::parse(jdata));
    LongitudinalLookupTables lon_lookup_tables(longitudinal_optimizer_config);
    lon_ref_ =
        std::make_shared<LongitudinalReference>(longitudinal_optimizer_config);
    lon_lookup_tables_ = std::make_shared<LongitudinalLookupTables>(
        longitudinal_optimizer_config);
  }

 protected:
  std::shared_ptr<LongitudinalReference> lon_ref_;
  std::shared_ptr<LongitudinalLookupTables> lon_lookup_tables_;
};

TEST_F(LongitudinalReferenceTest, TestDesignReferenceTrajectory) {
  Eigen::VectorXd x_init(3);
  x_init(0) = -2.0;
  x_init(1) = 5.0;
  SpeedLimit speed_limit_map_smooth;
  speed_limit_map_smooth.AppendSpeedLimit(-2.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(10.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(20.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(30.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(40.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(50.0, 8.0);
  std::vector<std::vector<std::pair<double, double>>> sv_follow_pairs;
  Blocker blocker_1(5);
  Blocker blocker_2(5);
  std::vector<Blocker> blockers;
  blocker_1.is_filtered = true;
  blocker_1.is_front = true;
  blocker_1.s[1] = 20.0;
  blocker_1.s[2] = 25.0;
  blocker_1.s[3] = 30.0;
  blocker_1.v[1] = 5.0;
  blocker_1.v[2] = 5.0;
  blocker_1.v[3] = 5.0;
  blocker_1.k_start = 1;
  blocker_1.k_end = 3;
  blocker_2.is_filtered = true;
  blocker_2.is_front = true;
  blocker_2.v[3] = 5.0;
  blocker_2.v[4] = 5.0;
  blocker_2.s[3] = 10.0;
  blocker_2.s[4] = 15.0;
  blocker_2.k_start = 3;
  blocker_2.k_end = 4;
  Obstacle obstacle1, obstacle2;
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_theta(0.0);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_id(1);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  obstacle1 = Obstacle("1_0", perception_obstacle);
  perception_obstacle.set_id(2);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_sub_type(perception::SubType::ST_UNKNOWN);
  obstacle2 = Obstacle("2_0", perception_obstacle);
  blocker_1.obs = &obstacle1;
  blocker_2.obs = &obstacle2;
  blockers.emplace_back(blocker_1);
  blockers.emplace_back(blocker_2);
  LongitudinalPadding lon_padding;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  std::vector<SpeedLimit> speed_limit_set;
  std::tie(x_ref, u_ref) = lon_ref_->DesignReferenceTrajectory(
      x_init, speed_limit_map_smooth, blockers, TimeGapLevel::LEVEL_3,
      *lon_lookup_tables_, lon_padding, speed_limit_set);
  EXPECT_EQ(x_ref.rows(), 2);
  EXPECT_EQ(x_ref.cols(), 5);
  EXPECT_EQ(u_ref.rows(), 1);
  EXPECT_EQ(u_ref.cols(), 4);
}

TEST_F(LongitudinalReferenceTest, TestLowerSpeedLimitCurveForObstacle) {
  SpeedLimit speed_limit;
  speed_limit.AppendSpeedLimit(0.0, 8.0);
  speed_limit.AppendSpeedLimit(10.0, 8.0);
  speed_limit.AppendSpeedLimit(20.0, 8.0);
  speed_limit.AppendSpeedLimit(30.0, 8.0);
  speed_limit.AppendSpeedLimit(40.0, 8.0);
  speed_limit.AppendSpeedLimit(50.0, 8.0);
  const double s_follow = 29.0;
  const double v_follow = 5.0;
  lon_ref_->LowerSpeedLimitCurveForObstacle(s_follow, v_follow, speed_limit);
  SpeedLimit matlab_lower_limit_results;
  matlab_lower_limit_results.AppendSpeedLimit(0.0, 8.0);
  matlab_lower_limit_results.AppendSpeedLimit(10.0, 8.0);
  matlab_lower_limit_results.AppendSpeedLimit(20.0, 8.0);
  matlab_lower_limit_results.AppendSpeedLimit(29.0, 5.0);
  matlab_lower_limit_results.AppendSpeedLimit(30.0, 5.0);
  matlab_lower_limit_results.AppendSpeedLimit(40.0, 5.0);
  matlab_lower_limit_results.AppendSpeedLimit(50.0, 5.0);
  EXPECT_DOUBLE_EQ(speed_limit.speed_limit_points().size(),
                   matlab_lower_limit_results.speed_limit_points().size());
  for (size_t i = 0; i < matlab_lower_limit_results.speed_limit_points().size();
       ++i) {
    EXPECT_DOUBLE_EQ(
        speed_limit.GetSpeedLimitByS(
            matlab_lower_limit_results.speed_limit_points().at(i).first),
        matlab_lower_limit_results.speed_limit_points().at(i).second);
  }
}

TEST_F(LongitudinalReferenceTest, TestDesignSpeedLimitCurveForObstacles) {
  Eigen::VectorXd x_init(3);
  x_init(0) = 0.0;
  x_init(1) = 5.0;
  SpeedLimit speed_limit_map_smooth;
  speed_limit_map_smooth.AppendSpeedLimit(-2.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(10.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(20.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(30.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(40.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(50.0, 8.0);
  std::vector<std::vector<std::pair<double, double>>> sv_follow_pairs;
  Blocker blocker_1(5);
  Blocker blocker_2(5);
  std::vector<Blocker> blockers;
  blocker_1.is_filtered = false;
  blocker_1.is_front = true;
  blocker_1.s[1] = 20.0;
  blocker_1.s[2] = 25.0;
  blocker_1.s[3] = 30.0;
  blocker_1.v[1] = 5.0;
  blocker_1.v[2] = 5.0;
  blocker_1.v[3] = 5.0;
  blocker_2.is_filtered = false;
  blocker_2.is_front = true;
  blocker_2.v[3] = 5.0;
  blocker_2.v[4] = 5.0;
  blocker_2.s[3] = 10.0;
  blocker_2.s[4] = 15.0;
  Obstacle obstacle1, obstacle2;
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_theta(0.0);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_id(1);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  obstacle1 = Obstacle("1_0", perception_obstacle);
  perception_obstacle.set_id(2);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_sub_type(perception::SubType::ST_UNKNOWN);
  obstacle2 = Obstacle("2_0", perception_obstacle);
  blocker_1.obs = &obstacle1;
  blocker_2.obs = &obstacle2;
  blockers.emplace_back(blocker_1);
  blockers.emplace_back(blocker_2);

  LongitudinalPadding lon_padding;
  std::vector<SpeedLimit> speed_limit_set =
      lon_ref_->DesignSpeedLimitCurveForObstacles(
          x_init, speed_limit_map_smooth, blockers, sv_follow_pairs,
          TimeGapLevel::LEVEL_3, *lon_lookup_tables_, lon_padding);

  SpeedLimit matlab_limit_results;
  matlab_limit_results.AppendSpeedLimit(-2.0, 5.385);
  matlab_limit_results.AppendSpeedLimit(0.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(4.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(10.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(20.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(30.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(40.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(50.0, 5.0);
  std::vector<SpeedLimit> matlab_results_speed_limit_set;
  matlab_results_speed_limit_set.emplace_back(matlab_limit_results);
  matlab_results_speed_limit_set.emplace_back(matlab_limit_results);
  matlab_results_speed_limit_set.emplace_back(matlab_limit_results);
  matlab_results_speed_limit_set.emplace_back(matlab_limit_results);
  matlab_limit_results.Clear();
  matlab_limit_results.AppendSpeedLimit(-2.0, 6.083);
  matlab_limit_results.AppendSpeedLimit(4.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(10.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(20.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(30.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(40.0, 5.0);
  matlab_limit_results.AppendSpeedLimit(50.0, 5.0);
  matlab_results_speed_limit_set.emplace_back(matlab_limit_results);
  EXPECT_EQ(speed_limit_set.size(), matlab_results_speed_limit_set.size());
  for (size_t i = 0; i < speed_limit_set.size(); ++i) {
    for (size_t k = 0; k < speed_limit_set.at(i).speed_limit_points().size();
         ++k) {
      EXPECT_NEAR(
          speed_limit_set[i].speed_limit_points().at(k).first,
          matlab_results_speed_limit_set[i].speed_limit_points().at(k).first,
          kTol);
    }
  }
}

TEST_F(LongitudinalReferenceTest, TestForwardSimulateTrajectory) {
  Eigen::VectorXd x_init(3);
  x_init(0) = 0.0;
  x_init(1) = 5.0;
  SpeedLimit speed_limit_map_smooth;
  speed_limit_map_smooth.AppendSpeedLimit(-2.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(10.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(20.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(30.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(40.0, 8.0);
  speed_limit_map_smooth.AppendSpeedLimit(50.0, 8.0);
  std::vector<std::vector<std::pair<double, double>>> sv_follow_pairs;
  Blocker blocker_1(5);
  Blocker blocker_2(5);
  std::vector<Blocker> blockers;
  blocker_1.is_filtered = true;
  blocker_1.is_front = true;
  blocker_1.s[1] = 20.0;
  blocker_1.s[2] = 25.0;
  blocker_1.s[3] = 30.0;
  blocker_1.v[1] = 5.0;
  blocker_1.v[2] = 5.0;
  blocker_1.v[3] = 5.0;
  blocker_2.is_filtered = true;
  blocker_2.is_front = true;
  blocker_2.v[3] = 5.0;
  blocker_2.v[4] = 5.0;
  blocker_2.s[3] = 10.0;
  blocker_2.s[4] = 15.0;
  Obstacle obstacle1, obstacle2;
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_theta(0.0);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_id(1);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  obstacle1 = Obstacle("1_0", perception_obstacle);
  perception_obstacle.set_id(2);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_sub_type(perception::SubType::ST_UNKNOWN);
  obstacle2 = Obstacle("2_0", perception_obstacle);
  blocker_1.obs = &obstacle1;
  blocker_2.obs = &obstacle2;
  blockers.emplace_back(blocker_1);
  blockers.emplace_back(blocker_2);

  LongitudinalPadding lon_padding;
  std::vector<SpeedLimit> speed_limit_set =
      lon_ref_->DesignSpeedLimitCurveForObstacles(
          x_init, speed_limit_map_smooth, blockers, sv_follow_pairs,
          TimeGapLevel::LEVEL_3, *lon_lookup_tables_, lon_padding);
  sv_follow_pairs.at(0).clear();
  sv_follow_pairs.at(1).clear();
  sv_follow_pairs.at(2).clear();
  sv_follow_pairs.at(3).clear();
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  std::tie(x_ref, u_ref) = lon_ref_->ForwardSimulateTrajectory(
      x_init, speed_limit_set, sv_follow_pairs,
      lon_lookup_tables_->a_accel_table(),
      lon_lookup_tables_->a_min_soft_table());
  Eigen::MatrixXd matlab_results_x_ref(2, 5);
  Eigen::MatrixXd matlab_results_u_ref(1, 4);
  matlab_results_x_ref << 0, 5.8246, 13.1103, 21.0714, 29.0714, 5, 6.6493,
      7.9221, 8.0, 8.0;
  matlab_results_u_ref << 1.6493, 1.2729, 0.0778, 0;
  for (int i = 0; i < x_ref.rows(); ++i) {
    for (int j = 0; j < x_ref.cols(); ++j) {
      EXPECT_NEAR(x_ref(i, j), matlab_results_x_ref(i, j), kTol);
    }
  }

  for (int i = 0; i < u_ref.rows(); ++i) {
    for (int j = 0; j < u_ref.cols(); ++j) {
      EXPECT_NEAR(u_ref(i, j), matlab_results_u_ref(i, j), kTol);
    }
  }
}

TEST_F(LongitudinalReferenceTest, TestForwardSimulateTrajectoryV2) {
  // scenario_full_stop
  Eigen::VectorXd x_init(3);
  x_init(0) = 0.0;
  x_init(1) = 5.0;
  SpeedLimit speed_limit_map_smooth;
  speed_limit_map_smooth.AppendSpeedLimit(-2.0, 15.0);
  speed_limit_map_smooth.AppendSpeedLimit(10.0, 15.0);
  speed_limit_map_smooth.AppendSpeedLimit(20.0, 15.0);
  speed_limit_map_smooth.AppendSpeedLimit(30.0, 15.0);
  speed_limit_map_smooth.AppendSpeedLimit(40.0, 15.0);
  speed_limit_map_smooth.AppendSpeedLimit(50.0, 15.0);
  speed_limit_map_smooth.AppendSpeedLimit(200.0, 15.0);
  Blocker blocker_1(8);
  std::vector<Blocker> blockers;
  blocker_1.is_filtered = false;
  blocker_1.is_front = true;
  blocker_1.s[0] = 20.0;
  blocker_1.s[1] = 20.0;
  blocker_1.s[2] = 20.0;
  blocker_1.s[3] = 20.0;
  blocker_1.s[4] = 20.0;
  blocker_1.s[5] = 20.0;
  blocker_1.s[6] = 20.0;
  blocker_1.s[7] = 20.0;
  blocker_1.v[0] = 0.0;
  blocker_1.v[1] = 0.0;
  blocker_1.v[2] = 0.0;
  blocker_1.v[3] = 0.0;
  blocker_1.v[4] = 0.0;
  blocker_1.v[5] = 0.0;
  blocker_1.v[6] = 0.0;
  blocker_1.v[7] = 0.0;
  blocker_1.k_start = 0;
  blocker_1.k_end = 7;
  Obstacle obstacle1;
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_theta(0.0);
  perception_obstacle.set_length(10.0);
  perception_obstacle.set_width(2.0);
  perception_obstacle.set_id(1);
  perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
  obstacle1 = Obstacle("1_0", perception_obstacle);
  blocker_1.obs = &obstacle1;
  blockers.emplace_back(blocker_1);

  LongitudinalPadding lon_padding;

  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  std::tie(x_ref, u_ref) = lon_ref_->ForwardSimulateTrajectoryV2(
      x_init, speed_limit_map_smooth, blockers,
      lon_lookup_tables_->a_accel_table(),
      lon_lookup_tables_->a_min_soft_table(), TimeGapLevel::LEVEL_3,
      *lon_lookup_tables_, lon_padding);
  Eigen::MatrixXd matlab_results_x_ref(2, 5);
  Eigen::MatrixXd matlab_results_u_ref(1, 4);
  matlab_results_x_ref << 0, 4.764, 8.7928, 11.822, 13.852, 5, 4.528, 3.5296,
      2.5287, 1.5296;
  matlab_results_u_ref << -0.4720, -0.9984, -1.0009, -1.0036;
  for (int i = 0; i < x_ref.rows(); ++i) {
    for (int j = 0; j < x_ref.cols(); ++j) {
      EXPECT_NEAR(x_ref(i, j), matlab_results_x_ref(i, j), kTol);
    }
  }

  for (int i = 0; i < u_ref.rows(); ++i) {
    for (int j = 0; j < u_ref.cols(); ++j) {
      EXPECT_NEAR(u_ref(i, j), matlab_results_u_ref(i, j), kTol);
    }
  }

  // scenario_follow_front_1
  x_init(0) = 0.0;
  x_init(1) = 23.0;
  speed_limit_map_smooth.Clear();
  speed_limit_map_smooth.AppendSpeedLimit(-2.0, 40.0);
  speed_limit_map_smooth.AppendSpeedLimit(10.0, 40.0);
  speed_limit_map_smooth.AppendSpeedLimit(20.0, 40.0);
  speed_limit_map_smooth.AppendSpeedLimit(30.0, 40.0);
  speed_limit_map_smooth.AppendSpeedLimit(40.0, 40.0);
  speed_limit_map_smooth.AppendSpeedLimit(50.0, 40.0);
  speed_limit_map_smooth.AppendSpeedLimit(200.0, 40.0);
  Blocker blocker_2(8);
  blockers.clear();
  blocker_2.is_filtered = false;
  blocker_2.is_front = true;
  blocker_2.s[0] = 30.0;
  blocker_2.s[1] = 50.0;
  blocker_2.s[2] = 70.0;
  blocker_2.s[3] = 90.0;
  blocker_2.s[4] = 110.0;
  blocker_2.s[5] = 130.0;
  blocker_2.s[6] = 150.0;
  blocker_2.s[7] = 170.0;
  blocker_2.v[0] = 20.0;
  blocker_2.v[1] = 20.0;
  blocker_2.v[2] = 20.0;
  blocker_2.v[3] = 20.0;
  blocker_2.v[4] = 20.0;
  blocker_2.v[5] = 20.0;
  blocker_2.v[6] = 20.0;
  blocker_2.v[7] = 20.0;
  blocker_2.k_start = 0;
  blocker_2.k_end = 7;
  Blocker blocker_3(4);
  blocker_3.s[0] = 50.0;
  blocker_3.s[1] = 72.0;
  blocker_3.s[2] = 94.0;
  blocker_3.s[3] = 116.0;
  blocker_3.v[0] = 22.0;
  blocker_3.v[1] = 22.0;
  blocker_3.v[2] = 22.0;
  blocker_3.v[3] = 22.0;
  blocker_3.k_start = 0;
  blocker_3.k_end = 3;

  Obstacle obstacle2;
  perception::PerceptionObstacle perception_obstacle_2;
  perception_obstacle_2.set_theta(0.0);
  perception_obstacle_2.set_length(10.0);
  perception_obstacle_2.set_width(2.0);
  perception_obstacle_2.set_id(1);
  perception_obstacle_2.set_sub_type(perception::SubType::ST_CAR);
  obstacle2 = Obstacle("2_0", perception_obstacle);
  blocker_2.obs = &obstacle2;
  blocker_3.obs = &obstacle2;
  blockers.emplace_back(blocker_2);
  blockers.emplace_back(blocker_3);
  std::tie(x_ref, u_ref) = lon_ref_->ForwardSimulateTrajectoryV2(
      x_init, speed_limit_map_smooth, blockers,
      lon_lookup_tables_->a_accel_table(),
      lon_lookup_tables_->a_min_soft_table(), TimeGapLevel::LEVEL_3,
      *lon_lookup_tables_, lon_padding);
  Eigen::MatrixXd matlab_results_x_ref_2(2, 5);
  Eigen::MatrixXd matlab_results_u_ref_2(1, 4);
  matlab_results_x_ref_2 << 0, 22.5, 44.0, 64.5, 84.368, 23, 22.0, 21.0, 20.0,
      19.737;
  matlab_results_u_ref_2 << -1.0, -1.0, -1.0, -0.263;

  for (int i = 0; i < x_ref.rows(); ++i) {
    for (int j = 0; j < x_ref.cols(); ++j) {
      EXPECT_NEAR(x_ref(i, j), matlab_results_x_ref_2(i, j), kTol);
    }
  }

  for (int i = 0; i < u_ref.rows(); ++i) {
    for (int j = 0; j < u_ref.cols(); ++j) {
      EXPECT_NEAR(u_ref(i, j), matlab_results_u_ref_2(i, j), kTol);
    }
  }
}

TEST_F(LongitudinalReferenceTest, TestComputeMaxDecelForRefTraj) {
  std::vector<std::vector<std::pair<double, double>>> sv_follow_pairs(26);
  sv_follow_pairs[15].emplace_back(std::make_pair(19.0, 5.0));
  sv_follow_pairs[16].emplace_back(std::make_pair(20.0, 5.0));
  sv_follow_pairs[17].emplace_back(std::make_pair(21.0, 5.0));
  sv_follow_pairs[18].emplace_back(std::make_pair(22.0, 5.0));
  sv_follow_pairs[19].emplace_back(std::make_pair(23.0, 5.0));
  sv_follow_pairs[20].emplace_back(std::make_pair(24.0, 5.0));
  sv_follow_pairs[21].emplace_back(std::make_pair(25.0, 5.0));
  sv_follow_pairs[22].emplace_back(std::make_pair(26.0, 5.0));
  sv_follow_pairs[23].emplace_back(std::make_pair(27.0, 5.0));
  sv_follow_pairs[24].emplace_back(std::make_pair(28.0, 5.0));
  sv_follow_pairs[25].emplace_back(std::make_pair(29.0, 5.0));

  Eigen::VectorXd x_init(3);
  x_init(0) = 0.0;
  x_init(1) = 10.0;
  const double matlab_a_ref_min = -1.973684210526;
  const double a_ref_min =
      lon_ref_->ComputeMaxDecelForRefTraj(x_init, sv_follow_pairs);
  EXPECT_NEAR(a_ref_min, matlab_a_ref_min, kTol);
}

TEST_F(LongitudinalReferenceTest, TestSmoothSpeedLimitCurve) {
  // forward
  LookupTable a_accel_table = lon_lookup_tables_->a_accel_table();
  const double a_decel = -1;
  SpeedLimit forawrd_pass_speed_limit;
  forawrd_pass_speed_limit.AppendSpeedLimit(0.0, 1.0);
  forawrd_pass_speed_limit.AppendSpeedLimit(5.0, 7.0);
  forawrd_pass_speed_limit.AppendSpeedLimit(10.0, 7.0);
  forawrd_pass_speed_limit.AppendSpeedLimit(15.0, 7.0);
  forawrd_pass_speed_limit.AppendSpeedLimit(20.0, 7.0);
  forawrd_pass_speed_limit.AppendSpeedLimit(25.0, 7.0);
  std::vector<double> forawrd_matlab_results{1.0,   3.937, 5.601,
                                             6.878, 7.0,   7.0};

  SpeedLimit forward_smooth_speed_limit_curve = lon_ref_->SmoothSpeedLimitCurve(
      forawrd_pass_speed_limit, a_accel_table, a_decel);
  for (size_t i = 0; i < forawrd_matlab_results.size(); ++i) {
    EXPECT_NEAR(
        forward_smooth_speed_limit_curve.speed_limit_points().at(i).second,
        forawrd_matlab_results[i], kTol);
  }

  // backward
  SpeedLimit backward_pass_speed_limit;
  backward_pass_speed_limit.AppendSpeedLimit(0.0, 7.0);
  backward_pass_speed_limit.AppendSpeedLimit(5.0, 7.0);
  backward_pass_speed_limit.AppendSpeedLimit(10.0, 7.0);
  backward_pass_speed_limit.AppendSpeedLimit(15.0, 7.0);
  backward_pass_speed_limit.AppendSpeedLimit(20.0, 7.0);
  backward_pass_speed_limit.AppendSpeedLimit(25.0, 1.0);
  SpeedLimit backward_smooth_speed_limit_curve =
      lon_ref_->SmoothSpeedLimitCurve(backward_pass_speed_limit, a_accel_table,
                                      a_decel);
  std::vector<double> backawrd_matlab_results{7.0,   6.403, 5.568,
                                              4.583, 3.317, 1};
  for (size_t i = 0; i < forawrd_matlab_results.size(); ++i) {
    EXPECT_NEAR(
        backward_smooth_speed_limit_curve.speed_limit_points().at(i).second,
        backawrd_matlab_results[i], kTol);
  }
}

}  // namespace planning
}  // namespace zark
