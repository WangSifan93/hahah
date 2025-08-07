#include "apps/planning/src/decision/lateral/frenet_trajectory_sampler.h"

#include <fstream>
#include <iostream>
#include <string>

#include "gtest/gtest.h"

namespace zark {
namespace planning {

using namespace zark::planning;

class FrenetTrajectorySamplerTest : public ::testing::Test {
 public:
  FrenetTrajectorySamplerTest() {
    LateralDeciderConfig::FrenetTrajectorySampler config_;
    config_.cost_lon = LateralDeciderConfig::FrenetTrajectorySampler::Cost{
        1.0, 1.0, 10.0, 0.0, 0.0};
    config_.cost_lat = LateralDeciderConfig::FrenetTrajectorySampler::Cost{
        1.0, 1.0, 10.0, 100.0, 10.0};
    config_.dT = 0.1;
    config_.t_total = 10.0;
    traj_sampler_.reset(new FrenetTrajectorySampler(config_));
  }

 protected:
  LateralDeciderConfig::FrenetTrajectorySampler config_;
  std::unique_ptr<FrenetTrajectorySampler> traj_sampler_;
};

void UpdateFrenetPoint(QuarticPolynomialCurve1d lon_traj,
                       QuinticPolynomialCurve1d lat_traj, const double t,
                       ::common::FrenetPoint& init_pt) {
  init_pt.s[0] = lon_traj.Evaluate(0, t);
  init_pt.s[1] = lon_traj.Evaluate(1, t);
  init_pt.s[2] = lon_traj.Evaluate(2, t);

  init_pt.l[0] = lat_traj.Evaluate(0, t);
  init_pt.l[1] = lat_traj.Evaluate(1, t);
  init_pt.l[2] = lat_traj.Evaluate(2, t);
}

void ComputeQuarticCoefficients(double s0, double s0_dot, double s0_dot_dot,
                                double s1_dot, double s1_dot_dot, double T,std::vector<double>& coeff) {
  double c_1 = s0;
  double c_2 = s0_dot;
  double c_3 = s0_dot_dot / 2;
  double c_4 =
      -(3 * s0_dot - 3 * s1_dot + 2 * T * s0_dot_dot + T * s1_dot_dot) /
      (3 * T * T);
  double c_5 = (2 * s0_dot - 2 * s1_dot + T * s0_dot_dot + T * s1_dot_dot) /
               (4 * T * T * T);
  coeff = {c_1, c_2, c_3, c_4, c_5};
}

TEST_F(FrenetTrajectorySamplerTest, TestComputeOptimalTrajectory) {
  static constexpr int kNumStates = 3;
  std::array<double, kNumStates> s_init{{0.0, 5.0, 0.0}};
  std::array<double, kNumStates> l_init{{0.0, 0.0, 0.0}};
  std::array<double, kNumStates> s_target{{200.0, 10.0, 0.0}};
  std::array<double, kNumStates> l_target{{0.5, 0.0, 0.0}};

  auto const traj_points = traj_sampler_->ComputeOptimalTrajectory(
      s_init, l_init, s_target, l_target);
  EXPECT_NEAR(traj_points.front().s[1], s_init[1], 1e-2);
  EXPECT_NEAR(traj_points.front().l[0], l_init[0], 1e-2);
  EXPECT_NEAR(traj_points.back().s[1], s_target[1], 1e-2);
  EXPECT_NEAR(traj_points.back().l[0], l_target[0], 1e-2);
}

TEST_F(FrenetTrajectorySamplerTest, TestEvaluateTrajectoryCost) {
  static constexpr int kNumStates = 3;
  std::array<double, kNumStates> s_init{{0.0, 5.0, 0.0}};
  std::array<double, kNumStates> l_init{{0.0, 0.0, 0.0}};
  std::array<double, kNumStates> s_target{{200.0, 10.0, 0.0}};
  std::array<double, kNumStates> l_target{{0.5, 0.0, 0.0}};

  QuarticPolynomialCurve1d c_s(s_init, {s_target[1], s_target[2]},
                               config_.t_total);
  QuinticPolynomialCurve1d c_l(l_init, l_target, config_.t_total);
  EXPECT_NEAR(traj_sampler_->EvaluateTrajectoryCost(c_s, c_l), 253.1095, 1e-2);
}

TEST_F(FrenetTrajectorySamplerTest, TestCalcTrajCost) {
  static constexpr int kNumStates = 3;
  std::array<double, kNumStates> s_init{{0.0, 5.0, 0.0}};
  std::array<double, kNumStates> s_target{{200.0, 10.0, 0.0}};
  QuarticPolynomialCurve1d c_s(s_init, {s_target[1], s_target[2]},
                               config_.t_total);
  EXPECT_NEAR(traj_sampler_->CalcTrajCost(c_s, config_.cost_lon), 13.09, 1e-2);

  std::array<double, kNumStates> l_init{{0.0, 0.0, 0.0}};
  std::array<double, kNumStates> l_target{{0.5, 0.0, 0.0}};
  QuinticPolynomialCurve1d c_l(l_init, l_target, config_.t_total);
  EXPECT_NEAR(traj_sampler_->CalcTrajCost(c_l, config_.cost_lat), 10.02, 1e-2);
}

TEST_F(FrenetTrajectorySamplerTest, TestIsTrajectoryFeasible) {
  static constexpr int kNumStates = 3;
  std::array<double, kNumStates> s_init{{0.0, 5.0, 0.0}};
  std::array<double, kNumStates> s_target{{200.0, 10.0, 0.0}};
  QuarticPolynomialCurve1d c_s(s_init, {s_target[1], s_target[2]},
                               config_.t_total);
  EXPECT_TRUE(traj_sampler_->IsTrajectoryFeasible(c_s));

  s_target.at(1) = -1.0;
  c_s = QuarticPolynomialCurve1d(s_init, {s_target[1], s_target[2]},
                                 config_.t_total);
  EXPECT_TRUE(!traj_sampler_->IsTrajectoryFeasible(c_s));
}

TEST_F(FrenetTrajectorySamplerTest, TestSampleTrajectoryCostConvergence) {
  const double dt = 0.1;
  ::common::FrenetPoint init_pt, ego;
  QuarticPolynomialCurve1d lon_traj;
  QuinticPolynomialCurve1d lat_traj;

  ego.s[0] = 0.0;
  ego.s[1] = 5.0;
  ego.s[2] = 0.1;

  ego.l[0] = 1.0;
  ego.l[1] = 0.1;
  ego.l[2] = 0.1;

  init_pt = ego;
  const double v_target = 10.0;
  const double l_target = 0.0;
  const int planning_sim_start_frame = 1;
  const int planning_sim_frames = 50;
  double opt_res = std::numeric_limits<double>::infinity();
  std::vector<double> lon_coeff;
  for (int k = planning_sim_start_frame; k <= planning_sim_frames; k++) {
    if (k == planning_sim_start_frame) {
      init_pt = ego;
    } else {
      UpdateFrenetPoint(lon_traj, lat_traj, dt, init_pt);
    }
    double cost = std::numeric_limits<double>::infinity();
    double opt_t = std::numeric_limits<double>::infinity();
    config_.dT = 0.1;
    config_.t_total = 5.0;

    for (double T{config_.dT}; T < config_.t_total; T += config_.dT) {
      QuarticPolynomialCurve1d c_s(init_pt.s, {v_target, 0.0}, T);
      QuinticPolynomialCurve1d c_l(init_pt.l, {l_target, 0.0, 0.0}, T);
      double cost_temp = traj_sampler_->EvaluateTrajectoryCost(c_s, c_l);
      if (cost_temp < cost) {
        cost = cost_temp;
        opt_t = T;
        lon_traj = c_s;
        lat_traj = c_l;
      }
    }
    ComputeQuarticCoefficients(init_pt.s[0], init_pt.s[1], init_pt.s[2],
                               v_target, 0.0, opt_t, lon_coeff);
    if(k == planning_sim_frames)
    opt_res = opt_t;
  }
  EXPECT_EQ(opt_res, 0.1);
}

}  // namespace planning
}  // namespace zark
