/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file frenet_trajectory_sampler.cc
 **/

#include "frenet_trajectory_sampler.h"

#include <limits>
#include <cmath>
#include "apps/planning/src/common/log.h"

namespace zark {
namespace planning {

FrenetTrajectorySampler::FrenetTrajectorySampler(
    LateralDeciderConfig::FrenetTrajectorySampler config)
    : config_(config) {}

std::vector<::common::FrenetPoint>
FrenetTrajectorySampler::ComputeOptimalTrajectory(
    const std::array<double, kNumStates> s_init,
    const std::array<double, kNumStates> l_init,
    const std::array<double, kNumStates> s_target,
    const std::array<double, kNumStates> l_target) {
  double cost = std::numeric_limits<double>::infinity();
  QuarticPolynomialCurve1d lon_traj;
  QuinticPolynomialCurve1d lat_traj;
  // T: terminal trajectory time [s]
  for (double T{config_.dT}; T < config_.t_total; T += config_.dT) {
    QuarticPolynomialCurve1d c_s(s_init, {s_target[1], s_target[2]}, T);
    if (!IsTrajectoryFeasible(c_s)) {
      continue;
    }
    QuinticPolynomialCurve1d c_l(l_init, l_target, T);
    double cost_temp = EvaluateTrajectoryCost(c_s, c_l);
    if (cost_temp < cost) {
      cost = cost_temp;
      lon_traj = c_s;
      lat_traj = c_l;
    }
  }
  std::vector<::common::FrenetPoint> res_traj;
  const double T = lon_traj.ParamLength();
  for (double t{0.0}; t <= config_.t_end; t += config_.dt) {
    ::common::FrenetPoint point;
    if (t <= T) {
      point.s[kIdxS] = lon_traj.Evaluate(kIdxS, t);
      point.s[kIdxV] = lon_traj.Evaluate(kIdxV, t);
      point.s[kIdxA] = lon_traj.Evaluate(kIdxA, t);
      point.l[kIdxS] = lat_traj.Evaluate(kIdxS, t);
      point.l[kIdxV] = lat_traj.Evaluate(kIdxV, t);
      point.l[kIdxA] = lat_traj.Evaluate(kIdxA, t);
    } else {
      point.s[kIdxS] = res_traj.back().s[kIdxS] + s_target[kIdxV] * config_.dt;
      point.s[kIdxV] = s_target[kIdxV];
      point.s[kIdxA] = 0.0;
      point.l[kIdxS] = l_target[kIdxS];
      point.l[kIdxV] = 0.0;
      point.l[kIdxA] = 0.0;
    }
    point.t = t;
    res_traj.emplace_back(point);
  }
  return res_traj;
}

double FrenetTrajectorySampler::EvaluateTrajectoryCost(
    const Curve1d& lon_trajectory, const Curve1d& lat_trajectory) {
  return CalcTrajCost(lon_trajectory, config_.cost_lon) * config_.cost_lon.k +
         CalcTrajCost(lat_trajectory, config_.cost_lat) * config_.cost_lat.k;
}

double FrenetTrajectorySampler::EvaluateTrajectoryCost(
    const PolynomialCurve1d& lon_trajectory,
    const PolynomialCurve1d& lat_trajectory) {
  return CalcTrajCost(lon_trajectory, config_.cost_lon) * config_.cost_lon.k +
         (CalcTrajCost(lat_trajectory, config_.cost_lat) +
          CalcMaxTrajOvershoot(lat_trajectory, config_.cost_lat)) *
             config_.cost_lat.k;
};

double FrenetTrajectorySampler::CalcTrajCost(
    const Curve1d& trajectory1d,
    LateralDeciderConfig::FrenetTrajectorySampler::Cost cost_parameter) {
  double jerk_sqr_sum{0.0};
  const double T = trajectory1d.ParamLength();
  for (double t{0.0}; t < T; t += config_.dt) {
    const double jerk = trajectory1d.Evaluate(kIdxJ, t);
    jerk_sqr_sum += jerk * jerk;
  }
  return cost_parameter.k_j * jerk_sqr_sum + cost_parameter.k_T * T;
}

double FrenetTrajectorySampler::CalcMaxTrajOvershoot(
    const PolynomialCurve1d& trajectory1d,
    const LateralDeciderConfig::FrenetTrajectorySampler::Cost& config_cost) {
  const int kQuinticOrder = 5;
  const double kMathEpsilon = 1e-10;
  const double kMaxDeviationAllowed = 0.1;
  const double c3 = trajectory1d.Coef(3);
  const double c4 = trajectory1d.Coef(4);
  const double c5 = (trajectory1d.Order() == kQuinticOrder) ? trajectory1d.Coef(5) : 0.0;
  const double length = trajectory1d.ParamLength();
  // dl(t) = (t - t_end)^2 * (A*t^2 + B*t + C)
  const double A = 5.0 * c5;
  const double B = 4.0 * c4 + 10.0 * c5 * length;
  const double C = 3.0 * c3 + 8.0 * c4 * length + 15.0 * c5 * length * length;
  const double delta = B * B - 4.0 * A * C;

  double root1, root2;
  if (std::abs(A) > kMathEpsilon && delta > kMathEpsilon) {
    root1 = (-B + sqrt(delta)) / (2.0 * A);
    root2 = (-B - sqrt(delta)) / (2.0 * A);
    root1 = std::min(std::max(0.0, root1), length);
    root2 = std::min(std::max(0.0, root2), length);
  } else if (std::abs(A) < kMathEpsilon && std::abs(B) > kMathEpsilon) {
    root1 = -C / B;
    root1 = std::min(std::max(0.0, root1), length);
    root2 = root1;
  } else {
    root1 = length;
    root2 = root1;
  }
  const double y1 = trajectory1d.Evaluate(0, root1);
  const double y2 = trajectory1d.Evaluate(0, root2);
  const bool sign1 = y1 * trajectory1d.Coef(0) > kMathEpsilon;
  const bool sign2 = y2 * trajectory1d.Coef(0) > kMathEpsilon;

  double max_overshoot, max_deviation;
  if (!sign1 && sign2) {
    max_overshoot = std::abs(y1);
    max_deviation = std::abs(y2);
  } else if (sign1 && !sign2) {
    max_overshoot = std::abs(y2);
    max_deviation = std::abs(y1);
  } else if (!sign1 && !sign2) {
    max_overshoot = std::max(std::abs(y1), std::abs(y2));
    max_deviation = 0.0;
  } else {
    max_overshoot = 0.0;
    max_deviation = std::max(std::abs(y1), std::abs(y2));
  }
  return max_deviation > kMaxDeviationAllowed
             ? (max_overshoot * config_cost.k_overshoot +
                max_deviation * config_cost.k_deviation)
             : max_overshoot * config_cost.k_overshoot;
};

bool FrenetTrajectorySampler::IsTrajectoryFeasible(
    const Curve1d& trajectory1d) {
  const double T = trajectory1d.ParamLength();
  for (double t{0.0}; t < T; t += config_.dt) {
    if (trajectory1d.Evaluate(kIdxV, t) < 0.0) {
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace zark
