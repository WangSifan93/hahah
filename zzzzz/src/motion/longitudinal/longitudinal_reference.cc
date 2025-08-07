/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_reference.cc
 **/

#include "apps/planning/src/motion/longitudinal/longitudinal_reference.h"

#include <algorithm>

#include "linear_interpolation.h"

namespace zark {
namespace planning {

LongitudinalReference::LongitudinalReference(
    const LongitudinalOptimizerConfig& config) {
  config_ = config;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
LongitudinalReference::DesignReferenceTrajectory(
    const Eigen::VectorXd& x_init, const SpeedLimit& speed_limit,
    const std::vector<Blocker>& blockers, const TimeGapLevel& time_gap_level,
    const LongitudinalLookupTables& lon_lookup_tables,
    const LongitudinalPadding& lon_padding,
    std::vector<SpeedLimit>& speed_limit_set) {
  SpeedLimit speed_limit_map_smooth =
      SmoothSpeedLimitCurve(speed_limit, lon_lookup_tables.a_accel_table(),
                            config_.ref.a_decel_speed_limit);

  if (!PlanningGflags::enable_lon_ref_v2) {
    std::vector<std::vector<std::pair<double, double>>> sv_follow_pairs;
    speed_limit_set = DesignSpeedLimitCurveForObstacles(
        x_init, speed_limit_map_smooth, blockers, sv_follow_pairs,
        time_gap_level, lon_lookup_tables, lon_padding);

    return ForwardSimulateTrajectory(x_init, speed_limit_set, sv_follow_pairs,
                                     config_.ref.a_accel_table,
                                     config_.con.a_min_soft_table);
  } else {
    speed_limit_set.emplace_back(speed_limit_map_smooth);
    return ForwardSimulateTrajectoryV2(
        x_init, speed_limit_map_smooth, blockers, config_.ref.a_accel_table,
        config_.con.a_min_soft_table, time_gap_level, lon_lookup_tables,
        lon_padding);
  }
}

std::vector<SpeedLimit>
LongitudinalReference::DesignSpeedLimitCurveForObstacles(
    const Eigen::VectorXd& x_init, const SpeedLimit& speed_limit_map_smooth,
    const std::vector<Blocker>& blockers,
    std::vector<std::vector<std::pair<double, double>>>& sv_follow_pairs,
    const TimeGapLevel& time_gap_level,
    const LongitudinalLookupTables& lon_lookup_tables,
    const LongitudinalPadding& lon_padding) {
  int n_nodes = config_.model.num_steps + 1;
  const double kVTol = 1.0e-4;
  const double kSFollowMin = 1.0e-5;

  std::vector<SpeedLimit> speed_limit_set(n_nodes);
  sv_follow_pairs.resize(n_nodes);
  for (int k = n_nodes - 1; k >= 0; --k) {
    speed_limit_set[k] =
        (k == n_nodes - 1) ? speed_limit_map_smooth : speed_limit_set[k + 1];
    for (const Blocker& blocker : blockers) {
      if (blocker.is_front && !blocker.is_filtered &&
          !std::isnan(blocker.s[k])) {
        const double s_max = blocker.s[k];
        const double v_follow = blocker.v[k];
        const double stiff_padding = lon_padding.GetFrontStiffPadding(
            lon_lookup_tables, blocker.obs->Perception().sub_type(),
            blocker.obs->IsVirtual(), v_follow, x_init(kIdxV));
        const double soft_padding =
            lon_padding.GetFrontSoftPadding(lon_lookup_tables, v_follow,
                                            x_init(kIdxV)) *
            lon_padding.GetTimeGapMultiplier(time_gap_level);
        const double s_follow =
            std::max(s_max - stiff_padding - soft_padding, kSFollowMin);
        const double v_follow_curr =
            speed_limit_set[k].GetSpeedLimitByS(s_follow);
        if (v_follow < v_follow_curr - kVTol) {
          sv_follow_pairs[k].emplace_back(std::make_pair(s_follow, v_follow));
          LowerSpeedLimitCurveForObstacle(s_follow, v_follow,
                                          speed_limit_set[k]);
        }
      }
    }
    if (k < n_nodes - 1 && sv_follow_pairs[k].empty()) {
      sv_follow_pairs[k] = sv_follow_pairs[k + 1];
    }
  }

  for (int k = 0; k < n_nodes; ++k) {
    speed_limit_set[k] = SmoothSpeedLimitCurve(
        speed_limit_set[k], lon_lookup_tables.a_accel_table(),
        config_.ref.a_decel_obs);
  }
  return speed_limit_set;
}

void LongitudinalReference::LowerSpeedLimitCurveForObstacle(
    const double s_follow, const double v_follow, SpeedLimit& speed_limit) {
  constexpr double kTol = 1.0e-5;
  auto func_comp = [](const std::pair<double, double>& sv_pair,
                      const double s) { return sv_pair.first < s; };
  auto mutable_speed_limit_points = speed_limit.mutable_speed_limit_points();
  auto it = std::lower_bound(mutable_speed_limit_points->begin(),
                             mutable_speed_limit_points->end(),
                             (s_follow - kTol), func_comp);
  for (auto iterator = it; iterator != mutable_speed_limit_points->end();
       ++iterator) {
    iterator->second = std::min(v_follow, iterator->second);
  }
  mutable_speed_limit_points->emplace_back(std::make_pair(s_follow, v_follow));
  std::sort(mutable_speed_limit_points->begin(),
            mutable_speed_limit_points->end(),
            [](const std::pair<double, double>& sv_pair_1,
               const std::pair<double, double>& sv_pair_2) {
              return sv_pair_1.first < sv_pair_2.first;
            });
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
LongitudinalReference::ForwardSimulateTrajectory(
    const Eigen::VectorXd& x_init, std::vector<SpeedLimit> speed_limit_set,
    const std::vector<std::vector<std::pair<double, double>>>& sv_follow_pairs,
    const LookupTable& a_accel_table, const LookupTable& a_min_soft_table) {
  const int n_steps = config_.model.num_steps;
  const int n_x = config_.model.num_states;
  const int n_u = config_.model.num_ctrls;
  const int n_nodes = config_.model.num_steps + 1;
  const double dt = config_.model.dt;
  const double a_ref_min = ComputeMaxDecelForRefTraj(x_init, sv_follow_pairs);
  const bool is_v_ego_above_speed_limit =
      x_init(kIdxV) > speed_limit_set[0].GetSpeedLimitByS(x_init(kIdxS));

  // forward simulate ego ref traj
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Zero(n_x, n_nodes);
  Eigen::MatrixXd u_ref = Eigen::MatrixXd::Zero(n_u, n_steps);
  x_ref.row(kIdxS)(0) = x_init(kIdxS);
  x_ref.row(kIdxV)(0) = x_init(kIdxV);
  for (int k = 0; k < n_steps; ++k) {
    double& s_curr = x_ref.row(kIdxS)(k);
    double& v_curr = x_ref.row(kIdxV)(k);
    double& s_next = x_ref.row(kIdxS)(k + 1);
    double& v_next = x_ref.row(kIdxV)(k + 1);
    double& a_curr = u_ref.row(kIdxA)(k);
    a_curr = a_accel_table.Evaluate(v_curr);
    v_next = v_curr + a_curr * dt;
    s_next = s_curr + v_curr * dt + 0.5 * a_curr * dt * dt;
    double v_max = speed_limit_set[k + 1].GetSpeedLimitByS(s_next);
    if (is_v_ego_above_speed_limit) {  // relax v_max when ego is initially
                                       // above speed limit curve
      a_curr = std::max(a_ref_min, a_min_soft_table.Evaluate(x_init(kIdxV)));
      v_max = std::max(v_max, v_curr + a_curr * dt);
    }
    if (v_next > v_max) {  // follow the speed limit curve
      v_next = v_max;
      a_curr =
          std::max((v_next - v_curr) / dt, a_min_soft_table.Evaluate(v_curr));
      v_next = v_curr + a_curr * dt;
      s_next = s_curr + v_curr * dt + 0.5 * a_curr * dt * dt;
    }
  }

  return std::make_pair(x_ref, u_ref);
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
LongitudinalReference::ForwardSimulateTrajectoryV2(
    const Eigen::VectorXd& x_init, const SpeedLimit& speed_limit_map_smooth,
    const std::vector<Blocker>& blockers, const LookupTable& a_accel_table,
    const LookupTable& a_min_soft_table, const TimeGapLevel& time_gap_level,
    const LongitudinalLookupTables& lon_lookup_tables,
    const LongitudinalPadding& lon_padding) {
  const int n_steps = config_.model.num_steps;
  const int n_x = config_.model.num_states;
  const int n_u = config_.model.num_ctrls;
  const int n_nodes = config_.model.num_steps + 1;
  const double dt = config_.model.dt;

  const double alpha = 0.2;
  const double kSMin = 1.0e-3;
  const double kVMin = 1.0e-3;
  const double kATol = 1.0e-3;
  const int n_iter_max = 50;

  const bool is_v_ego_above_speed_limit =
      x_init(kIdxV) > speed_limit_map_smooth.GetSpeedLimitByS(x_init(kIdxS));

  auto CustomSqrt = [](double num) -> double {
    return num >= 0 ? sqrt(num) : -sqrt(std::abs(num));
  };

  // forward simulate ego ref traj
  Eigen::MatrixXd x_ref = Eigen::MatrixXd::Zero(n_x, n_nodes);
  Eigen::MatrixXd u_ref = Eigen::MatrixXd::Zero(n_u, n_steps);
  x_ref.row(kIdxS)(0) = x_init(kIdxS);
  x_ref.row(kIdxV)(0) = x_init(kIdxV);

  for (int k = 0; k < n_steps; ++k) {
    double& s_curr = x_ref.row(kIdxS)(k);
    double& v_curr = x_ref.row(kIdxV)(k);
    double& s_next = x_ref.row(kIdxS)(k + 1);
    double& v_next = x_ref.row(kIdxV)(k + 1);
    double& a_curr = u_ref.row(kIdxA)(k);
    a_curr = a_accel_table.Evaluate(v_curr);

    for (const Blocker& blocker : blockers) {
      if (blocker.is_front && !blocker.is_filtered && k <= blocker.k_end) {
        const double s_obs =
            k < blocker.k_start ? blocker.s[blocker.k_start] : blocker.s[k];
        const double v_obs = k < blocker.k_start ? x_init(kIdxV) : blocker.v[k];

        const double v_relative_curr = v_curr - v_obs;
        const double stiff_padding = lon_padding.GetFrontStiffPadding(
            lon_lookup_tables, blocker.obs->Perception().sub_type(),
            blocker.obs->IsVirtual(), v_obs, x_init(kIdxV));
        const double soft_padding =
            lon_padding.GetFrontSoftPadding(lon_lookup_tables, v_obs,
                                            x_init(kIdxV)) *
            lon_padding.GetTimeGapMultiplier(time_gap_level);
        const double s_relative_curr =
            s_curr - (s_obs - soft_padding - stiff_padding);

        // iterative line search
        double a_curr_iter = a_curr;
        double a_next_iter = std::numeric_limits<double>::infinity();
        for (int i = 0; i < n_iter_max; i++) {
          if (v_curr < kVMin) {
            a_next_iter = 0.0;
            break;
          }

          double v_relative_next = v_relative_curr + a_curr_iter * dt;
          double s_relative_next = s_relative_curr + v_relative_curr * dt +
                                   0.5 * a_curr_iter * dt * dt;
          double a_decel_relax =
              std::fabs(s_relative_curr) < kSMin
                  ? kATol
                  : std::min(config_.ref.a_decel_obs,
                             v_relative_curr * v_relative_curr /
                                 (2 * s_relative_curr));
          double v_relative_min = std::min(
              CustomSqrt(2 * a_decel_relax * s_relative_next), v_relative_next);

          // use line search to solve for a(k)
          v_relative_next =
              (1 - alpha) * v_relative_next + alpha * v_relative_min;
          a_next_iter = std::max((v_relative_next - v_relative_curr) / dt,
                                 a_min_soft_table.Evaluate(v_relative_next));

          if (std::fabs(a_next_iter - a_curr_iter) < kATol) {
            break;
          }
          a_curr_iter = a_next_iter;
        }
        a_curr = a_next_iter;
      }
    }

    v_next = v_curr + a_curr * dt;
    s_next = s_curr + v_curr * dt + 0.5 * a_curr * dt * dt;
    double v_max =
        speed_limit_map_smooth.GetSpeedLimitByS(s_next);  // map speed limit
    if (is_v_ego_above_speed_limit) {  // relax v_max when ego is initially
                                       // above speed limit curve
      a_curr = config_.con.a_decel_speed_limit_violation;
      v_max = std::max(v_max, v_curr + a_curr * dt);
    }
    if (v_next > v_max) {  // follow the speed limit curve
      v_next = v_max;
      a_curr =
          std::max((v_next - v_curr) / dt, a_min_soft_table.Evaluate(v_curr));
      v_next = v_curr + a_curr * dt;
      s_next = s_curr + v_curr * dt + 0.5 * a_curr * dt * dt;
    }
  }

  return std::make_pair(x_ref, u_ref);
}

double LongitudinalReference::ComputeMaxDecelForRefTraj(
    const Eigen::VectorXd& x_init,
    const std::vector<std::vector<std::pair<double, double>>>&
        sv_follow_pairs) {
  const double kDMin = 1.0e-2;
  const bool is_obs_free = sv_follow_pairs[0].empty();
  double a_ref_min =
      is_obs_free
          ? config_.con.a_decel_speed_limit_violation  // use a smaller decel if
                                                       // there are NO obstacles
          : config_.ref
                .a_decel_obs;  // use the obs decel if there are obstacles
  for (int k = 0; k < static_cast<int>(sv_follow_pairs.size()); ++k) {
    for (int i = 0; i < static_cast<int>(sv_follow_pairs[k].size()); ++i) {
      const double s_follow = sv_follow_pairs[k][i].first;
      const double v_follow = sv_follow_pairs[k][i].second;
      if (s_follow < -kDMin || v_follow > x_init(kIdxV)) {
        continue;
      }
      const double d = std::max(s_follow - x_init(kIdxS), kDMin);
      a_ref_min = std::min(
          a_ref_min,
          (v_follow * v_follow - x_init(kIdxV) * x_init(kIdxV)) / (2.0 * d));
    }
  }
  return std::max(a_ref_min, config_.con.a_min_stiff);
}

SpeedLimit LongitudinalReference::SmoothSpeedLimitCurve(
    const SpeedLimit& speed_limit, const LookupTable& a_accel_table,
    const double& a_decel) {
  SpeedLimit speed_limit_smooth = speed_limit;

  const int kSizeMin = 2;
  const int size_pts = speed_limit_smooth.speed_limit_points().size();
  if (size_pts < kSizeMin) {
    return speed_limit_smooth;
  }

  // forward pass
  auto mutable_speed_limit_smooth =
      speed_limit_smooth.mutable_speed_limit_points();
  double a = 0;
  for (int i = 0; i < size_pts - 1; ++i) {
    const double s_curr = mutable_speed_limit_smooth->at(i).first;
    const double s_next = mutable_speed_limit_smooth->at(i + 1).first;
    const double v_curr = mutable_speed_limit_smooth->at(i).second;
    double& v_next = mutable_speed_limit_smooth->at(i + 1).second;
    if (v_next > v_curr) {
      a = a_accel_table.Evaluate(v_curr);
      v_next = std::min(
          v_next, std::sqrt(v_curr * v_curr + 2.0 * a * (s_next - s_curr)));
    }
  }

  // backward pass
  for (int i = size_pts - 1; i > 0; --i) {
    const double s_prev = mutable_speed_limit_smooth->at(i - 1).first;
    const double s_curr = mutable_speed_limit_smooth->at(i).first;
    const double v_curr = mutable_speed_limit_smooth->at(i).second;
    double& v_prev = mutable_speed_limit_smooth->at(i - 1).second;
    if (v_prev > v_curr) {
      v_prev = std::min(v_prev, std::sqrt(v_curr * v_curr +
                                          2.0 * a_decel * (s_prev - s_curr)));
    }
  }

  return speed_limit_smooth;
}

}  // namespace planning
}  // namespace zark
