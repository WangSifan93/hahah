/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_stop_hold.cc
 **/

#include "apps/planning/src/motion/longitudinal/longitudinal_stop_hold.h"

#include <limits>

#include "apps/planning/src/common/math/mpc/utility.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/motion/longitudinal/longitudinal_blocker.h"

namespace zark {
namespace planning {

LongitudinalStopHold::LongitudinalStopHold(
    LongitudinalOptimizerConfig& config) {
  config_ = config;
}

bool LongitudinalStopHold::IsStopHold(
    const Eigen::VectorXd& x_init, const std::vector<Blocker>& blockers,
    const TimeGapLevel& time_gap_level,
    const LongitudinalLookupTables& lon_lookup_tables,
    const LongitudinalPadding& lon_padding) {
  double s_follow_min = std::numeric_limits<double>::max();
  for (const Blocker& blocker : blockers) {
    if (blocker.obs->IsStatic() && blocker.is_front &&
        !std::isnan(blocker.s[0])) {
      const double s_max = blocker.s[0];
      const double v_follow = blocker.v[0];
      const double stiff_padding = lon_padding.GetFrontStiffPadding(
          lon_lookup_tables, blocker.obs->Perception().sub_type(),
          blocker.obs->IsVirtual(), v_follow, x_init(kIdxV));
      const double soft_padding =
          lon_padding.GetFrontSoftPadding(lon_lookup_tables, v_follow,
                                          x_init(kIdxV)) *
          lon_padding.GetTimeGapMultiplier(time_gap_level);
      const double s_follow = s_max - stiff_padding - soft_padding;
      s_follow_min = std::max(std::min(s_follow_min, s_follow), 0.0);
    }
  }

  const double s_stop = s_follow_min - x_init(kIdxS);
  const double v_ego = x_init(kIdxV);
  if ((v_ego < config_.stop_hold.v_min) && (s_stop < config_.stop_hold.s_tol)) {
    return true;
  }
  return false;
}

LonMPCData LongitudinalStopHold::GenerateStopTrajectory(
    const Eigen::VectorXd& u_prev, const double& dt_prev) {
  LonMPCData mpc_data;
  const int n_steps = config_.model.num_steps;
  const int n_x = config_.model.num_states;
  const int n_u = config_.model.num_ctrls;
  const int n_nodes = config_.model.num_steps + 1;
  const double dt = config_.model.dt;
  const int n_u_dot = config_.model.num_ctrl_rates;

  mpc_data.names.x.emplace_back("s");
  mpc_data.names.x.emplace_back("v");
  mpc_data.names.u.emplace_back("a");
  mpc_data.names.u_dot.emplace_back("j");
  mpc_data.t = util::TVector(dt, n_nodes);
  mpc_data.x_ref = Eigen::MatrixXd::Zero(n_x, n_nodes);
  mpc_data.u_ref = Eigen::MatrixXd::Zero(n_u, n_steps);
  mpc_data.x = Eigen::MatrixXd::Zero(n_x, n_nodes);
  mpc_data.u = Eigen::MatrixXd::Zero(n_u, n_steps);
  mpc_data.u_prev = u_prev;
  mpc_data.u_dot = Eigen::MatrixXd::Zero(n_u_dot, n_steps);
  mpc_data.constraints = MPCData::Constraints(n_x, n_u, n_u_dot);
  mpc_data.slacks = MPCData::Slacks(n_x, n_u, n_u_dot, n_nodes, n_steps);
  mpc_data.dt = dt;
  mpc_data.dt_prev = dt_prev;
  mpc_data.n_x = n_x;
  mpc_data.n_u = n_u;
  mpc_data.n_u_dot = n_u_dot;
  mpc_data.n_steps = n_steps;
  return mpc_data;
}

}  // namespace planning
}  // namespace zark
