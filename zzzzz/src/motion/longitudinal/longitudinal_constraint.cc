/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_constraint.cc
 **/

#include "apps/planning/src/motion/longitudinal/longitudinal_constraint.h"

namespace zark {
namespace planning {

LongitudinalConstraint::LongitudinalConstraint(
    const LongitudinalOptimizerConfig& config) {
  config_ = config;
}

MPCData::Constraints LongitudinalConstraint::DesignConstraints(
    const Eigen::VectorXd& x_init, const Eigen::MatrixXd x_ref,
    const std::vector<Blocker>& blockers, const SpeedLimit& speed_limit,
    const TimeGapLevel& time_gap_level,
    const LongitudinalLookupTables& lon_lookup_tables,
    const LongitudinalPadding& lon_padding) {
  const int n_steps = config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  const double dt = config_.model.dt;
  const double v_init = x_init(kIdxV);

  Constraints constraints;
  Constraints::Constraint& hard = constraints.hard;
  Constraints::Constraint& stiff = constraints.stiff;
  Constraints::Constraint& soft = constraints.soft;

  // s constraints
  soft.s_max = Eigen::RowVectorXd::Ones(n_nodes) * kInf;
  stiff.s_max = Eigen::RowVectorXd::Ones(n_nodes) * kInf;
  soft.s_min = Eigen::RowVectorXd::Zero(n_nodes);
  stiff.s_min = Eigen::RowVectorXd::Zero(n_nodes);
  for (const auto& blocker : blockers) {
    if (blocker.is_filtered) {
      continue;
    }
    for (int k = 0; k < n_nodes; ++k) {
      const double s_k = blocker.s[k];
      if (blocker.is_front) {
        const double stiff_padding = lon_padding.GetFrontStiffPadding(
            lon_lookup_tables, blocker.obs->Perception().sub_type(),
            blocker.obs->IsVirtual(), blocker.v[k], v_init);
        const double soft_padding =
            config_.con.constraint_padding_multiplier *
            lon_padding.GetFrontSoftPadding(lon_lookup_tables, blocker.v[k],
                                            v_init);
        stiff.s_max(k) =
            std::max(0.0, std::min(stiff.s_max(k), s_k - stiff_padding));
        soft.s_max(k) = std::max(
            0.0, std::min(soft.s_max(k), s_k - stiff_padding - soft_padding));
      } else {
        const double stiff_padding = lon_padding.GetRearStiffPadding(
            lon_lookup_tables, blocker.obs->Perception().sub_type(),
            blocker.obs->IsVirtual(), blocker.v[k], v_init);
        const double soft_padding = lon_padding.GetRearSoftPadding(
            lon_lookup_tables, blocker.v[k], v_init);
        stiff.s_min(k) = std::max(stiff.s_min(k), s_k + stiff_padding);
        soft.s_min(k) =
            std::max(soft.s_min(k), s_k + stiff_padding + soft_padding);
      }
    }
  }

  // v constraints
  stiff.v_max = Eigen::RowVectorXd::Ones(n_nodes) * kInf;
  soft.v_max = Eigen::RowVectorXd::Ones(n_nodes) * kInf;
  const double a_decel_speed_limit_violation =
      config_.con.a_decel_speed_limit_violation;
  const double v_soft_to_stiff = config_.con.v_soft_to_stiff;
  for (int k = 0; k < n_nodes; ++k) {
    soft.v_max(k) = std::max(
        0.0, std::max(speed_limit.GetSpeedLimitByS(x_ref(kIdxS, k)),
                      v_init + a_decel_speed_limit_violation * k * dt));
    stiff.v_max(k) = std::max(0.0, soft.v_max(k) + v_soft_to_stiff);
  }
  hard.v_min = Eigen::RowVectorXd::Zero(n_nodes);

  // a constraints
  stiff.a_max = Eigen::RowVectorXd::Ones(n_steps) * kInf;
  soft.a_max = Eigen::RowVectorXd::Ones(n_steps) * kInf;
  stiff.a_min = Eigen::RowVectorXd::Ones(n_steps) *
                (std::min(0.0, config_.con.a_min_stiff));
  soft.a_min = Eigen::RowVectorXd::Ones(n_steps) * kInf;
  for (int k = 0; k < n_steps; ++k) {
    stiff.a_max(k) = std::max(
        0.0, lon_lookup_tables.a_max_stiff_table().Evaluate(x_ref(kIdxV, k)));
    soft.a_max(k) = std::max(
        0.0, lon_lookup_tables.a_max_soft_table().Evaluate(x_ref(kIdxV, k)));
    soft.a_min(k) = std::min(
        0.0, lon_lookup_tables.a_min_soft_table().Evaluate(x_ref(kIdxV, k)));
  }

  // j constraints
  stiff.j_max = Eigen::RowVectorXd::Ones(n_steps) *
                (std::max(0.0, config_.con.j_max_stiff));
  soft.j_max = Eigen::RowVectorXd::Ones(n_steps) *
               (std::max(0.0, config_.con.j_max_soft));
  stiff.j_min = Eigen::RowVectorXd::Ones(n_steps) *
                (std::min(0.0, config_.con.j_min_stiff));
  soft.j_min = Eigen::RowVectorXd::Ones(n_steps) *
               (std::min(0.0, config_.con.j_min_soft));

  return ConvertToMPCConstraints(constraints);
}

MPCData::Constraints LongitudinalConstraint::ConvertToMPCConstraints(
    const Constraints& constraints) {
  MPCData::Constraints mpc_constraints;

  UpdateMPCConstraintIndices(constraints, mpc_constraints);

  UpdateMPCConstraintSizes(mpc_constraints);

  UpdateMPCConstraintWeights(config_.cost, mpc_constraints);

  UpdateMPCConstraints(constraints, mpc_constraints);

  return mpc_constraints;
}

void LongitudinalConstraint::UpdateMPCConstraintIndices(
    const Constraints& constraints, MPCData::Constraints& mpc_constraints) {
  auto UpdateConstraintIndices =
      [this](const Constraints::Constraint& constraint, const bool is_min,
             MPCData::Constraints::Constraint& mpc_constraint) {
        mpc_constraint.x.idx_enabled.resize(this->config_.model.num_states);
        mpc_constraint.u.idx_enabled.resize(this->config_.model.num_ctrls);
        mpc_constraint.u_dot.idx_enabled.resize(
            this->config_.model.num_ctrl_rates);
        if (is_min) {
          mpc_constraint.x.idx_enabled[0] = !(constraint.s_min.size() == 0);
          mpc_constraint.x.idx_enabled[1] = !(constraint.v_min.size() == 0);
          mpc_constraint.u.idx_enabled[0] = !(constraint.a_min.size() == 0);
          mpc_constraint.u_dot.idx_enabled[0] = !(constraint.j_min.size() == 0);
        } else {
          mpc_constraint.x.idx_enabled[0] = !(constraint.s_max.size() == 0);
          mpc_constraint.x.idx_enabled[1] = !(constraint.v_max.size() == 0);
          mpc_constraint.u.idx_enabled[0] = !(constraint.a_max.size() == 0);
          mpc_constraint.u_dot.idx_enabled[0] = !(constraint.j_max.size() == 0);
        }
      };

  UpdateConstraintIndices(constraints.soft, true, mpc_constraints.soft_min);
  UpdateConstraintIndices(constraints.soft, false, mpc_constraints.soft_max);
  UpdateConstraintIndices(constraints.stiff, true, mpc_constraints.stiff_min);
  UpdateConstraintIndices(constraints.stiff, false, mpc_constraints.stiff_max);
  UpdateConstraintIndices(constraints.hard, true, mpc_constraints.hard_min);
  UpdateConstraintIndices(constraints.hard, false, mpc_constraints.hard_max);
}

void LongitudinalConstraint::UpdateMPCConstraintSizes(
    MPCData::Constraints& mpc_constraints) {
  auto UpdateConstraintSizes =
      [this](MPCData::Constraints::Constraint& mpc_constraint) {
        auto UpdateConstraintInfoSizes =
            [](const int n_rows, const int n_cols,
               MPCData::Constraints::Constraint::ConstraintInfo& info) {
              info.n_enabled_row = 0;
              for (int i = 0; i < n_rows; ++i) {
                if (info.idx_enabled[i]) {
                  info.n_enabled_row++;
                }
              }
              info.n_enabled_col = n_cols;
            };

        UpdateConstraintInfoSizes(this->config_.model.num_states,
                                  this->config_.model.num_steps + 1,
                                  mpc_constraint.x);
        UpdateConstraintInfoSizes(this->config_.model.num_ctrls,
                                  this->config_.model.num_steps,
                                  mpc_constraint.u);
        UpdateConstraintInfoSizes(this->config_.model.num_ctrl_rates,
                                  this->config_.model.num_steps,
                                  mpc_constraint.u_dot);
        mpc_constraint.n_ineq = mpc_constraint.x.n_enabled_row *
                                    mpc_constraint.x.n_enabled_col +  //
                                mpc_constraint.u.n_enabled_row *
                                    mpc_constraint.u.n_enabled_col +  //
                                mpc_constraint.u_dot.n_enabled_row *
                                    mpc_constraint.u_dot.n_enabled_col;
      };

  UpdateConstraintSizes(mpc_constraints.soft_min);
  UpdateConstraintSizes(mpc_constraints.soft_max);
  UpdateConstraintSizes(mpc_constraints.stiff_min);
  UpdateConstraintSizes(mpc_constraints.stiff_max);
  UpdateConstraintSizes(mpc_constraints.hard_min);
  UpdateConstraintSizes(mpc_constraints.hard_max);
  mpc_constraints.n_ineq_hard =
      mpc_constraints.hard_min.n_ineq + mpc_constraints.hard_max.n_ineq;
  mpc_constraints.n_ineq_soft =
      mpc_constraints.stiff_min.n_ineq + mpc_constraints.stiff_max.n_ineq +
      mpc_constraints.soft_min.n_ineq + mpc_constraints.soft_max.n_ineq;
  mpc_constraints.n_ineq =
      mpc_constraints.n_ineq_hard + mpc_constraints.n_ineq_soft;
}

void LongitudinalConstraint::UpdateMPCConstraintWeights(
    const LongitudinalOptimizerConfig::Cost& cost_config,
    MPCData::Constraints& mpc_constraints) {
  auto UpdateSingleMPCConstraintWeights =
      [](const LongitudinalOptimizerConfig::Cost::Weight& weight,
         MPCData::Constraints::Constraint& mpc_constraint) {
        if (mpc_constraint.x.n_enabled_row > 0) {  // x
          mpc_constraint.x.weight.resize(mpc_constraint.x.n_enabled_row);
          int i = 0;
          if (mpc_constraint.x.idx_enabled[0]) {
            mpc_constraint.x.weight(i++) = weight.s;
          }
          if (mpc_constraint.x.idx_enabled[1]) {
            mpc_constraint.x.weight(i++) = weight.v;
          }
        }
        if (mpc_constraint.u.n_enabled_row > 0) {  // u
          mpc_constraint.u.weight.resize(mpc_constraint.u.n_enabled_row);
          mpc_constraint.u.weight(0) = weight.a;
        }
        if (mpc_constraint.u_dot.n_enabled_row > 0) {  // u_dot
          mpc_constraint.u_dot.weight.resize(
              mpc_constraint.u_dot.n_enabled_row);
          mpc_constraint.u_dot.weight(0) = weight.j;
        }
      };

  UpdateSingleMPCConstraintWeights(cost_config.soft_min,
                                   mpc_constraints.soft_min);
  UpdateSingleMPCConstraintWeights(cost_config.soft_max,
                                   mpc_constraints.soft_max);
  UpdateSingleMPCConstraintWeights(cost_config.stiff_min,
                                   mpc_constraints.stiff_min);
  UpdateSingleMPCConstraintWeights(cost_config.stiff_max,
                                   mpc_constraints.stiff_max);
}

void LongitudinalConstraint::UpdateMPCConstraints(
    const Constraints& constraints, MPCData::Constraints& mpc_constraints) {
  auto UpdateConstraints =
      [](const Constraints::Constraint& constraint, const bool is_min,
         MPCData::Constraints::Constraint& mpc_constraint) {
        if (is_min) {
          if (mpc_constraint.x.n_enabled_row > 0) {  // x
            mpc_constraint.x.constraint.resize(mpc_constraint.x.n_enabled_row,
                                               mpc_constraint.x.n_enabled_col);
            int i = 0;
            if (mpc_constraint.x.idx_enabled[0]) {
              mpc_constraint.x.constraint.row(i++) = constraint.s_min;
            }
            if (mpc_constraint.x.idx_enabled[1]) {
              mpc_constraint.x.constraint.row(i++) = constraint.v_min;
            }
          }
          if (mpc_constraint.u.idx_enabled[0]) {  // u
            mpc_constraint.u.constraint.resize(mpc_constraint.u.n_enabled_row,
                                               mpc_constraint.u.n_enabled_col);
            mpc_constraint.u.constraint.row(0) = constraint.a_min;
          }
          if (mpc_constraint.u_dot.idx_enabled[0]) {  // u_dot
            mpc_constraint.u_dot.constraint.resize(
                mpc_constraint.u_dot.n_enabled_row,
                mpc_constraint.u_dot.n_enabled_col);
            mpc_constraint.u_dot.constraint.row(0) = constraint.j_min;
          }
        } else {
          if (mpc_constraint.x.n_enabled_row > 0) {  // x
            mpc_constraint.x.constraint.resize(mpc_constraint.x.n_enabled_row,
                                               mpc_constraint.x.n_enabled_col);
            int i = 0;
            if (mpc_constraint.x.idx_enabled[0]) {
              mpc_constraint.x.constraint.row(i++) = constraint.s_max;
            }
            if (mpc_constraint.x.idx_enabled[1]) {
              mpc_constraint.x.constraint.row(i++) = constraint.v_max;
            }
          }
          if (mpc_constraint.u.idx_enabled[0]) {  // u
            mpc_constraint.u.constraint.resize(mpc_constraint.u.n_enabled_row,
                                               mpc_constraint.u.n_enabled_col);
            mpc_constraint.u.constraint.row(0) = constraint.a_max;
          }
          if (mpc_constraint.u_dot.idx_enabled[0]) {  // u_dot
            mpc_constraint.u_dot.constraint.resize(
                mpc_constraint.u_dot.n_enabled_row,
                mpc_constraint.u_dot.n_enabled_col);
            mpc_constraint.u_dot.constraint.row(0) = constraint.j_max;
          }
        }
      };
  UpdateConstraints(constraints.hard, true, mpc_constraints.hard_min);
  UpdateConstraints(constraints.hard, false, mpc_constraints.hard_max);
  UpdateConstraints(constraints.stiff, true, mpc_constraints.stiff_min);
  UpdateConstraints(constraints.stiff, false, mpc_constraints.stiff_max);
  UpdateConstraints(constraints.soft, true, mpc_constraints.soft_min);
  UpdateConstraints(constraints.soft, false, mpc_constraints.soft_max);
}

}  // namespace planning
}  // namespace zark
