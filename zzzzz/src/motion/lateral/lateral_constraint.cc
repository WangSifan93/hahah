/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_constraint.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_constraint.h"

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

MPCData::Constraints LateralConstraint::AssignConstraints(
    const Tube& tube) const {
  const int n_steps = config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  const double kLMax = 100.0;
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_half_width = vehicle_param.width() / 2.0;

  Constraints constraints;
  Constraints::Constraint& stiff = constraints.stiff;
  Constraints::Constraint& soft = constraints.soft;

  // l constraints
  soft.l_max = Eigen::RowVectorXd::Ones(n_nodes) * kLMax;
  stiff.l_max = Eigen::RowVectorXd::Ones(n_nodes) * kLMax;
  soft.l_min = Eigen::RowVectorXd::Ones(n_nodes) * -kLMax;
  stiff.l_min = Eigen::RowVectorXd::Ones(n_nodes) * -kLMax;

  for (int k = 0; k < n_nodes; ++k) {
      stiff.l_max(k) =
          std::min(stiff.l_max(k), tube.pts[k].l_left_stiff - adc_half_width);
      soft.l_max(k) = std::min(soft.l_max(k), tube.pts[k].l_left_soft - adc_half_width);
      stiff.l_min(k) =
          std::max(stiff.l_min(k), tube.pts[k].l_right_stiff + adc_half_width);
      soft.l_min(k) = std::max(soft.l_min(k), tube.pts[k].l_right_soft + adc_half_width);
  }

  // TODO:add l dot constraints
  // TODO:add psi constraints
  // TODO:add psi dot constraints

  // u and u dot constraints
  const double delta_min =
      -vehicle_param.max_steer_angle() / vehicle_param.steer_ratio();
  const double delta_max =
      vehicle_param.max_steer_angle() / vehicle_param.steer_ratio();
  const double delta_dot_min =
      -vehicle_param.max_steer_angle_rate() / vehicle_param.steer_ratio();
  const double delta_dot_max =
      vehicle_param.max_steer_angle_rate() / vehicle_param.steer_ratio();

  soft.delta_min = Eigen::RowVectorXd::Ones(n_steps) * delta_min;
  soft.delta_max = Eigen::RowVectorXd::Ones(n_steps) * delta_max;
  soft.delta_dot_min = Eigen::RowVectorXd::Ones(n_steps) * delta_dot_min;
  soft.delta_dot_max = Eigen::RowVectorXd::Ones(n_steps) * delta_dot_max;
  stiff.delta_min = Eigen::RowVectorXd::Ones(n_steps) * delta_min;
  stiff.delta_max = Eigen::RowVectorXd::Ones(n_steps) * delta_max;
  stiff.delta_dot_min = Eigen::RowVectorXd::Ones(n_steps) * delta_dot_min;
  stiff.delta_dot_max = Eigen::RowVectorXd::Ones(n_steps) * delta_dot_max;

  // TODO:(Chef) add u and u dot soft by lookuptable

  return ConvertToMPCConstraints(constraints);
}

MPCData::Constraints LateralConstraint::ConvertToMPCConstraints(
    Constraints& constraints) const {
  MPCData::Constraints mpc_constraints;

  UpdateMPCConstraintIndices(constraints, mpc_constraints);

  UpdateMPCConstraintSizes(mpc_constraints);

  UpdateMPCConstraintWeights(config_.cost, mpc_constraints);

  UpdateMPCConstraints(constraints, mpc_constraints);

  return mpc_constraints;
}

void LateralConstraint::UpdateMPCConstraintIndices(
    Constraints& constraints, MPCData::Constraints& mpc_constraints) const {
  auto UpdateConstraintIndices =
      [this](const Constraints::Constraint& constraint, const bool is_min,
             MPCData::Constraints::Constraint& mpc_constraint) {
        mpc_constraint.x.idx_enabled.resize(config_.model.num_states);
        mpc_constraint.u.idx_enabled.resize(config_.model.num_ctrls);
        mpc_constraint.u_dot.idx_enabled.resize(config_.model.num_ctrl_rates);
        if (is_min) {
          mpc_constraint.x.idx_enabled[kIdxL] = !(constraint.l_min.size() == 0);
          mpc_constraint.x.idx_enabled[kIdxLDot] =
              !(constraint.l_dot_min.size() == 0);
          mpc_constraint.x.idx_enabled[kIdxPsiS] =
              !(constraint.psi_min.size() == 0);
          mpc_constraint.x.idx_enabled[kIdxPsiSDot] =
              !(constraint.psi_dot_min.size() == 0);
          mpc_constraint.u.idx_enabled[kIdxDelta] =
              !(constraint.delta_min.size() == 0);
          mpc_constraint.u_dot.idx_enabled[kIdxDeltaDot] =
              !(constraint.delta_dot_min.size() == 0);
        } else {
          mpc_constraint.x.idx_enabled[kIdxL] = !(constraint.l_max.size() == 0);
          mpc_constraint.x.idx_enabled[kIdxLDot] =
              !(constraint.l_dot_max.size() == 0);
          mpc_constraint.x.idx_enabled[kIdxPsiS] =
              !(constraint.psi_max.size() == 0);
          mpc_constraint.x.idx_enabled[kIdxPsiSDot] =
              !(constraint.psi_dot_max.size() == 0);
          mpc_constraint.u.idx_enabled[kIdxDelta] =
              !(constraint.delta_max.size() == 0);
          mpc_constraint.u_dot.idx_enabled[kIdxDeltaDot] =
              !(constraint.delta_dot_max.size() == 0);
        }
      };

  UpdateConstraintIndices(constraints.soft, true, mpc_constraints.soft_min);
  UpdateConstraintIndices(constraints.soft, false, mpc_constraints.soft_max);
  UpdateConstraintIndices(constraints.stiff, true, mpc_constraints.stiff_min);
  UpdateConstraintIndices(constraints.stiff, false, mpc_constraints.stiff_max);
  UpdateConstraintIndices(constraints.hard, true, mpc_constraints.hard_min);
  UpdateConstraintIndices(constraints.hard, false, mpc_constraints.hard_max);
}

void LateralConstraint::UpdateMPCConstraintSizes(
    MPCData::Constraints& mpc_constraints) const {
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

        UpdateConstraintInfoSizes(config_.model.num_states,
                                  config_.model.num_steps + 1,
                                  mpc_constraint.x);
        UpdateConstraintInfoSizes(config_.model.num_ctrls,
                                  config_.model.num_steps, mpc_constraint.u);
        UpdateConstraintInfoSizes(config_.model.num_ctrl_rates,
                                  config_.model.num_steps,
                                  mpc_constraint.u_dot);
        mpc_constraint.n_ineq =
            mpc_constraint.x.n_enabled_row * mpc_constraint.x.n_enabled_col +
            mpc_constraint.u.n_enabled_row * mpc_constraint.u.n_enabled_col +
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

void LateralConstraint::UpdateMPCConstraintWeights(
    const LateralOptimizerConfig::Cost& cost,
    MPCData::Constraints& mpc_constraints) const {
  auto UpdateSingleMPCConstraintWeights =
      [](const LateralOptimizerConfig::Cost::Weight& weight,
         MPCData::Constraints::Constraint& mpc_constraint) {
        if (mpc_constraint.x.n_enabled_row > 0) {  // x
          mpc_constraint.x.weight.resize(mpc_constraint.x.n_enabled_row);
          int i = 0;
          if (mpc_constraint.x.idx_enabled[kIdxL]) {
            mpc_constraint.x.weight(i++) = weight.l;
          }
          if (mpc_constraint.x.idx_enabled[kIdxLDot]) {
            mpc_constraint.x.weight(i++) = weight.l_dot;
          }
          if (mpc_constraint.x.idx_enabled[kIdxPsiS]) {
            mpc_constraint.x.weight(i++) = weight.psi;
          }
          if (mpc_constraint.x.idx_enabled[kIdxPsiSDot]) {
            mpc_constraint.x.weight(i++) = weight.psi_dot;
          }
        }
        if (mpc_constraint.u.n_enabled_row > 0) {  // u
          mpc_constraint.u.weight.resize(mpc_constraint.u.n_enabled_row);
          mpc_constraint.u.weight(0) = weight.delta;
        }
        if (mpc_constraint.u_dot.n_enabled_row > 0) {  // u_dot
          mpc_constraint.u_dot.weight.resize(
              mpc_constraint.u_dot.n_enabled_row);
          mpc_constraint.u_dot.weight(0) = weight.delta_dot;
        }
      };

  UpdateSingleMPCConstraintWeights(config_.cost.soft_min,
                                   mpc_constraints.soft_min);
  UpdateSingleMPCConstraintWeights(config_.cost.soft_max,
                                   mpc_constraints.soft_max);
  UpdateSingleMPCConstraintWeights(config_.cost.stiff_min,
                                   mpc_constraints.stiff_min);
  UpdateSingleMPCConstraintWeights(config_.cost.stiff_max,
                                   mpc_constraints.stiff_max);
}

void LateralConstraint::UpdateMPCConstraints(
    const Constraints& constraints,
    MPCData::Constraints& mpc_constraints) const {
  auto UpdateConstraints =
      [](const Constraints::Constraint& constraint, const bool is_min,
         MPCData::Constraints::Constraint& mpc_constraint) {
        if (is_min) {
          if (mpc_constraint.x.n_enabled_row > 0) {  // x
            mpc_constraint.x.constraint.resize(mpc_constraint.x.n_enabled_row,
                                               mpc_constraint.x.n_enabled_col);
            int i = 0;
            if (mpc_constraint.x.idx_enabled[kIdxL]) {
              mpc_constraint.x.constraint.row(i++) = constraint.l_min;
            }
            if (mpc_constraint.x.idx_enabled[kIdxLDot]) {
              mpc_constraint.x.constraint.row(i++) = constraint.l_dot_min;
            }
            if (mpc_constraint.x.idx_enabled[kIdxPsiS]) {
              mpc_constraint.x.constraint.row(i++) = constraint.psi_min;
            }
            if (mpc_constraint.x.idx_enabled[kIdxPsiSDot]) {
              mpc_constraint.x.constraint.row(i++) = constraint.psi_dot_min;
            }
          }
          if (mpc_constraint.u.idx_enabled[kIdxDelta]) {  // u
            mpc_constraint.u.constraint.resize(mpc_constraint.u.n_enabled_row,
                                               mpc_constraint.u.n_enabled_col);
            mpc_constraint.u.constraint.row(0) = constraint.delta_min;
          }
          if (mpc_constraint.u_dot.idx_enabled[kIdxDeltaDot]) {  // u_dot
            mpc_constraint.u_dot.constraint.resize(
                mpc_constraint.u_dot.n_enabled_row,
                mpc_constraint.u_dot.n_enabled_col);
            mpc_constraint.u_dot.constraint.row(0) = constraint.delta_dot_min;
          }
        } else {
          if (mpc_constraint.x.n_enabled_row > 0) {  // x
            mpc_constraint.x.constraint.resize(mpc_constraint.x.n_enabled_row,
                                               mpc_constraint.x.n_enabled_col);
            int i = 0;
            if (mpc_constraint.x.idx_enabled[kIdxL]) {
              mpc_constraint.x.constraint.row(i++) = constraint.l_max;
            }
            if (mpc_constraint.x.idx_enabled[kIdxLDot]) {
              mpc_constraint.x.constraint.row(i++) = constraint.l_dot_max;
            }
            if (mpc_constraint.x.idx_enabled[kIdxPsiS]) {
              mpc_constraint.x.constraint.row(i++) = constraint.psi_max;
            }
            if (mpc_constraint.x.idx_enabled[kIdxPsiSDot]) {
              mpc_constraint.x.constraint.row(i++) = constraint.psi_dot_max;
            }
          }
          if (mpc_constraint.u.idx_enabled[kIdxDelta]) {  // u
            mpc_constraint.u.constraint.resize(mpc_constraint.u.n_enabled_row,
                                               mpc_constraint.u.n_enabled_col);
            mpc_constraint.u.constraint.row(0) = constraint.delta_max;
          }
          if (mpc_constraint.u_dot.idx_enabled[kIdxDeltaDot]) {  // u_dot
            mpc_constraint.u_dot.constraint.resize(
                mpc_constraint.u_dot.n_enabled_row,
                mpc_constraint.u_dot.n_enabled_col);
            mpc_constraint.u_dot.constraint.row(0) = constraint.delta_dot_max;
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
