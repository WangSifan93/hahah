/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_mpc_data_mocker.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_mpc_data_mocker.h"
#include "apps/planning/src/motion/lateral/lateral_stop_hold.h"
#include "apps/planning/src/common/configs/vehicle_config_helper.h"

namespace zark {
namespace planning {

LateralMPCDataMocker::LateralMPCDataMocker(LateralOptimizerConfig& config) {
  config_ = config;
}

LatMPCData LateralMPCDataMocker::GenerateLateralMPCData(
    const CorridorInfo& corridor_info, const LonMPCData& lon_mpc_data,
    LateralReference* lat_ref) const {
  const int n_steps = config_.model.num_steps;
  const int n_x = config_.model.num_states;
  const int n_u = config_.model.num_ctrls;
  const int n_nodes = config_.model.num_steps + 1;
  const double dt = config_.model.dt;
  const int n_u_dot = config_.model.num_ctrl_rates;
  LatMPCData mpc_data;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  mpc_data.tube = GenerateEmptyTube(corridor_info, lon_mpc_data);
  std::tie(x_ref, u_ref) = lat_ref->ConstructReferenceTrajectory(mpc_data.tube);

  mpc_data.names.x.emplace_back("l");
  mpc_data.names.x.emplace_back("l_dot");
  mpc_data.names.x.emplace_back("psi_s");
  mpc_data.names.x.emplace_back("psi_s_dot");
  mpc_data.names.u.emplace_back("delta");
  mpc_data.names.u_dot.emplace_back("delta_dot");

  mpc_data.t = util::TVector(dt, n_nodes);
  mpc_data.x_ref = x_ref;
  mpc_data.u_ref = u_ref;
  mpc_data.x = x_ref;
  mpc_data.u = u_ref;
  mpc_data.u_dot = Eigen::MatrixXd::Zero(n_u_dot, n_steps);
  mpc_data.constraints = MPCData::Constraints(n_x, n_u, n_u_dot);
  mpc_data.slacks = MPCData::Slacks(n_x, n_u, n_u_dot, n_nodes, n_steps);
  mpc_data.dt = dt;
  mpc_data.n_x = n_x;
  mpc_data.n_u = n_u;
  mpc_data.n_u_dot = n_u_dot;
  mpc_data.n_steps = n_steps;
  return mpc_data;
}

Tube LateralMPCDataMocker::GenerateEmptyTube(
    const CorridorInfo& corridor_info, const LonMPCData& lon_mpc_data) const {
  Tube tube;
  const int n_nodes = config_.model.num_steps + 1;
  const Corridor& corridor = corridor_info.GetCorridor();
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_half_width = vehicle_param.width() / 2.0;

  tube.pts.reserve(n_nodes);
  for (int k = 0; k < n_nodes; k++) {
    const CorridorPoint corridor_point =
        corridor.EvaluateByS(lon_mpc_data.x.coeff(0, k));
    Tube::TubePoint tube_point;
    tube_point.t = lon_mpc_data.t.coeff(k);
    tube_point.s = lon_mpc_data.x.coeff(kIdxS, k);
    tube_point.l_left_ref = corridor_point.l + adc_half_width;
    tube_point.l_right_ref = corridor_point.l - adc_half_width;
    tube_point.l_left_hard = corridor_point.l_left;
    tube_point.l_left_stiff = tube_point.l_left_hard;
    tube_point.l_left_soft = tube_point.l_left_stiff;
    tube_point.l_right_hard = corridor_point.l_right;
    tube_point.l_right_stiff = tube_point.l_right_hard;
    tube_point.l_right_soft = tube_point.l_right_stiff;
    tube.pts.emplace_back(tube_point);
  }
  return tube;
}

}  // namespace planning
}  // namespace zark
