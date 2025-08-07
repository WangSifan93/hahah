/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_optimizer.cc
 **/

#include "apps/planning/src/tasks/optimizers/lateral_optimizer/lateral_optimizer.h"

#include <cmath>

#include "apps/planning/src/common/conversion.h"
#include "apps/planning/src/motion/longitudinal/data_type.h"
#include "cartesian_frenet_conversion.h"
#include "point_factory.h"

namespace zark {
namespace planning {

using ::common::Status;
using ::math::CartesianFrenetConverter;
using ::util::PointFactory;

LateralOptimizer::LateralOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector) {
  config_ = config.task_config().lateral_optimizer_config;
  Eigen::VectorXd Q;
  Eigen::VectorXd R;
  Eigen::VectorXd R_dot;

  lat_nudger_ = std::make_shared<LateralNudger>(config_);
  lat_ref_ = std::make_shared<LateralReference>(config_);
  lat_con_ = std::make_shared<LateralConstraint>(config_);
  lat_table_ = std::make_shared<LateralLookupTables>(config_);
  lat_tube_ = std::make_shared<LateralTube>(config_);
  lat_stop_hold_ = std::make_shared<LateralStopHold>(config_);
  lat_mpc_data_mocker_ = std::make_shared<LateralMPCDataMocker>(config_);
  std::tie(Q, R, R_dot) = GetCostFunctionMatrix(config_);
  lat_mpc_problem_ = std::make_shared<LateralMPCProblem>(
      config_.model.num_steps, config_.model.num_states,
      config_.model.num_ctrls, config_.model.num_ctrl_rates, Q, R, R_dot,
      config_.model.dt, config_.cost.w_terminal);
}

Status LateralOptimizer::Execute(Frame* frame) {
  Status status;
  for (auto& proposal : *(frame->MutableProposals())) {
    status = Process(frame->PlanningStartPoint(),
                     *frame->FindCurrentLocalRoute(), proposal);
  }
  return status;
}

Status LateralOptimizer::Process(
    const ::common::TrajectoryPoint& planning_start_point,
    const LocalRoute& cur_local_route, Proposal& proposal) {
  if (proposal.GetCorridorInfo() == nullptr) {
    const std::string msg = "Failed to get corridor info.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const LonMPCData& lon_mpc_data = proposal.GetLonMPCData();
  Eigen::VectorXd u_prev;
  Eigen::VectorXd x_init;
  const CorridorInfo& corridor_info = *proposal.GetCorridorInfo();
  std::tie(x_init, u_prev) =
      ConstructEgoInitialState(planning_start_point, corridor_info);
  LatMPCData lat_mpc_data;

  if (lon_mpc_data.is_stop_hold) {
    lat_mpc_data = lat_stop_hold_->GenerateStopTrajectory(
        u_prev, planning_start_point.dt_prev());
  } else if (PlanningGflags::enable_lat_mpc_in_lane_keep &&
             proposal.GetCorridorInfo()->GetType() ==
                 CorridorInfo::Type::LANE_KEEP) {
    const std::vector<Nudger> lateral_nudgers = lat_nudger_->ConstructNudger(
        corridor_info.GetLateralObstacles(), lon_mpc_data, corridor_info);
    const STProposal& st_proposal = proposal.GetSTProposal();

    const bool is_prev_frame_valid =
        injector_->frame_history()->Latest() != nullptr &&
        !injector_->frame_history()->Latest()->GetProposals().empty() &&
        !injector_->frame_history()
             ->Latest()
             ->GetProposals()
             .front()
             .GetLatMPCData()
             .tube.pts.empty();
    const std::vector<Tube::TubePoint>& tube_prev =
        is_prev_frame_valid ? injector_->frame_history()
                                  ->Latest()
                                  ->GetProposals()
                                  .front()
                                  .GetLatMPCData()
                                  .tube.pts
                            : std::vector<Tube::TubePoint>();
    const Corridor corridor_prev = is_prev_frame_valid
                                       ? injector_->frame_history()
                                             ->Latest()
                                             ->GetProposals()
                                             .front()
                                             .GetCorridorInfo()
                                             ->GetCorridor()
                                       : Corridor();
    const Tube tube = lat_tube_->ConstructTubes(
        planning_start_point, lateral_nudgers, corridor_info, lon_mpc_data,
        corridor_prev, tube_prev, *lat_table_, *lat_padding_, st_proposal,
        cur_local_route);

    Eigen::MatrixXd x_ref;
    Eigen::MatrixXd u_ref;
    std::tie(x_ref, u_ref) = lat_ref_->ConstructReferenceTrajectory(tube);

    const MPCData::Constraints mpc_constraints =
        lat_con_->AssignConstraints(tube);

    Eigen::MatrixXd u_2 = ComputeU2(lon_mpc_data, tube, corridor_info);

    lat_mpc_problem_->SetV(lon_mpc_data.x.row(kIdxV));
    if (!lat_mpc_problem_->Solve(x_init, x_ref, u_ref, u_2, mpc_constraints,
                                 u_prev, planning_start_point.dt_prev(),
                                 lat_mpc_data)) {
      const std::string msg = "Failed to obtain a solution.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    lat_mpc_data.u_2 = u_2;
    lat_mpc_data.tube = tube;
  } else {
    lat_mpc_data = lat_mpc_data_mocker_->GenerateLateralMPCData(
        corridor_info, lon_mpc_data, lat_ref_.get());
  }

  proposal.SetLatMPCData(lat_mpc_data);
  proposal.SetTrajectory(ConvertToTrajectory(proposal));
  return Status::OK();
}

DiscretizedTrajectory LateralOptimizer::ConvertToTrajectory(
    Proposal& proposal) const {
  const double kTol = 1.0e-5;
  const int n_steps = config_.model.num_steps;
  const int n_nodes = config_.model.num_steps + 1;
  const LatMPCData& lat_mpc_data = proposal.GetLatMPCData();
  const LonMPCData& lon_mpc_data = proposal.GetLonMPCData();
  std::vector<::common::TrajectoryPoint> trajectory(n_nodes);
  for (int k = 0; k < n_nodes; ++k) {
    double cos_psi_s = std::cos(lat_mpc_data.x(kIdxPsiS, k));
    cos_psi_s = (std::signbit(cos_psi_s) ? -1 : 1) *
                std::max(std::fabs(cos_psi_s), kTol);
    trajectory[k].set_v(std::max(lon_mpc_data.x(kIdxV, k) / cos_psi_s, 0.0));
    trajectory[k].set_a(k < n_steps ? lon_mpc_data.u(kIdxA, k) / cos_psi_s
                                    : trajectory[n_steps - 1].a());
    trajectory[k].set_da(k < n_steps - 1
                             ? lon_mpc_data.u_dot(kIdxJ, k + 1) / cos_psi_s
                             : trajectory[n_steps - 2].da());
    trajectory[k].set_steer(k < n_steps ? lat_mpc_data.u(kIdxDelta, k)
                                        : trajectory[n_steps - 1].steer());
    trajectory[k].set_steer_rate(k < n_steps - 1
                                     ? lat_mpc_data.u_dot(kIdxDeltaDot, k + 1)
                                     : trajectory[n_steps - 2].steer_rate());
    trajectory[k].set_relative_time(k * config_.model.dt);
    const double omega =
        lat_mpc_data.x(kIdxPsiSDot, k) +
        lon_mpc_data.x(kIdxV, k) * proposal.GetCorridorInfo()
                                       ->GetCorridor()
                                       .EvaluateByS(lon_mpc_data.x(kIdxS, k))
                                       .kappa;
    trajectory[k].set_omega(omega);
    trajectory[k].set_v_y(lat_mpc_data.x(kIdxLDot, k) -
                          trajectory[k].v() *
                              std::sin(lat_mpc_data.x(kIdxPsiS, k)));
    ::common::PathPoint path_point;
    ::math::Vec2d xy_point(1.0, 2.0);
    proposal.GetCorridorInfo()->GetCorridor().SLToXY(
        ::common::SLPoint(lon_mpc_data.x(kIdxS, k), lat_mpc_data.x(kIdxL, k)),
        &xy_point);
    path_point.set_x(xy_point.x());
    path_point.set_y(xy_point.y());
    const double s = k > 0 ? trajectory[k - 1].path_point().s() +
                                 xy_point.DistanceTo(::math::Vec2d(
                                     trajectory[k - 1].path_point().x(),
                                     trajectory[k - 1].path_point().y()))
                           : 0.0;
    path_point.set_s(s);
    const double theta =
        lat_mpc_data.x(kIdxPsiS, k) + proposal.GetCorridorInfo()
                                          ->GetCorridor()
                                          .EvaluateByS(lon_mpc_data.x(kIdxS, k))
                                          .theta;
    path_point.set_theta(theta);
    const double kappa =
        omega / std::max(lon_mpc_data.x(kIdxV, k) / cos_psi_s, kTol);
    path_point.set_kappa(kappa);
    trajectory[k].set_path_point(path_point);
  }

  proposal.SetTrajectory(DiscretizedTrajectory(trajectory));
  return DiscretizedTrajectory(trajectory);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
LateralOptimizer::ConstructEgoInitialState(
    const ::common::TrajectoryPoint& planning_start_point,
    const CorridorInfo& corridor_info) const {
  Eigen::VectorXd x_init(config_.model.num_states);
  Eigen::VectorXd u_prev(config_.model.num_ctrls);
  const int idx_start_point = corridor_info.GetIdxStartPoint();
  const FrenetPoint init_frenet_point = corridor_info.GetInitFrenetPoint();
  // TODO(Sifan): Change 0 and 1 by Idx
  const double l = init_frenet_point.l[0];
  const double psi_s = ::math::NormalizeAngle(
      planning_start_point.path_point().theta() -
      corridor_info.GetCorridor().at(idx_start_point).theta);
  const double l_dot =
      planning_start_point.v_y() + planning_start_point.v() * std::sin(psi_s);
  const double psi_s_dot =
      planning_start_point.omega() -
      init_frenet_point.s[1] *
          corridor_info.GetCorridor().at(idx_start_point).kappa;
  x_init << l, l_dot, psi_s, psi_s_dot;
  u_prev(0) = planning_start_point.steer();
  return std::make_pair(x_init, u_prev);
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
LateralOptimizer::GetCostFunctionMatrix(const LateralOptimizerConfig& config) {
  auto AssignCostMatrix = [](const std::vector<double>& M_in) {
    Eigen::VectorXd M_out(M_in.size());
    for (int i = 0; i < static_cast<int>(M_in.size()); ++i) {
      M_out(i) = M_in[i];
    }
    return M_out;
  };

  Eigen::VectorXd Q = AssignCostMatrix(config.cost.Q);
  Eigen::VectorXd R = AssignCostMatrix(config.cost.R);
  Eigen::VectorXd R_dot = AssignCostMatrix(config.cost.R_dot);

  return std::make_tuple(Q, R, R_dot);
}

Eigen::MatrixXd LateralOptimizer::ComputeU2(const LonMPCData& lon_mpc_data,
                                            const Tube& tube,
                                            const CorridorInfo& corridor_info) {
  Eigen::MatrixXd u_2(config_.model.num_ctrl_rates, config_.model.num_steps);
  const Corridor& corridor = corridor_info.GetCorridor();
  for (int k = 0; k < u_2.cols(); k++) {
    const double s = tube.pts[k].s;
    u_2(0, k) = lon_mpc_data.x(kIdxV, k) * corridor.EvaluateByS(s).kappa;
  }
  return u_2;
}

}  // namespace planning
}  // namespace zark
