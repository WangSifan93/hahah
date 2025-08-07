/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_optimizer.cc
 **/

#include "apps/planning/src/tasks/optimizers/longitudinal_optimizer/longitudinal_optimizer.h"

#include "math_utils.h"

namespace zark {
namespace planning {

using ::common::Status;

LongitudinalOptimizer::LongitudinalOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector) {
  config_ = config.task_config().longitudinal_optimizer_config;
  lon_blocker_ = std::make_shared<LongitudinalBlocker>(config_.model);
  lon_ref_ = std::make_shared<LongitudinalReference>(config_);
  lon_con_ = std::make_shared<LongitudinalConstraint>(config_);
  lon_padding_ = std::make_shared<LongitudinalPadding>();
  lon_table_ = std::make_shared<LongitudinalLookupTables>(config_);
  lon_stop_hold_ = std::make_shared<LongitudinalStopHold>(config_);
  Eigen::VectorXd Q, R, R_dot;
  std::tie(Q, R, R_dot) = GetCostFunctionMatrix(config_);
  lon_mpc_problem_ = std::make_shared<LongitudinalMPCProblem>(
      config_.model.num_steps, config_.model.num_states,
      config_.model.num_ctrls, config_.model.num_ctrl_rates, Q, R, R_dot,
      config_.model.dt);
}

Status LongitudinalOptimizer::Execute(Frame* frame) {
  Status status;
  for (auto& proposal : *(frame->MutableProposals())) {
    status = Process(frame->PlanningStartPoint(), proposal,
                     frame->local_view().fct_output);
  }
  return status;
}

Status LongitudinalOptimizer::Process(
    const ::common::TrajectoryPoint& planning_start_point, Proposal& proposal,
    const std::shared_ptr<zark::ads_fct::FCT_Outputs> state_management) {
  if (proposal.GetCorridorInfo() == nullptr) {
    const std::string msg = "Failed to get corridor info.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  Eigen::VectorXd x_init;
  Eigen::VectorXd u_prev;
  std::tie(x_init, u_prev) = ConstructEgoInitialState(
      proposal.GetCorridorInfo(), planning_start_point);

  const STProposal& st_proposal = proposal.GetSTProposal();
  const IndexedPtrList<std::string, const Obstacle*>& obstacle_map =
      proposal.GetCorridorInfo()->GetObstacleMap();
  const Corridor& corridor = proposal.GetCorridorInfo()->GetCorridor();

  const std::vector<Blocker>& blockers =
      lon_blocker_->ConstructBlocker(st_proposal, obstacle_map, corridor);

  const TimeGapLevel time_gap =
      static_cast<TimeGapLevel>(state_management->fct_out_bus_accinfosts()
                                    .fct_out_sts_accdistmdistlvl_edsl());
  const bool is_stop_hold = lon_stop_hold_->IsStopHold(
      x_init, blockers, time_gap, *lon_table_, *lon_padding_);

  if (is_stop_hold || state_management->fct_out_bus_accinfosts()
                          .fct_out_is_accsgrsdslreq_bl()) {
    LonMPCData lon_mpc_data = lon_stop_hold_->GenerateStopTrajectory(
        u_prev, planning_start_point.dt_prev());
    lon_mpc_data.is_stop_hold = true;
    proposal.SetLonMPCData(lon_mpc_data);
    return Status::OK();
  }

  const SpeedLimit& speed_limit =
      proposal.GetCorridorInfo()->GetSpeedLimitMap().at("final");

  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  std::vector<SpeedLimit> speed_limit_set;
  std::tie(x_ref, u_ref) = lon_ref_->DesignReferenceTrajectory(
      x_init, speed_limit, blockers, time_gap, *lon_table_, *lon_padding_,
      speed_limit_set);

  const MPCData::Constraints& mpc_constraints =
      lon_con_->DesignConstraints(x_init, x_ref, blockers, speed_limit,
                                  time_gap, *lon_table_, *lon_padding_);

  LonMPCData lon_mpc_data;
  if (!lon_mpc_problem_->Solve(x_init, x_ref, u_ref, Eigen::MatrixXd(),
                               mpc_constraints, u_prev,
                               planning_start_point.dt_prev(), lon_mpc_data)) {
    const std::string msg = "Failed to obtain a solution.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  lon_mpc_data.speed_limit_set = speed_limit_set;
  lon_mpc_data.is_stop_hold = false;
  proposal.SetLonMPCData(lon_mpc_data);

  return Status::OK();
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd>
LongitudinalOptimizer::ConstructEgoInitialState(
    const CorridorInfo* corridor_info,
    const ::common::TrajectoryPoint& planning_start_point) {
  const double cos_psi_s = std::cos(
      ::math::NormalizeAngle(planning_start_point.path_point().theta() -
                             corridor_info->GetCorridor()
                                 .at(corridor_info->GetIdxStartPoint())
                                 .theta));
  Eigen::VectorXd x_init(config_.model.num_states);
  x_init(kIdxS) = 0.0;
  x_init(kIdxV) = planning_start_point.v() * cos_psi_s;
  Eigen::VectorXd u_prev(config_.model.num_ctrls);
  u_prev(kIdxA) = planning_start_point.a() * cos_psi_s;

  return std::tie(x_init, u_prev);
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
LongitudinalOptimizer::GetCostFunctionMatrix(
    const LongitudinalOptimizerConfig& config) {
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

}  // namespace planning
}  // namespace zark
