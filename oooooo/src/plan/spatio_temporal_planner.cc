
#include <cmath>
#include <cstddef>

#include "common/plan_start_point_info.h"
#include "common/planning_macros.h"
#include "common/timer.h"
#include "container/strong_int.h"
#include "context/e2e_planner_utils.h"
#include "context/lane_manager.h"
#include "decision_exploration/assist_util.h"
#include "decision_exploration/candidate_lane_sequences.h"
#include "decision_exploration/decision_exploration.h"
#include "decision_exploration/decision_exploration_input.h"
#include "decision_exploration/decision_exploration_output.h"
#include "decision_exploration/lane_graph/lane_path_finder.h"

namespace e2e_noa {
namespace spt {
SpationTemporal::SpationTemporal(
    const PathConfigBucket *config_bucket,
    const std::shared_ptr<PathTaskPipelineContext> &pipeline_context) {
  name_ = "SpationTemporal";
}

bool SpationTemporal::execute(Frame *frame) {
  auto *lat_ilqr_data = proto_data.mutable_lat_ilqr_data();
  auto *ilqr_input = lat_ilqr_data->mutable_lat_ilqr_input();

  auto *m_decision_output = ilqr_input->mutable_micro_decider_output();
  path_planner::MDecision m_decision;
  m_decision.Update(*input, *output, ref_path, m_decision_output,
                    &path_planner_common);
  const double end_micro_update = msquare::now_ms();
  t_ptr->set_micro_decider(end_micro_update - end_preprocessor_update);

  if (ConfigurationContext::Instance().planner_config().dump_gsr_proto) {
    planning_data.gap_selector_input =
        frame_->planning_context().gap_selector_input();
    planning_data.gap_selector_input.__add_to_json___ = true;

    planning_data.gap_selector_output =
        frame_->planning_context().gap_selector_output();
    planning_data.gap_selector_output.__add_to_json___ = true;
  }
  // run ilqr
  const double ilqr_start_time_ms = msquare::now_ms();
  auto &ilqr_path_planner = spt_ilqr::ILQRPathPlannerProblem::get_instance();
  auto *solver_config = ilqr_input->mutable_solver_config();
  solver_config->CopyFrom(*ilqr_path_planner.GetSolverConfig());
  auto *ilqr_config = solver_config->mutable_ilqr_config();

  // solver
  ilqr_path_planner.Reset();
  ilqr_path_planner.ResizeTimestep(
      static_cast<unsigned long>(ilqr_config->horizon()) + 1U);
  for (int i = 0; i < ilqr_config->horizon(); ++i) {
    ilqr_path_planner.SetTimeStep(
        input->lat_ref_speed(i + 1).t() - input->lat_ref_speed(i).t(),
        static_cast<unsigned long>(i));
  }

  auto *output = lat_ilqr_data->mutable_lat_ilqr_output();
  auto &path = pipeline_context_->reusable_result->obj->lat_dense_path;
  ilqr_path_planner.Update(ref_path->GetRefPoints(), ilqr_input, output,
                           &path.states);
  const double ilqr_end_time_ms = msquare::now_ms();
  t_ptr->set_ilqr_solver(ilqr_end_time_ms - ilqr_start_time_ms);
  VisSptTraj(*output);
  t_ptr->set_dump_data(0.0F);

  GenDensePathSAndL(path.states, frenet_coord_, &path.s_list, &path.l_list,
                    &path.path_s_list);
  SetOutputSList(frenet_coord_,
                 output->mutable_final_output()->mutable_states());
  const int traj_points_size = static_cast<int>(result.traj_points.size());
  const double ddp_last_s =
      result.traj_points.at(static_cast<unsigned long>(traj_points_size) - 1U)
          .s;
  const auto &final_output = output->final_output().states();

  for (auto &point : result.traj_points) {
    const auto index = GetIndexFromS(path.s_list, point.s);
    const auto state = GetInterpBetween(path.states, index);

    point.x = state.x();
    point.y = state.y();

    point.curvature = state.curvature();
    point.heading_angle = state.theta();
    point.l = GetInterpBetween(path.l_list, index);
  }
  const double lateral_optimizer_execute_end = msquare::now_ms();
  PlannerDebugUtils::append_cost_time(
      frame_->mutable_ego_prediction_context()->mutable_planner_debug(),
      "lateral_optimizer_execute",
      lateral_optimizer_execute_end - lateral_optimizer_execute_start);
  t_ptr->set_total_optimizer(lateral_optimizer_execute_end -
                             lateral_optimizer_execute_start);

  return true;
}
}  // namespace spt
}  // namespace e2e_noa
