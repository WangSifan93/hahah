#include "context/lane_change_command_update.h"

#include "common/type_def.h"
#include "context/ego_state.h"
#include "plan/planner_main_loop_internal.h"

namespace e2e_noa::planning {
void LaneChangeCommandUpdate::update(const PlannerWorldInput &input,
                                     const PlannerState &planner_state,
                                     PlannerWorldOutput *output) {
  preferred_lane_path_ = planner_state.preferred_lane_path;
  new_alc_state_ = planner_state.alc_state;
  new_lc_cmd_state_ = planner_state.lane_change_command;

  auto e2e_planner_context = e2e_planner_context_.lock();
  if (!e2e_planner_context) {
    LOG(ERROR) << "E2EPlannerContext is no longer available in LCCmdUpdate.";
    return;
  }

  const auto &vehicle_geometry =
      e2e_planner_context->ego_state()->vehicle_geometry();
  const auto ego_pos = e2e_planner_context->ego_state()->ego_pos();
  const auto ego_theta = e2e_planner_context->ego_state()->ego_theta();
  const auto &plan_start_point =
      e2e_planner_context->ego_state()->plan_start_point();
  const auto start_point_velocity =
      e2e_planner_context->ego_state()->start_point_velocity();

  auto last_lc_reason = planner_state.last_lc_reason;
  auto last_manual_lc_time = planner_state.last_manual_lc_time;
  const auto &psmm = *planner_state.planner_semantic_map_manager;
  const auto &alc_request =
      planner_state.selector_state.selector_lane_change_request;
  const bool alc_request_waiting =
      alc_request.has_lane_change_reason() &&
      alc_request.lane_change_reason() != LaneChangeReason::NO_CHANGE;
  const auto input_new_lc_cmd =
      alc_request_waiting ? DriverAction::LC_CMD_NONE : input.new_lc_command;

  output->input_lc_cmd = input.new_lc_command;

  if (input_new_lc_cmd != input.new_lc_command) {
    LOG(INFO) << "Ignore driver action: "
              << DriverAction_LaneChangeCommand_Name(input.new_lc_command)
              << ", when alc request waits.";
  }

  HandleManualLcCommand(
      plan_start_point, vehicle_geometry, psmm, input_new_lc_cmd,
      planner_state.prev_lane_path_before_lc, planner_state.lane_change_state,
      ego_pos, ego_theta, &preferred_lane_path_, &new_alc_state_,
      &new_lc_cmd_state_, &output->lc_unable_reason, &output->if_cancel_lc,
      last_manual_lc_time, last_lc_reason);
  Log2FG::LogDataV2("LaneChange",
                    "[ManualLc][raw]: " + std::to_string(input_new_lc_cmd));
  Log2FG::LogDataV2("Manual",
                    "[ManualLc][new]: " + std::to_string(new_lc_cmd_state_));
  Log2FG::LogDataV2("Manual", "[ManualLc][if_cancel_lc]: " +
                                  std::to_string(output->if_cancel_lc));

  Log2FG::LogDataV2(
      "LaneChange",
      "[ManualLc][style]: " +
          std::to_string(
              input.planner_params->lc_decision_params().lane_change_style()));

  auto lane_change_style_ = LC_STYLE_NORMAL;
  switch (input.planner_params->lc_decision_params().lane_change_style()) {
    case LaneChangeStyle::LC_STYLE_NORMAL:
      lane_change_style_ = LC_STYLE_NORMAL;
      break;
    case LaneChangeStyle::LC_STYLE_RADICAL:
      lane_change_style_ = LC_STYLE_RADICAL;
      break;
    case LaneChangeStyle::LC_STYLE_CONSERVATIVE:
      lane_change_style_ = LC_STYLE_CONSERVATIVE;
      break;
    default:
      lane_change_style_ = LC_STYLE_NORMAL;
      break;
  }

  if (input.pnp_infos && input.pnp_infos->infos_size() > 0) {
    const int lc_reason = input.pnp_infos->infos()[0].lc_reason();
    pnp_top1_reason_ = input.pnp_infos->infos()[0].lc_reason();
    if (lc_reason == LcReason::LC_REASON_FOR_NAVI &&
        start_point_velocity < 13.8) {
      lane_change_style_ = LC_STYLE_NORMAL;
    }
    if (lc_reason == LcReason::LC_REASON_FOR_MERGE &&
        start_point_velocity < 13.8) {
      lane_change_style_ = LC_STYLE_RADICAL;
    }
  }
  Log2FG::LogDataV2(
      "LaneChange",
      "[Style][PNP]: " + std::to_string(planner_state.last_lc_style));
}

}  // namespace e2e_noa::planning