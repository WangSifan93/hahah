#include "plan/st_planner.h"

#include <optional>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "async/async_util.h"
#include "async/thread_pool.h"
#include "common/log_data.h"
#include "common/planning_macros.h"
#include "common/timer.h"
#include "constraint.pb.h"
#include "descriptor/all_descriptor_builder.h"
#include "descriptor/constraint_manager.h"
#include "descriptor/descriptor_input.h"
#include "descriptor/descriptor_util.h"
#include "descriptor/leading_groups_builder.h"
#include "descriptor/traffic_gap_finder.h"
#include "lane_change.pb.h"
#include "object/partial_spacetime_object_trajectory.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "object/spacetime_planner_object_trajectories_builder.h"
#include "optimizer.pb.h"
#include "plan/discretized_path.h"
#include "plan/planner_defs.h"
#include "plan/planner_flags.h"
#include "plan/st_path_planner.h"
#include "plan/trajectory_validation.h"
#include "planner_object.pb.h"
#include "planner_status.pb.h"
#include "prediction/predicted_trajectory.h"
#include "speed/speed_planning.h"
#include "speed/speed_planning_flags.h"
#include "speed/speed_planning_input.h"
#include "speed/speed_planning_output.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "trajectory_initialization.pb.h"
#include "trajectory_validation.pb.h"
#include "util/planner_status_macros.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

namespace {
void ModifySpeedPlanningParamsStyle(
    const SpeedPlanningParamsProto& speed_planning_lc_radical_params,
    const SpeedPlanningParamsProto& speed_planning_lc_conservative_params,
    const TrafficGapResult& traffic_gap, LaneChangeStage lc_stage,
    SpeedResponseStyle speed_response_style,
    SpeedPlanningParamsProto* speed_planning_params) {
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    switch (speed_response_style) {
      case SPEED_RESPONSE_CONSERVATIVE:
        *speed_planning_params = speed_planning_lc_conservative_params;
      case SPEED_RESPONSE_NORMAL:
        break;
      case SPEED_RESPONSE_FAST:
      case SPEED_RESPONSE_RADICAL:
        *speed_planning_params = speed_planning_lc_radical_params;
        break;
      default:
        break;
    }
  } else if (traffic_gap.acc_gap_target_speed.has_value()) {
    *speed_planning_params = speed_planning_lc_radical_params;
  }
}
}  // namespace
PlannerStatus StPlanner(
    const StPlannerInput& input, DecisionExplorationOutput decision_exploration,
    StPlannerOutput* st_output, StPlannerDebug* debug_info,
    WorkerThreadManager* thread_pool,
    std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>&
        agent_status_history) {
  NOA_DebugFrameCreate(std::move(std::to_string(input.plan_id)));

  SCOPED_TRACE(__FUNCTION__);
  CHECK_NOTNULL(st_output);
  CHECK_NOTNULL(debug_info);

  st_output->plan_id = input.plan_id;
  absl::Cleanup fill_decision_exploration = [&st_output,
                                             &decision_exploration] {
    st_output->decision_exploration = std::move(decision_exploration);
  };

  const auto& spacetime_constraint_params =
      *CHECK_NOTNULL(input.spacetime_constraint_params);
  const auto& decision_constraint_config =
      *CHECK_NOTNULL(input.decision_constraint_config);
  const auto& vehicle_params = *input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decision_output,
      BuildDescriptorResults(
          {.plan_id = input.plan_id,
           .vehicle_geometry_params = &vehicle_geom_params,
           .spacetime_constraint_params = input.spacetime_constraint_params,
           .config = input.decision_constraint_config,
           .planner_semantic_map_manager = input.planner_semantic_map_manager,
           .lc_state = &decision_exploration.lane_change_state,
           .plan_start_point = &input.start_point_info->start_point,
           .lane_path_before_lc = &decision_exploration.lane_path_before_lc,
           .passage = &decision_exploration.plan_passage,
           .sl_boundary = &decision_exploration.sl_boundary,
           .borrow_lane_boundary = decision_exploration.borrow_lane,
           .obj_mgr = input.obj_mgr,
           .obs_history = input.obs_history,
           .st_traj_mgr = input.st_traj_mgr,

           .traffic_light_status_map = input.traffic_light_status_map,
           .pre_decision_state = input.decision_state,

           .enable_tl_ok_btn = input.enable_tl_ok_btn,
           .override_passable = input.override_passable,

           .max_reach_length = input.left_navi_dist_v2,
           .left_navi_dist_map = input.left_navi_dist,
           .lc_num = input.cur_navi_lc_num,
           .plan_time = input.start_point_info->plan_time,
           .route_target_info = input.route_target_info,
           .scene_reasoning = input.scene_reasoning,

           .behavior = input.behavior,
           .speed_state = input.speed_state,
           .cur_dist_to_junction = input.cur_dist_to_junction,
           .lc_lead_obj_ids = input.lc_lead_obj_ids,
           .push_dir = input.push_dir,
           .last_turn_type_v2 = input.last_turn_type_v2}),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);

  st_output->distance_to_traffic_light_stop_line =
      decision_output.distance_to_traffic_light_stop_line;
  st_output->tl_stop_interface = decision_output.tl_stop_interface;
  st_output->speed_state = decision_output.speed_state;

  auto init_st_planner_object_traj = BuildSpacetimePlannerObjectTrajectories(
      {.psmm = input.planner_semantic_map_manager,
       .passage = &decision_exploration.plan_passage,
       .sl_boundary = &decision_exploration.sl_boundary,
       .lane_change_state = &decision_exploration.lane_change_state,
       .veh_geom = &vehicle_geom_params,
       .plan_start_point = &input.start_point_info->start_point,
       .st_planner_start_offset = input.st_path_start_point_info
                                      ->relative_index_from_plan_start_point *
                                  kTrajectoryTimeStep,
       .prev_st_trajs = input.st_planner_object_trajectories,
       .time_aligned_prev_traj = input.time_aligned_prev_traj,
       .stop_lines = decision_output.constraint_manager.StopLine(),
       .spacetime_planner_object_trajectories_params =
           input.spacetime_planner_object_trajectories_params,
       .nudge_object_info = input.nudge_object_info},
      input.st_traj_mgr->trajectories());

  StPathPlannerOutput path_output;
  auto path_status = RunStPathPlanner(
      {.plan_id = input.plan_id,
       .st_path_start_point_info = input.st_path_start_point_info,
       .path_look_ahead_duration = input.st_path_start_point_info->plan_time -
                                   input.start_point_info->plan_time,
       .vehicle_params = input.vehicle_params,
       .planner_semantic_map_manager = input.planner_semantic_map_manager,
       .smooth_result_map = input.smooth_result_map,
       .decision_exploration = std::move(decision_exploration),
       .traj_mgr = input.st_traj_mgr,
       .pnp_infos = input.pnp_infos,
       .lane_change_style = input.lane_change_style,

       .start_point_info = input.start_point_info,
       .route_target_info = input.route_target_info,
       .obj_mgr = input.obj_mgr,

       .traffic_light_status_map = input.traffic_light_status_map,
       .prev_decision_state = input.decision_state,

       .init_st_planner_object_traj = std::move(init_st_planner_object_traj),
       .stalled_objects = input.stalled_objects,
       .scene_reasoning = input.scene_reasoning,
       .decision_output = std::move(decision_output),
       .prev_target_lane_path_from_start =
           input.prev_target_lane_path_from_start,
       .time_aligned_prev_traj = input.time_aligned_prev_traj,
       .prev_initialization_state = input.initialization_state,
       .trajectory_optimizer_state_proto =
           input.trajectory_optimizer_state_proto,

       .decision_constraint_config = &decision_constraint_config,
       .initialization_params = input.initialization_params,
       .trajectory_optimizer_params = input.trajectory_optimizer_params,
       .spacetime_constraint_params = &spacetime_constraint_params,
       .planner_functions_params = input.planner_functions_params,
       .vehicle_models_params = input.vehicle_models_params,
       .trajectory_optimizer_lc_radical_params =
           input.trajectory_optimizer_lc_radical_params,
       .trajectory_optimizer_lc_normal_params =
           input.trajectory_optimizer_lc_normal_params,
       .trajectory_optimizer_lc_conservative_params =
           input.trajectory_optimizer_lc_conservative_params,

       .behavior = input.behavior,
       .speed_state = input.speed_state,
       .miss_navi_scenario = input.miss_navi_scenario,
       .obs_history = input.obs_history,
       .cur_navi_lc_num = input.cur_navi_lc_num,
       .left_navi_dist = input.left_navi_dist,
       .left_navi_dist_v2 = input.left_navi_dist_v2,
       .prev_lane_change_stage = input.prev_lane_change_stage,
       .lc_cmd_state = input.lc_cmd_state,
       .push_dir = input.push_dir,
       .nudge_object_info = input.nudge_object_info,
       .cur_dist_to_junction = input.cur_dist_to_junction},
      &path_output, thread_pool);

  for (const auto& stop_line : path_output.constraint_manager.StopLine()) {
    if (stop_line.source().type_case() !=
            SourceProto::TypeCase::kEndOfPathBoundary &&
        !st_output->first_stop_s.has_value()) {
      st_output->first_stop_s = stop_line.s();
    }

    if (stop_line.source().type_case() ==
            SourceProto::TypeCase::kTrafficLight &&
        !st_output->redlight_lane_id.has_value()) {
      st_output->redlight_lane_id =
          mapping::ElementId(stop_line.source().traffic_light().lane_id());
    }
  }

  st_output->unsafe_object_ids = std::move(path_output.unsafe_object_ids);
  st_output->decision_state = std::move(path_output.decision_state);
  st_output->initialization_state = std::move(path_output.initialization_state);
  debug_info->initialization_debug_proto =
      std::move(path_output.initialization_debug_proto);
  st_output->trajectory_optimizer_state_proto =
      std::move(path_output.trajectory_optimizer_state_proto);
  st_output->traffic_gap =
      std::move(path_output.constraint_manager.TrafficGap());
  st_output->nudge_object_info = std::move(path_output.nudge_object_info);
  st_output->obj_lead = std::move(path_output.obs_leading);
  st_output->lc_status_code = std::move(path_output.lc_status_code);
  st_output->is_init_follow_scene = path_output.is_init_follow_scene;
  st_output->lc_lead_obj_id = path_output.lc_lead_obj_id;
  debug_info->optimizer_debug_proto =
      std::move(path_output.optimizer_debug_proto);
  st_output->st_path_points = std::move(path_output.st_path_points);

  for (const auto& st_planner_traj_info :
       path_output.st_planner_object_traj.trajectory_infos) {
    auto* st_planner_traj_proto =
        st_output->st_planner_object_trajectories.add_trajectory();
    st_planner_traj_proto->set_reason(st_planner_traj_info.reason);
    st_planner_traj_proto->set_id(st_planner_traj_info.object_id);
    st_planner_traj_proto->set_index(st_planner_traj_info.traj_index);
  }
  debug_info->st_planner_object_trajectories =
      st_output->st_planner_object_trajectories;

  debug_info->filtered_prediction_trajectories.mutable_filtered()->Reserve(
      input.st_traj_mgr->ignored_trajectories().size());
  for (const auto& ignored : input.st_traj_mgr->ignored_trajectories()) {
    auto* filtered =
        debug_info->filtered_prediction_trajectories.add_filtered();
    filtered->set_reason(ignored.reason);
    filtered->set_id(ignored.object_id);
    filtered->set_index(ignored.traj->index());
  }

  st_output->decision_exploration = std::move(path_output.decision_exploration);
  std::move(fill_decision_exploration).Cancel();

  if (!path_status.ok()) {
    FillDecisionConstraintDebugInfo(path_output.constraint_manager,
                                    &debug_info->decision_constraints);
    return path_status;
  }

  st_output->path = std::move(path_output.path);
  st_output->leading_trajs = std::move(path_output.leading_trajs);
  st_output->follower_set = std::move(path_output.follower_set);
  st_output->leader_set = std::move(path_output.leader_set);
  st_output->follower_max_decel = path_output.follower_max_decel;

  for (auto it = st_output->leading_trajs.begin();
       it != st_output->leading_trajs.end();) {
    if (it->second.reason() ==
        ConstraintProto::LeadingObjectProto::AFTER_STOPLINE) {
      it = st_output->leading_trajs.erase(it);
    } else {
      ++it;
    }
  }

  const auto& traffic_gap = path_output.constraint_manager.TrafficGap();
  auto speed_planning_params = *input.speed_planning_params;
  if (FLAGS_planner_enable_lc_style_params) {
    ModifySpeedPlanningParamsStyle(
        *input.speed_planning_lc_radical_params,
        *input.speed_planning_lc_conservative_params, traffic_gap,
        st_output->decision_exploration.lane_change_state.stage(),
        path_output.speed_response_style, &speed_planning_params);
  }

  if (input.behavior) {
    PiecewiseLinearFunction<double> headway_speed_buffer_plf(
        PiecewiseLinearFunctionFromProto(
            speed_planning_params.headway_speed_buffer_plf()));
    const double dynamic_headway = input.behavior->dynamic_headway();
    speed_planning_params.set_follow_time_headway(
        dynamic_headway + headway_speed_buffer_plf(Mps2Kph(
                              input.start_point_info->start_point.v())));
    speed_planning_params.set_large_vehicle_follow_time_headway(
        dynamic_headway +
        headway_speed_buffer_plf(
            Mps2Kph(input.start_point_info->start_point.v())) +
        0.2);
  }

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      ExecuteSpeedPlanning(
          {.base_name = "st",
           .behavior = input.behavior,
           .psmm = input.planner_semantic_map_manager,
           .traj_mgr = input.st_traj_mgr,
           .constraint_mgr = &path_output.constraint_manager,
           .leading_trajs = &st_output->leading_trajs,
           .follower_set = &st_output->follower_set,
           .leader_set = &st_output->leader_set,
           .consider_lane_change_gap = input.consider_lane_change_gap,
           .plan_passage = &st_output->decision_exploration.plan_passage,
           .path_sl_boundary = &st_output->decision_exploration.sl_boundary,
           .stalled_objects = input.stalled_objects,
           .path = &st_output->path,
           .st_path_points = &st_output->st_path_points,
           .plan_start_v = input.start_point_info->start_point.v(),
           .plan_start_a = input.start_point_info->start_point.a(),
           .plan_start_j = input.start_point_info->start_point.j(),
           .plan_time = input.start_point_info->plan_time,
           .plan_id = input.plan_id,
           .lc_stage =
               st_output->decision_exploration.lane_change_state.stage(),
           .lane_change_state =
               st_output->decision_exploration.lane_change_state,
           .attention_obj_id = input.speed_state->attention_obj_id,
           .nudge_object_info = input.nudge_object_info,
           .is_open_gap = input.is_open_gap,
           .ego_history = input.ego_history,
           .seq_num = input.seq_num,
           .agent_status_history = input.agent_status_history},
          vehicle_geom_params, vehicle_drive_params,
          spacetime_constraint_params, speed_planning_params, thread_pool,
          &st_output->curr_ego_frame),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);

  debug_info->speed_planning_debug =
      std::move(speed_output.speed_planning_proto);
  FillDecisionConstraintDebugInfo(speed_output.constraint_mgr,
                                  &debug_info->decision_constraints);

  st_output->traj_points = std::move(speed_output.trajectory_points);
  st_output->considered_st_objects =
      std::move(speed_output.considered_st_objects);
  st_output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  agent_status_history = std::move(speed_output.agent_status_history);
  st_output->alerted_front_vehicle =
      std::move(speed_output.alerted_front_vehicle);
  st_output->st_boundaries_with_decision =
      std::move(speed_output.st_boundaries_with_decision);
  st_output->obj_sl_map = speed_output.obj_sl_map;
  st_output->speed_state.attention_obj_id = speed_output.attention_obj_id;

  const bool valid = ValidateStTrajectory(
      *input.planner_semantic_map_manager, st_output->considered_st_objects,
      input.start_point_info->full_stop, st_output->decision_exploration,
      vehicle_geom_params, vehicle_drive_params, spacetime_constraint_params,
      st_output->traj_points, &debug_info->traj_validation_result, thread_pool);
  if (!valid) {
    return PlannerStatus(
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        absl::StrCat("Validation failed: ",
                     debug_info->traj_validation_result.DebugString()));
  }

  DestroyContainerAsyncMarkSource(std::move(path_output),
                                  "st_planner:path_output");

  return OkPlannerStatus();
}

}  // namespace planning
}  // namespace e2e_noa
