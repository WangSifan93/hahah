#include "plan/fallback_planner.h"

#include <map>
#include <optional>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "common/planning_macros.h"
#include "decision_exploration/path_boundary_builder.h"
#include "descriptor/all_descriptor_builder.h"
#include "descriptor/descriptor_input.h"
#include "descriptor/descriptor_output.h"
#include "descriptor/descriptor_util.h"
#include "math/geometry/util.h"
#include "object/low_likelihood_filter.h"
#include "object/plan_passage_filter.h"
#include "object/spacetime_trajectory_manager.h"
#include "object/trajectory_filter.h"
#include "plan/planner_defs.h"
#include "plan/planner_flags.h"
#include "plan/short_path_check.h"
#include "plan/trajectory_validation.h"
#include "planner_status.pb.h"
#include "router/plan_passage.h"
#include "router/plan_passage_builder.h"
#include "speed/speed_planning.h"
#include "speed/speed_planning_input.h"
#include "speed/speed_planning_output.h"
#include "trajectory_validation.pb.h"
#include "util/lane_path_util.h"
#include "util/planner_status_macros.h"
#include "util/vehicle_geometry_util.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

PlannerStatus RunFallbackPlanner(
    const FallbackPlannerInput& input, const VehicleParamsProto& vehicle_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const DecisionConstraintConfigProto& decision_constraint_config,
    const FallbackPlannerParamsProto& fallback_planner_params,
    FallbackPlannerOutput* output, StPlannerDebug* debug,
    WorkerThreadManager* thread_pool) {
  SCOPED_TRACE(__FUNCTION__);

  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.start_point_info);
  CHECK_NOTNULL(input.time_aligned_prev_trajectory);
  CHECK_NOTNULL(input.prev_target_lane_path_from_start);
  CHECK_NOTNULL(input.prev_lane_path_before_lc);

  CHECK_NOTNULL(input.obj_mgr);
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.stalled_objects);

  CHECK_NOTNULL(input.prev_lc_state);

  CHECK_NOTNULL(input.pre_decision_state);

  CHECK_NOTNULL(input.smooth_result_map);
  CHECK_NOTNULL(input.behavior);
  CHECK_NOTNULL(input.speed_state);

  output->plan_id = input.plan_id;
  const auto& time_aligned_prev_trajectory =
      *input.time_aligned_prev_trajectory;
  if (time_aligned_prev_trajectory.size() != kTrajectorySteps) {
    return PlannerStatus(PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE,
                         "Previous trajectory not available.");
  }
  if (input.prev_target_lane_path_from_start->IsEmpty()) {
    return PlannerStatus(PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE,
                         "Previous target lane path not available.");
  }

  const auto& plan_start_point = input.start_point_info->start_point;
  const auto& psmm = *input.psmm;
  const auto& vehicle_geometry_params =
      vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto clamped_prev_target_lane_path_from_start,
      TrimTrailingNotFoundLanes(psmm, *input.prev_target_lane_path_from_start),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  const auto backward_extended_lane_path =
      clamped_prev_target_lane_path_from_start;

  const double planning_horizon = 200.0;
  const mapping::LanePoint destination;
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto plan_passage,
      BuildPlanPassage(psmm, clamped_prev_target_lane_path_from_start,
                       backward_extended_lane_path, *input.station_anchor,
                       planning_horizon, destination, true,
                       input.cruising_speed_limit),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto path_sl_boundary,
      BuildPathBoundaryFromPose(psmm, plan_passage, plan_start_point,
                                vehicle_geometry_params, *input.st_traj_mgr,
                                *input.prev_lc_state, *input.smooth_result_map,
                                false, input.prev_smooth_state),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  const double min_fallback_path_length = 10.0;
  const double max_curvature =
      ComputeCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params);
  RETURN_PLANNER_STATUS_OR_ASSIGN(auto path_extension_output,
                                  ExtendPathAndDeleteUnreasonablePart(
                                      time_aligned_prev_trajectory,
                                      min_fallback_path_length, max_curvature),
                                  PlannerStatusProto::PATH_EXTENSION_FAILED);
  auto extended_path = DiscretizedPath::CreateResampledPath(
      path_extension_output, kPathSampleInterval);

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto ego_frenet_box,
      plan_passage.QueryFrenetBoxAt(ComputeAvBox(
          Extract2dVectorFromApolloProto(plan_start_point),
          plan_start_point.path_point().theta(), vehicle_geometry_params)),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  output->decision_exploration = DecisionExplorationOutput{
      .is_fallback = true,
      .plan_passage = std::move(plan_passage),
      .sl_boundary = std::move(path_sl_boundary),
      .lane_change_state = *input.prev_lc_state,
      .lane_path_before_lc = *input.prev_lane_path_before_lc,
      .length_along_route = input.prev_length_along_route,
      .max_reach_length = input.prev_max_reach_length,
      .av_frenet_box_on_plan_passage = ego_frenet_box};

  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_only_use_most_likely_trajectory);
  const PlanPassageFilter plan_passage_filter(
      &output->decision_exploration.plan_passage,
      &output->decision_exploration.sl_boundary);
  output->filtered_traj_mgr = SpacetimeTrajectoryManager(
      absl::Span<const TrajectoryFilter* const>(
          {&low_likelihood_filter, &plan_passage_filter}),
      input.obj_mgr->planner_objects(), thread_pool);

  DescriptorInput descriptor_input{
      .plan_id = 3,
      .vehicle_geometry_params = &vehicle_geometry_params,
      .spacetime_constraint_params = &spacetime_constraint_params,
      .config = &decision_constraint_config,
      .planner_semantic_map_manager = &psmm,
      .lc_state = input.prev_lc_state,
      .plan_start_point = &plan_start_point,
      .lane_path_before_lc = input.prev_lane_path_before_lc,
      .passage = &output->decision_exploration.plan_passage,
      .sl_boundary = &output->decision_exploration.sl_boundary,
      .obj_mgr = input.obj_mgr,
      .st_traj_mgr = &output->filtered_traj_mgr,

      .traffic_light_status_map = input.traffic_light_status_map,
      .pre_decision_state = input.pre_decision_state,

      .max_reach_length = output->decision_exploration.max_reach_length,
      .lc_num = output->decision_exploration.lc_num,
      .plan_time = input.start_point_info->plan_time,
      .scene_reasoning = input.scene_reasoning,
      .behavior = input.behavior,
      .speed_state = input.speed_state};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decision_output, BuildDescriptorResults(descriptor_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);
  output->decision_state = std::move(decision_output.decision_state);
  output->tl_stop_interface = std::move(decision_output.tl_stop_interface);
  output->speed_state = std::move(decision_output.speed_state);

  const std::map<std::string, ConstraintProto::LeadingObjectProto>
      leading_trajs;
  const absl::flat_hash_set<std::string> follower_set;
  const absl::flat_hash_set<std::string> leader_set;
  std::string attention_id = "";
  if (input.speed_state) {
    attention_id = input.speed_state->attention_obj_id;
  }
  SpeedPlanningInput speed_input{
      .base_name = "st",
      .behavior = input.behavior,
      .psmm = &psmm,
      .traj_mgr = &output->filtered_traj_mgr,
      .constraint_mgr = &decision_output.constraint_manager,
      .leading_trajs = &leading_trajs,
      .follower_set = &follower_set,
      .leader_set = &leader_set,
      .consider_lane_change_gap = true,
      .plan_passage = &output->decision_exploration.plan_passage,
      .path_sl_boundary = &output->decision_exploration.sl_boundary,
      .stalled_objects = input.stalled_objects,
      .path = &extended_path,
      .st_path_points = &path_extension_output,

      .plan_start_v = plan_start_point.v(),
      .plan_start_a = plan_start_point.a(),
      .plan_start_j = plan_start_point.j(),
      .plan_time = input.start_point_info->plan_time,
      .plan_id = input.plan_id,
      .lc_stage = input.prev_lc_state->stage(),
      .attention_obj_id = attention_id,
      .ego_history = input.ego_history};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      ExecuteSpeedPlanning(speed_input, vehicle_geometry_params,
                           vehicle_drive_params, spacetime_constraint_params,
                           fallback_planner_params.speed_planning_params(),
                           thread_pool, &output->curr_ego_frame),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);

  FillDecisionConstraintDebugInfo(speed_output.constraint_mgr,
                                  &debug->decision_constraints);
  debug->speed_planning_debug = std::move(speed_output.speed_planning_proto);
  output->trajectory_points = std::move(speed_output.trajectory_points);
  output->considered_st_objects = std::move(speed_output.considered_st_objects);
  output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  output->path = std::move(extended_path);
  output->st_path_points = std::move(path_extension_output);
  output->st_boundaries_with_decision =
      std::move(speed_output.st_boundaries_with_decision);
  output->speed_state.attention_obj_id = speed_output.attention_obj_id;

  if (!ValidateStTrajectory(
          psmm, output->considered_st_objects,
          input.start_point_info->full_stop, output->decision_exploration,
          vehicle_geometry_params, vehicle_drive_params,
          spacetime_constraint_params, output->trajectory_points,
          &debug->traj_validation_result, thread_pool)) {
    return PlannerStatus(
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        absl::StrCat("Validation failed: ",
                     debug->traj_validation_result.DebugString()));
  }

  return OkPlannerStatus();
}

}  // namespace e2e_noa::planning
