#include "plan/st_path_planner.h"

#include <algorithm>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "async/thread_pool.h"
#include "common/lane_change_safety.h"
#include "common/planner_status.h"
#include "common/timer.h"
#include "decision_exploration/decision_exploration_output.h"
#include "descriptor/descriptor_output.h"
#include "glog/logging.h"
#include "lane_change.pb.h"
#include "maps/lane_point.h"
#include "maps/lane_sequence.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_frame.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "optimization/ddp/trajectory_optimizer.h"
#include "optimization/ddp/trajectory_optimizer_input.h"
#include "optimization/ddp/trajectory_optimizer_output.h"
#include "optimization/ddp/trajectory_optimizer_util.h"
#include "optimizer.pb.h"
#include "plan/planner_defs.h"
#include "plan/planner_flags.h"
#include "plan/short_path_check.h"
#include "plan/trajectory_point.h"
#include "plan/trajectory_util.h"
#include "planner_object.pb.h"
#include "planner_status.pb.h"
#include "router/plan_passage.h"
#include "spacetime_search/search_spacetime.h"
#include "spacetime_search/spacetime_search_input.h"
#include "spacetime_search/spacetime_search_output.h"
#include "util/file_util.h"
#include "util/lane_path_util.h"
#include "util/planner_semantic_map_util.h"
#include "util/planner_status_macros.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa::planning {

namespace {

void UpdateTrajOptParams(
    const std::string& traj_opt_params_address,
    TrajectoryOptimizerParamsProto* trajectory_optimizer_params) {
  VLOG(3) << "Trajectory optimizer params before update: ";
  VLOG(3) << trajectory_optimizer_params->DebugString();
  if (!file_util::TextFileToProto(traj_opt_params_address,
                                  trajectory_optimizer_params)) {
    CHECK(false) << "Read trajectory optimizer params as text file failed!!!!";
  }
  LOG(INFO) << "New trajectory optimizer params are used.";
}

std::vector<ApolloTrajectoryPointProto> ResampleOptimizerTrajectory(
    const std::vector<ApolloTrajectoryPointProto>& opt_traj) {
  std::vector<ApolloTrajectoryPointProto> resampled(kTrajectorySteps);
  for (int i = 0; i < kTrajectorySteps; ++i) {
    resampled[i] = QueryApolloTrajectoryPointByT(
        opt_traj.begin(), opt_traj.end(), i * kTrajectoryTimeStep);
  }
  return resampled;
}

std::vector<ApolloTrajectoryPointProto> StitchPreviousTrajectoryAndStTrajectory(
    int stitch_index,
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    const std::vector<ApolloTrajectoryPointProto>& st_trajectory) {
  if (time_aligned_prev_traj.empty() || stitch_index <= 0) {
    return st_trajectory;
  }
  std::vector<ApolloTrajectoryPointProto> res_traj;
  res_traj.reserve(stitch_index + st_trajectory.size());
  for (int idx = 0; idx < stitch_index; ++idx) {
    res_traj.push_back(time_aligned_prev_traj[idx]);
  }
  const double s_offset = time_aligned_prev_traj[stitch_index].path_point().s();
  for (const auto& point : st_trajectory) {
    res_traj.push_back(point);
    res_traj.back().mutable_path_point()->set_s(point.path_point().s() +
                                                s_offset);
  }
  return res_traj;
}

absl::Status AddSpaceTimePlannerTrajectoryById(
    const SpacetimeTrajectoryManager& traj_mgr, absl::string_view traj_id,
    SpacetimePlannerObjectTrajectoryReason::Type reason,
    SpacetimePlannerObjectTrajectories* res) {
  if (!res->trajectory_ids.contains(traj_id)) {
    const auto* traj =
        traj_mgr.FindTrajectoryById({traj_id.data(), traj_id.size()});
    if (traj == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("Could not find trajectory ", traj_id));
    }
    ASSIGN_OR_RETURN(
        auto truncated_traj,
        traj->CreateTruncatedCopy(res->st_start_offset,
                                  kSpacetimePlannerTrajectoryHorizon));
    res->trajectories.push_back(std::move(truncated_traj));
    res->trajectory_infos.push_back(
        {.traj_index = traj->traj_index(),
         .object_id = traj->planner_object().is_sim_agent()
                          ? traj->planner_object().base_id()
                          : traj->planner_object().id(),
         .reason = reason});
    res->trajectory_ids.insert(std::string(traj_id));
  }
  return absl::OkStatus();
}

void ModifyTrajOptParamsStyle(
    const TrajectoryOptimizerParamsProto&
        trajectory_optimizer_lc_radical_params,
    const TrajectoryOptimizerParamsProto& trajectory_optimizer_lc_normal_params,
    const TrajectoryOptimizerParamsProto&
        trajectory_optimizer_lc_conservative_params,
    LaneChangeStage lc_stage, LaneChangeStyle lc_style,
    TrajectoryOptimizerParamsProto* trajectory_optimizer_params) {
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    switch (lc_style) {
      case LC_STYLE_NORMAL:
        *trajectory_optimizer_params = trajectory_optimizer_lc_normal_params;
        break;
      case LC_STYLE_RADICAL:
        *trajectory_optimizer_params = trajectory_optimizer_lc_radical_params;
        break;
      case LC_STYLE_CONSERVATIVE:
        *trajectory_optimizer_params =
            trajectory_optimizer_lc_conservative_params;
        break;
    }
  }
}

void ModifySLBoundaryByDPPath(
    const PlanPassage& passage, const FrenetBox& ego_frenet_box,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const std::vector<TrajectoryPoint>& dp_traj, PathSlBoundary* sl_boundary) {
  CHECK_NOTNULL(sl_boundary);

  std::vector<FrenetCoordinate> dp_frenet_points;
  dp_frenet_points.reserve(dp_traj.size());
  for (const auto& dp_pt : dp_traj) {
    const auto frenet_pt =
        passage.QueryLaterallyUnboundedFrenetCoordinateAt(dp_pt.pos());
    if (!frenet_pt.ok()) return;
    dp_frenet_points.emplace_back(*frenet_pt);
  }

  const double front_to_rear_axle = 4.0;
  const double rear_to_rear_axle = 1.1;
  const double half_ego_width = 1.0;
  for (const auto& trajectory : st_planner_object_traj.trajectories) {
    if (trajectory.is_stationary()) continue;
    ObjectType object_type = trajectory.object_type();

    const auto object_frenet_box =
        passage.QueryFrenetBoxAtContour(trajectory.planner_object().contour());
    if (!object_frenet_box.ok()) {
      continue;
    }
    if (object_frenet_box->s_max < ego_frenet_box.s_min ||
        object_frenet_box->s_min > sl_boundary->end_s()) {
      continue;
    }

    if (object_type == OT_MOTORCYCLIST || object_type == OT_TRICYCLIST ||
        object_type == OT_CYCLIST || object_type == OT_PEDESTRIAN) {
      double ob_l_min = DBL_MAX, ob_l_max = -DBL_MAX;
      double s_catchup = 0.0, s_overtake = 0.0;
      bool is_bypass = true, is_catchup = false;
      int bypass_direction = 0, idx_dp_pt = 0;
      for (const auto& state : trajectory.states()) {
        const auto frenet_box = passage.QueryFrenetBoxAt(state.box);
        if (!frenet_box.ok() || !state.traj_point) continue;

        const double ob_traj_t = state.traj_point->t();

        while (idx_dp_pt < dp_traj.size()) {
          if (dp_traj[idx_dp_pt].t() > ob_traj_t) break;
          idx_dp_pt++;
        }
        if (idx_dp_pt >= dp_traj.size()) break;
        double s_dp_align = dp_frenet_points[idx_dp_pt].s;
        double l_dp_align = dp_frenet_points[idx_dp_pt].l;
        if (idx_dp_pt > 0) {
          double t_factor =
              (dp_traj[idx_dp_pt].t() - ob_traj_t) /
              (dp_traj[idx_dp_pt].t() - dp_traj[idx_dp_pt - 1].t());
          l_dp_align -= (dp_frenet_points[idx_dp_pt].l -
                         dp_frenet_points[idx_dp_pt - 1].l) *
                        t_factor;
          s_dp_align -= (dp_frenet_points[idx_dp_pt].s -
                         dp_frenet_points[idx_dp_pt - 1].s) *
                        t_factor;
        }

        if (s_dp_align - rear_to_rear_axle > frenet_box->s_max) {
          s_overtake = s_dp_align + 1.0 * dp_traj[idx_dp_pt].v();
          break;
        }
        if (s_dp_align + front_to_rear_axle > frenet_box->s_min) {
          if (!is_catchup) {
            s_catchup = std::fmax(s_dp_align - 1.0 * dp_traj[idx_dp_pt].v(),
                                  object_frenet_box->s_min);
            s_overtake = s_dp_align + 1.0 * dp_traj[idx_dp_pt].v();
            is_catchup = true;
          }
          if (l_dp_align - half_ego_width > frenet_box->l_max &&
              object_frenet_box->center_l() < ego_frenet_box.center_l()) {
            ob_l_max = object_frenet_box->l_max;
            bypass_direction = 1;
          } else if (l_dp_align + half_ego_width < frenet_box->l_min &&
                     object_frenet_box->center_l() >
                         ego_frenet_box.center_l()) {
            ob_l_min = object_frenet_box->l_min;
            bypass_direction = -1;
          } else {
            is_bypass = false;
            break;
          }
        }
      }

      if (is_catchup && is_bypass) {
        const double bound_buffer = 0.1;
        if (bypass_direction == 1) {
          sl_boundary->ModifyHardBoundByDpLabel(passage, s_catchup, s_overtake,
                                                ob_l_max + bound_buffer, true);
        }
      }
    }
  }
}

bool IfMissNaviScenario(
    const DecisionExplorationOutput& decision_exploration_result, Vec2d ego_pos,
    int plan_id) {
  auto lane_seq_info = decision_exploration_result.plan_passage.lane_seq_info();
  if (!lane_seq_info) {
    return false;
  }
  if (lane_seq_info->dist_to_junction < 5.0) {
    return false;
  }
  if (lane_seq_info->lc_num != 1 && lane_seq_info->lc_num != 2) {
    return false;
  }
  if (lane_seq_info->lc_num == 1 && lane_seq_info->dist_to_navi_end < 5.0) {
    return false;
  }
  if (lane_seq_info->lc_num == 2 && lane_seq_info->dist_to_navi_end < 10.0) {
    return false;
  }
  if (lane_seq_info->dist_to_navi_end > 150.0) {
    return false;
  }
  if (lane_seq_info->dist_to_navi_end > lane_seq_info->dist_to_junction + 5.0) {
    return false;
  }
  auto tgt_lane_seq = lane_seq_info->lane_seq;
  if (!tgt_lane_seq) {
    return false;
  }
  auto nearest_lane = tgt_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
  if (!nearest_lane) {
    return false;
  }
  if (nearest_lane->type() != ad_e2e::planning::LANE_NORMAL &&
      nearest_lane->type() != ad_e2e::planning::LANE_BUS_NORMAL &&
      nearest_lane->type() != ad_e2e::planning::LANE_HOV_NORMAL) {
    return false;
  }
  if (decision_exploration_result.lane_change_state.stage() == LCS_WAITING ||
      decision_exploration_result.lane_change_state.stage() == LCS_NONE ||
      decision_exploration_result.lane_change_state.stage() == LCS_PAUSE ||
      decision_exploration_result.lane_change_state.stage() == LCS_EXECUTING) {
    return true;
  }

  return false;
}

bool GetDistanceToNearestNonSolidLineFromLaneSeqNaviEnd(
    const Vec2d& start_point,
    const ad_e2e::planning::LaneSeqInfoPtr& target_lane_seq_info, bool is_left,
    double& result_distance) {
  result_distance = std::numeric_limits<double>::infinity();
  if (!target_lane_seq_info || !target_lane_seq_info->lane_seq) {
    return false;
  }
  const double dis_to_navi_end = target_lane_seq_info->dist_to_navi_end;
  if (dis_to_navi_end > std::numeric_limits<double>::infinity() - 0.1) {
    return false;
  }
  auto target_lane_seq = target_lane_seq_info->lane_seq;
  const auto nearest_lane = target_lane_seq_info->nearest_lane;
  if (!nearest_lane) {
    return false;
  }
  if (!target_lane_seq_info->navi_end_lane) {
    return false;
  }

  double start_point_offset_to_seq_start = 0.0;
  double temp_proj_ego = target_lane_seq->GetProjectionDistance(
      {start_point.x(), start_point.y()}, &start_point_offset_to_seq_start);
  std::vector<ad_e2e::planning::LaneBoundaryType> lane_boundary_types;
  bool has_found_poi = false;

  for (auto target_lane = target_lane_seq->lanes().rbegin();
       target_lane != target_lane_seq->lanes().rend(); target_lane++) {
    lane_boundary_types.clear();

    if (target_lane->get()->id() == target_lane_seq_info->navi_end_lane->id()) {
      has_found_poi = true;
    }
    if (!has_found_poi) {
      continue;
    }

    auto lane_start_point = target_lane->get()->center_line().begin_point();
    double begin_point_offset_to_seq_start = 0.0;
    double temp_proj_lane = target_lane_seq->GetProjectionDistance(
        lane_start_point, &begin_point_offset_to_seq_start);
    const auto lane_boundary = is_left ? target_lane->get()->left_boundary()
                                       : target_lane->get()->right_boundary();
    if (!lane_boundary) {
      continue;
    }
    lane_boundary_types = lane_boundary->boundary_types();
    for (auto boundary_type = lane_boundary_types.rbegin();
         boundary_type != lane_boundary_types.rend(); boundary_type++) {
      auto line_type = (*boundary_type).line_type;

      if (line_type != ad_e2e::planning::SOLID &&
          line_type != ad_e2e::planning::SOLID_SOLID &&
          line_type != ad_e2e::planning::SHADED_AREA) {
        result_distance = begin_point_offset_to_seq_start -
                          start_point_offset_to_seq_start + (*boundary_type).s;
        return true;
      }
    }

    if (target_lane->get()->id() == nearest_lane->id()) {
      break;
    }
  }
  return false;
}

}  // namespace

PlannerStatus RunStPathPlanner(StPathPlannerInput input,
                               StPathPlannerOutput* out,
                               WorkerThreadManager* thread_pool) {
  Timer timer(__FUNCTION__);
  CHECK_NOTNULL(input.st_path_start_point_info);
  CHECK_NOTNULL(input.vehicle_params);
  CHECK_NOTNULL(input.planner_semantic_map_manager);
  CHECK_NOTNULL(input.stalled_objects);
  CHECK_NOTNULL(input.traj_mgr);
  CHECK_NOTNULL(input.prev_target_lane_path_from_start);
  CHECK_NOTNULL(input.time_aligned_prev_traj);
  CHECK_NOTNULL(input.prev_initialization_state);

  CHECK_NOTNULL(input.start_point_info);
  CHECK_NOTNULL(input.obj_mgr);

  CHECK_NOTNULL(input.prev_decision_state);

  CHECK_NOTNULL(input.decision_constraint_config);
  CHECK_NOTNULL(input.initialization_params);
  CHECK_NOTNULL(input.trajectory_optimizer_params);
  CHECK_NOTNULL(input.spacetime_constraint_params);
  CHECK_NOTNULL(input.planner_functions_params);
  CHECK_NOTNULL(input.vehicle_models_params);
  CHECK_NOTNULL(input.trajectory_optimizer_lc_radical_params);
  CHECK_NOTNULL(input.trajectory_optimizer_lc_normal_params);
  CHECK_NOTNULL(input.trajectory_optimizer_lc_conservative_params);
  CHECK_NOTNULL(input.obs_history);
  CHECK_NOTNULL(input.behavior);
  CHECK_NOTNULL(input.speed_state);

  const auto& psmm = *input.planner_semantic_map_manager;
  const auto& vehicle_params = *input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();
  const auto& map_func_id = input.behavior->function_id();

  out->st_planner_object_traj = std::move(input.init_st_planner_object_traj);

  auto lc_reason = LaneChangeReason::NO_CHANGE;
  if (input.lc_cmd_state == DriverAction::LC_CMD_LEFT ||
      input.lc_cmd_state == DriverAction::LC_CMD_RIGHT) {
    lc_reason = LaneChangeReason::MANUAL_CHANGE;
  } else {
    lc_reason = LaneChangeReason::DEFAULT_CHANGE;
  }

  InitializationInput initialization_input{
      .planner_semantic_map_manager = &psmm,
      .path_start_point_info = input.st_path_start_point_info,
      .path_look_ahead_duration = input.path_look_ahead_duration,
      .lane_change_state = &input.decision_exploration.lane_change_state,
      .prev_lc_stage = input.prev_lane_change_stage,
      .lane_change_style = input.lane_change_style,
      .stalled_objects = input.stalled_objects,
      .plan_passage = &input.decision_exploration.plan_passage,
      .st_traj_mgr = input.traj_mgr,
      .sl_boundary = &input.decision_exploration.sl_boundary,
      .prev_initialization_state = input.prev_initialization_state,
      .decision_constraint_config = input.decision_constraint_config,
      .initialization_params = input.initialization_params,
      .spacetime_constraint_params = input.spacetime_constraint_params,
      .vehicle_params = input.vehicle_params,
      .st_planner_object_traj = &out->st_planner_object_traj,
      .plan_id = input.plan_id,
      .scene_reasoning = input.scene_reasoning,
      .borrow_lane = input.decision_exploration.borrow_lane,
      .av_frenet_box =
          &input.decision_exploration.av_frenet_box_on_plan_passage,

      .start_point_info = input.start_point_info,
      .route_target_info = input.route_target_info,
      .smooth_result_map = input.smooth_result_map,
      .obj_mgr = input.obj_mgr,
      .traffic_light_status_map = input.traffic_light_status_map,
      .prev_target_lane_path_from_start =
          input.prev_target_lane_path_from_start,
      .prev_decision_state = input.prev_decision_state,

      .obs_history = input.obs_history,
      .behavior = input.behavior,
      .speed_state = input.speed_state,
      .cur_lc_num = input.cur_navi_lc_num,
      .left_navi_dist = input.left_navi_dist,
      .push_dir = input.push_dir,
      .nudge_object_info = input.nudge_object_info,
      .lc_reason = lc_reason

  };

  absl::flat_hash_set<std::string> unsafe_object_ids;
  auto traj_initialization_output_or = RunTrajectoryInitialization(
      initialization_input, &unsafe_object_ids, &input.decision_exploration,
      &input.decision_output, &out->initialization_debug_proto, thread_pool,
      &out->obs_leading, &out->lc_status_code);

  const auto& plan_start_point = input.start_point_info->start_point;
  const Vec2d ego_pos = Extract2dVectorFromApolloProto(plan_start_point);
  bool ifmissnavi = false;
  if (input.miss_navi_scenario) {
    ifmissnavi = true;

    auto lane_seq_info =
        input.decision_exploration.plan_passage.lane_seq_info();
    if (lane_seq_info) {
      auto lane_seq = lane_seq_info->lane_seq;
      int navi_lc_num = lane_seq_info->lc_num;

      bool is_left =
          lane_seq_info->navi_lc_command ==
                  ad_e2e::planning::BehaviorCommand::Command_LaneChangeLeft
              ? true
              : false;
      double dis_to_non_solidline = std::numeric_limits<double>::infinity();
      bool get_result = GetDistanceToNearestNonSolidLineFromLaneSeqNaviEnd(
          ego_pos, lane_seq_info, is_left, dis_to_non_solidline);
      double dist_navi_end_on_seq = lane_seq_info->dist_to_navi_end;
      double navi_end_to_non_solidline =
          dist_navi_end_on_seq - dis_to_non_solidline;
      if (map_func_id == Behavior_FunctionId_MAPLESS_NOA) {
        navi_end_to_non_solidline = 50.0;
      }

      double navi_end_to_stopline = navi_end_to_non_solidline;
      navi_end_to_stopline = std::clamp(navi_end_to_stopline, 10.0, 60.0);

      double min_dis_to_stopline =
          lane_seq_info->dist_to_junction - navi_end_to_stopline;
      min_dis_to_stopline = std::fmax(0.0, min_dis_to_stopline);
      min_dis_to_stopline = std::min(120.0, min_dis_to_stopline);
      bool anti_heavy_brake = false;
      double reaction_distance = plan_start_point.v() * 0.3;
      double front_to_rear_axle = 4.0;

      double distance =
          min_dis_to_stopline - reaction_distance - front_to_rear_axle;
      double brake_a = 1.0 * plan_start_point.v() * plan_start_point.v() / 2.0 /
                       std::max(distance, 0.5);
      if (brake_a > 2.5) {
        anti_heavy_brake = true;
      }

      double check_range = 60.0;
      const auto& passage = input.decision_exploration.plan_passage;
      double start_s = min_dis_to_stopline;
      double end_s = std::min(min_dis_to_stopline + 10.0, passage.end_s());
      start_s = std::min(start_s, passage.end_s());
      const auto start_point = passage.QueryPointXYAtS(start_s);
      const auto end_point = passage.QueryPointXYAtS(end_s);
      if (!start_point.ok())
        return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                             start_point.status().message());
      if (!end_point.ok())
        return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                             end_point.status().message());

      if (min_dis_to_stopline < check_range &&
          (navi_lc_num == 1 || navi_lc_num == 2) &&
          lane_seq_info->dist_to_junction < 100.0 &&
          input.decision_exploration.lane_change_state.stage() !=
              LCS_EXECUTING &&
          input.decision_exploration.lane_change_state.stage() != LCS_PAUSE) {
        if (anti_heavy_brake) {
          input.decision_output.constraint_manager.AddVLimit(
              std::max(plan_start_point.v() - 3.0, 0.0), 2.0,
              "miss navi slow dec");
        } else {
          ConstraintProto::StopLineProto stop_line;
          stop_line.set_s(min_dis_to_stopline + passage.lane_path_start_s());
          stop_line.set_standoff(0.0);
          stop_line.set_time(0.0);
          HalfPlane halfplane(start_point.value(), end_point.value());
          halfplane.ToProto(stop_line.mutable_half_plane());
          stop_line.set_id("miss_navi_stop_line");
          stop_line.mutable_source()->mutable_brake_to_stop()->set_reason(
              absl::StrCat("miss navi stop", "1"));
          input.decision_output.constraint_manager.AddStopLine(
              std::move(stop_line));
        }
      }
    }
  }
  out->decision_exploration = std::move(input.decision_exploration);
  out->constraint_manager = std::move(input.decision_output.constraint_manager);
  out->decision_state = std::move(input.decision_output.decision_state);
  out->unsafe_object_ids = std::move(unsafe_object_ids);
  if (!traj_initialization_output_or.ok()) {
    return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                         traj_initialization_output_or.status().message());
  }
  auto initialization_output = std::move(traj_initialization_output_or).value();
  out->leading_trajs = std::move(initialization_output.leading_trajs);
  out->lc_status_code = std::move(initialization_output.lc_status_code);
  out->is_init_follow_scene = initialization_output.is_init_follow_scene;
  out->lc_lead_obj_id = initialization_output.lc_lead_obj_id;
  out->speed_response_style =
      std::move(initialization_output.speed_response_style);

  bool if_can_cancel_lc = true;
  const auto lat_offset =
      out->decision_exploration.plan_passage.QueryFrenetLatOffsetAt(ego_pos);
  if (!lat_offset.ok())
    return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                         lat_offset.status().message());
  if (std::abs(lat_offset.value()) < 2.4) {
    if_can_cancel_lc = false;
  }

  if (!ifmissnavi &&
      (out->decision_exploration.lane_change_state.stage() == LCS_EXECUTING ||
       out->decision_exploration.lane_change_state.stage() == LCS_PAUSE) &&
      input.prev_lane_change_stage != LCS_EXECUTING) {
    const auto lane_seq_info =
        out->decision_exploration.plan_passage.lane_seq_info();
    if (lane_seq_info->nearest_lane->type() == ad_e2e::planning::LANE_UNKNOWN ||
        lane_seq_info->nearest_lane->type() ==
            ad_e2e::planning::LANE_NON_MOTOR) {
      return PlannerStatus(PlannerStatusProto::LC_SPECIAL_LANE_CHECK_FAILED,
                           "Unable to lane change for unknown/non-motor lane");
    }
    double dist_to_navi_end = lane_seq_info->dist_to_navi_end;
    if (map_func_id == Behavior_FunctionId_MAPLESS_NOA) {
      dist_to_navi_end = lane_seq_info->dist_to_navi_end_v2;
    }
    if (out->decision_exploration.pnp_top1_reason !=
            ad_e2e::planning::LC_REASON_FOR_NAVI &&
        dist_to_navi_end > 160.0 && lane_seq_info->lc_num <= 1 &&
        lane_seq_info->dist_to_bus_lane < dist_to_navi_end &&
        lane_seq_info->dist_to_bus_lane <
            std::max(5.0 * plan_start_point.v(), 50.0)) {
      return PlannerStatus(PlannerStatusProto::LC_SPECIAL_LANE_CHECK_FAILED,
                           "Unable to lane change for bus lane");
    }
  }

  if (input.left_navi_dist > 120 && input.left_navi_dist_v2 > 250 &&
      if_can_cancel_lc && !ifmissnavi &&
      (out->decision_exploration.pnp_top1_reason !=
           ad_e2e::planning::LC_REASON_FOR_MERGE ||
       input.lc_cmd_state != DriverAction::LC_CMD_NONE) &&
      out->decision_exploration.lane_change_state.stage() == LCS_EXECUTING &&
      input.prev_lane_change_stage != LCS_EXECUTING &&
      input.prev_lane_change_stage != LCS_PAUSE) {
    const auto lane_seq_info =
        out->decision_exploration.plan_passage.lane_seq_info();
    double dist_to_lc_solidline = std::numeric_limits<double>::infinity();
    if (lane_seq_info) {
      dist_to_lc_solidline =
          out->decision_exploration.lane_change_state.lc_left()
              ? lane_seq_info->dist_to_right_solid_line
              : lane_seq_info->dist_to_left_solid_line;
    } else {
    }
    if ((input.pnp_infos.has_value() &&
         !input.pnp_infos.value().infos().empty() &&
         input.pnp_infos.value().infos()[0].lc_reason() !=
             LcReason::LC_REASON_FOR_AVOID_LANE_BUS) ||
        input.lc_cmd_state != DriverAction::LC_CMD_NONE) {
      if (dist_to_lc_solidline < 15.0 + plan_start_point.v() * 2.0) {
        LOG(WARNING) << absl::StrFormat(
            "DecisionExplorationr branch deleted: Target lane linetype solid");
        return PlannerStatus(PlannerStatusProto::LC_LINE_CHECK_FAILED,
                             "Unable to lane change for solid line");
      }
    }
  }
  auto lane_seq_info_miss_navi =
      out->decision_exploration.plan_passage.lane_seq_info();

  if (map_func_id != Behavior_FunctionId_LKA &&
      map_func_id != Behavior_FunctionId_NONE &&
      input.prev_lane_change_stage == LCS_NONE &&
      input.lc_cmd_state == DriverAction::LC_CMD_NONE &&
      lane_seq_info_miss_navi &&
      lane_seq_info_miss_navi->dist_to_junction < 100.0 &&
      lane_seq_info_miss_navi->dist_to_junction > 1.0 &&
      std::abs(input.left_navi_dist -
               lane_seq_info_miss_navi->dist_to_navi_end) > 10.0 &&
      lane_seq_info_miss_navi->dist_to_navi_end - input.left_navi_dist < 10.0 &&
      input.cur_navi_lc_num < lane_seq_info_miss_navi->lc_num &&
      (out->decision_exploration.lane_change_state.stage() == LCS_EXECUTING ||
       out->decision_exploration.lane_change_state.stage() == LCS_PAUSE)) {
    return PlannerStatus(PlannerStatusProto::LC_MISS_NAVI_FAILED,
                         "Unable to lane change miss navi");
  }

  if (out->decision_exploration.lane_change_state.stage() ==
      LaneChangeStage::LCS_PAUSE) {
    const auto start_lane_id =
        out->decision_exploration.plan_passage.lane_path().front().lane_id();
    if (input.prev_target_lane_path_from_start->IsEmpty() ||
        input.prev_target_lane_path_from_start->front().lane_id() !=
            start_lane_id) {
      LOG(WARNING) << absl::StrFormat(
          "DecisionExplorationr branch deleted: Target lane %d not safe",
          start_lane_id);
      return PlannerStatus(
          PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
          absl::StrCat("Unsafe to initiate lane change to ",
                       out->decision_exploration.plan_passage.lane_path()
                           .front()
                           .lane_id()));
    }
  }

  out->follower_set = std::move(initialization_output.follower_set);
  out->leader_set = std::move(initialization_output.leader_set);
  out->follower_max_decel = initialization_output.follower_max_decel;

  for (const auto& [traj_id, _] : out->leading_trajs) {
    const auto status = AddSpaceTimePlannerTrajectoryById(
        *input.traj_mgr, traj_id,
        SpacetimePlannerObjectTrajectoryReason::LEADING,
        &out->st_planner_object_traj);
    VLOG_IF(3, !status.ok()) << status.ToString();
  }
  out->initialization_state =
      std::move(initialization_output.initialization_state);

  auto mutable_trajectory_optimizer_params = *input.trajectory_optimizer_params;

  if (FLAGS_planner_enable_lc_style_params) {
    ModifyTrajOptParamsStyle(
        *input.trajectory_optimizer_lc_radical_params,
        *input.trajectory_optimizer_lc_normal_params,
        *input.trajectory_optimizer_lc_conservative_params,
        out->decision_exploration.lane_change_state.stage(),
        input.lane_change_style, &mutable_trajectory_optimizer_params);
    auto lc_style_debug_string = absl::StrCat(
        "lc_style:", input.lane_change_style,
        ",lc_stage:", out->decision_exploration.lane_change_state.stage());
    Log2FG::LogDataV0("_task_" + std::to_string(input.plan_id) + "_lc_style",
                      lc_style_debug_string);
  }

  VLOG(3) << "Actual cost weights used in trajectory optimizer: ";
  VLOG(3)
      << mutable_trajectory_optimizer_params.cost_weight_params().DebugString();

  const std::vector<ApolloTrajectoryPointProto> previous_trajectory =
      OffsetTrajectoryTemporally(
          input.st_path_start_point_info->relative_index_from_plan_start_point *
              kTrajectoryTimeStep,
          *input.time_aligned_prev_traj,
          input.spacetime_constraint_params->max_accel_jerk(),
          input.spacetime_constraint_params->max_decel_jerk());

  std::optional<TrajectoryOptimizerState> trajectory_optimizer_state;
  if (input.trajectory_optimizer_state_proto != nullptr) {
    trajectory_optimizer_state.emplace(*input.trajectory_optimizer_state_proto);
  }

  TrajectoryOptimizerInput opt_input{
      .trajectory = initialization_output.traj_points,
      .nudge_info = initialization_output.nudge_info,
      .previous_trajectory = previous_trajectory,
      .trajectory_optimizer_state = std::move(trajectory_optimizer_state),
      .st_traj_mgr = input.traj_mgr,
      .st_planner_object_traj = &out->st_planner_object_traj,
      .plan_passage = &out->decision_exploration.plan_passage,
      .path_sl_boundary = &out->decision_exploration.sl_boundary,
      .constraint_mgr = &out->constraint_manager,
      .leading_trajs = &out->leading_trajs,
      .planner_semantic_map_mgr = &psmm,
      .plan_start_point = input.st_path_start_point_info->start_point,
      .plan_start_time = input.st_path_start_point_info->plan_time,
      .plan_id = input.plan_id,

      .borrow_lane = input.decision_exploration.borrow_lane,
      .lc_stage = out->decision_exploration.lane_change_state.stage(),
      .push_dir = input.push_dir,
      .trajectory_optimizer_params = &mutable_trajectory_optimizer_params,
      .spacetime_constraint_params = input.spacetime_constraint_params,
      .planner_functions_params = input.planner_functions_params,
      .vehicle_models_params = input.vehicle_models_params,
      .veh_geo_params = &vehicle_geom_params,
      .veh_drive_params = &vehicle_drive_params,
      .nudge_object_info = input.nudge_object_info};
  const bool is_compare_weight = false;
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto opt_output,
      OptimizeTrajectory(opt_input, &out->optimizer_debug_proto,
                         is_compare_weight, thread_pool),
      PlannerStatusProto::OPTIMIZER_FAILED);

  out->trajectory_optimizer_state_proto =
      opt_output.trajectory_optimizer_state.ToProto();

  out->nudge_object_info = std::move(opt_output.nudge_object_info);

  CHECK(!opt_output.trajectory_proto.empty());

  if (FLAGS_planner_est_decision_exploration_seperate_lc_pause &&
      (out->decision_exploration.lane_change_state.stage() ==
           LaneChangeStage::LCS_EXECUTING ||
       out->decision_exploration.lane_change_state.stage() ==
           LaneChangeStage::LCS_RETURN)) {
    const auto target_lane_path_ext = BackwardExtendLanePath(
        psmm,
        out->decision_exploration.plan_passage.extend_lane_path()
            .BeforeArclength(kLaneChangeCheckForwardLength),
        kLaneChangeCheckBackwardLength);
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        const auto target_frenet_frame,
        BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path_ext),
                               true),
        PlannerStatusProto::LC_SAFETY_CHECK_FAILED);

    constexpr double kLaneSpeedLimitPreviewTime = 6.0;
    const double speed_limit = psmm.QueryLaneSpeedLimitById(
        out->decision_exploration.plan_passage.lane_path()
            .ArclengthToLanePoint(
                input.st_path_start_point_info->start_point.v() *
                kLaneSpeedLimitPreviewTime)
            .lane_id());

    const auto resampled_traj =
        ResampleOptimizerTrajectory(opt_output.trajectory_proto);

    TrajEvalInfo eval_info{};

    auto lc_safety_status = CheckLaneChangeSafety(
        plan_start_point, resampled_traj, LeadingTrajs{}, target_frenet_frame,
        speed_limit, *input.traj_mgr, vehicle_geom_params,
        input.lane_change_style,
        input.decision_exploration.lane_change_state.stage(),
        input.prev_lane_change_stage, input.path_look_ahead_duration,
        &eval_info);
    if (!lc_safety_status.ok()) {
      return PlannerStatus(
          PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
          absl::StrCat("Lane change to ",
                       out->decision_exploration.plan_passage.lane_path()
                           .front()
                           .lane_id(),
                       " not safe: ", lc_safety_status.message()));
    } else {
      out->follower_set = eval_info.follower_set;
      out->leader_set = eval_info.leader_set;
      out->follower_max_decel = eval_info.follower_max_decel;
      out->unsafe_object_ids = {eval_info.unsafe_object_id};
    }
  }

  std::vector<ApolloTrajectoryPointProto> st_trajectory =
      StitchPreviousTrajectoryAndStTrajectory(
          input.st_path_start_point_info->relative_index_from_plan_start_point,
          *input.time_aligned_prev_traj, opt_output.trajectory_proto);

  constexpr double kRequiredMinPathLength = 20.0;

  constexpr double kCurvatureRelaxFactor = 1.05;
  const double max_curvature = ComputeRelaxedCenterMaxCurvature(
      vehicle_geom_params, vehicle_drive_params);

  RETURN_PLANNER_STATUS_OR_ASSIGN(auto raw_path_points,
                                  ExtendPathAndDeleteUnreasonablePart(
                                      st_trajectory, kRequiredMinPathLength,
                                      kCurvatureRelaxFactor * max_curvature),
                                  PlannerStatusProto::PATH_EXTENSION_FAILED);

  out->path = DiscretizedPath::CreateResampledPath(raw_path_points,
                                                   kPathSampleInterval);
  out->st_path_points = std::move(raw_path_points);

  return OkPlannerStatus();
}

}  // namespace e2e_noa::planning
