#include "plan/planner_main_loop_internal.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "affine_transformation.pb.h"
#include "common/constants.h"
#include "container/strong_int.h"
#include "glog/logging.h"
#include "maps/lane_point.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_common.h"
#include "math/util.h"
#include "math/vec.h"
#include "plan/discretized_path.h"
#include "plan/planner_defs.h"
#include "plan/planner_flags.h"
#include "plan/planner_util.h"
#include "plan/trajectory_util.h"
#include "router/plan_passage.h"
#include "router/plan_passage_builder.h"
#include "router/route_sections_info.h"
#include "router/route_util.h"
#include "util/autonomy_state_util.h"
#include "util/path_util.h"
#include "util/status_builder.h"
#include "util/status_macros.h"
#include "util/time_util.h"

namespace e2e_noa {
namespace planning {

bool MaybeReset(const ApolloTrajectoryPointProto& pre_reset_planned_point,
                const Vec2d& current_pos, double longitudinal_reset_error,
                double lateral_reset_error, const std::string& planner_name,
                ResetReasonProto::Reason* reset_reason) {
  const Vec2d pos_diff =
      current_pos - Vec2d(pre_reset_planned_point.path_point().x(),
                          pre_reset_planned_point.path_point().y());
  const Vec2d planned_tangent =
      Vec2d::FastUnitFromAngle(pre_reset_planned_point.path_point().theta());

  const double longitudinal_error = std::abs(pos_diff.Dot(planned_tangent));
  if (longitudinal_error > longitudinal_reset_error) {
    LOG(ERROR) << "Resetting due to longitudinal error:" << longitudinal_error;
    *reset_reason = ResetReasonProto::LON_ERROR_TOO_LARGE;
    return true;
  }

  const double lateral_error = std::abs(pos_diff.Dot(planned_tangent.Perp()));
  if (lateral_error > lateral_reset_error) {
    LOG(ERROR) << "Resetting due to lateral error:" << lateral_error;
    *reset_reason = ResetReasonProto::LAT_ERROR_TOO_LARGE;
    return true;
  }
  Log2FG::LogVehData(
      "Error", "longitudinal_error:" + std::to_string(longitudinal_error) +
                   " lateral_error:" + std::to_string(lateral_error));

  return false;
}

std::optional<int> InterpolatePointFromPrevTrajectory(
    absl::Time time, const TrajectoryProto& prev_traj) {
  const double t = ToUnixDoubleSeconds(time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      t > prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      t < prev_traj_start_time) {
    return std::nullopt;
  }

  return RoundToInt((t - prev_traj_start_time) / kTrajectoryTimeStep);
}

bool InterpolatePointFromPrevTrajectoryIncludingPast(
    absl::Time time, const TrajectoryProto& prev_traj,
    ApolloTrajectoryPointProto* point) {
  if (prev_traj.trajectory_point().empty()) return false;
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  const double prev_traj_end_time =
      prev_traj_start_time +
      prev_traj.trajectory_point().rbegin()->relative_time();
  const double prev_traj_begin_time =
      prev_traj.past_points().empty()
          ? prev_traj_start_time +
                prev_traj.trajectory_point().begin()->relative_time()
          : prev_traj_start_time +
                prev_traj.past_points().begin()->relative_time();
  const double t = ToUnixDoubleSeconds(time);
  if (t > prev_traj_end_time || t < prev_traj_begin_time) {
    return false;
  }

  const int relative_time_index =
      RoundToInt((t - prev_traj_start_time) / kTrajectoryTimeStep);
  CHECK_GE(relative_time_index, -prev_traj.past_points_size());
  CHECK_LT(relative_time_index, prev_traj.trajectory_point_size());
  *point = relative_time_index < 0
               ? prev_traj.past_points(relative_time_index +
                                       prev_traj.past_points_size())
               : prev_traj.trajectory_point(relative_time_index);

  return true;
}

bool MaybeResetByAutonomyState(const AutonomyStateProto& prev_autonomy_state,
                               const AutonomyStateProto& now_autonomy_state,
                               ResetReasonProto::Reason* reset_reason) {
  if (!IsAutoDrive(now_autonomy_state.autonomy_state()) &&
      now_autonomy_state.autonomy_state() !=
          AutonomyStateProto::AUTO_STEER_ONLY) {
    *reset_reason = ResetReasonProto::NON_AUTONOMY;
    return true;
  } else if (IS_ENGAGE(prev_autonomy_state.autonomy_state(),
                       now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::FIRST_ENGAGE;
    return true;
  } else if (IsAutoSteerOnlyToAutoDrive(prev_autonomy_state.autonomy_state(),
                                        now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::STEER_ONLY_ENGAGE;
    return true;
  } else if (IsAutoSpeedOnlyToAutoDrive(prev_autonomy_state.autonomy_state(),
                                        now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::SPEED_ONLY_ENGAGE;
    return true;
  } else if (now_autonomy_state.autonomy_state() ==
             AutonomyStateProto::AUTO_STEER_ONLY) {
    *reset_reason = ResetReasonProto::STEER_ONLY;
    return true;
  }

  return false;
}

bool NeedForceResetStPlanner(const TrajectoryProto& prev_trajectory,

                             bool is_emergency_stop, bool rerouted,
                             bool full_stopped,
                             ResetReasonProto::Reason* reset_reason) {
  if (rerouted) {
    *reset_reason = ResetReasonProto::REROUTED;
    return true;
  }

  if (full_stopped) {
    *reset_reason = ResetReasonProto::FULL_STOP;
    return true;
  }

  return false;
}

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByPlanPassage(
    const PlannerSemanticMapManager& psmm, const RouteSections& sections,
    const Vec2d& query_point) {
  ASSIGN_OR_RETURN(const auto nearest_lane_path,
                   FindClosestLanePathOnRouteSectionsToSmoothPoint(
                       psmm, sections, query_point));

  const double step_s = std::min(1.0, 0.5 * nearest_lane_path.length());
  ASSIGN_OR_RETURN(const auto plan_passage,
                   BuildPlanPassageFromLanePath(psmm, nearest_lane_path, step_s,
                                                false, 0.0, 0.0, std::nullopt),
                   _ << "FindSmoothPointOnRouteSectionsByPlanPassage: "
                        "BuildPlanPassageFromLanePath on "
                     << nearest_lane_path.DebugString() << " failed.");

  ASSIGN_OR_RETURN(
      const auto sl, plan_passage.QueryFrenetCoordinateAt(query_point),
      _ << "FindSmoothPointOnRouteSectionsByPlanPassage: Fail to project ego"
           "pos ("
        << query_point.x() << ", " << query_point.y()
        << ") on plan passage from lane path "
        << nearest_lane_path.DebugString());

  const auto start_lane_point = nearest_lane_path.AfterArclength(sl.s).front();
  for (int i = 0; i < sections.size(); ++i) {
    SMM_ASSIGN_SECTION_OR_CONTINUE_ISSUE(section_info, psmm,
                                         sections.section_ids()[i]);

    if (std::find(section_info.lanes().begin(), section_info.lanes().end(),
                  start_lane_point.lane_id()) != section_info.lanes().end()) {
      return PointOnRouteSections{.accum_s = sl.s,
                                  .section_idx = i,
                                  .fraction = start_lane_point.fraction(),
                                  .lane_id = start_lane_point.lane_id()};
    }
  }

  return absl::NotFoundError(
      absl::StrCat("FindSmoothPointOnRouteSectionsByPlanPassage: Point (",
                   query_point.x(), ", ", query_point.y(),
                   ") is not on route sections:", sections.DebugString()));
}

bool MaybeResetStPlanner(
    const ApolloTrajectoryPointProto& pre_reset_planned_point,
    const Vec2d& current_pos, ResetReasonProto::Reason* reset_reason) {
  constexpr double kLongitudinalErrorForReset = 2.0;
  return MaybeReset(
      pre_reset_planned_point, current_pos, kLongitudinalErrorForReset,
      FLAGS_planner_lateral_reset_error, "st_planner", reset_reason);
}

void FillTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto>& planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto>& past_points,
    const mapping::LanePath& target_lane_path_from_current,
    const LaneChangeStateProto& lane_change_state, TurnSignal turn_signal,

    const TrajectoryValidationResultProto& validate_result,
    TrajectoryProto* trajectory) {
  trajectory->set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));
  for (int i = 0; i < planned_trajectory.size(); ++i) {
    *trajectory->add_trajectory_point() = planned_trajectory[i];
  }

  for (const auto& past_point : past_points) {
    *trajectory->add_past_points() = past_point;
  }
  target_lane_path_from_current.ToProto(
      trajectory->mutable_target_lane_path_from_current());
  trajectory->set_turn_signal(turn_signal);

  trajectory->set_lane_change_stage(lane_change_state.stage());
  if (lane_change_state.stage() != LCS_NONE) {
    trajectory->set_lane_change_left(lane_change_state.lc_left());
  }

  trajectory->mutable_traj_validation_result()->CopyFrom(validate_result);
}

std::vector<ApolloTrajectoryPointProto> CreatePreviousTrajectory(
    absl::Time plan_time, const TrajectoryProto& previous_trajectory,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    bool reset) {
  if (previous_trajectory.trajectory_point().empty() || reset) return {};

  const double now_in_sec = ToUnixDoubleSeconds(plan_time);
  const double time_advancement = std::max(
      0.0, now_in_sec - previous_trajectory.trajectory_start_timestamp());
  const std::vector<ApolloTrajectoryPointProto> previous_trajectory_points(
      previous_trajectory.trajectory_point().begin(),
      previous_trajectory.trajectory_point().end());
  return OffsetTrajectoryTemporally(
      time_advancement, previous_trajectory_points,
      spacetime_constraint_params.max_decel_jerk(),
      spacetime_constraint_params.max_accel_jerk());
}

PlanStartPointInfo ComputeStPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state,

    bool rerouted, bool aeb, double front_wheel_angle,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool override) {
  bool reset = false;
  ResetReasonProto::Reason reset_reason = ResetReasonProto::NONE;
  absl::Time plan_start_time = predicted_plan_time;

  std::optional<ApolloTrajectoryPointProto> prev_planned_traj_point =
      std::nullopt;
  double path_s_increment_from_previous_frame = 0.0;
  bool full_stop = false;

  std::optional<int> start_index_on_prev_traj = std::nullopt;

  if (kSimEnable) {
    reset_reason = ResetReasonProto::NON_AUTONOMY;
    reset = true;
  } else {
    reset = MaybeResetByAutonomyState(prev_autonomy_state, now_autonomy_state,
                                      &reset_reason);
  }
  if (!reset) {
    start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
        predicted_plan_time, prev_trajectory);
    if (!start_index_on_prev_traj.has_value()) {
      reset = true;
      reset_reason = ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND;
    } else {
      prev_planned_traj_point = std::make_optional<ApolloTrajectoryPointProto>(
          prev_trajectory.trajectory_point(*start_index_on_prev_traj));
      plan_start_time =
          FromUnixDoubleSeconds(prev_planned_traj_point->relative_time() +
                                prev_trajectory.trajectory_start_timestamp());
      prev_planned_traj_point->set_relative_time(0.0);
      path_s_increment_from_previous_frame =
          prev_planned_traj_point->path_point().s();
      prev_planned_traj_point->mutable_path_point()->set_s(0.0);

      ApolloTrajectoryPointProto prev_planned_now_point;
      if (!InterpolatePointFromPrevTrajectoryIncludingPast(
              FromUnixDoubleSeconds(pose.timestamp()), prev_trajectory,
              &prev_planned_now_point)) {
        reset = true;
        reset_reason = ResetReasonProto::PREV_NOW_POINT_NOT_FOUND;
      } else {
        const Vec2d current_pos(pose.pos_smooth().x(), pose.pos_smooth().y());

        reset = MaybeResetStPlanner(prev_planned_now_point, current_pos,
                                    &reset_reason);
      }
    }
    constexpr double kFullStopSpeedThreshold = 0.05;
    constexpr double kLargeTimeErrorThreshold = 0.5;
    constexpr double kEps = 1e-4;
    const bool time_error_large =
        std::fabs(prev_trajectory.trajectory_start_timestamp() -
                  pose.timestamp()) > kLargeTimeErrorThreshold;
    full_stop = time_error_large && prev_planned_traj_point.has_value() &&
                prev_planned_traj_point->v() < kFullStopSpeedThreshold &&
                std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;

    if (!reset) {
      reset = NeedForceResetStPlanner(prev_trajectory,

                                      aeb, rerouted, full_stop, &reset_reason);
    }
    if ((!reset) && override) {
      reset = true;
      reset_reason = ResetReasonProto::STEER_ONLY;
    }
  }

  if (!reset) {
    CHECK(prev_planned_traj_point.has_value());
  } else {
    start_index_on_prev_traj = std::nullopt;
  }
  ApolloTrajectoryPointProto start_point;
  if (reset && reset_reason == ResetReasonProto::STEER_ONLY) {
    start_point = ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
        prev_trajectory, pose, front_wheel_angle, vehicle_geom_params,
        vehicle_drive_params);
  } else if (reset && reset_reason == ResetReasonProto::LAT_ERROR_TOO_LARGE) {
    start_point = ComputePlanStartPointAfterLateralReset(
        prev_planned_traj_point, pose, front_wheel_angle, vehicle_geom_params,
        vehicle_drive_params);
  } else if (reset) {
    start_point = ComputePlanStartPointAfterReset(
        prev_planned_traj_point, pose, front_wheel_angle,
        spacetime_constraint_params, vehicle_geom_params, vehicle_drive_params,
        true);
  } else {
    start_point = *prev_planned_traj_point;
  }

  return PlanStartPointInfo{
      .reset = reset,
      .start_index_on_prev_traj = start_index_on_prev_traj,
      .start_point = start_point,
      .path_s_increment_from_previous_frame =
          reset ? 0.0 : path_s_increment_from_previous_frame,
      .plan_time =
          reset ? FromUnixDoubleSeconds(pose.timestamp()) : plan_start_time,
      .full_stop = full_stop,
      .reset_reason = reset_reason};
}

absl::Duration GetStPathPlanLookAheadDuration(
    const PlanStartPointInfo& plan_start_point_info, const PoseProto& pose,
    absl::Duration planned_look_ahead_time,
    const TrajectoryProto& previous_trajectory) {
  absl::Duration look_ahead_time = planned_look_ahead_time;

  if (FLAGS_planner_enable_path_start_point_look_ahead &&
      plan_start_point_info.start_index_on_prev_traj.has_value() &&
      !previous_trajectory.trajectory_point().empty()) {
    const auto& previous_trajectory_points =
        previous_trajectory.trajectory_point();
    const Vec2d plan_start_pos =
        Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
    const auto closest_iter = absl::c_min_element(
        previous_trajectory_points,
        [&plan_start_pos](const ApolloTrajectoryPointProto& p1,
                          const ApolloTrajectoryPointProto& p2) {
          const Vec2d pos1(p1.path_point().x(), p1.path_point().y());
          const Vec2d pos2(p2.path_point().x(), p2.path_point().y());
          return (pos1 - plan_start_pos).squaredNorm() <
                 (pos2 - plan_start_pos).squaredNorm();
        });
    const int closest_index_on_prev_traj =
        std::distance(previous_trajectory_points.begin(), closest_iter);
    const auto diff_time =
        static_cast<double>((closest_index_on_prev_traj -
                             *plan_start_point_info.start_index_on_prev_traj) *
                            kTrajectoryTimeStep);
    if (diff_time > FLAGS_planner_path_start_point_time_diff_limit) {
      constexpr double kLookAheadTimeMaxTime = 5.0;
      look_ahead_time +=
          absl::Seconds(std::min(kLookAheadTimeMaxTime, diff_time));
    }
  }
  return look_ahead_time;
}

StPathPlanStartPointInfo GetStPathPlanStartPointInformation(
    const absl::Duration look_ahead_time,
    const PlanStartPointInfo& plan_start_point_info,
    const TrajectoryProto& previous_trajectory,
    std::optional<double> trajectory_optimizer_time_step,
    std::optional<absl::Time> last_st_path_plan_start_time) {
  absl::Time path_planning_time =
      plan_start_point_info.plan_time + look_ahead_time;

  if (last_st_path_plan_start_time.has_value() &&
      FLAGS_planner_st_path_planner_lookahead_for_trajectory_optimizer_synchronization) {
    CHECK(trajectory_optimizer_time_step.has_value());
    CHECK_GT(*trajectory_optimizer_time_step, 0.0);
    constexpr double kTimeEpsilon = 0.001;
    const double delta_t =
        std::max(absl::ToDoubleSeconds(path_planning_time -
                                       *last_st_path_plan_start_time),
                 0.0);
    const double res = std::fmod(delta_t, *trajectory_optimizer_time_step);
    if (res > kTimeEpsilon &&
        res < (*trajectory_optimizer_time_step) - kTimeEpsilon) {
      path_planning_time +=
          absl::Seconds((*trajectory_optimizer_time_step) - res);
    }
  }

  const auto path_start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
      path_planning_time, previous_trajectory);
  if (plan_start_point_info.reset ||
      !path_start_index_on_prev_traj.has_value()) {
    return {.reset = plan_start_point_info.reset,
            .relative_index_from_plan_start_point = 0,
            .start_point = plan_start_point_info.start_point,
            .plan_time = plan_start_point_info.plan_time};
  } else {
    ApolloTrajectoryPointProto path_plan_start_point =
        previous_trajectory.trajectory_point(*path_start_index_on_prev_traj);
    path_plan_start_point.set_relative_time(0.0);
    path_plan_start_point.mutable_path_point()->set_s(0.0);

    CHECK(plan_start_point_info.start_index_on_prev_traj.has_value());
    return {.reset = plan_start_point_info.reset,
            .relative_index_from_plan_start_point =
                *path_start_index_on_prev_traj -
                *plan_start_point_info.start_index_on_prev_traj,
            .start_point = path_plan_start_point,
            .plan_time = path_planning_time};
  }
}

absl::StatusOr<mapping::LanePath> FindPreferredLanePathFromTeleop(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start,
    const RouteNaviInfo& route_navi_info, mapping::ElementId ego_proj_lane_id,
    DriverAction::LaneChangeCommand lc_cmd) {
  if (lc_cmd == DriverAction::LC_CMD_NONE) {
    return absl::NotFoundError("Empty lane change command!");
  }

  SMM_ASSIGN_SECTION_OR_ERROR_ISSUE(section_info, psmm,
                                    route_sections_from_start.front().id);
  const auto& lane_ids = section_info.lanes();
  const auto current_it =
      std::find(lane_ids.begin(), lane_ids.end(), ego_proj_lane_id);
  if (current_it == lane_ids.end()) {
    return absl::NotFoundError(
        "Current lane not found in the current route section!");
  }

  constexpr double kForwardInitLength = 60.0;
  const auto short_route_sections = *ClampRouteSectionsBeforeArcLength(
      psmm, route_sections_from_start, kForwardInitLength);
  const RouteSectionsInfo short_sections_info(psmm, &short_route_sections);
  const double start_frac = short_sections_info.start_fraction();

  if (lc_cmd == DriverAction::LC_CMD_STRAIGHT) {
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *current_it, start_frac,
        short_sections_info.length());
  }

  if (lc_cmd == DriverAction::LC_CMD_LEFT) {
    if (current_it == lane_ids.begin()) {
      return absl::NotFoundError("Already on the leftmost lane!");
    }
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *std::prev(current_it),
        start_frac, short_sections_info.length());
  } else {
    if (std::next(current_it) == lane_ids.end()) {
      return absl::NotFoundError("Already on the rightmost lane!");
    }
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *std::next(current_it),
        start_frac, short_sections_info.length());
  }
}

absl::StatusOr<std::tuple<RouteSections, RouteSections, PointOnRouteSections>>
ProjectPointToRouteSections(const PlannerSemanticMapManager& psmm,
                            const RouteSections& route_sections,
                            const Vec2d& pos, double projection_range,
                            double keep_behind_length) {
  ASSIGN_OR_RETURN(const auto project_route_sections,
                   ClampRouteSectionsBeforeArcLength(psmm, route_sections,
                                                     projection_range));

  ASSIGN_OR_RETURN(auto point_proj, FindSmoothPointOnRouteSectionsByPlanPassage(
                                        psmm, project_route_sections, pos));

  std::vector<mapping::SectionId> sec_ids;
  for (int i = point_proj.section_idx; i < project_route_sections.size(); ++i) {
    sec_ids.push_back(project_route_sections.route_section_segment(i).id);
  }
  const RouteSections projected_route_sections_from_start(
      point_proj.fraction, project_route_sections.end_fraction(),
      std::move(sec_ids), project_route_sections.destination());

  ASSIGN_OR_RETURN(
      auto sections_from_start,
      AlignRouteSections(route_sections, projected_route_sections_from_start));

  RouteSections sections_with_behind;

  if (point_proj.accum_s > keep_behind_length) {
    ASSIGN_OR_RETURN(
        sections_with_behind,
        ClampRouteSectionsAfterArcLength(
            psmm, route_sections, point_proj.accum_s - keep_behind_length));
  } else {
    sections_with_behind = route_sections;
  }

  return std::make_tuple(std::move(sections_from_start),
                         std::move(sections_with_behind), point_proj);
}

bool CheckIfAllowCancel(const ApolloTrajectoryPointProto& plan_start_point,
                        const VehicleGeometryParamsProto& vehicle_geometry,
                        const Vec2d& ego_pos,
                        e2e_noa::mapping::LanePath* preferred_lane_path,
                        const double& dist_buffer) {
  double buffer = std::clamp(dist_buffer, -0.2, 0.2);
  double s;
  auto neareast_lane = preferred_lane_path->lane_seq()->GetNearestLane(
      ad_e2e::planning::Point2d(ego_pos.x(), ego_pos.y()));
  if (!neareast_lane) {
    return true;
  }
  neareast_lane->center_line().GetDistance(
      ad_e2e::planning::Point2d(ego_pos.x(), ego_pos.y()), nullptr, &s);
  double half_lane_width =
      std::max(neareast_lane->GetWidthAtAccumS(s) * 0.5, kMinHalfLaneWidth);
  double min_dist = DBL_MAX;
  const auto ego_width = vehicle_geometry.width();
  const auto ego_length = vehicle_geometry.length();
  e2e_noa::Box2d ego_box(ego_pos, plan_start_point.path_point().theta(),
                         ego_length, ego_width);

  const auto ego_corners = ego_box.GetAllCorners();
  for (auto corner : ego_corners) {
    auto temp_dist = neareast_lane->center_line().GetDistance(corner);
    min_dist = min_dist < temp_dist ? min_dist : temp_dist;
  }
  return min_dist > half_lane_width - buffer;
}

void HandleManualLcCommand(
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geometry,
    const PlannerSemanticMapManager& psmm,
    e2e_noa::DriverAction::LaneChangeCommand new_lc_cmd,
    const e2e_noa::mapping::LanePath& prev_lp_before_lc,
    const LaneChangeStateProto& prev_lc_state, const Vec2d& ego_pos,
    const double& ego_theta, e2e_noa::mapping::LanePath* preferred_lane_path,
    ALCState* alc_state, e2e_noa::DriverAction::LaneChangeCommand* lc_cmd_state,
    e2e_noa::planning::LcFeasibility* lc_unable_reason, bool* if_cancel_lc,
    int last_manual_lc_time, LaneChangeReason last_lc_reason) {
  if (!preferred_lane_path->IsEmpty()) {
    const auto& prev_pref_laneseq = preferred_lane_path->lane_seq();
    if (prev_pref_laneseq && prev_pref_laneseq->IsValid()) {
      std::vector<std::string> debug;
      std::vector<ad_e2e::planning::Point2d> debug_match_point;
      const auto seq = psmm.map_ptr()->GetSameLaneSequenceV2(
          prev_pref_laneseq, ego_pos.x(), ego_pos.y(), ego_theta, debug,
          debug_match_point);
      if (seq) {
        preferred_lane_path->set_lane_seq(seq);
        std::string str = "";
        for (const auto& lane : seq->lanes()) {
          str += lane->id() + ",";
        }
      }
    }
  }

  if (!prev_lp_before_lc.IsEmpty()) {
    std::string ids = "";
    for (const auto& lane : prev_lp_before_lc.lane_ids()) {
      ids += (lane + "; ");
    }
  }

  if (new_lc_cmd == DriverAction::LC_CMD_NONE ||
      new_lc_cmd == DriverAction::LC_CMD_STRAIGHT) {
    *lc_cmd_state = DriverAction::LC_CMD_NONE;
    return;
  }

  if ((new_lc_cmd == DriverAction::LC_CMD_LEFT ||
       new_lc_cmd == DriverAction::LC_CMD_RIGHT) &&
      last_lc_reason == LaneChangeReason::MANUAL_CHANGE &&
      last_manual_lc_time <= 5) {
    return;
  }

  if (new_lc_cmd == DriverAction::LC_CMD_CANCEL) {
    ALCState prev_alc_state = *alc_state;
    if (prev_lc_state.stage() == LCS_NONE) {
      *preferred_lane_path = mapping::LanePath();
      *alc_state = ALC_STANDBY_ENABLE;
      *lc_cmd_state = DriverAction::LC_CMD_NONE;
      return;
    }

    double dist_buffer = Lerp(0.3, 50.0 * ad_e2e::planning::Constants::KPH2MPS,
                              0.1, 70.0 * ad_e2e::planning::Constants::KPH2MPS,
                              plan_start_point.v(), true);
    if (!preferred_lane_path->IsEmpty()) {
      if (!CheckIfAllowCancel(plan_start_point, vehicle_geometry, ego_pos,
                              preferred_lane_path, dist_buffer)) {
        return;
      }
    }

    if (prev_lp_before_lc.IsEmpty()) {
      return;
    }

    const auto& nearest_lane =
        psmm.map_ptr()->GetNearestLane(ego_pos, ego_theta, 3.2, true, false);
    if (nearest_lane) {
      bool ego_lane_in_lp_before_lc = false;
      for (const auto& lane_id : prev_lp_before_lc.lane_ids()) {
        if (lane_id == nearest_lane->id()) {
          ego_lane_in_lp_before_lc = true;
          break;
        }
      }

      if (ego_lane_in_lp_before_lc == false) {
        *alc_state = ALC_RETURNING;
        if (prev_alc_state == ALC_RETURNING) {
          *lc_cmd_state = prev_lc_state.lc_left() ? DriverAction::LC_CMD_LEFT
                                                  : DriverAction::LC_CMD_RIGHT;
        } else {
          *lc_cmd_state = prev_lc_state.lc_left() ? DriverAction::LC_CMD_RIGHT
                                                  : DriverAction::LC_CMD_LEFT;
        }
        bool nearest_change_error = false;
        ad_e2e::planning::SLPoint ego_sl;
        nearest_lane->GetSLWithLimit(ego_pos, &ego_sl);

        if (*lc_cmd_state == DriverAction::LC_CMD_RIGHT) {
          nearest_change_error = (ego_sl.l < -0.5);
        } else if (*lc_cmd_state == DriverAction::LC_CMD_LEFT) {
          nearest_change_error = (ego_sl.l > 0.5);
        }

        if (!nearest_change_error) {
          e2e_noa::mapping::LanePath lane_path(psmm.map_ptr(),
                                               {nearest_lane->id()}, 0.0, 1.0);
          *preferred_lane_path = std::move(lane_path);
          return;
        }
      }
    }

    *preferred_lane_path = prev_lp_before_lc;
    *alc_state = ALC_RETURNING;

    if (prev_alc_state == ALC_RETURNING) {
      *lc_cmd_state = prev_lc_state.lc_left() ? DriverAction::LC_CMD_LEFT
                                              : DriverAction::LC_CMD_RIGHT;
    } else {
      *lc_cmd_state = prev_lc_state.lc_left() ? DriverAction::LC_CMD_RIGHT
                                              : DriverAction::LC_CMD_LEFT;
    }
    *if_cancel_lc = true;
    return;
  }

  if (!preferred_lane_path->IsEmpty()) {
    return;
  }
  if (prev_lc_state.stage() != LCS_NONE) {
    return;
  }

  const auto& nearest_lane =
      psmm.map_ptr()->GetNearestLane(ego_pos, ego_theta, 3.2, true, false);
  if (!nearest_lane) return;
  const auto left_lane =
      psmm.map_ptr()->GetLaneById(nearest_lane->left_lane_id());
  const auto right_lane =
      psmm.map_ptr()->GetLaneById(nearest_lane->right_lane_id());

  if (new_lc_cmd == DriverAction::LC_CMD_LEFT && left_lane) {
    const auto seq = psmm.map_ptr()->GetLaneSequence(left_lane, true);
    const auto lanes = seq->lanes();
    std::vector<std::string> lane_set;
    for (const auto lane : lanes) {
      if (lane) lane_set.emplace_back(lane->id());
    }
    e2e_noa::mapping::LanePath lane_path(psmm.map_ptr(), lane_set, 0.0, 1.0);
    *preferred_lane_path = std::move(lane_path);
  } else if (new_lc_cmd == DriverAction::LC_CMD_RIGHT && right_lane) {
    const auto seq = psmm.map_ptr()->GetLaneSequence(right_lane, true);
    const auto lanes = seq->lanes();
    std::vector<std::string> lane_set;
    for (const auto lane : lanes) {
      if (lane) lane_set.emplace_back(lane->id());
    }
    e2e_noa::mapping::LanePath lane_path(psmm.map_ptr(), lane_set, 0.0, 1.0);
    *preferred_lane_path = std::move(lane_path);
  } else {
    *lc_unable_reason = e2e_noa::planning::LcFeasibility::FEASIBILITY_NO_LANE;

    return;
  }

  *alc_state = ALC_ONGOING;
  *lc_cmd_state = new_lc_cmd;
  LOG(INFO) << "Processed new teleop lane change command: "
            << DriverAction_LaneChangeCommand_Name(new_lc_cmd);
  if (preferred_lane_path && preferred_lane_path->lane_seq() &&
      preferred_lane_path->lane_seq()->IsValid()) {
    std::string mlc_target_lanes = "";
    for (const auto& lane : preferred_lane_path->lane_seq()->lanes()) {
      if (lane) {
        mlc_target_lanes += (lane->id() + ",");
      }
    }
  }
}
}  // namespace planning
}  // namespace e2e_noa
