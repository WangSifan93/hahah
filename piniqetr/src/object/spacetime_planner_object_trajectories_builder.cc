#include "object/spacetime_planner_object_trajectories_builder.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "common/timer.h"
#include "math/linear_interpolation.h"
#include "math/math_utils.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_planner_object_trajectories_filter.h"
#include "object/spacetime_planner_object_trajectories_finder.h"
#include "plan/planner_defs.h"
#include "util/hmi_content_util.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa {
namespace planning {
namespace {
SpacetimePlannerObjectTrajectories GetSpacetimePlannerObjectTrajectories(
    const ApolloTrajectoryPointProto* plan_start_point,
    absl::Span<const SpacetimeObjectTrajectory> candidate_trajs,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFinder>>& finders,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFilter>>& filters,
    const PlanPassage* plan_passage,
    const LaneChangeStateProto* lane_change_state, double start_offset,
    const PathSlBoundary* sl_boundary,
    const VehicleGeometryParamsProto* veh_geom);

absl::StatusOr<double> Compute_Obstacles_Predicted_Trajectory_Length(
    const PlanPassage* plan_passage, const PathSlBoundary* sl_boundary,
    const LaneChangeStateProto* lane_change_state,
    const VehicleGeometryParamsProto* veh_geom,
    const ApolloTrajectoryPointProto* plan_start_point,
    const SpacetimeObjectTrajectory& traj, bool special_avoidance_conditions,
    bool is_current_lane_uturn);

absl::StatusOr<SpacetimeObjectTrajectory> CreateFakeTrajectoryByRelatedObstacle(
    const PlanPassage* plan_passage,
    const LaneChangeStateProto* lane_change_state,
    const VehicleGeometryParamsProto* veh_geom,
    const ApolloTrajectoryPointProto* plan_start_point,
    const SpacetimeObjectTrajectory& traj, double start_offset);

SpacetimePlannerObjectTrajectories GetSpacetimePlannerObjectTrajectories(
    const SpacetimePlannerObjectTrajectoriesBuilderInput& input,
    absl::Span<const SpacetimeObjectTrajectory> candidate_trajs,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFinder>>& finders,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFilter>>& filters) {
  const ApolloTrajectoryPointProto* plan_start_point = input.plan_start_point;
  const PlanPassage* plan_passage = input.passage;
  const LaneChangeStateProto* lane_change_state = input.lane_change_state;
  double start_offset = std::clamp(input.st_planner_start_offset, -0.5, 0.5);
  const PathSlBoundary* sl_boundary = input.sl_boundary;
  const VehicleGeometryParamsProto* veh_geom = input.veh_geom;

  SpacetimePlannerObjectTrajectories res;
  res.trajectories.reserve(candidate_trajs.size());
  res.trajectory_infos.reserve(candidate_trajs.size());
  res.st_start_offset = start_offset;

  const auto lane_id = plan_passage->lane_path().front().lane_id();
  const auto lane_info = input.psmm->FindCurveLaneByIdOrNull(lane_id);

  bool is_split = false, is_merge = false;
  auto pre_lane_ids = lane_info->pre_lane_ids();
  if (pre_lane_ids.size() == 1) {
    const auto pre_lane_info =
        input.psmm->FindCurveLaneByIdOrNull(pre_lane_ids[0]);
    if (pre_lane_info && pre_lane_info->next_lane_ids().size() >= 2) {
      is_split = true;
    }
  }

  auto next_lane_ids = lane_info->next_lane_ids();
  if (next_lane_ids.size() == 1) {
    const auto next_lane_info =
        input.psmm->FindCurveLaneByIdOrNull(next_lane_ids[0]);
    if (next_lane_info && next_lane_info->pre_lane_ids().size() >= 2) {
      is_merge = true;
    }
  }

  const bool special_avoidance_conditions =
      (lane_info && !lane_info->junction_id().empty()) || is_split || is_merge;

  for (const auto& traj : candidate_trajs) {
    for (const auto& finder : finders) {
      const auto selected_reason = finder->Find(traj);
      if (selected_reason != SpacetimePlannerObjectTrajectoryReason::NONE) {
        bool is_filtered = false;
        for (const auto& filter : filters) {
          if (filter->Filter(traj)) {
            is_filtered = true;
            break;
          }
        }
        if (is_filtered) {
          VLOG(2) << "filtered traj:" << traj.traj_id();
          break;
        }

        double st_planner_traj_horizon = kSpacetimePlannerTrajectoryHorizon;
        bool is_current_lane_uturn = false;
        if (!plan_passage->lane_path().IsEmpty()) {
          const auto current_lane_id = plan_passage->lane_path().lane_id(0);
          const auto current_lane_info =
              input.psmm->FindCurveLaneByIdOrNull(current_lane_id);
          if (current_lane_info != nullptr &&
              current_lane_info->turn_type() == ad_e2e::planning::U_TURN) {
            is_current_lane_uturn = true;
          }
        }
        auto obs_traj_horizon = Compute_Obstacles_Predicted_Trajectory_Length(
            plan_passage, sl_boundary, lane_change_state, veh_geom,
            plan_start_point, traj, special_avoidance_conditions,
            is_current_lane_uturn);
        if (obs_traj_horizon.ok()) {
          st_planner_traj_horizon =
              std::min(st_planner_traj_horizon, obs_traj_horizon.value());
        } else {
        }

        ASSIGN_OR_CONTINUE(
            auto truncated_traj,
            traj.CreateTruncatedCopy(start_offset, st_planner_traj_horizon));

        res.trajectories.push_back(std::move(truncated_traj));
        res.trajectory_infos.push_back(
            {.traj_index = traj.traj_index(),
             .object_id = traj.planner_object().is_sim_agent()
                              ? traj.planner_object().base_id()
                              : traj.planner_object().id(),
             .reason = selected_reason});
        res.trajectory_ids.insert(std::string(traj.traj_id()));
        break;
      }
    }
  }
  return res;
}

absl::StatusOr<SpacetimeObjectTrajectory> CreateFakeTrajectoryByRelatedObstacle(
    const PlanPassage* plan_passage,
    const LaneChangeStateProto* lane_change_state,
    const VehicleGeometryParamsProto* veh_geom,
    const ApolloTrajectoryPointProto* plan_start_point,
    const SpacetimeObjectTrajectory& traj, double start_offset) {
  if (traj.is_stationary() || lane_change_state->stage() != LCS_NONE) {
    return absl::InvalidArgumentError(
        absl::StrFormat("current lane change stage is:",
                        LaneChangeStage_Name(lane_change_state->stage()),
                        "traj.is_stationary() is:", traj.is_stationary()));
  }
  absl::string_view object_id = traj.planner_object().id();

  constexpr double kLateralMinSafeBufferInFakeStatic = 0.3;
  constexpr double kLateralMaxSpeedBufferInFakeStatic = 0.6;
  const auto& curr_path_point = plan_start_point->path_point();
  const auto& av_start_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *veh_geom);
  const auto av_start_frenet_box = plan_passage->QueryFrenetBoxAt(av_start_box);
  if (!av_start_frenet_box.ok()) return av_start_frenet_box.status();

  const auto object_start_frenet_box =
      plan_passage->QueryFrenetBoxAt(traj.bounding_box());
  if (!object_start_frenet_box.ok()) return object_start_frenet_box.status();

  if (std::fabs(traj.pose().v()) < kLateralMaxSpeedBufferInFakeStatic &&
      object_start_frenet_box->s_max > av_start_frenet_box->s_min &&
      !(object_start_frenet_box->l_min >
            av_start_frenet_box->l_max + kLateralMinSafeBufferInFakeStatic ||
        av_start_frenet_box->l_min >
            av_start_frenet_box->l_max + kLateralMinSafeBufferInFakeStatic) &&
      traj.object_type() == ObjectType::OT_CYCLIST) {
    if (traj.states().empty()) {
      return absl::FailedPreconditionError(
          absl::StrCat("The trajectory ", object_id, " has no states."));
    }
    std::vector<SpacetimeObjectState> truncated_states;
    truncated_states.reserve(traj.states().size());

    constexpr double kEps = 1e-6;
    const double start_t =
        traj.states()[0].traj_point->t() + start_offset - kEps;
    for (int i = 0, n = traj.states().size(); i < n; ++i) {
      if (traj.states()[i].traj_point->t() - start_t >
          kSpacetimePlannerTrajectoryHorizon) {
        break;
      }
      if (traj.states()[i].traj_point->t() >= start_t) {
        truncated_states.push_back(traj.states()[0]);
      }
    }
    if (truncated_states.empty()) {
      return absl::NotFoundError(absl::StrCat(
          "No trajectory for fake static trajectory:object_id", object_id));
    }
    PlannerObject planner_object = traj.planner_object();
    planner_object.set_stationary(true);
    return SpacetimeObjectTrajectory(
        planner_object, std::move(truncated_states), traj.traj_index(),
        traj.required_lateral_gap());
  }
  return absl::InvalidArgumentError(
      absl::StrCat("The current obstacle does not meet the fake static "
                   "obstacle conditions,object_id",
                   object_id));
}

absl::StatusOr<double> Compute_Obstacles_Predicted_Trajectory_Length(
    const PlanPassage* plan_passage, const PathSlBoundary* sl_boundary,
    const LaneChangeStateProto* lane_change_state,
    const VehicleGeometryParamsProto* veh_geom,
    const ApolloTrajectoryPointProto* plan_start_point,
    const SpacetimeObjectTrajectory& traj, bool special_avoidance_conditions,
    bool is_current_lane_uturn) {
  double st_planner_traj_horizon = kSpacetimePlannerTrajectoryHorizon;
  if (traj.is_stationary() || lane_change_state->stage() != LCS_NONE) {
    return st_planner_traj_horizon;
  }
  std::string_view object_id = traj.planner_object().id();
  const auto& traj_start_point = *traj.states().front().traj_point;
  const auto frenet_start_point =
      plan_passage->QueryUnboundedFrenetCoordinateAt(traj_start_point.pos());
  if (!frenet_start_point.ok()) return frenet_start_point.status();

  const auto lane_theta_at_pose =
      plan_passage->QueryTangentAngleAtS(frenet_start_point->s);
  if (!lane_theta_at_pose.ok()) return lane_theta_at_pose.status();

  constexpr double kSampleStepAlongS = 1.0;
  constexpr double kLateralMinSafeBufferToCurbDistance = 1.2;
  constexpr double kLateralMinSafeReverseTrajectoryHorizon = 2.0;
  const double kLateralMinSafeSameDirectionTrajectoryHorizon =
      special_avoidance_conditions ? 1.0 : 0.5;
  const double kLateralMinReserveSafeTrajectoryHorizon = 1.0;
  constexpr bool use_out_boundary = false;
  constexpr double kLateralTruncatingObstacleMinDistance = 0.6;
  constexpr double kDefaultHalfLaneWidth = 1.8;

  auto computer_obs_boundary = [sl_boundary](const FrenetBox& object_frenet_box,
                                             double& left_boundary,
                                             double& right_boundary) {
    left_boundary = std::numeric_limits<double>::infinity();
    right_boundary = -std::numeric_limits<double>::infinity();
    for (double sample_s = object_frenet_box.s_min;
         sample_s <= object_frenet_box.s_max; sample_s += kSampleStepAlongS) {
      const auto [right_l, left_l] =
          use_out_boundary ? sl_boundary->QueryBoundaryL(sample_s)
                           : sl_boundary->QueryTargetBoundaryL(sample_s);
      left_boundary = std::min({left_boundary, left_l, kDefaultHalfLaneWidth});
      right_boundary =
          std::max({right_boundary, right_l, -kDefaultHalfLaneWidth});
    }
  };

  const auto& curr_path_point = plan_start_point->path_point();
  const Vec2d& av_tangent = Vec2d::FastUnitFromAngle(curr_path_point.theta());
  const auto& av_start_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *veh_geom);
  const auto& av_start_frenet_box =
      plan_passage->QueryFrenetBoxAt(av_start_box, false);
  if (!av_start_frenet_box.ok()) return av_start_frenet_box.status();
  const auto& object_start_box =
      plan_passage->QueryFrenetBoxAt(traj.bounding_box(), false);
  if (!object_start_box.ok()) return object_start_box.status();

  double left_curb_boundary = std::numeric_limits<double>::infinity();
  double right_curb_boundary = -std::numeric_limits<double>::infinity();
  for (double sample_s = av_start_frenet_box->s_max;
       sample_s <= av_start_frenet_box->s_max +
                       std::clamp(0.5 * plan_start_point->v(), 10.0, 30.0);
       sample_s += kSampleStepAlongS) {
    const auto curb_l = plan_passage->QueryCurbOffsetAtS(sample_s);
    if (!curb_l.ok()) continue;
    left_curb_boundary = std::min(left_curb_boundary, curb_l->second);
    right_curb_boundary = std::max(right_curb_boundary, curb_l->first);
  }

  const double kLateralMinSafeBufferInReverseSameCar =
      av_start_frenet_box->center_l() > object_start_box->center_l()
          ? (left_curb_boundary - av_start_frenet_box->l_max <
                     kLateralMinSafeBufferToCurbDistance
                 ? 0.9
                 : (special_avoidance_conditions ? 0.0 : 0.5))
          : (av_start_frenet_box->l_min - right_curb_boundary <
                     kLateralMinSafeBufferToCurbDistance
                 ? 0.9
                 : (special_avoidance_conditions ? 0.0 : 0.5));
  double obs_frenet_l_min = std::numeric_limits<double>::infinity();
  double obs_frenet_l_max = -std::numeric_limits<double>::infinity();
  for (auto it = traj.states().begin(); it != traj.states().end(); ++it) {
    const auto& object_frenet_box = plan_passage->QueryFrenetBoxAt(it->box);
    if (!object_frenet_box.ok()) continue;
    obs_frenet_l_min = std::min(obs_frenet_l_min, object_frenet_box->l_min);
    obs_frenet_l_max = std::max(obs_frenet_l_max, object_frenet_box->l_max);
  }

  const bool cut_in_intention =
      av_start_frenet_box->center_l() > object_start_box->center_l()
          ? obs_frenet_l_max - object_start_box->l_max >
                kLateralTruncatingObstacleMinDistance
          : object_start_box->l_min - obs_frenet_l_min >
                kLateralTruncatingObstacleMinDistance;

  if (frenet_start_point->s - plan_start_point->path_point().s() > 0.0 &&
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj_start_point.theta())) < M_PI / 6 &&
      cut_in_intention) {
    for (auto it = traj.states().begin(); it != traj.states().end(); ++it) {
      if (it == traj.states().begin()) continue;
      const double time_count =
          std::distance(traj.states().begin(), it) * kTrajectoryTimeStep;
      const auto& object_frenet_box =
          plan_passage->QueryFrenetBoxAt(it->box, false);
      if (!object_frenet_box.ok()) {
        return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                          kSpacetimePlannerTrajectoryHorizon);
      }
      double left_boundary = std::numeric_limits<double>::infinity();
      double right_boundary = -std::numeric_limits<double>::infinity();
      computer_obs_boundary(object_frenet_box.value(), left_boundary,
                            right_boundary);
      if (object_start_box->center_l() >
              av_start_frenet_box->center_l() + 0.5 &&
          std::min(right_boundary, av_start_frenet_box->l_min) +
                  veh_geom->width() + kLateralMinSafeBufferInReverseSameCar >
              object_frenet_box.value().l_min) {
        return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                          kSpacetimePlannerTrajectoryHorizon);
      } else if (av_start_frenet_box->center_l() >
                     object_start_box->center_l() + 0.5 &&
                 std::max(left_boundary, av_start_frenet_box->l_max) -
                         veh_geom->width() -
                         kLateralMinSafeBufferInReverseSameCar <
                     object_frenet_box.value().l_max) {
        return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                          kSpacetimePlannerTrajectoryHorizon);
      }
    }
  }

  double safety_time = 0.0;
  bool find_safety_time = false;
  if (cut_in_intention &&
      frenet_start_point->s - plan_start_point->path_point().s() > -30.0 &&
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj_start_point.theta())) > M_PI / 6 - 1e-3) {
    for (auto it = traj.states().begin(); it != traj.states().end(); ++it) {
      const double time_count =
          std::distance(traj.states().begin(), it) * kTrajectoryTimeStep;
      const auto& object_frenet_box =
          plan_passage->QueryFrenetBoxAt(it->box, false);
      if (!object_frenet_box.ok()) {
        return std::clamp(time_count,
                          kLateralMinSafeSameDirectionTrajectoryHorizon,
                          kSpacetimePlannerTrajectoryHorizon);
      }

      double left_boundary = std::numeric_limits<double>::infinity();
      double right_boundary = -std::numeric_limits<double>::infinity();
      computer_obs_boundary(object_frenet_box.value(), left_boundary,
                            right_boundary);
      const double ego_lat_max =
          std::clamp(0.5 * 0.5 * Sqr(time_count), 0.4, 1.0);
      if (!find_safety_time) {
        if (object_start_box->center_l() >
                av_start_frenet_box->center_l() + 0.5 &&
            std::max(std::min(right_boundary, av_start_frenet_box->l_min) +
                         veh_geom->width(),
                     av_start_frenet_box->l_max - ego_lat_max) +
                    kLateralMinSafeBufferInReverseSameCar >
                object_frenet_box.value().l_min) {
          find_safety_time = true;
          safety_time = time_count;
        } else if (av_start_frenet_box->center_l() >
                       object_start_box->center_l() + 0.5 &&
                   std::min(
                       std::max(left_boundary, av_start_frenet_box->l_max) -
                           veh_geom->width(),
                       av_start_frenet_box->l_min + ego_lat_max) -
                           kLateralMinSafeBufferInReverseSameCar <
                       object_frenet_box.value().l_max) {
          find_safety_time = true;
          safety_time = time_count;
        }
      }
      const auto& av_time_box =
          ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()) +
                           av_tangent * (time_count * plan_start_point->v()),
                       curr_path_point.theta(), *veh_geom);
      const auto& av_frenet_box =
          plan_passage->QueryFrenetBoxAt(av_time_box, false);
      if (!av_frenet_box.ok()) return av_frenet_box.status();

      if (object_frenet_box.value().s_max < av_frenet_box.value().s_min ||
          object_frenet_box.value().s_min > av_frenet_box.value().s_max) {
        continue;
      }

      if (object_start_box->center_l() >
              av_start_frenet_box->center_l() + 0.5 &&
          std::max(std::min(right_boundary, av_start_frenet_box->l_min) +
                       veh_geom->width(),
                   av_start_frenet_box->l_max - ego_lat_max) +
                  kLateralMinSafeBufferInReverseSameCar >
              object_frenet_box.value().l_min) {
        return std::clamp(time_count,
                          kLateralMinSafeSameDirectionTrajectoryHorizon,
                          kSpacetimePlannerTrajectoryHorizon);
      } else if (av_start_frenet_box->center_l() >
                     object_start_box->center_l() + 0.5 &&
                 std::min(std::max(left_boundary, av_start_frenet_box->l_max) -
                              veh_geom->width(),
                          av_start_frenet_box->l_min + ego_lat_max) -
                         kLateralMinSafeBufferInReverseSameCar <
                     object_frenet_box.value().l_max) {
        return std::clamp(time_count,
                          kLateralMinSafeSameDirectionTrajectoryHorizon,
                          kSpacetimePlannerTrajectoryHorizon);
      }
    }
  }

  if (find_safety_time) {
    return std::clamp(safety_time, kLateralMinReserveSafeTrajectoryHorizon,
                      kSpacetimePlannerTrajectoryHorizon);
  }

  return st_planner_traj_horizon;
}

}  

SpacetimePlannerObjectTrajectories BuildSpacetimePlannerObjectTrajectories(
    const SpacetimePlannerObjectTrajectoriesBuilderInput& input,
    absl::Span<const SpacetimeObjectTrajectory> trajectories) {
  Timer timer(__FUNCTION__);
  CHECK_NOTNULL(input.passage);
  CHECK_NOTNULL(input.sl_boundary);
  CHECK_NOTNULL(input.veh_geom);
  CHECK_NOTNULL(input.plan_start_point);
  CHECK_NOTNULL(input.prev_st_trajs);
  CHECK_NOTNULL(input.spacetime_planner_object_trajectories_params);

  const auto& config = input.spacetime_planner_object_trajectories_params
                           ->spacetime_planner_object_trajectories_config();

  std::vector<std::unique_ptr<SpacetimePlannerObjectTrajectoriesFinder>>
      finders;
  if (config.enable_all_finder()) {
    finders.push_back(
        std::make_unique<AllSpacetimePlannerObjectTrajectoriesLocator>());
  }
  if (config.enable_stationary_finder()) {
    finders.push_back(
        std::make_unique<StationarySpacetimePlannerObjectTrajectoriesFinder>(
            input.psmm, input.passage->lane_path()));
  }
  const auto& curr_path_point = input.plan_start_point->path_point();
  const auto av_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *input.veh_geom);
  if (config.enable_front_side_moving_finder()) {
    finders.push_back(std::make_unique<
                      FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder>(
        av_box, input.passage, input.sl_boundary, input.plan_start_point->v(),
        input.prev_st_trajs, input.time_aligned_prev_traj, input.veh_geom,
        input.psmm, input.nudge_object_info, input.lane_change_state));
  }
  if (config.enable_dangerous_side_moving_finder()) {
    finders.push_back(
        std::make_unique<
            DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder>(
            av_box, input.passage, input.plan_start_point->v()));
  }

  if (config.enable_front_moving_finder()) {
    finders.push_back(
        std::make_unique<FrontMovingSpacetimePlannerObjectTrajectoriesFinder>(
            input.passage, input.plan_start_point, input.veh_geom->length()));
  }

  std::vector<std::unique_ptr<SpacetimePlannerObjectTrajectoriesFilter>>
      filters;

  filters.push_back(
      std::make_unique<CutInSpacetimePlannerObjectTrajectoriesFilter>(
          input.passage, input.lane_change_state, av_box,
          input.plan_start_point->v()));
  if (config.enable_cutin_vehicle_filter()) {
    const auto& curr_path_point = input.plan_start_point->path_point();
    const auto av_box =
        ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                     curr_path_point.theta(), *input.veh_geom);
    filters.push_back(
        std::make_unique<CutInVehicleSpacetimePlannerObjectTrajectoriesFilter>(
            input.passage, input.lane_change_state, av_box,
            input.plan_start_point->v()));
  }
  if (config.enable_crossing_filter()) {
    filters.push_back(
        std::make_unique<CrossingSpacetimePlannerObjectTrajectoriesFilter>(
            input.passage, input.psmm));
  }
  if (config.enable_reverse_vehicle_filter()) {
    const auto& curr_path_point = input.plan_start_point->path_point();
    auto av_box = ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                               curr_path_point.theta(), *input.veh_geom);
    filters.push_back(std::make_unique<
                      ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter>(
        input.passage, input.psmm, input.veh_geom, input.sl_boundary,
        input.nudge_object_info, std::move(av_box),
        input.plan_start_point->v()));
  }
  if (config.enable_beyond_stop_line_filter()) {
    filters.push_back(std::make_unique<
                      BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter>(
        input.passage, input.stop_lines));
  }
  return GetSpacetimePlannerObjectTrajectories(input, trajectories, finders,
                                               filters);
}
}  
}  
