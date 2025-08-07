#include "object/spacetime_planner_object_trajectories_finder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "common/log_data.h"
#include "glog/logging.h"
#include "math/geometry/polygon2d.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "perception.pb.h"
#include "plan/planner_defs.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "util/lane_path_util.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa {
namespace planning {
namespace {

constexpr double kLateralDistanceNudgeMovingObs = 0.2;
constexpr double kHysteresisLateralDistanceNudgeMovingObs = 0.1;
constexpr double kCloseMovingObjThreshold = 0.75;

struct LateralRelationInfo {
  double obj_lmin = std::numeric_limits<double>::lowest();
  double obj_lmax = std::numeric_limits<double>::max();
  double ref_lmin = std::numeric_limits<double>::lowest();
  double ref_lmax = std::numeric_limits<double>::max();
};

bool IsTrajEffectiveAndFillTrajBoxes(
    const PlanPassage& plan_passage, const PathSlBoundary& path_sl_boundary,
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    const VehicleGeometryParamsProto& veh_geo, double effective_check_time,
    double fill_boxes_time, std::vector<Box2d>* time_aligned_prev_traj_boxes) {
  const int comfortable_nudge_lat_speed_check_count =
      static_cast<int>(fill_boxes_time / kTrajectoryTimeStep + 0.5);
  time_aligned_prev_traj_boxes->reserve(
      comfortable_nudge_lat_speed_check_count);
  for (const auto& pt : time_aligned_prev_traj) {
    if (pt.relative_time() > effective_check_time) break;
    auto av_box = ComputeAvBox(Vec2d(pt.path_point().x(), pt.path_point().y()),
                               pt.path_point().theta(), veh_geo);

    const auto frenet_box_or = plan_passage.QueryFrenetBoxAt(av_box);
    if (!frenet_box_or.ok()) {
      time_aligned_prev_traj_boxes->clear();
      return false;
    }
    const auto [l_min, l_max] = path_sl_boundary.QueryBoundaryL(
        0.5 * (frenet_box_or->s_min + frenet_box_or->s_max));
    const bool is_box_inside_boundary =
        frenet_box_or->l_min > l_min && frenet_box_or->l_max < l_max;
    if (!is_box_inside_boundary) {
      time_aligned_prev_traj_boxes->clear();
      return false;
    }
    if (pt.relative_time() < fill_boxes_time) {
      time_aligned_prev_traj_boxes->push_back(std::move(av_box));
    }
  }
  return true;
}

bool IsVru(const SpacetimeObjectTrajectory& traj) {
  return traj.object_type() == ObjectType::OT_MOTORCYCLIST ||
         traj.object_type() == ObjectType::OT_CYCLIST ||
         traj.object_type() == ObjectType::OT_TRICYCLIST ||
         traj.object_type() == ObjectType::OT_PEDESTRIAN;
}

bool IsFrontTrajectory(const PlannerSemanticMapManager& psmm,
                       const PathSlBoundary& sl_boundary,
                       const PlanPassage& plan_passage, const Box2d& av_box,
                       const FrenetBox& av_sl_box, const Polygon2d& contour,
                       const SpacetimeObjectTrajectory& traj,
                       const VehicleGeometryParamsProto& veh_geo,
                       const NudgeObjectInfo* nudge_object_info,
                       const LaneChangeStateProto* lane_change_state,
                       bool prev_st_planner_obj, double av_speed,
                       bool is_use_back) {
  if (nudge_object_info && nudge_object_info->id == traj.object_id()) {
    return true;
  }

  const auto object_frenet_box =
      plan_passage.QueryFrenetBoxAtContour(traj.contour());
  if (!object_frenet_box.ok()) return false;
  const auto frenet_start_point = plan_passage.QueryUnboundedFrenetCoordinateAt(
      traj.states().front().traj_point->pos());
  if (!frenet_start_point.ok()) {
    return false;
  }
  const auto lane_theta_at_pose =
      plan_passage.QueryTangentAngleAtS(frenet_start_point->s);
  if (!lane_theta_at_pose.ok()) {
    return false;
  }

  if (IsVru(traj)) {
    const auto obj_ego_lon_dis =
        object_frenet_box.value().center_s() - av_sl_box.s_max;
    const double calculate_3v_lon_dis =
        std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                                traj.states().front().traj_point->theta())) <
                M_PI / 2
            ? 3.0 * (av_speed + traj.pose().v())
            : 3.0 * av_speed;
    if (std::clamp(calculate_3v_lon_dis, 30.0, 120.0) < obj_ego_lon_dis) {
      VLOG(2) << "front_side_finder"
              << absl::StrCat("vru lon far,dis:", obj_ego_lon_dis,
                              "-id:", traj.object_id());
      return false;
    }
  }

  const auto cur_lane_id = plan_passage.lane_path().front().lane_id();
  const auto cur_lane_info = psmm.FindCurveLaneByIdOrNull(cur_lane_id);
  const bool is_in_intersection =
      cur_lane_info && !cur_lane_info->junction_id().empty();
  if (IsVru(traj) && is_in_intersection) {
    auto delta_theta = NormalizeAngle(
        traj.states().front().traj_point->theta() - *lane_theta_at_pose);
    auto lat_min_dis = std::min(std::fabs(object_frenet_box.value().l_min),
                                std::fabs(object_frenet_box.value().l_max));
    if (lat_min_dis >
            veh_geo.width() / 2.0 + (prev_st_planner_obj ? 2.0 : 2.5) &&
        std::abs(delta_theta) < M_PI / 6 + (prev_st_planner_obj ? 0.1 : 0.0)) {
      VLOG(2) << "front_side_finder"
              << absl::StrCat("vru lat far,dis:", lat_min_dis,
                              "-id:", traj.object_id());
      return false;
    }
    if (std::abs(delta_theta) < M_PI / 6 + (prev_st_planner_obj ? 0.1 : 0.0) &&
        object_frenet_box.value().center_s() > av_sl_box.s_max &&
        traj.pose().v() + Kph2Mps(3.0) > av_speed) {
      constexpr bool use_out_boundary = false;
      constexpr double kMaxHalfLaneWidth = 2.5;
      constexpr double kMinHalfLaneWidth = 1.2;
      constexpr double kSampleStepAlongS = 1.0;
      const double kLatSafeDebounceDistance = prev_st_planner_obj ? 0.5 : 0.2;
      double initial_left_boundary = std::numeric_limits<double>::infinity();
      double initial_right_boundary = -std::numeric_limits<double>::infinity();
      for (double sample_s = object_frenet_box.value().s_min;
           sample_s <= object_frenet_box.value().s_max;
           sample_s += kSampleStepAlongS) {
        const auto [right_l, left_l] =
            use_out_boundary ? sl_boundary.QueryBoundaryL(sample_s)
                             : sl_boundary.QueryTargetBoundaryL(sample_s);
        initial_left_boundary = std::min(initial_left_boundary, left_l);
        initial_right_boundary = std::max(initial_right_boundary, right_l);
      }
      const double left_boundary = std::clamp(
          initial_left_boundary, kMinHalfLaneWidth, kMaxHalfLaneWidth);
      const double right_boundary = std::clamp(
          initial_right_boundary, -kMaxHalfLaneWidth, -kMinHalfLaneWidth);
      if (delta_theta > 0.0
              ? object_frenet_box.value().l_min <
                    right_boundary + veh_geo.width() - kLatSafeDebounceDistance
              : object_frenet_box.value().l_max >
                    left_boundary - veh_geo.width() +
                        kLatSafeDebounceDistance) {
        VLOG(2) << "front_side_finder"
                << absl::StrCat(
                       "vru is same but no space:", "-id:", traj.object_id());
        return false;
      }
    }
  }

  const auto& states = traj.states();
  ASSIGN_OR_RETURN(const auto obj_frenet_box,
                   plan_passage.QueryFrenetBoxAtContour(states[0].contour),
                   false);
  const PiecewiseLinearFunction<double, double> av_l_dis_ttc = {{0.5, 1.0},
                                                                {0.0, 2.0}};
  double back_considered_ttc =
      is_use_back ? 2.0 : av_l_dis_ttc(std::abs(av_sl_box.center_l()));
  double speed_diff =
      (back_considered_ttc > 0.0 &&
       states[0].traj_point->v() > av_speed - 0.5 &&
       obj_frenet_box.s_max < av_sl_box.center_s())
          ? std::clamp(
                back_considered_ttc * (states[0].traj_point->v() - av_speed),
                2.0, 30.0)
          : 0.0;
  const std::string speed_diff_debug =
      std::string(traj.object_id()) +
      " speed_diff: " + std::to_string(speed_diff);

  constexpr double kFrontRelativeSOffset = 1.0;
  const double hysteresis_rel_s_offset =
      prev_st_planner_obj ? veh_geo.front_edge_to_center() : 0.0;
  if (av_sl_box.center_s() <
      obj_frenet_box.s_max + speed_diff + hysteresis_rel_s_offset) {
    return obj_frenet_box.s_max > av_sl_box.center_s()
               ? true
               : (std::fabs((states[0].box.center() - av_box.center())
                                .Dot(av_box.tangent().Perp())) > 1.2
                      ? true
                      : false);
  }
  return false;
}

void CheckInvading(const SpacetimeObjectTrajectory& traj,
                   const PlanPassage& plan_passage,
                   const PathSlBoundary& path_sl_boundary,
                   const FrenetBox& av_sl_box, const double av_speed,
                   const FrenetBox& obj_frenet_box, double kInvadeThreshold,
                   bool& is_invading, bool& is_aggressive_invading,
                   double& invade_time) {
  const auto ref_center_l =
      path_sl_boundary.QueryReferenceCenterL(av_sl_box.center_s());
  const auto av_cur_half_width = av_sl_box.width() * 0.5;
  const auto obj_l_center = obj_frenet_box.center_l();
  const auto ego_l_min =
      std::min(av_sl_box.l_min, ref_center_l - av_cur_half_width);
  const auto ego_l_max =
      std::max(av_sl_box.l_max, ref_center_l + av_cur_half_width);
  const auto av_right_check_l = ego_l_min - kInvadeThreshold;
  const auto av_left_check_l = ego_l_max + kInvadeThreshold;

  for (const auto& state : traj.states()) {
    ASSIGN_OR_CONTINUE(const auto frenet_box,
                       plan_passage.QueryFrenetBoxAt(state.box));
    if (frenet_box.center_s() < av_sl_box.s_min) {
      continue;
    }
    if ((frenet_box.l_min < av_left_check_l &&
         frenet_box.l_min > av_right_check_l) ||
        (frenet_box.l_max < av_left_check_l &&
         frenet_box.l_max > av_right_check_l)) {
      invade_time = std::min(invade_time, state.traj_point->t());
      is_invading = true;
    }

    bool is_laterally_cross = (obj_l_center > av_sl_box.center_l() &&
                               frenet_box.center_l() < ego_l_min) ||
                              (obj_l_center < av_sl_box.center_l() &&
                               frenet_box.center_l() > ego_l_max);
    if (is_laterally_cross) {
      auto ref_angle =
          plan_passage.QueryTangentAngleAtS(obj_frenet_box.center_s());
      if (ref_angle.ok()) {
        double state_angle_diff =
            std::abs(NormalizeAngle(state.traj_point->theta() - *ref_angle));
        is_aggressive_invading =
            !(frenet_box.center_s() - av_sl_box.s_max > 5.0 * av_speed &&
              state_angle_diff < d2r(20.0));
      }
    }
  }
}

bool IsTrajNudgeAllowed(const PlanPassage& plan_passage,
                        const PlannerSemanticMapManager& psmm,
                        const PathSlBoundary& path_sl_boundary,
                        const Box2d& av_box, const FrenetBox& av_sl_box,
                        const double av_speed,
                        const SpacetimeObjectTrajectory& traj,
                        const VehicleGeometryParamsProto& veh_geo) {
  const double kInvadeThreshold = 0.5;
  ASSIGN_OR_RETURN(const auto obj_frenet_box,
                   plan_passage.QueryFrenetBoxAt(traj.states()[0].box), false);
  const auto obj_heading = traj.states()[0].traj_point->theta();
  auto ego_heading = av_box.heading();
  auto heading_diff = std::abs(NormalizeAngle(obj_heading - ego_heading));
  const auto passage_theta_at_object_s =
      plan_passage.QueryTangentAngleAtS(obj_frenet_box.center_s());
  if (passage_theta_at_object_s.ok()) {
    heading_diff =
        std::abs(NormalizeAngle(*passage_theta_at_object_s - obj_heading));
  }
  bool is_invading = false;
  bool is_aggressive_invading = false;
  double invade_time = std::numeric_limits<double>::max();

  CheckInvading(traj, plan_passage, path_sl_boundary, av_sl_box, av_speed,
                obj_frenet_box, kInvadeThreshold, is_invading,
                is_aggressive_invading, invade_time);

  if (!is_invading) {
    return true;
  }
  const auto cur_lane_id = plan_passage.lane_path().front().lane_id();
  const auto cur_lane_info = psmm.FindCurveLaneByIdOrNull(cur_lane_id);
  const auto cur_turn_type =
      cur_lane_info ? cur_lane_info->turn_type() : ad_e2e::planning::NO_TURN;

  if (traj.object_type() == ObjectType::OT_MOTORCYCLIST ||
      traj.object_type() == ObjectType::OT_CYCLIST ||
      traj.object_type() == ObjectType::OT_TRICYCLIST ||
      traj.object_type() == ObjectType::OT_PEDESTRIAN) {
    if (av_sl_box.l_min - obj_frenet_box.l_max > 3 * kInvadeThreshold ||
        obj_frenet_box.l_min - av_sl_box.l_max > 3 * kInvadeThreshold) {
      return false;
    }
    const double lon_meet_time =
        std::max((obj_frenet_box.center_s() - av_sl_box.s_max) /
                     (av_speed + traj.states()[0].traj_point->v()),
                 0.0);
    bool is_heading_within_range =
        heading_diff < d2r(30.0) ||
        (heading_diff > d2r(150.0) && lon_meet_time < 6.0);
    if (is_heading_within_range && !is_aggressive_invading) {
      return true;
    }
    return false;
  }

  if ((heading_diff < d2r(30.0) ||
       cur_turn_type == ad_e2e::planning::RIGHT_TURN ||
       (heading_diff > d2r(160.0) && invade_time < 2.0)) &&
      !is_aggressive_invading) {
    return true;
  }
  return false;
}

bool IsComfortablyNudgeableTrajectory(
    const PlanPassage& plan_passage,
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    const PathSlBoundary& path_sl_boundary, const Box2d& av_box,
    const FrenetBox& av_sl_box, double av_speed, double av_width,
    const SpacetimeObjectTrajectory& traj, double lat_nudge_dist,
    const VehicleGeometryParamsProto& veh_geo,
    double comfortable_nudge_check_time,
    double comfortable_nudge_lat_speed_check_time,
    std::optional<bool>* is_prev_traj_effective,
    std::vector<Box2d>* time_aligned_prev_traj_boxes) {
  av_speed = std::max(av_speed, 0.0);

  const PiecewiseLinearFunction<double, double> av_speed_close_moving_dist_plf =
      {{0.0, 10.0, 20.0}, {0.2, 0.3, 0.4}};
  const auto& states = traj.states();
  const double av_to_obj_dist = av_box.DistanceTo(states[0].box);
  if (av_to_obj_dist < av_speed_close_moving_dist_plf(av_speed)) {
    const std::string too_close_obs =
        std::string(traj.object_id()) +
        "av_to_obj_dist: " + std::to_string(av_to_obj_dist) +
        " av_speed_close_moving_dist_plf" +
        std::to_string(av_speed_close_moving_dist_plf(av_speed));

    VLOG(2) << "Object is too close to AV to be nudged. Distance: "
            << av_to_obj_dist;
    return false;
  }

  struct NudgeInfo {
    double t;
    double obj_l_min;
    double obj_l_max;
    double path_l_min;
    double path_l_max;
    bool lon_overlapped;
    double s_min;
    double s_max;
    const Box2d* box;
    double theta;
  };
  std::vector<NudgeInfo> nudge_info_vec;
  nudge_info_vec.reserve(states.size());
  constexpr double kAvMaxAccel = 0.2;
  constexpr double kAvMinAccel = -0.2;
  const double brake_to_stop_time = -av_speed / kAvMinAccel;
  const double brake_to_stop_dist = -0.5 * Sqr(av_speed) / kAvMinAccel;
  for (int i = 0; i < states.size(); ++i) {
    const auto& state = states[i];
    if (state.traj_point->t() > comfortable_nudge_check_time) break;
    ASSIGN_OR_CONTINUE(const auto frenet_box,
                       plan_passage.QueryFrenetBoxAt(state.box));
    const auto [l_min, l_max] = path_sl_boundary.QueryBoundaryL(
        0.5 * (frenet_box.s_min + frenet_box.s_max));

    const double half_t_sqr = 0.5 * Sqr(state.traj_point->t());
    const double av_max_progress =
        av_speed * state.traj_point->t() + half_t_sqr * kAvMaxAccel;
    const double av_min_progress =
        state.traj_point->t() < brake_to_stop_time
            ? av_speed * state.traj_point->t() + half_t_sqr * kAvMinAccel
            : brake_to_stop_dist;
    const double av_s_min = av_sl_box.s_min + av_min_progress;
    const double av_s_max = av_sl_box.s_max + av_max_progress;
    constexpr double kLonOverlapExtent = 1.0;
    const bool lon_overlapped =
        frenet_box.s_min < av_s_max + kLonOverlapExtent &&
        frenet_box.s_max > av_s_min - kLonOverlapExtent;
    VLOG(3) << "At point " << i << " obj_s_min " << frenet_box.s_min
            << " obj_s_max " << frenet_box.s_max << " obj_l_min "
            << frenet_box.l_min << " obj_l_max " << frenet_box.l_max
            << " path_l_min " << l_min << " path_l_max " << l_max
            << " av_s_min " << av_s_min << " av_s_max " << av_s_max
            << " lon_overlapped " << int(lon_overlapped) << " t "
            << state.traj_point->t();
    nudge_info_vec.push_back({.t = state.traj_point->t(),
                              .obj_l_min = frenet_box.l_min,
                              .obj_l_max = frenet_box.l_max,
                              .path_l_min = l_min,
                              .path_l_max = l_max,
                              .lon_overlapped = lon_overlapped,
                              .s_min = frenet_box.s_min,
                              .s_max = frenet_box.s_max,
                              .box = &state.box,
                              .theta = state.traj_point->theta()});
  }

  constexpr double kRelSpeedThres = 2.5;
  constexpr double kObjectSExtent = 2.0;
  const auto& nudge_info_first = nudge_info_vec.front();
  if (traj.planner_object().velocity().Dot(av_box.tangent()) - av_speed >
      kRelSpeedThres) {
    const bool cur_lon_overlapped =
        (av_sl_box.s_max > (nudge_info_first.s_min - kObjectSExtent) &&
         av_sl_box.s_max < (nudge_info_first.s_max + kObjectSExtent)) ||
        (av_sl_box.s_min > (nudge_info_first.s_min - kObjectSExtent) &&
         av_sl_box.s_min < (nudge_info_first.s_max + kObjectSExtent));
    if (!cur_lon_overlapped) {
      double last_lon_overlap_time = 0.0;
      for (auto it = nudge_info_vec.rbegin(); it != nudge_info_vec.rend();
           it++) {
        if (it->lon_overlapped) {
          last_lon_overlap_time = it->t;
        }
      }
      constexpr double kLonOverlapTimeThres = 0.5;
      if (last_lon_overlap_time < kLonOverlapTimeThres) {
        VLOG(2) << "last_lon_overlap_time: " << last_lon_overlap_time;
        return false;
      }
    }
  }

  std::optional<bool> is_prev_traj_close_to_object_traj;

  constexpr double kMinimumNudgeBuffer = 0.5;
  constexpr double kTimeEps = 1e-3;
  const PiecewiseLinearFunction<double, double> av_speed_lat_speed_thres_plf = {
      {0.0, 5.0, 10.0, 15.0, 20}, {3.0, 2.0, 1.2, 0.6, 0.3}};
  const double av_lat_speed_thres = av_speed_lat_speed_thres_plf(av_speed);
  const double av_center_l = 0.5 * (av_sl_box.l_max + av_sl_box.l_min);
  const double av_half_width = 0.5 * av_width;
  bool can_comfortably_nudge_from_left = true, left_have_space = true;
  int first_left_no_space_index = std::numeric_limits<int>::infinity();
  VLOG(2) << "Check comfortable nudge from left.";
  for (int i = 0; i < nudge_info_vec.size(); ++i) {
    const auto& nudge_info = nudge_info_vec[i];
    if (!nudge_info.lon_overlapped) {
      VLOG(3) << "Not lon overlapped at point " << i;
      continue;
    }
    if (left_have_space) {
      const double space =
          nudge_info.path_l_max - nudge_info.obj_l_max - av_width;
      if (space < lat_nudge_dist) {
        const std::string no_space =
            std::string(traj.object_id()) + "space: " + std::to_string(space) +
            " obj_l_min" + std::to_string(nudge_info.obj_l_min) +
            "path_l_min: " + std::to_string(nudge_info.path_l_min) +
            "lat_nudge_dist: " + std::to_string(lat_nudge_dist);

        VLOG(2) << "Not enough left nudge space at point " << i
                << ", path_l_max: " << nudge_info.path_l_max
                << ", obj_l_max: " << nudge_info.obj_l_max
                << ", av_width: " << av_width
                << ", lat_nudge_dist: " << lat_nudge_dist;
        left_have_space = false;
        first_left_no_space_index = i;
      }
    }

    if (traj.object_type() == ObjectType::OT_PEDESTRIAN) continue;
    if (nudge_info.t < kTimeEps) continue;
    if (nudge_info.t < comfortable_nudge_lat_speed_check_time) {
      const double estimate_nudge_lat_movement =
          std::max(nudge_info.obj_l_max + kMinimumNudgeBuffer + av_half_width -
                       av_center_l,
                   0.0);
      const double estimate_nudge_lat_speed =
          estimate_nudge_lat_movement / nudge_info.t;
      VLOG(2) << "Left nudge at point " << i << ", estimate_nudge_lat_movement "
              << estimate_nudge_lat_movement
              << ", estimate_nudge_lat_speed: " << estimate_nudge_lat_speed;
      if (estimate_nudge_lat_speed > av_lat_speed_thres) {
        VLOG(2) << "Uncomfortable left nudge lat speed! Threshold: "
                << av_lat_speed_thres;
        can_comfortably_nudge_from_left = false;
      }
    }
  }

  constexpr double kDistanceToPrevTrajBuffer = 0.6;
  const auto evaluate_prev_traj_and_object_traj =
      [&time_aligned_prev_traj_boxes, &nudge_info_vec]() -> bool {
    for (int idx = 0; idx < time_aligned_prev_traj_boxes->size() &&
                      idx < nudge_info_vec.size();
         ++idx) {
      const auto& av_box = (*time_aligned_prev_traj_boxes)[idx];
      const double dist_to_prev_traj =
          av_box.DistanceTo(*nudge_info_vec[idx].box);
      if (dist_to_prev_traj < kDistanceToPrevTrajBuffer) {
        return true;
      }
    }
    return false;
  };

  if (!can_comfortably_nudge_from_left) {
    if (!is_prev_traj_effective->has_value()) {
      *is_prev_traj_effective = IsTrajEffectiveAndFillTrajBoxes(
          plan_passage, path_sl_boundary, time_aligned_prev_traj, veh_geo,
          comfortable_nudge_check_time, comfortable_nudge_lat_speed_check_time,
          time_aligned_prev_traj_boxes);
    }
    if (*is_prev_traj_effective) {
      is_prev_traj_close_to_object_traj = evaluate_prev_traj_and_object_traj();
      if (!*is_prev_traj_close_to_object_traj) {
        can_comfortably_nudge_from_left = true;
      }
    }
  }
  if (can_comfortably_nudge_from_left && left_have_space) {
    const std::string l_judge_condition_l =
        std::string(traj.object_id()) + "can_comfortably_nudge_from_right: " +
        std::to_string(can_comfortably_nudge_from_left) +
        " left_have_space :" + std::to_string(left_have_space) +
        "can_comfortably_decel: "
        "index: " +
        std::to_string(first_left_no_space_index);
    VLOG(2) << "l_judge_condition_l" << l_judge_condition_l;
    return true;
  }

  constexpr double kComfortableDecel = -3.0;
  const auto can_comfortably_decel = [&plan_passage, &nudge_info_vec, &av_speed,
                                      &av_sl_box](int index) -> bool {
    if (index < nudge_info_vec.size()) {
      const auto& nudge_info = nudge_info_vec[index];
      const auto passage_theta_at_object_s =
          plan_passage.QueryTangentAngleAtS(nudge_info.s_min);
      if (passage_theta_at_object_s.ok()) {
        if (std::abs(NormalizeAngle(*passage_theta_at_object_s -
                                    nudge_info.theta)) < M_PI_2) {
          const double decelerate_time =
              std::min(nudge_info.t, av_speed / (-kComfortableDecel));
          const double decelerate_s =
              av_speed * decelerate_time +
              0.5 * kComfortableDecel * Sqr(decelerate_time);
          VLOG(2) << "Av can decelerate with constant acc(" << kComfortableDecel
                  << "m/s^2) for " << decelerate_time << "s, distance is "
                  << decelerate_s << ", object_min_s is " << nudge_info.s_min
                  << ", av_max_s is " << av_sl_box.s_max;
          if ((decelerate_s + av_sl_box.s_max) < nudge_info.s_min) {
            VLOG(2) << "AV may comfortably decelerate to safe position behind "
                       "object at time "
                    << nudge_info.t << "(s).";
            return true;
          }
        }
      }
    }
    return false;
  };
  if (can_comfortably_nudge_from_left && (!left_have_space) &&
      can_comfortably_decel(first_left_no_space_index)) {
    return true;
  }
  const std::string l_judge_condition =
      std::string(traj.object_id()) + "can_comfortably_nudge_from_right: " +
      std::to_string(can_comfortably_nudge_from_left) +
      " right_have_space :" + std::to_string(left_have_space) +
      "can_comfortably_decel: " +
      std::to_string(can_comfortably_decel(first_left_no_space_index)) +
      "index: " + std::to_string(first_left_no_space_index);

  VLOG(2) << "l_judge-condition" << l_judge_condition;

  bool can_comfortably_nudge_from_right = true, right_have_space = true;
  int first_right_no_space_index = std::numeric_limits<int>::infinity();
  VLOG(2) << "Check comfortable nudge from right.";
  for (int i = 0; i < nudge_info_vec.size(); ++i) {
    const auto& nudge_info = nudge_info_vec[i];
    if (!nudge_info.lon_overlapped) continue;
    if (right_have_space) {
      const double space =
          nudge_info.obj_l_min - nudge_info.path_l_min - av_width;
      if (space < lat_nudge_dist) {
        const std::string no_space =
            std::string(traj.object_id()) + "space: " + std::to_string(space) +
            " obj_l_min" + std::to_string(nudge_info.obj_l_min) +
            "path_l_min: " + std::to_string(nudge_info.path_l_min) +
            "lat_nudge_dist: " + std::to_string(lat_nudge_dist);

        VLOG(2) << "Not enough right nudge space at point " << i
                << ", obj_l_min: " << nudge_info.obj_l_min
                << ", path_l_min: " << nudge_info.path_l_min
                << ", av_width: " << av_width
                << ", lat_nudge_dist: " << lat_nudge_dist;
        right_have_space = false;
        first_right_no_space_index = i;
      }
    }

    if (traj.object_type() == ObjectType::OT_PEDESTRIAN) continue;
    if (nudge_info.t < kTimeEps) continue;
    if (nudge_info.t < comfortable_nudge_lat_speed_check_time) {
      const double estimate_nudge_lat_movement =
          std::max(av_center_l - nudge_info.obj_l_min + kMinimumNudgeBuffer +
                       av_half_width,
                   0.0);
      const double estimate_nudge_lat_speed =
          estimate_nudge_lat_movement / nudge_info.t;
      VLOG(2) << "Right nudge at point " << i
              << ", estimate_nudge_lat_movement " << estimate_nudge_lat_movement
              << ", estimate_nudge_lat_speed: " << estimate_nudge_lat_speed;
      if (estimate_nudge_lat_speed > av_lat_speed_thres) {
        VLOG(2) << "Uncomfortable right nudge lat speed! Threshold: "
                << av_lat_speed_thres;
        can_comfortably_nudge_from_right = false;
      }
    }
  }

  if (!can_comfortably_nudge_from_right) {
    if (!is_prev_traj_effective->has_value()) {
      *is_prev_traj_effective = IsTrajEffectiveAndFillTrajBoxes(
          plan_passage, path_sl_boundary, time_aligned_prev_traj, veh_geo,
          comfortable_nudge_check_time, comfortable_nudge_lat_speed_check_time,
          time_aligned_prev_traj_boxes);
    }
    if (*is_prev_traj_effective) {
      if (!is_prev_traj_close_to_object_traj.has_value()) {
        is_prev_traj_close_to_object_traj =
            evaluate_prev_traj_and_object_traj();
      }
      if (!*is_prev_traj_close_to_object_traj) {
        can_comfortably_nudge_from_right = true;
      }
    }
  }

  const std::string r_judge_condition =
      std::string(traj.object_id()) + "can_comfortably_nudge_from_right: " +
      std::to_string(can_comfortably_nudge_from_right) +
      " right_have_space: " + std::to_string(right_have_space) +
      "can_comfortably_decel: " +
      std::to_string(can_comfortably_decel(first_right_no_space_index)) +
      "index: " + std::to_string(first_right_no_space_index);

  if (can_comfortably_nudge_from_right && right_have_space) {
    VLOG(2) << "r_judge_condition_r" << r_judge_condition;
    return true;
  }

  VLOG(2) << "r_judge-condition" << r_judge_condition;
  if (can_comfortably_nudge_from_right && (!right_have_space) &&
      can_comfortably_decel(first_right_no_space_index)) {
    return true;
  }

  return false;
}

bool IsFrontOrSideObject(const PlanPassage& plan_passage,
                         const FrenetCoordinate& av_sl, double av_length,
                         const Box2d& bbox) {
  ASSIGN_OR_RETURN(const auto obj_frenet_box,
                   plan_passage.QueryFrenetBoxAt(bbox), false);

  if (av_sl.s + av_length * 0.5 < obj_frenet_box.s_max) {
    return true;
  }
  return false;
}

bool IsFrontOrSideObjectForEmergencySituation(const PlanPassage& plan_passage,
                                              const FrenetBox& av_sl_box,
                                              const Box2d& bbox) {
  ASSIGN_OR_RETURN(const auto obj_frenet_box,
                   plan_passage.QueryFrenetBoxAt(bbox), false);

  if (av_sl_box.s_max < obj_frenet_box.s_max) {
    return true;
  }
  return false;
}

}  // namespace

StationarySpacetimePlannerObjectTrajectoriesFinder::
    StationarySpacetimePlannerObjectTrajectoriesFinder(
        const PlannerSemanticMapManager* psmm,
        const mapping::LanePath& lane_path) {
  constexpr double kForwardDistance = 50.0;
  constexpr double kBoomBarrierBoxLength = 1.0;
  constexpr double kBoomBarrierBoxWidth = 2.0;
  const auto lanes_info = GetLanesInfoContinueIfNotFound(
      *psmm, lane_path.BeforeArclength(kForwardDistance));
  for (const auto& lane_info : lanes_info) {
    if (lane_info->endpoint_toll()) {
      CHECK_GE(lane_info->points().size(), 2);
      const auto& points = lane_info->points();
      const Vec2d end_vec(points.back() - points[points.size() - 2]);
      bool_barrier_box_or_ = Box2d(points.back(), end_vec.FastAngle(),
                                   kBoomBarrierBoxLength, kBoomBarrierBoxWidth);
    }
  }
}

SpacetimePlannerObjectTrajectoryReason::Type
StationarySpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  if (traj.is_stationary()) {
    if (!bool_barrier_box_or_.has_value() ||
        !traj.contour().HasOverlap(*bool_barrier_box_or_)) {
      return SpacetimePlannerObjectTrajectoryReason::STATIONARY;
    }
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  return SpacetimePlannerObjectTrajectoryReason::NONE;
}

FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder::
    FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder(
        const Box2d& av_box, const PlanPassage* plan_passage,
        const PathSlBoundary* sl_boundary, double av_speed,
        const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs,
        const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
        const VehicleGeometryParamsProto* veh_geo,
        const PlannerSemanticMapManager* psmm,
        const NudgeObjectInfo* nudge_object_info,
        const LaneChangeStateProto* lane_change_state)
    : av_box_(av_box),
      plan_passage_(CHECK_NOTNULL(plan_passage)),
      path_sl_boundary_(CHECK_NOTNULL(sl_boundary)),
      veh_geo_(CHECK_NOTNULL(veh_geo)),
      av_speed_(av_speed),
      time_aligned_prev_traj_(time_aligned_prev_traj),
      psmm_(psmm),
      nudge_object_info_(nudge_object_info),
      lane_change_state_(lane_change_state) {
  CHECK_NOTNULL(time_aligned_prev_traj);
  const auto av_sl_box_or = plan_passage_->QueryFrenetBoxAt(av_box_);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
  for (const auto& st_traj_proto : prev_st_trajs->trajectory()) {
    prev_st_planner_obj_id_.insert(st_traj_proto.id());
  }
}

SpacetimePlannerObjectTrajectoryReason::Type
FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  VLOG(2) << "Find trajectory " << traj.traj_id()
          << " by front-side spacetime planner object trajectory finder.";

  if (!av_sl_box_.has_value()) {
    VLOG(2) << "AV box can't be mapped on plan passage, skip.";
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  if (traj.is_stationary()) {
    VLOG(2) << "Trajectory is stationary, skip.";
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  const auto& states = traj.states();
  const bool prev_st_planner_obj = prev_st_planner_obj_id_.contains(
      traj.planner_object().is_sim_agent() ? traj.planner_object().base_id()
                                           : traj.planner_object().id());

  const auto lane_id = plan_passage_->lane_path().front().lane_id();
  const auto lane_info = psmm_->FindCurveLaneByIdOrNull(lane_id);

  bool is_split = false, is_merge = false;
  auto pre_lane_ids = lane_info->pre_lane_ids();
  if (pre_lane_ids.size() == 1) {
    const auto pre_lane_info = psmm_->FindCurveLaneByIdOrNull(pre_lane_ids[0]);
    if (pre_lane_info && pre_lane_info->next_lane_ids().size() >= 2) {
      is_split = true;
    }
  }

  auto next_lane_ids = lane_info->next_lane_ids();
  if (next_lane_ids.size() == 1) {
    const auto next_lane_info =
        psmm_->FindCurveLaneByIdOrNull(next_lane_ids[0]);
    if (next_lane_info && next_lane_info->pre_lane_ids().size() >= 2) {
      is_merge = true;
    }
  }

  bool is_uturn = false;
  if (lane_info && lane_info->turn_type() == ad_e2e::planning::U_TURN) {
    is_uturn = true;
  }
  bool use_ttc = (lane_info && !lane_info->junction_id().empty()) || is_split ||
                 is_merge || is_uturn;

  const std::string pre_left = "use_ttc: " + std::to_string(use_ttc) +
                               " prev_st_planner_obj" +
                               std::to_string(prev_st_planner_obj) +
                               "obs_id: " + std::string(traj.object_id());
  VLOG(2) << "pre_left" << pre_left;

  if (!IsFrontTrajectory(*psmm_, *path_sl_boundary_, *plan_passage_, av_box_,
                         *av_sl_box_, states[0].contour, traj, *veh_geo_,
                         nudge_object_info_, lane_change_state_,
                         prev_st_planner_obj, av_speed_, use_ttc)) {
    VLOG(2) << "Trajectory is not in front, skip.";
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  return SpacetimePlannerObjectTrajectoryReason::SIDE;
}

DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder::
    DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder(
        const Box2d& av_box, const PlanPassage* plan_passage,
        double av_velocity)
    : av_box_(av_box),
      plan_passage_(plan_passage),
      av_tangent_(av_box.tangent()),
      av_velocity_(av_velocity) {
  auto av_sl_box_or = plan_passage_->QueryFrenetBoxAt(av_box);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
}
SpacetimePlannerObjectTrajectoryReason::Type
DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  if (!av_sl_box_.has_value()) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  if (traj.is_stationary()) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  constexpr double kRelSpeedThres = 3.5;
  const Vec2d obj_v =
      traj.planner_object().pose().v() *
      Vec2d::FastUnitFromAngle(traj.planner_object().pose().theta());
  if (obj_v.Dot(av_tangent_) - av_velocity_ > kRelSpeedThres) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  if (av_box_.DistanceTo(traj.states()[0].box) > kCloseMovingObjThreshold) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  if (IsFrontOrSideObjectForEmergencySituation(*plan_passage_, *av_sl_box_,
                                               traj.states()[0].box)) {
    return SpacetimePlannerObjectTrajectoryReason::EMERGENCY_AVOIDANCE;
  }
  return SpacetimePlannerObjectTrajectoryReason::NONE;
}

FrontMovingSpacetimePlannerObjectTrajectoriesFinder::
    FrontMovingSpacetimePlannerObjectTrajectoriesFinder(
        const PlanPassage* plan_passage,
        const ApolloTrajectoryPointProto* plan_start_point, double av_length)
    : plan_passage_(plan_passage),
      plan_start_point_(plan_start_point),
      av_length_(av_length) {
  const Vec2d av_pos(plan_start_point_->path_point().x(),
                     plan_start_point_->path_point().y());
  auto av_sl_or = plan_passage_->QueryFrenetCoordinateAt(av_pos);
  CHECK(av_sl_or.ok());
  av_sl_ = *av_sl_or;
}

SpacetimePlannerObjectTrajectoryReason::Type
FrontMovingSpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  if (traj.is_stationary()) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  if (IsFrontOrSideObject(*plan_passage_, av_sl_, av_length_,
                          traj.states()[0].box)) {
    return SpacetimePlannerObjectTrajectoryReason::FRONT;
  }
  return SpacetimePlannerObjectTrajectoryReason::NONE;
}

}  // namespace planning
}  // namespace e2e_noa
