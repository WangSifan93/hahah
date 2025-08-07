#include "object/spacetime_planner_object_trajectories_filter.h"

#include <algorithm>
#include <ostream>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "common/log_data.h"
#include "glog/logging.h"
#include "math/linear_interpolation.h"
#include "math/util.h"
#include "object/spacetime_object_state.h"
#include "perception.pb.h"
#include "plan/planner_util.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"

namespace e2e_noa {
namespace planning {
namespace {

bool IsMaybeCutInVehicleTrajectory(
    const PlanPassage& plan_passage,
    const LaneChangeStateProto& lane_change_state, const Box2d& av_box,
    const FrenetBox& av_sl_box, double av_speed,
    const SpacetimeObjectTrajectory& traj) {
  if (lane_change_state.stage() != LCS_NONE) {
    return false;
  }
  if (traj.object_type() != ObjectType::OT_VEHICLE &&
      traj.object_type() != ObjectType::OT_LARGE_VEHICLE) {
    return false;
  }

  constexpr double kObjectDistThreshold = 0.8;
  const auto& object_box = traj.bounding_box();
  const double dist_to_av_box = object_box.DistanceTo(av_box);
  if (dist_to_av_box < kObjectDistThreshold) {
    VLOG(2) << traj.traj_id() << " dist to close " << dist_to_av_box << "m.";
    return false;
  }
  const auto& object_frenet_box = plan_passage.QueryFrenetBoxAt(object_box);
  if (!object_frenet_box.ok()) {
    return false;
  }
  const auto& states = traj.states();
  const auto& object_start_pt = *states.front().traj_point;
  const auto current_object_sl_pt =
      plan_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
          object_start_pt.pos());
  if (!current_object_sl_pt.ok()) {
    VLOG(2) << traj.traj_id()
            << " current pose not in plan_passage, skip evaluation.";
    return false;
  }
  const auto current_lane_heading =
      plan_passage.QueryTangentAngleAtS(current_object_sl_pt->s);
  if (!current_lane_heading.ok()) {
    VLOG(2) << traj.traj_id() << " current lane heading query failed.";
    return false;
  }

  constexpr double kLargeLatSpeed = 0.3;
  const double object_lat_speed =
      object_start_pt.v() *
      fast_math::Sin(object_start_pt.theta() - *current_lane_heading);
  const bool lat_speed_large = current_object_sl_pt->l > 0.0
                                   ? object_lat_speed < -kLargeLatSpeed
                                   : object_lat_speed > kLargeLatSpeed;

  constexpr double kObjectSExtent = 2.0;
  const bool cur_lon_overlapped =
      (av_sl_box.s_max > (object_frenet_box->s_min - kObjectSExtent) &&
       av_sl_box.s_max < (object_frenet_box->s_max + kObjectSExtent)) ||
      (av_sl_box.s_min > (object_frenet_box->s_min - kObjectSExtent) &&
       av_sl_box.s_min < (object_frenet_box->s_max + kObjectSExtent));

  if (lat_speed_large && cur_lon_overlapped &&
      dist_to_av_box < kObjectSExtent) {
    VLOG(2) << traj.traj_id() << " may be immoral.";
    return false;
  }

  const auto final_object_sl_pt =
      plan_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
          states.back().traj_point->pos());

  if (!final_object_sl_pt.ok()) {
    VLOG(2) << traj.traj_id()
            << " final pose not in plan_passage, skip evaluation.";
    return false;
  }

  const auto current_lane_boundary_offset =
      plan_passage.QueryNearestBoundaryLateralOffset(current_object_sl_pt->s);
  if (!current_lane_boundary_offset.ok()) {
    VLOG(2) << traj.traj_id() << " current lateral boundary query failed.";
    return false;
  }
  if (current_object_sl_pt->l < current_lane_boundary_offset->second &&
      current_object_sl_pt->l > current_lane_boundary_offset->first) {
    VLOG(2) << traj.traj_id() << " object in lane.";
    return false;
  }
  const auto final_lane_boundary_offset =
      plan_passage.QueryNearestBoundaryLateralOffset(final_object_sl_pt->s);
  if (!final_lane_boundary_offset.ok()) {
    VLOG(2) << traj.traj_id() << " fianl lateral boundary query failed.";
    return false;
  }

  constexpr double kCenterOffsetThreshold = 0.8;
  bool is_traj_slope_to_center = false;

  if (current_object_sl_pt->l < current_lane_boundary_offset->first) {
    is_traj_slope_to_center =
        final_object_sl_pt->l > current_lane_boundary_offset->first &&
        final_object_sl_pt->l > current_object_sl_pt->l;
  } else if (current_object_sl_pt->l > current_lane_boundary_offset->second) {
    is_traj_slope_to_center =
        final_object_sl_pt->l < current_lane_boundary_offset->second &&
        final_object_sl_pt->l < current_object_sl_pt->l;
  }
  if (is_traj_slope_to_center) {
    constexpr double kCutInThetaThreshold = 0.05;
    const double toward_center_theta_diff =
        current_object_sl_pt->l > 0.0
            ? NormalizeAngle(*current_lane_heading - traj.pose().theta())
            : NormalizeAngle(traj.pose().theta() - *current_lane_heading);
    if (toward_center_theta_diff > kCutInThetaThreshold) {
      return true;
    }

    const double toward_center_offset =
        current_object_sl_pt->l > 0.0
            ? current_object_sl_pt->l - final_object_sl_pt->l
            : final_object_sl_pt->l - current_object_sl_pt->l;
    VLOG(3) << traj.traj_id()
            << " toward_center_offset: " << toward_center_offset << " "
            << toward_center_theta_diff;
    constexpr double kMaybeCutInThetaThreshold = 0.01;

    if (toward_center_offset > kCenterOffsetThreshold &&
        toward_center_theta_diff > kMaybeCutInThetaThreshold) {
      VLOG(2) << traj.traj_id() << " prediction toward center offset "
              << toward_center_offset << "m > " << kCenterOffsetThreshold
              << "m, toward center theta diff is " << kMaybeCutInThetaThreshold
              << "rad, maybe cutin traj, refuse to nudge.";
      return true;
    }
  }

  return false;
}

bool IsVru(const SpacetimeObjectTrajectory& traj) {
  return traj.object_type() == ObjectType::OT_MOTORCYCLIST ||
         traj.object_type() == ObjectType::OT_CYCLIST ||
         traj.object_type() == ObjectType::OT_TRICYCLIST ||
         traj.object_type() == ObjectType::OT_PEDESTRIAN;
}

bool IsCrossingTrajectory(const PlanPassage& plan_passage,
                          const PlannerSemanticMapManager& psmm,
                          const SpacetimeObjectTrajectory& traj) {
  if (traj.is_stationary() || IsStaticObjectType(traj.object_type())) {
    return false;
  }
  const bool is_vru = IsVru(traj);
  const auto cur_lane_id = plan_passage.lane_path().front().lane_id();
  const auto cur_lane_info = psmm.FindCurveLaneByIdOrNull(cur_lane_id);
  bool is_left_turn = false;
  bool is_intersection_straight = false;
  if (cur_lane_info &&
      cur_lane_info->turn_type() == ad_e2e::planning::LEFT_TURN) {
    is_left_turn = true;
  }
  bool is_in_intersection =
      cur_lane_info && !cur_lane_info->junction_id().empty();
  if (is_in_intersection &&
      cur_lane_info->turn_type() == ad_e2e::planning::NO_TURN) {
    is_intersection_straight = true;
  }
  const auto& traj_start_point = *traj.states().front().traj_point;
  const auto frenet_start_point =
      plan_passage.QueryUnboundedFrenetCoordinateAt(traj_start_point.pos());
  if (!frenet_start_point.ok()) {
    return false;
  }

  if (!is_vru && is_intersection_straight && traj_start_point.v() < 4.0) {
    const auto frenet_end_point = plan_passage.QueryUnboundedFrenetCoordinateAt(
        (*traj.states().back().traj_point).pos());
    if (frenet_end_point.ok() &&
        (frenet_start_point->l * frenet_end_point->l > 0.0 ||
         std::fabs(frenet_end_point->l) < 1.5)) {
      return false;
    }
  }
  const auto lane_theta_at_pose =
      plan_passage.QueryTangentAngleAtS(frenet_start_point->s);
  if (!lane_theta_at_pose.ok()) {
    return false;
  }

  const double lane_normal_theta_at_pose =
      NormalizeAngle(*lane_theta_at_pose + M_PI_2);
  const double lane_negative_normal_theta_at_pose =
      NormalizeAngle(*lane_theta_at_pose - M_PI_2);

  const bool obs_reverse_flag =
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj_start_point.theta())) < M_PI / 6;

  if (obs_reverse_flag) {
    is_intersection_straight = false;
    is_in_intersection = false;
  }

  if (is_vru && (!(obs_reverse_flag ||
                   std::abs(NormalizeAngle(*lane_theta_at_pose -
                                           traj_start_point.theta())) <
                       M_PI / 6 + 1e-3))) {
    return true;
  }

  const std::vector<double> intersection_thresholds = {1.57, 1.57, 1.57};
  const std::vector<double> theta_diff_thresholds = {1.0, 0.78, 0.52};
  const std::vector<double> ratio_limits = {0.8, 0.6, 0.4};
  const int level_size = theta_diff_thresholds.size();
  std::vector<int> level_counts(level_size, 0);

  const auto& states = traj.states();
  const int states_size = is_vru ? states.size() / 2 : states.size();
  for (int i = 0; i < states_size; ++i) {
    const auto& state = states[i];
    const double theta = state.traj_point->theta();
    const double theta_diff_to_lane_normal =
        NormalizeAngle(theta - lane_normal_theta_at_pose);
    const double theta_diff_to_negative_lane_normal =
        NormalizeAngle(lane_negative_normal_theta_at_pose - theta);
    for (int j = 0; j < level_size; ++j) {
      if (!is_vru && is_left_turn && 0.0 < theta_diff_to_lane_normal &&
          theta_diff_to_lane_normal < intersection_thresholds[j]) {
        level_counts[j]++;
      } else if (!is_vru && is_intersection_straight &&
                 0.0 < theta_diff_to_negative_lane_normal &&
                 theta_diff_to_negative_lane_normal <
                     intersection_thresholds[j]) {
        level_counts[j]++;
      } else if (std::abs(theta_diff_to_lane_normal) <
                     theta_diff_thresholds[j] ||
                 std::abs(theta_diff_to_negative_lane_normal) <
                     theta_diff_thresholds[j]) {
        level_counts[j]++;
      }
    }
  }
  for (int j = 0; j < level_size; ++j) {
    const double ratio =
        static_cast<double>(level_counts[j]) / static_cast<double>(states_size);
    if (ratio < ratio_limits[j]) {
      return false;
    }
  }
  return true;
}

bool IsTrajectoryBeyondStopLine(const PlanPassage& plan_passage,
                                double first_stop_line_s,
                                const SpacetimeObjectTrajectory& traj) {
  if (std::isinf(first_stop_line_s)) {
    return false;
  }
  if (traj.is_stationary()) {
    const auto frenet_box = plan_passage.QueryFrenetBoxAt(traj.bounding_box());
    if (!frenet_box.ok()) {
      return false;
    }
    if (frenet_box->s_min < first_stop_line_s) {
      return false;
    }
  } else {
    const auto& states = traj.states();
    for (const auto& state : states) {
      const auto frenet_box = plan_passage.QueryFrenetBoxAt(state.box);
      if (!frenet_box.ok()) {
        continue;
      }
      if (frenet_box->s_min < first_stop_line_s) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

bool IsCutInObjectTrajectory(const PlanPassage& plan_passage,
                             const bool is_lane_change, const double av_speed,
                             const FrenetBox& av_sl_box,
                             const SpacetimeObjectTrajectory& traj) {
  if (is_lane_change) return false;

  if (traj.object_type() != ObjectType::OT_VEHICLE &&
      traj.object_type() != ObjectType::OT_LARGE_VEHICLE) {
    return false;
  }

  const double obj_speed = traj.pose().v();
  if (av_speed - obj_speed < 1.5) {
    return false;
  }

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
  const bool obs_reverse_flag =
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj.states().front().traj_point->theta())) <
      M_PI / 3;
  if (obs_reverse_flag) {
    return false;
  }

  const double kObjectDistThreshold = 0.8;
  const double kBrakeAcc = 3.0;
  const double min_brake_dist =
      (av_speed - obj_speed) * (av_speed - obj_speed) / (kBrakeAcc * 2.0);
  const auto& object_box = traj.bounding_box();
  const auto& obj_frenet_box = plan_passage.QueryFrenetBoxAt(object_box);
  if (!obj_frenet_box.ok()) {
    return false;
  }
  if (obj_frenet_box->s_min - av_sl_box.s_max <
      std::fmax(kObjectDistThreshold, min_brake_dist)) {
    return false;
  }

  const auto& object_frenet_box = plan_passage.QueryFrenetBoxAt(object_box);
  if (!object_frenet_box.ok()) {
    return false;
  }

  const double center_range = 1.5;
  if (object_frenet_box->l_min >
          0.6 * ad_e2e::planning::Constants::DEFAULT_LANE_WIDTH ||
      object_frenet_box->l_max <
          -0.6 * ad_e2e::planning::Constants::DEFAULT_LANE_WIDTH ||
      std::fabs(object_frenet_box->center_l()) < center_range) {
    return false;
  }

  const double cutin_l_threshold = 1.0;
  const double delta_l_threshold = 0.2;
  double prev_l = object_frenet_box->l_min > ad_e2e::planning::Constants::ZERO
                      ? object_frenet_box->l_min
                      : object_frenet_box->l_max;
  for (const auto& obj_state : traj.states()) {
    const auto pred_box = plan_passage.QueryFrenetBoxAt(obj_state.box);
    if (!pred_box.ok()) {
      break;
    }
    if (object_frenet_box->center_l() > ad_e2e::planning::Constants::ZERO) {
      if (pred_box->l_min < cutin_l_threshold &&
          object_frenet_box->l_min - pred_box->l_min > delta_l_threshold) {
        std::string cutin_filter_msg = " cutin filter: ";
        cutin_filter_msg += std::string(traj.object_id());
        Log2FG::LogDataV2("cutin_filter", cutin_filter_msg);
        return true;
      }
      prev_l = pred_box->l_min;
    } else if (object_frenet_box->center_l() <
               -ad_e2e::planning::Constants::ZERO) {
      if (pred_box->l_max > -cutin_l_threshold &&
          pred_box->l_max - object_frenet_box->l_max > delta_l_threshold) {
        std::string cutin_filter_msg = " cutin filter: ";
        cutin_filter_msg += std::string(traj.object_id());
        Log2FG::LogDataV2("cutin_filter", cutin_filter_msg);
        return true;
      }
      prev_l = pred_box->l_max;
    }
  }

  return false;
}

CutInSpacetimePlannerObjectTrajectoriesFilter::
    CutInSpacetimePlannerObjectTrajectoriesFilter(
        const PlanPassage* plan_passage,
        const LaneChangeStateProto* lane_change_state, const Box2d& av_box,
        double av_speed)
    : plan_passage_(plan_passage),
      lane_change_state_(lane_change_state),
      av_box_(av_box),
      av_speed_(av_speed) {
  const auto av_sl_box_or = plan_passage_->QueryFrenetBoxAt(av_box_);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
}

bool CutInSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (!av_sl_box_.has_value()) {
    VLOG(2) << "AV box can't be mapped on plan passage, skip.";
    return false;
  }
  const bool is_lane_change =
      ((*lane_change_state_).stage() == LaneChangeStage::LCS_EXECUTING ||
       (*lane_change_state_).stage() == LaneChangeStage::LCS_RETURN ||
       (*lane_change_state_).stage() == LaneChangeStage::LCS_PAUSE);
  return IsCutInObjectTrajectory(*plan_passage_, is_lane_change, av_speed_,
                                 *av_sl_box_, traj);
}

CutInVehicleSpacetimePlannerObjectTrajectoriesFilter::
    CutInVehicleSpacetimePlannerObjectTrajectoriesFilter(
        const PlanPassage* plan_passage,
        const LaneChangeStateProto* lane_change_state, const Box2d& av_box,
        double av_speed)
    : plan_passage_(plan_passage),
      lane_change_state_(lane_change_state),
      av_box_(av_box),
      av_speed_(av_speed) {
  const auto av_sl_box_or = plan_passage_->QueryFrenetBoxAt(av_box_);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
}

bool CutInVehicleSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (!av_sl_box_.has_value()) {
    VLOG(2) << "AV box can't be mapped on plan passage, skip.";
    return false;
  }
  return IsMaybeCutInVehicleTrajectory(*plan_passage_, *lane_change_state_,
                                       av_box_, *av_sl_box_, av_speed_, traj);
}

CrossingSpacetimePlannerObjectTrajectoriesFilter::
    CrossingSpacetimePlannerObjectTrajectoriesFilter(
        const PlanPassage* plan_passage, const PlannerSemanticMapManager* psmm)
    : plan_passage_(plan_passage), psmm_(psmm) {}

bool CrossingSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (IsCrossingTrajectory(*plan_passage_, *psmm_, traj)) {
    Log2FG::LogDataV2("Cross_filter", absl::StrCat(traj.traj_id()));
    return true;
  }
  return false;
}

BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter::
    BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter(
        const PlanPassage* plan_passage,
        absl::Span<const ConstraintProto::StopLineProto> stop_lines)
    : plan_passage_(plan_passage) {
  for (const auto& stop_line : stop_lines) {
    first_stop_line_s_ =
        std::min(first_stop_line_s_, stop_line.s() - stop_line.standoff());
  }
}

bool BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  return IsTrajectoryBeyondStopLine(*plan_passage_, first_stop_line_s_, traj);
}

ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter::
    ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter(
        const PlanPassage* plan_passage, const PlannerSemanticMapManager* psmm,
        const VehicleGeometryParamsProto* vehicle_geometry_params,
        const PathSlBoundary* sl_boundary,
        const NudgeObjectInfo* nudge_object_info, Box2d av_box, double av_speed)
    : plan_passage_(plan_passage),
      psmm_(psmm),
      vehicle_geometry_params_(vehicle_geometry_params),
      sl_boundary_(sl_boundary),
      nudge_object_info_(nudge_object_info),
      av_box_(std::move(av_box)),
      av_speed_(av_speed) {}

bool ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (traj.object_type() != ObjectType::OT_VEHICLE &&
      traj.object_type() != ObjectType::OT_LARGE_VEHICLE) {
    return false;
  }

  const auto ego_frenet_box_outut = plan_passage_->QueryFrenetBoxAt(av_box_);
  if (!ego_frenet_box_outut.ok()) return false;
  const auto& ego_frenet_box = ego_frenet_box_outut.value();

  const auto object_frenet_box_output =
      plan_passage_->QueryFrenetBoxAtContour(traj.contour());
  if (!object_frenet_box_output.ok()) return false;
  const auto& object_frenet_box = object_frenet_box_output.value();
  const double object_s = object_frenet_box.center_s();

  const auto cur_lane_id = plan_passage_->lane_path().front().lane_id();
  const auto cur_lane_info = psmm_->FindCurveLaneByIdOrNull(cur_lane_id);
  const bool is_in_intersection =
      cur_lane_info && !cur_lane_info->junction_id().empty();
  bool is_left_turn = false;
  if (cur_lane_info &&
      cur_lane_info->turn_type() == ad_e2e::planning::LEFT_TURN) {
    is_left_turn = true;
  }

  const auto lane_theta_at_pose = plan_passage_->QueryTangentAngleAtS(object_s);
  if (!lane_theta_at_pose.ok()) {
    return false;
  }

  const auto& ego_nearest_station = plan_passage_->FindNearestStationAtS(
      ego_frenet_box.center_s() + std::clamp(5 * av_speed_, 10.0, 100.0));

  if (ego_nearest_station.is_virtual() && is_left_turn && is_in_intersection &&
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj.pose().theta())) < M_PI / 2) {
    Log2FG::LogDataV2(
        "reverse_filter",
        absl::StrCat("ego_turn_left,ignore:", traj.object_id(),
                     ",far_distance:", std::clamp(5 * av_speed_, 10.0, 100.0)));
    return true;
  }
  const auto& nearest_station = plan_passage_->FindNearestStationAtS(object_s);
  if (nearest_station.is_virtual()) {
    return false;
  }

  if (object_frenet_box.s_min < ego_frenet_box.s_max) return false;

  constexpr double kMaxHalfLaneWidth = 4.5;
  constexpr double kMinHalfLaneWidth = 1.2;
  constexpr double kSampleStepAlongS = 1.0;
  double initial_left_boundary = std::numeric_limits<double>::infinity();
  double initial_right_boundary = -std::numeric_limits<double>::infinity();

  constexpr bool use_out_boundary = true;
  for (double sample_s = object_frenet_box.s_min;
       sample_s <= object_frenet_box.s_max; sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] =
        use_out_boundary ? sl_boundary_->QueryBoundaryL(sample_s)
                         : sl_boundary_->QueryTargetBoundaryL(sample_s);
    initial_left_boundary = std::min(initial_left_boundary, left_l);
    initial_right_boundary = std::max(initial_right_boundary, right_l);
  }

  const double left_boundary =
      std::clamp(initial_left_boundary, kMinHalfLaneWidth, kMaxHalfLaneWidth);
  const double right_boundary = std::clamp(
      initial_right_boundary, -kMaxHalfLaneWidth, -kMinHalfLaneWidth);

  constexpr double kThetaThreshold = 0.25;
  const double kLatSafeDistanceThreshold =
      nudge_object_info_ && (nudge_object_info_->nudge_state ==
                                 NudgeObjectInfo::NudgeState::NUDGE ||
                             nudge_object_info_->nudge_state ==
                                 NudgeObjectInfo::NudgeState::BORROW)
          ? 0.0
          : 0.2;

  const double ego_obs_delta_lon_dis =
      object_frenet_box.s_min - ego_frenet_box.s_max;
  auto const lon_dis_buffer = ad_e2e::planning::math::lerp(
      0.0, 10.0, 0.6, 60.0, ego_obs_delta_lon_dis, true);
  const bool object_overlap_with_safe_boundary =
      object_frenet_box.center_l() > 0.0
          ? object_frenet_box.l_min >
                right_boundary + vehicle_geometry_params_->width() +
                    kLatSafeDistanceThreshold - lon_dis_buffer
          : object_frenet_box.l_max <
                left_boundary - vehicle_geometry_params_->width() -
                    kLatSafeDistanceThreshold + lon_dis_buffer;

  const double object_relative_tangent_opposite = std::fabs(NormalizeAngle(
      nearest_station.tangent().FastAngle() + M_PI - traj.pose().theta()));

  if (object_frenet_box.center_l() > right_boundary &&
      object_frenet_box.center_l() < 0.0 && traj.pose().v() > Kph2Mps(15.0) &&
      object_relative_tangent_opposite < kThetaThreshold) {
    Log2FG::LogDataV2("reverse_filter",
                      absl::StrCat("fast right:", traj.object_id()));
    return true;
  }

  if (!object_overlap_with_safe_boundary &&
      object_relative_tangent_opposite < kThetaThreshold &&
      !traj.is_stationary()) {
    Log2FG::LogDataV2("reverse_filter",
                      absl::StrCat("no space:", traj.object_id()));
    return true;
  }

  return false;
}

}  // namespace planning
}  // namespace e2e_noa
