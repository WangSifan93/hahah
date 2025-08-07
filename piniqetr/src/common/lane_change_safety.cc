#include "common/lane_change_safety.h"

#include <float.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <ostream>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/box2d.h"
#include "math/geometry/util.h"
#include "math/math_utils.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/planner_defs.h"
#include "plan/planner_flags.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction.h"
#include "util/status_builder.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa::planning {

constexpr double kEnterTargetLateralThreshold = 1.5;
constexpr double kFullyEnterTargetLateralThreshold = 1.2;
constexpr double kStaticEnterTargetLateralThreshold = 0.6;

constexpr double kMaxAllowedDecelForObject = 1.5;
constexpr double kMaxAllowedDecelForEgo = 2.5;

constexpr double kMinLonBufferToFront = 2.0;
constexpr double kMinLatBufferToCloseBlocking = 0.8;
constexpr double kLatThresholdToCloseBlocking = 0.5;

constexpr double kFollowerStandardResponseTime = 0.6;
constexpr double kEgoResponseTime = 0.4;
constexpr double kEgoFollowTimeBufferPrepare = 0.5;
constexpr double kEgoLeadTimeBufferPrepare = 0.5;
constexpr double kEgoFollowTimeBufferProcess = 0.45;
constexpr double kEgoLeadTimeBufferProcess = 0.45;

constexpr double kEpsilon = 1e-5;
const std::vector<double> MIN_FRONT_DISTANCE_VEC = {1.2, 2.0, 2.5, 3.0,
                                                    4.0, 5.0, 10.0};
const std::vector<double> MIN_BACK_DISTANCE_VEC = {0.8, 1.0, 1.5, 2.5,
                                                   3.8, 5.0, 10.0};
const std::vector<double> V_EGO_VEHICLE_VEC = {0.0, 15, 20, 30, 60, 90, 120};

const std::vector<double> K_FRONT_HEAD_WAY_VEC = {0.70, 0.73, 0.76, 0.78,
                                                  0.86, 1.0,  1.0};
const std::vector<double> K_BACK_HEAD_WAY_VEC = {0.67, 0.70, 0.73, 0.78,
                                                 0.86, 1.0,  1.0};
const std::vector<double> V_HEAD_WAY_VEC = {0.0, 15, 20, 30, 60, 90, 120};

const double kMinCompensationFactor = 0.38;
const double kMaxCompensationFactor = 1.0;

const double kRearObjIgnoreLatOverlapFactor = 0.2;
const double kLaneChangeMinLimitTTC = 2.0;
double kExtremelySlowVehicleSpeedkph = 7.0;
double kStationaryVehicleSpeedkph = 1.5;
double g_MinLonBuffer = 2.0;

const std::vector<double> F_DV_FRONT_DEC_VEC = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
const std::vector<double> F_DV_STATIC_DEC_VEC = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
const std::vector<double> F_DV_BACK_DEC_VEC = {0.6, 1.0, 1.35, 1.7, 2.0, 3.0};
const std::vector<double> DV_TTC_VEC = {0.0, 10, 20, 30, 40, 120};

const std::vector<double> kEenterTargetLaneLatDis = {0.6, 0.5,  0.4, 0.3,
                                                     0.0, -0.3, -0.5};
const std::vector<double> kEgoVehSpdKphVct = {0.0, 15, 20, 30, 60, 90, 120};

std::vector<double> occupied_width_vec = {0.9, 0.8, 0.7, 0.6, 0.5};
std::vector<double> rel_distance_vec = {0.0, 30, 60, 80, 120};

bool HasEnteredTargetLane(const double center_l, const double half_width) {
  return std::abs(center_l) < kEnterTargetLateralThreshold + half_width;
}

bool HasEnteredTargetLane(const FrenetBox& obj_box, const double lane_width) {
  bool has_entered = false;

  if (obj_box.l_max * obj_box.l_min > 0) {
    double min_dis_target =
        std::min(std::fabs(obj_box.l_max), std::fabs(obj_box.l_min));
    double intrusion_dis_thrd =
        std::clamp((0.38 * (lane_width / kDefaultLaneWidth)), 0.2, 0.5);
    has_entered = min_dis_target < lane_width * 0.5 - intrusion_dis_thrd;
  } else {
    has_entered = true;
  }
  return has_entered;
}

bool HasFullyEnteredTargetLane(const double center_l, const double half_width) {
  return std::abs(center_l) < kFullyEnterTargetLateralThreshold + half_width;
}

bool HasFullyEnteredTargetLane(const double center_l, const double half_width,
                               const double& lateral_threshold) {
  return std::abs(center_l) < lateral_threshold + half_width;
}

bool HasFullyEnteredTargetLane(const FrenetBox& obj_box,
                               const double half_width) {
  bool corner_has_entered = false;
  bool center_has_entered = false;

  if (obj_box.l_max * obj_box.l_min > 0) {
    double min_dis_target =
        std::min(std::fabs(obj_box.l_max), std::fabs(obj_box.l_min));
    corner_has_entered =
        std::fabs(min_dis_target) < kFullyEnterTargetLateralThreshold * 0.8;
  } else {
    corner_has_entered = true;
  }

  center_has_entered = std::abs(obj_box.center_l()) <
                       kFullyEnterTargetLateralThreshold + half_width;
  return corner_has_entered || center_has_entered;
}

bool HasFullyCenteredInTargetLane(const FrenetBox& obj_box,
                                  const double half_width) {
  bool corner_has_centered = false;
  bool center_has_entered = false;

  if (obj_box.l_max * obj_box.l_min < 0) {
    corner_has_centered = true;
  } else {
    corner_has_centered = false;
  }

  center_has_entered =
      std::abs(obj_box.center_l()) < kFullyEnterTargetLateralThreshold * 0.5;
  return corner_has_centered && center_has_entered;
}

bool isEgoFullyOccupyTargetLane(const FrenetBox& ego_frenet_box,
                                const double ego_v_kph, const bool lc_left) {
  double ref_center_l = 0;
  const double ref_l_min = ego_frenet_box.l_min - ref_center_l;
  const double ref_l_max = ego_frenet_box.l_max - ref_center_l;
  double enter_target_lane_dis_thrd = ad_e2e::planning::math::interp1_inc(
      kEgoVehSpdKphVct, kEenterTargetLaneLatDis, ego_v_kph);

  bool ego_entered_target_lane =
      (lc_left ? ref_l_max > enter_target_lane_dis_thrd
               : ref_l_min < -enter_target_lane_dis_thrd);

  return ego_entered_target_lane;
}

bool isStaticObsRideLine(const FrenetBox& ego_box, const FrenetBox& obj_box,
                         const bool lc_left, const double rel_ds) {
  double occupied_width_limit = ad_e2e::planning::math::interp1_inc(
      rel_distance_vec, occupied_width_vec, rel_ds);

  bool ride_line = lc_left ? (obj_box.l_max <= -1.3 * occupied_width_limit &&
                              obj_box.l_min >= ego_box.l_max)
                           : (obj_box.l_min >= 1.3 * occupied_width_limit &&
                              obj_box.l_max <= ego_box.l_min);
  return ride_line;
}

inline bool IsVehicle(
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory) {
    return false;
  }
  auto obs_type = obstacle_trajectory->object_type();
  bool is_vehicle = (obs_type == OT_VEHICLE || obs_type == OT_LARGE_VEHICLE ||
                     obs_type == OT_TRICYCLIST);
  return is_vehicle;
}

inline bool IsVehicle(const PlannerObject* const obstacle_ptr) {
  if (!obstacle_ptr) {
    return false;
  }
  auto obs_type = obstacle_ptr->type();
  bool is_vehicle = (obs_type == OT_VEHICLE || obs_type == OT_LARGE_VEHICLE ||
                     obs_type == OT_TRICYCLIST);
  return is_vehicle;
}

bool isStaticObsOccupyTargetLane(
    const SpacetimeObjectTrajectory* const obstacle_trajectory,
    const FrenetBox& obj_box, const FrenetBox& ego_box, const double half_width,
    const bool lc_left, const double rel_ds) {
  const bool is_vehicle = IsVehicle(obstacle_trajectory);
  double occupied_width_limit = ad_e2e::planning::math::interp1_inc(
      rel_distance_vec, occupied_width_vec, rel_ds);

  bool corner_has_entered = false;
  bool center_has_entered = false;
  double min_dis_target =
      std::min(std::fabs(obj_box.l_max), std::fabs(obj_box.l_min));

  bool far_away_ego_obs =
      (obj_box.l_max * obj_box.l_min > 0) &&
      (lc_left ? (obj_box.l_min > 0.3) : (obj_box.l_max < -0.3));

  if (is_vehicle && far_away_ego_obs) {
    return false;
  }

  if (obj_box.l_max * obj_box.l_min > 0) {
    bool close_ego_side_obs =
        lc_left ? (obj_box.l_max < 0) : (obj_box.l_min > 0);
    corner_has_entered =
        std::fabs(min_dis_target) <
        occupied_width_limit * (close_ego_side_obs ? 1.0 : 0.8);
  } else {
    corner_has_entered = true;
  }

  center_has_entered = std::abs(obj_box.center_l()) <
                       kStaticEnterTargetLateralThreshold + half_width;
  return corner_has_entered || center_has_entered;
}

namespace {
inline bool IsBigVehicle(
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory) {
    return false;
  }
  auto obs_type = obstacle_trajectory->object_type();
  double obs_length = !obstacle_trajectory->states().empty()
                          ? obstacle_trajectory->states().front().box.length()
                          : 5.0;
  double obs_width = !obstacle_trajectory->states().empty()
                         ? obstacle_trajectory->states().front().box.width()
                         : 2.0;
  bool is_big_vehicle =
      (obs_type == OT_LARGE_VEHICLE &&
       (obs_length > 6.0 || obs_width > 2.6 || obs_length * obs_width > 11.6));
  return is_big_vehicle;
}

inline bool IsVRU(const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory) {
    return false;
  }
  auto obs_type = obstacle_trajectory->object_type();
  bool is_vru = (obs_type == OT_MOTORCYCLIST || obs_type == OT_PEDESTRIAN ||
                 obs_type == OT_CYCLIST);
  return is_vru;
}

inline bool IsConstructionObs(
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory) {
    return false;
  }
  auto obs_type = obstacle_trajectory->object_type();
  bool is_cst = (obs_type == OT_UNKNOWN_STATIC ||
                 obs_type == OT_UNKNOWN_MOVABLE || obs_type == OT_CONE);
  return is_cst;
}

inline bool IsSizeQualified(
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory || obstacle_trajectory->states().empty()) {
    return false;
  }

  auto obs_length = obstacle_trajectory->states().front().box.length();
  auto obs_width = obstacle_trajectory->states().front().box.width();
  auto obs_height = 1.0;

  bool size_qualified = false;
  if (obs_length >= 0.3 || obs_width >= 0.3 || obs_height >= 0.3 ||
      (obs_length * obs_width * obs_height >= 0.003)) {
    size_qualified = true;
  }

  return size_qualified;
}

inline double getMinDisCompensation(bool if_obs_front, bool is_big_vehicle,
                                    const Box2d& obj_cur_box) {
  bool frt_big_veh = is_big_vehicle && if_obs_front;
  double min_dis_compensation = 0.0;

  if (frt_big_veh) {
    double ego_veh_length = 5;
    double factor =
        std::floor(obj_cur_box.length() / ((2 * ego_veh_length) + kEpsilon));
    min_dis_compensation = std::clamp((factor * 0.5), 0.0, 3.0);
  }

  return min_dis_compensation;
}

inline double QuadraticLerp(double x0, double t0, double x1, double t1,
                            double t, bool clamp = true) {
  if (std::abs(t1 - t0) <= 0.0001) {
    return x0;
  }
  if (t0 > t1) {
    std::swap(t0, t1);
    std::swap(x0, x1);
  }
  if (clamp) {
    if (t0 <= t1) {
      if (t <= t0) {
        return x0;
      }
      if (t >= t1) {
        return x1;
      }
    }
  }

  if (t0 > t1) {
    std::swap(t0, t1);
    std::swap(x0, x1);
  }

  const double a = (x0 - x1) / pow(t0 - t1, 2);
  const double b = x1;

  const double x = a * pow(t - t1, 2) + b;
  return x;
}

double getLongOverlap(const e2e_noa::FrenetBox& ego_cur_frenet_box,
                      const e2e_noa::FrenetBox& obj_cur_frenet_box,
                      double& rel_ds) {
  const bool if_obs_front =
      (ego_cur_frenet_box.s_max < obj_cur_frenet_box.s_min);
  const double long_overlap =
      std::min(ego_cur_frenet_box.s_max, obj_cur_frenet_box.s_max) -
      std::max(ego_cur_frenet_box.s_min, obj_cur_frenet_box.s_min);
  rel_ds =
      (long_overlap >= 0) ? 0 : (if_obs_front ? -long_overlap : long_overlap);
  return long_overlap;
}

double getLatOverlap(const e2e_noa::FrenetBox& ego_cur_frenet_box,
                     const e2e_noa::FrenetBox& obj_cur_frenet_box,
                     double& rel_dl) {
  const bool if_obs_left =
      (ego_cur_frenet_box.l_max < obj_cur_frenet_box.l_min);
  const double lat_overlap =
      std::min(ego_cur_frenet_box.l_max, obj_cur_frenet_box.l_max) -
      std::max(ego_cur_frenet_box.l_min, obj_cur_frenet_box.l_min);
  rel_dl = (lat_overlap >= 0) ? 0 : (if_obs_left ? -lat_overlap : lat_overlap);
  return lat_overlap;
};

double getLatOverlapWithLcDirection(
    const e2e_noa::FrenetBox& ego_cur_frenet_box,
    const e2e_noa::FrenetBox& obj_cur_frenet_box, double& rel_dl) {
  const double kLcDirectionMaxSpacingDis = 3.75;
  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  double lat_overlap_with_direction = 0;
  const bool if_obs_left =
      (ego_cur_frenet_box.l_max < obj_cur_frenet_box.l_min);
  const double lat_overlap =
      std::min(ego_cur_frenet_box.l_max, obj_cur_frenet_box.l_max) -
      std::max(ego_cur_frenet_box.l_min, obj_cur_frenet_box.l_min);
  rel_dl = (lat_overlap >= 0) ? 0 : (if_obs_left ? -lat_overlap : lat_overlap);

  if (lc_left &&
      (lat_overlap >= 0 ||
       (rel_dl > 0 && obj_cur_frenet_box.l_min - ego_cur_frenet_box.l_max <
                          kLcDirectionMaxSpacingDis))) {
    lat_overlap_with_direction =
        obj_cur_frenet_box.l_max - ego_cur_frenet_box.l_min;
  } else if (!lc_left && (lat_overlap >= 0 ||
                          (rel_dl < 0 &&
                           ego_cur_frenet_box.l_min - obj_cur_frenet_box.l_max <
                               kLcDirectionMaxSpacingDis))) {
    lat_overlap_with_direction =
        ego_cur_frenet_box.l_max - obj_cur_frenet_box.l_min;
  } else {
    lat_overlap_with_direction = 0;
  }

  return lat_overlap_with_direction;
};

inline double getObjPredictedTrajectoryTime(
    const SpacetimeObjectTrajectory* const obstacle_trajectory,
    bool if_obs_front) {
  double obj_trj_time = 6.0;
  auto obs_type = obstacle_trajectory->object_type();
  bool is_pedestrian = (obs_type == OT_PEDESTRIAN);

  if (is_pedestrian) {
    obj_trj_time = 3.0;
  } else {
    obj_trj_time = 5.0;
  }
  return obj_trj_time;
}

inline double getObjTypeFactor(
    const SpacetimeObjectTrajectory* const obstacle_trajectory,
    bool if_obs_front) {
  double obj_type_factor = 1.0;
  if (IsBigVehicle(obstacle_trajectory)) {
    if (if_obs_front) {
      obj_type_factor = 1.2;
    } else {
      obj_type_factor = 1.5;
    }
  } else if (IsVRU(obstacle_trajectory)) {
    if (if_obs_front) {
      obj_type_factor = 1.8;
    } else {
      obj_type_factor = 1.38;
    }
  } else {
    obj_type_factor = 1.0;
  }
  return obj_type_factor;
}

double getLatOffsetFactor(const e2e_noa::FrenetBox& ego_cur_frenet_box,
                          double ego_half_width, double intrusion_dis) {
  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  const double closet_corner_lat_offset =
      lc_left ? std::abs(std::min(0.0, ego_cur_frenet_box.l_max))
              : std::abs(std::max(0.0, ego_cur_frenet_box.l_min));

  double lat_offset_factor = QuadraticLerp(
      kMinCompensationFactor, (kDefaultHalfLaneWidth - intrusion_dis),
      kMaxCompensationFactor, (kDefaultLaneWidth - ego_half_width),
      closet_corner_lat_offset, true);
  return lat_offset_factor;
}

double getLatOffsetFactor(const e2e_noa::FrenetBox& ego_cur_frenet_box,
                          double ego_half_width, double min_compensation_factor,
                          double intrusion_dis) {
  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  const double closet_corner_lat_offset =
      lc_left ? std::abs(std::min(0.0, ego_cur_frenet_box.l_max))
              : std::abs(std::max(0.0, ego_cur_frenet_box.l_min));

  double lat_offset_factor = QuadraticLerp(
      min_compensation_factor, (kDefaultHalfLaneWidth - intrusion_dis),
      kMaxCompensationFactor, (kDefaultLaneWidth - ego_half_width),
      closet_corner_lat_offset, true);
  return lat_offset_factor;
}

inline bool IsLatClosestBlocking(const FrenetBox& ego_box,
                                 const FrenetBox& obj_box, double obj_lat_ext) {
  double rel_dl = 0;

  const bool lc_left = ego_box.center_l() < 0.0;

  return !HasFullyEnteredTargetLane(obj_box, 0.5 * obj_box.width()) &&
         (obj_box.center_l() * ego_box.center_l() > 0.0) &&
         (std::abs(ego_box.center_l()) > std::abs(obj_box.center_l()) ||
          std::fabs(rel_dl) < ego_box.width() * 0.6) &&
         ((!lc_left &&
           ego_box.center_l() - ego_box.width() * 0.5 - obj_box.width() * 0.5 -
                   std::max(kMinLatBufferToCloseBlocking,
                            kLatThresholdToCloseBlocking + obj_lat_ext) <
               obj_box.center_l()) ||
          (lc_left &&
           obj_box.center_l() - ego_box.width() * 0.5 - obj_box.width() * 0.5 -
               std::max(kMinLatBufferToCloseBlocking,
                        kLatThresholdToCloseBlocking - obj_lat_ext)) <
              ego_box.center_l());
}

inline bool IsBlockingObjectAbreast(const FrenetBox& ego_box,
                                    const FrenetBox& obj_box,
                                    const double obj_front_ext,
                                    const double obj_lat_ext) {
  return (HasFullyEnteredTargetLane(obj_box, 0.5 * obj_box.width()) ||
          IsLatClosestBlocking(ego_box, obj_box, obj_lat_ext)) &&
         ego_box.s_max + std::max(g_MinLonBuffer, obj_front_ext) >
             obj_box.s_min &&
         obj_box.s_max + std::max(g_MinLonBuffer, obj_front_ext) >
             ego_box.s_min;
}

inline double GetLaneChangeStyleFactor(LaneChangeStyle lc_style) {
  switch (lc_style) {
    case LC_STYLE_NORMAL:
      return 1.0;
    case LC_STYLE_RADICAL:
      return FLAGS_planner_lc_safety_radical_factor;
    case LC_STYLE_CONSERVATIVE:
      return FLAGS_planner_lc_safety_conservative_factor;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

inline Box2d LerpBox2d(const Box2d& box1, const Box2d& box2, double t) {
  return Box2d(box1.half_length(), box1.half_width(),
               Lerp(box1.center(), box2.center(), t),
               LerpAngle(box1.heading(), box2.heading(), t));
}

absl::StatusOr<Box2d> FindPredictedObjectBox(
    absl::Span<const SpacetimeObjectState> obj_states,
    double start_time_offset) {
  for (size_t i = 1; i < obj_states.size(); ++i) {
    const auto& obj_state = obj_states[i];
    if (obj_state.traj_point->t() < start_time_offset) continue;

    return LerpBox2d(
        obj_states[i - 1].box, obj_state.box,
        (start_time_offset - obj_states[i - 1].traj_point->t()) /
            (obj_state.traj_point->t() - obj_states[i - 1].traj_point->t()));
  }
  return absl::NotFoundError(
      "Path start point is not covered by predicted trajectory.");
}

double EstimateObjectSpeed(const PlannerObject& object, double preview_time) {
  double obj_v = object.pose().v();
  const auto& accel_hist = object.long_term_behavior().accel_history;
  if (!accel_hist.empty()) {
    constexpr int kMaxConsideredAccelHistoryItem = 5;
    constexpr std::array<double, 5> kAccelWeights{8.0, 6.0, 4.0, 2.0, 1.0};

    int idx = 0;
    double avg_accel = 0.0;
    for (auto it = accel_hist.rbegin(); it != accel_hist.rend(); ++it) {
      avg_accel += *it * kAccelWeights[idx++];
      if (idx >= kMaxConsideredAccelHistoryItem) break;
    }
    avg_accel /= std::accumulate(kAccelWeights.begin(),
                                 kAccelWeights.begin() + idx, 0.0);
    obj_v += avg_accel * preview_time;
  }
  return obj_v;
}

bool IsLeavingTargetLanePath(const FrenetFrame& target_frenet_frame,
                             bool ego_lc_left, const FrenetBox& ego_cur_box,
                             const FrenetBox& obj_cur_box,
                             absl::Span<const SpacetimeObjectState> obj_states,
                             double lon_safe_min_dist) {
  const double obj_width = obj_cur_box.width();
  const auto obj_preview_l =
      target_frenet_frame.XYToSL(obj_states.back().traj_point->pos()).l;

  if (HasFullyEnteredTargetLane(obj_preview_l, 0.5 * obj_width)) return false;

  const double obj_cur_l = obj_cur_box.center_l();
  const bool obj_lc_left = (obj_preview_l > obj_cur_l);
  const bool obj_cut_out_clearly =
      (std::fabs(obj_preview_l) > std::fabs(obj_cur_l)) &&
      (std::fabs(obj_preview_l - obj_cur_l) > 0.5);

  if ((ego_lc_left == obj_lc_left && ego_cur_box.s_max < obj_cur_box.s_min) ||
      (ego_lc_left != obj_lc_left &&
       ego_cur_box.s_min < obj_cur_box.s_max + g_MinLonBuffer) ||
      (std::fabs(ego_cur_box.center_s() - obj_cur_box.center_s()) <
       lon_safe_min_dist + ego_cur_box.length() * 0.5 +
           obj_cur_box.length() * 0.5)) {
    return false;
  }

  constexpr double kLeaveLaneLatRatio = 0.15;
  return (ego_lc_left && obj_lc_left && obj_cut_out_clearly &&
          obj_cur_box.l_min > -kLeaveLaneLatRatio * obj_width) ||
         (!ego_lc_left && !obj_lc_left && obj_cut_out_clearly &&
          obj_cur_box.l_max < kLeaveLaneLatRatio * obj_width);
}

bool ObjIsLeavingTargetLane(const FrenetBox& obj_cur_box,
                            const e2e_noa::FrenetCoordinate& obj_preview_pnt) {
  const double obj_width = obj_cur_box.width();
  const double obj_cur_l = obj_cur_box.center_l();
  const auto obj_preview_l = obj_preview_pnt.l;
  bool obj_preview_not_in_target =
      !HasFullyEnteredTargetLane(obj_preview_l, 0.5 * obj_width);

  const bool obj_cut_out_clearly =
      obj_preview_not_in_target &&
      (std::fabs(obj_preview_l) > std::fabs(obj_cur_l)) &&
      (std::fabs(obj_preview_l - obj_cur_l) > 0.3);

  return obj_cut_out_clearly;
}

bool ObjIsEnteringTargetLane(const FrenetBox& obj_cur_box,
                             const e2e_noa::FrenetCoordinate& obj_preview_pnt) {
  const double obj_width = obj_cur_box.width();
  const double obj_cur_l = obj_cur_box.center_l();
  const auto obj_preview_l = obj_preview_pnt.l;
  bool obj_preview_in_target =
      HasFullyEnteredTargetLane(obj_preview_l, 0.5 * obj_width);

  bool obj_current_not_in_target =
      !HasFullyEnteredTargetLane(obj_cur_l, 0.5 * obj_width);

  const bool obj_entering_clearly =
      (obj_preview_in_target || obj_current_not_in_target) &&
      (std::fabs(obj_preview_l) < std::fabs(obj_cur_l)) &&
      (std::fabs(obj_preview_l - obj_cur_l) > 0.3);

  return obj_entering_clearly;
}

bool PathHasOverlap(absl::Span<const Box2d> ego_boxes, double obj_v,
                    absl::Span<const SpacetimeObjectState> obj_states) {
  constexpr double kFrontExtensionTime = 2.0;
  constexpr double kLateralExtension = 2 * 0.5;

  auto obj_cur_ext_box =
      obj_states[0].box.ExtendedAtFront(obj_v * kFrontExtensionTime);
  obj_cur_ext_box.LateralExtend(kLateralExtension);
  if (ego_boxes[0].HasOverlap(obj_cur_ext_box)) return true;

  for (const auto& obj_state : obj_states) {
    Box2d obj_ext_box = obj_state.box;
    obj_ext_box.LateralExtend(kLateralExtension);
    for (const auto& ego_box : ego_boxes) {
      if (ego_box.HasOverlap(obj_ext_box)) {
        return true;
      }
    }
  }
  return false;
}

double ComputeEnterTargetTime(
    const FrenetFrame& target_frenet_frame,
    absl::Span<const SpacetimeObjectState> obj_states) {
  const double obj_half_width = obj_states[0].box.width() * 0.5;
  for (const auto& state : obj_states) {
    const auto obj_sl = target_frenet_frame.XYToSL(state.traj_point->pos());
    if (HasFullyEnteredTargetLane(obj_sl.l, obj_half_width)) {
      return state.traj_point->t() - obj_states.front().traj_point->t();
    }
  }
  return DBL_MAX;
}

double ComputeMinLonBufferSimilarSpeedFactor(
    double lead_v, double follow_v,
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  const std::vector<double> K_SPEED_FACTOR_AGGR_VEC = {
      1.2, 1.1, 1.0, 0.95, 0.9, 0.8, 0.78, 0.75, 0.7};
  const std::vector<double> K_SPEED_FACTOR_CNSV_VEC = {
      2.0, 1.7, 1.5, 1.2, 1.0, 0.95, 0.9, 0.85, 0.75};
  const std::vector<double> V_FOLLOW_DV_VEC = {-30, -15, -10, -5, 0,
                                               5,   10,  15,  30};

  auto obs_type = obstacle_trajectory->object_type();
  bool is_motor = (obs_type == OT_MOTORCYCLIST || obs_type == OT_CYCLIST);

  double dv_to_follow = (lead_v - follow_v);

  double k_speed_factor = ad_e2e::planning::math::interp1_inc(
      V_FOLLOW_DV_VEC,
      (!is_motor ? K_SPEED_FACTOR_AGGR_VEC : K_SPEED_FACTOR_CNSV_VEC),
      Mps2Kph(dv_to_follow));

  return std::clamp(k_speed_factor, 0.7, 2.0);
}

double ComputeSimilarSpeedFactor(double lead_v, double follow_v, double ref_v) {
  constexpr double kMaxSpeedDiffThres = 2.5;
  constexpr double kSimilarSpeedThresRatio = 0.25;

  double SpeedDiffThres =
      std::min(kMaxSpeedDiffThres, kSimilarSpeedThresRatio * ref_v);

  return std::clamp((follow_v - lead_v + SpeedDiffThres) / SpeedDiffThres, 0.0,
                    1.0);
}

double ComputeEgoLeadTime(double speed_limit, double ego_v, double obj_v,
                          bool is_lane_change_state) {
  double k_speed_factor = ad_e2e::planning::math::interp1_inc(
      V_HEAD_WAY_VEC, K_BACK_HEAD_WAY_VEC, Mps2Kph(obj_v));
  constexpr double kMinSimilarSpeedFactor = 0.0;
  const double similar_speed_factor = std::max(
      ComputeSimilarSpeedFactor(ego_v, obj_v, obj_v), kMinSimilarSpeedFactor);

  constexpr double kExceedSpeedLimitRatio = 0.95;
  constexpr double kExceedSpeedLimitSlope = 10.0;

  double ego_stationary_time_compensation = 0;
  if (ego_v < kEpsilon) {
    ego_stationary_time_compensation = is_lane_change_state ? 0.5 : 1.0;
  } else if (ego_v < 1) {
    ego_stationary_time_compensation = is_lane_change_state ? 0.2 : 0.5;
  } else if (ego_v < 2) {
    ego_stationary_time_compensation = is_lane_change_state ? 0.1 : 0.2;
  } else {
    ego_stationary_time_compensation = 0;
  }

  double kEgoLeadTimeBuffer =
      is_lane_change_state
          ? (kEgoLeadTimeBufferProcess + ego_stationary_time_compensation)
          : (kEgoLeadTimeBufferPrepare + ego_stationary_time_compensation);

  const double min_lead_time =
      (obj_v != 0) ? g_MinLonBuffer / (obj_v + kEpsilon) : 0;

  const double speed_limit_factor =
      1.0 +
      kExceedSpeedLimitSlope *
          Sqr(std::max(0.0, obj_v / speed_limit - kExceedSpeedLimitRatio));

  return std::max(min_lead_time, kEgoLeadTimeBuffer * similar_speed_factor *
                                     speed_limit_factor * k_speed_factor);
}

double ComputeEgoFollowTime(double obj_v, double ego_v,
                            bool is_lane_change_state) {
  double k_speed_factor = ad_e2e::planning::math::interp1_inc(
      V_HEAD_WAY_VEC, K_FRONT_HEAD_WAY_VEC, Mps2Kph(ego_v));

  constexpr double kHigherSpeedThresRatio = 0.95;

  double kEgoFollowTimeBuffer = is_lane_change_state
                                    ? kEgoFollowTimeBufferProcess
                                    : kEgoFollowTimeBufferPrepare;

  const double min_follow_time =
      (ego_v != 0) ? g_MinLonBuffer / (ego_v + kEpsilon) : 0;

  if (obj_v > ego_v * kHigherSpeedThresRatio && ego_v != 0) {
    return min_follow_time;
  }

  return std::max(min_follow_time,
                  kEgoFollowTimeBuffer *
                      ComputeSimilarSpeedFactor(obj_v, ego_v, ego_v)) *
         k_speed_factor;
}

absl::Status CheckDeceleration(double lon_dist, absl::string_view name_lead,
                               absl::string_view name_follow, double v_lead,
                               double v_follow, double response_time,
                               double lead_time, double max_allowed_decel,
                               double acc_compensation_dis,
                               std::string& ok_debug_info,
                               double* max_decel = nullptr) {
  const double v_diff = std::max(0.0, v_follow - v_lead);
  const double buffered_lon_dist =
      lon_dist - v_diff * (lead_time - (v_lead / max_allowed_decel)) -
      g_MinLonBuffer - acc_compensation_dis;
  if (buffered_lon_dist <= kEpsilon) {
    return absl::CancelledError(absl::StrFormat(
        "No space left for %s to decelerate behind %s. "
        "lon_dist:%.2f lead_time:%.2f "
        "v_lead:%.2f v_follow:%.2f v_diff:%.2f "
        "max_ald_decel:%.2f MinLonBuffer:%.2f acc_comp_dis:%.2f",
        name_follow, name_lead, lon_dist, lead_time, v_lead, v_follow, v_diff,
        max_allowed_decel, g_MinLonBuffer, acc_compensation_dis));
  }

  const double hypo_decel =
      (v_diff * (v_follow + v_lead)) / (2.0 * buffered_lon_dist);

  bool decel_danger = hypo_decel > max_allowed_decel;

  auto debug_str = absl::StrFormat(
      "decel_danger:%d. (hypo_decel:-%.2f<max_allowed_decel:-%.2f) for %s "
      "behind %s. "
      "lon_dist:%.2f lead_time:%.2f "
      "v_lead:%.2f v_follow:%.2f v_diff:%.2f "
      "MinLonBuffer:%.2f acc_comp_dis:%.2f",
      decel_danger, hypo_decel, max_allowed_decel, name_follow, name_lead,
      lon_dist, lead_time, v_lead, v_follow, v_diff, g_MinLonBuffer,
      acc_compensation_dis);

  if (decel_danger) {
    return absl::CancelledError(debug_str);
  }
  if (max_decel != nullptr && hypo_decel > *max_decel) *max_decel = hypo_decel;

  ok_debug_info.clear();
  ok_debug_info = debug_str;

  return absl::OkStatus();
}

double getAccelerationDistanceCompensation(double offset_factor, double ego_acc,
                                           double obs_acc, double rel_vs,
                                           bool if_obs_front,
                                           bool is_lane_change_state,
                                           bool is_distance_shortening) {
  double rel_acc = ego_acc - obs_acc;
  double rel_abs_acc = std::fabs(ego_acc - obs_acc);
  double rel_acc_temp = std::clamp(rel_acc, -5.0, 2.0);
  double acc_safe_dis = 0;
  double pred_time = 3;
  double res_sign = 0;
  double non_change_pred_time = 1.414;
  double pred_time_min = 0.8;
  double pred_time_max = 1.0;

  if (if_obs_front) {
    if (rel_acc_temp > 0) {
      res_sign = 1;
    } else if (rel_acc_temp < 0 && ego_acc < 0.5) {
      res_sign = -1;
    } else {
      res_sign = 0;
    }
  } else {
    if (rel_acc_temp < 0) {
      res_sign = 1;
    } else if (rel_acc_temp > 0 && ego_acc > -0.5) {
      res_sign = -1;
    } else {
      res_sign = 0;
    }
  }

  if (res_sign == 1) {
    if (is_distance_shortening && !if_obs_front &&
        ((rel_vs > 3 && (ego_acc < -0.5 || obs_acc > 0.8)) || ego_acc < -1.5)) {
      non_change_pred_time = 1.6;
      pred_time_max = 1.414;
      pred_time_min = 1.26;
    } else {
      non_change_pred_time = 1.414;
      pred_time_max = 1.26;
      pred_time_min = 1.1;
    }

    pred_time = is_lane_change_state
                    ? Lerp(pred_time_max, kMaxCompensationFactor, pred_time_min,
                           kMinCompensationFactor, offset_factor, true)
                    : non_change_pred_time;
  } else if (res_sign == -1) {
    if (!if_obs_front) {
      non_change_pred_time = 1.51;
      pred_time_min = 1.6;
      pred_time_max = 1.68;
    } else {
      non_change_pred_time = 1.0;
      pred_time_min = 1.1;
      pred_time_max = 1.26;
    }
    pred_time = is_lane_change_state
                    ? Lerp(pred_time_min, kMaxCompensationFactor, pred_time_max,
                           kMinCompensationFactor, offset_factor, true)
                    : non_change_pred_time;
  } else {
    pred_time = 0;
  }

  acc_safe_dis =
      0.5 * res_sign * rel_abs_acc * pred_time * pred_time * offset_factor;

  return acc_safe_dis;
}

bool isLaneChangingNotIncludePause(const e2e_noa::LaneChangeStage& lc_state) {
  const bool is_lane_change_state =
      (lc_state == LaneChangeStage::LCS_EXECUTING) ||
      (lc_state == LaneChangeStage::LCS_RETURN);
  return is_lane_change_state;
}

bool isLaneChanging(const e2e_noa::LaneChangeStage& lc_state) {
  const bool is_lane_change_state =
      (lc_state == LaneChangeStage::LCS_EXECUTING) ||
      (lc_state == LaneChangeStage::LCS_PAUSE) ||
      (lc_state == LaneChangeStage::LCS_RETURN);
  return is_lane_change_state;
}

}  // namespace

absl::Status CheckLaneChangeSafety(
    const ApolloTrajectoryPointProto& start_point,
    const std::vector<ApolloTrajectoryPointProto>& ego_traj_pts,
    const LeadingTrajs& leading_trajs, const FrenetFrame& target_frenet_frame,
    double speed_limit, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geom, LaneChangeStyle lc_style,
    const e2e_noa::LaneChangeStage& lc_state,
    const e2e_noa::LaneChangeStage& prev_lc_stage,
    absl::Duration path_look_ahead_duration, TrajEvalInfo* eval_info) {
  auto& follower_set = eval_info->follower_set;
  auto& leader_set = eval_info->leader_set;
  auto& follower_max_decel = eval_info->follower_max_decel;
  auto& leader_max_decel = eval_info->leader_max_decel;
  auto& unsafe_object_id = eval_info->unsafe_object_id;
  auto& status_code = eval_info->status_code;

  auto& follower_debug_info = eval_info->follower_debug_info;
  auto& leader_debug_info = eval_info->leader_debug_info;
  auto& enable_pause_counter = eval_info->enable_pause_counter;
  const bool is_lc_state_not_pause_prev =
      isLaneChangingNotIncludePause(prev_lc_stage);
  const bool is_lc_state_prev = isLaneChanging(prev_lc_stage);

  double kEgoLeadTimeBuffer = is_lc_state_not_pause_prev
                                  ? kEgoLeadTimeBufferProcess
                                  : kEgoLeadTimeBufferPrepare;
  double kEgoFollowTimeBuffer = is_lc_state_not_pause_prev
                                    ? kEgoFollowTimeBufferProcess
                                    : kEgoFollowTimeBufferPrepare;

  const double preview_time = 0;
  const double ego_v = start_point.v() + start_point.a() * preview_time;
  const double ego_speed_kph = Mps2Kph(ego_v);

  g_MinLonBuffer = kMinLonBufferToFront;

  const int ego_traj_size = ego_traj_pts.size();
  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(ego_traj_size);
  int ego_enter_target_idx = -1;
  double ego_half_width = vehicle_geom.width() * 0.5;
  std::ostringstream dynamic_obs_debug;
  dynamic_obs_debug.str("");

  std::ostringstream static_obs_debug;
  static_obs_debug.str("");

  for (int i = 0; i < ego_traj_size; ++i) {
    const auto& traj_pt = ego_traj_pts[i];
    const Vec2d traj_pos = Extract2dVectorFromApolloProto(traj_pt);
    const auto ego_sl = target_frenet_frame.XYToSL(traj_pos);
    ego_boxes.push_back(
        ComputeAvBox(traj_pos, traj_pt.path_point().theta(), vehicle_geom));
    if (ego_enter_target_idx == -1 &&
        HasFullyEnteredTargetLane(ego_sl.l, ego_half_width)) {
      ego_enter_target_idx = i;
    }
  }
  dynamic_obs_debug << "/***/";
  static_obs_debug << "/***/"
                   << absl::StrFormat("enter_target_idx:%d",
                                      ego_enter_target_idx);

  constexpr int kMustEnterTargetStep = 0.95 * kTrajectorySteps;
  if ((ego_enter_target_idx == -1 ||
       ego_enter_target_idx > kMustEnterTargetStep) &&
      ego_speed_kph > 10) {
    status_code = PlannerStatusProto::TRAJECTORY_NOT_ENTERING_TARGET_LANE;
    return absl::CancelledError(absl::StrFormat(
        "Trajectory not entering the target lane (enter_target_idx:%d).",
        ego_enter_target_idx));
  }
  auto ego_cur_box = ego_boxes.front();
  ASSIGN_OR_RETURN(
      const auto ego_cur_frenet_box,
      target_frenet_frame.QueryFrenetBoxAt(ego_cur_box),
      _ << "Cannot project the current ego box onto drive passage.");

  auto ego_corner = ego_boxes.front().GetCornersCounterClockwise();
  auto ego_front_vct =
      std::vector<e2e_noa::Vec2d>{ego_corner.front(), ego_corner.back()};
  ASSIGN_OR_RETURN(
      const auto ego_front_frenet_box,
      target_frenet_frame.QueryFrenetBoxAtPoints(ego_front_vct),
      _ << "Cannot project the current ego box onto drive passage.");

  const double ego_acc = start_point.a();
  const auto ego_target_tangent_cos = ego_cur_box.tangent().Dot(
      target_frenet_frame.InterpolateTangentByS(ego_cur_frenet_box.center_s()));
  const auto ego_target_normal_cos = ego_cur_box.tangent().Dot(
      target_frenet_frame.InterpolateTangentByS(ego_cur_frenet_box.center_s())
          .Rotate(M_PI_2));

  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  const double path_start_time_offset =
      absl::ToDoubleSeconds(path_look_ahead_duration);
  double conserv_base =
      (FLAGS_planner_enable_lc_style_params ? GetLaneChangeStyleFactor(lc_style)
                                            : 1.0) *
      getLatOffsetFactor(ego_cur_frenet_box, ego_half_width, 0.2);

  double conserv = conserv_base;

  bool ego_occupy_target =
      isEgoFullyOccupyTargetLane(ego_cur_frenet_box, ego_speed_kph, lc_left);
  bool ego_centered_in_target_lane = HasFullyCenteredInTargetLane(
      ego_cur_frenet_box, 0.5 * ego_cur_frenet_box.width());

  double temp_min_front_dist = ad_e2e::planning::math::interp1_inc(
      V_EGO_VEHICLE_VEC, MIN_FRONT_DISTANCE_VEC, ego_speed_kph);
  double temp_min_back_dist = ad_e2e::planning::math::interp1_inc(
      V_EGO_VEHICLE_VEC, MIN_BACK_DISTANCE_VEC, ego_speed_kph);

  const double ego_enter_target_time =
      ego_enter_target_idx * kTrajectoryTimeStep;

  const double closet_corner_lat_offset =
      lc_left ? std::abs(std::min(0.0, ego_cur_frenet_box.l_max))
              : std::abs(std::max(0.0, ego_cur_frenet_box.l_min));

  double pause_factor =
      (prev_lc_stage == LaneChangeStage::LCS_PAUSE && ego_enter_target_idx > 10)
          ? 1.1
          : 1.0;

  e2e_noa::FrenetBox frnt_static_filter_obs_box{DBL_MAX, DBL_MAX, DBL_MAX,
                                                DBL_MAX};
  std::string frnt_static_filter_obs_id = "";

  e2e_noa::FrenetBox frnt_obs_box{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  e2e_noa::FrenetBox rear_obs_box{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  std::string frnt_obs_id = "";
  std::string rear_obs_id = "";

  for (const auto* traj_ptr : st_traj_mgr.moving_object_trajs()) {
    ASSIGN_OR_CONTINUE(
        const auto obj_cur_box,
        FindPredictedObjectBox(traj_ptr->states(), path_start_time_offset));
    CHECK(!traj_ptr->states().empty())
        << "No prediction state for trajectory " << traj_ptr->traj_id();
    ASSIGN_OR_CONTINUE(
        const auto obj_cur_frenet_box,
        target_frenet_frame.QueryFrenetBoxWithHeading(obj_cur_box));
    const double obj_v =
        ego_cur_frenet_box.s_max < obj_cur_frenet_box.s_min
            ? traj_ptr->planner_object().pose().v()
            : EstimateObjectSpeed(traj_ptr->planner_object(), preview_time);

    const bool is_vehicle = IsVehicle(traj_ptr);
    const bool is_big_vehicle = IsBigVehicle(traj_ptr);
    const bool is_vru = IsVRU(traj_ptr);
    const double rel_vs = (obj_v - ego_v);
    const double abs_rel_vs_kph = std::fabs(Mps2Kph(rel_vs));

    const bool if_obs_front =
        (ego_cur_frenet_box.s_max < obj_cur_frenet_box.s_min);
    const bool if_obs_rear =
        (obj_cur_frenet_box.s_max < ego_cur_frenet_box.s_min);

    double rel_ds, rel_dl = 0;
    double long_overlap =
        getLongOverlap(ego_cur_frenet_box, obj_cur_frenet_box, rel_ds);
    double lat_overlap =
        getLatOverlap(ego_cur_frenet_box, obj_cur_frenet_box, rel_dl);
    double lat_overlap_lc = getLatOverlapWithLcDirection(
        ego_front_frenet_box, obj_cur_frenet_box, rel_dl);

    const bool if_obs_left =
        (ego_cur_frenet_box.l_max < obj_cur_frenet_box.l_min);

    bool is_distance_shortening = rel_ds * rel_vs < 0;
    bool is_rear_approaching_vehicle =
        (is_distance_shortening && !if_obs_front);
    double obs_acc = traj_ptr->planner_object().pose().a();
    double rel_acc = ego_acc - obs_acc;
    double rel_abs_acc = std::fabs(ego_acc - obs_acc);
    bool is_ego_stationary = (ego_speed_kph < kStationaryVehicleSpeedkph);
    bool is_ego_extra_slow = (ego_speed_kph < kExtremelySlowVehicleSpeedkph);
    bool obs_in_target_lane = HasFullyEnteredTargetLane(
        obj_cur_frenet_box, 0.5 * obj_cur_frenet_box.width());
    bool obs_centered_in_target_lane = HasFullyCenteredInTargetLane(
        obj_cur_frenet_box, 0.5 * obj_cur_frenet_box.width());

    auto using_prediction_time_num = static_cast<size_t>(4.0 / 0.1);
    auto obj_preview_state =
        (traj_ptr->states().size() > using_prediction_time_num)
            ? traj_ptr->states().at(using_prediction_time_num)
            : traj_ptr->states().back();
    const auto obj_preview_pnt =
        target_frenet_frame.XYToSL(obj_preview_state.traj_point->pos());
    bool obj_is_leaving_target =
        ObjIsLeavingTargetLane(obj_cur_frenet_box, obj_preview_pnt);
    bool obj_is_entering_target =
        ObjIsEnteringTargetLane(obj_cur_frenet_box, obj_preview_pnt);

    const bool obj_lc_left =
        (obj_preview_pnt.l > obj_cur_frenet_box.center_l());

    const auto obj_target_tangent_cos =
        obj_cur_box.tangent().Dot(target_frenet_frame.InterpolateTangentByS(
            obj_cur_frenet_box.center_s()));
    const auto obj_target_normal_cos = obj_cur_box.tangent().Dot(
        target_frenet_frame.InterpolateTangentByS(obj_cur_frenet_box.center_s())
            .Rotate(M_PI_2));

    const auto leader_vel = if_obs_front ? obj_v : ego_v;
    const auto leader_box = if_obs_front ? obj_cur_box : ego_cur_box;
    const auto leader_target_tangent_cos =
        if_obs_front ? obj_target_tangent_cos : ego_target_tangent_cos;
    const auto leader_target_normal_cos =
        if_obs_front ? obj_target_normal_cos : ego_target_normal_cos;

    const auto follower_vel = if_obs_front ? ego_v : obj_v;
    const auto follower_box = if_obs_front ? ego_cur_box : obj_cur_box;
    const auto follower_target_tangent_cos =
        if_obs_front ? ego_target_tangent_cos : obj_target_tangent_cos;
    const auto follower_target_normal_cos =
        if_obs_front ? ego_target_normal_cos : obj_target_normal_cos;

    const double obj_enter_target_time = std::clamp(
        ComputeEnterTargetTime(target_frenet_frame, traj_ptr->states()), 0.0,
        255.0);

    bool obj_pred_to_target =
        !(obj_enter_target_time > 1.2 * ego_enter_target_time &&
          obj_enter_target_time >
              getObjPredictedTrajectoryTime(traj_ptr, if_obs_front) *
                  conserv_base) &&
        !obj_is_leaving_target;

    conserv =
        conserv_base * pause_factor * getObjTypeFactor(traj_ptr, if_obs_front);
    const double follow_obj_resp_time = conserv * kFollowerStandardResponseTime;

    double min_lon_buffer_factor =
        std::clamp(conserv * ComputeMinLonBufferSimilarSpeedFactor(
                                 leader_vel, follower_vel, traj_ptr),
                   0.38, 2.0);

    g_MinLonBuffer =
        min_lon_buffer_factor *
        ((if_obs_front ? temp_min_front_dist : temp_min_back_dist) +
         getMinDisCompensation(if_obs_front, is_big_vehicle, obj_cur_box));

    bool veh_will_stop =
        (obj_v < kExtremelySlowVehicleSpeedkph) ||
        ((obs_acc < 0) && (-obj_v / obs_acc < 2.0) && (-obj_v / obs_acc >= 0));

    const double lat_overlap_thrd =
        conserv * kEgoFollowTimeBufferPrepare *
        (ego_v * ego_target_normal_cos - obj_v * obj_target_normal_cos) *
        (lc_left ? 1.0 : -1.0);

    bool obj_overlap_ignore =
        !obs_centered_in_target_lane
            ? lat_overlap > kRearObjIgnoreLatOverlapFactor * conserv_base *
                                obj_cur_frenet_box.width()
            : lat_overlap >= std::min(-lat_overlap_thrd, 0.0);

    dynamic_obs_debug << "-->Mobj:" << traj_ptr->object_id();

    if (is_ego_stationary && !if_obs_front && is_rear_approaching_vehicle &&
        obs_in_target_lane && !obj_overlap_ignore) {
      g_MinLonBuffer += 5.0;
      dynamic_obs_debug << "-buf+";
    } else if (!is_lc_state_prev && !obs_in_target_lane &&
               ego_enter_target_idx != 0 && is_vehicle && if_obs_front &&
               rel_dl == 0 && veh_will_stop) {
      g_MinLonBuffer = 5.0 * conserv_base;
      std::vector<double> lat_threshold = {0.2, 1.0, 1.5, 2.0, 4.0};
      std::vector<double> lon_dis = {0.5, 3.0, 4.8, 6.0, 10.0};
      auto lat_overlap_safe_dis = ad_e2e::planning::math::interp1_inc(
          lat_threshold, lon_dis, lat_overlap_lc);

      double lat_overlap_factor =
          std::clamp(lat_overlap_lc / vehicle_geom.width(), 0.0, 2.0);

      double state_debounce_factor =
          ((prev_lc_stage != LaneChangeStage::LCS_EXECUTING ||
            std::fabs(ego_target_normal_cos) < 0.06)
               ? 1.0
               : 0.8);

      auto front_safe_dis = std::max(
          lat_overlap_safe_dis, ego_v * 3.5 * conserv_base *
                                    lat_overlap_factor * state_debounce_factor);
      bool obj_danger = rel_ds < front_safe_dis;
      dynamic_obs_debug << "-frtSta:" << obj_danger;
      if (obj_danger) {
        unsafe_object_id = {traj_ptr->object_id().data(),
                            traj_ptr->object_id().size()};
        status_code = PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_CURRENT_LANE;
        leader_debug_info << static_obs_debug.str();
        follower_debug_info << dynamic_obs_debug.str();
        follower_set.insert(rear_obs_id);
        leader_set.insert(frnt_obs_id);
        enable_pause_counter = true;
        return absl::CancelledError(absl::StrFormat(
            "front low speed vehicle %s currently too "
            "close.lat_overlap:%.3f lat_overlap_lc:%.3f "
            "front_safe_dis:%.3f rel_ds:%.3f conserv_base:%.3f"
            "lat_overlap_safe_dis:%.3f ego_v:%.3f ",
            traj_ptr->object_id(), lat_overlap, lat_overlap_lc, front_safe_dis,
            rel_ds, conserv_base, lat_overlap_safe_dis, ego_v));
      }
    }

    const std::vector<double> lon_safety_dis = {2.0, 2.5, 3.0, 3.5, 4.0};
    const std::vector<double> safe_speed_diff = {
        0.0, Kph2Mps(10.0), Kph2Mps(20.0), Kph2Mps(30.0), Kph2Mps(40.0)};
    const auto lon_safe_min_dist = ad_e2e::planning::math::interp1_inc(
        safe_speed_diff, lon_safety_dis, abs_rel_vs_kph);

    if (!if_obs_front && !(obj_pred_to_target || obs_in_target_lane)) {
      dynamic_obs_debug << "-SkpRr";
      continue;
    }

    if (if_obs_front && !(obj_pred_to_target || obs_in_target_lane) &&
        (std::fabs(obj_cur_frenet_box.center_l()) >
         std::fabs(ego_cur_frenet_box.center_l()) + ego_half_width) &&
        (obj_cur_frenet_box.center_l() * ego_cur_frenet_box.center_l() > 0)) {
      dynamic_obs_debug << "-SkpFf";
      continue;
    }

    double acc_safe_dis = 0;
    if (!(obj_v < 0) && !(ego_v < 0) && !(ego_v == 0 && if_obs_front)) {
      acc_safe_dis = getAccelerationDistanceCompensation(
          conserv_base, ego_acc, obs_acc, rel_vs, if_obs_front,
          is_lc_state_prev, is_distance_shortening);
    }

    const double rel_head_way_time =
        if_obs_front
            ? ComputeEgoFollowTime(obj_v, ego_v, prev_lc_stage)
            : ComputeEgoLeadTime(speed_limit, ego_v, obj_v, prev_lc_stage);

    const double obj_front_extension = conserv * rel_head_way_time *
                                           follower_vel *
                                           follower_target_tangent_cos +
                                       acc_safe_dis;

    const double obj_lat_extension =
        conserv * rel_head_way_time *
        (leader_vel * leader_target_normal_cos -
         follower_vel * follower_target_normal_cos);

    DLOG(INFO) << "object_id()" << traj_ptr->object_id();

    bool obj_is_abreast =
        IsBlockingObjectAbreast(ego_cur_frenet_box, obj_cur_frenet_box,
                                obj_front_extension, obj_lat_extension);
    dynamic_obs_debug << "-abr:" << obj_is_abreast
                      << "-e:" << obs_in_target_lane
                      << "-p:" << obj_pred_to_target
                      << "-ot:" << obj_enter_target_time
                      << "-et:" << ego_enter_target_time;
    auto obj_abreast_str = absl::StrFormat(
        "Object:%s abreast:%d. "
        "front_ext:%.3f lat_ext:%.3f "
        "conserv:%.3f conserv_base:%.3f MinLonBuf:%.3f "
        "MinLonBuf_fct:%.3f  head_way:%.3f head_vel:%.3f "
        "rel_ds:%.3f rel_dl:%.3f ego_v:%.3f obs_v:%.3f "
        "ego_acc:%.3f obs_acc:%.3f acc_dis:%.3f "
        "ego_cos:%.3f obj_cos:%.3f ego_sin:%.3f obj_sin:%.3f "
        "ego_lmax:%.3f ego_lmin:%.3f obj_lmax:%.3f obj_lmin:%.3f ",
        traj_ptr->object_id(), obj_is_abreast, obj_front_extension,
        obj_lat_extension, conserv, conserv_base, g_MinLonBuffer,
        min_lon_buffer_factor, rel_head_way_time, follower_vel, rel_ds, rel_dl,
        ego_v, obj_v, ego_acc, obs_acc, acc_safe_dis, ego_target_tangent_cos,
        obj_target_tangent_cos, ego_target_normal_cos, obj_target_normal_cos,
        ego_cur_frenet_box.l_max, ego_cur_frenet_box.l_min,
        obj_cur_frenet_box.l_max, obj_cur_frenet_box.l_min);

    if (if_obs_front && (obj_pred_to_target || obs_in_target_lane) &&
        (frnt_obs_id.empty() ||
         obj_cur_frenet_box.s_min < frnt_obs_box.s_min)) {
      frnt_obs_id = {traj_ptr->object_id().data(),
                     traj_ptr->object_id().size()};
      frnt_obs_box = obj_cur_frenet_box;
      leader_debug_info.clear();
      leader_debug_info.str("");
      if (obj_is_abreast) {
        leader_debug_info << "Object:" << traj_ptr->object_id()
                          << " is abreast.";
      } else {
        leader_debug_info << obj_abreast_str;
      }
    } else if (if_obs_rear && (obj_pred_to_target || obs_in_target_lane) &&
               (rear_obs_id.empty() ||
                obj_cur_frenet_box.s_max > rear_obs_box.s_max)) {
      rear_obs_id = {traj_ptr->object_id().data(),
                     traj_ptr->object_id().size()};
      rear_obs_box = obj_cur_frenet_box;
      follower_debug_info.clear();
      follower_debug_info.str("");
      if (obj_is_abreast) {
        follower_debug_info << "Object:" << traj_ptr->object_id()
                            << " is abreast.";
      } else {
        follower_debug_info << obj_abreast_str;
      }
    }

    if ((if_obs_front && obs_in_target_lane) &&
        (frnt_static_filter_obs_id.empty() ||
         obj_cur_frenet_box.s_min < frnt_static_filter_obs_box.s_min)) {
      frnt_static_filter_obs_id = {traj_ptr->object_id().data(),
                                   traj_ptr->object_id().size()};
      frnt_static_filter_obs_box = obj_cur_frenet_box;
    }

    if (!is_vru && obs_in_target_lane && obj_is_leaving_target &&
        (lc_left ? obj_cur_frenet_box.l_min > 0
                 : obj_cur_frenet_box.l_max < 0) &&
        lat_overlap < -1.8 && obj_lc_left == lc_left) {
      dynamic_obs_debug << "-Lea1";
      continue;
    }

    if (IsLeavingTargetLanePath(target_frenet_frame, lc_left,
                                ego_cur_frenet_box, obj_cur_frenet_box,
                                traj_ptr->states(), lon_safe_min_dist)) {
      dynamic_obs_debug << "-Lea2";
      continue;
    } else {
      dynamic_obs_debug << "-LvT:" << obj_is_leaving_target;
    }

    if (!is_vru && obs_centered_in_target_lane && obj_overlap_ignore &&
        ego_enter_target_idx < 10) {
      dynamic_obs_debug << "-SkpTr:" << lat_overlap_thrd;
      continue;
    }

    if (obj_is_abreast) {
      unsafe_object_id = {traj_ptr->object_id().data(),
                          traj_ptr->object_id().size()};
      status_code =
          obs_in_target_lane
              ? (if_obs_front
                     ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                     : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_TARGET_LANE)
              : (if_obs_front
                     ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                     : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
      leader_debug_info << static_obs_debug.str();
      follower_debug_info << dynamic_obs_debug.str();
      follower_set.insert(rear_obs_id);
      leader_set.insert(frnt_obs_id);
      return absl::CancelledError(obj_abreast_str);
    }

    if ((ego_enter_target_idx == 0 && ego_occupy_target) ||
        ego_centered_in_target_lane) {
      dynamic_obs_debug << "-OcT";
      continue;
    }

    if ((prev_lc_stage != LaneChangeStage::LCS_PAUSE) &&
        !(obj_pred_to_target || obs_in_target_lane) &&
        !PathHasOverlap(ego_boxes, obj_v, traj_ptr->states())) {
      dynamic_obs_debug << "-Nol";
      continue;
    }

    if (!(obj_pred_to_target || obs_in_target_lane)) {
      dynamic_obs_debug << "-Net" << "-t:" << obj_enter_target_time;
      continue;
    }

    double debounce_braking_deceleration = 0;

    if (obj_cur_frenet_box.s_min <= ego_cur_frenet_box.s_max) {
      follower_set.insert(std::string(traj_ptr->object_id()));

      if (ego_v >= obj_v &&
          ego_v + std::min(ego_acc, 0.0) * 3.0 * conserv_base >=
              obj_v + obs_acc * 3.0 * conserv_base) {
        dynamic_obs_debug << "-Rsml";
        continue;
      } else {
        double rel_ttc = (obj_v > ego_v) ? -rel_ds / (obj_v - ego_v) : DBL_MAX;
        double impending_collision_scene =
            rel_ttc < conserv_base * (is_lc_state_prev ? 6.0 : 7.0);

        if (obj_overlap_ignore &&
            (!obs_centered_in_target_lane ? !impending_collision_scene
                                          : true)) {
          dynamic_obs_debug << "-Olap";
          continue;
        }

        double max_decel_for_rear_object = ad_e2e::planning::math::interp1_inc(
            DV_TTC_VEC, F_DV_BACK_DEC_VEC, abs_rel_vs_kph);

        if (is_lc_state_prev) {
          double acc_factor =
              impending_collision_scene
                  ? ad_e2e::planning::math::interp1_inc(
                        std::vector<double>{-2.0, -1.0, 0, 1.0, 1.5, 2.0, 3.0},
                        std::vector<double>{1.6, 1.3, 1.0, 0, -0.3, -0.6, -1.2},
                        obs_acc)
                  : 1.0;

          debounce_braking_deceleration =
              std::clamp(((1.0 - conserv_base) * 2.5 * acc_factor), -1.2, 1.5);

          max_decel_for_rear_object += debounce_braking_deceleration;
        }

        if (is_vru) {
          dynamic_obs_debug << "-f:0.40";
          max_decel_for_rear_object *= 0.40;
        } else if (obj_is_entering_target && obj_lc_left == lc_left) {
          dynamic_obs_debug << "-f:0.38";
          max_decel_for_rear_object *= 0.38;
        } else if (is_big_vehicle) {
          dynamic_obs_debug << "-f:0.6";
          max_decel_for_rear_object *= 0.6;
        } else if (obj_is_leaving_target) {
          double obs_abs_center_l = std::fabs(obj_cur_frenet_box.center_l());
          double obs_offset_factor = ad_e2e::planning::math::interp1_inc(
              std::vector<double>{0.0, 0.3, 0.5, 1.5, 2.0},
              std::vector<double>{1.1, 1.2, 1.3, 1.5, 2.0}, obs_abs_center_l);
          dynamic_obs_debug << "-f:" << obs_offset_factor;
          max_decel_for_rear_object *= obs_offset_factor;
        } else if ((obs_acc > std::max(1.5, 1.2 * ego_acc)) &&
                   impending_collision_scene && !obj_is_leaving_target) {
          dynamic_obs_debug << "-f:0.7";
          max_decel_for_rear_object *= 0.7;
        } else if ((ego_acc < -0.5 || obs_acc < -2.5) &&
                   !obj_is_leaving_target && impending_collision_scene) {
          dynamic_obs_debug << "-f:0.8";
          max_decel_for_rear_object *= 0.8;
        } else if (obj_is_entering_target && obj_lc_left != lc_left) {
          dynamic_obs_debug << "-f:0.85";
          max_decel_for_rear_object *= 0.85;
        } else {
          dynamic_obs_debug << "-f:1.0";
        }

        if (max_decel_for_rear_object < -obs_acc) {
          dynamic_obs_debug << "-avr";
          max_decel_for_rear_object =
              (max_decel_for_rear_object - obs_acc) * 0.5;
        }

        max_decel_for_rear_object =
            std::clamp(max_decel_for_rear_object / pause_factor, 0.6, 3.8);

        std::string dec_str = "";
        auto check_status = CheckDeceleration(
            std::fabs(rel_ds), "ego", traj_ptr->object_id(),
            (ego_v * ego_target_tangent_cos), (obj_v * obj_target_tangent_cos),
            follow_obj_resp_time, conserv * rel_head_way_time,
            max_decel_for_rear_object, acc_safe_dis, dec_str,
            &follower_max_decel);
        dynamic_obs_debug << "-dec:" << !check_status.ok()
                          << "-ttc:" << rel_ttc;

        if (rear_obs_id == traj_ptr->object_id()) {
          if (check_status.ok()) {
            follower_debug_info << "/***/CheckDeceleration:" << dec_str;
          }
        }

        if (!check_status.ok()) {
          unsafe_object_id = {traj_ptr->object_id().data(),
                              traj_ptr->object_id().size()};
          status_code =
              obs_in_target_lane
                  ? (if_obs_front ? PlannerStatusProto::
                                        FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                                  : PlannerStatusProto::
                                        REAR_DANGEROUS_VEHICLE_TARGET_LANE)
                  : (if_obs_front ? PlannerStatusProto::
                                        FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                                  : PlannerStatusProto::
                                        REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
          leader_debug_info << static_obs_debug.str();
          follower_debug_info << dynamic_obs_debug.str();
          follower_set.insert(rear_obs_id);
          leader_set.insert(frnt_obs_id);
          enable_pause_counter = true;
          return absl::CancelledError(check_status.ToString());
        }
      }
    } else {
      leader_set.insert(std::string(traj_ptr->object_id()));

      if (ego_v <= obj_v &&
          ego_v + std::max(ego_acc, 0.0) * 3.0 * conserv_base <=
              obj_v + obs_acc * 3.0 * conserv_base) {
        dynamic_obs_debug << "-Flrg";
        continue;
      } else {
        double max_decel_for_front_object = ad_e2e::planning::math::interp1_inc(
            DV_TTC_VEC, F_DV_FRONT_DEC_VEC, abs_rel_vs_kph);

        double rel_ttc = (ego_v > obj_v) ? rel_ds / (ego_v - obj_v) : DBL_MAX;
        double impending_collision_scene =
            rel_ttc < conserv_base * (is_lc_state_prev ? 5.0 : 6.0);

        if (is_lc_state_prev) {
          double acc_factor =
              impending_collision_scene
                  ? ad_e2e::planning::math::interp1_inc(
                        std::vector<double>{-3.0, -2.0, -1.5 - 1.0, 0, 0.5, 1.0,
                                            2.0},
                        std::vector<double>{-1.0, -0.8, -0.5, 0, 1.0, 1.2, 1.3,
                                            1.6},
                        obs_acc)
                  : 1.0;

          debounce_braking_deceleration =
              std::clamp(((1.0 - conserv_base) * 2.0 * acc_factor), -1.5, 2.0);

          max_decel_for_front_object += debounce_braking_deceleration;
        }

        bool front_obj_is_leaving =
            obj_is_leaving_target && !obs_centered_in_target_lane;

        if (is_vru) {
          dynamic_obs_debug << "-f:0.6";
          max_decel_for_front_object *= 0.6;
        } else if (ego_acc > 0.5 && !front_obj_is_leaving &&
                   impending_collision_scene) {
          dynamic_obs_debug << "-f:0.7";
          max_decel_for_front_object *= 0.7;
        } else if ((obs_acc < std::min(-1.0, ego_acc) || ego_acc > 0.1) &&
                   !front_obj_is_leaving && impending_collision_scene) {
          dynamic_obs_debug << "-f:0.8";
          max_decel_for_front_object *= 0.8;
        } else if (front_obj_is_leaving) {
          dynamic_obs_debug << "-f:1.2";
          max_decel_for_front_object *= 1.2;
        } else {
          dynamic_obs_debug << "-f:1.0";
        }

        max_decel_for_front_object =
            std::clamp(max_decel_for_front_object / pause_factor, 1.0, 5.0);

        std::string dec_str = "";
        auto check_status = CheckDeceleration(
            std::fabs(rel_ds), traj_ptr->object_id(), "ego",
            (obj_v * obj_target_tangent_cos), (ego_v * ego_target_tangent_cos),
            kEgoResponseTime, conserv * rel_head_way_time,
            max_decel_for_front_object, acc_safe_dis, dec_str,
            &leader_max_decel);
        dynamic_obs_debug << "-dec:" << check_status.ok() << "-ttc:" << rel_ttc;

        if (frnt_obs_id == traj_ptr->object_id()) {
          if (check_status.ok()) {
            leader_debug_info << "/***/CheckDeceleration:" << dec_str;
          }
        }
        if (!check_status.ok()) {
          unsafe_object_id = {traj_ptr->object_id().data(),
                              traj_ptr->object_id().size()};
          status_code =
              obs_in_target_lane
                  ? (if_obs_front ? PlannerStatusProto::
                                        FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                                  : PlannerStatusProto::
                                        REAR_DANGEROUS_VEHICLE_TARGET_LANE)
                  : (if_obs_front ? PlannerStatusProto::
                                        FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                                  : PlannerStatusProto::
                                        REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
          leader_debug_info << static_obs_debug.str();
          follower_debug_info << dynamic_obs_debug.str();
          follower_set.insert(rear_obs_id);
          leader_set.insert(frnt_obs_id);
          if (max_decel_for_front_object > 2.5) {
            enable_pause_counter = true;
          }
          return absl::CancelledError(check_status.ToString());
        }
      }
    }

    if (is_distance_shortening) {
      const std::vector<double> K_FRONT_TTC_VEC = {0.83, 0.86, 0.90, 0.93,
                                                   0.96, 1.0,  1.2};
      const std::vector<double> K_BACK_TTC_VEC = {0.83, 0.86, 0.90, 0.93,
                                                  0.96, 1.0,  1.2};
      const std::vector<double> V_TTC_VEC = {0.0, 15, 20, 30, 60, 90, 120};
      double front_ttc = 2.0, back_ttc = 2.0;
      bool quick_response_type = false;
      bool is_jam_scen = false;
      double spd_diff_kph_threshold = 100;
      if (is_distance_shortening) {
        if (if_obs_front) {
          if (quick_response_type) {
            spd_diff_kph_threshold = 100;
          } else {
            spd_diff_kph_threshold = 40;
          }
        } else {
          if (is_ego_stationary) {
            spd_diff_kph_threshold = 8;
          } else if (is_ego_extra_slow) {
            spd_diff_kph_threshold = 10;
          } else {
            if (quick_response_type) {
              spd_diff_kph_threshold = 30;
            } else {
              spd_diff_kph_threshold = 30;
            }
          }
        }
        if (is_lc_state_prev) spd_diff_kph_threshold += 5;
      } else {
        spd_diff_kph_threshold = 100;
      }

      bool large_spd_diff_scene =
          (is_distance_shortening && abs_rel_vs_kph > (spd_diff_kph_threshold));

      bool rear_large_spd_acc_scene =
          (is_distance_shortening &&
           abs_rel_vs_kph > (spd_diff_kph_threshold) * 0.8 && obs_acc > 2.0 &&
           !if_obs_front);
      bool is_rear_big_approaching_vehicle =
          (is_big_vehicle && is_rear_approaching_vehicle);

      if (!is_rear_big_approaching_vehicle && !is_ego_extra_slow &&
          !large_spd_diff_scene && !rear_large_spd_acc_scene) {
        front_ttc = 0;
        back_ttc = 0;
      } else {
        double k_front_ttc = ad_e2e::planning::math::interp1_inc(
            V_TTC_VEC, K_FRONT_TTC_VEC, ego_speed_kph);

        double k_back_ttc = ad_e2e::planning::math::interp1_inc(
            V_TTC_VEC, K_BACK_TTC_VEC, Mps2Kph(obj_v));

        std::vector<double> f_dv_front_ttc_vec = {
            (front_ttc - 1.0), (front_ttc + 0.0), (front_ttc + 0.5),
            (front_ttc + 0.8), (front_ttc + 1.0), (front_ttc + 1.5)};
        std::vector<double> f_dv_back_ttc_vec = {
            (back_ttc - 0.0), (back_ttc + 0.5), (back_ttc + 1.0),
            (back_ttc + 1.5), (back_ttc + 2.0), (back_ttc + 2.5)};
        std::vector<double> dv_ttc_vec = {0.0, 20, 30, 45, 60, 120};

        double f_dv_front_ttc =
            std::max(ad_e2e::planning::math::interp1_inc(
                         dv_ttc_vec, f_dv_front_ttc_vec, abs_rel_vs_kph),
                     front_ttc);

        double f_dv_back_ttc =
            std::max(ad_e2e::planning::math::interp1_inc(
                         dv_ttc_vec, f_dv_back_ttc_vec, abs_rel_vs_kph),
                     back_ttc);

        front_ttc =
            std::max(k_front_ttc * f_dv_front_ttc, kLaneChangeMinLimitTTC);
        back_ttc = std::max(k_back_ttc * f_dv_back_ttc, kLaneChangeMinLimitTTC);

        if (is_jam_scen || large_spd_diff_scene) {
          if (!is_lc_state_prev) {
            front_ttc += (1.5 * conserv_base);
            back_ttc += (2.0 * conserv_base);
          } else {
            front_ttc += (0.5 * conserv_base);
            back_ttc += (0.5 * conserv_base);
          }
        } else {
          if (!is_lc_state_prev) {
            front_ttc += (0.7 * conserv_base);
            back_ttc += (1.0 * conserv_base);
          } else {
            front_ttc += (0.2 * conserv_base);
            back_ttc += (0.2 * conserv_base);
          }
        }
      }

      double ttc_thrd = (if_obs_front ? front_ttc : back_ttc);
      double ttc_distance = std::fabs(rel_vs) * ttc_thrd;
      double ttc_rel = std::fabs(rel_ds) / (std::fabs(rel_vs) + kEpsilon);

      bool ttc_dis_danger = std::fabs(rel_ds) < ttc_distance;
      dynamic_obs_debug << "-ttc:" << ttc_dis_danger;
      auto ttc_debug_str = absl::StrFormat(
          "/***/Check_ttc:%d. obstacle %s will collide with ego after %.3f "
          "seconds (ttc_thrd:%.3f ttc distance:%.3f rel_vs:%.3f).",
          ttc_dis_danger, traj_ptr->object_id(), ttc_rel, ttc_thrd,
          ttc_distance, rel_vs);

      if (if_obs_front && frnt_obs_id == traj_ptr->object_id()) {
        leader_debug_info << ttc_debug_str;
      } else if (if_obs_rear && rear_obs_id == traj_ptr->object_id()) {
        follower_debug_info << ttc_debug_str;
      }

      if (ttc_dis_danger) {
        unsafe_object_id = {traj_ptr->object_id().data(),
                            traj_ptr->object_id().size()};
        status_code =
            obs_in_target_lane
                ? (if_obs_front
                       ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                       : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_TARGET_LANE)
                : (if_obs_front ? PlannerStatusProto::
                                      FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                                : PlannerStatusProto::
                                      REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
        leader_debug_info << static_obs_debug.str();
        follower_debug_info << dynamic_obs_debug.str();
        follower_set.insert(rear_obs_id);
        leader_set.insert(frnt_obs_id);
        enable_pause_counter = true;
        return absl::CancelledError(ttc_debug_str);
      }
    }
  }

  constexpr double kStationaryBuffer = 0.3;
  e2e_noa::FrenetBox frnt_static_current{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  e2e_noa::FrenetBox frnt_static_target{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  std::string frnt_static_current_id = "";
  std::string frnt_static_target_id = "";

  int16_t ride_line_uu_num = 0;

  std::ostringstream ride_line_debug;
  ride_line_debug.str("");
  ride_line_debug << "ride_line_uu:";

  static_obs_debug << " frnt_stc_fltr:" << frnt_static_filter_obs_id << " ";

  for (const auto* traj_ptr : st_traj_mgr.stationary_object_trajs()) {
    conserv = conserv_base * getObjTypeFactor(traj_ptr, true);

    const bool is_vehicle = IsVehicle(traj_ptr);
    const bool is_vru = IsVRU(traj_ptr);
    const auto& obj_contour =
        !is_vehicle ? traj_ptr->contour()
                    : Polygon2d(traj_ptr->bounding_box().GetAllCorners());
    ASSIGN_OR_CONTINUE(
        const auto obj_frenet_box,
        target_frenet_frame.QueryFrenetBoxAtContour(obj_contour));

    const bool if_obs_front = (ego_cur_frenet_box.s_max < obj_frenet_box.s_min);
    const bool if_obs_rear = (obj_frenet_box.s_max < ego_cur_frenet_box.s_min);

    double rel_ds, rel_dl = 0;
    double long_overlap =
        getLongOverlap(ego_cur_frenet_box, obj_frenet_box, rel_ds);
    double lat_overlap =
        getLatOverlap(ego_cur_frenet_box, obj_frenet_box, rel_dl);
    const auto obj_cur_box = traj_ptr->states().front().box;
    const auto obj_target_tangent_cos = obj_cur_box.tangent().Dot(
        target_frenet_frame.InterpolateTangentByS(obj_frenet_box.center_s()));
    const auto obj_target_normal_cos = obj_cur_box.tangent().Dot(
        target_frenet_frame.InterpolateTangentByS(obj_frenet_box.center_s())
            .Rotate(M_PI_2));

    static_obs_debug << "-->Sobj:" << traj_ptr->object_id();

    if (rel_ds < -6.0 || rel_ds > 135 ||
        (lc_left ? rel_dl < -0.5 : rel_dl > 0.5) || std::fabs(rel_dl) > 6.0) {
      static_obs_debug << "|skp";
      continue;
    }

    double lat_overlap_lc = getLatOverlapWithLcDirection(
        ego_front_frenet_box, obj_frenet_box, rel_dl);
    const double obj_v = traj_ptr->planner_object().pose().v();
    bool obs_in_target_lane =
        HasFullyEnteredTargetLane(obj_frenet_box, 0.5 * obj_frenet_box.width());

    double ttc_thrd = is_lc_state_prev ? 6.0 : 8.0;
    double ttc_dis_danger =
        rel_ds <
        std::min(135.0, std::fmax(ego_v * ttc_thrd * conserv_base, 5.0));

    bool is_construction =
        IsConstructionObs(traj_ptr) && IsSizeQualified(traj_ptr);

    conserv = std::clamp(conserv, 0.5, 1.0);

    bool static_occupy_target = isStaticObsOccupyTargetLane(
        traj_ptr, obj_frenet_box, ego_cur_frenet_box,
        0.5 * obj_frenet_box.width(), lc_left, rel_ds);

    bool target_lane_construction = is_construction && if_obs_front &&
                                    static_occupy_target && ttc_dis_danger;

    double frt_obj_rel_ds, frt_obj_rel_dl = 0;

    if ((is_vehicle && if_obs_front && obs_in_target_lane) &&
        (frnt_static_filter_obs_id.empty() ||
         obj_frenet_box.s_min < frnt_static_filter_obs_box.s_min)) {
      frnt_static_filter_obs_id = {traj_ptr->object_id().data(),
                                   traj_ptr->object_id().size()};
      frnt_static_filter_obs_box = obj_frenet_box;
    }

    if (!frnt_static_filter_obs_id.empty() &&
        frnt_static_filter_obs_id != traj_ptr->object_id()) {
      auto frt_obj_long_overlap = getLongOverlap(
          frnt_static_filter_obs_box, obj_frenet_box, frt_obj_rel_ds);
      auto frt_obj_lat_overlap = getLatOverlap(frnt_static_filter_obs_box,
                                               obj_frenet_box, frt_obj_rel_dl);
    }

    if (!frnt_static_filter_obs_id.empty() && target_lane_construction &&
        frt_obj_rel_ds >= 0 && std::fabs(frt_obj_rel_dl) < 0.3) {
      static_obs_debug << "|frt_obj:" << frnt_static_filter_obs_id
                       << "-rel_ds:" << frt_obj_rel_ds
                       << "-rel_dl:" << frt_obj_rel_dl;

      static_obs_debug << "|FDt";
      continue;
    }

    bool ride_middle_line = isStaticObsRideLine(
        ego_cur_frenet_box, obj_frenet_box, lc_left, rel_ds);

    bool ride_line_condition =
        is_construction && ride_middle_line && rel_ds >= -6.0;

    bool occupy_current_condition =
        !static_occupy_target && ego_enter_target_idx != 0 &&
        (is_vehicle || is_construction) && if_obs_front && rel_dl == 0 &&
        obj_v < kStationaryVehicleSpeedkph;

    static_obs_debug << "|c:" << occupy_current_condition
                     << "-r:" << ride_middle_line
                     << "-o:" << static_occupy_target;

    static_obs_debug << "|cc:" << occupy_current_condition
                     << "-rc:" << ride_line_condition
                     << "-tc:" << target_lane_construction;

    if (ride_line_condition) {
      ride_line_uu_num++;
      ride_line_debug << traj_ptr->object_id() << "->";
    }

    if (occupy_current_condition || ride_line_condition) {
      g_MinLonBuffer = 5.0 * 1.0;

      std::vector<double> lat_threshold = {0.2, 1.0, 1.5, 2.0, 4.0};
      std::vector<double> lon_dis = {0.5, 3.0, 4.8, 6.0, 10.0};
      auto lat_overlap_safe_dis = ad_e2e::planning::math::interp1_inc(
          lat_threshold, lon_dis, lat_overlap_lc);

      double lat_overlap_factor =
          std::clamp(lat_overlap_lc <= 1.5 * vehicle_geom.width()
                         ? (lat_overlap_lc / vehicle_geom.width())
                         : (lat_overlap_lc + vehicle_geom.width()) /
                               (2 * vehicle_geom.width()),
                     0.0, 2.5);

      double state_debounce_factor =
          ((prev_lc_stage != LaneChangeStage::LCS_EXECUTING ||
            std::fabs(ego_target_normal_cos) < 0.06)
               ? 1.0
               : 0.8);

      auto front_safe_dis = std::max(
          lat_overlap_safe_dis,
          ego_v * ((is_vehicle && if_obs_front) ? 3.5 : 3.0) * conserv_base *
              lat_overlap_factor * state_debounce_factor);

      bool front_obs_too_close = rel_ds < front_safe_dis;

      auto front_veh_debug_str = absl::StrFormat(
          "stationary obs %s currently "
          "close:%d.lat_overlap:%.3f lat_overlap_lc:%.3f "
          "front_safe_dis:%.3f rel_ds:%.3f conserv_base:%.3f"
          "conserv:%.3f lat_overlap_safe_dis:%.3f ego_v:%.3f ",
          traj_ptr->object_id(), front_obs_too_close, lat_overlap,
          lat_overlap_lc, front_safe_dis, rel_ds, conserv_base, conserv,
          lat_overlap_safe_dis, ego_v);

      if (front_obs_too_close) {
        unsafe_object_id = {traj_ptr->object_id().data(),
                            traj_ptr->object_id().size()};
        if (is_construction) {
          status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
        } else {
          status_code =
              PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_CURRENT_LANE;
        }

        leader_debug_info << static_obs_debug.str();
        follower_set.insert(rear_obs_id);
        leader_set.insert(frnt_obs_id);
        enable_pause_counter = true;
        return absl::CancelledError(front_veh_debug_str);
      }
    } else if (target_lane_construction) {
      if (static_occupy_target) {
        unsafe_object_id = {traj_ptr->object_id().data(),
                            traj_ptr->object_id().size()};
        status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
        leader_debug_info << static_obs_debug.str();
        follower_set.insert(rear_obs_id);
        leader_set.insert(frnt_obs_id);
        enable_pause_counter = true;
        return absl::CancelledError(
            absl::StrFormat("UU or cone %s currently occupy target lane.",
                            traj_ptr->object_id()));
      }
    } else if (ride_line_uu_num >= 3 && ride_line_condition) {
      unsafe_object_id = {traj_ptr->object_id().data(),
                          traj_ptr->object_id().size()};
      status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
      leader_debug_info << static_obs_debug.str();
      follower_set.insert(rear_obs_id);
      leader_set.insert(frnt_obs_id);
      enable_pause_counter = true;
      return absl::CancelledError(
          absl::StrFormat("Many uu or cone static obstacles ride line %s. %s",
                          traj_ptr->object_id(), ride_line_debug.str()));
    } else {
      g_MinLonBuffer = kMinLonBufferToFront;
    }

    if (obj_frenet_box.s_max < ego_cur_frenet_box.s_min ||
        std::abs(obj_frenet_box.center_l()) >
            0.5 * obj_frenet_box.width() + ego_half_width + kStationaryBuffer) {
      static_obs_debug << "|skp2";
      continue;
    }

    if (if_obs_front && obs_in_target_lane && (is_vehicle || is_vru) &&
        (frnt_obs_id.empty() || obj_frenet_box.s_min < frnt_obs_box.s_min)) {
      frnt_obs_id = {traj_ptr->object_id().data(),
                     traj_ptr->object_id().size()};
      frnt_obs_box = obj_frenet_box;
    }

    for (int i = 0; i < ego_traj_size; ++i) {
      if ((is_vehicle && static_occupy_target) ||
          obj_contour.HasOverlap(ego_boxes[i])) {
        static_obs_debug << "|overlap:" << i << "-occ:" << static_occupy_target;
        const double rel_head_way_time =
            if_obs_front
                ? ComputeEgoFollowTime(obj_v, ego_v, is_lc_state_not_pause_prev)
                : ComputeEgoLeadTime(speed_limit, ego_v, obj_v,
                                     is_lc_state_not_pause_prev);
        const auto follower_vel = if_obs_front ? ego_v : obj_v;

        const double obj_front_extension =
            conserv * follower_vel * rel_head_way_time * ego_target_tangent_cos;
        const double obj_lat_extension =
            -conserv * follower_vel * rel_head_way_time * ego_target_normal_cos;

        bool is_abreast =
            IsBlockingObjectAbreast(ego_cur_frenet_box, obj_frenet_box,
                                    obj_front_extension, obj_lat_extension);
        static_obs_debug << "|abr:" << is_abreast;

        if (is_abreast) {
          unsafe_object_id = {traj_ptr->object_id().data(),
                              traj_ptr->object_id().size()};
          status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
          leader_debug_info << static_obs_debug.str();
          follower_set.insert(rear_obs_id);
          leader_set.insert(frnt_obs_id);
          return absl::CancelledError(absl::StrFormat(
              "Object %s currently abreast.", traj_ptr->object_id()));
        }

        bool ego_is_entered = (ego_enter_target_idx == 0 && ego_occupy_target);
        static_obs_debug << "|etr:" << ego_is_entered;
        if (ego_is_entered || ego_centered_in_target_lane) {
          break;
        }

        const double lon_dist = obj_frenet_box.s_min - ego_cur_frenet_box.s_max;

        double max_decel_for_front_object = ad_e2e::planning::math::interp1_inc(
            DV_TTC_VEC, F_DV_STATIC_DEC_VEC, Mps2Kph(ego_v));

        if (is_lc_state_prev) {
          double debounce_braking_deceleration =
              std::clamp(((1.0 - conserv_base) * 2.0), 0.0, 1.5);
          max_decel_for_front_object += debounce_braking_deceleration;
          max_decel_for_front_object =
              std::clamp(max_decel_for_front_object, 0.0, 5.0);
        }

        if (is_vru) {
          max_decel_for_front_object *= 0.6;
        }
        std::string dec_str = "";
        auto check_status = CheckDeceleration(
            lon_dist, traj_ptr->object_id(), "ego", 0.0,
            (ego_v * ego_target_tangent_cos), 0.0, kEgoFollowTimeBuffer,
            max_decel_for_front_object, 0.0, dec_str, &leader_max_decel);
        static_obs_debug << "|dec:" << check_status.ok()
                         << "-max_dec:" << max_decel_for_front_object
                         << "-hpy_dec:" << leader_max_decel << " ";
        if (!check_status.ok()) {
          unsafe_object_id = {traj_ptr->object_id().data(),
                              traj_ptr->object_id().size()};
          status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
          leader_debug_info << static_obs_debug.str();
          follower_set.insert(rear_obs_id);
          leader_set.insert(frnt_obs_id);
          enable_pause_counter = true;
          return check_status;
        }
        break;
      }
    }
    static_obs_debug << "|sf:";
  }

  const auto& nearest_front_obj_ptr =
      st_traj_mgr.FindObjectByObjectId(frnt_obs_id);

  if (ego_speed_kph < 60 && !frnt_obs_id.empty() && !is_lc_state_prev &&
      nearest_front_obj_ptr && IsVehicle(nearest_front_obj_ptr)) {
    std::vector<std::string> leading_trajs_id{};
    std::string delimiter = "-";
    std::string leading_objs_str = "";
    for (const auto& obj_traj_str : leading_trajs) {
      size_t pos = obj_traj_str.find(delimiter);

      if (pos != std::string::npos) {
        leading_trajs_id.emplace_back(obj_traj_str.substr(0, pos));
        leading_objs_str += leading_trajs_id.back();
        leading_objs_str += "-";
      }
    }

    auto itr_nearest_front = std::find(leading_trajs_id.begin(),
                                       leading_trajs_id.end(), frnt_obs_id);

    if (itr_nearest_front == leading_trajs_id.end()) {
      auto leading_debug_str = absl::StrFormat(
          "/***/Not enter the gap behind leader_objs:%s."
          "(the nearest vehicle ahead:%s)",
          leading_objs_str, frnt_obs_id);
      return absl::CancelledError(leading_debug_str);
    }
  }

  follower_set.insert(rear_obs_id);
  leader_set.insert(frnt_obs_id);

  status_code = PlannerStatusProto::OK;
  leader_debug_info << static_obs_debug.str();
  follower_debug_info << dynamic_obs_debug.str();
  return absl::OkStatus();
}

}  // namespace e2e_noa::planning
