#include "speed/decision/interaction_util.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "absl/types/span.h"
#include "math/geometry/box2d.h"
#include "math/geometry/polygon2d.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/spacetime_object_state.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "speed/speed_point.h"
#include "util/path_util.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa {
namespace planning {

namespace {
constexpr double kFovAngle = M_PI * 75.0 / 180.0;

constexpr double kAccPreviewTime = 1.0;
constexpr double kVelocityFluctuationRange = 0.5;
constexpr double kPreliminarySpeedTimeLimit = 3.0;
constexpr double kMaxLatAcc = 2.5;
constexpr double kKappaPreviewDist = 10.0;
constexpr double kAvMaxAcc = 0.5;

constexpr double kEps = 1e-3;

inline double CalculateAvAvgAcceleration(const SpeedVector& preliminary_speed,
                                         double t) {
  if (t < kEps) return 0.0;
  const auto speed_point = preliminary_speed.EvaluateByTime(t);
  if (!speed_point.has_value()) {
    return 0.0;
  }
  return (speed_point->v() - preliminary_speed.front().v()) / t;
}

}  // namespace

TtcInfo GetAvOverlapTtcInformation(
    const std::vector<OverlapInfo>& overlap_infos,
    const StOverlapMetaProto& overlap_meta, const DiscretizedPath& path,
    const SpeedVector& preliminary_speed) {
  CHECK(!preliminary_speed.empty());
  CHECK(!overlap_infos.empty());
  TtcInfo ttc_info;
  const bool is_high_priority =
      overlap_meta.priority() == StOverlapMetaProto::HIGH;
  const auto& fo_info = overlap_infos.front();
  double s = path[fo_info.av_end_idx].s();
  if (is_high_priority) {
    for (const auto& overlap_info : overlap_infos) {
      s = std::min(path[overlap_info.av_start_idx].s(), s);
    }
  }

  const auto speed_point = preliminary_speed.EvaluateByS(s);
  const double current_v = preliminary_speed.front().v();
  const double min_ttc =
      (-current_v + std::sqrt(Sqr(current_v) + 2.0 * kAvMaxAcc * s)) /
      kAvMaxAcc;
  const double preliminary_speed_ttc = std::max(
      min_ttc, speed_point.has_value() ? preliminary_speed.EvaluateByS(s)->t()
                                       : preliminary_speed.back().t());

  const double v = preliminary_speed_ttc > kPreliminarySpeedTimeLimit
                       ? s / preliminary_speed_ttc
                       : current_v;
  ttc_info.ttc_lower_limit = s / std::max(v + kVelocityFluctuationRange, kEps);
  ttc_info.ttc_upper_limit = s / std::max(v - kVelocityFluctuationRange, kEps);
  return ttc_info;
}

TtcInfo GetObjectOverlapTtcInfo(const OverlapInfo& fo_info,
                                const SpacetimeObjectTrajectory& spacetime_obj,
                                const std::optional<double>& reaction_time) {
  TtcInfo ttc_info;
  const auto& states = spacetime_obj.states();
  const double s = states[fo_info.obj_idx].traj_point->s();

  int kappa_size = 0;
  double avg_kappa = 0;
  for (int i = 0; i < states.size(); ++i) {
    if (states[i].traj_point->s() < s - kKappaPreviewDist) {
      continue;
    }
    if (states[i].traj_point->s() > s + kKappaPreviewDist) {
      break;
    }
    avg_kappa += states[i].traj_point->kappa();
    ++kappa_size;
  }
  avg_kappa = avg_kappa / static_cast<double>(kappa_size);

  const double vel_limit = std::sqrt(kMaxLatAcc / std::abs(avg_kappa));
  const double current_a = spacetime_obj.planner_object().pose().a();
  const double obj_reaction_time =
      reaction_time.has_value() ? *reaction_time : kAccPreviewTime;
  const double preview_v =
      spacetime_obj.planner_object().pose().v() + obj_reaction_time * current_a;
  const double v =
      vel_limit > preview_v ? preview_v : 0.5 * (preview_v + vel_limit);

  ttc_info.ttc_lower_limit = s / std::max(v + kVelocityFluctuationRange, kEps);
  ttc_info.ttc_upper_limit = s / std::max(v - kVelocityFluctuationRange, kEps);
  return ttc_info;
}

bool IsAvInObjectFov(const PathPoint& current_path_point,
                     const PlannerObject& planner_object,
                     const VehicleGeometryParamsProto& vehicle_geo_params) {
  const auto av_box =
      ComputeAvBox(ToVec2d(current_path_point), current_path_point.theta(),
                   vehicle_geo_params);
  const auto& obj_pose = planner_object.pose();
  const auto& obj_center = obj_pose.pos();
  const auto obj_heading = obj_pose.theta();
  for (const auto& corner : av_box.GetCornersCounterClockwise()) {
    if (std::abs(NormalizeAngle((corner - obj_center).FastAngle() -
                                obj_heading)) < kFovAngle) {
      return true;
    }
  }
  return false;
}

bool IsAvCompletelyInObjectFov(
    const PathPoint& current_path_point, const PlannerObject& planner_object,
    const VehicleGeometryParamsProto& vehicle_geo_params) {
  const auto av_box =
      ComputeAvBox(ToVec2d(current_path_point), current_path_point.theta(),
                   vehicle_geo_params);
  const auto& obj_pose = planner_object.pose();
  const auto& obj_center = obj_pose.pos();
  const auto obj_heading = obj_pose.theta();
  for (const auto& corner : av_box.GetCornersCounterClockwise()) {
    if (std::abs(NormalizeAngle((corner - obj_center).FastAngle() -
                                obj_heading)) >= kFovAngle) {
      return false;
    }
  }
  return true;
}

bool IsObjectInAvFov(const PathPoint& current_path_point,
                     const PlannerObject& planner_object) {
  const auto& obj_contour = planner_object.contour();
  const auto& av_pos = ToVec2d(current_path_point);
  const auto av_heading = current_path_point.theta();
  for (const auto& corner : obj_contour.GetAllVertices()) {
    if (std::abs(NormalizeAngle((corner - av_pos).FastAngle() - av_heading)) <
        kFovAngle) {
      return true;
    }
  }
  return false;
}

double CalcObjectYieldingTime(const std::vector<OverlapInfo>& overlap_infos,
                              const StOverlapMetaProto& overlap_meta,
                              const DiscretizedPath& path,
                              const SpeedVector& preliminary_speed) {
  CHECK(!preliminary_speed.empty());
  CHECK(!overlap_infos.empty());
  const bool is_lane_cross =
      overlap_meta.source() == StOverlapMetaProto::LANE_CROSS;
  const auto& fo_info = overlap_infos.front();

  double s = path[fo_info.av_start_idx].s();
  if (is_lane_cross) {
    for (const auto& overlap_info : overlap_infos) {
      s = std::max(path[overlap_info.av_end_idx].s(), s);
    }
  }
  const auto av_speed_point = s > preliminary_speed.back().s()
                                  ? preliminary_speed.back()
                                  : preliminary_speed.EvaluateByS(s);

  const double current_v = preliminary_speed.front().v();
  const double min_ttc =
      (-current_v + std::sqrt(Sqr(current_v) + 2 * kAvMaxAcc * s)) / kAvMaxAcc;
  const double object_yielding_time_preliminary_speed =
      std::max(min_ttc, av_speed_point->t());
  const double object_yielding_time_spacetime = s / (current_v + kEps);

  const double object_yielding_time =
      object_yielding_time_preliminary_speed > kPreliminarySpeedTimeLimit
          ? object_yielding_time_preliminary_speed
          : object_yielding_time_spacetime;
  return object_yielding_time;
}

bool IsParallelMerging(const StOverlapMetaProto& overlap_meta) {
  constexpr double kParallelHeadingThreshold = M_PI / 18.0;
  return overlap_meta.source() == StOverlapMetaProto::LANE_MERGE &&
         overlap_meta.has_theta_diff() &&
         std::abs(overlap_meta.theta_diff()) < kParallelHeadingThreshold;
}

bool IsAggressiveLeading(const SpeedVector& preliminary_speed,
                         double first_overlap_time) {
  const double av_avg_acc =
      CalculateAvAvgAcceleration(preliminary_speed, first_overlap_time);
  constexpr double kAvAggressiveLeadAcc = 0.8;
  return av_avg_acc > kAvAggressiveLeadAcc;
}

bool HasYieldingIntentionToFrontAv(
    const PathPoint& current_path_point, const PlannerObject& planner_object,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const StOverlapMetaProto& overlap_meta,
    const SpeedVector& preliminary_speed, double first_overlap_time) {
  if (overlap_meta.priority() == StOverlapMetaProto::LOW) return false;
  if (IsAggressiveLeading(preliminary_speed, first_overlap_time)) return false;
  const bool is_av_completely_in_object_fov = IsAvCompletelyInObjectFov(
      current_path_point, planner_object, vehicle_geo_params);
  if (is_av_completely_in_object_fov) return true;
  const double object_a = planner_object.pose().a();
  const bool is_av_in_object_fov =
      IsAvInObjectFov(current_path_point, planner_object, vehicle_geo_params);
  constexpr double kAggressiveLeadAcc = 0.3;
  return is_av_in_object_fov && object_a < kAggressiveLeadAcc;
}

}  // namespace planning
}  // namespace e2e_noa
