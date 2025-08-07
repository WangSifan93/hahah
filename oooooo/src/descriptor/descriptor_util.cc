#include "descriptor/descriptor_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "affine_transformation.pb.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "object/spacetime_object_state.h"
#include "plan/planner_defs.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction_util.h"
#include "util/status_macros.h"

namespace e2e_noa {
namespace planning {

namespace {

constexpr double kEpsilon = 1e-3;

SpeedProfile IntegrateSpeedProfile(const PlanPassage& plan_passage,
                                   const std::vector<double>& v_s) {
  CHECK_EQ(plan_passage.size(), v_s.size());
  std::vector<double> path_s(plan_passage.size());
  for (int i = 0; i < plan_passage.size(); ++i) {
    path_s[i] = plan_passage.station(StationIndex(i)).accumulated_s();
  }
  const PiecewiseSqrtFunction<double, double> v_s_plf(path_s, v_s);
  std::vector<double> t(kTrajectorySteps);
  for (int i = 0; i < t.size(); ++i) t[i] = i * kTrajectoryTimeStep;
  std::vector<double> s(t.size());
  double current_s = 0.0;
  for (int i = 0; i < t.size(); ++i) {
    s[i] = current_s;
    const double current_v = v_s_plf(current_s);
    current_s += std::max(current_v, kEpsilon) * kTrajectoryTimeStep;
    current_s = std::min(path_s.back(), current_s);
  }

  return SpeedProfile(PiecewiseLinearFunction(std::move(t), std::move(s)));
}

void ApplySpeedConstraintForReferenceSpeedProfile(
    double s_start, double s_end, double v_max, const PlanPassage& plan_passage,
    std::vector<double>* v_s) {
  CHECK_LE(s_start, s_end);
  CHECK_EQ(plan_passage.size(), v_s->size());

  const double v_max_sqr = Sqr(v_max);
  const auto compute_v_constraint_sqr = [s_start, s_end, v_max_sqr](double s) {
    constexpr double kAccelLimit = 2.0;
    if (s < s_start) {
      return v_max_sqr + (s_start - s) * (kAccelLimit * 2.0);
    }
    if (s > s_end) {
      return v_max_sqr + (s - s_end) * (kAccelLimit * 2.0);
    }
    return v_max_sqr;
  };

  for (int i = 0; i < plan_passage.size(); ++i) {
    const double s = plan_passage.station(StationIndex(i)).accumulated_s();
    const double v_constraint_sqr = compute_v_constraint_sqr(s);
    if (v_constraint_sqr < Sqr((*v_s)[i])) {
      (*v_s)[i] = std::sqrt(v_constraint_sqr);
    }
  }
}

}  // namespace

SpeedProfile CreateSpeedProfile(
    double v_now, const PlanPassage& plan_passage,
    const PlannerSemanticMapManager& psmm,
    const absl::Span<const ConstraintProto::SpeedRegionProto>& speed_zones,
    const absl::Span<const ConstraintProto::StopLineProto>& stop_points) {
  const auto dp_size = plan_passage.size();

  std::vector<double> v_s;
  v_s.reserve(dp_size);
  for (int i = 0; i < dp_size; ++i) {
    const double speed_limit_mps =
        plan_passage.station(StationIndex(i)).speed_limit();
    v_s.push_back(speed_limit_mps);
  }

  for (const auto& speed_zone : speed_zones) {
    ApplySpeedConstraintForReferenceSpeedProfile(
        speed_zone.start_s(), speed_zone.end_s(), speed_zone.max_speed(),
        plan_passage, &v_s);
  }
  for (const auto& stop_point : stop_points) {
    ApplySpeedConstraintForReferenceSpeedProfile(
        stop_point.s(), std::numeric_limits<double>::infinity(), 0.0,
        plan_passage, &v_s);
  }

  for (int i = 0; i < dp_size; ++i) {
    const double s = plan_passage.station(StationIndex(i)).accumulated_s();

    const double v_constraint =
        std::pow(std::abs(s) + pow(std::abs(v_now) / 2.3, 2.5), 0.4) * 2.3;
    (v_s)[i] = std::min((v_s)[i], v_constraint);
  }

  return IntegrateSpeedProfile(plan_passage, v_s);
}

ConstraintProto::SpeedRegionProto MergeSameElement(
    absl::Span<const ConstraintProto::SpeedRegionProto> elements) {
  CHECK_GT(elements.size(), 0);
  ConstraintProto::SpeedRegionProto merged_ele = elements[0];
  int begin_idx = 0;
  double min_s = std::numeric_limits<double>::infinity();
  int end_idx = 0;
  double max_s = -std::numeric_limits<double>::infinity();
  for (int i = 0, n = elements.size(); i < n; ++i) {
    const auto& ele = elements[i];
    if (ele.start_s() < min_s) {
      min_s = ele.start_s();
      begin_idx = i;
    }
    if (ele.end_s() > max_s) {
      max_s = ele.end_s();
      end_idx = i;
    }
  }
  merged_ele.set_start_s(min_s);
  merged_ele.set_end_s(max_s);
  *merged_ele.mutable_start_point() = elements[begin_idx].start_point();
  *merged_ele.mutable_end_point() = elements[end_idx].end_point();
  return merged_ele;
}

void FillDecisionConstraintDebugInfo(const ConstraintManager& constraint_mgr,
                                     ConstraintProto* constraint) {
  CHECK_NOTNULL(constraint);
  constraint->Clear();
  for (const auto& stop_line : constraint_mgr.StopLine()) {
    constraint->add_stop_line()->CopyFrom(stop_line);
  }
  for (const auto& speed_region : constraint_mgr.SpeedRegion()) {
    constraint->add_speed_region()->CopyFrom(speed_region);
  }
  for (const auto& path_speed_region : constraint_mgr.PathSpeedRegion()) {
    constraint->add_path_speed_region()->CopyFrom(path_speed_region);
  }
  for (const auto& path_stop_line : constraint_mgr.PathStopLine()) {
    constraint->add_path_stop_line()->CopyFrom(path_stop_line);
  }
  for (const auto& avoid_line : constraint_mgr.AvoidLine()) {
    constraint->add_avoid_line()->CopyFrom(avoid_line);
  }
  for (const auto& speed_profile : constraint_mgr.SpeedProfiles()) {
    constraint->add_speed_profile()->CopyFrom(speed_profile);
  }

  const auto& traffic_gap = constraint_mgr.TrafficGap();
  if (traffic_gap.leader_id.has_value()) {
    constraint->mutable_traffic_gap()->set_leader_id(*traffic_gap.leader_id);
  }
  if (traffic_gap.follower_id.has_value()) {
    constraint->mutable_traffic_gap()->set_follower_id(
        *traffic_gap.follower_id);
  }
}

bool IsLeadingObjectType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_PEDESTRIAN:

    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

std::pair<double, double> CalcSlBoundaries(const PathSlBoundary& sl_boundary,
                                           const FrenetBox& frenet_box) {
  if (frenet_box.s_max < sl_boundary.start_s() ||
      frenet_box.s_min > sl_boundary.end_s()) {
    return {0.0, 0.0};
  }

  const auto [boundary_front_right_l, boundary_front_left_l] =
      sl_boundary.QueryBoundaryL(frenet_box.s_max);
  const double boundary_front_center_l =
      sl_boundary.QueryReferenceCenterL(frenet_box.s_max);

  const auto [boundary_rear_right_l, boundary_rear_left_l] =
      sl_boundary.QueryBoundaryL(frenet_box.s_min);
  const double boundary_rear_center_l =
      sl_boundary.QueryReferenceCenterL(frenet_box.s_min);

  constexpr double kMinHalfBoundaryWidth = 1.1;
  const double boundary_l_min =
      std::min(std::min(boundary_front_right_l,
                        boundary_front_center_l - kMinHalfBoundaryWidth),
               std::min(boundary_rear_right_l,
                        boundary_rear_center_l - kMinHalfBoundaryWidth));
  const double boundary_l_max =
      std::max(std::max(boundary_front_left_l,
                        boundary_front_center_l + kMinHalfBoundaryWidth),
               std::max(boundary_rear_left_l,
                        boundary_rear_center_l + kMinHalfBoundaryWidth));

  return {boundary_l_max, boundary_l_min};
}

ConstraintProto::LeadingObjectProto CreateLeadingObject(
    const SpacetimeObjectTrajectory& traj, const PlanPassage& passage,
    ConstraintProto::LeadingObjectProto::Reason reason, bool is_group_tail) {
  ConstraintProto::LeadingObjectProto leading_object;
  leading_object.set_traj_id(std::string(traj.traj_id()));
  leading_object.set_reason(reason);
  leading_object.set_is_group_tail(is_group_tail);

  constexpr double kSampleTimeInterval = 1.0;

  if (traj.is_stationary()) {
    ASSIGN_OR_RETURN(const auto obj_frenet_box,
                     passage.QueryFrenetBoxAt(traj.bounding_box()),
                     leading_object);
    for (double sample_time = 0.0; sample_time <= kTrajectoryTimeHorizon;
         sample_time += kSampleTimeInterval) {
      auto* constraint = leading_object.add_st_constraints();
      constraint->set_s(obj_frenet_box.s_min);
      constraint->set_t(sample_time);
    }
    if (kTrajectoryTimeHorizon - leading_object.st_constraints().rbegin()->t() >
        kEpsilon) {
      auto* constraint = leading_object.add_st_constraints();
      constraint->set_s(obj_frenet_box.s_min);
      constraint->set_t(kTrajectoryTimeHorizon);
    }

    return leading_object;
  }

  double sample_time = 0.0;
  const double traj_last_time = traj.states().back().traj_point->t();

  for (const auto& state : traj.states()) {
    const auto* traj_point = state.traj_point;
    const double t = traj_point->t();

    if (std::abs(t - sample_time) > kEpsilon &&
        std::abs(traj_last_time - t) > kEpsilon) {
      continue;
    }

    ASSIGN_OR_CONTINUE(const auto obj_frenet_box,
                       passage.QueryFrenetBoxAt(state.box));

    ASSIGN_OR_CONTINUE(const auto passage_tangent,
                       passage.QueryTangentAngleAtS(obj_frenet_box.s_min));
    const double angle_diff =
        std::abs(NormalizeAngle(passage_tangent - traj_point->theta()));
    if (angle_diff > M_PI_2) continue;

    auto* constraint = leading_object.add_st_constraints();
    constraint->set_s(obj_frenet_box.s_min);
    constraint->set_t(t);

    sample_time += kSampleTimeInterval;
  }

  prediction::ExtendPredictionTraj(traj, &leading_object);

  return leading_object;
}

bool IsTrafficLightControlledLane(const ad_e2e::planning::Lane& lane) {
  return lane.light_status() != ad_e2e::planning::LightStatus::NONE_LIGHT;
}

}  // namespace planning
}  // namespace e2e_noa
