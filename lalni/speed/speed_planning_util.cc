#include "speed/speed_planning_util.h"

#include <algorithm>
#include <limits>
#include <ostream>
#include <utility>

#include "aabox3d.pb.h"
#include "absl/strings/str_cat.h"
#include "math/geometry/box2d.h"
#include "math/piecewise_linear_function.h"
#include "math/vec.h"
#include "plan/planner_defs.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/map_util.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa {
namespace planning {

namespace {

constexpr double kEpsilon = 0.01;

}

void KeepNearestStationarySpacetimeTrajectoryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  double nearest_stationary_s = std::numeric_limits<double>::max();
  const std::string* nearest_stationary_id_ptr = nullptr;
  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    const auto& st_boundary = *st_boundary_with_decision.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary()) continue;
    if (st_boundary.min_s() < kEpsilon) continue;
    if (const double follow_s =
            st_boundary.min_s() -
            st_boundary_with_decision.follow_standstill_distance();
        follow_s < nearest_stationary_s) {
      nearest_stationary_s = follow_s;
      nearest_stationary_id_ptr = &st_boundary.id();
    }
  }
  if (nearest_stationary_id_ptr != nullptr) {
    st_boundaries_with_decision->erase(
        std::remove_if(
            st_boundaries_with_decision->begin(),
            st_boundaries_with_decision->end(),
            [nearest_stationary_id_ptr](
                const StBoundaryWithDecision& st_boundary_with_decision) {
              const auto& st_boundary =
                  *st_boundary_with_decision.raw_st_boundary();
              if (st_boundary.source_type() !=
                  StBoundarySourceTypeProto::ST_OBJECT) {
                return false;
              }
              if (!st_boundary.is_stationary()) return false;
              if (st_boundary.min_s() < kEpsilon) return false;
              return st_boundary.id() != *nearest_stationary_id_ptr;
            }),
        st_boundaries_with_decision->end());
  }
}
std::optional<std::string> GetNearestStationaryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  double nearest_stationary_s = std::numeric_limits<double>::max();
  std::optional<std::string> nearest_stationary_id = std::nullopt;
  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    const auto& st_boundary = *st_boundary_with_decision.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary() &&
        (std::abs(st_boundary_with_decision.obj_pose_info().v()) >
         Kph2Mps(2.0)))
      continue;
    if (st_boundary.min_s() < kEpsilon) continue;
    if (const double follow_s = st_boundary.min_s();
        follow_s < nearest_stationary_s) {
      nearest_stationary_s = follow_s;
      nearest_stationary_id = st_boundary.id();
    }
  }
  return nearest_stationary_id;
}

void ConfigureSpacetimeBoundaryDebug(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    SpeedPlanningDebugProto* speed_planning_proto) {
  CHECK_NOTNULL(speed_planning_proto);
  speed_planning_proto->mutable_st_boundaries()->clear();
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    StBoundaryProto st_boundary_proto;
    const auto* st_boundary = boundary_with_decision.st_boundary();
    st_boundary_proto.set_decision_type(boundary_with_decision.decision_type());
    st_boundary_proto.set_decision_reason(
        boundary_with_decision.decision_reason());
    st_boundary_proto.set_decision_info(
        std::string(boundary_with_decision.decision_info()));
    st_boundary_proto.set_follow_standstill_distance(
        boundary_with_decision.follow_standstill_distance());
    st_boundary_proto.set_lead_standstill_distance(
        boundary_with_decision.lead_standstill_distance());
    st_boundary_proto.set_object_type(st_boundary->object_type());
    st_boundary_proto.set_probability(st_boundary->probability());
    st_boundary_proto.set_min_s(st_boundary->min_s());
    st_boundary_proto.set_max_s(st_boundary->max_s());
    st_boundary_proto.set_min_t(st_boundary->min_t());
    st_boundary_proto.set_max_t(st_boundary->max_t());
    *st_boundary_proto.mutable_decision_prob() =
        boundary_with_decision.decision_prob();
    st_boundary_proto.set_is_stationary(st_boundary->is_stationary());

    speed_planning_proto->mutable_st_boundaries()->insert(
        {st_boundary->id(), st_boundary_proto});
  }
}

std::vector<StBoundaryWithDecision> InitializeStBoundaryWithDecision(
    std::vector<StBoundaryRef> raw_st_boundaries) {
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.reserve(raw_st_boundaries.size());
  for (auto& st_boundary : raw_st_boundaries) {
    st_boundaries_with_decision.emplace_back(std::move(st_boundary));
  }
  return st_boundaries_with_decision;
}

int GetSpeedPlanningTrajectorySteps(double init_v, double default_speed_limit) {
  constexpr double kMaxSpeedOfFixedTrajectorySteps = 22.23;
  if (default_speed_limit <= kMaxSpeedOfFixedTrajectorySteps) {
    return kTrajectorySteps;
  }

  const PiecewiseLinearFunction<double> plf = {
      {kMaxSpeedOfFixedTrajectorySteps, default_speed_limit},
      {kTrajectorySteps, kSpeedPlanningMaxTrajectorySteps}};
  return static_cast<int>(plf(init_v));
}

void PostProcessSpeedByFullStop(
    const SpeedPlanningParamsProto& speed_planning_params,
    SpeedVector* speed_data) {
  CHECK_NOTNULL(speed_data);

  bool force_brake = true;
  int i = 0;
  while ((*speed_data)[i].t() <
         speed_planning_params.full_stop_traj_time_threshold()) {
    if ((*speed_data)[i].v() >
        speed_planning_params.full_stop_speed_threshold()) {
      force_brake = false;
      break;
    }
    ++i;
  }
  if (speed_data->TotalLength() >=
      speed_planning_params.full_stop_traj_length_threshold()) {
    force_brake = false;
  }
  if (force_brake) {
    for (int i = 0; i < speed_data->size(); ++i) {
      (*speed_data)[i].set_s(0.0);
      (*speed_data)[i].set_v(0.0);
      (*speed_data)[i].set_a(0.0);
      (*speed_data)[i].set_j(0.0);
    }
  }
}

std::vector<Box2d> BuildAvHeadBoundingBoxes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<VehicleShapeBasePtr>& av_shapes) {
  const double hc_to_gc =
      0.5 * (vehicle_geom.back_edge_to_center() + vehicle_geom.wheel_base());
  const double half_length = 0.5 * vehicle_geom.length() - hc_to_gc;
  const double half_width = vehicle_geom.width() * 0.5;
  std::vector<Box2d> av_head_boxes;
  av_head_boxes.reserve(av_shapes.size());
  for (const auto& av_shape : av_shapes) {
    const auto heading = av_shape->heading();
    const Vec2d tangent = Vec2d::FastUnitFromAngle(heading);
    const Vec2d center = av_shape->center() + tangent * hc_to_gc;
    av_head_boxes.push_back(
        Box2d(half_length, half_width, center, heading, tangent));
  }
  return av_head_boxes;
}

std::vector<VehicleShapeBasePtr> BuildAvShapes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path_points) {
  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  const double rac_to_center = half_length - vehicle_geom.back_edge_to_center();
  std::vector<VehicleShapeBasePtr> av_shapes;
  const int num_points = path_points.size();
  av_shapes.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const auto& pt = path_points[i];
    const double theta = pt.theta();
    const Vec2d rac(pt.x(), pt.y());
    const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
    const Vec2d center = rac + tangent * rac_to_center;
    av_shapes.push_back(std::make_unique<VehicleBoxShape>(
        vehicle_geom, rac, center, tangent, theta, half_length, half_width));
  }
  return av_shapes;
}

std::unique_ptr<KdtreeSegmentMatcher> BuildPathKdTree(
    const DiscretizedPath& path_points) {
  std::vector<Vec2d> points;
  points.reserve(path_points.size());
  for (const auto& point : path_points) {
    points.emplace_back(point.x(), point.y());
  }
  return std::make_unique<KdtreeSegmentMatcher>(points);
}

std::optional<PathApprox> BuildPathApproxForMirrors(
    const PathApprox& path_approx,
    const VehicleGeometryParamsProto& vehicle_geom) {
  if (!vehicle_geom.has_left_mirror() || !vehicle_geom.has_right_mirror()) {
    return std::nullopt;
  }

  const auto& mirror = vehicle_geom.left_mirror();
  if (!mirror.has_width() || !mirror.has_length()) {
    return std::nullopt;
  }
  std::vector<PathSegment> mirror_segments;
  mirror_segments.reserve(path_approx.segments().size());
  const double mirrors_center_offset = 0.5 * vehicle_geom.length() -
                                       vehicle_geom.front_edge_to_center() +
                                       mirror.x();
  for (const auto& segment : path_approx.segments()) {
    const Vec2d mirrors_center =
        segment.center() + segment.tangent() * mirrors_center_offset;
    const double mirror_box_length =
        std::max(segment.length() - vehicle_geom.length(), 0.0) +
        mirror.width();
    Box2d path_segment_box(0.5 * mirror_box_length,
                           0.5 * mirror.length() + mirror.y(), mirrors_center,
                           segment.heading(), segment.tangent());
    mirror_segments.emplace_back(segment.first_index(), segment.last_index(),
                                 segment.first_ra(), segment.last_ra(),
                                 segment.first_s(), segment.last_s(),
                                 std::move(path_segment_box));
  }
  return PathApprox(std::move(mirror_segments), path_approx.path_kd_tree());
}

std::vector<PartialSpacetimeObjectTrajectory> GetConsideredStObjects(
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& obj_mgr,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>
        processed_st_objects) {
  std::unordered_map<std::string, PartialSpacetimeObjectTrajectory>
      considered_st_objects_map;
  considered_st_objects_map.reserve(st_boundaries_with_decision.size());
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    const auto decision_type = st_boundary_with_decision.decision_type();
    if (decision_type == StBoundaryProto::IGNORE ||
        decision_type == StBoundaryProto::UNKNOWN ||
        st_boundary_with_decision.st_boundary()->source_type() !=
            StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    const auto& traj_id = st_boundary_with_decision.traj_id();
    CHECK(traj_id.has_value());
    const auto* raw_st_boundary = st_boundary_with_decision.raw_st_boundary();
    if (considered_st_objects_map.find(*traj_id) ==
        considered_st_objects_map.end()) {
      if (processed_st_objects.find(*traj_id) != processed_st_objects.end()) {
        considered_st_objects_map.emplace(
            *traj_id, PartialSpacetimeObjectTrajectory(std::move(
                          FindOrDie(processed_st_objects, *traj_id))));
      } else {
        considered_st_objects_map.emplace(
            *traj_id, PartialSpacetimeObjectTrajectory(*CHECK_NOTNULL(
                          obj_mgr.FindTrajectoryById(*traj_id))));
      }
    }
    CHECK(decision_type == StBoundaryProto::OVERTAKE ||
          decision_type == StBoundaryProto::YIELD ||
          decision_type == StBoundaryProto::FOLLOW);
    FindOrDie(considered_st_objects_map, *traj_id)
        .AppendTimeRangeAndDecisonType(
            raw_st_boundary->min_t(), raw_st_boundary->max_t(),
            decision_type == StBoundaryProto::OVERTAKE
                ? PartialSpacetimeObjectTrajectory::DecisionType::LEAD
                : PartialSpacetimeObjectTrajectory::DecisionType::FOLLOW);
  }
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  considered_st_objects.reserve(considered_st_objects_map.size());
  for (auto& [_, st_obj] : considered_st_objects_map) {
    considered_st_objects.push_back(std::move(st_obj));
  }
  return considered_st_objects;
}

void CutoffSpeedByTimeHorizon(SpeedVector* speed_data) {
  CHECK_NOTNULL(speed_data);
  constexpr double kTimeHorizon = kTrajectoryTimeStep * kTrajectorySteps;
  speed_data->erase(
      std::lower_bound(speed_data->begin(), speed_data->end(), kTimeHorizon,
                       [](const auto& pt, double t) { return pt.t() < t; }),
      speed_data->end());
}

SpeedVector GenerateReferenceSpeed(
    const std::vector<SpeedBoundWithInfo>& min_speed_limit, double init_v,
    double ref_speed_bias, double ref_speed_static_limit_bias, double max_accel,
    double max_decel, double total_time, double delta_t) {
  CHECK_GT(delta_t, 0.0);
  constexpr double kMaxComfortAcc = 1.4;
  const double max_speed_limit = init_v + kMaxComfortAcc * total_time;
  const double init_v_ref = std::max(init_v, 0.0) + ref_speed_bias;
  std::vector<double> reference_speed;
  reference_speed.reserve(min_speed_limit.size());
  reference_speed.push_back(std::min(
      init_v_ref, min_speed_limit[0].bound + ref_speed_static_limit_bias));
  for (int i = 1; i < min_speed_limit.size(); ++i) {
    const double min_static_limit =
        min_speed_limit[i].bound + ref_speed_static_limit_bias;
    const double bound = std::min(min_static_limit, max_speed_limit);
    reference_speed.push_back(
        std::min(reference_speed.back() + max_accel * delta_t, bound));
  }
  for (int i = reference_speed.size() - 2; i >= 0; --i) {
    reference_speed[i] = std::min(reference_speed[i],
                                  reference_speed[i + 1] - max_decel * delta_t);
  }
  SpeedVector speed_points;
  speed_points.reserve(reference_speed.size());
  double s = 0.0;
  double t = 0.0;
  for (int i = 0; i < reference_speed.size() - 1; ++i) {
    speed_points.emplace_back(
        t, s, reference_speed[i],
        (reference_speed[i + 1] - reference_speed[i]) / delta_t, 0.0);
    t += delta_t;
    s += (reference_speed[i + 1] + reference_speed[i]) * 0.5 * delta_t;
  }
  speed_points.emplace_back(t, s, reference_speed.back(), 0.0, 0.0);
  return speed_points;
}

std::vector<std::pair<double, double>> GenerateAccelerationBound(
    const e2e_noa::SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const std::optional<double> acc_target_a,
    const std::optional<double> dec_target_a, double start_a, double delta_t,
    int knot_num) {
  std::vector<std::pair<double, double>> accel_bound;
  accel_bound.clear();
  accel_bound.reserve(knot_num);
  constexpr double kComfortAccJerk = 0.5;
  constexpr double reach_target_time = 2.0;
  const double upper_bound = spacetime_constraint_params.max_acceleration();
  const double lower_bound = spacetime_constraint_params.max_deceleration();
  const double max_decel =
      acc_target_a.has_value() ? acc_target_a.value() : lower_bound;
  double expect_jerk = kComfortAccJerk;
  double modified_target_a =
      std::min(max_decel, start_a + kComfortAccJerk * reach_target_time);
  bool reach_target = false;
  double prev_a = start_a;
  for (int i = 0; i < knot_num; i++) {
    const double t = i * delta_t;
    if (t < reach_target_time) {
      double curr_a = prev_a + expect_jerk * delta_t;
      double modified_lower_bound = std::min(curr_a, modified_target_a);
      prev_a = curr_a;
      if (modified_lower_bound > 0.8 * modified_target_a) {
        reach_target = true;
      }
      accel_bound.emplace_back(std::make_pair(
          upper_bound, std::max(lower_bound, modified_lower_bound)));
      if (reach_target) {
        expect_jerk -= kComfortAccJerk * delta_t;
      }
    } else {
      accel_bound.emplace_back(std::make_pair(upper_bound, lower_bound));
    }
  }
  return accel_bound;
}

bool IsConsiderOncomingObs(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const StBoundary& st_boundary, const PlanPassage plan_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params, double current_v,
    const DiscretizedPath& path) {
  const auto& overlap_info = st_boundary.overlap_infos().front();
  const int overlap_av_idx =
      (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;

  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto obj_pose = traj->planner_object().pose().pos();
  const auto obj_vel = traj->planner_object().pose().v();
  const auto& nearest_station = plan_passage.FindNearestStation(obj_pose);
  const auto lane_id = nearest_station.lane_id();
  const auto& overlap_meta = *st_boundary.overlap_meta();
  if (st_boundary.object_type() == StBoundaryProto::VEHICLE &&
      overlap_meta.pattern() == StOverlapMetaProto::LEAVE) {
    return false;
  }
  const auto& curr_path_point = path.front();
  const auto av_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), vehicle_geometry_params);
  const auto av_frenet_box_or = plan_passage.QueryFrenetBoxAt(av_box);

  const auto obj_frenet_box_or =
      plan_passage.QueryFrenetBoxAtContour(traj->contour(), false);
  if (!obj_frenet_box_or.ok() || !av_frenet_box_or.ok()) {
    return false;
  }
  const auto& obj_frenet_box = *obj_frenet_box_or;
  const auto& av_frenet_box = *av_frenet_box_or;

  constexpr double kBufferlat = 0.35;
  constexpr double kTTC_buffer = 1.5;
  bool is_safte_lon =
      obj_frenet_box.s_min - av_frenet_box.s_max >
      std::min(35.0, std::max(15.0, kTTC_buffer * (obj_vel + current_v)));
  const auto lane_boundary_info =
      plan_passage.QueryEnclosingLaneBoundariesAtS(obj_frenet_box.center_s());
  const double lane_width = lane_boundary_info.left->lat_offset -
                            lane_boundary_info.right->lat_offset;
  std::string debug = "";
  debug =
      "traj_id: " + traj_id + " [ " + std::to_string(is_safte_lon) + " ] " +
      " lat: " +
      std::to_string(std::min(std::fabs(obj_frenet_box.l_max),
                              std::fabs(obj_frenet_box.l_min))) +
      " lon " +
      std::to_string(std::fabs(obj_frenet_box.s_min - av_frenet_box.s_max)) +
      " obj_lon " + std::to_string(std::fabs(obj_frenet_box.s_min)) +
      " av_lon " + std::to_string(std::fabs(av_frenet_box.s_max)) + " lon_th " +
      std::to_string(std::fabs(kTTC_buffer * (obj_vel + current_v))) +
      "lane_width" + std::to_string(lane_width);
  Log2FG::LogDataV0("IsConsiderOncomingObs", debug);
  double lat_threshold = 0.5 * vehicle_geometry_params.width() + kBufferlat;
  constexpr double DEFAULT_LANE_WIDTH = 3.75;
  if (st_boundary.object_type() == StBoundaryProto::VEHICLE) {
    lat_threshold = std::max(
        0.0, vehicle_geometry_params.width() + kBufferlat - lane_width * 0.5);
  } else if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN ||
             st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    lat_threshold = 0.5 * vehicle_geometry_params.width() + kBufferlat;
  }
  if (!is_safte_lon &&
      ((std::min(std::fabs(obj_frenet_box.l_max),
                 std::fabs(obj_frenet_box.l_min)) < lat_threshold) ||
       obj_frenet_box.l_max * obj_frenet_box.l_min < 0.0)) {
    return true;
  }
  return false;
}
}  // namespace planning
}  // namespace e2e_noa
