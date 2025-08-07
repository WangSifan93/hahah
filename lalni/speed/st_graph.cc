#include "speed/st_graph.h"

#include <stddef.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <utility>

#include "aabox3d.pb.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "async/parallel_for.h"
#include "common/path_approx_overlap.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "halfplane.pb.h"
#include "math/frenet_common.h"
#include "math/geometry/box2d.h"
#include "math/geometry/segment2d.h"
#include "math/geometry/util.h"
#include "math/util.h"
#include "object/planner_object.h"
#include "perception.pb.h"
#include "plan/planner_defs.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction_defs.h"
#include "speed/overlap_info.h"
#include "speed/speed_planning_util.h"
#include "speed/st_close_trajectory_generator.h"
#include "speed/st_point.h"
#include "speed/vt_point.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/path_util.h"
#include "util/perception_util.h"
#include "util/vehicle_geometry_util.h"

DEFINE_bool(planner_use_path_approx_based_st_mapping, true,
            "Whether to path-approx-based st boundaries mapping.");
DECLARE_bool(planner_enable_moving_close_traj_speed_limit);

namespace e2e_noa::planning {

namespace {

constexpr double kSearchBuffer = 0.2;
constexpr double kMaxLatDist = 1.5;
constexpr double kEps = 1e-4;

using ::e2e_noa::prediction::PredictedTrajectory;

std::vector<DistanceInfo> CalcDistanceInfoToImpassableBoundaries(
    const DiscretizedPath& path_points,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const VehicleGeometryParamsProto& vehicle_geom,
    const PlannerSemanticMapManager& psman_mgr, double search_radius,
    double min_mirrors_height, bool consider_mirrors_by_default,
    double buffer) {
  std::vector<DistanceInfo> res;
  res.reserve(av_shapes.size());
  constexpr std::array<const char*, 3> kPartName{"MainBody", "LeftMirror",
                                                 "RightMirror"};
  const auto& av_head_boxes = BuildAvHeadBoundingBoxes(vehicle_geom, av_shapes);
  for (size_t i = 0; i < av_shapes.size(); ++i) {
    const auto& path_point = path_points[i];
    const auto& av_shape = av_shapes[i];
    const auto& head_box = av_head_boxes[i];
    const auto segs_info =
        psman_mgr.GetImpassableBoundariesInfo(head_box.center(), search_radius);
    double min_dist = std::numeric_limits<double>::infinity();
    double min_dist_to_left = std::numeric_limits<double>::max();
    double min_dist_to_right = -std::numeric_limits<double>::max();
    int part_idx = -1;
    const std::string* id_ptr = nullptr;
    ad_e2e::planning::RoadBoundaryType boundary_type;
    for (const auto& seg_info : segs_info) {
      const double signed_dist =
          seg_info.segment.SignedDistanceTo(head_box.center());
      double dist =
          std::copysign(head_box.DistanceTo(seg_info.segment), signed_dist);
      if (std::fabs(dist) < std::fabs(min_dist)) {
        min_dist = dist;
        part_idx = 0;
        id_ptr = &seg_info.id;
        boundary_type = seg_info.type;
      }
      if (dist > kEps && dist < min_dist_to_left) {
        min_dist_to_left = dist;
      }
      if (dist < -kEps && dist > min_dist_to_right) {
        min_dist_to_right = dist;
      }
      const bool has_height = seg_info.height.has_value();
      const bool consider_mirrors =
          (consider_mirrors_by_default && !has_height) ||
          (has_height && *seg_info.height > min_mirrors_height);
      if (consider_mirrors) {
        if (const auto dist = av_shape->LeftMirrorDistanceTo(seg_info.segment);
            dist.has_value() && std::fabs(*dist) < std::fabs(min_dist)) {
          min_dist = *dist;
          part_idx = 1;
          id_ptr = &seg_info.id;
          boundary_type = seg_info.type;
        }
        if (const auto dist = av_shape->RightMirrorDistanceTo(seg_info.segment);
            dist.has_value() && std::fabs(*dist) < std::fabs(min_dist)) {
          min_dist = *dist;
          part_idx = 2;
          id_ptr = &seg_info.id;
          boundary_type = seg_info.type;
        }
      }
    }
    if (std::isfinite(min_dist)) {
      res.push_back(
          {.s = path_point.s(),
           .dist = min_dist,
           .id = *id_ptr,
           .info = absl::StrFormat("|%s|s: %.2f|id: {%s}|dist: %.2f",
                                   kPartName[part_idx], path_point.s(), *id_ptr,
                                   min_dist),
           .dists = std::make_pair(min_dist_to_left, min_dist_to_right),
           .type = boundary_type});
    } else {
      res.push_back({.s = path_point.s(),
                     .dist = min_dist,
                     .id = "",
                     .info = "|None.",
                     .type = boundary_type});
    }
    if (std::fabs(min_dist) <= buffer) break;
  }
  return res;
}

absl::Status CheckStBoundary(const StBoundary& st_boundary) {
  const auto& id = st_boundary.id();
  absl::Status status = absl::OkStatus();
  if (st_boundary.lower_points().size() < 2) {
    status = absl::InternalError(absl::StrFormat(
        "st_boundary: %s lower points size is less than 2.", id));
    return status;
  }

  if (st_boundary.max_s() < 0.0) {
    status = absl::InternalError(
        absl::StrFormat("st_boundary: %s max_s() < 0.0.", id));
  }

  if (st_boundary.max_t() < 0.0) {
    status = absl::InternalError(
        absl::StrFormat("st_boundary: %s max_t() < 0.0.", id));
  }

  if (!status.ok()) {
    VLOG(2) << status.ToString();
  }

  return absl::OkStatus();
}

void CheckAndRemoveInvalidStBoundaryPoints(
    std::vector<StBoundaryPoints>* boundaries_points) {
  CHECK_NOTNULL(boundaries_points);
  CHECK(!boundaries_points->empty());
  const auto& points = boundaries_points->back();
  CHECK(!points.lower_points.empty());
  CHECK_EQ(points.lower_points.size(), points.upper_points.size());
  if (points.lower_points.size() < 2) boundaries_points->pop_back();
}

bool IsMappableSpacetimeObject(
    const SpacetimeObjectTrajectory& spacetime_object) {
  if (spacetime_object.states().empty()) {
    VLOG(2) << "Skip mapping spacetime object " << spacetime_object.traj_id()
            << " because its states are empty.";
    return false;
  }
  if (ToStBoundaryObjectType(spacetime_object.planner_object().type()) ==
      StBoundaryProto::IGNORABLE) {
    VLOG(2) << "Skip mapping spacetime object " << spacetime_object.traj_id()
            << " because its object type "
            << ObjectType_Name(spacetime_object.planner_object().type())
            << " is ignorable.";
    return false;
  }
  return true;
}

double ComputeRelativeSpeed(double v_heading, double v, double ref_heading) {
  return v * Vec2d::FastUnitFromAngle(v_heading).dot(
                 Vec2d::FastUnitFromAngle(ref_heading));
}

std::string MakeStBoundaryId(absl::string_view traj_id, int idx) {
  return absl::StrFormat("%s|%d", traj_id, idx);
}

std::string MakeProtectiveStBoundaryId(absl::string_view traj_id) {
  return absl::StrFormat("%s|%c", traj_id, 'p');
}

std::optional<std::pair<double, double>> ConvertToOverlapRange(
    absl::Span<const AgentOverlap> agent_overlaps) {
  if (agent_overlaps.empty()) return std::nullopt;
  std::optional<std::pair<double, double>> overlap_range;
  for (const auto& agent_overlap : agent_overlaps) {
    if (agent_overlap.lat_dist != 0.0) continue;
    if (!overlap_range.has_value()) {
      overlap_range =
          std::make_pair(agent_overlap.first_ra_s, agent_overlap.last_ra_s);
    } else {
      overlap_range->first =
          std::min(overlap_range->first, agent_overlap.first_ra_s);
      overlap_range->second =
          std::max(overlap_range->second, agent_overlap.last_ra_s);
    }
  }
  if (overlap_range.has_value() &&
      overlap_range->first >= overlap_range->second) {
    return std::nullopt;
  }
  return overlap_range;
}

void MergeStBoundaryPoints(
    std::vector<StBoundaryPoints>& st_boundaries_points) {
  if (st_boundaries_points.size() < 1) {
    return;
  }
  if (st_boundaries_points.front().lower_points.front().s() <
      st_boundaries_points.back().upper_points.back().s()) {
    StBoundaryPoints merge_st_boundary_points = st_boundaries_points.front();
    for (int i = 1; i < st_boundaries_points.size(); i++) {
      const auto& stitch_start_obj_state_idx =
          merge_st_boundary_points.overlap_infos.back().obj_idx;
      const auto& stitch_end_obj_state_idx =
          st_boundaries_points[i].overlap_infos.front().obj_idx;

      const double start_time =
          merge_st_boundary_points.speed_points.back().t();
      const double end_time = st_boundaries_points[i].speed_points.front().t();

      const double start_v = merge_st_boundary_points.speed_points.back().v();
      const double end_v = st_boundaries_points[i].speed_points.front().v();

      const double start_low_s =
          merge_st_boundary_points.lower_points.back().s();
      const double end_low_s = st_boundaries_points[i].lower_points.front().s();

      const double start_high_s =
          merge_st_boundary_points.upper_points.back().s();
      const double end_high_s =
          st_boundaries_points[i].upper_points.front().s();

      const double av_start_low_idx =
          merge_st_boundary_points.overlap_infos.back().av_start_idx;
      const double av_end_low_idx =
          st_boundaries_points[i].overlap_infos.front().av_start_idx;

      const double av_start_high_idx =
          merge_st_boundary_points.overlap_infos.back().av_end_idx;
      const double av_end_high_idx =
          st_boundaries_points[i].overlap_infos.front().av_end_idx;

      for (int idx = stitch_start_obj_state_idx; idx < stitch_end_obj_state_idx;
           idx++) {
        double factor =
            LerpFactor(double(stitch_start_obj_state_idx),
                       double(stitch_end_obj_state_idx), double(idx));
        const double time = Lerp(start_time, end_time, factor);
        const double speed = Lerp(start_v, end_v, factor);
        const double low_s = Lerp(start_low_s, end_low_s, factor);
        const double high_s = Lerp(start_high_s, end_high_s, factor);
        const int av_low_idx =
            int(Lerp(av_start_low_idx, av_end_low_idx, factor));
        const int av_high_idx =
            int(Lerp(av_start_high_idx, av_end_high_idx, factor));
        merge_st_boundary_points.speed_points.emplace_back(speed, time);
        merge_st_boundary_points.lower_points.emplace_back(low_s, time);
        merge_st_boundary_points.upper_points.emplace_back(high_s, time);
        merge_st_boundary_points.overlap_infos.push_back(
            OverlapInfo{.time = time,
                        .obj_idx = static_cast<int>(i),
                        .av_start_idx = av_low_idx,
                        .av_end_idx = av_high_idx});
      }
      merge_st_boundary_points.speed_points.insert(
          merge_st_boundary_points.speed_points.end(),
          st_boundaries_points[i].speed_points.begin(),
          st_boundaries_points[i].speed_points.end());
      merge_st_boundary_points.lower_points.insert(
          merge_st_boundary_points.lower_points.end(),
          st_boundaries_points[i].lower_points.begin(),
          st_boundaries_points[i].lower_points.end());
      merge_st_boundary_points.upper_points.insert(
          merge_st_boundary_points.upper_points.end(),
          st_boundaries_points[i].upper_points.begin(),
          st_boundaries_points[i].upper_points.end());
      merge_st_boundary_points.overlap_infos.insert(
          merge_st_boundary_points.overlap_infos.end(),
          st_boundaries_points[i].overlap_infos.begin(),
          st_boundaries_points[i].overlap_infos.end());
    }
    st_boundaries_points.clear();
    st_boundaries_points.emplace_back(merge_st_boundary_points);
  } else {
    st_boundaries_points.erase(st_boundaries_points.begin() + 1,
                               st_boundaries_points.end());
  }
}

using StNearestPoint = StCloseTrajectory::StNearestPoint;
using CloseTrajPoints = std::vector<StNearestPoint>;

std::vector<CloseTrajPoints> GenerateMovingStCloseTrajPoints(
    absl::Span<const StGraph::AgentNearestPoint> agent_nearest_points,
    bool is_cut_in_vehicle, StBoundaryProto::ObjectType object_type,
    const NudgeObjectInfo* nudge_object_info, absl::string_view obj_id) {
  std::vector<CloseTrajPoints> close_trajs_points;
  double prev_lat_dist = std::numeric_limits<double>::max();
  bool prev_state_is_close = false;
  constexpr double kCyclistLatDistThreshold = 0.8;
  constexpr double kNudgeCyclistLatDistThreshold = 0.6;

  double cyclist_dist_lat_threshold = kCyclistLatDistThreshold;
  if (nullptr != nudge_object_info && obj_id == nudge_object_info->id &&
      StBoundaryProto::CYCLIST == object_type) {
    cyclist_dist_lat_threshold = kNudgeCyclistLatDistThreshold;
  }

  for (size_t i = 0; i < agent_nearest_points.size(); ++i) {
    const auto& nearest_point = agent_nearest_points[i];
    const auto ra_heading = nearest_point.ra_heading;
    const auto obj_heading = nearest_point.obj_heading;
    if (is_cut_in_vehicle) {
      constexpr double kParallelHeadingThreshold = M_PI / 9.0;
      const double theta_diff =
          std::fabs(NormalizeAngle(ra_heading - obj_heading));
      constexpr double kSmallAngleCutInLateralBuffer = 0.6;
      if (std::fabs(nearest_point.lat_dist) > kSmallAngleCutInLateralBuffer ||
          theta_diff > kParallelHeadingThreshold) {
        prev_state_is_close = false;
        continue;
      }
    } else if (std::fabs(nearest_point.lat_dist) > kMaxLatDist) {
      prev_state_is_close = false;
      continue;
    }
    if (!prev_state_is_close) {
      close_trajs_points.emplace_back();
      close_trajs_points.back().reserve(agent_nearest_points.size() - i);
    }
    prev_state_is_close = true;

    double lat_v = nearest_point.obj_v *
                   Vec2d::FastUnitFromAngle(obj_heading)
                       .CrossProd(Vec2d::FastUnitFromAngle(ra_heading));

    double lat_t =
        std::fabs(nearest_point.lat_dist) / std::max(std::fabs(lat_v), 1e-6);
    if (StBoundaryProto::CYCLIST == object_type) {
      if (std::fabs(nearest_point.lat_dist) > cyclist_dist_lat_threshold) {
        if (lat_t > 2.0) continue;
      }
    } else {
      if (std::fabs(nearest_point.lat_dist) > 0.5) {
        if (lat_t > 2.0) continue;
      }
    }

    if (prev_lat_dist < std::fabs(nearest_point.lat_dist)) {
      continue;
    }

    prev_lat_dist = std::fabs(nearest_point.lat_dist);
    auto& close_traj_points = close_trajs_points.back();
    close_traj_points.push_back(
        {.s = nearest_point.ra_s,
         .t = nearest_point.t,
         .v = nearest_point.obj_v *
              Vec2d::FastUnitFromAngle(obj_heading)
                  .dot(Vec2d::FastUnitFromAngle(ra_heading)),
         .lat_dist = std::fabs(nearest_point.lat_dist),
         .obj_idx = nearest_point.obj_idx,
         .lat_t = lat_t});
  }
  return close_trajs_points;
}

void GenerateCutInProtectiveStBoundary(
    absl::Span<const CloseTrajPoints> curr_close_trajs,
    std::vector<StBoundaryPoints>* st_boundaries_points, double path_length,
    double plan_start_v, double max_decel) {
  CHECK_NOTNULL(st_boundaries_points);
  const auto& fisrt_overlap_boundary_points = st_boundaries_points->front();
  const double fisrt_overlap_lower_s =
      fisrt_overlap_boundary_points.lower_points.front().s();

  constexpr double kNearDistanceThres = 3.0;
  if (!curr_close_trajs.empty()) {
    size_t cnt = 0;
    auto end_iter = curr_close_trajs.rbegin();
    for (; end_iter != curr_close_trajs.rend(); ++end_iter) {
      if (end_iter->back().s + kNearDistanceThres < fisrt_overlap_lower_s) {
        break;
      }
      cnt += end_iter->size();
    }
    StBoundaryPoints boundary_points;
    boundary_points.protection_type = StBoundaryProto::SMALL_ANGLE_CUT_IN;
    boundary_points.Reserve(cnt);

    const double fisrt_overlap_upper_s =
        fisrt_overlap_boundary_points.upper_points.front().s();
    const double fisrt_overlap_delta_s =
        fisrt_overlap_upper_s - fisrt_overlap_lower_s;
    const double fisrt_overlap_time =
        fisrt_overlap_boundary_points.speed_points.front().t();
    boundary_points.speed_points.push_back(
        fisrt_overlap_boundary_points.speed_points.front());
    boundary_points.lower_points.push_back(
        fisrt_overlap_boundary_points.lower_points.front());
    boundary_points.upper_points.push_back(
        fisrt_overlap_boundary_points.upper_points.front());
    boundary_points.overlap_infos.push_back(
        fisrt_overlap_boundary_points.overlap_infos.front());

    for (auto curr_close_traj_iter = curr_close_trajs.rbegin();
         curr_close_traj_iter != end_iter; ++curr_close_traj_iter) {
      for (auto pt_iter = curr_close_traj_iter->rbegin();
           pt_iter != curr_close_traj_iter->rend(); ++pt_iter) {
        const auto& pt = *pt_iter;
        constexpr double kSpeedBuffer = 2.0;
        constexpr double kMaxExtendedTime = 3.0;
        const double last_s = boundary_points.lower_points.back().s();
        const double last_time = boundary_points.lower_points.back().t();
        const double cur_v = pt.v + kSpeedBuffer;
        if (fisrt_overlap_time - pt.t > kMaxExtendedTime || last_time < pt.t) {
          continue;
        }
        const double lower_s = last_s - (last_time - pt.t) * cur_v;
        constexpr double kMaxLonDist = 30.0;
        constexpr double kMinLowerS = 1.0;
        if (lower_s < kMinLowerS ||
            lower_s < fisrt_overlap_lower_s - kMaxLonDist) {
          break;
        }
        const double max_brake_v =
            std::max(0.0, pt.t * max_decel + plan_start_v);
        const double max_brake_s = (Sqr(max_brake_v) - Sqr(plan_start_v)) /
                                   std::min(2.0 * max_decel, -kEps);
        if (lower_s < max_brake_s) {
          break;
        }
        boundary_points.speed_points.emplace_back(cur_v, pt.t);
        boundary_points.lower_points.emplace_back(lower_s, pt.t);
        boundary_points.upper_points.emplace_back(
            std::min(path_length, lower_s + fisrt_overlap_delta_s), pt.t);
        boundary_points.overlap_infos.push_back(OverlapInfo{
            .time = pt.t,
            .obj_idx = pt.obj_idx,
            .av_start_idx = FloorToInt(pt.s / kPathSampleInterval),
            .av_end_idx = FloorToInt(
                (std::min(path_length, pt.s + fisrt_overlap_delta_s)) /
                kPathSampleInterval)});
      }
    }
    std::reverse(boundary_points.speed_points.begin(),
                 boundary_points.speed_points.end());
    std::reverse(boundary_points.lower_points.begin(),
                 boundary_points.lower_points.end());
    std::reverse(boundary_points.upper_points.begin(),
                 boundary_points.upper_points.end());
    std::reverse(boundary_points.overlap_infos.begin(),
                 boundary_points.overlap_infos.end());
    if (boundary_points.speed_points.size() > 2) {
      st_boundaries_points->emplace(st_boundaries_points->begin(),
                                    std::move(boundary_points));
    }
  }
}

}  // namespace

StGraph::StGraph(
    const DiscretizedPath* path_points, int traj_steps, double plan_start_v,
    double max_decel, const VehicleGeometryParamsProto* vehicle_geo_params,
    const SpeedPlanningParamsProto::StGraphParamsProto* st_graph_params,
    const std::vector<VehicleShapeBasePtr>* av_shape_on_path_points,
    const KdtreeSegmentMatcher* path_kd_tree, const PathApprox* path_approx,
    const PathApprox* path_approx_for_mirrors)
    : total_plan_time_(kTrajectoryTimeStep * traj_steps),
      plan_start_v_(plan_start_v),
      max_decel_(max_decel),
      path_points_(CHECK_NOTNULL(path_points)),
      av_shape_on_path_points_(CHECK_NOTNULL(av_shape_on_path_points)),
      path_kd_tree_(CHECK_NOTNULL(path_kd_tree)),
      vehicle_geo_params_(CHECK_NOTNULL(vehicle_geo_params)),
      st_graph_params_(CHECK_NOTNULL(st_graph_params)),
      path_approx_(path_approx),
      path_approx_for_mirrors_(path_approx_for_mirrors) {
  ego_radius_ = Hypot(std::max(vehicle_geo_params_->front_edge_to_center(),
                               vehicle_geo_params_->back_edge_to_center()),
                      vehicle_geo_params_->right_edge_to_center());
  if (vehicle_geo_params_->has_left_mirror() &&
      vehicle_geo_params_->has_right_mirror()) {
    const auto& mirror = vehicle_geo_params_->left_mirror();
    ego_radius_for_mirrors_ =
        Hypot(mirror.x(), mirror.y()) + 0.5 * mirror.length();
  }
  std::tie(min_mirror_height_avg_, max_mirror_height_avg_) =
      ComputeMinMaxMirrorAverageHeight(*vehicle_geo_params_);
}

void StGraph::CalculateObjectSlPosition(
    const PlanPassage plan_passage,
    const SpacetimeTrajectoryManager& traj_mgr) {
  auto ego_s_offset =
      plan_passage.QueryFrenetLonOffsetAt(ToVec2d((*path_points_).front()));
  auto ego_l_offset =
      plan_passage.QueryFrenetLatOffsetAt(ToVec2d((*path_points_).front()));
  if ((!ego_s_offset.ok()) || (!ego_l_offset.ok())) {
    return;
  }

  for (const auto& obj : traj_mgr.object_trajectories_map()) {
    if (obj.second.empty()) {
      continue;
    }
    const auto& obstacle = obj.second.front();
    const double ego_front_to_center =
        vehicle_geo_params_->front_edge_to_center();
    const double ego_back_to_center =
        vehicle_geo_params_->back_edge_to_center();
    const double ego_half_width = vehicle_geo_params_->width() * 0.5;
    double ds = 0.0;
    double dl = 0.0;

    const Vec2d ego_dir_unit =
        Vec2d::FastUnitFromAngle((*path_points_)[0].theta());
    bool is_back_obj = true;
    for (const auto& corner_point : obstacle->contour().points()) {
      if (ego_dir_unit.Dot(corner_point - ToVec2d((*path_points_).front())) >
          0.0) {
        is_back_obj = false;
        break;
      }
    }

    FrenetPolygon obj_frenet_polygon;
    double av_yaw = (*path_points_).front().theta();
    if (is_back_obj) {
      auto frenet_box =
          plan_passage.QueryFrenetBoxAtContour(obstacle->contour(), false);
      auto frenet_point =
          plan_passage.QueryUnboundedFrenetCoordinateAt(obstacle->pose().pos());
      if ((!frenet_box.ok()) || (!frenet_point.ok())) {
        continue;
      }
      obj_frenet_polygon.center = frenet_point.value();
      obj_frenet_polygon.s_max =
          frenet_box.value().s_max - ego_s_offset.value().accum_s;
      obj_frenet_polygon.s_min =
          frenet_box.value().s_min - ego_s_offset.value().accum_s;
      obj_frenet_polygon.l_max =
          frenet_box.value().l_max - ego_l_offset.value();
      obj_frenet_polygon.l_min =
          frenet_box.value().l_min - ego_l_offset.value();

    } else {
      obj_frenet_polygon =
          path_points_->XYToSL(obstacle->contour(), obstacle->pose().pos());
      av_yaw = path_points_->Evaluate(obj_frenet_polygon.center.s).theta();
    }

    if (obj_frenet_polygon.s_min > ego_front_to_center) {
      ds = obj_frenet_polygon.s_min - ego_front_to_center;
    } else if (obj_frenet_polygon.s_max < (-1.0 * ego_front_to_center)) {
      ds = obj_frenet_polygon.s_max + ego_back_to_center;
    } else {
      ds = 0.0;
    }
    if (obj_frenet_polygon.l_min > ego_half_width) {
      dl = obj_frenet_polygon.l_min - ego_half_width;
    } else if (obj_frenet_polygon.l_max < (-1.0 * ego_half_width)) {
      dl = obj_frenet_polygon.l_max + ego_half_width;
    } else {
      dl = 0.0;
    }

    const auto path_nearest_seg =
        path_kd_tree_->GetNearestSegment(obstacle->bounding_box().center_x(),
                                         obstacle->bounding_box().center_y());
    const auto ob_v_vec = obstacle->planner_object().velocity();
    double vs = ob_v_vec.Dot(path_nearest_seg->unit_direction());
    double vl_sign =
        ob_v_vec.CrossProd(path_nearest_seg->unit_direction()) >= kEps ? 1 : -1;
    double vl =
        ob_v_vec.Dot(path_nearest_seg->unit_direction().Perp()) * vl_sign;

    ObjectSlInfo agent_sl_info{
        .ds = ds,
        .dl = dl,
        .vs = vs,
        .vl = vl,
        .yaw_diff = NormalizeAngle(av_yaw - obstacle->pose().theta()),
        .frenet_polygon = obj_frenet_polygon,
    };
    obj_sl_map_[std::string(obstacle->object_id())] = agent_sl_info;
  }
}

StGraph::StBoundaryOutput StGraph::GetStBoundaries(
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    bool consider_lane_change_gap, const ConstraintManager& constraint_mgr,
    const PlannerSemanticMapManager* psman_mgr, const PlanPassage* plan_passage,
    const PathSlBoundary* path_sl_boundary,
    const NudgeObjectInfo* nudge_object_info,
    WorkerThreadManager* thread_pool) {
  StBoundaryOutput boundary_output;
  std::vector<StBoundaryRef>& st_boundaries = boundary_output.st_boundaries;
  std::vector<SltInfoRef>& slt_infos = boundary_output.slt_infos;

  if (path_points_->size() < 2) {
    LOG(WARNING) << "Fail to get st_boundary because of too few path points.";
    return boundary_output;
  }

  CalculateObjectSlPosition(*plan_passage, traj_mgr);

  const auto& traffic_gap = constraint_mgr.TrafficGap();
  const auto& moving_spacetime_objects = traj_mgr.moving_object_trajs();
  const int moving_obj_num = moving_spacetime_objects.size();
  std::vector<StBoundaryOutput> st_boundaries_for_workers(moving_obj_num);
  ParallelFor(
      0, moving_obj_num, thread_pool,
      [&](parallel_for::WorkerIndex worker_index, int i) {
        const SpacetimeObjectTrajectory* st_traj = moving_spacetime_objects[i];
        const std::string traj_id(st_traj->traj_id());
        const std::string object_id(st_traj->object_id());
        VLOG(2) << "[speed finder] id: " << object_id << " traj_id: " << traj_id
                << " intention:" << st_traj->trajectory().intention();
        const auto leader_id = traffic_gap.leader_id.has_value()
                                   ? traffic_gap.leader_id.value()
                                   : "-1";
        const auto follower_id = traffic_gap.follower_id.has_value()
                                     ? traffic_gap.follower_id.value()
                                     : "-1";
        DLOG(INFO) << "map st, object_id: " << object_id
                   << ", traffic_gap: " << leader_id << ", " << follower_id
                   << ", gap: " << consider_lane_change_gap;
        const bool generate_lane_change_gap =
            consider_lane_change_gap &&
            ((traffic_gap.leader_id.has_value() &&
              *traffic_gap.leader_id == object_id) ||
             (traffic_gap.follower_id.has_value() &&
              *traffic_gap.follower_id == object_id));
        if (generate_lane_change_gap) {
          Log2FG::LogDataV2("gap_debug", "object_id: " + object_id +
                                             " leader id: " + leader_id +
                                             " follower id: " + follower_id);
        }

        if (IsMappableSpacetimeObject(*st_traj) &&
            leading_objs.find(traj_id) == leading_objs.end()) {
          auto moving_obj_output = MapMovingSpacetimeObject(
              *st_traj, generate_lane_change_gap,
              FLAGS_planner_enable_moving_close_traj_speed_limit,
              nudge_object_info);
          for (int j = 0; j < moving_obj_output.st_boundaries.size(); ++j) {
            if (CheckStBoundary(*moving_obj_output.st_boundaries[j]).ok()) {
              st_boundaries_for_workers[i].st_boundaries.push_back(
                  std::move(moving_obj_output.st_boundaries[j]));
            }
          }
          for (int j = 0; j < moving_obj_output.slt_infos.size(); ++j) {
            if (moving_obj_output.slt_infos[j] &&
                moving_obj_output.slt_infos[j]->IsVaild()) {
              st_boundaries_for_workers[i].slt_infos.push_back(
                  std::move(moving_obj_output.slt_infos[j]));
            }
          }
        }
      });

  for (auto& single_worker_st_boundaries : st_boundaries_for_workers) {
    if (!single_worker_st_boundaries.st_boundaries.empty()) {
      st_boundaries.insert(
          st_boundaries.end(),
          std::make_move_iterator(
              single_worker_st_boundaries.st_boundaries.begin()),
          std::make_move_iterator(
              single_worker_st_boundaries.st_boundaries.end()));
    }
    if (!single_worker_st_boundaries.slt_infos.empty()) {
      slt_infos.insert(
          slt_infos.end(),
          std::make_move_iterator(
              single_worker_st_boundaries.slt_infos.begin()),
          std::make_move_iterator(single_worker_st_boundaries.slt_infos.end()));
    }
  }

  auto stationary_st_boundaries = MapStationarySpacetimeObjects(
      traj_mgr.stationary_object_trajs(), leading_objs);
  for (size_t i = 0; i < stationary_st_boundaries.size(); ++i) {
    if (CheckStBoundary(*stationary_st_boundaries[i]).ok()) {
      VLOG(3) << "Map stationary objects on st_graph, id: "
              << stationary_st_boundaries[i]->id();
      st_boundaries.push_back(std::move(stationary_st_boundaries[i]));
    }
  }

  const int num_stop_lines = constraint_mgr.StopLine().size();
  std::vector<StBoundaryRef> st_boundaries_from_stop_lines(num_stop_lines);
  ParallelFor(0, num_stop_lines, thread_pool, [&](int i) {
    const auto& stop_line = constraint_mgr.StopLine()[i];
    if (StBoundaryRef st_boundary = MapStopLine(stop_line)) {
      if (CheckStBoundary(*st_boundary).ok()) {
        st_boundaries_from_stop_lines[i] = std::move(st_boundary);
        DLOG(INFO) << "Map stop line on st_graph, id: " << stop_line.id();
      }
    }
  });
  for (auto& st_boundary : st_boundaries_from_stop_lines) {
    if (st_boundary != nullptr) {
      st_boundaries.push_back(std::move(st_boundary));
    }
  }

  const int num_path_stop_lines = constraint_mgr.PathStopLine().size();
  std::vector<StBoundaryRef> st_boundaries_from_path_stop_lines;
  st_boundaries_from_path_stop_lines.reserve(num_path_stop_lines);
  for (const auto& path_stop_line : constraint_mgr.PathStopLine()) {
    if (StBoundaryRef st_boundary = MapPathStopLine(path_stop_line)) {
      if (CheckStBoundary(*st_boundary).ok()) {
        st_boundaries_from_path_stop_lines.emplace_back(std::move(st_boundary));
        VLOG(3) << "Map path stop line on st_graph, id: "
                << path_stop_line.id();
      }
    }
  }
  for (auto& st_boundary : st_boundaries_from_path_stop_lines) {
    st_boundaries.push_back(std::move(st_boundary));
  }

  if (psman_mgr != nullptr) {
    if (auto nearest_impassable_st_boundary =
            MapNearestImpassableBoundary(*psman_mgr)) {
      if (CheckStBoundary(*nearest_impassable_st_boundary).ok()) {
        VLOG(3) << "Map impassable boundary on st_graph, id: "
                << nearest_impassable_st_boundary->id();
        st_boundaries.push_back(std::move(nearest_impassable_st_boundary));
      }
    }
  }

  if (plan_passage != nullptr && path_sl_boundary != nullptr) {
    if (auto nearest_path_boundary_st_boundary =
            MapNearestPathBoundary(*plan_passage, *path_sl_boundary)) {
      if (CheckStBoundary(*nearest_path_boundary_st_boundary).ok()) {
        VLOG(3) << "Map neareast path boundary on st_graph.";
        st_boundaries.push_back(std::move(nearest_path_boundary_st_boundary));
      }
    }
  }

  if (plan_passage != nullptr && path_sl_boundary != nullptr &&
      !leading_objs.empty()) {
    const bool lc_acc_gap = traffic_gap.acc_gap_target_speed.has_value() &&
                            traffic_gap.acc_gap_target_a.has_value();
    for (auto& leading_obj_st_boundary :
         MapLeadingSpacetimeObjects(traj_mgr, leading_objs, lc_acc_gap,
                                    *plan_passage, *path_sl_boundary)) {
      if (CheckStBoundary(*leading_obj_st_boundary).ok()) {
        st_boundaries.push_back(std::move(leading_obj_st_boundary));
      }
    }
  }

  for (auto& st_boundary : st_boundaries) {
    if (st_boundary->object_type() != StBoundaryProto::VIRTUAL &&
        st_boundary->object_type() != StBoundaryProto::IMPASSABLE_BOUNDARY &&
        st_boundary->object_type() != StBoundaryProto::PATH_BOUNDARY) {
      if (st_boundary == nullptr) {
        continue;
      }
      if (!st_boundary->object_id().has_value()) {
        continue;
      }
      const auto obj_sl = obj_sl_map_.find(st_boundary->object_id().value());
      if (obj_sl != obj_sl_map_.end()) {
        st_boundary->set_obj_sl_info(obj_sl->second);
      }
    }
  }
  return boundary_output;
}

StBoundaryRef StGraph::MapNearestImpassableBoundary(
    const PlannerSemanticMapManager& psman_mgr) {
  const double search_radius_for_impassable_boundaries =
      std::max(ego_radius_, ego_radius_for_mirrors_) +
      st_graph_params_->impassable_boundaries_search_radius_buffer();
  constexpr double kExtendBuffer = 0.1;
  distance_info_to_impassable_boundaries_ =
      CalcDistanceInfoToImpassableBoundaries(
          *path_points_, *av_shape_on_path_points_, *vehicle_geo_params_,
          psman_mgr, search_radius_for_impassable_boundaries,
          min_mirror_height_avg_,
          st_graph_params_->consider_mirrors_by_default(), kExtendBuffer);

  const auto& distance_info = distance_info_to_impassable_boundaries_.back();
  if (std::fabs(distance_info.dist) > kExtendBuffer) return nullptr;

  const double s_min = std::max(0.0, distance_info.s);
  const double s_max = path_points_->back().s();
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                     StPoint(s_min, total_plan_time_)};
  st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                     StPoint(s_max, total_plan_time_)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                     VtPoint(0.0, total_plan_time_)};

  return CHECK_NOTNULL(StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::IMPASSABLE_BOUNDARY,
      distance_info.id, 1.0, true, StBoundaryProto::NON_PROTECTIVE, false,
      false));
}

StBoundaryRef StGraph::MapNearestPathBoundary(
    const PlanPassage& plan_passage,
    const PathSlBoundary& path_sl_boundary) const {
  constexpr double kPathBoundaryViolationLimit = 1.2;
  std::optional<double> low_s = std::nullopt;
  const double offset =
      0.5 * vehicle_geo_params_->width() - kPathBoundaryViolationLimit;
  for (const auto& path_point : *path_points_) {
    const auto av_sl = plan_passage.QueryFrenetCoordinateAt(
        Vec2d(path_point.x(), path_point.y()));
    if (!av_sl.ok()) {
      low_s = path_point.s();
      break;
    }
    const auto boundary_l_pair = path_sl_boundary.QueryBoundaryL(av_sl->s);
    if (boundary_l_pair.second - av_sl->l - offset < 0.0 ||
        boundary_l_pair.first - av_sl->l + offset > 0.0) {
      low_s = path_point.s();
      break;
    }
  }
  if (low_s.has_value()) {
    const double s_min = std::max(0.0, *low_s);
    const double s_max = path_points_->back().s();
    StBoundaryPoints st_boundary_points;
    st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                       StPoint(s_min, total_plan_time_)};
    st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                       StPoint(s_max, total_plan_time_)};
    st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                       VtPoint(0.0, total_plan_time_)};

    return CHECK_NOTNULL(StBoundary::CreateInstance(
        st_boundary_points, StBoundaryProto::PATH_BOUNDARY, "PATH_BOUNDARY",
        1.0, true, StBoundaryProto::NON_PROTECTIVE, false, false));
  } else {
    return nullptr;
  }
}

std::optional<std::pair<int, int>> StGraph::FindSegment2dOverlapRange(
    const Segment2d& segment) const {
  std::pair<int, int> overlap_range_idx;
  const auto center = segment.center();
  const double radius = segment.length() * 0.5 + ego_radius_ + kSearchBuffer;
  const double driving_direction =
      (-segment.unit_direction().Perp()).FastAngle();

  if (FLAGS_planner_use_path_approx_based_st_mapping &&
      path_approx_ != nullptr) {
    const double path_step_length =
        (*path_points_)[1].s() - (*path_points_)[0].s();

    double first_s = std::numeric_limits<double>::infinity(), last_s = 0.0;

    constexpr double kSmallBoxWidth = 0.01;
    const auto agent_overlaps = ComputeAgentOverlapsWithBufferAndHeading(
        *path_approx_, path_step_length, 0, path_points_->size() - 1,
        Polygon2d(Box2d(segment, kSmallBoxWidth)), kSearchBuffer, 0.0, 0.0,
        radius, driving_direction, M_PI_2);
    const auto overlap_range = ConvertToOverlapRange(agent_overlaps);
    if (overlap_range.has_value()) {
      std::tie(first_s, last_s) = *overlap_range;
    }
    if (first_s > last_s) {
      return std::nullopt;
    }
    overlap_range_idx.first =
        std::max(0, CeilToInteger(first_s / path_step_length - kEps));
    overlap_range_idx.second =
        std::min(static_cast<int>(path_points_->size() - 1),
                 FloorToInt(last_s / path_step_length + kEps));
    return overlap_range_idx;
  }

  auto indices = path_kd_tree_->GetSegmentIndexInRadiusWithHeading(
      center.x(), center.y(), driving_direction, radius, M_PI_2);
  if (indices.empty()) return std::nullopt;
  std::stable_sort(indices.begin(), indices.end());

  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& av_shape = (*av_shape_on_path_points_)[*it];
    if (av_shape->MainBodyHasOverlapWithBuffer(segment, 0.0, 0.0)) {
      overlap_range_idx.first = *it;
      updated = true;
      break;
    }
  }
  if (!updated) return std::nullopt;
  for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
    const auto& av_shape = (*av_shape_on_path_points_)[*it];
    if (av_shape->MainBodyHasOverlapWithBuffer(segment, 0.0, 0.0)) {
      overlap_range_idx.second = *it;
      break;
    }
  }
  return overlap_range_idx;
}

std::optional<std::pair<int, int>> StGraph::FindStopLine2dOverlapRange(
    const Segment2d& segment, bool is_extended) const {
  std::pair<int, int> overlap_range_idx;
  const auto center = segment.center();
  const double radius = segment.length() * 0.5 + ego_radius_ + kSearchBuffer;
  const double driving_direction =
      (-segment.unit_direction().Perp()).FastAngle();

  if (FLAGS_planner_use_path_approx_based_st_mapping &&
      path_approx_ != nullptr) {
    const double path_step_length =
        (*path_points_)[1].s() - (*path_points_)[0].s();

    double first_s = std::numeric_limits<double>::infinity(), last_s = 0.0;

    constexpr double kSmallBoxWidth = 0.01;

    std::pair<int, int> minmax_index = {0, 0};
    auto agent_overlaps = ComputeAgentOverlapsWithBufferAndHeading(
        *path_approx_, path_step_length, 0, path_points_->size() - 1,
        Polygon2d(Box2d(segment, kSmallBoxWidth)), kSearchBuffer, 0.0, 0.0,
        radius, driving_direction, M_PI_2, minmax_index);
    DLOG(INFO) << "agent_overlaps:" << agent_overlaps.size();

    if (agent_overlaps.empty() && !(minmax_index.first == minmax_index.second &&
                                    minmax_index.first == 0)) {
      DLOG(INFO) << "get minmax index " << minmax_index.first << ", "
                 << minmax_index.second;
      int first_index = 0, last_index = path_points_->size() - 1;
      std::tie(first_index, last_index) = minmax_index;

      const auto& sl_coord = path_points_->XYToSL(segment.center());
      DLOG(INFO) << " ------- checkout sl" << sl_coord.s << ", " << sl_coord.l
                 << ", xy " << segment.center().x() << ", "
                 << segment.center().y();
      if (!is_extended) {
        if (sl_coord.s < 0 && sl_coord.s > -5.0) {
          DLOG(INFO) << " ------- to mork agent overlap" << sl_coord.s << ", "
                     << sl_coord.l;
          AgentOverlap agent_overlap;
          agent_overlap.first_ra_s = 0.0;
          agent_overlap.last_ra_s = 10.0;
          agent_overlaps.emplace_back(agent_overlap);
        }
      } else {
        if (sl_coord.s < 0 && sl_coord.s > -5.0 && sl_coord.l < 1.0 &&
            sl_coord.l > -1.0) {
          DLOG(INFO) << " ------- to mork agent overlap -- extended "
                     << sl_coord.s << ", " << sl_coord.l;
          AgentOverlap agent_overlap;
          agent_overlap.first_ra_s = 0.0;
          agent_overlap.last_ra_s = 10.0;
          agent_overlaps.emplace_back(agent_overlap);
        }
      }
    }
    const auto overlap_range = ConvertToOverlapRange(agent_overlaps);
    if (overlap_range.has_value()) {
      std::tie(first_s, last_s) = *overlap_range;
    }
    if (first_s > last_s) {
      return std::nullopt;
    }
    overlap_range_idx.first =
        std::max(0, CeilToInteger(first_s / path_step_length - kEps));
    overlap_range_idx.second =
        std::min(static_cast<int>(path_points_->size() - 1),
                 FloorToInt(last_s / path_step_length + kEps));
    return overlap_range_idx;
  }

  auto indices = path_kd_tree_->GetSegmentIndexInRadiusWithHeading(
      center.x(), center.y(), driving_direction, radius, M_PI_2);
  if (indices.empty()) return std::nullopt;
  std::stable_sort(indices.begin(), indices.end());

  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& av_shape = (*av_shape_on_path_points_)[*it];
    if (av_shape->MainBodyHasOverlapWithBuffer(segment, 0.0, 0.0)) {
      overlap_range_idx.first = *it;
      updated = true;
      break;
    }
  }
  if (!updated) return std::nullopt;
  for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
    const auto& av_shape = (*av_shape_on_path_points_)[*it];
    if (av_shape->MainBodyHasOverlapWithBuffer(segment, 0.0, 0.0)) {
      overlap_range_idx.second = *it;
      break;
    }
  }
  return overlap_range_idx;
}

std::optional<std::pair<int, int>> StGraph::FindStopLineOverlapRange(
    const ConstraintProto::StopLineProto& stop_line) const {
  const Segment2d stop_line_seg(Vec2dFromProto(stop_line.half_plane().start()),
                                Vec2dFromProto(stop_line.half_plane().end()));
  DLOG(INFO) << "FindStopLineSeg: id(" << stop_line.id() << ") "
             << stop_line_seg.start().x() << ", " << stop_line_seg.start().y()
             << " to " << stop_line_seg.end().x() << ","
             << stop_line_seg.end().y();
  return FindStopLine2dOverlapRange(stop_line_seg, stop_line.is_extended());
}

StBoundaryRef StGraph::MapStopLine(
    const ConstraintProto::StopLineProto& stop_line) const {
  const auto overlap_range = FindStopLineOverlapRange(stop_line);
  if (!overlap_range.has_value()) {
    DLOG(INFO) << "Stopline St Bound cannot find overlap range by start "
               << stop_line.half_plane().start().x() << ", "
               << stop_line.half_plane().start().y();
    return nullptr;
  }
  const double s_min = std::max(0.0, (*path_points_)[overlap_range->first].s());
  const double s_max = std::max(s_min, path_points_->back().s());
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                     StPoint(s_min, total_plan_time_)};
  st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                     StPoint(s_max, total_plan_time_)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                     VtPoint(0.0, total_plan_time_)};
  DLOG(INFO) << "building stopline stb: " << stop_line.id() << " s min/max("
             << s_min << ", " << s_max << ")";
  return CHECK_NOTNULL(StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VIRTUAL, stop_line.id(), 1.0, true,
      StBoundaryProto::NON_PROTECTIVE, false, stop_line.is_traffic_light()));
}

StBoundaryRef StGraph::MapPathStopLine(
    const ConstraintProto::PathStopLineProto& path_stop_line) const {
  double s_min = 0.0;
  double s_max = 0.0;

  CHECK_GE(path_stop_line.s(), 0.0);
  CHECK_LE(path_stop_line.s(), path_points_->back().s());
  s_min = path_stop_line.s();
  s_max = path_points_->back().s();

  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                     StPoint(s_min, total_plan_time_)};
  st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                     StPoint(s_max, total_plan_time_)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                     VtPoint(0.0, total_plan_time_)};

  return CHECK_NOTNULL(StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VIRTUAL, path_stop_line.id(), 1.0,
      true, StBoundaryProto::NON_PROTECTIVE, false, false));
}

StGraph::StBoundaryOutput StGraph::MapMovingSpacetimeObject(
    const SpacetimeObjectTrajectory& spacetime_object,
    bool generate_lane_change_gap, bool calc_moving_close_traj,
    const NudgeObjectInfo* nudge_object_info) const {
  CHECK(!spacetime_object.is_stationary());
  StBoundaryOutput st_boundary_output;
  std::vector<StBoundaryRef>& st_boundaries = st_boundary_output.st_boundaries;
  std::vector<SltInfoRef>& slt_infos = st_boundary_output.slt_infos;
  std::vector<NearestSlPoint> sl_points;
  const auto st_boundaries_points = GetMovingObjStBoundaryPoints(
      spacetime_object, generate_lane_change_gap, calc_moving_close_traj,
      &sl_points, nudge_object_info);
  if (st_boundaries_points.empty()) {
    if (!sl_points.empty()) {
      slt_infos.push_back(SltInfoRef(new SltInfo(
          std::move(sl_points), std::string(spacetime_object.traj_id()))));
    }
    return st_boundary_output;
  }

  const size_t size = st_boundaries_points.size();
  st_boundaries.reserve(size);
  int protective_boundary_cnt = 0;
  for (size_t i = 0; i < size; ++i) {
    std::string st_boundary_id;
    if (st_boundaries_points[i].protection_type !=
        StBoundaryProto::NON_PROTECTIVE) {
      st_boundary_id = MakeProtectiveStBoundaryId(spacetime_object.traj_id());
      ++protective_boundary_cnt;
    } else {
      st_boundary_id = (size - protective_boundary_cnt) == 1
                           ? std::string(spacetime_object.traj_id())
                           : MakeStBoundaryId(spacetime_object.traj_id(), i);
    }
    CHECK_EQ(st_boundaries_points[i].lower_points.size(),
             st_boundaries_points[i].upper_points.size());
    if (st_boundaries_points[i].lower_points.size() < 2) continue;
    auto st_boundary = StBoundary::CreateInstance(
        st_boundaries_points[i],
        ToStBoundaryObjectType(spacetime_object.planner_object().type()),
        std::move(st_boundary_id), spacetime_object.trajectory().probability(),
        false, st_boundaries_points[i].protection_type,
        spacetime_object.planner_object().is_large_vehicle(), false,
        spacetime_object.planner_object().pose());
    st_boundaries.push_back(std::move(st_boundary));
  }

  CHECK_LT(protective_boundary_cnt, 2);
  if (st_boundaries.size() >= 2 && st_boundaries[0]->is_protective()) {
    st_boundaries[0]->set_protected_st_boundary_id(st_boundaries[1]->id());
  }
  if (calc_moving_close_traj) {
    absl::MutexLock lock(&mutex_);
    std::sort(moving_close_trajs_.begin(), moving_close_trajs_.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.min_t() < rhs.min_t();
              });
  }
  for (auto& st_boundary : st_boundaries) {
    if (st_boundary == nullptr) {
      continue;
    }
    if (!st_boundary->object_id().has_value()) {
      continue;
    }
    const auto obj_sl = obj_sl_map_.find(st_boundary->object_id().value());
    if (obj_sl != obj_sl_map_.end()) {
      st_boundary->set_obj_sl_info(obj_sl->second);
    }
  }
  return st_boundary_output;
}

std::vector<StBoundaryRef> StGraph::MapStationarySpacetimeObjects(
    absl::Span<const SpacetimeObjectTrajectory* const>
        stationary_spacetime_objs,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs) const {
  constexpr double kLargeKappaCheckingDis = 5.0;
  std::vector<StBoundaryRef> st_boundaries;
  st_boundaries.reserve(stationary_spacetime_objs.size());

  double max_kappa = 0.0;
  for (const auto& pt : *path_points_) {
    if (std::fabs(pt.kappa()) > std::fabs(max_kappa)) {
      max_kappa = pt.kappa();
    }
  }

  for (size_t i = 0; i < stationary_spacetime_objs.size(); ++i) {
    const SpacetimeObjectTrajectory* obj = stationary_spacetime_objs[i];
    const std::string traj_id(obj->traj_id());
    CHECK(obj->is_stationary());
    if (!IsMappableSpacetimeObject(*obj)) continue;
    if (leading_objs.find(traj_id) != leading_objs.end()) continue;
    auto st_boundary_points_or =
        GetStationaryObjStBoundaryPoints(*obj, max_kappa);
    if (!st_boundary_points_or) {
      VLOG(3) << "GetStationaryObjStBoundaryPoints fails. Id: " << traj_id;
      continue;
    }
    st_boundaries.push_back(StBoundary::CreateInstance(
        *st_boundary_points_or, ToStBoundaryObjectType(obj->object_type()),
        traj_id, obj->trajectory().probability(), true,
        StBoundaryProto::NON_PROTECTIVE,
        obj->planner_object().is_large_vehicle(), false,
        obj->planner_object().pose()));
  }

  return st_boundaries;
}

std::vector<StBoundaryRef> StGraph::MapLeadingSpacetimeObjects(
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const bool lc_acc_gap, const PlanPassage& plan_passage,
    const PathSlBoundary& path_sl_boundary) const {
  std::vector<StBoundaryRef> st_boundaries;
  st_boundaries.reserve(leading_objs.size());
  for (const auto& [traj_id, _] : leading_objs) {
    const SpacetimeObjectTrajectory* st_traj =
        traj_mgr.FindTrajectoryById(traj_id);

    if (st_traj == nullptr) continue;
    if (!IsMappableSpacetimeObject(*st_traj)) continue;
    if (st_traj->is_stationary()) {
      const auto lower_index = FindLeadingObjectLowerIndex(
          st_traj->contour(), plan_passage, path_sl_boundary);
      if (!lower_index.has_value()) continue;
      const double low_s = (*path_points_)[*lower_index].s();
      const double high_s = path_points_->back().s();
      StBoundaryPoints st_boundary_points;
      st_boundary_points.lower_points = {{low_s, 0.0},
                                         {low_s, total_plan_time_}};
      st_boundary_points.upper_points = {{high_s, 0.0},
                                         {high_s, total_plan_time_}};
      st_boundary_points.speed_points = {{0.0, 0.0}, {0.0, total_plan_time_}};
      st_boundary_points.overlap_infos = {OverlapInfo{
          .time = 0.0,
          .obj_idx = 0,
          .av_start_idx = *lower_index,
          .av_end_idx = static_cast<int>(path_points_->size() - 1)}};
      st_boundaries.push_back(StBoundary::CreateInstance(
          st_boundary_points,
          ToStBoundaryObjectType(st_traj->planner_object().type()), traj_id,
          st_traj->trajectory().probability(), true,
          StBoundaryProto::NON_PROTECTIVE,
          st_traj->planner_object().is_large_vehicle(), false,
          st_traj->planner_object().pose()));
    } else {
      const auto& object_states = st_traj->states();
      CHECK_GT(object_states.size(), 0);
      StBoundaryPoints st_boundary_points;
      st_boundary_points.Reserve(object_states.size());
      DLOG(INFO) << "Build leading st: " << traj_id << " size "
                 << object_states.size();
      double t_max = object_states.back().traj_point->t();
      if (lc_acc_gap) {
        t_max = GetStBoundaryTRangeByProtectionType(
                    *st_traj, StBoundaryProto::LANE_CHANGE_GAP)
                    .second;
      }
      for (size_t i = 0; i < object_states.size(); ++i) {
        const auto& curr_state = object_states[i];
        const auto* traj_point = curr_state.traj_point;
        const auto lower_index = FindLeadingObjectLowerIndex(
            curr_state.contour, plan_passage, path_sl_boundary);
        if (!lower_index.has_value()) break;
        const double speed =
            ComputeRelativeSpeed(traj_point->theta(), traj_point->v(),
                                 (*path_points_)[*lower_index].theta());
        const double t = traj_point->t();
        if (t > t_max) break;
        st_boundary_points.speed_points.emplace_back(speed, t);
        st_boundary_points.lower_points.emplace_back(
            (*path_points_)[*lower_index].s(), t);
        st_boundary_points.upper_points.emplace_back(path_points_->back().s(),
                                                     t);
        st_boundary_points.overlap_infos.push_back(OverlapInfo{
            .time = t,
            .obj_idx = static_cast<int>(i),
            .av_start_idx = *lower_index,
            .av_end_idx = static_cast<int>(path_points_->size() - 1)});
      }

      if (st_boundary_points.lower_points.size() >= 2) {
        st_boundaries.push_back(StBoundary::CreateInstance(
            st_boundary_points,
            ToStBoundaryObjectType(st_traj->planner_object().type()), traj_id,
            st_traj->trajectory().probability(), false,
            StBoundaryProto::NON_PROTECTIVE,
            st_traj->planner_object().is_large_vehicle(), false,
            st_traj->planner_object().pose()));
      }
    }
  }
  return st_boundaries;
}

std::optional<StBoundaryPoints> StGraph::GetStationaryObjStBoundaryPoints(
    const SpacetimeObjectTrajectory& spacetime_object,
    const double max_kappa) const {
  constexpr double kLowSpeedThres = 2.0;
  constexpr double kKappaThres = 0.05;

  CHECK(spacetime_object.is_stationary());
  StBoundaryPoints st_boundary_points;

  const Box2d& obj_box = spacetime_object.bounding_box();
  const double obj_radius =
      obj_box.diagonal() * 0.5 + spacetime_object.required_lateral_gap();
  const double search_radius = obj_radius + ego_radius_ + kSearchBuffer;
  const double search_radius_for_mirrors =
      obj_radius + ego_radius_for_mirrors_ + kSearchBuffer;

  double lateral_gap_extend = 0.0;
  if (plan_start_v_ < 2.0 && std::fabs(max_kappa) > kKappaThres &&
      ToStBoundaryObjectType(spacetime_object.object_type()) ==
          StBoundaryProto::VEHICLE) {
    const auto& sl_info =
        obj_sl_map_.find(std::string(spacetime_object.object_id()));
    if (sl_info != obj_sl_map_.end()) {
      bool maybe_nudge = (max_kappa > 0.0 && sl_info->second.dl < 0.0) ||
                         (max_kappa < 0.0 && sl_info->second.dl > 0.0);
      if (sl_info->second.ds < 10.0 && sl_info->second.ds > 0.0 &&
          maybe_nudge) {
        lateral_gap_extend = 0.4;
      }
    }
  }

  int low_idx = 0;
  int high_idx = 0;
  const bool consider_mirrors =
      IsConsiderMirrorObject(spacetime_object.planner_object().object_proto(),
                             min_mirror_height_avg_, max_mirror_height_avg_);

  if (!FindOverlapRangeOrNearestPoint(
          obj_box.center(), search_radius, search_radius_for_mirrors,
          spacetime_object.states()[0], 0, consider_mirrors,
          spacetime_object.required_lateral_gap() + lateral_gap_extend,
          spacetime_object.required_lateral_gap(), &low_idx, &high_idx,
          nullptr)) {
    return std::nullopt;
  }
  if (low_idx == 0) {
    if (!FindOverlapRangeOrNearestPoint(
            obj_box.center(), search_radius, search_radius_for_mirrors,
            spacetime_object.states()[0], 0, consider_mirrors, 0.0, 0.0,
            &low_idx, &high_idx, nullptr)) {
      return std::nullopt;
    }
  }
  const double low_s = (*path_points_)[low_idx].s();

  const double high_s = path_points_->back().s();
  st_boundary_points.lower_points = {{low_s, 0.0}, {low_s, total_plan_time_}};
  st_boundary_points.upper_points = {{high_s, 0.0}, {high_s, total_plan_time_}};
  st_boundary_points.speed_points = {{0.0, 0.0}, {0.0, total_plan_time_}};
  st_boundary_points.overlap_infos = {OverlapInfo{.time = 0.0,
                                                  .obj_idx = 0,
                                                  .av_start_idx = low_idx,
                                                  .av_end_idx = high_idx}};
  return st_boundary_points;
}

std::vector<StBoundaryPoints> StGraph::GetMovingObjStBoundaryPoints(
    const SpacetimeObjectTrajectory& spacetime_object,
    bool generate_lane_change_gap, bool calc_moving_close_traj,
    std::vector<NearestSlPoint>* const sl_points,
    const NudgeObjectInfo* nudge_object_info) const {
  CHECK(!spacetime_object.is_stationary());
  std::vector<StBoundaryPoints> st_boundaries_points;
  const Box2d& obj_box = spacetime_object.bounding_box();
  const double obj_radius =
      obj_box.diagonal() * 0.5 + spacetime_object.required_lateral_gap();
  const double search_radius = obj_radius + ego_radius_ + kSearchBuffer;
  const double search_radius_for_mirrors =
      obj_radius + ego_radius_for_mirrors_ + kSearchBuffer;
  const Vec2d ego_dir_unit =
      Vec2d::FastUnitFromAngle((*path_points_)[0].theta());

  const auto& object_states = spacetime_object.states();
  CHECK_GT(object_states.size(), 0);

  bool prev_state_has_overlap = false;
  constexpr double kNegtiveTimeThreshold = -1e-6;
  int low_idx = 0;
  int high_idx = 0;
  const bool consider_mirrors =
      IsConsiderMirrorObject(spacetime_object.planner_object().object_proto(),
                             min_mirror_height_avg_, max_mirror_height_avg_);
  const auto& path_points = *path_points_;
  std::vector<AgentNearestPoint> agent_nearest_points;
  for (size_t i = 0; i < object_states.size(); ++i) {
    const auto& cur_state = object_states[i];
    const auto* traj_point = cur_state.traj_point;
    CHECK_GT(traj_point->t(), kNegtiveTimeThreshold);
    if (!FindOverlapRangeOrNearestPoint(
            cur_state.box.center(), search_radius, search_radius_for_mirrors,
            cur_state, i, consider_mirrors,
            spacetime_object.required_lateral_gap(),
            spacetime_object.required_lateral_gap(), &low_idx, &high_idx,
            &agent_nearest_points)) {
      if (prev_state_has_overlap) {
        CheckAndRemoveInvalidStBoundaryPoints(&st_boundaries_points);
      }
      prev_state_has_overlap = false;
      continue;
    }
    if (!prev_state_has_overlap) {
      st_boundaries_points.emplace_back();
      st_boundaries_points.back().Reserve(object_states.size() - i);
    }
    prev_state_has_overlap = true;

    auto& boundary_points = st_boundaries_points.back();
    const double middle_s =
        (path_points[high_idx].s() + path_points[low_idx].s()) * 0.5;
    const auto middle_point = path_points.Evaluate(middle_s);
    const double middle_speed = ComputeRelativeSpeed(
        traj_point->theta(), traj_point->v(), middle_point.theta());
    const double low_s = (*path_points_)[low_idx].s();
    const double high_s = (*path_points_)[high_idx].s();
    boundary_points.speed_points.emplace_back(middle_speed, traj_point->t());
    boundary_points.lower_points.emplace_back(low_s, traj_point->t());
    boundary_points.upper_points.emplace_back(high_s, traj_point->t());
    boundary_points.overlap_infos.push_back(
        OverlapInfo{.time = traj_point->t(),
                    .obj_idx = static_cast<int>(i),
                    .av_start_idx = low_idx,
                    .av_end_idx = high_idx});
  }

  MergeStBoundaryPoints(st_boundaries_points);

  std::vector<NearestSlPoint> near_sl_points;
  if (agent_nearest_points.size() > 2) {
    for (int i = 0; i < agent_nearest_points.size(); i++) {
      double obj_vl = 0.0;
      if (i < agent_nearest_points.size() - 1) {
        obj_vl =
            (agent_nearest_points[i + 1].lat_dist -
             agent_nearest_points[i].lat_dist) /
            std::fmax(agent_nearest_points[i + 1].t - agent_nearest_points[i].t,
                      1e-5);
      } else {
        obj_vl = near_sl_points.back().obj_vl;
      }
      const auto& nearest_pt = agent_nearest_points[i];
      near_sl_points.push_back(
          NearestSlPoint{.t = nearest_pt.t,
                         .av_s = nearest_pt.ra_s,
                         .av_heading = nearest_pt.ra_heading,
                         .obj_v = nearest_pt.obj_v,
                         .obj_heading = nearest_pt.obj_heading,
                         .obj_idx = nearest_pt.obj_idx,
                         .lat_dist = nearest_pt.lat_dist,
                         .obj_vl = obj_vl});
    }
  }
  agent_nearest_points.erase(
      std::remove_if(
          agent_nearest_points.begin(), agent_nearest_points.end(),
          [](AgentNearestPoint pt) { return std::fabs(pt.lat_dist) <= 1e-6; }),
      agent_nearest_points.end());

  constexpr double kMaxOverlapTimeThres = 7.0;
  if (st_boundaries_points.empty() ||
      st_boundaries_points.front().overlap_infos.front().time >
          kMaxOverlapTimeThres) {
    if (calc_moving_close_traj) {
      auto st_close_traj_points = GenerateMovingStCloseTrajPoints(
          agent_nearest_points, false,
          ToStBoundaryObjectType(spacetime_object.object_type()),
          nudge_object_info, spacetime_object.object_id());
      auto st_close_trajs = GenerateMovingStCloseTrajectories(
          spacetime_object, std::move(st_close_traj_points));
      absl::MutexLock lock(&mutex_);
      moving_close_trajs_.insert(
          moving_close_trajs_.end(),
          std::make_move_iterator(st_close_trajs.begin()),
          std::make_move_iterator(st_close_trajs.end()));
    }

    st_boundaries_points.clear();
  }
  if (generate_lane_change_gap) {
    constexpr double kLaneChangeGapLatBuffer = 2.5;
    GenerateLatInflatedStBoundary(
        spacetime_object, StBoundaryProto::LANE_CHANGE_GAP,
        kLaneChangeGapLatBuffer, &st_boundaries_points);
  } else if (st_boundaries_points.empty()) {
    constexpr double kMaxRelVel = 3.0;
    constexpr double kMinVel = 10.0;
    if (st_graph_params_->consider_large_vehicle_blind_spot() &&
        spacetime_object.planner_object().is_large_vehicle() &&
        plan_start_v_ > kMinVel &&
        std::abs(spacetime_object.planner_object().pose().v() - plan_start_v_) <
            kMaxRelVel) {
      constexpr double kPreviewDistance = 10.0;
      constexpr double kParallelHeadingDiff = M_PI / 36.0;

      const auto& obj_contour = spacetime_object.contour();
      const auto& current_path_point = path_points.front();
      const Vec2d current_pos = ToVec2d(current_path_point);
      const Vec2d cur_heading_dir =
          Vec2d::FastUnitFromAngle(current_path_point.theta());
      const double theta_diff = NormalizeAngle(current_path_point.theta() -
                                               spacetime_object.pose().theta());
      Vec2d front_most, back_most;
      obj_contour.ExtremePoints(cur_heading_dir, &back_most, &front_most);

      const double front_dis = (front_most - current_pos).Dot(cur_heading_dir) +
                               vehicle_geo_params_->back_edge_to_center();
      const double back_dis = (back_most - current_pos).Dot(cur_heading_dir) -
                              vehicle_geo_params_->front_edge_to_center();
      if (std::abs(theta_diff) < kParallelHeadingDiff &&
          back_dis < kPreviewDistance && front_dis > 0.0) {
        constexpr double kBlindSpotAreaLatBuffer = 2.5;
        GenerateLatInflatedStBoundary(
            spacetime_object, StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT,
            kBlindSpotAreaLatBuffer, &st_boundaries_points, true);
      }
    }
  } else if (st_graph_params_
                 ->enable_small_angle_cut_in_protective_st_boundary() &&
             path_approx_ != nullptr &&
             ego_dir_unit.Dot(obj_box.center() - ToVec2d(path_points[0])) >
                 0.0) {
    const auto st_close_trajs = GenerateMovingStCloseTrajPoints(
        agent_nearest_points, true,
        ToStBoundaryObjectType(spacetime_object.object_type()),
        nudge_object_info, spacetime_object.object_id());
    GenerateCutInProtectiveStBoundary(st_close_trajs, &st_boundaries_points,
                                      path_points.back().s(), plan_start_v_,
                                      max_decel_);
  }

  for (auto& st_boundary_points : st_boundaries_points) {
    st_boundary_points.nearest_sl_points = near_sl_points;
  }
  *sl_points = near_sl_points;
  return st_boundaries_points;
}

std::optional<int> StGraph::FindLeadingObjectLowerIndex(
    const Polygon2d& obj_shape, const PlanPassage& plan_passage,
    const PathSlBoundary& path_sl_boundary) const {
  const absl::StatusOr<FrenetBox> frenet_box_or =
      plan_passage.QueryFrenetBoxAtContour(obj_shape);
  if (!frenet_box_or.ok() || frenet_box_or->s_min > path_sl_boundary.end_s()) {
    return std::nullopt;
  }

  const auto [right, left] =
      path_sl_boundary.QueryBoundaryXY(frenet_box_or->s_min);
  const Segment2d leading_seg(right, left);

  const auto [right_l, left_l] =
      path_sl_boundary.QueryBoundaryL(frenet_box_or->s_min);

  if (frenet_box_or->l_min > left_l || frenet_box_or->l_max < right_l) {
    return std::nullopt;
  }
  const auto overlap_range = FindSegment2dOverlapRange(leading_seg);
  return overlap_range.has_value() ? std::optional(overlap_range->first)
                                   : std::nullopt;
}

bool StGraph::FindOverlapRangeOrNearestPoint(
    const Vec2d& search_point, double search_radius,
    double search_radius_for_mirrors, const SpacetimeObjectState& obj_state,
    int obj_idx, bool consider_mirrors, double lat_buffer, double lon_buffer,
    int* low_idx, int* high_idx,
    std::vector<AgentNearestPoint>* agent_nearest_points) const {
  CHECK_NOTNULL(low_idx);
  CHECK_NOTNULL(high_idx);

  const auto& obj_shape = obj_state.contour;
  const auto& traj_point = obj_state.traj_point;
  const double required_max_gap = std::max(lat_buffer, lon_buffer);
  if (FLAGS_planner_use_path_approx_based_st_mapping &&
      path_approx_ != nullptr) {
    const double path_step_length =
        (*path_points_)[1].s() - (*path_points_)[0].s();
    double first_s = std::numeric_limits<double>::infinity(), last_s = 0.0;
    const auto agent_overlaps = ComputeAgentOverlapsWithBuffer(
        *path_approx_, path_step_length, 0, path_points_->size() - 1, obj_shape,
        kMaxLatDist, lat_buffer, lon_buffer, search_radius);
    if (!agent_overlaps.empty()) {
      if (agent_overlaps.front().lat_dist == 0.0) {
        const auto overlap_range = ConvertToOverlapRange(agent_overlaps);
        if (overlap_range.has_value()) {
          std::tie(first_s, last_s) = *overlap_range;
        }
      }
      if (agent_nearest_points != nullptr) {
        const auto agent_overlap = agent_overlaps.front();

        agent_nearest_points->push_back(
            AgentNearestPoint{.ra_s = agent_overlap.first_ra_s,
                              .ra_heading = agent_overlap.ra_heading,
                              .t = traj_point->t(),
                              .obj_v = traj_point->v(),
                              .obj_heading = traj_point->theta(),
                              .lat_dist = agent_overlap.lat_dist,
                              .obj_idx = obj_idx});
      }
    }
    if (path_approx_for_mirrors_ != nullptr && consider_mirrors) {
      const auto agent_overlaps_with_mirrors = ComputeAgentOverlapsWithBuffer(
          *path_approx_for_mirrors_, path_step_length, 0,
          path_points_->size() - 1, obj_shape, kSearchBuffer, lat_buffer,
          lon_buffer, search_radius_for_mirrors);
      const auto overlap_range_for_mirrors =
          ConvertToOverlapRange(agent_overlaps_with_mirrors);
      if (overlap_range_for_mirrors.has_value()) {
        first_s = std::min(first_s, overlap_range_for_mirrors->first);
        last_s = std::max(last_s, overlap_range_for_mirrors->second);
      }
    }
    if (first_s > last_s) {
      VLOG(3) << "First collision s is larger than last collision s";
      return false;
    }
    *low_idx = std::max(0, CeilToInteger(first_s / path_step_length - kEps));
    *high_idx = std::min(static_cast<int>(path_points_->size() - 1),
                         FloorToInt(last_s / path_step_length + kEps));
    return true;
  }

  const double radius = std::max(search_radius, search_radius_for_mirrors);
  auto indices = path_kd_tree_->GetSegmentIndexInRadius(
      search_point.x(), search_point.y(), radius);
  if (indices.empty()) {
    VLOG(3) << absl::StrFormat("No point found within %.2f meter radius.",
                               radius);
    return false;
  }

  std::stable_sort(indices.begin(), indices.end());

  bool updated = false;
  AgentNearestPoint agent_nearest_point{
      .ra_s = 0.0,
      .ra_heading = 0.0,
      .t = traj_point->t(),
      .obj_v = traj_point->v(),
      .obj_heading = traj_point->theta(),
      .lat_dist = std::numeric_limits<double>::infinity(),
      .obj_idx = obj_idx};
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& raw_av_shape = (*av_shape_on_path_points_)[*it];
    if (agent_nearest_points == nullptr) {
      if (raw_av_shape->HasOverlapWithBuffer(obj_shape, lat_buffer, lon_buffer,
                                             consider_mirrors)) {
        *low_idx = *it;
        updated = true;
        break;
      }
    } else {
      const double dist_to_main_body =
          raw_av_shape->MainBodyDistanceTo(obj_shape);
      if (dist_to_main_body < agent_nearest_point.lat_dist) {
        agent_nearest_point.ra_s = (*path_points_)[*it].s();
        agent_nearest_point.ra_heading = (*path_points_)[*it].theta();
        agent_nearest_point.lat_dist = dist_to_main_body;
      }
      if (dist_to_main_body < required_max_gap) {
        *low_idx = *it;
        updated = true;
        break;
      }
      if (consider_mirrors) {
        if (const auto dist = raw_av_shape->LeftMirrorDistanceTo(obj_shape);
            dist.has_value() && *dist < required_max_gap) {
          if (dist_to_main_body < required_max_gap) {
            *low_idx = *it;
            updated = true;
            break;
          }
        }
        if (const auto dist = raw_av_shape->RightMirrorDistanceTo(obj_shape);
            dist.has_value() && *dist < required_max_gap) {
          if (dist_to_main_body < required_max_gap) {
            *low_idx = *it;
            updated = true;
            break;
          }
        }
      }
    }
  }
  if (!updated) {
    if (agent_nearest_points != nullptr) {
      agent_nearest_points->push_back(agent_nearest_point);
    }
    VLOG(3) << "search_point has no overlap with path.";
    return false;
  }
  for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
    const auto& raw_av_shape = (*av_shape_on_path_points_)[*it];
    if (raw_av_shape->HasOverlapWithBuffer(obj_shape, lat_buffer, lon_buffer,
                                           consider_mirrors)) {
      *high_idx = *it;
      break;
    }
  }

  return true;
}

std::pair<double, double> StGraph::GetStBoundaryTRangeByProtectionType(
    const SpacetimeObjectTrajectory& spacetime_object,
    StBoundaryProto::ProtectionType protection_type) const {
  constexpr double kStBoundaryMinT = 5.0;
  constexpr double kStBoundaryMaxT = 7.0;
  constexpr double kLowerTTC = 5.0;
  constexpr double kUpperTTC = 15.0;
  constexpr double kLowerStMaxT = 2.0;
  constexpr double kLowerStMinT = 0.0;
  if (protection_type != StBoundaryProto::LANE_CHANGE_GAP) {
    return {kStBoundaryMinT, kStBoundaryMaxT};
  }
  auto it = obj_sl_map_.find(std::string{spacetime_object.object_id()});
  if (it == obj_sl_map_.end()) {
    return {kStBoundaryMinT, kStBoundaryMaxT};
  }
  double obj_ds = it->second.ds;
  double obj_v = it->second.vs;
  double ttc = obj_ds / std::fmax(plan_start_v_ - obj_v, kEps);
  if (ttc < kLowerTTC) {
    return {kStBoundaryMinT, kStBoundaryMaxT};
  } else if (ttc > kUpperTTC) {
    return {kLowerStMinT, kLowerStMaxT};
  }
  PiecewiseLinearFunction<double> t_max_by_ttc_plf(
      {kLowerTTC, kUpperTTC}, {kStBoundaryMaxT, kLowerStMaxT});
  double t_max = t_max_by_ttc_plf(ttc);
  double t_min = std::fmax(kLowerStMinT, t_max - kLowerStMaxT);
  return {t_min, t_max};
}

void StGraph::GenerateLatInflatedStBoundary(
    const SpacetimeObjectTrajectory& spacetime_object,
    StBoundaryProto::ProtectionType protection_type, double lat_buffer,
    std::vector<StBoundaryPoints>* st_boundaries_points,
    bool large_vehicle_blind_stop) const {
  const auto st_boundary_t_range =
      GetStBoundaryTRangeByProtectionType(spacetime_object, protection_type);

  double max_time = st_boundary_t_range.second;
  if (!st_boundaries_points->empty()) {
    max_time = std::min(max_time,
                        st_boundaries_points->front().speed_points.front().t());
  }

  const Box2d& obj_box = spacetime_object.bounding_box();
  const double obj_radius = obj_box.diagonal() * 0.5 + lat_buffer;
  const double search_radius = obj_radius + ego_radius_ + kSearchBuffer;

  const auto& object_states = spacetime_object.states();
  const auto& path_points = *path_points_;
  int low_idx = 0;
  int high_idx = 0;
  int mid_idx = 0;

  CHECK_NOTNULL(st_boundaries_points);
  constexpr double kMaxAccLimit = 0.5;
  const double current_v = spacetime_object.planner_object().pose().v();
  const bool use_prediction =
      std::abs(spacetime_object.planner_object().pose().a()) > kMaxAccLimit;
  CHECK_GT(current_v, 0.0);
  const size_t current_size = st_boundaries_points->size();
  for (size_t i = 0; i < object_states.size(); ++i) {
    const auto& cur_state = object_states[i];
    const auto* traj_point = cur_state.traj_point;
    const double t =
        use_prediction ? traj_point->t() : traj_point->s() / current_v;
    if (t < st_boundary_t_range.first) {
      continue;
    }
    if (t > max_time) {
      break;
    }
    if (!FindOverlapRangeOrNearestPoint(cur_state.box.center(), search_radius,
                                        0.0, cur_state, i, false, lat_buffer,
                                        0.0, &low_idx, &high_idx, nullptr)) {
      if (st_boundaries_points->size() > current_size) {
        CheckAndRemoveInvalidStBoundaryPoints(st_boundaries_points);
        if (st_boundaries_points->size() > current_size) return;
      }
      continue;
    }

    if (st_boundaries_points->size() == current_size) {
      st_boundaries_points->emplace_back();
      st_boundaries_points->back().Reserve(object_states.size() - i);
      st_boundaries_points->back().protection_type = protection_type;
    }
    if (large_vehicle_blind_stop) {
      mid_idx = (low_idx + high_idx) / 2;
      low_idx = std::max(mid_idx - 2, low_idx);
      high_idx = std::min(mid_idx + 2, high_idx);
    }
    auto& boundary_points = st_boundaries_points->back();
    const double middle_s =
        (path_points[high_idx].s() + path_points[low_idx].s()) * 0.5;
    const auto middle_point = path_points.Evaluate(middle_s);
    const double middle_speed = ComputeRelativeSpeed(
        traj_point->theta(), traj_point->v(), middle_point.theta());
    const double low_s = (*path_points_)[low_idx].s();
    const double high_s = (*path_points_)[high_idx].s();
    boundary_points.speed_points.emplace_back(middle_speed, t);
    boundary_points.lower_points.emplace_back(low_s, t);
    boundary_points.upper_points.emplace_back(high_s, t);
    boundary_points.overlap_infos.push_back(
        OverlapInfo{.time = t,
                    .obj_idx = static_cast<int>(i),
                    .av_start_idx = low_idx,
                    .av_end_idx = high_idx});
  }
}

bool StGraph::GetStDistancePointInfo(const SpacetimeObjectState& state,
                                     double slow_down_radius,
                                     StDistancePoint* st_distance_point) const {
  CHECK_NOTNULL(st_distance_point);
  const Polygon2d& obj_shape = state.contour;
  const double search_radius =
      obj_shape.CircleRadius() + ego_radius_ + slow_down_radius;

  if (FLAGS_planner_use_path_approx_based_st_mapping &&
      path_approx_ != nullptr) {
    const double path_step_length =
        (*path_points_)[1].s() - (*path_points_)[0].s();
    const auto agent_overlaps = ComputeAgentOverlapsWithBuffer(
        *path_approx_, path_step_length, 0, path_points_->size() - 1, obj_shape,
        slow_down_radius, 0.0, 0.0, search_radius);
    if (agent_overlaps.empty()) return false;
    const auto agent_overlap = agent_overlaps.front();
    const double dist = std::fabs(agent_overlap.lat_dist);
    if (dist > slow_down_radius) return false;

    st_distance_point->path_s = agent_overlap.first_ra_s;
    st_distance_point->distance = dist;
    st_distance_point->relative_v =
        ComputeRelativeSpeed((*state.traj_point).theta(),
                             (*state.traj_point).v(), agent_overlap.ra_heading);
    return true;
  }

  const auto indices = path_kd_tree_->GetSegmentIndexInRadius(
      obj_shape.CircleCenter().x(), obj_shape.CircleCenter().y(),
      search_radius);
  if (indices.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::max();
  PathPoint nearest_pt;
  for (const auto index : indices) {
    const auto& pt = (*path_points_)[index];
    const double dist = obj_shape.DistanceTo(ToVec2d(pt));
    if (dist < min_dist) {
      min_dist = dist;
      nearest_pt = pt;
    }
  }

  min_dist -= vehicle_geo_params_->width() * 0.5;
  if (min_dist > slow_down_radius) {
    return false;
  }

  st_distance_point->path_s = nearest_pt.s();
  st_distance_point->distance = min_dist;
  st_distance_point->relative_v = ComputeRelativeSpeed(
      (*state.traj_point).theta(), (*state.traj_point).v(), nearest_pt.theta());
  return true;
}

std::vector<CloseSpaceTimeObject> StGraph::GetCloseSpaceTimeObjects(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    absl::Span<const SpacetimeObjectTrajectory* const> spacetime_object_trajs,
    double slow_down_radius) const {
  absl::flat_hash_set<std::string> traj_id_set;
  traj_id_set.reserve(st_boundaries_with_decision.size());
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    if (const auto& traj_id = st_boundary_with_decision.traj_id();
        traj_id.has_value()) {
      traj_id_set.insert(*traj_id);
    }
  }
  std::vector<CloseSpaceTimeObject> close_space_time_objects;
  close_space_time_objects.reserve(spacetime_object_trajs.size());
  for (const SpacetimeObjectTrajectory* obj_traj : spacetime_object_trajs) {
    if (traj_id_set.contains(obj_traj->traj_id()) ||
        obj_traj->states().empty()) {
      continue;
    }
    if (ToStBoundaryObjectType(obj_traj->planner_object().type()) ==
        StBoundaryProto::IGNORABLE) {
      continue;
    }

    StDistancePoint st_dis_point;
    if (!GetStDistancePointInfo(obj_traj->states()[0], slow_down_radius,
                                &st_dis_point)) {
      continue;
    }

    auto& object = close_space_time_objects.emplace_back();
    object.st_distance_points.push_back(st_dis_point);
    object.is_stationary = obj_traj->is_stationary();

    if (!object.is_stationary) {
      CHECK_GE(obj_traj->states().size(), 2);
      StDistancePoint second_st_dis_point;
      if (!GetStDistancePointInfo(obj_traj->states()[1], slow_down_radius,
                                  &second_st_dis_point)) {
        object.is_away_from_traj = true;
      } else {
        if (second_st_dis_point.distance - st_dis_point.distance >
            e2e_noa::prediction::kPredictionTimeStep *
                std::fabs(st_dis_point.relative_v)) {
          object.is_away_from_traj = true;
        } else {
          object.is_away_from_traj = false;
        }
      }
    }
    object.contour = obj_traj->states()[0].contour;
    object.object_type =
        ToStBoundaryObjectType(obj_traj->planner_object().type());
    object.id = {obj_traj->traj_id().data(), obj_traj->traj_id().size()};
  }
  return close_space_time_objects;
}

}  // namespace e2e_noa::planning
