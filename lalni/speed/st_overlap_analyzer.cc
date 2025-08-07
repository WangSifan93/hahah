#include "speed/st_overlap_analyzer.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "container/strong_int.h"
#include "glog/logging.h"
#include "maps/lane_point.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/circle2d.h"
#include "math/geometry/polygon2d.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "speed/overlap_info.h"
#include "speed/st_point.h"
#include "speed_planning.pb.h"
#include "trajectory_point.pb.h"
#include "util/lane_path_util.h"
#include "util/lane_point_util.h"
#include "util/map_util.h"
#include "util/path_util.h"
#include "util/perception_util.h"
#include "util/planner_semantic_map_util.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa {
namespace planning {

namespace {

constexpr double kEps = 1e-6;

struct LaneInteraction {
  double fraction;
  mapping::LanePoint other_lane_point;
  StOverlapMetaProto::OverlapPriority priority;
  ad_e2e::planning::LaneType other_lane_type;

  LaneInteraction(double frac, mapping::LanePoint other_lane_pt,
                  StOverlapMetaProto::OverlapPriority prio,
                  ad_e2e::planning::LaneType other_lane_tp)
      : fraction(frac),
        other_lane_point(other_lane_pt),
        priority(prio),
        other_lane_type(other_lane_tp) {}
};

using LaneInteractionMap =
    absl::flat_hash_map<mapping::ElementId, std::vector<LaneInteraction>>;

bool RunStOverlapAnalzyerByStBoundaryObjectType(
    StBoundaryProto::ObjectType object_type) {
  switch (object_type) {
    case StBoundaryProto::VEHICLE:
    case StBoundaryProto::CYCLIST:
    case StBoundaryProto::PEDESTRIAN:
      return true;
    case StBoundaryProto::STATIC:
    case StBoundaryProto::UNKNOWN_OBJECT:
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::VIRTUAL:
    case StBoundaryProto::IMPASSABLE_BOUNDARY:
    case StBoundaryProto::PATH_BOUNDARY:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

bool RunStOverlapnalyzerByStBoundarySourceType(
    StBoundarySourceTypeProto::Type source) {
  switch (source) {
    case StBoundarySourceTypeProto::ST_OBJECT:
      return true;
    case StBoundarySourceTypeProto::UNKNOWN:
    case StBoundarySourceTypeProto::VIRTUAL:
    case StBoundarySourceTypeProto::IMPASSABLE_BOUNDARY:
    case StBoundarySourceTypeProto::PATH_BOUNDARY:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

bool IsOverlapAreaOutOfCurrentLane(
    const SpacetimeObjectTrajectory& st_traj, const PlanPassage& plan_passage,
    const VehicleGeometryParamsProto& vehicle_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, int av_idx, int obj_idx,
    bool is_current_lane) {
  std::function<bool(double, double)> comp;
  if (is_current_lane) {
    comp = std::greater<double>();
  } else {
    comp = std::less<double>();
  }
  const auto& obj_contour = st_traj.states()[obj_idx].contour;

  const double buffer = st_traj.required_lateral_gap();

  const PiecewiseLinearFunction<double> time_boundary_buffer_plf(
      {0.0, 5.0, 7.0, 10.0}, {0.0, 0.0, 0.2, 0.5});
  const double boundary_buffer =
      time_boundary_buffer_plf(st_traj.states()[obj_idx].traj_point->t());

  const auto& av_shape_ptr = av_shapes[av_idx];

  const auto is_polygon_in_lane = [&plan_passage, &comp](
                                      const Polygon2d& polygon,
                                      double boundary_buffer) -> bool {
    const auto frenet_box_or = plan_passage.QueryFrenetBoxAtContour(polygon);
    if (frenet_box_or.ok()) {
      const auto s_min_boundary =
          plan_passage.QueryEnclosingLaneBoundariesAtS(frenet_box_or->s_min);
      const auto s_max_boundary =
          plan_passage.QueryEnclosingLaneBoundariesAtS(frenet_box_or->s_max);

      return (s_min_boundary.left.has_value() &&
              s_max_boundary.left.has_value() &&
              s_min_boundary.right.has_value() &&
              s_max_boundary.right.has_value() &&
              comp(s_min_boundary.left->lat_offset + boundary_buffer,
                   frenet_box_or->l_max) &&
              comp(s_max_boundary.left->lat_offset + boundary_buffer,
                   frenet_box_or->l_max) &&
              comp(frenet_box_or->l_min,
                   s_min_boundary.right->lat_offset - boundary_buffer) &&
              comp(frenet_box_or->l_min,
                   s_max_boundary.right->lat_offset - boundary_buffer));
    }
    return true;
  };

  const auto [min_mirror_height, max_mirror_height] =
      ComputeMinMaxMirrorAverageHeight(vehicle_params);
  const bool consider_mirrors =
      IsConsiderMirrorObject(st_traj.planner_object().object_proto(),
                             min_mirror_height, max_mirror_height);
  if (consider_mirrors) {
    const auto is_circle_in_lane =
        [&plan_passage, &comp](const Circle2d& circle, double buffer) -> bool {
      const auto frenet_pt_or =
          plan_passage.QueryFrenetCoordinateAt(circle.center());
      if (frenet_pt_or.ok()) {
        const auto s_boundary =
            plan_passage.QueryEnclosingLaneBoundariesAtS(frenet_pt_or->s);
        const double buffered_radius = circle.radius() + buffer;

        return (s_boundary.right.has_value() && s_boundary.left.has_value() &&
                comp(frenet_pt_or->l - buffered_radius,
                     s_boundary.right->lat_offset) &&
                comp(s_boundary.left->lat_offset,
                     frenet_pt_or->l + buffered_radius));
      }
      return true;
    };
    const double mirror_buffer = buffer - boundary_buffer;
    if ((av_shape_ptr->LeftMirrorHasOverlapWithBuffer(obj_contour, buffer) &&
         !is_circle_in_lane(*av_shape_ptr->left_mirror(), mirror_buffer)) ||
        (av_shape_ptr->RightMirrorHasOverlapWithBuffer(obj_contour, buffer) &&
         !is_circle_in_lane(*av_shape_ptr->right_mirror(), mirror_buffer))) {
      return true;
    }
  }

  Polygon2d overlap_polygon;
  const Polygon2d av_polygon = Polygon2d(
      av_shape_ptr->GetCornersWithBufferCounterClockwise(buffer, buffer), true);
  if (obj_contour.ComputeOverlap(av_polygon, &overlap_polygon) &&
      !is_polygon_in_lane(overlap_polygon, boundary_buffer)) {
    return true;
  }

  return false;
}

absl::StatusOr<OverlapSourcePriority> AnalyzeOverlapSourceAndRank(
    const StBoundaryRef& st_boundary,
    StOverlapMetaProto::OverlapPattern overlap_pattern,
    const SpacetimeObjectTrajectory& st_traj,
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlanPassage& plan_passage,
    const LaneInteractionMap& lane_interaction_map,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const LaneChangeStage& lc_stage,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v) {
  OverlapSourcePriority res;

  if (overlap_pattern != StOverlapMetaProto::ENTER &&
      overlap_pattern != StOverlapMetaProto::CROSS &&
      overlap_pattern != StOverlapMetaProto::INTERFERE) {
    return res;
  }

  if (st_boundary->bottom_left_point().s() <= 0.0) {
    return res;
  }

  if (st_boundary->object_type() == StBoundaryProto::PEDESTRIAN) {
    if (st_boundary->overlap_infos().size() > 0) {
      res.source = StOverlapMetaProto::OBJECT_CUTIN;
      res.priority = StOverlapMetaProto::HIGH;
      res.priority_reason = absl::StrCat(
          "HIGH priority for object type ",
          StBoundaryProto::ObjectType_Name(st_boundary->object_type()));
    } else {
      res.source = StOverlapMetaProto::OTHER;
      res.priority = StOverlapMetaProto::LOW;
      res.priority_reason = absl::StrCat(
          "LOW priority for object type ",
          StBoundaryProto::ObjectType_Name(st_boundary->object_type()));
    }

    return res;
  }

  const auto& overlap_infos = st_boundary->overlap_infos();

  const auto& fo_info = overlap_infos.front();

  enum class LaneChangeSemantic { NONE = 0, LEFT = 1, RIGHT = 2 };
  LaneChangeSemantic fo_av_lane_change_semantic = LaneChangeSemantic::NONE;
  std::vector<mapping::ElementId> fo_av_lane_id_list;
  constexpr double kLaneChangeCheckLookAheadDist = 20.0;

  std::optional<std::vector<int>> lc_lane_path_id_history = std::nullopt;
  const double check_lc_end_path_s =
      path[fo_info.av_end_idx].s() + kLaneChangeCheckLookAheadDist;
  for (int i = fo_info.av_start_idx;
       i < path_semantics.size() && path[i].s() <= check_lc_end_path_s; ++i) {
    const auto& lane_path_id_history = path_semantics[i].lane_path_id_history;
    if (fo_av_lane_change_semantic == LaneChangeSemantic::NONE &&
        lane_path_id_history.back() != 0) {
      CHECK_GT(lane_path_id_history.size(), 1);
      CHECK_NE(lane_path_id_history.back(),
               lane_path_id_history[lane_path_id_history.size() - 2]);
      if (lane_path_id_history.back() >
          lane_path_id_history[lane_path_id_history.size() - 2]) {
        fo_av_lane_change_semantic = LaneChangeSemantic::LEFT;
      } else {
        fo_av_lane_change_semantic = LaneChangeSemantic::RIGHT;
      }
      lc_lane_path_id_history = lane_path_id_history;
    }
    if (i <= fo_info.av_end_idx &&
        std::find(fo_av_lane_id_list.begin(), fo_av_lane_id_list.end(),
                  path_semantics[i].closest_lane_point.lane_id()) ==
            fo_av_lane_id_list.end()) {
      fo_av_lane_id_list.push_back(
          path_semantics[i].closest_lane_point.lane_id());
    }
  }

  if (fo_info.av_end_idx >= path_semantics.size()) {
    return absl::NotFoundError("Overlap index exceeds path semantics range!");
  }

  struct DeviationInformation {
    int av_idx;
    int obj_idx;
    double deviation_distance;
    DeviationInformation(int av_idx, int obj_idx, double deviation_distance)
        : av_idx(av_idx),
          obj_idx(obj_idx),
          deviation_distance(deviation_distance) {}
  };
  std::optional<DeviationInformation> max_deviation_info = std::nullopt;
  for (const auto& overlap_info : overlap_infos) {
    const int av_idx =
        (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;
    if (av_idx >= path_semantics.size() ||
        path_semantics[av_idx].lane_path_id_history.size() > 1) {
      continue;
    }
    if (path_semantics[av_idx].lane_info->IsVirtual()) {
      const auto& nearest_station =
          plan_passage.FindNearestStation(ToVec2d(path[av_idx]));
      if (nearest_station.lane_id() == path_semantics[av_idx].lane_info->id() ||
          nearest_station.is_virtual()) {
        continue;
      }
    }
    if (!max_deviation_info.has_value() ||
        path_semantics[av_idx].deviation_distance >
            max_deviation_info->deviation_distance) {
      max_deviation_info =
          DeviationInformation(av_idx, overlap_info.obj_idx,
                               path_semantics[av_idx].deviation_distance);
    }
  }

  if (max_deviation_info.has_value()) {
    const auto av_idx = max_deviation_info->av_idx;
    const auto obj_idx = max_deviation_info->obj_idx;
    const auto& nearest_station =
        plan_passage.FindNearestStation(ToVec2d(path[av_idx]));
    const auto current_lane_id = path_semantics[av_idx].lane_info->id();
    bool is_current_lane = false;
    if (nearest_station.lane_id().empty()) {
      is_current_lane = (lc_stage == LaneChangeStage::LCS_NONE);
    } else {
      is_current_lane = nearest_station.lane_id() == current_lane_id;
    }
    if (IsOverlapAreaOutOfCurrentLane(st_traj, plan_passage,
                                      vehicle_geometry_params, av_shapes,
                                      av_idx, obj_idx, is_current_lane)) {
      const auto& av_pos = path[0];
      auto av_lane_id = plan_passage.lane_path().lane_ids().front();
      for (const auto& lane_id : plan_passage.lane_path().lane_ids()) {
        double start_fraction = 0.0;
        double end_fraction = 1.0;
        const auto& lane = psmm.FindLaneByIdOrNull(lane_id);
        if (!lane) continue;

        const auto ff = BuildBruteForceFrenetFrame(lane->points(), false);
        if (!ff.ok()) continue;
        FrenetCoordinate sl;
        Vec2d normal;
        std::pair<int, int> index_pair;
        double alpha;
        ff.value().XYToSL({av_pos.x(), av_pos.y()}, &sl, &normal, &index_pair,
                          &alpha);
        start_fraction = sl.s / std::fmax(lane->curve_length(), 1e-2);
        if (start_fraction < 0.0 - kEps || start_fraction > 1.0 - kEps) {
          continue;
        } else {
          av_lane_id = lane_id;
        }
      }
      const auto& nearest_lane = psmm.FindLaneByIdOrNull(av_lane_id);

      if (nearest_lane && nearest_lane->center_line().IsValid() &&
          nearest_lane->type() == ad_e2e::planning::LANE_VIRTUAL_JUNCTION) {
        bool find_start = false;
        constexpr double kSpeedDiffThreshold = 10.0;
        Vec2d ref_point = nearest_lane->points().back();
        for (const auto& seg : plan_passage.lane_path()) {
          if (seg.lane_id == av_lane_id || find_start) {
            find_start = true;
            const auto& lane = psmm.FindLaneByIdOrNull(seg.lane_id);
            if (!lane) break;
            if (lane->type() != ad_e2e::planning::LANE_VIRTUAL_JUNCTION) {
              ref_point = lane->points().front();
              break;
            } else {
              ref_point = lane->points().back();
            }
          } else {
            continue;
          }
        }
        const auto& lane_theta = plan_passage.QueryTangentAt(ref_point);
        bool smaller_theta_diff = false;
        if (lane_theta.ok() &&
            nearest_lane->turn_type() == ad_e2e::planning::LEFT_TURN) {
          smaller_theta_diff =
              (path[av_idx].theta() - lane_theta.value().Angle() > kEps) &&
              (st_traj.states()[obj_idx].traj_point->theta() -
                   lane_theta.value().Angle() >
               kEps) &&
              (path[av_idx].theta() - lane_theta.value().Angle() <
               st_traj.states()[obj_idx].traj_point->theta() -
                   lane_theta.value().Angle());
        } else if (lane_theta.ok() &&
                   nearest_lane->turn_type() == ad_e2e::planning::RIGHT_TURN) {
          smaller_theta_diff =
              (path[av_idx].theta() - lane_theta.value().Angle() < kEps) &&
              (st_traj.states()[obj_idx].traj_point->theta() -
                   lane_theta.value().Angle() <
               kEps) &&
              (path[av_idx].theta() - lane_theta.value().Angle() >
               st_traj.states()[obj_idx].traj_point->theta() -
                   lane_theta.value().Angle());
        }

        if (st_traj.states().front().traj_point->v() - init_v >
                Kph2Mps(kSpeedDiffThreshold) ||
            !smaller_theta_diff) {
          res.source = StOverlapMetaProto::AV_CUTIN;
          res.priority = StOverlapMetaProto::LOW;
          res.priority_reason =
              "LOW priority for AV cutting in object (overlap method)";
          return res;
        }
      } else {
        res.source = StOverlapMetaProto::AV_CUTIN;
        res.priority = StOverlapMetaProto::LOW;
        res.priority_reason =
            "LOW priority for AV cutting in object (overlap method)";
        return res;
      }
    }
  }

  const auto* fo_obj_point = st_traj.states()[fo_info.obj_idx].traj_point;
  const auto& fo_obj_pos = fo_obj_point->pos();
  const int av_mean_idx = (fo_info.av_start_idx + fo_info.av_end_idx) / 2;
  if (fo_av_lane_change_semantic != LaneChangeSemantic::NONE) {
    const auto& fo_av_path_point = path[av_mean_idx];
    const bool is_fo_obj_on_path_left =
        Vec2d::FastUnitFromAngle(fo_av_path_point.theta())
            .CrossProd(fo_obj_pos - ToVec2d(fo_av_path_point)) >= 0.0;
    if ((fo_av_lane_change_semantic == LaneChangeSemantic::LEFT &&
         is_fo_obj_on_path_left) ||
        (fo_av_lane_change_semantic == LaneChangeSemantic::RIGHT &&
         !is_fo_obj_on_path_left)) {
      res.source = StOverlapMetaProto::AV_CUTIN;
      res.priority = StOverlapMetaProto::LOW;
      res.priority_reason = "LOW priority for AV cutting in object";
      CHECK(lc_lane_path_id_history.has_value());
      for (int i = 0; i < path_semantics.size(); ++i) {
        if (path_semantics[i].lane_path_id_history ==
            *lc_lane_path_id_history) {
          res.time_to_lc_complete = path[i].s() / (init_v + 1e-6);
          break;
        }
      }
      CHECK(res.time_to_lc_complete.has_value());
      return res;
    }
  } else {
  }

  if (handle_intersection(st_boundary, path_semantics, psmm, st_traj, path,
                          fo_av_lane_id_list, fo_info, av_mean_idx, res,
                          lc_stage)) {
    return res;
  }

  res.source = StOverlapMetaProto::OBJECT_CUTIN;
  res.priority = StOverlapMetaProto::HIGH;
  res.priority_reason = "HIGH priority for AV being cut in by object";
  return res;
}

bool IsDrivingParallel(const std::optional<double>& theta_diff) {
  if (!theta_diff.has_value()) return false;
  constexpr double kDrivingParallelHeadingThres = M_PI / 6.0;
  return std::abs(*theta_diff) < kDrivingParallelHeadingThres;
}

std::optional<double> CalculateThetaDiff(
    const SpacetimeObjectTrajectory& st_traj, const PlanPassage& plan_passage,
    const std::optional<KdTreeFrenetCoordinate>& target_frenet_frame,
    StOverlapMetaProto::OverlapSource overlap_source) {
  if (overlap_source != StOverlapMetaProto::LANE_MERGE &&
      overlap_source != StOverlapMetaProto::OBJECT_CUTIN) {
    return std::nullopt;
  }

  constexpr double kMaxLLimit = 10.0;
  const Vec2d& obj_pos = st_traj.pose().pos();
  if (target_frenet_frame.has_value()) {
    const auto obj_frenet_coord = target_frenet_frame->XYToSL(obj_pos);
    if (std::abs(obj_frenet_coord.l) > kMaxLLimit) {
      return std::nullopt;
    }
    const auto obj_frenet_tangent =
        target_frenet_frame->InterpolateTangentByS(obj_frenet_coord.s);
    const double ref_theta = obj_frenet_tangent.FastAngle();
    const double theta_diff =
        NormalizeAngle(st_traj.pose().theta() - ref_theta);
    return theta_diff;
  } else {
    const auto obj_frenet_coord = plan_passage.QueryFrenetCoordinateAt(obj_pos);
    if (obj_frenet_coord.ok()) {
      if (std::abs(obj_frenet_coord->l) > kMaxLLimit) {
        return std::nullopt;
      }
      const auto obj_frenet_theta =
          plan_passage.QueryTangentAngleAtS(obj_frenet_coord->s);
      if (obj_frenet_theta.ok()) {
        const double ref_theta = *obj_frenet_theta;
        const double theta_diff =
            NormalizeAngle(st_traj.pose().theta() - ref_theta);
        return theta_diff;
      }
    }
  }
  return std::nullopt;
}

std::optional<std::pair<double, double>> CalculateProjectionDistance(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& path,
    StOverlapMetaProto::OverlapSource overlap_source) {
  if (overlap_source != StOverlapMetaProto::LANE_MERGE &&
      overlap_source != StOverlapMetaProto::LANE_CROSS &&
      overlap_source != StOverlapMetaProto::OBJECT_CUTIN) {
    return std::nullopt;
  }
  const auto& obj_contour = st_traj.contour();

  const Vec2d av_heading_dir = Vec2d::FastUnitFromAngle(path.front().theta());
  const Vec2d av_pos = Vec2d(path.front().x(), path.front().y());
  Vec2d front_most, back_most;
  obj_contour.ExtremePoints(av_heading_dir, &back_most, &front_most);

  const double front_dis = (front_most - av_pos).Dot(av_heading_dir);
  const double back_dis = (back_most - av_pos).Dot(av_heading_dir);
  return std::make_pair(front_dis, back_dis);
}

StOverlapMetaProto::ModificationType AnalyzeOverlapModificationCategory(
    const StBoundaryRef& st_boundary, const std::optional<double>& theta_diff,
    StOverlapMetaProto::OverlapSource overlap_source,
    StOverlapMetaProto::OverlapPriority overlap_priority) {
  if (overlap_priority == StOverlapMetaProto::UNKNOWN_PRIORITY) {
    return StOverlapMetaProto::NON_MODIFIABLE;
  }
  CHECK(st_boundary->object_type() == StBoundaryProto::VEHICLE ||
        st_boundary->object_type() == StBoundaryProto::CYCLIST ||
        st_boundary->object_type() == StBoundaryProto::PEDESTRIAN);

  if (st_boundary->object_type() == StBoundaryProto::VEHICLE ||
      st_boundary->object_type() == StBoundaryProto::CYCLIST) {
    switch (overlap_source) {
      case StOverlapMetaProto::LANE_MERGE:
      case StOverlapMetaProto::LANE_CROSS:
      case StOverlapMetaProto::AV_CUTIN: {
        return StOverlapMetaProto::LON_MODIFIABLE;
      }
      case StOverlapMetaProto::OBJECT_CUTIN: {
        if (IsDrivingParallel(theta_diff)) {
          return StOverlapMetaProto::LON_LAT_MODIFIABLE;
        } else {
          return StOverlapMetaProto::LON_MODIFIABLE;
        }
      }
      case StOverlapMetaProto::UNKNOWN_SOURCE:
      case StOverlapMetaProto::OTHER: {
        return StOverlapMetaProto::NON_MODIFIABLE;
      }
    }
  }

  if (st_boundary->object_type() == StBoundaryProto::PEDESTRIAN &&
      st_boundary->overlap_infos().size() > 0) {
    switch (overlap_source) {
      case StOverlapMetaProto::OBJECT_CUTIN: {
        return StOverlapMetaProto::LON_LAT_MODIFIABLE;
      }
    }
  }
  return StOverlapMetaProto::NON_MODIFIABLE;
}

absl::StatusOr<StOverlapMetaProto> AnalyzeStOverlap(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const LaneInteractionMap& lane_interaction_map,
    const PlanPassage& plan_passage,
    const std::optional<KdTreeFrenetCoordinate>& target_frenet_frame,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const LaneChangeStage& lc_stage,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v) {
  StOverlapMetaProto overlap_meta;

  overlap_meta.set_pattern(AnalyzeOverlapPattern(st_boundary, st_traj, path,
                                                 vehicle_geometry_params));

  ASSIGN_OR_RETURN(auto source_priority,
                   AnalyzeOverlapSourceAndRank(
                       st_boundary, overlap_meta.pattern(), st_traj, psmm, path,
                       path_semantics, plan_passage, lane_interaction_map,
                       vehicle_geometry_params, lc_stage, av_shapes, init_v));
  VLOG(2) << "[speed finder] id" << st_boundary->id() << " source "
          << source_priority.source
          << " | priority: " << source_priority.priority
          << " | priority_reason: " << source_priority.priority_reason;
  overlap_meta.set_source(source_priority.source);
  overlap_meta.set_priority(source_priority.priority);
  overlap_meta.set_priority_reason(std::move(source_priority.priority_reason));
  if (source_priority.time_to_lc_complete.has_value()) {
    overlap_meta.set_time_to_lc_complete(*source_priority.time_to_lc_complete);
  }
  if (source_priority.is_making_u_turn.has_value()) {
    overlap_meta.set_is_making_u_turn(*source_priority.is_making_u_turn);
  }
  if (source_priority.is_merging_straight_lane.has_value()) {
    overlap_meta.set_is_merging_straight_lane(
        *source_priority.is_merging_straight_lane);
  }
  if (source_priority.is_crossing_straight_lane.has_value()) {
    overlap_meta.set_is_crossing_straight_lane(
        *source_priority.is_crossing_straight_lane);
  }
  if (source_priority.obj_lane_direction.has_value()) {
    overlap_meta.set_obj_lane_direction(
        static_cast<e2e_noa::StOverlapMetaProto_LaneDirection>(
            *source_priority.obj_lane_direction));
  }
  if (source_priority.is_unprotected_left_turn.has_value()) {
    overlap_meta.set_is_unprotected_left_turn(
        *source_priority.is_unprotected_left_turn);
  }
  const auto theta_diff = CalculateThetaDiff(
      st_traj, plan_passage, target_frenet_frame, overlap_meta.source());
  if (theta_diff.has_value()) {
    overlap_meta.set_theta_diff(*theta_diff);
  }
  const auto projection_dis_pair =
      CalculateProjectionDistance(st_traj, path, overlap_meta.source());
  if (projection_dis_pair.has_value()) {
    overlap_meta.set_front_most_projection_distance(projection_dis_pair->first);
    overlap_meta.set_rear_most_projection_distance(projection_dis_pair->second);
  }

  overlap_meta.set_modification_type(AnalyzeOverlapModificationCategory(
      st_boundary, theta_diff, overlap_meta.source(), overlap_meta.priority()));

  return overlap_meta;
}

}  // namespace

StOverlapMetaProto::OverlapPattern AnalyzeOverlapPattern(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto& overlap_infos = st_boundary->overlap_infos();
  CHECK(!overlap_infos.empty());

  const auto& first_overlap_info = overlap_infos.front();
  const auto& last_overlap_info = overlap_infos.back();

  const int state_num = st_traj.states().size();
  const int path_num = path.size();

  enum class RelativeLonPosition {
    BEHIND = 0,
    AHEAD = 1,
    INTERSECT = 2,
  };
  const auto compute_obj_relative_position_with_av_box =
      [&path, &st_traj, &vehicle_geometry_params](
          int av_path_idx, int obj_traj_idx) -> RelativeLonPosition {
    const auto& obj_contour = st_traj.states()[obj_traj_idx].contour;
    const auto& av_path_point = path[av_path_idx];
    const auto av_path_dir = Vec2d::FastUnitFromAngle(av_path_point.theta());
    Vec2d front, back;
    obj_contour.ExtremePoints(av_path_dir, &back, &front);
    const Vec2d av_path_pos = ToVec2d(av_path_point);
    if ((back - av_path_pos).Dot(av_path_dir) >
        vehicle_geometry_params.front_edge_to_center() +
            st_traj.required_lateral_gap()) {
      return RelativeLonPosition::AHEAD;
    } else if ((front - av_path_pos).Dot(av_path_dir) <
               -vehicle_geometry_params.back_edge_to_center() -
                   st_traj.required_lateral_gap()) {
      return RelativeLonPosition::BEHIND;
    } else {
      return RelativeLonPosition::INTERSECT;
    }
  };

  if (first_overlap_info.obj_idx == 0 ||
      compute_obj_relative_position_with_av_box(
          0, first_overlap_info.obj_idx - 1) == RelativeLonPosition::BEHIND) {
    if (last_overlap_info.obj_idx == state_num - 1 ||
        compute_obj_relative_position_with_av_box(
            path_num - 1, last_overlap_info.obj_idx + 1) ==
            RelativeLonPosition::AHEAD) {
      return StOverlapMetaProto::STAY;
    } else {
      return StOverlapMetaProto::LEAVE;
    }
  }

  if (last_overlap_info.obj_idx == state_num - 1 ||
      compute_obj_relative_position_with_av_box(
          path_num - 1, last_overlap_info.obj_idx + 1) ==
          RelativeLonPosition::AHEAD) {
    return StOverlapMetaProto::ENTER;
  } else {
    const auto get_side_on_path = [&path, &st_traj](int av_path_idx,
                                                    int obj_traj_idx) -> bool {
      const auto& av_path_point = path[av_path_idx];
      const auto& obj_traj_point = *st_traj.states()[obj_traj_idx].traj_point;
      const Vec2d ref = obj_traj_point.pos() - ToVec2d(av_path_point);
      const bool is_on_path_left =
          Vec2d::FastUnitFromAngle(av_path_point.theta()).CrossProd(ref) >= 0.0;
      return is_on_path_left;
    };

    const bool first_overlap_side = get_side_on_path(
        (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2,
        first_overlap_info.obj_idx);
    const bool last_overlap_side = get_side_on_path(
        (last_overlap_info.av_start_idx + last_overlap_info.av_end_idx) / 2,
        last_overlap_info.obj_idx);
    if (first_overlap_side == last_overlap_side) {
      return StOverlapMetaProto::INTERFERE;
    } else {
      return StOverlapMetaProto::CROSS;
    }
  }
}

bool IsAnalyzableStBoundary(const StBoundaryRef& st_boundary) {
  if (!RunStOverlapnalyzerByStBoundarySourceType(st_boundary->source_type())) {
    return false;
  }
  if (!RunStOverlapAnalzyerByStBoundaryObjectType(st_boundary->object_type())) {
    return false;
  }
  if (st_boundary->is_stationary()) return false;
  return true;
}

void AnalyzeStOverlaps(
    const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlanPassage& plan_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const LaneChangeStage& lc_stage,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v,
    std::vector<StBoundaryRef>* st_boundaries) {
  constexpr double kBackwardExtendLength = 30.0;
  std::optional<KdTreeFrenetCoordinate> target_frenet_frame = std::nullopt;
  const auto target_lane_path_extend = BackwardExtendLanePath(
      psmm, plan_passage.extend_lane_path(), kBackwardExtendLength);
  auto target_frenet_frame_or = BuildKdTreeFrenetFrame(
      SampleLanePathPoints(psmm, target_lane_path_extend), true);
  if (target_frenet_frame_or.ok()) {
    target_frenet_frame = std::move(*target_frenet_frame_or);
  }

  LaneInteractionMap path_lane_interaction_map;
  for (const auto& path_semantic : path_semantics) {
    const auto& current_closest_lane_point =
        path_semantics[0].closest_lane_point;
    const auto& closest_lane_point = path_semantic.closest_lane_point;
    if (path_lane_interaction_map.contains(closest_lane_point.lane_id())) {
      continue;
    }
    const auto lane_proto_ptr =
        psmm.FindCurveLaneByIdOrNull(closest_lane_point.lane_id());
    if (nullptr == lane_proto_ptr) continue;
    const auto& lane_proto = *lane_proto_ptr;
    path_lane_interaction_map[closest_lane_point.lane_id()] = {};
  }

  for (auto& st_boundary : *st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;

    CHECK(st_boundary->traj_id().has_value());
    const auto& traj_id = *st_boundary->traj_id();
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

    ASSIGN_OR_CONTINUE(
        StOverlapMetaProto overlap_meta,
        AnalyzeStOverlap(st_boundary, *traj, psmm, path, path_semantics,
                         path_lane_interaction_map, plan_passage,
                         target_frenet_frame, vehicle_geometry_params, lc_stage,
                         av_shapes, init_v));

    st_boundary->set_overlap_meta(std::move(overlap_meta));
  }
}

bool handle_intersection(
    const StBoundaryRef& st_boundary,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& path,
    const std::vector<mapping::ElementId>& fo_av_lane_id_list,
    const OverlapInfo& fo_info, const int& av_mean_idx,
    OverlapSourcePriority& res, const LaneChangeStage& lc_stage) {
  bool fo_av_going_straight = false;
  for (const auto lane_id : fo_av_lane_id_list) {
    const auto lane_info_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
    if (nullptr == lane_info_ptr) continue;
    const auto& lane_info = *lane_info_ptr;
    if (lane_info.turn_type() == e2e_noa::planning::TurnType::NO_TURN) {
      fo_av_going_straight = true;
      break;
    }
  }

  const double init_obj_heading = st_traj.states().front().traj_point->theta();
  const double end_obj_heading = st_traj.states().back().traj_point->theta();
  constexpr double kUTurnHeadingDiff = 0.75 * M_PI;
  res.is_making_u_turn =
      (fo_av_going_straight &&
       std::abs(NormalizeAngle(end_obj_heading - init_obj_heading)) >=
           kUTurnHeadingDiff);

  const auto get_angle_diff = [&path, &st_traj](int av_path_idx,
                                                int obj_traj_idx) -> double {
    const auto& obj_traj_point = *st_traj.states()[obj_traj_idx].traj_point;
    const auto& av_path_point = path[av_path_idx];
    return NormalizeAngle(obj_traj_point.theta() - av_path_point.theta());
  };
  if (av_mean_idx >= path.size() || av_mean_idx >= path_semantics.size()) {
    DLOG(ERROR) << "[speed finder] av_mean_idx error " << av_mean_idx << " | "
                << path.size() << " | " << path_semantics.size();
    return false;
  }

  const double theta_diff = get_angle_diff(av_mean_idx, fo_info.obj_idx);
  constexpr double kCrossingAngleDiffThre = M_PI_4;
  res.is_crossing_straight_lane =
      (fo_av_going_straight &&
       st_boundary->object_type() == StBoundaryProto::VEHICLE &&
       std::abs(theta_diff) > kCrossingAngleDiffThre);
  VLOG(2) << "[speed finder] st id: " << st_boundary->id()
          << " |is_making_u_turn "
          << static_cast<int>(res.is_making_u_turn.has_value() &&
                              res.is_making_u_turn.value())
          << " |fo_av_going_straight " << fo_av_going_straight
          << " |is_crossing_straight_lane "
          << static_cast<int>(res.is_crossing_straight_lane.has_value() &&
                              res.is_crossing_straight_lane.value());

  const ad_e2e::planning::LaneConstPtr lane_info =
      path_semantics[av_mean_idx].lane_info;
  if (!lane_info->IsVirtual()) {
    VLOG(2) << "[speed finder] st id: " << st_boundary->id()
            << " not in intersection, lane_id" << lane_info->id();
    return false;
  }

  VLOG(2) << "[speed finder] st id: " << st_boundary->id() << " | obj type "
          << st_boundary->object_type() << " | obj intention "
          << st_traj.trajectory().intention();
  switch (path_semantics[av_mean_idx].lane_semantic) {
    case LaneSemantic::INTERSECTION_LEFT_TURN: {
      if (st_traj.trajectory().intention() !=
              TrajectoryIntention::INTENTION_PARALLEL &&
          st_boundary->object_type() == StBoundaryProto::VEHICLE) {
        VLOG(2) << "[speed finder] st id: " << st_boundary->id()
                << " not straight";
        break;
      }
      bool is_protected = false;
      for (const auto& in_coming_id : lane_info->pre_lane_ids()) {
        const auto in_lane_ptr = psmm.FindCurveLaneByIdOrNull(in_coming_id);
        if (in_lane_ptr && (!in_lane_ptr->junction_id().empty())) {
          is_protected = true;
          break;
        }
      }
      VLOG(2) << "[speed finder] st id: " << st_boundary->id()
              << " |is_protected " << is_protected;
      if (is_protected &&
          st_boundary->object_type() == StBoundaryProto::VEHICLE) {
        res.source = StOverlapMetaProto::LANE_CROSS;
        res.priority = StOverlapMetaProto::EQUAL;
        res.priority_reason = "EQUAL priority for AV protected left turn";
        res.is_unprotected_left_turn = false;
      } else {
        res.source = StOverlapMetaProto::LANE_CROSS;
        res.priority = StOverlapMetaProto::LOW;
        res.priority_reason = "LOW priority for AV unprotected left turn";
        res.is_unprotected_left_turn = true;
      }
      return true;
    }
    case LaneSemantic::INTERSECTION_RIGHT_TURN: {
      res.priority = StOverlapMetaProto::LOW;
      if (st_boundary->object_type() == StBoundaryProto::VEHICLE) {
        res.source = StOverlapMetaProto::LANE_MERGE;
      }
      res.priority_reason = "LOW priority for AV right turn";
      return true;
    }
    case LaneSemantic::INTERSECTION_STRAIGHT: {
      if (st_boundary->object_type() == StBoundaryProto::VEHICLE) {
        res.priority = StOverlapMetaProto::HIGH;
        if (st_traj.trajectory().intention() ==
            TrajectoryIntention::INTENTION_TURN_LEFT) {
          res.source = StOverlapMetaProto::LANE_CROSS;
          res.priority_reason = "HIGH priority for AV straight to left";
        } else if (st_traj.trajectory().intention() ==
                   TrajectoryIntention::INTENTION_TURN_RIGHT) {
          res.source = StOverlapMetaProto::LANE_MERGE;
          res.priority_reason = "HIGH priority for AV straight to right";
        } else if (st_traj.trajectory().intention() ==
                       TrajectoryIntention::INTENTION_PARALLEL &&
                   lc_stage == LaneChangeStage::LCS_NONE) {
          return false;
        } else {
          return false;
        }
      } else if (st_boundary->object_type() == StBoundaryProto::CYCLIST) {
        res.priority = StOverlapMetaProto::LOW;
        res.source = StOverlapMetaProto::LANE_CROSS;
        res.priority_reason = "HIGH priority for AV to cyclist";
      }
      return true;
    }
    case LaneSemantic::INTERSECTION_UTURN:
    case LaneSemantic::ROAD:
    case LaneSemantic::NONE:
      VLOG(2) << "[speed finder] id: " << st_boundary->id()
              << " ego lane_semantic:"
              << static_cast<int>(path_semantics[av_mean_idx].lane_semantic);
      break;
  }
  return false;
}

}  // namespace planning
}  // namespace e2e_noa
