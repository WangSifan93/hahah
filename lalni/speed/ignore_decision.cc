#include "speed/ignore_decision.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_common.h"
#include "math/geometry/polygon2d.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "speed/decision/pre_brake_util.h"
#include "speed/overlap_info.h"
#include "speed/speed_planning_util.h"
#include "speed/st_boundary.h"
#include "speed/st_point.h"
#include "speed_planning.pb.h"
#include "trajectory_point.pb.h"
#include "util/perception_util.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa {
namespace planning {
namespace {

constexpr double kEpsilon = 0.01;

bool IsCollisionHead(const VehicleShapeBasePtr& av_shape,
                     const Polygon2d& obj_contour, const PathPoint& path_point,
                     const VehicleGeometryParamsProto& vehicle_geometry_params,
                     double lat_buffer, double lon_buffer) {
  const Polygon2d av_polygon = Polygon2d(
      av_shape->GetCornersWithBufferCounterClockwise(lat_buffer, lon_buffer),
      true);
  Polygon2d overlap_polygon;
  if (!av_polygon.ComputeOverlap(obj_contour, &overlap_polygon)) {
    return false;
  } else {
    const Vec2d av_tangent = Vec2d::FastUnitFromAngle(path_point.theta());
    Vec2d front, back;
    overlap_polygon.ExtremePoints(av_tangent, &back, &front);
    constexpr double kVehicleHeadRatio = 0.25;
    const double head_lower_limit =
        vehicle_geometry_params.front_edge_to_center() -
        vehicle_geometry_params.length() * kVehicleHeadRatio + lon_buffer;

    if ((front - Vec2d(path_point.x(), path_point.y())).dot(av_tangent) >
        head_lower_limit) {
      return true;
    }
  }
  return false;
}

bool IsCollisionMirrors(
    const VehicleShapeBasePtr& av_shape, const Polygon2d& obj_contour,
    const SpacetimeObjectTrajectory& st_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const double buffer = st_traj.required_lateral_gap();
  const auto [min_mirror_height, max_mirror_height] =
      ComputeMinMaxMirrorAverageHeight(vehicle_geometry_params);
  const bool consider_mirrors =
      IsConsiderMirrorObject(st_traj.planner_object().object_proto(),
                             min_mirror_height, max_mirror_height);
  return consider_mirrors &&
         (av_shape->LeftMirrorHasOverlapWithBuffer(obj_contour, buffer) ||
          av_shape->RightMirrorHasOverlapWithBuffer(obj_contour, buffer));
}

bool IgnoreLeftTurnCrossInteractionStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PathPoint& current_path_point, const StBoundary& st_boundary,
    double current_v) {
  const bool is_left_turn_to_staight_vehicle =
      (st_boundary.obj_scenario_info().relationship == Relationship::Cross ||
       st_boundary.obj_scenario_info().relationship ==
           Relationship::OnComing) &&
      st_boundary.obj_scenario_info().interaction_zone ==
          InteractionZone::TurnLeft;
  if (!is_left_turn_to_staight_vehicle) return false;

  const double buffer = 0.0;
  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  bool is_collied_with_main_body = false;
  const Polygon2d av_polygon = Polygon2d(
      av_shapes[0]->GetCornersWithBufferCounterClockwise(buffer, buffer), true);

  for (const auto& state : traj->states()) {
    const auto& obj_contour = state.contour;
    Polygon2d overlap_polygon;
    if (av_polygon.ComputeOverlap(obj_contour, &overlap_polygon)) {
      is_collied_with_main_body = true;
      break;
    }
  }
  constexpr double kSpeedThres = 2.5;
  if (is_collied_with_main_body && current_v < kSpeedThres) {
    return true;
  }

  return false;
}

bool IgnoreHitAvCurrentPositionStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PathPoint& current_path_point, const StBoundary& st_boundary,
    double current_v) {
  const double first_overlap_time = st_boundary.bottom_left_point().t();
  constexpr double kObjMinVel = Kph2Mps(5.0);
  constexpr double kAvMinVel = Kph2Mps(2.0);
  constexpr double kTimeThreshold = 2.0;

  if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN ||
      st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    CHECK(st_boundary.traj_id().has_value());
    const auto& traj_id = *st_boundary.traj_id();
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
    double obj_vel = traj->planner_object().pose().v();

    if (current_v < kAvMinVel && obj_vel <= kObjMinVel &&
        first_overlap_time <= kTimeThreshold) {
      return false;
    }
  }

  const bool is_merge_scenario =
      st_boundary.obj_scenario_info().relationship == Relationship::Merge;
  const bool is_object_at_left_side =
      st_boundary.obj_scenario_info().bearing == Bearing::Left;
  const bool is_right_interaction =
      st_boundary.obj_scenario_info().interaction_zone ==
      InteractionZone::TurnRight;
  if (is_merge_scenario && is_object_at_left_side && is_right_interaction) {
    return false;
  }

  if (st_boundary.bottom_left_point().s() <= kEpsilon &&
      first_overlap_time >= kEpsilon) {
    return true;
  } else {
    CHECK(st_boundary.traj_id().has_value());
    const auto& traj_id = *st_boundary.traj_id();
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

    const auto& overlap_infos = st_boundary.overlap_infos();
    CHECK(!overlap_infos.empty());
    constexpr double kCheckTime = 0.5;
    const double check_time_limit = first_overlap_time + kCheckTime;
    for (auto& overlap_info : overlap_infos) {
      if (overlap_info.time > check_time_limit) {
        break;
      }
      if (overlap_info.av_start_idx == 0) {
        const bool is_stationary_object = traj->is_stationary();
        const double buffer =
            is_stationary_object ? 0.0 : traj->required_lateral_gap();
        const auto& obj_contour = traj->states()[overlap_info.obj_idx].contour;
        if (is_stationary_object) {
          return av_shapes[0]->MainBodyHasOverlapWithBuffer(obj_contour, buffer,
                                                            buffer) &&
                 !IsCollisionHead(av_shapes[0], obj_contour, current_path_point,
                                  vehicle_geometry_params, buffer, buffer);
        } else {
          return !IsCollisionMirrors(av_shapes[0], obj_contour, *traj,
                                     vehicle_geometry_params) &&
                 !IsCollisionHead(av_shapes[0], obj_contour, current_path_point,
                                  vehicle_geometry_params, buffer, buffer);
        }
      }
    }
  }
  return false;
}

bool IsBackStaightCutInVehicle(const SpacetimeTrajectoryManager& st_traj_mgr,
                               const PathPointSemantic& current_path_semantic,
                               const StBoundary& st_boundary) {
  constexpr double kTtcThreshold = 3.0;
  constexpr double kThetaThreshold = 0.34;
  const auto current_lane_semantic = current_path_semantic.lane_semantic;
  if (current_lane_semantic != LaneSemantic::ROAD &&
      current_lane_semantic != LaneSemantic::INTERSECTION_STRAIGHT) {
    return false;
  }

  if (!st_boundary.overlap_meta().has_value()) {
    return false;
  }

  const auto& overlap_pattern = st_boundary.overlap_meta()->pattern();
  if (overlap_pattern != StOverlapMetaProto::ENTER &&
      overlap_pattern != StOverlapMetaProto::CROSS &&
      overlap_pattern != StOverlapMetaProto::INTERFERE) {
    return false;
  }

  if (st_boundary.overlap_meta()->source() == StOverlapMetaProto::AV_CUTIN) {
    return false;
  }

  if (st_boundary.object_type() != StBoundaryProto::VEHICLE) {
    return false;
  }

  if (st_boundary.bottom_left_point().s() <= 0.0) {
    return false;
  }

  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto first_point_heading = traj->states().front().traj_point->theta();
  const auto end_point_heading = traj->states().back().traj_point->theta();
  if (std::abs(NormalizeAngle(first_point_heading - end_point_heading)) >
      kThetaThreshold) {
    return false;
  }

  return true;
}

double CalObjDisToAv(const SpacetimeTrajectoryManager& st_traj_mgr,
                     const StBoundary& st_boundary,
                     const PathPoint& current_path_point) {
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto& obj_contour = traj->contour();
  const Vec2d av_heading_dir =
      Vec2d::FastUnitFromAngle(current_path_point.theta());
  const Vec2d av_pos = Vec2d(current_path_point.x(), current_path_point.y());
  Vec2d front_most, back_most;
  obj_contour.ExtremePoints(av_heading_dir, &back_most, &front_most);
  return (front_most - av_pos).Dot(av_heading_dir);
}

bool IgnoreBackCutInStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PathPointSemantic& current_path_semantic,
    const PathPoint& current_path_point, bool on_left_turn_waiting_lane,
    const StBoundary& st_boundary, Merge_Direction merge_direction,
    double obj_dl, double obj_ds, double av_obj_ttc,
    SplitTopology split_topology) {
  constexpr double kTtcThreshold = 0.0;
  constexpr double kTtcThresholdMergeScene = 2.5;

#if 0
  if (st_boundary.object_id().has_value()){
    std::cout << " back_cut_in " << " object_id " << st_boundary.object_id().value()
              << " dl " << obj_dl << " merge_direction " << merge_direction
              << " ds " << obj_ds << " av_obj_ttc " << av_obj_ttc << " split_topo " << split_topology
              << std::endl;
  }
#endif

  if (!st_boundary.overlap_meta().has_value()) return false;
  const auto current_lane_semantic = current_path_semantic.lane_semantic;
  if (current_lane_semantic == LaneSemantic::NONE ||
      current_lane_semantic == LaneSemantic::INTERSECTION_UTURN) {
    return false;
  }
  const bool is_back_straight_vehicle = IsBackStaightCutInVehicle(
      st_traj_mgr, current_path_semantic, st_boundary);
  if (!is_back_straight_vehicle &&
      st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::OBJECT_CUTIN &&
      st_boundary.overlap_meta()->source() != StOverlapMetaProto::LANE_MERGE &&
      st_boundary.overlap_meta()->source() != StOverlapMetaProto::LANE_CROSS) {
    return false;
  }

  if (MERGE_LEFT == merge_direction && obj_dl > 0) {
    return false;
  } else if (MERGE_RIGHT == merge_direction && obj_dl < 0) {
    return false;
  }
  double ttc_threshold;

  if (MERGE_LEFT == merge_direction || MERGE_RIGHT == merge_direction) {
    ttc_threshold = kTtcThresholdMergeScene;
  } else {
    ttc_threshold = kTtcThreshold;
  }

  if (obj_ds > 0) return false;

  if (av_obj_ttc < ttc_threshold) return false;
  double front_most_projection_distance;
  if (st_boundary.overlap_meta()->has_front_most_projection_distance()) {
    front_most_projection_distance =
        st_boundary.overlap_meta()->front_most_projection_distance();
  } else {
    if (is_back_straight_vehicle) {
      front_most_projection_distance =
          CalObjDisToAv(st_traj_mgr, st_boundary, current_path_point);
    } else {
      return false;
    }
  }

  constexpr double kDriverAwarenessFactor = 0.5;
  const double driver_awareness_area =
      vehicle_geometry_params.front_edge_to_center() -
      vehicle_geometry_params.length() * kDriverAwarenessFactor;
  const bool behind_current_path_point_center =
      front_most_projection_distance < driver_awareness_area;

  if (current_lane_semantic == LaneSemantic::ROAD ||
      current_lane_semantic == LaneSemantic::INTERSECTION_STRAIGHT) {
    if (!is_back_straight_vehicle && st_boundary.overlap_meta()->source() !=
                                         StOverlapMetaProto::OBJECT_CUTIN) {
      return false;
    }
    if (merge_direction == MERGE_LEFT || merge_direction == MERGE_RIGHT) {
      return false;
    }

    if (split_topology == SplitTopology::TOPOLOGY_SPLIT_LEFT ||
        split_topology == SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
      return false;
    }

    return behind_current_path_point_center;
  } else if (current_lane_semantic == LaneSemantic::INTERSECTION_LEFT_TURN ||
             current_lane_semantic == LaneSemantic::INTERSECTION_RIGHT_TURN) {
    if ((st_boundary.overlap_meta()->source() !=
             StOverlapMetaProto::OBJECT_CUTIN &&
         st_boundary.overlap_meta()->priority() == StOverlapMetaProto::LOW) ||
        (st_boundary.overlap_meta()->has_obj_lane_direction() &&
         st_boundary.overlap_meta()->obj_lane_direction() ==
             e2e_noa::StOverlapMetaProto::NO_TURN)) {
      return false;
    }
    if (behind_current_path_point_center) return true;

    if (on_left_turn_waiting_lane) {
      const bool behind_current_path_point_front =
          front_most_projection_distance <
          vehicle_geometry_params.front_edge_to_center();
      if (behind_current_path_point_front) {
        return true;
      }
    }

    return false;
  }

  LOG(FATAL) << "Should not be here.";
  return false;
}

#define DEBUG_TURN_LEFT_BACK_IGNORE (0)

bool IgnoreTurnLeftBackCutInStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PathPoint& current_path_point, const StBoundary& st_boundary,
    double obj_dl, const std::vector<DrivingProcess>& driving_process_seq,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetPolygon& obj_frenet_polygon) {
  constexpr double kDeltaTheta = ad_e2e::planning::Constants::DEG2RAD * 5.0;

  if (driving_process_seq.size() == 0) return false;
  if (!st_boundary.object_id().has_value()) return false;

#if DEBUG_TURN_LEFT_BACK_IGNORE
  std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
            << " ]:  obj_max_s " << obj_frenet_polygon.s_max << " dl " << obj_dl
            << " ego_front_to_center "
            << vehicle_geometry_params.front_edge_to_center()
            << " ego_back_to_center "
            << vehicle_geometry_params.back_edge_to_center() << std::endl;
#endif

  bool is_av_turn_left = false;
  for (int i = 0; i < 2 && i < driving_process_seq.size(); i++) {
    if (driving_process_seq[i].zone_type == InteractionZone::TurnLeft) {
      is_av_turn_left = true;
      break;
    }
  }

  const PlannerObject* obj =
      st_traj_mgr.FindObjectByObjectId(st_boundary.object_id().value());

  if (nullptr == obj) return false;

  double delta_theta = std::fabs(ad_e2e::planning::math::AngleDiff(
      current_path_point.theta(), obj->pose().theta()));

#if DEBUG_TURN_LEFT_BACK_IGNORE
  std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
            << " ]: is_av_turn_left " << is_av_turn_left << " delta_theta "
            << ad_e2e::planning::Constants::RAD2DEG * delta_theta << std::endl;
#endif

  if (!is_av_turn_left) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " F]: reason:  is_av_not_turn_left " << std::endl;
    std::cout << std::endl;
#endif
    return false;
  }

  const double ego_front_to_center =
      vehicle_geometry_params.front_edge_to_center();
  const double ego_back_to_center =
      vehicle_geometry_params.back_edge_to_center();

  double ego_half_s =
      ego_front_to_center - vehicle_geometry_params.length() * 0.25;

  if (obj_frenet_polygon.s_max > ego_half_s) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " F]: reason:  s_max > ego_half_s " << std::endl;
    std::cout << std::endl;
#endif
    return false;
  }

  bool is_agent_back = false;
  bool is_agent_back_side = false;

  if (obj_frenet_polygon.s_max < -ego_back_to_center) {
    is_agent_back = true;
  } else if (obj_frenet_polygon.s_max <= ego_half_s) {
    is_agent_back_side = true;
  }

  if (obj_dl >= 0.0) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " F]: reason:  obj_dl >= 0.0 " << std::endl;
    std::cout << std::endl;
#endif
    return false;
  }

#if DEBUG_TURN_LEFT_BACK_IGNORE
  std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
            << " ]: is_agent_back " << is_agent_back << " is_agent_back_side "
            << is_agent_back_side << std::endl;
#endif
  if (is_agent_back_side && delta_theta < kDeltaTheta) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " T]: reason: back_side ignore " << std::endl;
    std::cout << std::endl;
#endif
    return true;
  }

  if (is_agent_back) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " T]: reason: back ignore " << std::endl;
    std::cout << std::endl;
#endif
    return true;
  }

  return false;
}

bool IgnoreOnRoadParallelCutInStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const FrenetBox& av_frenet_box, bool av_in_plan_passage_lane,
    const PlanPassage& plan_passage, double current_v,
    const PlannerSemanticMapManager& psmm,
    const std::vector<DrivingProcess>& driving_process_seq,
    const std::vector<mapping::ElementId>& upcoming_merge_lane_ids,
    bool ignore_late_parallel_cut_in_vehicle, const StBoundary& st_boundary) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& fo_info = overlap_infos.front();

  if (path_semantics[fo_info.av_start_idx].lane_semantic !=
      LaneSemantic::ROAD) {
    return false;
  }
  if (!av_in_plan_passage_lane) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto obj_frenet_box_or =
      plan_passage.QueryFrenetBoxAtContour(traj->contour());
  if (!obj_frenet_box_or.ok()) {
    return false;
  }
  const auto& obj_frenet_box = *obj_frenet_box_or;

  constexpr double kLateCutInTimeThres = 3.5;
  if (!ignore_late_parallel_cut_in_vehicle ||
      st_boundary.object_type() != StBoundaryProto::VEHICLE ||
      st_boundary.min_t() < kLateCutInTimeThres) {
    const double object_length = obj_frenet_box.s_max - obj_frenet_box.s_min;

    const PiecewiseLinearFunction rel_speed_surpass_dist_plf(
        std::vector<double>({-2.0, 0.0, 2.0}),
        std::vector<double>({0.8 * object_length, 0.3 * object_length, 0.0}));
    constexpr double kRearSRangeOffset = 0.5;
    if (obj_frenet_box.s_max < av_frenet_box.s_min - kRearSRangeOffset ||
        obj_frenet_box.s_max >
            av_frenet_box.s_max +
                rel_speed_surpass_dist_plf(traj->planner_object().pose().v() -
                                           current_v)) {
      return false;
    }
  }

  if (!st_boundary.overlap_meta()->has_theta_diff()) return false;
  const auto& theta_diff = st_boundary.overlap_meta()->theta_diff();
  constexpr double kParallelHeadingDiff = 0.17453292519943295;
  if (std::abs(theta_diff) > kParallelHeadingDiff) {
    return false;
  }

  const double obj_s_mean = 0.5 * (obj_frenet_box.s_min + obj_frenet_box.s_max);
  const auto obj_s_min_boundary =
      plan_passage.QueryEnclosingLaneBoundariesAtS(obj_frenet_box.s_min);
  const auto obj_s_mean_boundary =
      plan_passage.QueryEnclosingLaneBoundariesAtS(obj_s_mean);
  const auto obj_s_max_boundary =
      plan_passage.QueryEnclosingLaneBoundariesAtS(obj_frenet_box.s_max);
  constexpr double kOutOfLaneOffset = 0.8;
  const double obj_l_min = obj_frenet_box.l_min + kOutOfLaneOffset;
  const double obj_l_max = obj_frenet_box.l_max - kOutOfLaneOffset;
  const bool left_of_left_boundary =
      obj_s_min_boundary.left.has_value() &&
      obj_s_mean_boundary.left.has_value() &&
      obj_s_max_boundary.left.has_value() &&
      obj_l_min > obj_s_min_boundary.left->lat_offset &&
      obj_l_min > obj_s_mean_boundary.left->lat_offset &&
      obj_l_min > obj_s_max_boundary.left->lat_offset;
  const bool right_of_right_boundary =
      obj_s_min_boundary.right.has_value() &&
      obj_s_mean_boundary.right.has_value() &&
      obj_s_max_boundary.right.has_value() &&
      obj_l_max < obj_s_min_boundary.right->lat_offset &&
      obj_l_max < obj_s_mean_boundary.right->lat_offset &&
      obj_l_max < obj_s_max_boundary.right->lat_offset;
  const bool out_of_lane = left_of_left_boundary || right_of_right_boundary;
  if (!out_of_lane) return false;

  double fraction = 0.0;
  double min_dist = 0.0;
  bool obj_near_upcoming_merge_lane = false;
  constexpr double kNearLaneDist = 2.0;
  for (const auto& merge_lane_id : upcoming_merge_lane_ids) {
    if (psmm.GetLaneProjection(traj->pose().pos(), merge_lane_id, &fraction,
                               nullptr, &min_dist) &&
        min_dist < kNearLaneDist) {
      obj_near_upcoming_merge_lane = true;
      break;
    }
  }
  if (obj_near_upcoming_merge_lane) return false;

  for (const auto& driving_process : driving_process_seq) {
    if ((driving_process.merge_topology ==
         e2e_noa::planning::MergeTopology::TOPOLOGY_MERGE_LEFT) ||
        (driving_process.merge_topology ==
         e2e_noa::planning::MergeTopology::TOPOLOGY_MERGE_RIGHT)) {
      return false;
    }
  }

  return true;
}

bool IgnoreReverseDrivingStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, const StBoundary& st_boundary) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::OBJECT_CUTIN &&
      st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::UNKNOWN_SOURCE) {
    return false;
  }
  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE ||
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  if (traj->planner_object().pose().v() >= -kEpsilon) {
    return false;
  }
  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      traj->states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kParallelDrivingThreshold = M_PI / 6.0;
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) >
      kParallelDrivingThreshold) {
    return false;
  }
  return true;
}

bool IgnoreUncomfortableBrakeOncomingStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, const StBoundary& st_boundary,
    const PlanPassage& plan_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }
  constexpr double kMinTimeLimit = 0.5;
  if (st_boundary.bottom_left_point().t() < kMinTimeLimit) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      traj->states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kOnComingThreshold = 2.9670597283903604;
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) <
      kOnComingThreshold) {
    return false;
  }

  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (st_boundary.bottom_right_point().s() > const_speed_s) {
    return false;
  }
  const double estimated_av_decel =
      2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
      Sqr(st_boundary.bottom_right_point().t());
  constexpr double kUncomfortableDecel = 0.3;
  if (estimated_av_decel < kUncomfortableDecel) {
    return false;
  }
  if (IsConsiderOncomingObs(st_traj_mgr, st_boundary, plan_passage,
                            vehicle_geometry_params, current_v, path)) {
    return false;
  }
  return true;
}

bool IgnoreOncomingStBoundaryWithoutObviousCutInIntention(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const PlanPassage& plan_passage, double current_v,
    const StBoundary& st_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const DiscretizedPath& path) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& fo_info = overlap_infos.front();

  const auto fo_lane_semantic =
      path_semantics[fo_info.av_start_idx].lane_semantic;
  if (fo_lane_semantic != LaneSemantic::ROAD &&
      fo_lane_semantic != LaneSemantic::INTERSECTION_STRAIGHT) {
    return false;
  }

  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }
  constexpr double kMinTimeLimit = 0.1;
  if (st_boundary.bottom_left_point().t() < kMinTimeLimit) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

  const Vec2d& obj_pos = traj->planner_object().pose().pos();
  const auto obj_frenet_coord = plan_passage.QueryFrenetCoordinateAt(obj_pos);
  if (!obj_frenet_coord.ok()) {
    return false;
  }
  const auto obj_frenet_theta =
      plan_passage.QueryTangentAngleAtS(obj_frenet_coord->s);
  if (!obj_frenet_theta.ok()) {
    return false;
  }
  const double ref_theta = *obj_frenet_theta;
  const double theta_diff =
      NormalizeAngle(traj->planner_object().pose().theta() - ref_theta);

  constexpr double kOncomingThresholdVehicle = 3.0543261909900768;
  constexpr double kOncomingThresholdCyclist = 2.9670597283903604;
  const double oncoming_threshold =
      st_boundary.object_type() == StBoundaryProto::VEHICLE
          ? kOncomingThresholdVehicle
          : kOncomingThresholdCyclist;
  if (std::abs(theta_diff) < oncoming_threshold) {
    return false;
  }
  if (IsConsiderOncomingObs(st_traj_mgr, st_boundary, plan_passage,
                            vehicle_geometry_params, current_v, path)) {
    return false;
  }

  constexpr double kFollowDistance = 3.0;
  const double brake_dis = st_boundary.min_s() - kFollowDistance;
  const double brake_time = st_boundary.bottom_right_point().t();
  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (brake_dis < kEpsilon) {
    return true;
  }
  if (brake_dis > const_speed_s) {
    return false;
  }
  const bool brake_to_stop =
      0.5 * brake_dis / std::max(current_v, kEpsilon) < brake_time;
  const double estimated_av_decel =
      brake_to_stop
          ? 0.5 * Sqr(current_v) / brake_dis
          : 2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
                Sqr(st_boundary.bottom_right_point().t());
  constexpr double kUncomfortableDecel = 1.0;
  if (estimated_av_decel < kUncomfortableDecel) {
    return false;
  }
  return true;
}

std::optional<VtSpeedLimit> MakeOncomingPreBrakeDecisionForStBoundary(
    double current_v, double max_v, double time_step, int step_num,
    const StBoundaryWithDecision& st_boundary_wd) {
  constexpr double kMinTimeLimit = 0.1;
  constexpr double kMaxTimeLimit = 3.5;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  const double pre_brake_time = st_boundary.bottom_left_point().t();
  if (pre_brake_time < kMinTimeLimit || pre_brake_time > kMaxTimeLimit) {
    return std::nullopt;
  }

  constexpr double kEmergencyStopAccel = -3.0;
  const double s_limit_lower = current_v * pre_brake_time +
                               0.5 * kEmergencyStopAccel * Sqr(pre_brake_time);
  if (st_boundary.bottom_left_point().s() < s_limit_lower) {
    return std::nullopt;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  constexpr double kMildDecel = -0.6;
  constexpr double kMinVel = 4.0;
  constexpr double kExtraTime = 1.0;
  const auto info = absl::StrCat("Pre brake for oncoming object ", *traj_id);
  return GenerateConstAccSpeedLimit(0.0, pre_brake_time + kExtraTime, current_v,
                                    kMinVel, max_v, kMildDecel, time_step,
                                    step_num, info);
}

#define DEBUG_PARALLEL_MERGE (0)

bool IgnoreBackParallelMergeAgent(
    const StBoundary& st_boundary,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlanPassage& plan_passage, const FrenetBox& av_frenet_box,
    const DiscretizedPath& path, LaneChangeStage lc_stage,
    Merge_Direction merge_direction, double obj_dl, double av_obj_ttc,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetPolygon& obj_frenet_polygon, SplitTopology split_topo) {
  constexpr double kConsiderMaxPathLength = 30.0;
  constexpr double kConsiderAvLengthRatio = 1.0;
  constexpr double kMaxCurvature = 0.002;
  constexpr double kTtcThreshold = 0.0;
  constexpr double kTtcThresholdMergeScene = 2.5;

#if DEBUG_PARALLEL_MERGE
  if (st_boundary.object_id().has_value()) {
    std::cout << "back_parallel: object_id " << st_boundary.object_id().value()
              << " obj_max_s " << obj_frenet_polygon.s_max << " dl " << obj_dl
              << " merge_direction " << merge_direction << " av_obj_ttc "
              << av_obj_ttc << std::endl;
  }
#endif

  if (LaneChangeStage::LCS_NONE != lc_stage) return false;

  if (split_topo == SplitTopology::TOPOLOGY_SPLIT_LEFT ||
      split_topo == SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
    return false;
  }

  if (MERGE_LEFT == merge_direction && obj_dl > 0) {
    return false;
  } else if (MERGE_RIGHT == merge_direction && obj_dl < 0) {
    return false;
  }

  double ttc_threshold;

  if (MERGE_LEFT == merge_direction || MERGE_RIGHT == merge_direction) {
    ttc_threshold = kTtcThresholdMergeScene;
  } else {
    ttc_threshold = kTtcThreshold;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  double av_frenet_box_max_s = vehicle_geometry_params.front_edge_to_center();

#if DEBUG_PARALLEL_MERGE
  std::cout << " av_max_s " << av_frenet_box_max_s << " traj_intention "
            << traj->trajectory().intention() << std::endl;
#endif

  if (TrajectoryIntention::INTENTION_PARALLEL !=
      traj->trajectory().intention()) {
    return false;
  }

  double ego_length = vehicle_geometry_params.length();

  if ((av_frenet_box_max_s - obj_frenet_polygon.s_max) <
      kConsiderAvLengthRatio * ego_length) {
    return false;
  }

  if (av_obj_ttc < ttc_threshold) return false;

  double average_curvature = 0.0;
  int32_t point_nums = 0;

  for (const auto& point : path) {
    average_curvature += std::abs(point.kappa());
    point_nums++;
    if (point.s() > kConsiderMaxPathLength) {
      break;
    }
  }
  if (0 != point_nums) {
    average_curvature /= point_nums;
  }

#if DEBUG_PARALLEL_MERGE
  std::cout << " average_curvature " << average_curvature << std::endl;
#endif
  if (kMaxCurvature < average_curvature) return false;

  return true;
}

std::optional<double> GetPassageOccupyPercentage(
    const PlanPassage& plan_passage, const FrenetBox& av_frenet_box,
    double* occupy_width) {
  const auto av_s_min_boundary =
      plan_passage.QueryEnclosingLaneBoundariesAtS(av_frenet_box.s_min);
  const auto av_s_max_boundary =
      plan_passage.QueryEnclosingLaneBoundariesAtS(av_frenet_box.s_max);
  if (!av_s_min_boundary.left.has_value() ||
      !av_s_max_boundary.left.has_value() ||
      !av_s_min_boundary.right.has_value() ||
      !av_s_max_boundary.right.has_value()) {
    return std::nullopt;
  }

  bool av_in_plan_passage_lane =
      (av_frenet_box.l_max < av_s_min_boundary.left->lat_offset) &&
      (av_frenet_box.l_max < av_s_max_boundary.left->lat_offset) &&
      (av_frenet_box.l_min > av_s_min_boundary.right->lat_offset) &&
      (av_frenet_box.l_min > av_s_max_boundary.right->lat_offset);

  bool av_out_of_plan_passage_lane =
      (av_frenet_box.l_max < av_s_min_boundary.right->lat_offset) &&
      (av_frenet_box.l_max < av_s_max_boundary.right->lat_offset) &&
      (av_frenet_box.l_min > av_s_min_boundary.left->lat_offset) &&
      (av_frenet_box.l_min > av_s_max_boundary.left->lat_offset);
  if (av_in_plan_passage_lane) {
    return 1.0;
  } else if (av_out_of_plan_passage_lane) {
    return 0.0;
  } else {
    const double ego_width = av_frenet_box.l_max - av_frenet_box.l_min;
    const double left_bound_mean_l = (av_s_max_boundary.left->lat_offset +
                                      av_s_min_boundary.left->lat_offset) /
                                     2.0;
    const double right_bound_mean_l = (av_s_max_boundary.right->lat_offset +
                                       av_s_min_boundary.right->lat_offset) /
                                      2.0;
    if (av_frenet_box.center_l() > kEpsilon) {
      *occupy_width = left_bound_mean_l - av_frenet_box.l_min;
    } else {
      *occupy_width = av_frenet_box.l_max - right_bound_mean_l;
    }
    return *occupy_width / ego_width;
  }
  return std::nullopt;
}

bool IgnoreBackWhenAVOccupyTargetLane(
    const StBoundaryWithDecision& st_boundary_wd,
    const PlanPassage& plan_passage, const FrenetBox& av_frenet_box,
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto& st_boundary = st_boundary_wd.st_boundary();
  constexpr double kOccupyThreshold = 0.5;
  constexpr double kDefaultLatSpeed = 0.2;
  if (!st_boundary->overlap_meta().has_value()) return false;
  if (st_boundary->overlap_meta()->source() == StOverlapMetaProto::AV_CUTIN &&
      st_boundary_wd.ds() < -kEpsilon) {
    bool is_av_occupy_target_lane = false;
    double occupy_width = 0.0;
    const auto& occupy_percentage =
        GetPassageOccupyPercentage(plan_passage, av_frenet_box, &occupy_width);
    if (occupy_percentage.has_value() && occupy_percentage.value() > kEpsilon) {
      double ttc = st_boundary->min_t();
      is_av_occupy_target_lane = ((occupy_width > kOccupyThreshold) ||
                                  (occupy_width + ttc * kDefaultLatSpeed >
                                   vehicle_geometry_params.width() * 0.5));
    }
    return is_av_occupy_target_lane;
  }
  return false;
}

double CalcTtcConsiderRelativeAcc(double ds, double av_vel, double av_acc,
                                  const SpacetimeTrajectoryManager& st_traj_mgr,
                                  const StBoundary& st_boundary,
                                  bool consider_acc) {
  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id().value();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  double obj_vel = traj->planner_object().pose().v();
  double obj_acc = traj->planner_object().pose().a();
  double relative_acc = obj_acc - av_acc;
  double relative_vel = std::fabs(obj_vel - av_vel);

  double ttc = 0.0;
  if (ds >= 0) {
    relative_acc = -relative_acc;
  }
  ds = std::fabs(ds);

  double stop_acc = Sqr(relative_vel) / (2 * ds);

  if (!consider_acc) {
    relative_acc = 0.0;
  }

  if (relative_acc <= 0 && std::fabs(relative_acc) > stop_acc) {
    ttc = ds / (relative_vel + kEpsilon);
  } else {
    ttc =
        ds / (std::sqrt(Sqr(relative_vel) + 2 * relative_acc * ds) + kEpsilon);
  }

  return ttc;
}

void MakeIgnoreAndPreBrakeDecisionForStBoundary(
    const SpeedPlanningParamsProto::IgnoreDeciderParamsProto& params,
    const DiscretizedPath& path,
    const std::vector<PathPointSemantic>& path_semantics,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlanPassage& plan_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double current_v,
    double max_v, double time_step, double trajectory_steps,
    const PlannerSemanticMapManager& psmm,

    bool on_left_turn_waiting_lane,
    const std::optional<FrenetBox>& av_frenet_box, bool av_in_plan_passage_lane,
    const std::vector<mapping::ElementId>& upcoming_merge_lane_ids,
    StBoundaryWithDecision* st_boundary_wd,
    std::optional<VtSpeedLimit>* speed_limit, LaneChangeStage lc_stage,
    Merge_Direction merge_direction, double current_a,
    const std::vector<DrivingProcess>& driving_process_seq,
    SplitTopology split_topo, const bool is_narrow_near_large_vehicle,
    bool* const is_left_turn_ignore) {
  CHECK_NOTNULL(st_boundary_wd);

  if (st_boundary_wd->decision_type() != StBoundaryProto::UNKNOWN) {
    return;
  }
  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();
  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return;
  }

  double av_obj_ttc =
      CalcTtcConsiderRelativeAcc(st_boundary_wd->ds(), current_v, current_a,
                                 st_traj_mgr, st_boundary, false);

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         StBoundaryWithDecision* st_boundary_wd) {
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd->set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
        st_boundary_wd->set_ignore_reason(ignore_reason);
        st_boundary_wd->set_decision_info(decision_info);
      };

  if (IgnoreHitAvCurrentPositionStBoundary(st_traj_mgr, vehicle_geometry_params,
                                           av_shapes, path.front(), st_boundary,
                                           current_v)) {
    make_ignore_decision("ignore hit current AV position",
                         StBoundaryProto::HIT_CURRENT_POS, st_boundary_wd);
    return;
  }

  if (IgnoreLeftTurnCrossInteractionStBoundary(
          st_traj_mgr, vehicle_geometry_params, av_shapes, path.front(),
          st_boundary, current_v)) {
    *is_left_turn_ignore = true;

    make_ignore_decision("ignore left turn stop-leading vehicles",
                         StBoundaryProto::LEFT_TURN_STOP, st_boundary_wd);
    return;
  }

  if (st_boundary.is_stationary()) return;

  if (!path_semantics.empty() &&
      IgnoreBackCutInStBoundary(
          st_traj_mgr, vehicle_geometry_params, path_semantics.front(),
          path.front(), on_left_turn_waiting_lane, st_boundary, merge_direction,
          st_boundary_wd->dl(), st_boundary_wd->ds(), av_obj_ttc, split_topo)) {
    make_ignore_decision("ignore back cutin", StBoundaryProto::BACK_CUT_IN,
                         st_boundary_wd);
    return;
  }

  if (!path_semantics.empty() &&
      IgnoreTurnLeftBackCutInStBoundary(
          st_traj_mgr, path.front(), st_boundary, st_boundary_wd->dl(),
          driving_process_seq, vehicle_geometry_params,
          st_boundary_wd->obj_frenet_polygon())) {
    make_ignore_decision("ignore turn_left_back cutin",
                         StBoundaryProto::BACK_CUT_IN, st_boundary_wd);
    return;
  }

  if (!is_narrow_near_large_vehicle && !path_semantics.empty() &&
      av_frenet_box.has_value() &&
      IgnoreOnRoadParallelCutInStBoundary(
          st_traj_mgr, path_semantics, *av_frenet_box, av_in_plan_passage_lane,
          plan_passage, current_v, psmm, driving_process_seq,
          upcoming_merge_lane_ids, params.ignore_late_parallel_cut_in_vehicle(),
          st_boundary)) {
    make_ignore_decision("ignore on-road parallel cutin",
                         StBoundaryProto::PARALLEL_CUT_IN, st_boundary_wd);
    return;
  }

  if (params.ignore_reverse_driving() &&
      IgnoreReverseDrivingStBoundary(st_traj_mgr, path, current_v,
                                     st_boundary)) {
    make_ignore_decision("ignore reverse-driving object",
                         StBoundaryProto::REVERSE_DRIVING, st_boundary_wd);
  }

  if (IgnoreUncomfortableBrakeOncomingStBoundary(st_traj_mgr, path, current_v,
                                                 st_boundary, plan_passage,
                                                 vehicle_geometry_params)) {
    make_ignore_decision("ignore uncomfortable brake oncoming",
                         StBoundaryProto::ONCOMING_OBJECT, st_boundary_wd);
    auto speed_limit_opt = MakeOncomingPreBrakeDecisionForStBoundary(
        current_v, max_v, time_step, trajectory_steps, *st_boundary_wd);
    if (speed_limit_opt.has_value()) {
      if (speed_limit->has_value()) {
        MergeVtSpeedLimit(speed_limit_opt.value(), &speed_limit->value());
      } else {
        *speed_limit = speed_limit_opt;
      }
    }
    return;
  }

  if (!path_semantics.empty() &&
      IgnoreOncomingStBoundaryWithoutObviousCutInIntention(
          st_traj_mgr, path_semantics, plan_passage, current_v, st_boundary,
          vehicle_geometry_params, path)) {
    make_ignore_decision(
        "ignore oncoming objects without obvious cut-in intention",
        StBoundaryProto::ONCOMING_OBJECT, st_boundary_wd);
    const auto speed_limit_opt = MakeOncomingPreBrakeDecisionForStBoundary(
        current_v, max_v, time_step, trajectory_steps, *st_boundary_wd);
    if (speed_limit_opt.has_value()) {
      if (speed_limit->has_value()) {
        MergeVtSpeedLimit(speed_limit_opt.value(), &speed_limit->value());
      } else {
        *speed_limit = speed_limit_opt;
      }
    }
    return;
  }

  if (params.ignore_back_parallel_merge_obj() && av_frenet_box.has_value() &&
      IgnoreBackParallelMergeAgent(
          st_boundary, st_traj_mgr, plan_passage, *av_frenet_box, path,
          lc_stage, merge_direction, st_boundary_wd->dl(), av_obj_ttc,
          vehicle_geometry_params, st_boundary_wd->obj_frenet_polygon(),
          split_topo)) {
    make_ignore_decision("ignore back parallel far merge obj",
                         StBoundaryProto::BACK_PARALLEL_MERGE, st_boundary_wd);
    return;
  }

  if (IgnoreBackWhenAVOccupyTargetLane(*st_boundary_wd, plan_passage,
                                       av_frenet_box.value(),
                                       vehicle_geometry_params)) {
    make_ignore_decision("av cutin occupy enough plan passage lane",
                         StBoundaryProto::BACK_CUT_IN, st_boundary_wd);
  }
}

void MakeIgnoreDecisionForNonNearestStationaryStBoundaries(
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  CHECK_NOTNULL(st_boundaries_wd);

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         StBoundaryWithDecision* st_boundary_wd) {
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd->set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
        st_boundary_wd->set_ignore_reason(ignore_reason);
        st_boundary_wd->set_decision_info(decision_info);
      };

  StBoundaryWithDecision* nearest_stationary_st_boundary = nullptr;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary()) continue;
    if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) continue;
    if (nullptr == nearest_stationary_st_boundary) {
      nearest_stationary_st_boundary = &st_boundary_wd;
    } else if (st_boundary.min_s() <
               nearest_stationary_st_boundary->raw_st_boundary()->min_s()) {
      make_ignore_decision("ignore non-nearest stationary st-boundary",
                           StBoundaryProto::NON_NEAREST_STATIONARY,
                           nearest_stationary_st_boundary);
      nearest_stationary_st_boundary = &st_boundary_wd;
    } else {
      make_ignore_decision("ignore non-nearest stationary st-boundary",
                           StBoundaryProto::NON_NEAREST_STATIONARY,
                           &st_boundary_wd);
    }
  }
  return;
}

}  // namespace

void MakeIgnoreAndPreBrakeDecisionForStBoundaries(
    const IgnoreDescriptorInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::optional<VtSpeedLimit>* speed_limit, bool* const is_left_turn_ignore) {
  CHECK_NOTNULL(input.params);
  CHECK_NOTNULL(input.path);
  CHECK_NOTNULL(input.path_semantics);
  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.plan_passage);
  CHECK_NOTNULL(input.vehicle_geometry_params);
  CHECK_NOTNULL(input.av_shapes);
  CHECK_GT(input.max_v, 0.0);
  CHECK_GT(input.time_step, 0.0);
  CHECK_GT(input.trajectory_steps, 0);

  const auto& path_semantics = *input.path_semantics;
  bool on_left_turn_waiting_lane = false;
  if (!path_semantics.empty() && path_semantics.front().lane_semantic ==
                                     LaneSemantic::INTERSECTION_LEFT_TURN) {
    CHECK_NOTNULL(path_semantics.front().lane_info);
    const auto& current_lane_info = *path_semantics.front().lane_info;
    CHECK(current_lane_info.turn_type() ==
          e2e_noa::planning::TurnType::LEFT_TURN);

    on_left_turn_waiting_lane = true;
    if (!current_lane_info.next_lane_ids().empty()) {
      for (const auto& lane_id : current_lane_info.next_lane_ids()) {
        const auto& outgoing_lane_info_ptr =
            input.psmm->FindCurveLaneByIdOrNull(lane_id);
        if (outgoing_lane_info_ptr == nullptr) continue;
        if (outgoing_lane_info_ptr->turn_type() !=
            e2e_noa::planning::TurnType::LEFT_TURN) {
          on_left_turn_waiting_lane = false;
          break;
        }
      }
    } else {
      on_left_turn_waiting_lane = false;
    }
  }

  Merge_Direction merge_direction = MERGE_NONE;

  for (const auto& driving_process : input.driving_process_seq) {
    if (e2e_noa::planning::MergeTopology::TOPOLOGY_MERGE_LEFT ==
        driving_process.merge_topology) {
      merge_direction = MERGE_LEFT;
      break;
    } else if (e2e_noa::planning::MergeTopology::TOPOLOGY_MERGE_RIGHT ==
               driving_process.merge_topology) {
      merge_direction = MERGE_RIGHT;
      break;
    }
  }

  SplitTopology split_direction = SplitTopology::TOPOLOGY_SPLIT_NONE;
  for (const auto& driving_process : input.driving_process_seq) {
    if (e2e_noa::planning::SplitTopology::TOPOLOGY_SPLIT_LEFT ==
        driving_process.split_topology) {
      split_direction = SplitTopology::TOPOLOGY_SPLIT_LEFT;
      break;
    } else if (e2e_noa::planning::SplitTopology::TOPOLOGY_SPLIT_RIGHT ==
               driving_process.split_topology) {
      split_direction = SplitTopology::TOPOLOGY_SPLIT_RIGHT;
      break;
    }
  }

  const auto& curr_path_point = input.path->front();
  const auto av_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *input.vehicle_geometry_params);
  const auto av_frenet_box_or = input.plan_passage->QueryFrenetBoxAt(av_box);
  std::optional<FrenetBox> av_frenet_box = std::nullopt;
  bool av_in_plan_passage_lane = false;
  if (av_frenet_box_or.ok()) {
    av_frenet_box = *av_frenet_box_or;
    const auto av_s_min_boundary =
        input.plan_passage->QueryEnclosingLaneBoundariesAtS(
            av_frenet_box_or->s_min);
    const auto av_s_max_boundary =
        input.plan_passage->QueryEnclosingLaneBoundariesAtS(
            av_frenet_box_or->s_max);

    av_in_plan_passage_lane =
        av_s_min_boundary.left.has_value() &&
        av_s_max_boundary.left.has_value() &&
        av_s_min_boundary.right.has_value() &&
        av_s_max_boundary.right.has_value() &&
        av_frenet_box->l_max < av_s_min_boundary.left->lat_offset &&
        av_frenet_box->l_max < av_s_max_boundary.left->lat_offset &&
        av_frenet_box->l_min > av_s_min_boundary.right->lat_offset &&
        av_frenet_box->l_min > av_s_max_boundary.right->lat_offset;
  }

  std::vector<mapping::ElementId> upcoming_merge_lane_ids;
  absl::flat_hash_map<std::string, StBoundaryWithDecision*>
      protective_st_boundary_wd_map;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (!st_boundary_wd.raw_st_boundary()->is_protective() ||
        st_boundary_wd.raw_st_boundary()->protection_type() ==
            StBoundaryProto::LANE_CHANGE_GAP) {
      continue;
    }
    const auto& protected_st_boundary_id =
        st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
    if (!protected_st_boundary_id.has_value()) continue;
    protective_st_boundary_wd_map.emplace(*protected_st_boundary_id,
                                          &st_boundary_wd);
  }

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }
    MakeIgnoreAndPreBrakeDecisionForStBoundary(
        *input.params, *input.path, path_semantics, *input.st_traj_mgr,
        *input.plan_passage, *input.vehicle_geometry_params, *input.av_shapes,
        input.current_v, input.max_v, input.time_step, input.trajectory_steps,
        *input.psmm, on_left_turn_waiting_lane, av_frenet_box,
        av_in_plan_passage_lane, upcoming_merge_lane_ids, &st_boundary_wd,
        speed_limit, input.lc_stage, merge_direction, input.current_acc,
        input.driving_process_seq, split_direction,
        input.is_narrow_near_large_vehicle, is_left_turn_ignore);
    VLOG(2) << "[speed finder] id: " << st_boundary_wd.id()
            << " | decision info " << st_boundary_wd.decision_info()
            << " | decision_type " << st_boundary_wd.decision_type();
    if (const auto protective_st_boundary_wd_ptr =
            FindOrNull(protective_st_boundary_wd_map, st_boundary_wd.id());
        nullptr != protective_st_boundary_wd_ptr) {
      auto& protective_st_boundary_wd = **protective_st_boundary_wd_ptr;

      if (protective_st_boundary_wd.decision_type() !=
          st_boundary_wd.decision_type()) {
        protective_st_boundary_wd.set_decision_type(
            st_boundary_wd.decision_type());
        protective_st_boundary_wd.set_decision_reason(
            StBoundaryProto::FOLLOW_PROTECTED);
        protective_st_boundary_wd.set_ignore_reason(
            st_boundary_wd.ignore_reason());
        protective_st_boundary_wd.set_decision_info(
            st_boundary_wd.decision_info());
      }
    }
  }

  MakeIgnoreDecisionForNonNearestStationaryStBoundaries(st_boundaries_wd);
}
}  // namespace planning
}  // namespace e2e_noa
