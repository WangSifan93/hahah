#include "speed/object_scene_recognition.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "speed/decision/interaction_util.h"

namespace e2e_noa::planning {
namespace {

constexpr double kEps = 1e-6;
constexpr double kOverlapLengthValve = 5.0;
constexpr double kSameDirAngleDiff = M_PI / 18.0;
constexpr double kOnComingAngleThreshold = M_PI * 7 / 8;
constexpr double kOnComingTimeThreshold = 2.0;
constexpr double kOnComingLatThreshold = 3.6;

constexpr double kInteractionLatDis = 0.2;

struct ClosePointInfo {
  int first_close_point_idx = -1;
  int final_close_point_idx = -1;
  int closest_point_idx = -1;
  bool has_samedir_close_point = false;
};

std::vector<DrivingProcess> SeparateAvDrivingProcess(
    const PlannerSemanticMapManager& psmm, const PlanPassage& plan_passage,
    const PathPoint av_pos) {
  std::vector<DrivingProcess> driving_process_seq;

  const auto& lane_path = plan_passage.lane_path();
  bool find_start = false;
  double accum_s = 0.0;
  for (const auto& lane_id : lane_path.lane_ids()) {
    double start_fraction = 0.0;
    double end_fraction = 1.0;
    const auto& lane = psmm.FindLaneByIdOrNull(lane_id);
    if (!lane) continue;

    if (!find_start) {
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
        find_start = true;
      }
    }

    const double process_length =
        std::fmax(0.0, (end_fraction - start_fraction) * lane->curve_length());
    DrivingProcess driving_process;
    driving_process.start_s = accum_s;
    driving_process.end_s = driving_process.start_s + process_length;
    driving_process.merge_topology = lane->merge_topology();
    driving_process.split_topology = lane->split_topology();
    accum_s = driving_process.end_s;

    if (lane->turn_type() == ad_e2e::planning::NO_TURN) {
      if (lane->type() == ad_e2e::planning::LANE_VIRTUAL_COMMON ||
          lane->type() == ad_e2e::planning::LANE_VIRTUAL_JUNCTION) {
        driving_process.zone_type = InteractionZone::JunctionStraight;
      } else {
        driving_process.zone_type = InteractionZone::Straight;
      }
    } else if (lane->turn_type() == ad_e2e::planning::LEFT_TURN) {
      driving_process.zone_type = InteractionZone::TurnLeft;
    } else if (lane->turn_type() == ad_e2e::planning::RIGHT_TURN) {
      driving_process.zone_type = InteractionZone::TurnRight;
    } else {
      driving_process.zone_type = InteractionZone::Unknown;
    }

    if (driving_process_seq.empty()) {
      driving_process_seq.push_back(driving_process);
    } else {
      driving_process_seq.push_back(driving_process);
    }

    driving_process.lane_id = lane_id;
  }

  return driving_process_seq;
}

void SeparateAvTurnType(const PlannerSemanticMapManager& psmm,
                        const PlanPassage& plan_passage, const PathPoint av_pos,
                        std::vector<TurnTypeInfo>& turn_type_info) {
  const auto& lane_path = plan_passage.lane_path();
  const auto& extend_lane_path = plan_passage.extend_lane_path();
  double av_accum_s = 0.0;
  ad_e2e::planning::TurnType last_turn_type = ad_e2e::planning::NO_TURN;
  bool is_first_find_start = true;
  bool find_start = false;
  for (const auto& lane_id : extend_lane_path.lane_ids()) {
    double start_fraction = 0.0;
    const auto& lane = psmm.FindLaneByIdOrNull(lane_id);
    if (!lane) {
      DLOG(ERROR) << "lane not found";
      continue;
    }
    if (!find_start) {
      const auto ff = BuildBruteForceFrenetFrame(lane->points(), false);
      if (!ff.ok()) {
        DLOG(ERROR) << "ff not ok";
        continue;
      }
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha;
      ff.value().XYToSL({av_pos.x(), av_pos.y()}, &sl, &normal, &index_pair,
                        &alpha);
      start_fraction = sl.s / std::fmax(lane->curve_length(), 1e-2);
      if (start_fraction < 0.0 - kEps || start_fraction > 1.0 - kEps) {
        start_fraction = 1.0;
      } else {
        find_start = true;
      }
    }
    const double process_length =
        std::fmax(0.0, start_fraction * lane->curve_length());
    av_accum_s += process_length;
    if (find_start) {
      break;
    }
  }

  if (!find_start) {
    return;
  }

  double accum_s = -av_accum_s;
  for (const auto& lane_id : extend_lane_path.lane_ids()) {
    const auto& lane = psmm.FindLaneByIdOrNull(lane_id);
    if (!lane) {
      DLOG(ERROR) << "lane not found";
      continue;
    }

    if (is_first_find_start) {
      auto& turn_type = turn_type_info.emplace_back();
      turn_type.start_s = accum_s;
      turn_type.end_s = accum_s + lane->curve_length();
      turn_type.turn_type = lane->turn_type();
      is_first_find_start = false;
      last_turn_type = lane->turn_type();
    } else {
      if (last_turn_type != lane->turn_type()) {
        auto& turn_type = turn_type_info.emplace_back();
        turn_type.start_s = accum_s;
        turn_type.end_s = accum_s + lane->curve_length();
        turn_type.turn_type = lane->turn_type();
        last_turn_type = lane->turn_type();
      } else {
        auto& turn_type = turn_type_info.back();
        turn_type.end_s = accum_s + lane->curve_length();
      }
    }
    accum_s = accum_s + lane->curve_length();
  }
}

void MatchObjectInteractionZone(
    const StBoundaryRef& st_boundary,
    const std::vector<DrivingProcess>& driving_process_seq,
    ObjectScenarioInfo* const obj_scene_info) {
  if (!obj_scene_info) return;
  const auto& slt_info = st_boundary->nearest_sl_points();
  for (const auto sl_info : slt_info) {
    if (std::fabs(sl_info.lat_dist) < kInteractionLatDis) {
      const auto driving_zone = std::lower_bound(
          driving_process_seq.begin(), driving_process_seq.end(), sl_info.av_s,
          [](const DrivingProcess& driving_process, double av_s) {
            return driving_process.end_s < av_s;
          });
      if (driving_zone != driving_process_seq.end()) {
        obj_scene_info->interaction_zone = driving_zone->zone_type;
      }
      return;
    }
  }
  return;
}

bool IsMergeObject(const DiscretizedPath& path,
                   const ClosePointInfo& close_point_info,
                   const SpacetimeObjectTrajectory* spacetime_obj) {
  CHECK_NOTNULL(spacetime_obj);
  const auto& obj_states = spacetime_obj->states();
  if (obj_states.empty()) return false;
  SpacetimeObjectState first_close_point_state, final_close_point_state;
  if (close_point_info.first_close_point_idx >= 0) {
    first_close_point_state =
        obj_states[close_point_info.first_close_point_idx];
  }

  if (close_point_info.final_close_point_idx >= 0) {
    final_close_point_state =
        obj_states[close_point_info.final_close_point_idx];
  }
  if (close_point_info.has_samedir_close_point &&
      close_point_info.first_close_point_idx > 0) {
    const auto& first_frenet_polygon =
        path.XYToSL(first_close_point_state.contour,
                    first_close_point_state.traj_point->pos());
    const auto& final_frenet_polygon =
        path.XYToSL(final_close_point_state.contour,
                    final_close_point_state.traj_point->pos());
    const double overlap_length =
        final_frenet_polygon.s_max - first_frenet_polygon.s_min;
    if (overlap_length > kOverlapLengthValve) {
      return true;
    }
  }
  return false;
}

double ExtendCrossReferenceLine(const DiscretizedPath& path,
                                const SpacetimeObjectState& end_point_state,
                                const double extend_length) {
  Vec2d extend_point, end_point(end_point_state.traj_point->pos());
  Vec2d unit_vec2d =
      Vec2d::CreateUnitVec2d(end_point_state.traj_point->theta());
  unit_vec2d = unit_vec2d * extend_length;

  extend_point.set_x(end_point.x() + unit_vec2d.x());
  extend_point.set_y(end_point.y() + unit_vec2d.y());
  FrenetCoordinate sl_extend = path.XYToSL(extend_point);
  FrenetCoordinate sl_end = path.XYToSL(end_point);

  if (sl_extend.l * sl_extend.l < 0.0) {
    double cross_point_s = sl_end.s + (sl_extend.s - sl_end.s) *
                                          fabs(sl_end.l) /
                                          (fabs(sl_end.l) + fabs(sl_extend.l));
    return cross_point_s;
  }

  return std::numeric_limits<double>::lowest();
}

bool IsCrossObject(const DiscretizedPath& path,
                   const ClosePointInfo& close_point_info,
                   const SpacetimeObjectTrajectory* spacetime_obj) {
  CHECK_NOTNULL(spacetime_obj);
  const auto& obj_states = spacetime_obj->states();
  if (obj_states.empty()) {
    return false;
  }
  SpacetimeObjectState first_close_point_state, final_close_point_state;
  if (close_point_info.first_close_point_idx >= 0) {
    first_close_point_state =
        obj_states[close_point_info.first_close_point_idx];
  }

  if (close_point_info.final_close_point_idx >= 0) {
    final_close_point_state =
        obj_states[close_point_info.final_close_point_idx];
  }
  const auto& closest_point_state =
      obj_states[close_point_info.closest_point_idx];
  const auto& closest_point_frenet_polygon = path.XYToSL(
      closest_point_state.contour, closest_point_state.traj_point->pos());
  const double distance_to_ref =
      std::fmin(std::fabs(closest_point_frenet_polygon.l_min),
                std::fabs(closest_point_frenet_polygon.l_max));

  double overlap_length = 0.0;
  if (close_point_info.has_samedir_close_point) {
    const auto& first_frenet_polygon =
        path.XYToSL(first_close_point_state.contour,
                    first_close_point_state.traj_point->pos());
    const auto& final_frenet_polygon =
        path.XYToSL(final_close_point_state.contour,
                    final_close_point_state.traj_point->pos());
    const double overlap_length =
        final_frenet_polygon.s_max - first_frenet_polygon.s_min;
  }

  constexpr double kExtendThreshold = 1.5;
  if (distance_to_ref < kExtendThreshold) {
    const auto& av_path_pt =
        path.Evaluate(closest_point_frenet_polygon.center_s());
    const double yaw_diff = NormalizeAngle(
        closest_point_state.traj_point->theta() - av_path_pt.theta());
    if (std::fabs(yaw_diff) > kSameDirAngleDiff &&
        overlap_length < kOverlapLengthValve) {
      return true;
    }
  } else {
    const double extend_length =
        2.0 * kDefaultLaneWidth + spacetime_obj->bounding_box().half_length();
    const auto& end_point_state = obj_states.back();
    double cross_point_s =
        ExtendCrossReferenceLine(path, end_point_state, extend_length);

    if (cross_point_s > 0.0) {
      const auto& av_path_pt = path.Evaluate(cross_point_s);
      const double yaw_diff = NormalizeAngle(
          end_point_state.traj_point->theta() - av_path_pt.theta());
      if (std::fabs(yaw_diff) > kSameDirAngleDiff &&
          overlap_length < kOverlapLengthValve) {
        return true;
      }
    }
  }

  return false;
}

bool IsOnComingObject(const DiscretizedPath& path,
                      const StBoundaryRef& st_boundary,
                      const SpacetimeObjectTrajectory* spacetime_obj) {
  CHECK_NOTNULL(spacetime_obj);
  constexpr double kLowSpeedThreshold = 2.0;
  constexpr double kEpsilon = 1.0e-6;
  if (spacetime_obj->pose().v() < kLowSpeedThreshold) {
    return false;
  }
  const auto& obj_states = spacetime_obj->states();
  if (obj_states.empty()) {
    return false;
  }
  auto obj_states_iter = obj_states.rbegin();
  while (obj_states_iter != obj_states.rend()) {
    if (obj_states_iter->traj_point->t() <= kOnComingTimeThreshold - kEpsilon) {
      break;
    }
    FrenetCoordinate obj_sl = path.XYToSL(obj_states_iter->traj_point->pos());
    const auto& ref_av_pt = path.Evaluate(obj_sl.s);
    const double yaw_diff = NormalizeAngle(
        obj_states_iter->traj_point->theta() - ref_av_pt.theta());
    const auto& frenet_polygon = path.XYToSL(
        obj_states_iter->contour, obj_states_iter->traj_point->pos());

    double boundary_to_ref =
        std::fabs(frenet_polygon.l_min) < std::fabs(frenet_polygon.l_max)
            ? frenet_polygon.l_min
            : frenet_polygon.l_max;

    if (std::fabs(boundary_to_ref) > kOnComingLatThreshold ||
        std::fabs(yaw_diff) < kOnComingAngleThreshold) {
      return false;
    }

    if (std::fabs(st_boundary->obj_sl_info().dl) > 1e-5 &&
        ((boundary_to_ref < 0.0) != (st_boundary->obj_sl_info().dl < 0.0))) {
      return false;
    }
    ++obj_states_iter;
  }

  return true;
}

bool IsSameDirection(const StBoundaryRef& st_boundary) {
  auto& nearest_sl_points = st_boundary->nearest_sl_points();
  bool has_adjacency = false, has_overlap = false, angle_in_range = false;
  constexpr double kAdjacencyLaneCoeff = 1.5;
  for (const auto& sl_point : nearest_sl_points) {
    if (sl_point.lat_dist < kAdjacencyLaneCoeff * kDefaultLaneWidth) {
      has_adjacency = true;
      if (sl_point.av_s > 0.0) {
        has_overlap = true;
      }
      const double yaw_diff =
          NormalizeAngle(sl_point.av_heading - sl_point.obj_heading);
      if (std::fabs(yaw_diff) < kSameDirAngleDiff) {
        angle_in_range = true;
      }
    }
  }
  if (has_adjacency && has_overlap && angle_in_range) {
    return true;
  }
  return false;
}

void ObjectRelationshipDecider(
    const DiscretizedPath& path,
    const SpacetimeObjectTrajectory* spacetime_object,
    const StBoundaryRef& st_boundary) {
  auto& obj_scene_info = st_boundary->mutable_obj_scenario_info();

  if (spacetime_object->is_stationary()) {
    obj_scene_info.relationship = Relationship::Static;
    return;
  }

  double minimum_distance_to_ref = std::numeric_limits<double>::max();
  bool has_samedir_close_point = false;
  ClosePointInfo close_point_info;
  auto& nearest_sl_points = st_boundary->nearest_sl_points();
  auto& lower_points = st_boundary->lower_points();

  constexpr double kCloseTimeThreshold = 8.0;
  constexpr double kSameDirLaneWidthCoeff = 0.5;
  for (const auto& sl_point : nearest_sl_points) {
    if (sl_point.t > kCloseTimeThreshold) {
      continue;
    }
    if (std::fabs(sl_point.lat_dist) < minimum_distance_to_ref) {
      close_point_info.closest_point_idx = sl_point.obj_idx;
      minimum_distance_to_ref = std::fabs(sl_point.lat_dist);
    }
    const double yaw_diff =
        NormalizeAngle(sl_point.av_heading - sl_point.obj_heading);

    if (sl_point.lat_dist < kSameDirLaneWidthCoeff * kDefaultLaneWidth &&
        std::fabs(yaw_diff) < kSameDirAngleDiff) {
      close_point_info.final_close_point_idx = sl_point.obj_idx;
      if (!has_samedir_close_point) {
        close_point_info.first_close_point_idx = sl_point.obj_idx;
        has_samedir_close_point = true;
      }
    }
  }

  close_point_info.has_samedir_close_point = has_samedir_close_point;

  constexpr double kNotRelevantLaneWidthCoeff = 2.0;
  if (minimum_distance_to_ref >
      kNotRelevantLaneWidthCoeff * kDefaultLaneWidth) {
    obj_scene_info.relationship = Relationship::NotRelevant;
    return;
  }

  if (IsOnComingObject(path, st_boundary, spacetime_object)) {
    obj_scene_info.relationship = Relationship::OnComing;
    return;
  }

  if (IsMergeObject(path, close_point_info, spacetime_object)) {
    obj_scene_info.relationship = Relationship::Merge;
    return;
  }

  if (IsCrossObject(path, close_point_info, spacetime_object)) {
    obj_scene_info.relationship = Relationship::Cross;
    return;
  }

  if (IsSameDirection(st_boundary)) {
    obj_scene_info.relationship = Relationship::SameDir;
    return;
  }
}

#define DEBUG_DECISION_PARAMS (0)

void SetDecisionParam(
    ObjectScenarioInfo* const obj_scene_info,
    const SpeedPlanningParamsProto::ObjectSceneParamsProto& obj_scene_params,
    StBoundaryProto::ObjectType object_type, double av_speed,
    std::string obj_id) {
  constexpr double kAvSpeedThreshold = Kph2Mps(10.0);

  if (!obj_scene_info) return;
  auto& param = obj_scene_info->obj_decision_param;

  if (object_type == StBoundaryProto::VEHICLE &&
      obj_scene_info->interaction_zone == InteractionZone::Straight &&
      obj_scene_info->bearing == Bearing::Right &&
      (!obj_scene_info->is_av_completely_in_obj_fov) &&
      obj_scene_info->obj_sl_info.ds > 1e-2) {
    if (obj_scene_info->delta_heading < M_PI / 6 &&
        obj_scene_info->delta_heading > M_PI / 12 &&
        std::fabs(obj_scene_info->current_vl) > 0.8) {
      param.pass_time_additional_buffer = 0.8;
      param.dp_follow_lead_ratio = 1.0;
      param.enable_interact_first_point_decision = false;
      param.enable_interact_last_point_decision = false;
      param.interact_av_decel_factor = 0.6;
      param.interact_obj_follow_dist_buffer = 2.0;
      return;
    }
    if (obj_scene_info->delta_heading < M_PI / 2 &&
        obj_scene_info->delta_heading > M_PI / 6) {
      param.enable_add_collision_risk_cost_for_dp = true;
      param.pass_time_additional_buffer = 0.3;
      param.dp_follow_lead_ratio = 1.0;
      param.enable_interact_first_point_decision = false;
      param.enable_interact_last_point_decision = false;
      param.interact_av_decel_factor = 0.6;
      param.interact_obj_follow_dist_buffer = 2.0;
      return;
    }
  }

#if DEBUG_DECISION_PARAMS
  std::cout << " object_id " << obj_id << std::endl;
  std::cout << " obj_scene_info->interaction_zone "
            << static_cast<int>(obj_scene_info->interaction_zone)
            << " obj_scene_info->bearing "
            << static_cast<int>(obj_scene_info->bearing)
            << " obj_scene_info->delta_heading "
            << obj_scene_info->delta_heading << " is_av_completely_in_obj_fov "
            << obj_scene_info->is_av_completely_in_obj_fov << std::endl;
#endif

  if ((obj_scene_info->interaction_zone == InteractionZone::Straight ||
       obj_scene_info->interaction_zone == InteractionZone::JunctionStraight) &&
      obj_scene_info->bearing == Bearing::Right &&
      obj_scene_info->delta_heading < M_PI / 2 &&
      obj_scene_info->delta_heading > M_PI / 6 &&
      std::fabs(obj_scene_info->obj_sl_info.dl) < 30.0 &&
      StBoundaryProto::VEHICLE == object_type) {
    param.enable_add_collision_risk_cost_for_dp = true;
  }

  if (StBoundaryProto::PEDESTRIAN == object_type) {
    if (av_speed <= kAvSpeedThreshold) {
      param.pass_time_additional_buffer = 0.8;
      param.dp_follow_lead_ratio = 1.0;
    }
  }

  if (StBoundaryProto::VEHICLE == object_type &&
      obj_scene_info->interaction_zone == InteractionZone::TurnLeft &&
      obj_scene_info->bearing == Bearing::Right) {
    param.pass_time_additional_buffer =
        obj_scene_params.lt_str_pass_time_addi_buffer();
    param.dp_follow_lead_ratio = obj_scene_params.lt_str_dp_follow_lead_ratio();
    param.enable_interact_first_point_decision = false;
    param.enable_interact_last_point_decision = false;
    param.interact_av_decel_factor = 0.6;

    param.interact_obj_follow_dist_buffer =
        obj_scene_params.lt_str_obj_follow_dist_buffer();
    param.enable_left_turn_overtake_cost =
        obj_scene_params.enable_lt_overtake_cost();
  }

  return;
}

void ModifyObjectTimeBuffForSpecificScenario(
    ObjectScenarioInfo* obj_scene_info) {
  if (!obj_scene_info) return;
  auto& param = obj_scene_info->obj_decision_param;

  const bool is_left_turn_to_staight =
      (obj_scene_info->interaction_zone == InteractionZone::TurnLeft) &&
      (obj_scene_info->relationship == Relationship::Cross ||
       obj_scene_info->relationship == Relationship::OnComing);

  if (is_left_turn_to_staight) {
    param.yield_time_additional_buffer = 0.8;
  }
}

}  // namespace

void MakeObjectSceneRecognition(
    const PlannerSemanticMapManager& psmm, const PlanPassage& plan_passage,
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedPlanningParamsProto::ObjectSceneParamsProto& obj_scene_params,
    std::vector<StBoundaryRef>* st_boundaries, double av_speed,
    std::vector<DrivingProcess>* driving_process_seq,
    std::vector<TurnTypeInfo>& turn_type_info) {
  if (driving_process_seq == nullptr) {
    return;
  }
  if (path.size() < 2) {
    return;
  }

  *driving_process_seq = SeparateAvDrivingProcess(psmm, plan_passage, path[0]);
  SeparateAvTurnType(psmm, plan_passage, path[0], turn_type_info);

  for (auto& st_boundary : *st_boundaries) {
    auto& obj_scene_info = st_boundary->mutable_obj_scenario_info();
    if (st_boundary->source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }

    if (st_boundary->obj_sl_info().dl > kEps) {
      obj_scene_info.bearing = Bearing::Left;
    } else if (st_boundary->obj_sl_info().dl < -kEps) {
      obj_scene_info.bearing = Bearing::Right;
    } else {
      obj_scene_info.bearing = Bearing::Overlap;
    }
    obj_scene_info.obj_sl_info = st_boundary->obj_sl_info();

    if (!st_boundary->object_id().has_value()) {
      continue;
    }
    const auto& obj =
        st_traj_mgr.FindObjectByObjectId(st_boundary->object_id().value());
    const auto& av_path_pt =
        path.Evaluate(st_boundary->obj_sl_info().frenet_polygon.s_max);
    const double theta_diff =
        NormalizeAngle(obj->pose().theta() - av_path_pt.theta());
    obj_scene_info.delta_heading = std::fabs(theta_diff);
    obj_scene_info.current_vl = obj->pose().v() * sin(theta_diff);

    CHECK(st_boundary->traj_id().has_value());
    const auto& traj_id = st_boundary->traj_id();
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
    obj_scene_info.is_av_completely_in_obj_fov = IsAvCompletelyInObjectFov(
        path.front(), traj->planner_object(), vehicle_geometry_params);

    ObjectRelationshipDecider(path, traj, st_boundary);

    MatchObjectInteractionZone(st_boundary, *driving_process_seq,
                               &obj_scene_info);

    SetDecisionParam(&obj_scene_info, obj_scene_params,
                     st_boundary->object_type(), av_speed,
                     st_boundary.get()->object_id().has_value()
                         ? st_boundary.get()->object_id().value()
                         : "-1");
    ModifyObjectTimeBuffForSpecificScenario(&obj_scene_info);
  }
}

}  // namespace e2e_noa::planning
