#include "descriptor/all_descriptor_builder.h"

#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "behavior.pb.h"
#include "common/log_data.h"
#include "common/timer.h"
#include "common/type_def.h"
#include "constraint.pb.h"
#include "crosswalk_state.pb.h"
#include "descriptor/cautious_brake_descriptor.h"
#include "descriptor/constraint_manager.h"
#include "descriptor/crosswalk_descriptor.h"
#include "descriptor/descriptor_util.h"
#include "descriptor/end_of_current_lane_path.h"
#include "descriptor/end_of_path_boundary.h"
#include "descriptor/lc_end_of_current_lane_constraint.h"
#include "descriptor/pedestrians_descriptor.h"
#include "descriptor/solid_line_within_boundary.h"
#include "descriptor/speed_bump.h"
#include "descriptor/standstill_descriptor.h"
#include "descriptor/traffic_gap_finder.h"
#include "descriptor/traffic_light_descriptor.h"
#include "glog/logging.h"
#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "math/geometry/halfplane.h"
#include "math/math_utils.h"
#include "math/util.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "util/status_macros.h"
#include "util/time_util.h"
#include "util/vehicle_geometry_util.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {
namespace {
using StopLineInterface = ad_e2e::planning::StopLineInterface;
struct PlannerObjectProjectionInfo {
  absl::Span<const SpacetimeObjectTrajectory* const> st_trajectories;
  FrenetBox frenet_box;
};

constexpr double kZero = 1e-6;

std::vector<PlannerObjectProjectionInfo> GetObstaclesOnLane(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  std::vector<PlannerObjectProjectionInfo> objects_on_lane;
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    if (trajectories.empty()) continue;
    const PlannerObject& obj = trajectories.front()->planner_object();

    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

    constexpr double kLateralThreshold = 1.0;
    if (frenet_box.l_min > kLateralThreshold ||
        frenet_box.l_max < -kLateralThreshold) {
      continue;
    }

    if (ego_frenet_box.center_s() > frenet_box.center_s()) {
      continue;
    }

    objects_on_lane.emplace_back(PlannerObjectProjectionInfo{
        .st_trajectories = trajectories, .frenet_box = frenet_box});
  }

  std::stable_sort(objects_on_lane.begin(), objects_on_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.frenet_box.s_min < b.frenet_box.s_min;
                   });
  return objects_on_lane;
}

absl::StatusOr<bool> IsOncomingObjectJudgeByPlanPassage(
    const PlanPassage& passage,
    const TrajectoryPointWithAcceleration& obj_pose) {
  ASSIGN_OR_RETURN(const auto tangent, passage.QueryTangentAt(obj_pose.pos()));
  const double passage_angle = tangent.Angle();
  const double angle_diff =
      std::abs(NormalizeAngle(passage_angle - obj_pose.theta()));

  return angle_diff > M_PI_2;
}

absl::StatusOr<FrenetBox> FilterObjectViaPlanPassage(
    const PlannerObject& object, const PlanPassage& passage,
    const PathSlBoundary& sl_boundary, const FrenetBox& ego_frenet_box) {
  ASSIGN_OR_RETURN(const auto object_frenet_box,
                   passage.QueryFrenetBoxAtContour(object.contour()));

  if (object_frenet_box.s_min < ego_frenet_box.s_max ||
      object_frenet_box.s_min > sl_boundary.end_s()) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of longitudinal boundary, s range: ( %.2f, %.2f)",
        object.id(), object_frenet_box.s_min, object_frenet_box.s_max));
  }

  constexpr double kLateralEnterThres = 0.5;
  const auto [boundary_l_max, boundary_l_min] =
      CalcSlBoundaries(sl_boundary, object_frenet_box);

  if (object_frenet_box.l_min > boundary_l_max - kLateralEnterThres ||
      object_frenet_box.l_max < boundary_l_min + kLateralEnterThres) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of lateral boundary, l range: (%.2f, %.2f)", object.id(),
        object_frenet_box.l_min, object_frenet_box.l_max));
  }
  return object_frenet_box;
}

bool YieldToVulnerableRoadUserScenario(
    const PlannerObjectController& obj_mgr,
    const ObjectHistoryController& obs_history,
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geo_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlanPassage& passage, ad_e2e::planning::SpeedState* speed_state) {
  auto& vru_interact_timer = speed_state->vru_interact_timer;
  auto pre_yield_ids = speed_state->infront_vru_ids;
  auto& infront_vru_ids = speed_state->infront_vru_ids;
  infront_vru_ids.clear();
  const double wheel_base = vehicle_geo_params.wheel_base();
  const double ego_front_to_center = vehicle_geo_params.front_edge_to_center();
  const double ego_back_to_center = vehicle_geo_params.back_edge_to_center();
  const double ego_half_width = vehicle_geo_params.width() * 0.5;

  Vec2d ego_pos(plan_start_point.path_point().x(),
                plan_start_point.path_point().y());
  const auto& ego_lon_offset = passage.QueryFrenetLonOffsetAt(ego_pos);
  const auto& ego_lat_offset = passage.QueryFrenetLatOffsetAt(ego_pos);
  if (!ego_lon_offset.ok() || !ego_lat_offset.ok()) {
    return false;
  }
  constexpr double kResetSpeed = 15.0;
  if (plan_start_point.v() > Kph2Mps(kResetSpeed)) {
    vru_interact_timer = 0.0;
    return false;
  }

  bool set_yield_timer = false;

  double ds_range = plan_start_point.v() * 3.0;
  ds_range = std::fmax(std::fmin(ds_range, 20.0), 5.0);
  for (const auto& object : obj_mgr.planner_objects()) {
    if (object.type() != OT_PEDESTRIAN && object.type() != OT_CYCLIST &&
        object.type() != OT_TRICYCLIST) {
      continue;
    }
    const auto& obj_history = obs_history.GetObjHistory(object.id());
    if (!obj_history || obj_history->Empty()) {
      continue;
    }
    bool pre_lon_yield =
        obj_history->GetLatestFrame()->lon_decision == StBoundaryProto::YIELD ||
        obj_history->GetLatestFrame()->lon_decision == StBoundaryProto::FOLLOW;

    auto obj_frenet_box =
        passage.QueryFrenetBoxAtContour(object.contour(), false);
    auto obj_frenet_point =
        passage.QueryUnboundedFrenetCoordinateAt(object.pose().pos());
    if ((!obj_frenet_box.ok()) || (!obj_frenet_point.ok())) {
      continue;
    }
    FrenetPolygon obj_frenet_polygon;
    obj_frenet_polygon.center = obj_frenet_point.value();
    obj_frenet_polygon.s_max =
        obj_frenet_box.value().s_max - ego_lon_offset.value().accum_s;
    obj_frenet_polygon.s_min =
        obj_frenet_box.value().s_min - ego_lon_offset.value().accum_s;
    obj_frenet_polygon.l_max =
        obj_frenet_box.value().l_max - ego_lat_offset.value();
    obj_frenet_polygon.l_min =
        obj_frenet_box.value().l_min - ego_lat_offset.value();

    double ds = 0.0;
    double dl = 0.0;
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

    if (pre_lon_yield && ds > -kZero && ds < ds_range && std::fabs(dl) < 5.0 &&
        object.velocity().norm() < 1.0) {
      infront_vru_ids.emplace_back(object.id());
      set_yield_timer = true;
      vru_interact_timer = 0.7;
      continue;
    }

    double relative_s = obj_frenet_box.value().center_s() -
                        ego_lon_offset.value().accum_s - wheel_base;
    double relative_s_min = obj_frenet_box.value().s_min -
                            ego_lon_offset.value().accum_s - 0.5 * wheel_base;
    double relative_s_max = obj_frenet_box.value().s_max -
                            ego_lon_offset.value().accum_s -
                            ego_front_to_center;
    double relative_dl = std::fabs(dl);
    auto it = find(pre_yield_ids.begin(), pre_yield_ids.end(), object.id());
    double dl_range = it != pre_yield_ids.end() ? 5.0 : 2.0;
    PiecewiseLinearFunction<double, double> dl_area_limit_plf({kZero, 10.0},
                                                              {dl_range, 0.5});
    double dl_area_limit = dl_area_limit_plf(relative_s);
    if (relative_s_min > kZero && relative_s_max < 10.0 &&
        relative_dl < dl_area_limit) {
      Vec2d obj_prev_pt = object.traj(0).points().back().pos();
      auto obj_prev_frenet_point =
          passage.QueryUnboundedFrenetCoordinateAt(obj_prev_pt);
      const auto& obs_l1 = obj_frenet_point.value().l;
      const auto& obs_l2 =
          obj_prev_frenet_point.ok() ? obj_prev_frenet_point.value().l : obs_l1;
      const double angle_diff = NormalizeAngle(
          object.pose().theta() - plan_start_point.path_point().theta());
      if (relative_dl > 0.1 && obs_l1 * obs_l2 > -kZero &&
          fabs(obs_l2) > fabs(obs_l1)) {
        continue;
      } else if (dl < -kZero &&
                 (angle_diff < 30.0 * ad_e2e::planning::Constants::DEG2RAD ||
                  angle_diff > 150.0 * ad_e2e::planning::Constants::DEG2RAD)) {
        continue;
      } else if (dl > kZero &&
                 (angle_diff > -30.0 * ad_e2e::planning::Constants::DEG2RAD ||
                  angle_diff < -150.0 * ad_e2e::planning::Constants::DEG2RAD)) {
        continue;
      }
      infront_vru_ids.emplace_back(object.id());
    }
  }

  if (vru_interact_timer > kZero) {
    if (infront_vru_ids.empty()) {
      vru_interact_timer = 0.0;
      return false;
    }
    if (plan_start_point.v() > Kph2Mps(3.0)) {
      return false;
    }
    vru_interact_timer -= set_yield_timer ? 0.0 : 0.1;
    return true;
  }
  vru_interact_timer = 0.0;
  return false;
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildStopLineConstraint(
    const std::string& type_str, const PlanPassage& passage,
    const DecisionConstraintConfigProto& config, double front_to_ra,
    double stop_dist) {
  const double stop_s = passage.lane_path_start_s() + stop_dist + front_to_ra -
                        config.normal_offset();

  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(stop_s));
  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(stop_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(type_str);
  stop_line.mutable_source()->mutable_brake_to_stop()->set_reason(
      absl::StrCat("Teleop triggered ", type_str));

  return stop_line;
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildBrakeToStopConstraint(
    const std::string& type_str, const PlanPassage& passage, double front_to_ra,
    double ego_v, double brake) {
  constexpr double kStandStillDist = 0.3;

  const double stop_dist =
      ego_v < 1.0 ? kStandStillDist : Sqr(ego_v) * 0.5 / brake;
  const double stop_s = passage.lane_path_start_s() + stop_dist + front_to_ra;

  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(stop_s));
  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(stop_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(type_str);
  stop_line.mutable_source()->mutable_brake_to_stop()->set_reason(
      absl::StrCat("Teleop triggered ", type_str));

  return stop_line;
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildLCCStopConstraint(
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const DecisionConstraintConfigProto& config,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage,
    const bool lcc_keep_brake, double* stop_s) {
  const double kStopBuffer = 1.0;
  bool find_target_arrow = false;
  bool lcc_stop_flag = false;
  bool current_lane_valid = false;
  for (int i = 0; i < lane_path_from_start.lane_ids_size(); i++) {
    const auto lane =
        psmm.FindLaneByIdOrNull(lane_path_from_start.lane_ids().at(i));
    if (!lane) {
      continue;
    }
    if (find_target_arrow && !lane->junction_id().empty()) {
      lcc_stop_flag = true;
      break;
    } else {
      find_target_arrow = false;
    }
    if (i == 0 && lane->center_line().IsValid()) {
      if (!lane->junction_id().empty() && lcc_keep_brake) {
        lcc_stop_flag = true;
        current_lane_valid = true;
        *stop_s = 0.0;
        break;
      }
      Vec2d start_xy(plan_start_point.path_point().x(),
                     plan_start_point.path_point().y());
      double s = 0.0, l = 0.0;
      constexpr double kLaneRemainLengthThreshold = 100.0;
      if (lane->center_line().GetProjection(start_xy, &s, &l) &&
          lane->center_line().length() - s < kLaneRemainLengthThreshold) {
        *stop_s += lane->center_line().length() - s;
        current_lane_valid = true;
      } else {
        *stop_s += lane->topo_length();
        current_lane_valid = false;
      }
    } else {
      *stop_s += lane->topo_length();
    }
    if (!(lane->arrow_type() & 1) &&
        ((lane->arrow_type() & 2) || (lane->arrow_type() & 4) ||
         (lane->arrow_type() & 8))) {
      if (i == lane_path_from_start.lane_ids_size() - 1) {
        lcc_stop_flag = true;
        break;
      } else {
        find_target_arrow = true;
      }
    }
  }
  double s_offset =
      current_lane_valid ? 0.0 : psmm.map_ptr()->route()->navi_start().s_offset;
  *stop_s -=
      s_offset + vehicle_geometry_params.front_edge_to_center() + kStopBuffer;
  if (*stop_s < kMathEpsilon) {
    *stop_s = kMathEpsilon;
  }
  if (!lcc_stop_flag) {
    return absl::NotFoundError("lcc stop flag false.");
  }

  constexpr double kLCCStopBrake = 3.0;
  double brake_a =
      std::fmin(kLCCStopBrake, plan_start_point.v() * plan_start_point.v() /
                                   (2.0 * (*stop_s)));
  if (brake_a < kLCCStopBrake - kMathEpsilon) {
    return BuildStopLineConstraint(
        "lcc_turn_stop_line", passage, config,
        vehicle_geometry_params.front_edge_to_center(), *stop_s);
  }
  return BuildBrakeToStopConstraint(
      "lcc_turn_stop_soft", passage,
      vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
      brake_a);
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildTJunctionStopConstraint(
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const DecisionConstraintConfigProto& config,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage,
    double* stop_s) {
  const double kTJunctionStopBuffer = 1.0;
  bool T_junction_stop_flag = false;

  bool current_lane_valid = false;
  for (int i = 0; i < lane_path_from_start.lane_ids_size(); i++) {
    const auto lane =
        psmm.FindLaneByIdOrNull(lane_path_from_start.lane_ids().at(i));
    if (!lane) {
      continue;
    }
    if (!lane->junction_id().empty() || !lane->is_navigation()) {
      T_junction_stop_flag = true;
      break;
    }
    if (i == 0 && lane->center_line().IsValid()) {
      Vec2d start_xy(plan_start_point.path_point().x(),
                     plan_start_point.path_point().y());
      double s = 0.0, l = 0.0;
      if (lane->center_line().GetProjection(start_xy, &s, &l)) {
        *stop_s += lane->center_line().length() - s;
        current_lane_valid = true;
      } else {
        *stop_s += lane->topo_length();
        current_lane_valid = false;
      }
    } else {
      *stop_s += lane->topo_length();
    }
    if (i == lane_path_from_start.lane_ids_size() - 1) {
      T_junction_stop_flag = true;
    }
  }
  double s_offset =
      current_lane_valid ? 0.0 : psmm.map_ptr()->route()->navi_start().s_offset;
  *stop_s -= s_offset + vehicle_geometry_params.front_edge_to_center() +
             kTJunctionStopBuffer;
  if (std::fabs(*stop_s) < kMathEpsilon) {
    *stop_s = kMathEpsilon;
  }
  if (!T_junction_stop_flag) {
    return absl::NotFoundError("T Junction stop flag false.");
  }

  constexpr double kLCCStopBrake = 3.0;
  double brake_a =
      std::fmin(kLCCStopBrake, plan_start_point.v() * plan_start_point.v() /
                                   (2.0 * (*stop_s)));
  if (brake_a < kLCCStopBrake - kMathEpsilon) {
    return BuildStopLineConstraint(
        "T_Junction_stop_line", passage, config,
        vehicle_geometry_params.front_edge_to_center(), *stop_s);
  }
  return BuildBrakeToStopConstraint(
      "T_Junction_stop_line", passage,
      vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
      brake_a);
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildYieldToVruConstraint(
    const PlannerObjectController& obj_mgr,
    const ObjectHistoryController& obs_history,
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage,
    std::optional<double> traffic_stopline_dist,
    ad_e2e::planning::SpeedState* speed_state) {
  constexpr double kPreviewDist = 10.0;
  bool vru_interact_zone = false;
  double accum_s = -vehicle_geometry_params.front_edge_to_center();
  for (int i = 0; i < lane_path_from_start.lane_ids_size(); i++) {
    if (accum_s > kPreviewDist) break;
    const auto lane =
        psmm.FindLaneByIdOrNull(lane_path_from_start.lane_ids().at(i));
    if (!lane) continue;
    if (!lane->junction_id().empty()) {
      vru_interact_zone = true;
      break;
    }
    if (i == 0 && lane->center_line().IsValid()) {
      Vec2d start_xy(plan_start_point.path_point().x(),
                     plan_start_point.path_point().y());
      double s = 0.0, l = 0.0;
      if (lane->center_line().GetProjection(start_xy, &s, &l)) {
        accum_s += lane->center_line().length() - s;
      } else {
        accum_s += lane->topo_length() -
                   psmm.map_ptr()->route()->navi_start().s_offset;
      }
    } else {
      accum_s += lane->topo_length();
    }
  }

  if (!vru_interact_zone) {
    return absl::NotFoundError("not vru interact zone.");
  }
  if (traffic_stopline_dist.has_value() &&
      traffic_stopline_dist.value() > kZero &&
      traffic_stopline_dist.value() - passage.lane_path_start_s() <
          kPreviewDist) {
    return absl::NotFoundError(
        "has traffic stop line, no need to yield to vru.");
  }

  if (!YieldToVulnerableRoadUserScenario(
          obj_mgr, obs_history, vehicle_geometry_params, plan_start_point,
          passage, speed_state)) {
    return absl::NotFoundError("no need to yield to vru.");
  }
  if (plan_start_point.v() > Kph2Mps(3.0)) {
    return absl::NotFoundError("av speed high, no need to yield to vru.");
  }

  const double kYieldToVRUBrake = plan_start_point.v() < kZero ? 0.5 : 0.1;
  return BuildBrakeToStopConstraint(
      "Yield_to_VRU", passage, vehicle_geometry_params.front_edge_to_center(),
      plan_start_point.v(), kYieldToVRUBrake);
}
}  // namespace

absl::Status ProcessLaneChangeEndOfCurrentLane(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  const auto& config = *descriptor_input.config;
  const auto& lc_state = *descriptor_input.lc_state;
  const auto& passage = *descriptor_input.passage;
  
  if (!config.enable_lc_end_of_current_lane() ||
      lc_state.stage() != LaneChangeStage::LCS_PAUSE ||
      descriptor_input.lane_path_before_lc == nullptr) {
    return absl::OkStatus();
  }

  const auto& lane_path_before_lc = *descriptor_input.lane_path_before_lc;
  if (!lane_path_before_lc.IsEmpty()) {
    auto lcp_speed = BuildLcEndOfCurrentLaneConstraints(
        passage, lane_path_before_lc, descriptor_input.plan_start_point->v());
    if (lcp_speed.ok()) {
      constraint_manager.AddSpeedRegion(std::move(lcp_speed).value());
    }
  }
  return absl::OkStatus();
}

absl::Status ProcessCrosswalkConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager,
    DeciderStateProto& new_decision_state) {
  const auto& config = *descriptor_input.config;
  
  if (!config.enable_crosswalk()) {
    return absl::OkStatus();
  }

  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& planner_semantic_map_manager = *descriptor_input.planner_semantic_map_manager;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& passage = *descriptor_input.passage;
  const auto& obj_mgr = *descriptor_input.obj_mgr;
  const auto& pre_decision_state = *descriptor_input.pre_decision_state;
  const double start_s_offset = descriptor_input.target_offset_from_start;
  
  const auto lane_path_from_start =
      start_s_offset == 0.0
          ? passage.lane_path()
          : passage.lane_path().AfterArclength(start_s_offset);

  bool has_traffic_light = true;
  const std::string focus_tl_id =
      new_decision_state.traffic_light_decision_state()
          .fsd_tld_state()
          .light_id();
  const auto& tl_ptr =
      planner_semantic_map_manager.map_ptr()->GetTrafficLightInfoById(
          focus_tl_id);
  if (!tl_ptr) {
    has_traffic_light = false;
  }

  ASSIGN_OR_RETURN(
      auto cw_decision_output,
      BuildCrosswalkConstraints(CrosswalkDescriptorInput{
          .vehicle_geometry_params = &vehicle_geometry_params,
          .psmm = &planner_semantic_map_manager,
          .plan_start_point = &plan_start_point,
          .passage = &passage,
          .lane_path_from_start = &lane_path_from_start,
          .obj_mgr = &obj_mgr,
          .last_crosswalk_states = &pre_decision_state.crosswalk_state(),
          .now_in_seconds = ToUnixDoubleSeconds(descriptor_input.plan_time),
          .s_offset = start_s_offset,
          .has_traffic_light = has_traffic_light,
      }));

  for (auto& cw_stop_line : cw_decision_output.stop_lines) {
    constraint_manager.AddStopLine(std::move(cw_stop_line));
  }
  for (auto& cw_speed_region : cw_decision_output.speed_regions) {
    constraint_manager.AddSpeedRegion(std::move(cw_speed_region));
  }
  for (auto& crosswalk_state : cw_decision_output.crosswalk_states) {
    *new_decision_state.add_crosswalk_state() = std::move(crosswalk_state);
  }
  
  return absl::OkStatus();
}

absl::Status ProcessPedestrianConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  const auto& config = *descriptor_input.config;
  const auto& lc_state = *descriptor_input.lc_state;
  
  if (!config.enable_pedestrians() || lc_state.stage() != LaneChangeStage::LCS_NONE) {
    return absl::OkStatus();
  }

  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& planner_semantic_map_manager = *descriptor_input.planner_semantic_map_manager;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& passage = *descriptor_input.passage;
  const auto& sl_boundary = *descriptor_input.sl_boundary;
  const auto& st_traj_mgr = *descriptor_input.st_traj_mgr;
  const double start_s_offset = descriptor_input.target_offset_from_start;
  
  const auto lane_path_from_start =
      start_s_offset == 0.0
          ? passage.lane_path()
          : passage.lane_path().AfterArclength(start_s_offset);

  ASSIGN_OR_RETURN(
      auto ped_speed_regions,
      BuildPedestriansConstraints(
          vehicle_geometry_params, planner_semantic_map_manager,
          plan_start_point, passage, lane_path_from_start, start_s_offset,
          sl_boundary, st_traj_mgr));

  for (auto& ped_speed_region : ped_speed_regions) {
    constraint_manager.AddSpeedRegion(std::move(ped_speed_region));
  }
  
  return absl::OkStatus();
}

absl::Status ProcessSpeedBumpConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  const auto& config = *descriptor_input.config;
  
  if (!config.enable_speed_bump()) {
    return absl::OkStatus();
  }

  const auto& planner_semantic_map_manager = *descriptor_input.planner_semantic_map_manager;
  const auto& passage = *descriptor_input.passage;
  
  auto speed_bumps = BuildSpeedBumpConstraints(planner_semantic_map_manager, passage);
  for (auto& speed_bump : speed_bumps) {
    CHECK_LE(speed_bump.start_s(), speed_bump.end_s())
        << speed_bump.ShortDebugString();
    constraint_manager.AddSpeedRegion(std::move(speed_bump));
  }
  
  return absl::OkStatus();
}

absl::Status ProcessCautiousBrakeConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  const auto& config = *descriptor_input.config;
  
  if (!config.enable_cautious_brake()) {
    return absl::OkStatus();
  }

  const auto& planner_semantic_map_manager = *descriptor_input.planner_semantic_map_manager;
  const auto& passage = *descriptor_input.passage;
  const auto& st_traj_mgr = *descriptor_input.st_traj_mgr;
  const double start_s_offset = descriptor_input.target_offset_from_start;
  
  const auto lane_path_from_start =
      start_s_offset == 0.0
          ? passage.lane_path()
          : passage.lane_path().AfterArclength(start_s_offset);

  auto cautious_brake_regions = BuildCautiousBrakeConstraints(
      planner_semantic_map_manager, passage, lane_path_from_start,
      start_s_offset, st_traj_mgr);
  for (auto& cautious_brake : cautious_brake_regions) {
    CHECK_LE(cautious_brake.start_s(), cautious_brake.end_s())
        << cautious_brake.ShortDebugString();
    constraint_manager.AddSpeedRegion(std::move(cautious_brake));
  }
  
  return absl::OkStatus();
}

absl::StatusOr<std::pair<std::optional<double>, int>> ProcessTrafficLightConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager,
    DeciderStateProto& new_decision_state) {
  const auto& config = *descriptor_input.config;
  
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  int tl_stop_interface = 0;
  
  if (descriptor_input.traffic_light_status_map == nullptr || !config.enable_traffic_light()) {
    return std::make_pair(distance_to_traffic_light_stop_line, tl_stop_interface);
  }

  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& passage = *descriptor_input.passage;
  const auto& planner_semantic_map_manager = *descriptor_input.planner_semantic_map_manager;
  const auto& pre_decision_state = *descriptor_input.pre_decision_state;
  const auto& st_traj_mgr = *descriptor_input.st_traj_mgr;
  const auto& map_func_id = descriptor_input.behavior->function_id();
  const auto& traffic_light_fun_enable = descriptor_input.behavior->traffic_light_func_enable();
  const double start_s_offset = descriptor_input.target_offset_from_start;
  
  const auto lane_path_from_start =
      start_s_offset == 0.0
          ? passage.lane_path()
          : passage.lane_path().AfterArclength(start_s_offset);

  std::vector<std::string> tl_input_debug;
  bool need_prebrake_for_lka = false;
  tl_input_debug.emplace_back(
      std::to_string(descriptor_input.plan_id) + "*ok-" +
      std::to_string(descriptor_input.enable_tl_ok_btn));
  Log2FG::LogDataV2("fsd-traffic", tl_input_debug);
  
  auto tl_decision_output = BuildTrafficLightConstraints(
      vehicle_geometry_params, plan_start_point, passage,
      lane_path_from_start, *descriptor_input.traffic_light_status_map,
      planner_semantic_map_manager,
      pre_decision_state.traffic_light_decision_state(),
      descriptor_input.enable_tl_ok_btn, descriptor_input.override_passable,
      config, map_func_id, st_traj_mgr, descriptor_input.plan_id,
      traffic_light_fun_enable, need_prebrake_for_lka);

  if (!tl_decision_output.ok()) {
    LOG(WARNING) << "Build tl stop lines failed with message: "
                 << tl_decision_output.status().ToString();
  } else {
    for (auto& tl_stop_line : tl_decision_output.value().stop_lines) {
      if (!distance_to_traffic_light_stop_line.has_value()) {
        distance_to_traffic_light_stop_line = tl_stop_line.s();
      }
      constraint_manager.AddStopLine(tl_stop_line);
    }

    tl_stop_interface = tl_decision_output.value()
                            .traffic_light_decision_state.fsd_tld_state()
                            .tl_stop_interface();
    *new_decision_state.mutable_traffic_light_decision_state() =
        tl_decision_output.value().traffic_light_decision_state;

    const int stop_line_size = tl_decision_output.value().stop_lines.size();
    for (int i = 0; i < stop_line_size; ++i) {
      const double traffic_stop_line_s =
          tl_decision_output.value().stop_lines[i].s();
      std::vector<Vec2d> stop_line_points;
      auto left_point = passage.QueryPointXYAtSL(traffic_stop_line_s, 1.5);
      auto right_point = passage.QueryPointXYAtSL(traffic_stop_line_s, -1.5);
      if (left_point.ok() && right_point.ok()) {
        stop_line_points.push_back(left_point.value());
        stop_line_points.push_back(right_point.value());
      }
    }
  }
  
  if (need_prebrake_for_lka) {
    Log2FG::LogDataV2("fsd-traffic", "lka prebrake!!!");
    constraint_manager.AddALimit(-1.0, Kph2Mps(70.0), "lka prebrake");
  }
  
  return std::make_pair(distance_to_traffic_light_stop_line, tl_stop_interface);
}

absl::StatusOr<int> ProcessLKAConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager,
    ad_e2e::planning::SpeedState& speed_state_output) {
  const auto& map_func_id = descriptor_input.behavior->function_id();
  
  if (map_func_id != Behavior_FunctionId_LKA || 
      descriptor_input.behavior->auto_navi_lc_enable_status()) {
    return 0;
  }

  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& config = *descriptor_input.config;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& planner_semantic_map_manager = *descriptor_input.planner_semantic_map_manager;
  const auto& passage = *descriptor_input.passage;
  const auto& speed_state = *descriptor_input.speed_state;
  const double start_s_offset = descriptor_input.target_offset_from_start;
  
  const auto lane_path_from_start =
      start_s_offset == 0.0
          ? passage.lane_path()
          : passage.lane_path().AfterArclength(start_s_offset);

  int tl_stop_interface = 0;
  
  // LCC Stop Constraint
  double stop_s = 0.0;
  auto stop_line_or = BuildLCCStopConstraint(
      vehicle_geometry_params, config, plan_start_point, lane_path_from_start,
      planner_semantic_map_manager, passage, speed_state.lcc_keep_brake,
      &stop_s);

  if (stop_line_or.ok()) {
    constraint_manager.AddStopLine(std::move(stop_line_or).value());
    speed_state_output.lcc_keep_brake = true;

    const double kUIOutputDist = 100.0;
    if (stop_s < kUIOutputDist) {
      tl_stop_interface = StopLineInterface::STOP_LINE_LCC_TURN;
    }
  } else {
    speed_state_output.lcc_keep_brake = false;
  }

  // T-Junction Stop Constraint
  const auto& sub_type = planner_semantic_map_manager.map_ptr()->sub_type();
  if (sub_type == ad_e2e::planning::JUNCTION_T_MAP) {
    double t_junction_stop_s = 0.0;
    auto t_junction_stop_line_or = BuildTJunctionStopConstraint(
        vehicle_geometry_params, config, plan_start_point,
        lane_path_from_start, planner_semantic_map_manager, passage, &t_junction_stop_s);

    if (t_junction_stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(t_junction_stop_line_or).value());

      const double kTJunctionOutputDist = 80.0;
      if (t_junction_stop_s < kTJunctionOutputDist) {
        tl_stop_interface = StopLineInterface::STOP_LINE_T_JUNCTION;
      }
      std::vector<Vec2d> stop_line_points;
      auto left_point = passage.QueryPointXYAtSL(t_junction_stop_s, 1.5);
      auto right_point = passage.QueryPointXYAtSL(t_junction_stop_s, -1.5);
      if (left_point.ok() && right_point.ok()) {
        stop_line_points.push_back(left_point.value());
        stop_line_points.push_back(right_point.value());
      }
    }
  }
  
  return tl_stop_interface;
}

absl::Status ProcessPrepareLaneChangeConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager,
    bool no_acc_gap) {
  const auto& config = *descriptor_input.config;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& passage = *descriptor_input.passage;
  const auto& st_traj_mgr = *descriptor_input.st_traj_mgr;
  // const auto& descriptor_input_lc_state = *descriptor_input.lc_state;
  const auto& left_navi_dist_v2 = descriptor_input.max_reach_length;
  const auto& lc_num = descriptor_input.lc_num;
  const auto& cur_dist_to_junction = descriptor_input.cur_dist_to_junction;

  if (!config.enable_prepare_lc() ||
      descriptor_input.route_target_info == nullptr ||
      descriptor_input.borrow_lane_boundary) {
    return absl::OkStatus();
  }
  const auto traffic_gaps = FindCandidateTrafficGapsOnLanePath(
      descriptor_input.route_target_info->frenet_frame,
      descriptor_input.route_target_info->ego_frenet_box,
      descriptor_input.route_target_info->st_traj_mgr,
      descriptor_input.lc_lead_obj_ids);
  const auto ego_pos = Extract2dVectorFromApolloProto(plan_start_point);
  double speed_limit = descriptor_input.behavior->cruising_speed_limit();
  Log2FG::LogDataV2("gap_debug",
                    "speed limit: " + std::to_string(speed_limit));
  const Box2d ego_box =
      ComputeAvBox(ego_pos, plan_start_point.path_point().theta(),
                    vehicle_geometry_params);
  const auto& ego_frenet_frame = *passage.frenet_frame();
  const auto ego_frenet_box = *ego_frenet_frame.QueryFrenetBoxAt(ego_box);
  std::vector<PlannerObjectProjectionInfo> objects_on_target_lane;
  std::vector<PlannerObjectProjectionInfo> objects_on_current_lane;
  objects_on_current_lane =
      GetObstaclesOnLane(ego_frenet_frame, ego_frenet_box, st_traj_mgr);
  objects_on_target_lane =
      GetObstaclesOnLane(descriptor_input.route_target_info->frenet_frame,
                          descriptor_input.route_target_info->ego_frenet_box,
                          descriptor_input.route_target_info->st_traj_mgr);

  double leader_obj_s_min = -1.0;
  double leader_obj_v = -1.0;
  if (!objects_on_current_lane.empty()) {
    leader_obj_s_min = objects_on_current_lane.front().frenet_box.s_min -
                        ego_frenet_box.s_max;
    leader_obj_v =
        objects_on_current_lane.front().st_trajectories.front()->pose().v();
  }

  Log2FG::LogDataV2("gap_debug",
                    "leader_obj_s_min: " + std::to_string(leader_obj_s_min) +
                        " leader_obj_v: " + std::to_string(leader_obj_v));

  auto gap_or = EvaluateAndTakeBestTrafficGap(
      traffic_gaps, descriptor_input.route_target_info->ego_frenet_box,
      descriptor_input.route_target_info->frenet_frame, plan_start_point.v(),
      speed_limit, left_navi_dist_v2, lc_num, no_acc_gap, leader_obj_s_min,
      leader_obj_v, descriptor_input.push_dir);
  if (gap_or.ok()) {
    constraint_manager.SetTrafficGap(std::move(gap_or).value());
    if (gap_or.value().acc_gap_target_a.has_value() &&
        gap_or.value().acc_gap_target_speed.has_value()) {
      const auto acc_gap_a = gap_or.value().acc_gap_target_a.value();
      const auto acc_gap_v = gap_or.value().acc_gap_target_speed.value();
      constraint_manager.AddALimit(acc_gap_a, acc_gap_v,
                                    "acc gap speed result");
      Log2FG::LogDataV2("gap_debug",
                        "plan id: " + std::to_string(descriptor_input.plan_id) +
                            " acc_gap_a: " + std::to_string(acc_gap_a) +
                            " acc_gap_v: " + std::to_string(acc_gap_v));
    }
    if (gap_or.value().dec_gap_target_a.has_value() &&
        gap_or.value().dec_gap_target_speed.has_value()) {
      const auto dec_gap_a = gap_or.value().dec_gap_target_a.value();
      const auto dec_gap_v = gap_or.value().dec_gap_target_speed.value();
      Log2FG::LogDataV2("gap_debug",
                        "plan id: " + std::to_string(descriptor_input.plan_id) +
                            " dec_gap_a: " + std::to_string(dec_gap_a) +
                            " dec_gap_v: " + std::to_string(dec_gap_v));
    }
  } else {
    Log2FG::LogDataV2("gap_debug", "can not find reasonable gap");
    if (!objects_on_target_lane.empty() &&
        (descriptor_input.push_dir == PushDirection::Push_Congestion_Left ||
          descriptor_input.push_dir == PushDirection::Push_Congestion_Right)) {
      int obstacle_num = 0;
      double obstacle_speed_sum = 0.0;
      double max_dist = plan_start_point.v() * 6.0;
      double min_dist = plan_start_point.v() * 4.0;
      for (const auto& object_on_lane : objects_on_target_lane) {
        if (object_on_lane.frenet_box.s_min -
                    descriptor_input.route_target_info->ego_frenet_box.s_max >
                0 &&
            object_on_lane.frenet_box.s_min -
                    descriptor_input.route_target_info->ego_frenet_box.s_max <
                max_dist) {
          obstacle_num++;
          obstacle_speed_sum += object_on_lane.st_trajectories.front()
                                    ->planner_object()
                                    .pose()
                                    .v();
        }
      }
      if (obstacle_num != 0) {
        double obstacle_average_speed =
            std::max(obstacle_speed_sum / obstacle_num - 1.5,
                      plan_start_point.v() - 1.5);
        Log2FG::LogDataV2("gap_debug",
                          "obstacle average speed: " +
                              std::to_string(obstacle_average_speed));
        double min_average_speed = 5.0;
        if (cur_dist_to_junction > 100 && cur_dist_to_junction < 150) {
          min_average_speed = 7.0;
        } else if (cur_dist_to_junction >= 150) {
          min_average_speed = 9.0;
        }
        constraint_manager.AddVLimit(
            std::max(obstacle_average_speed, min_average_speed), 1.5,
            "no reasonable gap");
      }
    }
  }
  return absl::OkStatus();
}

absl::Status ProcessStandstillConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  const auto& config = *descriptor_input.config;
  
  if (!config.enable_standstill()) {
    return absl::OkStatus();
  }

  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& passage = *descriptor_input.passage;
  
  ASSIGN_OR_RETURN(
      auto ss_stop_lines,
      BuildStandstillConstraints(vehicle_geometry_params, plan_start_point,
                                 passage, constraint_manager.StopLine()));
  for (auto& ss_stop_line : ss_stop_lines) {
    constraint_manager.AddStopLine(std::move(ss_stop_line));
  }
  
  return absl::OkStatus();
}

absl::Status ProcessSolidLineConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  const auto& config = *descriptor_input.config;
  
  if (!config.enable_solid_line_within_boundary()) {
    return absl::OkStatus();
  }

  const auto& passage = *descriptor_input.passage;
  const auto& sl_boundary = *descriptor_input.sl_boundary;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  
  const auto solid_lines = BuildSolidLineWithinBoundaryConstraint(
      passage, sl_boundary, plan_start_point);
  if (solid_lines.ok()) {
    for (auto& solid_line : *solid_lines) {
      constraint_manager.AddAvoidLine(solid_line);
    }
  }
  
  return absl::OkStatus();
}

absl::Status ProcessYieldToVruConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager,
    ad_e2e::planning::SpeedState& speed_state_output,
    std::optional<double> distance_to_traffic_light_stop_line) {
  const auto& config = *descriptor_input.config;
  
  if (!config.enable_yield_to_vru() || !descriptor_input.obs_history) {
    return absl::OkStatus();
  }

  const auto& obj_mgr = *descriptor_input.obj_mgr;
  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& planner_semantic_map_manager = *descriptor_input.planner_semantic_map_manager;
  const auto& passage = *descriptor_input.passage;
  const double start_s_offset = descriptor_input.target_offset_from_start;
  
  const auto lane_path_from_start =
      start_s_offset == 0.0
          ? passage.lane_path()
          : passage.lane_path().AfterArclength(start_s_offset);

  auto stop_line_or = BuildYieldToVruConstraint(
      obj_mgr, *descriptor_input.obs_history, vehicle_geometry_params,
      plan_start_point, lane_path_from_start, planner_semantic_map_manager,
      passage, distance_to_traffic_light_stop_line, &speed_state_output);

  if (stop_line_or.ok()) {
    constraint_manager.AddStopLine(std::move(stop_line_or).value());
    speed_state_output.yield_to_vru = true;
  } else {
    speed_state_output.yield_to_vru = false;
  }
  
  return absl::OkStatus();
}


absl::StatusOr<bool> ShouldUseSpecialAccGapScenario(
    const DescriptorInput& descriptor_input,
    double min_distance_to_end_or_junction) {
  const auto& st_traj_mgr = *descriptor_input.st_traj_mgr;
  const int lc_num = descriptor_input.lc_num;
  
  if (lc_num != 1 || min_distance_to_end_or_junction <= 30.0 ||
      descriptor_input.route_target_info == nullptr) {
    return false;
  }

  std::vector<PlannerObjectProjectionInfo> front_objects_on_target_lane;
  const auto target_frenet_frame = descriptor_input.route_target_info->frenet_frame;
  const auto target_ego_frenet_box = descriptor_input.route_target_info->ego_frenet_box;
  
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    if (trajectories.empty()) continue;
    const PlannerObject& obj = trajectories.front()->planner_object();
    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

    constexpr double kLateralThreshold = 1.0;
    if (frenet_box.l_min > kLateralThreshold ||
        frenet_box.l_max < -kLateralThreshold) {
      continue;
    }

    if (target_ego_frenet_box.center_s() > frenet_box.center_s()) {
      continue;
    }

    front_objects_on_target_lane.emplace_back(PlannerObjectProjectionInfo{
        .st_trajectories = trajectories, .frenet_box = frenet_box});
  }
  
  if (front_objects_on_target_lane.empty() || front_objects_on_target_lane.size() == 1) {
    return true;
  }

  std::stable_sort(front_objects_on_target_lane.begin(),
                   front_objects_on_target_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.frenet_box.s_min < b.frenet_box.s_min;
                   });
  
  double obstacle_density = min_distance_to_end_or_junction / front_objects_on_target_lane.size();
  double nearest_gap_length = front_objects_on_target_lane[1].frenet_box.s_min -
                             front_objects_on_target_lane[0].frenet_box.s_max;
  
  Log2FG::LogDataV2("special_acc_gap_debug",
                    "obstacle_density: " + std::to_string(obstacle_density) +
                    " nearest_gap_length: " + std::to_string(nearest_gap_length));
  
  return obstacle_density > 40.0 && nearest_gap_length > 25.0;
}


void ApplySpeedControlForLaneChanges(
    const DescriptorInput& descriptor_input,
    double min_distance_to_end_or_junction,
    bool special_acc_gap_scenario,
    ConstraintManager& constraint_manager,
    bool& no_acc_gap) {
  
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& passage = *descriptor_input.passage;
  const auto& behavior = *descriptor_input.behavior;
  const int lc_num = descriptor_input.lc_num;
  const double left_navi_dist_v2 = descriptor_input.max_reach_length;
  const auto map_func_id = behavior.function_id();
  const auto last_turn_type_v2 = descriptor_input.last_turn_type_v2;

  const std::vector<double> KSpeedDownDists = {
      std::numeric_limits<double>::lowest(), 140.0, 160.0, 200.0, 250.0};
  const std::vector<double> kTargetSpeeds = {25.0, 30.0, 35.0, 40.0};

  bool lcc_pre_dec_on = true;
  if (map_func_id == Behavior_FunctionId_LKA &&
      !behavior.auto_navi_lc_enable_status()) {
    lcc_pre_dec_on = false;
  }

  int speed_level = -1;
  if (min_distance_to_end_or_junction < 140.0) {
    speed_level = 0;
  } else if (min_distance_to_end_or_junction < 160.0) {
    speed_level = 1;
  } else if (min_distance_to_end_or_junction < 200.0) {
    speed_level = 2;
  } else if (min_distance_to_end_or_junction < 250.0) {
    speed_level = 3;
  }

  double speed_down_dist = KSpeedDownDists[ad_e2e::planning::math::Clamp(lc_num, 0, 4)];

  Log2FG::LogDataV2("pre_dec", Log2FG::TaskPrefix(descriptor_input.plan_id) +
                                   "lc num: " + std::to_string(lc_num));
  Log2FG::LogDataV2("pre_dec", Log2FG::TaskPrefix(descriptor_input.plan_id) +
                                   "left navi dist v2: " + std::to_string(left_navi_dist_v2));
  Log2FG::LogDataV2("pre_dec", Log2FG::TaskPrefix(descriptor_input.plan_id) +
                                   "cur_dist_to_junction: " +
                                   std::to_string(descriptor_input.cur_dist_to_junction));

  if (min_distance_to_end_or_junction < speed_down_dist && speed_level >= 0 &&
      lc_num > 0 && lcc_pre_dec_on) {
    double target_speed = kTargetSpeeds[speed_level] * ad_e2e::planning::Constants::KPH2MPS;
    target_speed = std::max(target_speed, plan_start_point.v() - 1.5);
    double end_s = std::max(8 * plan_start_point.v(), 80.0);
    end_s = std::min(end_s, passage.end_s());

    Log2FG::LogDataV2("pre_dec",
                      Log2FG::TaskPrefix(descriptor_input.plan_id) +
                          "dec to speed: " + std::to_string(target_speed));
    if (!special_acc_gap_scenario) {
      constraint_manager.AddVLimit(target_speed, 3.0, "navi lc pre dec");
      no_acc_gap = true;
    }
  } else if (left_navi_dist_v2 < 170 &&
             map_func_id != Behavior_FunctionId_CITY_NOA &&
             last_turn_type_v2 != ad_e2e::planning::V2TurnInfo::V2DetailTurnType::CONTINUE &&
             lcc_pre_dec_on) {
    double pre_target_speed = 45 * ad_e2e::planning::Constants::KPH2MPS;
    if (last_turn_type_v2 == ad_e2e::planning::V2TurnInfo::V2DetailTurnType::LEFT_MERGE) {
      pre_target_speed = 60 * ad_e2e::planning::Constants::KPH2MPS;
    }
    pre_target_speed = std::max(pre_target_speed, plan_start_point.v() - 1.5);
    double end_s = std::max(8 * plan_start_point.v(), 80.0);
    end_s = std::min(passage.end_s(), end_s);

    Log2FG::LogDataV2("pre_dec", Log2FG::TaskPrefix(descriptor_input.plan_id) +
                                     "dec to speed for pre: " +
                                     std::to_string(pre_target_speed));

    if (!special_acc_gap_scenario) {
      constraint_manager.AddVLimit(pre_target_speed, 3.0, "navi lc pre dec");
      no_acc_gap = true;
    }
  }
}


void AddBasicConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  
  const auto& passage = *descriptor_input.passage;
  const auto& sl_boundary = *descriptor_input.sl_boundary;

  auto end_of_cur_lp_constraint = BuildEndOfCurrentLanePathConstraint(passage);
  if (end_of_cur_lp_constraint.ok()) {
    constraint_manager.AddStopLine(std::move(end_of_cur_lp_constraint).value());
  }

  auto end_of_path_boundary_constraint = BuildEndOfPathBoundaryConstraint(passage, sl_boundary);
  if (end_of_path_boundary_constraint.ok()) {
    constraint_manager.AddStopLine(std::move(end_of_path_boundary_constraint).value());
  } else {
    LOG(WARNING) << "Build end of path boundary constraint failed: "
                 << end_of_path_boundary_constraint.status().ToString();
  }
}

void AddSystemBrakeConstraint(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  
  if (!descriptor_input.behavior->system_break_stop()) {
    return;
  }

  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& passage = *descriptor_input.passage;

  constexpr double kSystemBrake = 1.5;
  auto stop_line_or = BuildBrakeToStopConstraint(
      "BEHAVIOR_CHOICE_MRM_BRAKE", passage,
      vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
      kSystemBrake);

  if (stop_line_or.ok()) {
    constraint_manager.AddStopLine(std::move(stop_line_or).value());
  }
}


void AddCollisionAvoidanceConstraints(
    const DescriptorInput& descriptor_input,
    ConstraintManager& constraint_manager) {
  
  const auto& vehicle_geometry_params = *descriptor_input.vehicle_geometry_params;
  const auto& plan_start_point = *descriptor_input.plan_start_point;
  const auto& passage = *descriptor_input.passage;
  const auto& obj_mgr = *descriptor_input.obj_mgr;

  const auto ego_pos = Extract2dVectorFromApolloProto(plan_start_point);
  const Box2d ego_box = ComputeAvBox(ego_pos, plan_start_point.path_point().theta(),
                                   vehicle_geometry_params);
  const auto ego_frenet_box = passage.QueryFrenetBoxAt(ego_box, false);

  const double l_ttc = 2.0;
  for (const auto& object : obj_mgr.moving_objects()) {
    if (object->type() != OT_VEHICLE && object->type() != OT_LARGE_VEHICLE &&
        object->type() != OT_MOTORCYCLIST) {
      continue;
    }
    
    auto obj_frenet_box = passage.QueryFrenetBoxAtContour(object->contour(), false);
    if (!obj_frenet_box.ok()) {
      continue;
    }
    
    const double head_ds = obj_frenet_box.value().s_max - ego_frenet_box.value().s_max;
    
    double obs_dl = 0.0;
    double obs_ds = 0.0;
    if (obj_frenet_box.value().s_min > ego_frenet_box.value().s_max) {
      obs_ds = obj_frenet_box.value().s_min - ego_frenet_box.value().s_max;
    } else if (ego_frenet_box.value().s_min > obj_frenet_box.value().s_max) {
      obs_ds = obj_frenet_box.value().s_max - ego_frenet_box.value().s_min;
    } else {
      obs_ds = 0.0;
    }
    
    if (obj_frenet_box.value().l_min > ego_frenet_box.value().l_max) {
      obs_dl = obj_frenet_box.value().l_min - ego_frenet_box.value().l_max;
    } else if (ego_frenet_box.value().l_min > obj_frenet_box.value().l_max) {
      obs_dl = obj_frenet_box.value().l_max - ego_frenet_box.value().l_min;
    } else {
      obs_dl = 0.0;
    }
    
    if (obs_ds > -kZero && obs_ds <= 15.0 && std::fabs(obs_dl) > kZero) {
      const auto obj_heading = object->pose().theta();
      const double preview_t = 1.5;
      
      ASSIGN_OR_CONTINUE(
          const auto preview_tangent,
          passage.QueryTangentAtS(obj_frenet_box.value().center_s() +
                                  object->velocity().norm() * preview_t));
      
      const auto passage_heading = preview_tangent.Angle();
      const auto obj_heading_diff = NormalizeAngle(passage_heading - obj_heading);

      if (std::abs(obj_heading_diff) > M_PI_2) continue;

      const double obj_v_s = object->velocity().norm() * std::cos(obj_heading_diff);
      const double obj_v_l = object->velocity().norm() * std::sin(obj_heading_diff);
      double lon_safe_dis = 3.0;
      double collision_t = obs_dl / obj_v_l;
      bool collision_s = false;
      
      if (collision_t < kZero) {
        collision_s = false;
      } else if (collision_t > kZero && obs_ds < kZero && head_ds > kZero) {
        collision_s = true;
      } else if (obs_ds > kZero &&
                 (plan_start_point.v() * collision_t +
                      0.5 * plan_start_point.a() * collision_t * collision_t -
                      obj_v_s * collision_t >
                  obs_ds - lon_safe_dis)) {
        collision_s = true;
      }

      double head_ds_safe = (object->type() == OT_LARGE_VEHICLE) ? 15.0 : 10.0;
      if (head_ds > kZero && head_ds < head_ds_safe && collision_t < l_ttc && collision_s) {
        std::vector<double> acc_vector{-1.0, -0.5};
        std::vector<double> ttc_vector{l_ttc / 2.0, l_ttc};
        double dec_a = ad_e2e::planning::math::interp1_inc(ttc_vector, acc_vector, collision_t);
        constraint_manager.AddALimit(dec_a, Kph2Mps(15.0), "maybe cutin");
        Log2FG::LogDataV2("AddALimit",
                          absl::StrCat(Log2FG::TaskPrefix(descriptor_input.plan_id),
                                       object->id(), " -collision_t ", collision_t,
                                       " dec_a ", dec_a));
      }
    }
  }
}


absl::StatusOr<Descriptor> BuildDescriptorResults(
    const DescriptorInput& descriptor_input) {
  Timer timer(__FUNCTION__);

  // Validate required input parameters
  CHECK_NOTNULL(descriptor_input.vehicle_geometry_params);
  CHECK_NOTNULL(descriptor_input.spacetime_constraint_params);
  CHECK_NOTNULL(descriptor_input.config);
  CHECK_NOTNULL(descriptor_input.planner_semantic_map_manager);
  CHECK_NOTNULL(descriptor_input.lc_state);
  CHECK_NOTNULL(descriptor_input.plan_start_point);
  CHECK_NOTNULL(descriptor_input.passage);
  CHECK_NOTNULL(descriptor_input.obj_mgr);
  CHECK_NOTNULL(descriptor_input.st_traj_mgr);
  CHECK_NOTNULL(descriptor_input.pre_decision_state);
  CHECK_NOTNULL(descriptor_input.behavior);
  CHECK_NOTNULL(descriptor_input.speed_state);

  // Extract key input variables for easier access
  const auto& lc_state = *descriptor_input.lc_state;
  const auto& left_navi_dist_v2 = descriptor_input.max_reach_length;
  const auto& left_navi_dist_map = descriptor_input.left_navi_dist_map;
  const auto& map_func_id = descriptor_input.behavior->function_id();
  const auto& speed_state = *descriptor_input.speed_state;
  const auto& cur_dist_to_junction = descriptor_input.cur_dist_to_junction;
  
  // Initialize output data structures
  ConstraintManager constraint_manager;
  DeciderStateProto new_decision_state;
  ad_e2e::planning::SpeedState speed_state_output = speed_state;

  // Flag to control acceleration gap behavior for lane changes
  bool no_acc_gap = false;
  
  // Validate passage availability
  if (descriptor_input.passage->empty()) {
    return absl::UnavailableError(
        "Plan passage on target lane path not available.");
  }

  // Calculate minimum distance to end of route or junction for speed control
  double min_distance_to_end_or_junction = std::numeric_limits<double>::max();
  if (cur_dist_to_junction > 10) {
    min_distance_to_end_or_junction =
        std::min(left_navi_dist_v2, cur_dist_to_junction);
  } else {
    min_distance_to_end_or_junction = left_navi_dist_v2;
  }
  if (map_func_id == Behavior_FunctionId_CITY_NOA) {
    min_distance_to_end_or_junction = left_navi_dist_map;
  }

  // Determine if special acceleration gap scenario should be used
  ASSIGN_OR_RETURN(bool special_acc_gap_scenario,
                   ShouldUseSpecialAccGapScenario(descriptor_input, min_distance_to_end_or_junction));

  Log2FG::LogDataV2(
      "special_acc_gap_debug",
      "special_acc_gap_secnario: " + std::to_string(special_acc_gap_scenario));

  // Apply speed control constraints for lane change scenarios
  ApplySpeedControlForLaneChanges(descriptor_input, min_distance_to_end_or_junction,
                                  special_acc_gap_scenario, constraint_manager, no_acc_gap);

  // Process lane change specific constraints
  RETURN_IF_ERROR(ProcessLaneChangeEndOfCurrentLane(descriptor_input, constraint_manager));
  
  // Process pedestrian related constraints
  RETURN_IF_ERROR(ProcessPedestrianConstraints(descriptor_input, constraint_manager));

  // Add fundamental constraints (end of lane path, path boundary)
  AddBasicConstraints(descriptor_input, constraint_manager);

  // Process speed bump constraints
  RETURN_IF_ERROR(ProcessSpeedBumpConstraints(descriptor_input, constraint_manager));
  
  // Process cautious brake constraints for safety
  RETURN_IF_ERROR(ProcessCautiousBrakeConstraints(descriptor_input, constraint_manager));

  // Process traffic light constraints and get stop line distance
  ASSIGN_OR_RETURN(auto traffic_light_result, 
                   ProcessTrafficLightConstraints(descriptor_input, constraint_manager, new_decision_state));

  std::optional<double> distance_to_traffic_light_stop_line = traffic_light_result.first;
  int tl_stop_interface = traffic_light_result.second;

  // Process crosswalk constraints and update decision state
  RETURN_IF_ERROR(ProcessCrosswalkConstraints(descriptor_input, constraint_manager, new_decision_state));

  // Process LKA (Lane Keeping Assist) specific constraints
  ASSIGN_OR_RETURN(int lka_stop_interface, 
                   ProcessLKAConstraints(descriptor_input, constraint_manager, speed_state_output));

  // LKA stop interface takes priority over traffic light interface
  if (lka_stop_interface != 0) {
    tl_stop_interface = lka_stop_interface;
  }

  // Process lane change preparation constraints including traffic gap analysis
  RETURN_IF_ERROR(ProcessPrepareLaneChangeConstraints(descriptor_input, constraint_manager, no_acc_gap));
  
  // Process standstill constraints for stationary scenarios
  RETURN_IF_ERROR(ProcessStandstillConstraints(descriptor_input, constraint_manager));
  
  // Process solid line constraints to prevent illegal lane crossing
  RETURN_IF_ERROR(ProcessSolidLineConstraints(descriptor_input, constraint_manager));

  // Add system brake constraint if emergency brake is triggered
  AddSystemBrakeConstraint(descriptor_input, constraint_manager);

  // Process constraints for yielding to vulnerable road users (VRU)
  RETURN_IF_ERROR(ProcessYieldToVruConstraints(descriptor_input, constraint_manager, 
                                               speed_state_output, distance_to_traffic_light_stop_line));

  // Add collision avoidance constraints for moving objects
  AddCollisionAvoidanceConstraints(descriptor_input, constraint_manager);

  // Return the complete descriptor with all constraints and decision state
  return Descriptor{
      .constraint_manager = std::move(constraint_manager),
      .decision_state = std::move(new_decision_state),
      .distance_to_traffic_light_stop_line =
          distance_to_traffic_light_stop_line,
      .tl_stop_interface = tl_stop_interface,
      .speed_state = speed_state_output,
  };
}

}  // namespace planning
}  // namespace e2e_noa
