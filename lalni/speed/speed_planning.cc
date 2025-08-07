#include "speed/speed_planning.h"

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "async/async_util.h"
#include "common/log_data.h"
#include "common/path_approx.h"
#include "common/timer.h"
#include "constraint.pb.h"
#include "descriptor/constraint_manager.h"
#include "glog/logging.h"
#include "make_style_estimation.h"
#include "maps/lane_point.h"
#include "math/geometry/offset_rect.h"
#include "math/geometry/polygon2d.h"
#include "math/util.h"
#include "math/vec.h"
#include "messages/planning/driving/noa_debug_info.pb.h"
#include "object/partial_spacetime_object_trajectory.h"
#include "object/planner_object.h"
#include "object/processed_spacetime_trajectory_manager.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "perception.pb.h"
#include "plan/discretized_path.h"
#include "plan/planner_defs.h"
#include "planner_params.pb.h"
#include "prediction.pb.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction.h"
#include "router/plan_passage.h"
#include "speed/close_trajectory_decision.h"
#include "speed/constraint_generator.h"
#include "speed/cross_blind_decision.h"
#include "speed/decision/pre_brake_decision.h"
#include "speed/decision/pre_st_boundary_modifier.h"
#include "speed/decision/st_boundary_pre_decision.h"
#include "speed/ignore_decision.h"
#include "speed/interactive_speed_decision.h"
#include "speed/mcts_game_decision.h"
#include "speed/object_scene_recognition.h"
#include "speed/overlap_info.h"
#include "speed/path_semantic_analyzer.h"
#include "speed/path_speed_combiner.h"
#include "speed/speed_bound.h"
#include "speed/speed_debug.h"
#include "speed/speed_limit.h"
#include "speed/speed_limit_generator.h"
#include "speed/speed_limit_provider.h"
#include "speed/speed_optimizer.h"
#include "speed/speed_optimizer_config_dispatcher.h"
#include "speed/speed_optimizer_object.h"
#include "speed/speed_optimizer_object_manager.h"
#include "speed/speed_planning_flags.h"
#include "speed/speed_planning_util.h"
#include "speed/speed_point.h"
#include "speed/speed_vector.h"
#include "speed/st_boundary.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "speed/st_overlap_analyzer.h"
#include "speed/st_point.h"
#include "speed/standstill_distance_decision.h"
#include "speed/time_buffer_decision.h"
#include "speed/vt_speed_limit.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/map_util.h"
#include "util/path_util.h"
#include "util/spacetime_util.h"
#include "util/status_macros.h"
#include "util/time_util.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa {
namespace planning {

namespace {

absl::Status OptimizeSpeed(
    std::string_view base_name, double init_v, double init_a, double total_time,
    double delta_t, const SpeedOptimizerObjectManager& opt_obj_mgr,
    const SpeedBoundMapType& speed_bound_map, double path_length,
    const SpeedVector& reference_speed,
    const std::vector<std::pair<double, double>>& accel_soft_bound,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const SpeedPlanningParamsProto& speed_planning_params,
    SpeedVector* optimized_speed,
    SpeedPlanningDebugProto* speed_planning_debug_proto,
    const LaneChangeStage& lc_stage) {
  CHECK_NOTNULL(optimized_speed);
  CHECK_NOTNULL(speed_planning_debug_proto);
  CHECK_GT(path_length, 0.0);

  SpeedOptimizer speed_optimizer(
      base_name, init_v, init_a, &spacetime_constraint_params,
      &speed_planning_params, path_length,
      spacetime_constraint_params.default_speed_limit(), delta_t);
  const absl::Status status = speed_optimizer.Optimize(
      opt_obj_mgr, speed_bound_map, reference_speed, accel_soft_bound,
      optimized_speed, speed_planning_debug_proto);
  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Speed optimizer failed: ", status.message()));
  }

  return absl::OkStatus();
}

std::optional<TrajectoryEndInfoProto> SetTrajectoryEndInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    double speed_length) {
  std::optional<TrajectoryEndInfoProto> traj_end_info = std::nullopt;
  for (const StBoundaryWithDecision& st_boundary_with_decision :
       st_boundaries_with_decision) {
    const auto* st_boundary = st_boundary_with_decision.st_boundary();
    if (!st_boundary->is_stationary()) {
      continue;
    }

    const double min_s = st_boundary->min_s();
    const double object_upper_bound =
        min_s - st_boundary_with_decision.follow_standstill_distance();
    const double intrusion_value = speed_length - object_upper_bound;

    if (intrusion_value > 0.0) {
      if (!traj_end_info.has_value() || min_s < traj_end_info->end_s()) {
        TrajectoryEndInfoProto end_info_proto;
        end_info_proto.set_st_boundary_id(st_boundary->id());
        end_info_proto.set_end_s(min_s);
        end_info_proto.set_intrusion_value(intrusion_value);
        end_info_proto.set_type(st_boundary->source_type());
        traj_end_info = std::move(end_info_proto);
      }
    }
  }
  return traj_end_info;
}

ObjectsPredictionProto ConstructModifiedPrediction(
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const int plan_id,
    std::map<std::string, prediction::ObjectPrediction>& predictions_debug) {
  struct PredElements {
    std::vector<prediction::PredictedTrajectory> trajs;
    const ObjectProto* object_proto = nullptr;
  };

  std::map<std::string, PredElements> processed_preds;
  for (const auto& [traj_id, st_obj] : processed_st_objects) {
    const auto object_id =
        SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(traj_id);
    if (processed_preds[object_id].object_proto == nullptr) {
      processed_preds[object_id].object_proto =
          &st_obj.planner_object().object_proto();
    }
    processed_preds[object_id].trajs.push_back(st_obj.trajectory());
  }

  ObjectsPredictionProto modified_prediction;
  for (auto& [object_id, pred_elements] : processed_preds) {
    const prediction::ObjectPrediction obj_pred(std::move(pred_elements.trajs),
                                                *pred_elements.object_proto);
    obj_pred.ToProto(modified_prediction.add_objects());
    predictions_debug[object_id] = std::move(obj_pred);
  }

  return modified_prediction;
}

StBoundaryWithDecision GenerateStBoundaryWithDecisionByStopLine(
    const StGraph& st_graph,
    const ConstraintProto::PathStopLineProto& path_stop_line) {
  StBoundaryWithDecision stb_wd(st_graph.MapPathStopLine(path_stop_line),
                                StBoundaryProto::YIELD,
                                StBoundaryProto::CONSTRAINT_GENERATOR);
  stb_wd.set_follow_standstill_distance(path_stop_line.standoff());
  return stb_wd;
}

inline bool IsObjectBeyondAvFrontEdge(
    const PlannerObject& planner_obj, const PathPoint& av_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto& obj_contour = planner_obj.contour();
  const Vec2d av_tan = Vec2d::FastUnitFromAngle(av_point.theta());
  const Vec2d& obj_center = obj_contour.CircleCenter();
  return av_tan.Dot(obj_center - ToVec2d(av_point)) +
             obj_contour.CircleRadius() >
         vehicle_geometry_params.front_edge_to_center();
}

std::optional<std::string> GetAlertedFrontVehicleForHMI(
    const SpeedOptimizerObjectManager& opt_obj_mgr,
    const SpacetimeTrajectoryManager& traj_mgr,
    const PathPoint& current_path_point, const SpeedVector& optimized_speed,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    std::string_view base_name) {
  const double max_decel =
      std::min_element(
          optimized_speed.begin(), optimized_speed.end(),
          [](const auto& lhs, const auto& rhs) { return lhs.a() < rhs.a(); })
          ->a();
  constexpr double kDecelThres = -1.0;
  if (max_decel > kDecelThres) return std::nullopt;

  constexpr double kMaxTime = 3.0;
  constexpr double kDeltaT = 0.2;
  std::map<std::string, double> object_id_to_slack;
  for (const SpeedOptimizerObject& opt_obj :
       opt_obj_mgr.MovingFollowObjects()) {
    if (opt_obj.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (opt_obj.is_protective()) continue;
    if (opt_obj.object_type() != StBoundaryProto::VEHICLE) continue;
    const PlannerObject* planner_obj =
        CHECK_NOTNULL(traj_mgr.FindObjectByObjectId(
            {opt_obj.id().data(), opt_obj.id().size()}));
    if (!IsObjectBeyondAvFrontEdge(*planner_obj, current_path_point,
                                   vehicle_geometry_params)) {
      continue;
    }
    for (double t = 0.0; t <= kMaxTime; t += kDeltaT) {
      const auto& overlap_state = opt_obj.GetOverlapStateByTime(t);
      if (!overlap_state.has_value()) continue;
      const auto speed_pt = optimized_speed.EvaluateByTime(t);
      CHECK(speed_pt.has_value());
      const double slack =
          speed_pt->s() - overlap_state->bound + overlap_state->lon_buffer;
      if (slack > 0.0) {
        object_id_to_slack[std::string(opt_obj.id())] += slack;
      }
    }
  }
  std::optional<std::string> alerted_front_vehicle_id;
  double max_slack = 0.0;
  for (const auto& [id, slack] : object_id_to_slack) {
    if (slack > max_slack) {
      max_slack = slack;
      alerted_front_vehicle_id = id;
    }
  }
  if (alerted_front_vehicle_id.has_value()) {
  }
  return alerted_front_vehicle_id;
}

SpeedBoundMapType EstimateSpeedBound(
    const SpeedLimitProvider& speed_limit_provider,
    const SpeedVector& preliminary_speed, double init_v,
    double allowed_max_speed, int knot_num, double delta_t,
    std::string_view base_name) {
  const auto fill_speed_bound =
      [](const std::optional<SpeedLimit::SpeedLimitInfo>& speed_limit_info,
         SpeedBoundWithInfo* speed_bound, double allowed_max_speed) {
        CHECK_NOTNULL(speed_bound);
        speed_bound->bound = speed_limit_info.has_value()
                                 ? speed_limit_info->speed_limit
                                 : allowed_max_speed;
        speed_bound->info =
            speed_limit_info.has_value() ? speed_limit_info->info : "NO_TYPE";
      };

  SpeedBoundMapType speed_upper_bound_map;

  std::vector<double> estimated_s;
  estimated_s.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    const double t = i * delta_t;
    const auto speed_point = preliminary_speed.EvaluateByTime(t);
    const double s = speed_point.has_value() ? speed_point->s() : init_v * t;
    estimated_s.push_back(s);
  }

  speed_upper_bound_map.emplace(
      SpeedLimitTypeProto::DEFAULT,
      std::vector<SpeedBoundWithInfo>(
          knot_num,
          SpeedBoundWithInfo{.bound = allowed_max_speed, .info = "Default."}));

  for (const auto& [type, speed_limit] :
       speed_limit_provider.static_speed_limit_map()) {
    std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
    speed_bounds_with_info.reserve(knot_num);
    for (int i = 0; i < knot_num; ++i) {
      const auto speed_limit_info =
          speed_limit.GetSpeedLimitInfoByS(estimated_s[i]);
      fill_speed_bound(speed_limit_info, &speed_bounds_with_info.emplace_back(),
                       allowed_max_speed);
    }
    speed_upper_bound_map.emplace(type, std::move(speed_bounds_with_info));
  }

  constexpr double kCloseTrajHardBrakeThres = -1.0;
  std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
  speed_bounds_with_info.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    const double t = i * delta_t;
    const auto dynamic_speed_limit_info =
        speed_limit_provider.GetDynamicSpeedLimitInfoByTimeAndS(t,
                                                                estimated_s[i]);
    SpeedBoundWithInfo speed_bound;
    fill_speed_bound(dynamic_speed_limit_info, &speed_bound, allowed_max_speed);

    const double decel = i == 0 ? 0.0 : (speed_bound.bound - init_v) / t;
    if (decel < kCloseTrajHardBrakeThres) {
    }
    speed_bounds_with_info.push_back(std::move(speed_bound));
  }
  speed_upper_bound_map.emplace(SpeedLimitTypeProto::MOVING_CLOSE_TRAJ,
                                std::move(speed_bounds_with_info));

  for (const auto& [type, _] : speed_limit_provider.vt_speed_limit_map()) {
    const bool has_type =
        speed_upper_bound_map.find(type) != speed_upper_bound_map.end();

    for (int i = 0; i < knot_num; ++i) {
      const double t = i * delta_t;
      const auto vt_speed_limit_info =
          speed_limit_provider.GetVtSpeedLimitInfoByTypeAndTime(type, t);
      if (!has_type) {
        SpeedBoundWithInfo speed_bound = {
            .bound = vt_speed_limit_info.has_value()
                         ? vt_speed_limit_info->speed_limit
                         : allowed_max_speed,
            .info = vt_speed_limit_info.has_value()
                        ? absl::StrCat(vt_speed_limit_info->info, "-vt")
                        : "NO_TYPE"};
        speed_upper_bound_map[type].push_back(std::move(speed_bound));
      } else {
        if (!vt_speed_limit_info.has_value()) continue;
        SpeedBoundWithInfo& origin = speed_upper_bound_map[type][i];
        if (vt_speed_limit_info->speed_limit < origin.bound) {
          origin.bound = vt_speed_limit_info->speed_limit;
          origin.info = absl::StrCat(vt_speed_limit_info->info, "-vt");
        }
      }
    }
  }

  return speed_upper_bound_map;
}

std::vector<SpeedBoundWithInfo> GenerateMinSpeedLimitWithLaneAndCurvature(
    const std::vector<SpeedBoundWithInfo>& lane_speed_limit,
    const std::vector<SpeedBoundWithInfo>& curvature_speed_limit) {
  std::vector<SpeedBoundWithInfo> min_speed_limit;
  CHECK_EQ(lane_speed_limit.size(), curvature_speed_limit.size());
  int knot_num = static_cast<int>(lane_speed_limit.size());
  min_speed_limit.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    min_speed_limit.push_back(SpeedBoundWithInfo{
        .bound =
            std::min(lane_speed_limit[i].bound, curvature_speed_limit[i].bound),
        .info = "Min speed limit between lane and curvature."});
  }
  return min_speed_limit;
}

DiscretizedPath EgoPredictTrajectory(const PathPoint& current_path_point,
                                     double duration, double dt,
                                     double current_speed) {
  DiscretizedPath ego_predict_path;
  if (current_speed < 0.5) return ego_predict_path;
  PathPoint state = current_path_point;
  const double yaw_rate = current_speed * current_path_point.kappa();
  if (yaw_rate > 0.0) return ego_predict_path;
  for (double t = 0; t <= duration; t += dt) {
    state.set_x(state.x() + current_speed * std::cos(state.theta()) * dt);
    state.set_y(state.y() + current_speed * std::sin(state.theta()) * dt);
    state.set_theta(yaw_rate * dt + state.theta());
    state.set_s(state.s() + current_speed * dt);

    ego_predict_path.push_back(state);
  }

  return ego_predict_path;
}

bool IsNarrowNearLargeVehicle(
    const double av_s, const PlanPassage& plan_passage,
    const PathSlBoundary& sl_boundary,
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::vector<StBoundaryRef>& st_boundaries,
    const std::vector<StCloseTrajectory>& moving_close_trajs) {
  constexpr double kPassageEps = 0.1;
  constexpr double kQueryLength = 5;
  constexpr double kWidthThreshold = 3.05;
  const double max_query_s = plan_passage.end_s() - kPassageEps;
  const double query_start_s = av_s;
  const double query_end_s =
      std::fmin(query_start_s + kQueryLength, max_query_s);
  double query_s = query_start_s;
  double accum_width = 0.0;
  int accum_n = 0;
  while (query_s < query_end_s) {
    const auto [l_min, l_max] = sl_boundary.QueryBoundaryL(query_s);
    const double width = std::fabs(l_max - l_min);
    accum_width += width;
    accum_n++;
    query_s += 0.3;
  }

  const double avg_width = accum_width / std::max(double(accum_n), 0.001);
  const bool is_narrow = avg_width <= kWidthThreshold;

  bool is_near_large_vehicle = false;
  for (const auto& moving_close_traj : moving_close_trajs) {
    const PlannerObject* obj =
        traj_mgr.FindObjectByObjectId({moving_close_traj.object_id().data(),
                                       moving_close_traj.object_id().size()});
    if (!obj->is_large_vehicle()) {
      continue;
    }

    double lat_dist = std::numeric_limits<double>::max();
    for (const auto& single_nearest_point :
         moving_close_traj.st_nearest_points()) {
      lat_dist = std::min(std::fabs(single_nearest_point.lat_dist), lat_dist);
    }

    if (lat_dist < 0.45) {
      is_near_large_vehicle = true;
    }
  }

  for (const auto& st_boundary : st_boundaries) {
    if (nullptr == st_boundary) {
      continue;
    }
    if (!st_boundary->object_id().has_value()) {
      continue;
    }
    const PlannerObject* obj =
        traj_mgr.FindObjectByObjectId(st_boundary->object_id().value());

    if (nullptr == obj || !obj->is_large_vehicle()) {
      continue;
    }

    if (std::fabs(st_boundary->obj_sl_info().dl) < 0.45) {
      is_near_large_vehicle = true;
    }
  }

  return false;
}

void DealDenseTrafficFlowConstraint(
    const double plan_start_v,
    const SpacetimeTrajectoryManager& speed_traj_manager,
    const std::vector<PathPointSemantic>& path_semantics,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const StGraph& st_graph,
    std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    ConstraintManager& constraint_mgr) {
  auto dense_traffic_flow_constraint = GenerateDenseTrafficFlowConstraint(
      st_boundaries_with_decision, speed_traj_manager, path_semantics, path,
      plan_start_v, vehicle_geometry_params);

  for (auto& path_stop_line : dense_traffic_flow_constraint.path_stop_lines) {
    st_boundaries_with_decision.push_back(
        GenerateStBoundaryWithDecisionByStopLine(st_graph, path_stop_line));
    constraint_mgr.AddPathStopLine(std::move(path_stop_line));
  }
}

void DealStaticObstacles(
    const bool is_open_gap, const double gap_expand_follow_standstill,
    const StandstillDistanceDescriptorInput&
        standstill_distance_descriptor_input,
    std::vector<StBoundaryWithDecision>& st_boundaries_with_decision) {
  auto nearest_stationary_id =
      GetNearestStationaryStBoundary(&st_boundaries_with_decision);
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DecideStandstillDistanceForStBoundary(standstill_distance_descriptor_input,
                                          &st_boundary_with_decision);
    if (is_open_gap && nearest_stationary_id.has_value() &&
        st_boundary_with_decision.id() == nearest_stationary_id.value()) {
      if (st_boundary_with_decision.st_boundary()->is_large_vehicle()) {
        st_boundary_with_decision.set_follow_standstill_distance(
            std::max(st_boundary_with_decision.follow_standstill_distance(),
                     1.5 * gap_expand_follow_standstill));
      } else {
        st_boundary_with_decision.set_follow_standstill_distance(
            std::max(st_boundary_with_decision.follow_standstill_distance(),
                     gap_expand_follow_standstill));
      }
      Log2FG::LogDataV0(
          "gap_expand_follow_standstill",
          "ID: " + st_boundary_with_decision.id() + " , " +
              std::to_string(
                  st_boundary_with_decision.follow_standstill_distance()));
    }
  }

  KeepNearestStationarySpacetimeTrajectoryStBoundary(
      &st_boundaries_with_decision);
}

void ModifyTimeBuffersOfStboundaries(
    const double plan_start_v,
    const SpacetimeTrajectoryManager& speed_traj_manager,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    std::vector<StBoundaryWithDecision>& st_boundaries_with_decision) {
  absl::flat_hash_set<std::string> disable_pass_time_buffer_id_set;
  for (const auto& st_boundary_wd : st_boundaries_with_decision) {
    if (!st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }
    const auto& protected_st_boundary_id =
        st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
    if (!protected_st_boundary_id.has_value()) continue;
    switch (st_boundary_wd.raw_st_boundary()->protection_type()) {
      case StBoundaryProto::SMALL_ANGLE_CUT_IN:
      case StBoundaryProto::LANE_CHANGE_GAP: {
        disable_pass_time_buffer_id_set.insert(*protected_st_boundary_id);
        break;
      }
      case StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT:
      case StBoundaryProto::NON_PROTECTIVE:
        break;
    }
  }

  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    const bool disable_pass_time_buffer = ContainsKey(
        disable_pass_time_buffer_id_set, st_boundary_with_decision.id());
    DecideTimeBuffersForStBoundary(&st_boundary_with_decision, plan_start_v,
                                   vehicle_geometry_params, speed_traj_manager,
                                   disable_pass_time_buffer);
  }
}

bool CheckIsNearConsecutiveStationaryObjects(
    const StGraph& st_graph,
    const SpacetimeTrajectoryManager& speed_traj_manager,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision) {
  constexpr double kCloseStationaryObjectDistance = 2.5;
  constexpr double kFilterStationrayObjectMaxDs = 100.0;
  constexpr double kFilterStationrayObjectMinDs = 0.0;
  constexpr double kFilterStationrayObjectDlThreshold = 0.1;

  const auto close_stationary_objects = st_graph.GetCloseSpaceTimeObjects(
      st_boundaries_with_decision, speed_traj_manager.stationary_object_trajs(),
      kCloseStationaryObjectDistance);

  int left_stationary_obj_num = 0, right_stationary_obj_num = 0;

  const auto& object_sl_map = st_graph.obj_sl_map();
  for (const auto& close_object : close_stationary_objects) {
    if (close_object.object_type != StBoundaryProto::VEHICLE) {
      continue;
    }
    const auto object_id =
        SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(close_object.id);
    const auto dsdl = object_sl_map.find(object_id);
    if (dsdl == object_sl_map.end()) {
      continue;
    }

    if (dsdl->second.ds < kFilterStationrayObjectMinDs ||
        dsdl->second.ds > kFilterStationrayObjectMaxDs ||
        std::fabs(dsdl->second.dl) < kFilterStationrayObjectDlThreshold) {
      continue;
    }

    if (dsdl->second.dl > 0.01) {
      left_stationary_obj_num++;
    }

    if (dsdl->second.dl < -0.01) {
      right_stationary_obj_num++;
    }
  }

  constexpr int kConsecutiveStationaryObjectsNum = 4;
  return left_stationary_obj_num >= kConsecutiveStationaryObjectsNum ||
         right_stationary_obj_num >= kConsecutiveStationaryObjectsNum;
}

void StationaryCloseObjectSlowdown(
    const bool is_on_highway, const double plan_start_v,
    const DiscretizedPath& path, const PlanPassage& plan_passage,
    const PathSlBoundary& path_sl_boundary, const StGraph& st_graph,
    const SpacetimeTrajectoryManager& speed_traj_manager,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    ConstraintManager& constraint_mgr) {
  const bool is_near_consecutive_stationary_objects =
      CheckIsNearConsecutiveStationaryObjects(st_graph, speed_traj_manager,
                                              st_boundaries_with_decision);
  auto stationary_close_object_constraint =
      GenerateStationaryCloseObjectConstraints(
          st_boundaries_with_decision, st_graph, speed_traj_manager, path,
          plan_passage, path_sl_boundary, plan_start_v, vehicle_geometry_params,
          is_on_highway, is_near_consecutive_stationary_objects);
  for (auto& path_speed_region :
       stationary_close_object_constraint.path_speed_regions) {
    constraint_mgr.AddPathSpeedRegion(std::move(path_speed_region));
  }
  for (auto& path_stop_line :
       stationary_close_object_constraint.path_stop_lines) {
    constraint_mgr.AddPathStopLine(std::move(path_stop_line));
  }
}

void MakePreBrakeDecision(
    const double plan_start_v, const double max_v, const int trajectory_steps,
    const SpacetimeTrajectoryManager& speed_traj_manager,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedVector& preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    SpeedLimitProvider& speed_limit_provider) {
  const auto uncertain_vehicle_speed_limit =
      MakeUncertainVehiclePreBrakeDecision(
          speed_traj_manager, path, vehicle_geometry_params, plan_start_v,
          max_v, kSpeedLimitProviderTimeStep, trajectory_steps,
          preliminary_speed, st_boundaries_with_decision);

  if (uncertain_vehicle_speed_limit.has_value()) {
    speed_limit_provider.AddVtSpeedLimit(SpeedLimitTypeProto::UNCERTAIN_VEHICLE,
                                         *uncertain_vehicle_speed_limit);
  }

  const auto creep_interaction_speed_limit = MakeCreepInteractionDecision(
      speed_traj_manager, path, vehicle_geometry_params, plan_start_v, max_v,
      kSpeedLimitProviderTimeStep, trajectory_steps, preliminary_speed,
      st_boundaries_with_decision);

  if (creep_interaction_speed_limit.has_value()) {
    speed_limit_provider.AddVtSpeedLimit(SpeedLimitTypeProto::CREEP_INTERACTION,
                                         *creep_interaction_speed_limit);
  }
}

absl::Status ComputeBoundaryAndOptimizeSpeed(
    const std::string_view base_name, const double plan_start_v,
    const double plan_start_a, const int knot_num, const double plan_total_time,
    const double plan_time_interval, const double planner_speed_cap,
    const DiscretizedPath& path,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const SpeedPlanningParamsProto& speed_planning_params,
    const LaneChangeStage& lc_stage, const PathApprox& path_approx,
    const KdtreeSegmentMatcher& path_kd_tree, const OffsetRect& vehicle_rect,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedLimitProvider& speed_limit_provider,
    const SpeedVector& preliminary_speed,
    const SpacetimeTrajectoryManager& speed_traj_manager,
    const ConstraintManager& constraint_mgr,
    const SpeedOptimizerObjectManager& opt_obj_mgr,
    SpeedPlanningDebugProto* speed_planning_proto,
    SpeedPlanningParamsProto& new_speed_planning_params,
    SpeedVector* optimized_speed) {
  const double path_step_length = path.at(1).s() - path.at(0).s();
  CHECK_GT(path_step_length, 0.0);

  const SpeedPlanningParamsProto* new_speed_planning_params_ptr =
      &speed_planning_params;
  std::unique_ptr<SpeedPlanningParamsProto>
      dispatched_speed_planning_params_ptr;
  auto dispatched_speed_optimizer_params = DispatchSpeedOptimizerConfig(
      st_boundaries_with_decision, speed_traj_manager, path_approx,
      path_kd_tree, vehicle_rect.radius(), path_step_length,
      static_cast<int>(path.size() - 1), plan_start_v, path.front(),
      vehicle_geometry_params, speed_planning_params.speed_optimizer_params(),
      speed_planning_params.speed_optimizer_config_dispatcher_params());
  if (dispatched_speed_optimizer_params.has_value()) {
    dispatched_speed_planning_params_ptr =
        std::make_unique<SpeedPlanningParamsProto>(speed_planning_params);
    *dispatched_speed_planning_params_ptr->mutable_speed_optimizer_params() =
        std::move(*dispatched_speed_optimizer_params);
    new_speed_planning_params_ptr = dispatched_speed_planning_params_ptr.get();
  }
  new_speed_planning_params = *new_speed_planning_params_ptr;

  const auto speed_bound_map = EstimateSpeedBound(
      speed_limit_provider, preliminary_speed, plan_start_v, planner_speed_cap,
      knot_num, plan_time_interval, base_name);
  const auto reference_speed = GenerateReferenceSpeed(
      GenerateMinSpeedLimitWithLaneAndCurvature(
          FindOrDie(speed_bound_map, SpeedLimitTypeProto::LANE),
          FindOrDie(speed_bound_map, SpeedLimitTypeProto::CURVATURE)),
      plan_start_v,
      new_speed_planning_params.speed_optimizer_params().ref_speed_bias(),
      new_speed_planning_params.speed_optimizer_params()
          .ref_speed_static_limit_bias(),
      spacetime_constraint_params.max_acceleration(),
      spacetime_constraint_params.max_deceleration(), plan_total_time,
      plan_time_interval);
  const auto accel_soft_bound = GenerateAccelerationBound(
      spacetime_constraint_params, constraint_mgr.TrafficGap().acc_gap_target_a,
      constraint_mgr.TrafficGap().dec_gap_target_a, plan_start_a,
      plan_time_interval, knot_num);
  return OptimizeSpeed(base_name, plan_start_v, plan_start_a, plan_total_time,
                       plan_time_interval, opt_obj_mgr, speed_bound_map,
                       path.length(), reference_speed, accel_soft_bound,
                       spacetime_constraint_params, new_speed_planning_params,
                       optimized_speed, speed_planning_proto, lc_stage);
}

bool CheckSpeedPlanningInput(const SpeedPlanningInput& input) {
  if (nullptr == input.path || input.path->size() < 2) {
    return false;
  }
  if (nullptr == input.st_path_points || input.st_path_points->empty()) {
    return false;
  }
  if (nullptr == input.behavior) {
    return false;
  }
  if (nullptr == input.traj_mgr) {
    return false;
  }
  if (nullptr == input.plan_passage) {
    return false;
  }
  if (nullptr == input.constraint_mgr) {
    return false;
  }

  constexpr double kEpsilon = 1.0e-6;
  CHECK_NEAR(input.path->front().x(), input.st_path_points->front().x(),
             kEpsilon);
  CHECK_NEAR(input.path->front().y(), input.st_path_points->front().y(),
             kEpsilon);
  return true;
}

void MakeObstaclesSceneAnalyze(
    const double plan_start_v, const DiscretizedPath& path,
    const LaneChangeStage& lc_stage, const PlanPassage& plan_passage,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ProcessedSpacetimeTrajectoryController& speed_traj_manager,
    const SpeedPlanningParamsProto::ObjectSceneParamsProto& obj_scene_params,
    std::vector<PathPointSemantic>& path_semantics,
    std::vector<DrivingProcess>& driving_process_seq,
    std::vector<TurnTypeInfo>& turn_type_info,
    std::vector<StBoundaryRef>& st_boundaries,
    WorkerThreadManager* thread_pool) {
  int max_analyze_path_index = -1;
  for (const auto& st_boundary : st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;
    for (const auto& overlap_info : st_boundary->overlap_infos()) {
      max_analyze_path_index =
          std::max(max_analyze_path_index, overlap_info.av_end_idx);
    }
  }
  if (max_analyze_path_index >= 0) {
    auto path_semantics_or = AnalyzePathSemantics(path, max_analyze_path_index,
                                                  psmm, nullptr, thread_pool);

    if (path_semantics_or.ok()) {
      AnalyzeStOverlaps(path, *path_semantics_or, psmm, speed_traj_manager,
                        plan_passage, vehicle_geometry_params, lc_stage,
                        av_shapes, plan_start_v, &st_boundaries);
      path_semantics = std::move(*path_semantics_or);
    } else {
      LOG(WARNING)
          << "Path semantic analyzer fails, skip analyzing overlap meta: "
          << path_semantics_or.status().message();
    }
  }

  MakeObjectSceneRecognition(psmm, plan_passage, path, speed_traj_manager,
                             vehicle_geometry_params, obj_scene_params,
                             &st_boundaries, plan_start_v, &driving_process_seq,
                             turn_type_info);
}
}  // namespace

absl::StatusOr<SpeedPlanningOutput> ExecuteSpeedPlanning(
    const SpeedPlanningInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const SpeedPlanningParamsProto& speed_planning_params,
    WorkerThreadManager* thread_pool, EgoFrame* curr_ego_frame) {
  Timer timer(__FUNCTION__);
  Timer start_time;
  double total_time = 0.0;
  double tmp_time = 0.0;
  if (!CheckSpeedPlanningInput(input)) {
    return absl::FailedPreconditionError("SpeedPlanningInput has nullptr.");
  }

  NOA_SPEED_DEBUG->Clear();

  const absl::Cleanup timeout_trigger = [start_time = Timer(), &input]() {
    constexpr double kSpeedPlanningTimeLimitMs = 30.0;
    const auto speed_total_time_ms = start_time.TimeNs() / 1e6;
    if (speed_total_time_ms > kSpeedPlanningTimeLimitMs) {
    }
  };

  auto time_trigger = [&start_time, &total_time,
                       &tmp_time](const std::string& tag) {
    tmp_time = start_time.TimeNs() / 1E6;
    total_time += tmp_time;
    Log2FG::LogDataV2("[speed][time]", tag + ": " + std::to_string(tmp_time));
    start_time.Reset();
  };

  ProcessedSpacetimeTrajectoryController speed_traj_manager(*(input.traj_mgr));
  speed_traj_manager.ModifySpacetimeTrajectory(input, vehicle_geometry_params,
                                               speed_planning_params);

  const int trajectory_steps = GetSpeedPlanningTrajectorySteps(
      input.plan_start_v,
      Mph2Mps(spacetime_constraint_params.default_speed_limit()));

  constexpr double kPathApproxTolerance = 0.05;
  const auto vehicle_rect =
      CreateOffsetRectFromVehicleGeometry(vehicle_geometry_params);
  const auto path_kd_tree = BuildPathKdTree(*input.path);
  const auto path_approx = BuildPathApprox(
      *input.path, vehicle_rect, kPathApproxTolerance, path_kd_tree.get());
  PathApprox* path_approx_for_mirrors_ptr = nullptr;
  std::optional<PathApprox> path_approx_for_mirrors;
  if (FLAGS_planner_use_path_approx_based_st_mapping) {
    path_approx_for_mirrors =
        BuildPathApproxForMirrors(path_approx, vehicle_geometry_params);
    path_approx_for_mirrors_ptr = path_approx_for_mirrors.has_value()
                                      ? &(*path_approx_for_mirrors)
                                      : nullptr;
  }
  const auto av_shapes = BuildAvShapes(vehicle_geometry_params, *input.path);

  time_trigger("check_input");

  auto st_graph = std::make_unique<StGraph>(
      input.path, trajectory_steps, input.plan_start_v,
      spacetime_constraint_params.max_deceleration(), &vehicle_geometry_params,
      &speed_planning_params.st_graph_params(), &av_shapes, path_kd_tree.get(),
      &path_approx, path_approx_for_mirrors_ptr);

  StGraph::StBoundaryOutput st_boundary_output = st_graph->GetStBoundaries(
      speed_traj_manager, *input.leading_trajs, input.consider_lane_change_gap,
      *input.constraint_mgr, input.psmm, input.plan_passage,
      input.path_sl_boundary, input.nudge_object_info, thread_pool);
  auto& st_boundaries = st_boundary_output.st_boundaries;

  time_trigger("st_graph");

  std::vector<PathPointSemantic> path_semantics;
  std::vector<DrivingProcess> driving_process_seq;
  std::vector<TurnTypeInfo> turn_type_info;
  MakeObstaclesSceneAnalyze(
      input.plan_start_v, *input.path, input.lc_stage, *input.plan_passage,
      av_shapes, *input.psmm, vehicle_geometry_params, speed_traj_manager,
      speed_planning_params.obj_scene_params(), path_semantics,
      driving_process_seq, turn_type_info, st_boundaries, thread_pool);

  const auto& av_curr_path = (*input.path).front();
  const auto av_sl = input.plan_passage->QueryFrenetCoordinateAt(
      Vec2d(av_curr_path.x(), av_curr_path.y()));
  const bool is_narrow_near_large_vehicle =
      av_sl.ok() ? IsNarrowNearLargeVehicle(av_sl->s, *input.plan_passage,
                                            *input.path_sl_boundary,
                                            *(input.traj_mgr), st_boundaries,
                                            st_graph->moving_close_trajs())
                 : false;

  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));

  time_trigger("scence_analyze");

  SpeedPlanningOutput output;
  // std::unordered_map<std::string, double> obstacle_style_map;
  // output.agent_status_history = input.agent_status_history;
  // MakeStyleEstimation(speed_planning_params, *input.path, input.plan_start_v,
  //                     input.plan_start_a, speed_traj_manager, input.seq_num,
  //                     st_boundaries_with_decision,
  //                     output.agent_status_history, obstacle_style_map);

  std::vector<MCTSInteractiveResult> mcts_interactive_results;
  MakeMCTSGameDecision(
      speed_planning_params, vehicle_geometry_params, *st_graph, *input.path,
      input.plan_start_v, input.plan_start_a, speed_traj_manager, input.seq_num,
      thread_pool, &st_boundaries_with_decision, mcts_interactive_results);
  Log2FG::LogDataV0("seq_num", input.seq_num);
#if VRU_DEBUG
  DLOG(INFO) << "SEQ NUM: " << input.seq_num;
#endif

  time_trigger("mcts");

  DealDenseTrafficFlowConstraint(
      input.plan_start_v, speed_traj_manager, path_semantics, *input.path,
      vehicle_geometry_params, *st_graph, st_boundaries_with_decision,
      output.constraint_mgr);

  const StandstillDistanceDescriptorInput standstill_distance_descriptor_input{
      .speed_planning_params = &speed_planning_params,
      .stalled_object_ids = input.stalled_objects,
      .planner_semantic_map_manager = input.psmm,
      .lane_path = &input.plan_passage->lane_path(),
      .st_traj_mgr = &speed_traj_manager,
      .constraint_mgr = input.constraint_mgr,
      .extra_follow_standstill_for_large_vehicle =
          PiecewiseLinearFunctionFromProto(
              speed_planning_params
                  .extra_follow_standstill_distance_for_large_vehicle_plf())(
              input.plan_start_v)};
  DealStaticObstacles(
      input.is_open_gap, speed_planning_params.gap_expand_follow_standstill(),
      standstill_distance_descriptor_input, st_boundaries_with_decision);

  std::optional<VtSpeedLimit> ignore_speed_limit = std::nullopt;
  bool is_left_turn_ignore = false;
  const auto ignore_descriptor_input = IgnoreDescriptorInput(
      {.params = &speed_planning_params.ignore_decision_params(),
       .path = input.path,
       .path_semantics = &path_semantics,
       .psmm = input.psmm,
       .st_traj_mgr = &speed_traj_manager,
       .plan_passage = input.plan_passage,
       .vehicle_geometry_params = &vehicle_geometry_params,
       .av_shapes = &av_shapes,
       .current_v = input.plan_start_v,
       .max_v = Mph2Mps(spacetime_constraint_params.default_speed_limit()),
       .time_step = kSpeedLimitProviderTimeStep,
       .trajectory_steps = trajectory_steps,
       .lc_stage = input.lc_stage,
       .driving_process_seq = driving_process_seq,
       .current_acc = input.plan_start_a});
  MakeIgnoreAndPreBrakeDecisionForStBoundaries(
      ignore_descriptor_input, &st_boundaries_with_decision,
      &ignore_speed_limit, &is_left_turn_ignore);

  const double duration = std::clamp(10.0 / input.plan_start_v, 1.0, 15.0);
  DiscretizedPath ego_predict_trajectory = EgoPredictTrajectory(
      input.path->front(), duration, 0.2, input.plan_start_v);

  const auto pre_descriptor_input = DescriptorInput({
      .params = &speed_planning_params.st_boundary_pre_decision_params(),
      .leading_trajs = input.leading_trajs,
      .follower_set = input.follower_set,
      .lane_change_gap = &input.constraint_mgr->TrafficGap(),
      .st_traj_mgr = &speed_traj_manager,
      .path = input.path,
      .vehicle_params = &vehicle_geometry_params,
      .plan_passage = input.plan_passage,
      .current_v = input.plan_start_v,
      .max_v = Mph2Mps(spacetime_constraint_params.default_speed_limit()),
      .time_step = kSpeedLimitProviderTimeStep,
      .trajectory_steps = trajectory_steps,
      .plan_time = input.plan_time,
      .run_act_net_speed_decision = input.run_act_net_speed_decision,
  });

  std::optional<VtSpeedLimit> parallel_cut_in_speed_limit = std::nullopt;
  MakePreDecisionForStBoundaries(pre_descriptor_input,
                                 &st_boundaries_with_decision,
                                 &parallel_cut_in_speed_limit);

  if (FLAGS_planner_enable_cross_blind_close_decision) {
    const CrossBlindInput cross_close_input{
        .vehicle_geom = &vehicle_geometry_params,
        .st_graph = st_graph.get(),
        .st_traj_mgr = &speed_traj_manager,
        .current_v = input.plan_start_v,
        .current_a = input.plan_start_a,
        .path = input.path,
        .ego_predict_path = &ego_predict_trajectory,
        .path_semantics = &path_semantics,
        .plan_passage = input.plan_passage};
    CrossCloseDecider(cross_close_input, &st_boundaries_with_decision,
                      driving_process_seq);
  }

  ModifyTimeBuffersOfStboundaries(input.plan_start_v, speed_traj_manager,
                                  vehicle_geometry_params,
                                  st_boundaries_with_decision);

  OpenLoopSpeedLimit open_loop_speed_limit =
      input.constraint_mgr->OpenLoopSpeedLimits();
  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;
  const PreStboundaryModifierInput pre_st_boundary_modifier_input{
      .vehicle_geom = &vehicle_geometry_params,
      .st_graph = st_graph.get(),
      .st_traj_mgr = &speed_traj_manager,
      .current_v = input.plan_start_v,
      .current_a = input.plan_start_a,
      .path = input.path,
      .path_semantics = &path_semantics,
      .plan_passage = input.plan_passage,
      .follower_set = input.follower_set,
      .leader_set = input.leader_set,
      .lc_stage = input.lc_stage,
      .st_boundaries_with_decision = &st_boundaries_with_decision};
  PreModifyStBoundaries(pre_st_boundary_modifier_input,
                        &st_boundaries_with_decision, &processed_st_objects,
                        &open_loop_speed_limit);

  output.constraint_mgr = *input.constraint_mgr;

  if (speed_planning_params.close_object_params()
          .enable_stationary_close_object_slowdown()) {
    const auto& map_func_id = input.behavior->function_id();
    const bool is_on_highway = input.psmm->IsOnHighway(map_func_id);
    StationaryCloseObjectSlowdown(
        is_on_highway, input.plan_start_v, *input.path, *input.plan_passage,
        *input.path_sl_boundary, *st_graph, speed_traj_manager,
        vehicle_geometry_params, st_boundaries_with_decision,
        output.constraint_mgr);
  }

  time_trigger("pre_decision");

  DLOG(INFO) << "[speed finder] Step 4: Calculate speed limit.";
  const double planner_speed_cap =
      Mph2Mps(spacetime_constraint_params.default_speed_limit());
  const double av_max_acc = spacetime_constraint_params.max_acceleration();
  const SpeedLimitMapInput speed_limit_map_input{
      .speed_limit_config = &speed_planning_params.speed_limit_params(),
      .veh_geo_params = &vehicle_geometry_params,
      .veh_drive_params = &vehicle_drive_params,
      .max_speed_limit = planner_speed_cap,
      .av_max_acc = av_max_acc,
      .is_narrow_near_large_vehicle = is_narrow_near_large_vehicle,
      .av_speed = input.plan_start_v,
      .av_acc = input.plan_start_a,
      .discretized_points = input.path,
      .st_path_points = input.st_path_points,
      .path_sl_boundary = input.path_sl_boundary,
      .plan_passage = input.plan_passage,
      .planner_semantic_map_manager = input.psmm,
      .behavior = input.behavior,
      .lane_change_state = &input.lane_change_state,
      .nudge_object_info = input.nudge_object_info,
      .ego_history = input.ego_history,
      .constraint_mgr = &output.constraint_mgr,
      .distance_info_to_impassable_path_boundaries =
          &st_graph->distance_info_to_impassable_boundaries(),
      .moving_close_trajs = &st_graph->moving_close_trajs(),
      .traj_mgr = &speed_traj_manager,
      .turn_type_info = &turn_type_info,
      .ego_predict_trajectory = &ego_predict_trajectory,
      .path_semantics = &path_semantics,
      .st_boundaries_with_decision = &st_boundaries_with_decision,
      .driving_process_seq = &driving_process_seq,
  };
  auto speed_limit_map =
      GetSpeedLimitMap(speed_limit_map_input, curr_ego_frame);

  std::vector<std::optional<SpeedLimit>> close_traj_speed_limit;
  if (FLAGS_planner_enable_moving_close_traj_speed_limit) {
    close_traj_speed_limit = GetMovingCloseTrajSpeedLimits(
        st_graph->moving_close_trajs(), input.path->length(),
        std::max(0.0, input.plan_start_v), kSpeedLimitProviderTimeStep,
        speed_planning_params.speed_limit_params()
            .moving_close_traj_max_time());
  }

  const VtSpeedLimitMapInput vt_speed_limit_input{
      .traj_steps = trajectory_steps,
      .time_step = kSpeedLimitProviderTimeStep,
      .plan_start_v = input.plan_start_v,
      .plan_start_a = input.plan_start_a,
      .soft_acc_jerk =
          speed_planning_params.speed_limit_params().soft_acc_jerk(),
      .constraint_mgr = input.constraint_mgr,
      .spacetime_constraint_params = &spacetime_constraint_params,
      .parallel_cut_in_speed_limit = parallel_cut_in_speed_limit,
      .ignore_speed_limit = ignore_speed_limit,
  };
  std::map<SpeedLimitTypeProto::Type, VtSpeedLimit> vt_speed_limit_map =
      GetVtSpeedLimitMap(vt_speed_limit_input);

  const bool is_av_occupy_target_lane =
      av_sl.ok() ? std::fabs(av_sl->l) < 3.75 / 2.0 : true;
  const OpenLoopSpeedLimitInput open_loop_speed_limit_input{
      .is_narrow_near_large_vehicle = is_narrow_near_large_vehicle,
      .is_left_turn_ignore = is_left_turn_ignore,
      .is_av_occupy_target_lane = is_av_occupy_target_lane,
      .consider_lane_change_gap = input.consider_lane_change_gap,
      .trajectory_steps = trajectory_steps,
      .plan_start_v = input.plan_start_v,
      .plan_start_a = input.plan_start_a,
      .av_max_acc = av_max_acc,
      .constraint_mgr = input.constraint_mgr,
      .lc_stage = &input.lc_stage,
      .st_boundaries_with_decision = &st_boundaries_with_decision,
  };
  std::optional<VtSpeedLimit> open_loop_vt_limit = GetOpenLoopVtSpeedLimit(
      open_loop_speed_limit_input, open_loop_speed_limit);
  if (open_loop_vt_limit.has_value()) {
    vt_speed_limit_map[SpeedLimitTypeProto::FAST_SPEED_LIMIT] =
        *open_loop_vt_limit;
  }

  SpeedLimitProvider speed_limit_provider(
      std::move(speed_limit_map), std::move(close_traj_speed_limit),
      std::move(vt_speed_limit_map), kSpeedLimitProviderTimeStep);

  time_trigger("speed_limit");

  SpeedVector preliminary_speed;
  InteractiveSpeedDebugProto interactive_speed_debug;
  RETURN_IF_ERROR(MakeSpeedSearchAndInteractiveDecision(
      input.base_name, vehicle_geometry_params, spacetime_constraint_params,
      *st_graph, speed_traj_manager, *input.path, input.plan_start_v,
      input.plan_start_a, speed_planning_params, planner_speed_cap,
      trajectory_steps, &speed_limit_provider, &preliminary_speed,
      &st_boundaries_with_decision, &processed_st_objects,
      &interactive_speed_debug, thread_pool, *input.leading_trajs, av_shapes,
      input.plan_passage, input.lane_change_state, input.nudge_object_info));

  ConfigureSpacetimeBoundaryDebug(st_boundaries_with_decision,
                                  &output.speed_planning_proto);

  MakePreBrakeDecision(
      input.plan_start_v,
      Mph2Mps(spacetime_constraint_params.default_speed_limit()),
      trajectory_steps, speed_traj_manager, *input.path,
      vehicle_geometry_params, preliminary_speed, &st_boundaries_with_decision,
      speed_limit_provider);

  time_trigger("speed_search");

  SpeedVector optimized_speed;
  SpeedPlanningParamsProto new_speed_planning_params;
  const double plan_total_time = kTrajectoryTimeStep * trajectory_steps;
  const int knot_num =
      speed_planning_params.speed_optimizer_params().knot_num();
  CHECK_GT(knot_num - 1, 0);
  const double plan_time_interval = plan_total_time / (knot_num - 1);
  const SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, speed_traj_manager, input.plan_start_v,
      plan_total_time, plan_time_interval, speed_planning_params,
      input.lc_stage);
  RETURN_IF_ERROR(ComputeBoundaryAndOptimizeSpeed(
      input.base_name, input.plan_start_v, input.plan_start_a, knot_num,
      plan_total_time, plan_time_interval, planner_speed_cap, *input.path,
      spacetime_constraint_params, speed_planning_params, input.lc_stage,
      path_approx, *path_kd_tree, vehicle_rect, st_boundaries_with_decision,
      vehicle_geometry_params, speed_limit_provider, preliminary_speed,
      speed_traj_manager, *input.constraint_mgr, opt_obj_mgr,
      &output.speed_planning_proto, new_speed_planning_params,
      &optimized_speed));

  time_trigger("optimize_speed");

  CHECK(!optimized_speed.empty()) << "Optimized speed points is empty!";

  CutoffSpeedByTimeHorizon(&optimized_speed);

  output.trajectory_end_info = SetTrajectoryEndInfo(
      st_boundaries_with_decision, optimized_speed.TotalLength());

  if (new_speed_planning_params.enable_full_stop()) {
    PostProcessSpeedByFullStop(new_speed_planning_params, &optimized_speed);
  }

  if (new_speed_planning_params.enable_front_vehicle_alert()) {
    output.alerted_front_vehicle = GetAlertedFrontVehicleForHMI(
        opt_obj_mgr, speed_traj_manager, input.path->front(), optimized_speed,
        vehicle_geometry_params, input.base_name);
  }

  std::vector<ApolloTrajectoryPointProto> output_trajectory_points;
  output_trajectory_points.reserve(optimized_speed.size());
  RETURN_IF_ERROR(CombinePathAndSpeed(*input.path, true, optimized_speed,
                                      &output_trajectory_points));
  for (const auto& p : output_trajectory_points) {
    *output.speed_planning_proto.add_trajectory() = p;
  }

  output.trajectory_points = std::move(output_trajectory_points);
  std::map<std::string, prediction::ObjectPrediction> predictions_debug;
  *output.speed_planning_proto.mutable_modified_prediction() =
      ConstructModifiedPrediction(processed_st_objects, input.plan_id,
                                  predictions_debug);

  output.considered_st_objects =
      GetConsideredStObjects(st_boundaries_with_decision, speed_traj_manager,
                             std::move(processed_st_objects));

  output.obj_sl_map.clear();
  if (st_graph) {
    output.obj_sl_map = st_graph->obj_sl_map();
  }

  output.speed_planning_proto.set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(input.plan_time));
  DumpSpeedToDebugFrame(
      input.plan_id, input.seq_num, input.path->length(), trajectory_steps,
      mcts_interactive_results, speed_planning_params,
      new_speed_planning_params, ego_predict_trajectory, predictions_debug,
      preliminary_speed, optimized_speed, st_boundaries_with_decision, output);

  if (!FLAGS_planner_send_speed_optimizer_debug) {
    output.speed_planning_proto.clear_speed_optimizer();
  }
  DestroyContainerAsyncMarkSource(std::move(st_graph), std::string{});

  output.st_boundaries_with_decision = std::move(st_boundaries_with_decision);
  output.attention_obj_id = opt_obj_mgr.attention_obj_id();

  time_trigger("post_process");
  Log2FG::LogDataV0("[speed][time]", "total: " + std::to_string(total_time));

  return output;
}

}  // namespace planning
}  // namespace e2e_noa
