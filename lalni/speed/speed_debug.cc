#include "speed/speed_debug.h"

#include <algorithm>
#include <cmath>

#include "plan/planner_defs.h"
#include "util/map_util.h"

namespace e2e_noa::planning {

namespace {
static void LogMarker(const std::string& id,
                      zark::e2e_noa::debug::Marker::Type type,
                      const zark::e2e_noa::debug::Marker::Color& color,
                      bool is_closed, const std::vector<double>& xs,
                      const std::vector<double>& ys,
                      zark::e2e_noa::debug::Marker* marker_debug) {
  if (xs.size() != ys.size()) {
    return;
  }
  marker_debug->set_id(id);
  marker_debug->set_type(type);
  marker_debug->mutable_color()->MergeFrom(color);

  marker_debug->mutable_points()->Clear();
  for (int i = 0; i < xs.size(); ++i) {
    auto point = marker_debug->add_points();
    point->set_x(xs[i]);
    point->set_y(ys[i]);
  }
}

void DumpStSpeedToDebugFrame(const std::string& name, const SpeedVector& speed,
                             const int plan_id, const Log2FG::Color color,
                             zark::e2e_noa::debug::Marker* marker_debug) {
  std::vector<double> xs;
  std::vector<double> ys;
  for (auto point : speed) {
    xs.emplace_back(point.t());
    ys.emplace_back(point.s());
  }
  LogMarker(name, zark::e2e_noa::debug::Marker::Type::Marker_Type_LINE, color,
            false, xs, ys, marker_debug);
}

int GetSoftBoundIndex(
    const ::google::protobuf::RepeatedField<double>& repeated_time, double t) {
  return std::distance(
      repeated_time.begin(),
      std::lower_bound(repeated_time.begin(), repeated_time.end(), t));
}

void DumpSoftBoundToDebugFrame(
    const StBoundaryWithDecision& st_boundary_with_decision,
    const SpeedPlanningDebugProto& speed_planning_proto, const int plan_id,
    zark::e2e_noa::debug::Marker* marker_debug) {
  const StBoundary* st_boundary = st_boundary_with_decision.st_boundary();
  const auto decision = st_boundary_with_decision.decision_type();
  if (decision == StBoundaryProto::UNKNOWN ||
      decision == StBoundaryProto::IGNORE) {
    return;
  }

  const auto id =
      GetStBoundaryIntegrationId(*st_boundary_with_decision.st_boundary());
  const auto& speed_opt_debug = speed_planning_proto.speed_optimizer();

  CHECK(decision == StBoundaryProto::YIELD ||
        decision == StBoundaryProto::OVERTAKE ||
        decision == StBoundaryProto::FOLLOW);
  const auto* soft_bound_data =
      (decision == StBoundaryProto::YIELD ||
       decision == StBoundaryProto::FOLLOW)
          ? FindOrNull(speed_opt_debug.soft_s_upper_bound(), id)
          : FindOrNull(speed_opt_debug.soft_s_lower_bound(), id);
  if (!soft_bound_data || soft_bound_data->time_size() < 2) return;

  const double min_t = st_boundary->min_t();
  const double max_t = st_boundary->max_t();
  const int start_idx = GetSoftBoundIndex(soft_bound_data->time(), min_t);

  const int soft_bound_size = soft_bound_data->time_size();
  std::vector<StPoint> stb_st;
  stb_st.reserve(soft_bound_size);

  const auto stb_start_pt =
      st_boundary->GetBoundarySRange(soft_bound_data->time(start_idx));
  if (!stb_start_pt.has_value()) return;
  const double stb_start_point_s = (decision == StBoundaryProto::YIELD ||
                                    decision == StBoundaryProto::FOLLOW)
                                       ? stb_start_pt->second
                                       : stb_start_pt->first;

  stb_st.emplace_back(stb_start_point_s, soft_bound_data->time(start_idx));

  int end_idx = 0.0;
  for (int i = start_idx; i < soft_bound_size; ++i) {
    const double curr_time = soft_bound_data->time(i);
    if (!InRange(curr_time, min_t, max_t)) break;
    end_idx = i;
    stb_st.emplace_back(soft_bound_data->value(i), curr_time);
  }

  const auto stb_end_pt =
      st_boundary->GetBoundarySRange(soft_bound_data->time(end_idx));
  if (!stb_end_pt.has_value()) return;
  const double stb_end_point_s = (decision == StBoundaryProto::YIELD ||
                                  decision == StBoundaryProto::FOLLOW)
                                     ? stb_end_pt->second
                                     : stb_end_pt->first;
  stb_st.emplace_back(stb_end_point_s, soft_bound_data->time(end_idx));

  std::vector<double> xs;
  std::vector<double> ys;
  for (auto point : stb_st) {
    xs.emplace_back(point.t());
    ys.emplace_back(point.s());
  }
  LogMarker(st_boundary->id() + "_softbound",
            zark::e2e_noa::debug::Marker::Type::Marker_Type_LINE, Log2FG::kGray,
            false, xs, ys, marker_debug);
}

void DumpVtSpeedToDebugFrame(const std::string& name, const SpeedVector& speed,
                             const int plan_id, const Log2FG::Color color,
                             zark::e2e_noa::debug::Marker* marker_debug) {
  std::vector<double> xs;
  std::vector<double> ys;
  for (auto point : speed) {
    xs.emplace_back(point.t());
    ys.emplace_back(point.v());
  }
  LogMarker(name, zark::e2e_noa::debug::Marker::Type::Marker_Type_LINE, color,
            false, xs, ys, marker_debug);
}

void DumpAtSpeedToDebugFrame(const std::string& name, const SpeedVector& speed,
                             const int plan_id, const Log2FG::Color color,
                             zark::e2e_noa::debug::Marker* marker_debug) {
  std::vector<double> xs;
  std::vector<double> ys;
  for (auto point : speed) {
    xs.emplace_back(point.t());
    ys.emplace_back(point.a());
  }
  LogMarker(name, zark::e2e_noa::debug::Marker::Type::Marker_Type_LINE, color,
            false, xs, ys, marker_debug);
}

void DumpJtSpeedToDebugFrame(const std::string& name, const SpeedVector& speed,
                             const int plan_id, const Log2FG::Color color,
                             zark::e2e_noa::debug::Marker* marker_debug) {
  std::vector<double> xs;
  std::vector<double> ys;
  for (auto point : speed) {
    xs.emplace_back(point.t());
    ys.emplace_back(point.j());
  }
  LogMarker(name, zark::e2e_noa::debug::Marker::Type::Marker_Type_LINE, color,
            false, xs, ys, marker_debug);
}

void DumpVtSpeedLimitToDebugFrame(
    const SpeedPlanningDebugProto& speed_planning_proto, int plan_id) {
  const std::unordered_map<SpeedLimitTypeProto::Type, Log2FG::Color> types = {
      {SpeedLimitTypeProto::LANE, Log2FG::kWhite},
      {SpeedLimitTypeProto::CURVATURE, Log2FG::kRed},
      {SpeedLimitTypeProto::EXTERNAL, Log2FG::kBlue},
      {SpeedLimitTypeProto::MOVING_CLOSE_TRAJ, Log2FG::kDarkGreen},
      {SpeedLimitTypeProto::UNCERTAIN_PEDESTRAIN, Log2FG::kLightBlue},
      {SpeedLimitTypeProto::UNCERTAIN_VEHICLE, Log2FG::kViolet},
      {SpeedLimitTypeProto::STEER_RATE, Log2FG::kAqua},
      {SpeedLimitTypeProto::NEAR_PARALLEL_VEHICLE, Log2FG::kHotpink},
      {SpeedLimitTypeProto::IGNORE_OBJECT, Log2FG::kMiddleBlueGreen},
      {SpeedLimitTypeProto::CLOSE_CURB, Log2FG::kCoral},
      {SpeedLimitTypeProto::CROSS_CURB, Log2FG::kTiffanyBlue},
      {SpeedLimitTypeProto::DEFAULT, Log2FG::kTiffanyBlue},
      {SpeedLimitTypeProto::SOFT_ACC, Log2FG::kBrown},
      {SpeedLimitTypeProto::APPROACH_CURB, Log2FG::kLime},
      {SpeedLimitTypeProto::TOLL_SPEED_LIMIT, Log2FG::kPurple},
      {SpeedLimitTypeProto::V2_TURN_TYPE, Log2FG::kDarkGreen},
      {SpeedLimitTypeProto::HACK_TURN, Log2FG::kDarkGreen},
      {SpeedLimitTypeProto::RIGHT_TURN_CLOSE, Log2FG::kDarkGreen},
      {SpeedLimitTypeProto::BIG_JUNCTION, Log2FG::kDarkBlue},
      {SpeedLimitTypeProto::FAST_SPEED_LIMIT, Log2FG::kDarkRed},
      {SpeedLimitTypeProto::NEAREST_CLOSE, Log2FG::kGrassGreen},
      {SpeedLimitTypeProto::IN_JUNCTION_T_MAP, Log2FG::kDarkBlue},
      {SpeedLimitTypeProto::TRAFFIC_ACC_GAP, Log2FG::kPink},
      {SpeedLimitTypeProto::CREEP_INTERACTION, Log2FG::kTiffanyBlue}};
  for (const auto& [type, color] : types) {
    const auto type_name = "vt_limit_" + SpeedLimitTypeProto::Type_Name(type);
    const auto* data =
        FindOrNull(speed_planning_proto.speed_optimizer().speed_limit(),
                   SpeedLimitTypeProto::Type_Name(type));
    if (data == nullptr) {
      continue;
    }
    CHECK_EQ(data->time_size(), data->value_size());
    constexpr double kMaxPlotSpeedLimit = 30.0;
    constexpr double kEpsilon = 1.0e-6;
    SpeedVector speed_limit;
    speed_limit.reserve(data->time_size());

    for (int i = 0; i < static_cast<int>(data->time_size()); ++i) {
      const auto time = data->time(i);
      const auto value = data->value(i);
      speed_limit.emplace_back(time, 0.0, std::min(value, kMaxPlotSpeedLimit),
                               0.0, 0.0);
    }
    auto speed_limit_debug =
        NOA_SPEED_DEBUG->mutable_pre_decision()->add_speed_limit();
    DumpVtSpeedToDebugFrame(type_name, speed_limit, plan_id, color,
                            speed_limit_debug);
  }
}

void DumpEgoPredictPathToDebugFrame(const DiscretizedPath& ego_predict_path) {
  if (ego_predict_path.empty()) {
    return;
  }
  std::vector<double> xs;
  std::vector<double> ys;
  for (const auto& point : ego_predict_path) {
    xs.emplace_back(point.x());
    ys.emplace_back(point.y());
  }
  LogMarker("ego_predict_trajectory",
            zark::e2e_noa::debug::Marker::Type::Marker_Type_LINE,
            Log2FG::kOrange, false, xs, ys,
            NOA_SPEED_DEBUG->mutable_pre_decision()
                ->mutable_ego_predict_trajectory());
}

void DumpObjectPredictionToDebugFrame(
    const int plan_id,
    const std::map<std::string, prediction::ObjectPrediction>&
        predictions_debug) {
  const auto& prefix = Log2FG::TaskPrefix(plan_id) + "modified-pre_";
  for (auto& [object_id, obj_pred] : predictions_debug) {
    std::vector<double> xs;
    std::vector<double> ys;
    for (auto point : obj_pred.contour().points()) {
      xs.emplace_back(point.x());
      ys.emplace_back(point.y());
    }
    LogMarker(absl::StrCat(prefix, object_id, "_contour"),
              zark::e2e_noa::debug::Marker_Type_POLYGON,
              Log2FG::kMiddleBlueGreen, false, xs, ys,
              NOA_SPEED_DEBUG->mutable_pre_decision()->add_object_contour());
    for (size_t i = 0; i < obj_pred.trajectories().size(); ++i) {
      auto obj_traj =
          NOA_SPEED_DEBUG->mutable_pre_decision()->add_object_traj();
      xs.clear();
      ys.clear();
      for (auto point : obj_pred.trajectories()[i].points()) {
        xs.emplace_back(point.pos().x());
        ys.emplace_back(point.pos().y());
      }
      LogMarker(absl::StrCat(prefix, object_id, "_traj-", std::to_string(i)),
                zark::e2e_noa::debug::Marker::Type::Marker_Type_LINE,
                Log2FG::kDarkRed, false, xs, ys, obj_traj);
    }
  }
}

}  // namespace

void DumpSpeedToDebugFrame(
    const int plan_id, const uint64_t seq_num, const double path_length,
    const int trajectory_steps,
    const std::vector<MCTSInteractiveResult>& mcts_interactive_results,
    const SpeedPlanningParamsProto& speed_planning_params,
    const SpeedPlanningParamsProto& new_speed_planning_params,
    const DiscretizedPath& ego_predict_path,
    const std::map<std::string, prediction::ObjectPrediction>&
        predictions_debug,
    const SpeedVector& preliminary_speed, const SpeedVector& optimized_speed,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpeedPlanningOutput& output) {
  if (speed_planning_params.enable_online_mcts_debug()) {
    OnlineMCTSDebug(plan_id, seq_num, mcts_interactive_results);
  } else {
    OnlineMCTSSimpleDebug(mcts_interactive_results);
  }
  if (speed_planning_params.enable_offline_mcts_debug()) {
    OfflineMCTSDebug(plan_id, seq_num, st_boundaries_with_decision,
                     mcts_interactive_results);
  }

  DumpEgoPredictPathToDebugFrame(ego_predict_path);
  DumpObjectPredictionToDebugFrame(plan_id, predictions_debug);
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DumpSoftBoundToDebugFrame(
        st_boundary_with_decision, output.speed_planning_proto, plan_id,
        NOA_SPEED_DEBUG->mutable_pre_decision()->add_st_softbound());
    const auto& raw_st_boundary = st_boundary_with_decision.raw_st_boundary();
    const auto& st_boundary = st_boundary_with_decision.st_boundary();

    auto get_log_points = [](const StBoundary* const st_boundary) {
      std::vector<StPoint> log_st_points;
      if (!st_boundary) {
        return log_st_points;
      }
      const auto& lower_points = st_boundary->lower_points();
      const auto& upper_points = st_boundary->upper_points();
      if (lower_points.empty() || upper_points.empty()) {
        return log_st_points;
      }

      for (int i = 0; i < lower_points.size(); i++) {
        log_st_points.push_back(lower_points.at(i));
      }
      for (int i = lower_points.size() - 1; i >= 0; i--) {
        log_st_points.push_back(upper_points.at(i));
      }
      return log_st_points;
    };

    std::string decision_type = "";
    switch (st_boundary_with_decision.decision_type()) {
      case StBoundaryProto::FOLLOW: {
        decision_type = "_F";
        break;
      }
      case StBoundaryProto::YIELD: {
        decision_type = "_Y";
        break;
      }
      case StBoundaryProto::OVERTAKE: {
        decision_type = "_O";
        break;
      }
      case StBoundaryProto::IGNORE: {
        decision_type = "_I";
        break;
      }
      case StBoundaryProto::UNKNOWN: {
        decision_type = "_U";
        break;
      }
      default:
        break;
    }

    std::string ignore_reason = "";
    if (st_boundary_with_decision.decision_type() == StBoundaryProto::IGNORE) {
      ignore_reason = "_" + StBoundaryProto::IgnoreReason_Name(
                                st_boundary_with_decision.ignore_reason());
    }

    const auto log_raw_st_points = get_log_points(raw_st_boundary);
    const auto log_st_points = get_log_points(st_boundary);
    if (!log_raw_st_points.empty()) {
      Log2FG::Color color =
          st_boundary_with_decision.decision_type() == StBoundaryProto::IGNORE
              ? Log2FG::kTiffanyBlue
              : Log2FG::kOrange;
      std::vector<double> xs;
      std::vector<double> ys;
      for (auto point : log_raw_st_points) {
        xs.emplace_back(point.t());
        ys.emplace_back(point.s() - 0.5);
      }
      LogMarker(
          "st_raw_" +
              std::string({raw_st_boundary->id().data(),
                           raw_st_boundary->id().size()}) +
              decision_type + ignore_reason,
          zark::e2e_noa::debug::Marker_Type_POLYGON, color, true, xs, ys,
          NOA_SPEED_DEBUG->mutable_pre_decision()->add_raw_st_boundaries());
    }
    if (!log_st_points.empty()) {
      Log2FG::Color color =
          st_boundary_with_decision.decision_type() == StBoundaryProto::IGNORE
              ? Log2FG::kTiffanyBlue
              : Log2FG::kPurple;
      std::vector<double> xs;
      std::vector<double> ys;
      for (auto point : log_st_points) {
        xs.emplace_back(point.t());
        ys.emplace_back(point.s() - 0.5);
      }
      LogMarker("st_" +
                    std::string(
                        {st_boundary->id().data(), st_boundary->id().size()}) +
                    decision_type + ignore_reason,
                zark::e2e_noa::debug::Marker_Type_POLYGON, color, true, xs, ys,
                NOA_SPEED_DEBUG->mutable_pre_decision()->add_st_boundaries());
    }
  }
  SpeedVector end_of_path(
      {SpeedPoint(0.0, path_length, 0.0, 0.0, 0.0),
       SpeedPoint((trajectory_steps - 1) * kTrajectoryTimeStep, path_length,
                  0.0, 0.0, 0.0)});
  DumpStSpeedToDebugFrame(
      "end_of_path", end_of_path, plan_id, Log2FG::kGray,
      NOA_SPEED_DEBUG->mutable_pre_decision()->mutable_end_of_path());
  DumpStSpeedToDebugFrame("preliminary_speed", preliminary_speed, plan_id,
                          Log2FG::kOrange,
                          NOA_SPEED_DEBUG->mutable_interactive_speed_decision()
                              ->mutable_preliminary_speed_st());
  DumpStSpeedToDebugFrame(
      "optimized_speed", optimized_speed, plan_id, Log2FG::kGreen,
      NOA_SPEED_DEBUG->mutable_optimization()->mutable_optimized_speed_st());
  const auto& comfortable_brake_speed_proto =
      output.speed_planning_proto.speed_optimizer().comfortable_brake_speed();
  if (new_speed_planning_params.speed_optimizer_params()
          .enable_comfort_brake_speed()) {
    SpeedVector comfortable_brake_speed;
    comfortable_brake_speed.reserve(comfortable_brake_speed_proto.size());
    for (const auto& speed_pt : comfortable_brake_speed_proto) {
      comfortable_brake_speed.emplace_back().FromProto(speed_pt);
    }
    DumpStSpeedToDebugFrame("comfortable_brake_speed", comfortable_brake_speed,
                            plan_id, Log2FG::kLightGray,
                            NOA_SPEED_DEBUG->mutable_optimization()
                                ->mutable_comfortable_brake_speed());
  }
  const auto& max_brake_speed_proto =
      output.speed_planning_proto.speed_optimizer().max_brake_speed();
  SpeedVector max_brake_speed;
  max_brake_speed.reserve(max_brake_speed_proto.size());
  for (const auto& speed_pt : max_brake_speed_proto) {
    max_brake_speed.emplace_back().FromProto(speed_pt);
  }
  DumpStSpeedToDebugFrame(
      "max_brake_speed", max_brake_speed, plan_id, Log2FG::kLightGray,
      NOA_SPEED_DEBUG->mutable_optimization()->mutable_max_brake_speed());

  DumpVtSpeedLimitToDebugFrame(output.speed_planning_proto, plan_id);
  DumpVtSpeedToDebugFrame("preliminary_speed ", preliminary_speed, plan_id,
                          Log2FG::kOrange,
                          NOA_SPEED_DEBUG->mutable_interactive_speed_decision()
                              ->mutable_preliminary_speed_vt());
  DumpVtSpeedToDebugFrame(
      "optimized_speed", optimized_speed, plan_id, Log2FG::kGreen,
      NOA_SPEED_DEBUG->mutable_optimization()->mutable_optimized_speed_vt());
  const auto& ref_speed_proto =
      output.speed_planning_proto.speed_optimizer().ref_speed();
  SpeedVector ref_speeds;
  ref_speeds.reserve(ref_speed_proto.size());
  for (const auto& speed_pt : ref_speed_proto) {
    ref_speeds.emplace_back().FromProto(speed_pt);
  }
  DumpVtSpeedToDebugFrame(
      "ref_speed", ref_speeds, plan_id, Log2FG::kMagenta,
      NOA_SPEED_DEBUG->mutable_optimization()->mutable_ref_speed_vt());
  DumpAtSpeedToDebugFrame("preliminary_speed", preliminary_speed, plan_id,
                          Log2FG::kOrange,
                          NOA_SPEED_DEBUG->mutable_interactive_speed_decision()
                              ->mutable_preliminary_speed_at());
  DumpAtSpeedToDebugFrame(
      "optimized_speed", optimized_speed, plan_id, Log2FG::kGreen,
      NOA_SPEED_DEBUG->mutable_optimization()->mutable_optimized_speed_at());
  DumpJtSpeedToDebugFrame("preliminary_speed", preliminary_speed, plan_id,
                          Log2FG::kOrange,
                          NOA_SPEED_DEBUG->mutable_interactive_speed_decision()
                              ->mutable_preliminary_speed_jt());
  DumpJtSpeedToDebugFrame(
      "optimized_speed", optimized_speed, plan_id, Log2FG::kGreen,
      NOA_SPEED_DEBUG->mutable_optimization()->mutable_optimized_speed_jt());
}

}  // namespace e2e_noa::planning
