#include "plan/main_planner.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/meta/type_traits.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "async/async_util.h"
#include "async/future.h"
#include "async/parallel_for.h"
#include "common/plan_start_point_info.h"
#include "common/planning_macros.h"
#include "common/timer.h"
#include "container/strong_int.h"
#include "context/e2e_planner_utils.h"
#include "context/lane_manager.h"
#include "decision_exploration/assist_util.h"
#include "decision_exploration/candidate_lane_sequences.h"
#include "decision_exploration/decision_exploration.h"
#include "decision_exploration/decision_exploration_input.h"
#include "decision_exploration/decision_exploration_output.h"
#include "decision_exploration/lane_graph/lane_path_finder.h"
#include "decision_exploration/target_lane_path_filter.h"
#include "descriptor/traffic_gap_finder.h"
#include "driving_style.pb.h"
#include "gflags/gflags.h"
#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "maps/lane_point.h"
#include "maps/lane_sequence.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_defs.h"
#include "object/object_vector.h"
#include "object/partial_spacetime_object_trajectory.h"
#include "object/planner_object.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "object/spacetime_trajectory_manager_builder.h"
#include "optimizer.pb.h"
#include "plan/discretized_path.h"
#include "plan/fallback_planner.h"
#include "plan/planner_defs.h"
#include "plan/planner_flags.h"
#include "plan/planner_main_loop_internal.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/planner_util.h"
#include "plan/pnp_util.h"
#include "plan/short_path_check.h"
#include "plan/st_planner.h"
#include "plan/st_planner_output.h"
#include "plan/trajectory_point.h"
#include "plan/trajectory_util.h"
#include "planner_params.pb.h"
#include "planner_status.pb.h"
#include "prediction.pb.h"
#include "prediction/prediction.h"
#include "router/lane_graph/v2/route_lane_graph_builder.h"
#include "router/navi/route_navi_info.h"
#include "router/plan_passage.h"
#include "router/route_manager_output.h"
#include "router/route_sections.h"
#include "router/route_sections_info.h"
#include "scene/construction_scene_identification.h"
#include "selector/selector.h"
#include "selector/selector_input.h"
#include "selector_debug.pb.h"
#include "selector_params.pb.h"
#include "speed_planning.pb.h"
#include "trajectory.pb.h"
#include "trajectory_point.pb.h"
#include "util/lane_path_util.h"
#include "util/lane_point_util.h"
#include "util/map_util.h"
#include "util/planner_semantic_map_util.h"
#include "util/planner_status_macros.h"
#include "util/time_util.h"
#include "util/vehicle_geometry_util.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

using MapConstPtr = ad_e2e::planning::MapConstPtr;
using LaneConstPtr = ad_e2e::planning::LaneConstPtr;
using LaneSequencePtr = ad_e2e::planning::LaneSequencePtr;
using LaneType = ad_e2e::planning::LaneType;
using StationaryObstacle = ad_e2e::planning::StationaryObstacle;

namespace {

bool IsCongestionLane(const ApolloTrajectoryPointProto plan_start_point,
                      const PlannerObjectController* obj_mgr,
                      const bool& lc_left,
                      const ad_e2e::planning::LaneSeqInfoPtr lane_seq_info,
                      const bool& last_lc_push_state) {
  if (!obj_mgr || !lane_seq_info || !(lane_seq_info->lane_seq)) {
    Log2FG::LogDataV0("LCCongestion", "normal fail");
    return false;
  }
  double ego_s = DBL_MAX, ego_l = DBL_MAX;
  const auto laneseq = lane_seq_info->lane_seq;
  const Vec2d ego_pos = Extract2dVectorFromApolloProto(plan_start_point);
  laneseq->GetProjectionDistance(ego_pos, &ego_s, &ego_l);
  const double required_s_max =
      std::fmin(lane_seq_info->dist_to_junction, 50.0);
  const int required_obs_cnt = last_lc_push_state ? 3 : 4;
  const double required_obs_avg_speed = last_lc_push_state ? 60.0 : 55.0;
  Log2FG::LogDataV0("LCCongestion",
                    "required_s_max:" + std::to_string(required_s_max));

  std::vector<PlannerObject> considered_objects;
  for (const auto& obs : obj_mgr->planner_objects()) {
    if (obs.type() != OT_VEHICLE && obs.type() != OT_MOTORCYCLIST &&
        obs.type() != OT_TRICYCLIST && obs.type() != OT_LARGE_VEHICLE) {
      continue;
    }
    if (obs.is_stationary() &&
        obs.prediction().perception_object().obstacle_light().brake_lights() ==
            ObstacleLightType::LIGHT_OFF) {
    }
    double obs_s = DBL_MAX, obs_l = DBL_MAX;
    double obs_length = obs.bounding_box().length();
    laneseq->GetProjectionDistance(obs.pose().pos(), &obs_s, &obs_l);
    if ((ego_s - 20.0 > obs_s + 0.5 * obs_length) ||
        (ego_s + required_s_max < obs_s - 0.5 * obs_length)) {
      continue;
    }
    if ((lc_left && ego_l > obs_l) || (!lc_left && obs_l > ego_l)) {
      continue;
    }
    double dl =
        std::fabs(obs_l - ego_l) - 0.5 * obs.bounding_box().width() - 1.0;
    if (dl > 3.0) {
      continue;
    }
    considered_objects.emplace_back(obs);
  }
  if (considered_objects.size() <= required_obs_cnt) {
    Log2FG::LogDataV0(
        "LaneChange",
        "[LCCongestion]cnt fail:" + std::to_string(considered_objects.size()));
    return false;
  }

  double avg_speed = 0.0;
  for (const auto& obs : considered_objects) {
    avg_speed += obs.pose().v();
    Log2FG::LogDataV0("LaneChange", "[LCCongestion]" + obs.id());
  }
  if (avg_speed / considered_objects.size() >
      required_obs_avg_speed * ad_e2e::planning::Constants::KPH2MPS) {
    Log2FG::LogDataV0(
        "LaneChange",
        "[LCCongestion] v fail:" +
            std::to_string(avg_speed / considered_objects.size()));
    return false;
  }
  return true;
}

PlannerStatus CheckLateralSafety(const std::vector<double>& last_angles,
                                 const LaneSequencePtr& new_tgt_seq,
                                 const std::shared_ptr<EgoState> ego_state) {
  std::vector<double> new_angles;
  Log2FG::LogDataV0("LatUnsafety", "new_angles:");

  GetInterestPointsOfLatSafetyCheck(ego_state->ego_pos(),
                                    ego_state->start_point_velocity(),
                                    new_tgt_seq, &new_angles);

  if (last_angles.size() > 1 && last_angles.size() == new_angles.size()) {
    Log2FG::LogDataV0("LatUnsafety", "compare:");

    for (int i = 0; i < last_angles.size(); i++) {
      if (std::fabs(ad_e2e::planning::math::NormalizeAngle(
              last_angles[i] - new_angles[i])) > M_PI / 4.0) {
        return PlannerStatus(PlannerStatusProto::TARGET_LANE_JUMPED_FAIL,
                             "Lat unsafety!");
      }
    }
  }

  return PlannerStatus(PlannerStatusProto::OK, "");
}

void UpdateLaneChangeNoticeAndJunctionStatus(
    double left_navi_dist_v2, double cur_dist_to_junction, int cur_navi_lc_num,
    ad_e2e::planning::V2TurnInfo::V2DetailTurnType cur_nearest_turn_type,
    PlannerWorldOutput* output) {
  double min_dis_to_junction_or_left_navi =
      std::min(left_navi_dist_v2, cur_dist_to_junction);

  if (min_dis_to_junction_or_left_navi < 1.0) {
    output->has_passed_this_junction = true;
  }

  Log2FG::LogDataV2("LcNotice",
                    "[CurNaviLcNum]: " + std::to_string(cur_navi_lc_num));

  int cur_navi_lc_num_clamped =
      ad_e2e::planning::math::Clamp(cur_navi_lc_num, 0, 2);

  int dist_threshold = 0;
  if (cur_navi_lc_num_clamped == 1) {
    dist_threshold = 70;
  } else if (cur_navi_lc_num_clamped == 2) {
    dist_threshold = 100;
  }

  Log2FG::LogDataV2("LcNotice",
                    "[min_dis_to_junction_or_left_navi]: " +
                        std::to_string(min_dis_to_junction_or_left_navi));
  Log2FG::LogDataV2("LcNotice", "[cur_nearest_turn_type]: " +
                                    std::to_string(cur_nearest_turn_type));

  if (cur_navi_lc_num_clamped > 0 &&
      dist_threshold > min_dis_to_junction_or_left_navi &&
      min_dis_to_junction_or_left_navi > 10.0 &&
      cur_nearest_turn_type !=
          ad_e2e::planning::V2TurnInfo::V2DetailTurnType::CONTINUE) {
    output->if_lc_to_congestion = true;

    if (cur_nearest_turn_type ==
            ad_e2e::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT ||
        cur_nearest_turn_type ==
            ad_e2e::planning::V2TurnInfo::V2DetailTurnType::SLIGHT_LEFT ||
        cur_nearest_turn_type ==
            ad_e2e::planning::V2TurnInfo::V2DetailTurnType::UTURN) {
      output->lc_notice = Notice_LeTurn_Hard;
    } else if (cur_nearest_turn_type ==
                   ad_e2e::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT ||
               cur_nearest_turn_type == ad_e2e::planning::V2TurnInfo::
                                            V2DetailTurnType::SLIGHT_RIGHT ||
               cur_nearest_turn_type == ad_e2e::planning::V2TurnInfo::
                                            V2DetailTurnType::TURN_RIGHT_ONLY) {
      output->lc_notice = Notice_RiTurn_Hard;
    } else {
      output->lc_notice = Notice_General_Hard;
    }
  }
}

absl::StatusOr<PlannerStatus> ProcessPlcResultAndUpdateAlcState(
    const PlcInternalResult::PlcInternalStatus& plc_status,
    const PlannerState& planner_state, const PlannerWorldInput& input,
    const DecisionExplorationOutput& decision_exploration,
    mapping::LanePath& updated_preferred_lane_path, ALCState& new_alc_state,
    DriverAction::LaneChangeCommand& new_lc_cmd_state,
    const std::shared_ptr<EgoState>& ego_state) {
  if (plc_status == PlcInternalResult::kBranchNotFound) {
    updated_preferred_lane_path = mapping::LanePath();
    new_alc_state = ALC_STANDBY_ENABLE;
    new_lc_cmd_state = DriverAction::LC_CMD_NONE;
  } else if (plc_status == PlcInternalResult::kSolidBoundary ||
             plc_status == PlcInternalResult::kUnsafeObject ||
             plc_status == PlcInternalResult::kBranchFailedInternal) {
    new_alc_state = ALC_PREPARE;
  } else {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        new_alc_state,
        UpdateAlcState(
            new_alc_state == ALC_PREPARE ? ALC_ONGOING : new_alc_state,
            ego_state->ego_pos(), ego_state->ego_pos(), ego_state->ego_theta(),
            decision_exploration.plan_passage),
        PlannerStatusProto::SCHEDULER_UNAVAILABLE);
    if (new_alc_state == ALC_STANDBY_ENABLE) {
      updated_preferred_lane_path = mapping::LanePath();
      new_lc_cmd_state = DriverAction::LC_CMD_NONE;
    }
  }

  if (new_alc_state == ALC_PREPARE &&
      planner_state.plc_prepare_start_time.has_value() &&
      absl::ToDoubleSeconds(input.start_point_info->plan_time -
                            *planner_state.plc_prepare_start_time) >=
          FLAGS_planner_paddle_lane_change_max_prepare_time) {
    updated_preferred_lane_path = mapping::LanePath();
    new_alc_state = ALC_STANDBY_ENABLE;
    new_lc_cmd_state = DriverAction::LC_CMD_NONE;
  }

  return PlannerStatus(PlannerStatusProto::OK, "");
}

void UpdateOutputStates(
    const mapping::LanePath& updated_preferred_lane_path,
    ALCState new_alc_state, DriverAction::LaneChangeCommand new_lc_cmd_state,
    LaneChangeReason new_lane_change_reason, const PlannerWorldInput& input,
    const ad_e2e::planning::LaneSequencePtr& scene_target_lane_seq,
    const std::vector<StPlannerOutput>& results,
    const DecisionExplorationOutput& decision_exploration,
    PlannerWorldOutput* output) {
  if (output->plc_result.has_value()) {
    if (!updated_preferred_lane_path.IsEmpty() &&
        new_lc_cmd_state != DriverAction::LC_CMD_NONE) {
      output->selector_state.lane_change_reason =
          LaneChangeReason::MANUAL_CHANGE;
    }

    output->alc_state = new_alc_state;
    output->plc_result->preferred_lane_path =
        std::move(updated_preferred_lane_path);
    output->plc_result->lane_change_command = new_lc_cmd_state;

    Log2FG::LogDataV2(
        "lc_cmd_run",
        "lc_cmd_run" + std::to_string(output->plc_result->lane_change_command));
  }

  if (input.behavior->function_id() == Behavior_FunctionId_LKA &&
      scene_target_lane_seq && scene_target_lane_seq->IsValid()) {
    output->selector_state.lane_change_reason = new_lane_change_reason;
  }

  output->nudge_object_info = results[0].nudge_object_info;
  output->tl_stop_interface = results[0].tl_stop_interface;
  output->speed_state = results[0].speed_state;

  if (input.start_point_info) {
    auto start_point_sl_pt =
        decision_exploration.plan_passage.QueryUnboundedFrenetCoordinateAt(
            Extract2dVectorFromApolloProto(
                input.start_point_info->start_point));
    if (start_point_sl_pt.ok()) {
      output->start_point_frenet = std::move(start_point_sl_pt).value();
    }
  }

  if (input.ego_pos) {
    auto ego_pose_sl_pt =
        decision_exploration.plan_passage.QueryUnboundedFrenetCoordinateAt(
            *input.ego_pos);
    if (ego_pose_sl_pt.ok()) {
      output->ego_pose_frenet = std::move(ego_pose_sl_pt).value();
    }
  }
}

}  // namespace

std::optional<double> ComputeCruisingSpeedLimit(
    const std::optional<double>& noa_cruising_speed_limit, double start_v) {
  if (!noa_cruising_speed_limit.has_value()) {
    return std::nullopt;
  }

  std::optional<double> speed_limit = noa_cruising_speed_limit;

  if (start_v >
      speed_limit.value() + FLAGS_planner_max_drop_cruising_speed_limit) {
    speed_limit = start_v - FLAGS_planner_max_drop_cruising_speed_limit;
  }

  return speed_limit;
}

absl::StatusOr<int> FindRouteTargetIndex(
    absl::Span<const DecisionExplorationOutput> decision_explorations,
    const mapping::LanePath& preferred_lane_path, double lc_num,
    double navi_dist, const LaneConstPtr& nearest_lane, bool is_open_gap) {
  if (!preferred_lane_path.IsEmpty()) {
    absl::flat_hash_set<mapping::ElementId> preferred_lanes(
        preferred_lane_path.lane_ids().begin(),
        preferred_lane_path.lane_ids().end());
    for (int i = 0; i < decision_explorations.size(); ++i) {
      if (preferred_lanes.contains(decision_explorations[i]
                                       .plan_passage.lane_path()
                                       .front()
                                       .lane_id())) {
        return i;
      }
    }
  }

  if (nearest_lane) {
  }

  if (!is_open_gap) {
    return absl::AbortedError("Abort gap select.");
  }

  if (is_open_gap && nearest_lane) {
    for (auto it = decision_explorations.begin();
         it != decision_explorations.end(); ++it) {
      bool is_lc_target = true;
      for (auto lane : it->plan_passage.lane_path().lane_ids()) {
        if (lane == nearest_lane->id()) {
          is_lc_target = false;
          break;
        }
      }
      if (is_lc_target) {
        return std::distance(decision_explorations.begin(), it);
      }
    }
  }

  const auto longest_it = std::max_element(
      decision_explorations.begin(), decision_explorations.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.max_reach_length < rhs.max_reach_length;
      });
  for (auto it = decision_explorations.begin();
       it != decision_explorations.end(); ++it) {
    if (it->max_reach_length == longest_it->max_reach_length &&
        it != longest_it) {
      return absl::AbortedError("More than one target.");
    }
  }

  return std::distance(decision_explorations.begin(), longest_it);
}

std::optional<RouteTargetInfo> FindRouteTargetInfo(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& preferred_lane_path, const Box2d& ego_box,
    absl::Span<const DecisionExplorationOutput> decision_explorations,
    absl::Span<const SpacetimeTrajectoryManager> st_traj_mgr_list,
    double lc_num, double navi_dist, const LaneConstPtr& nearest_lane,
    bool is_open_gap) {
  ASSIGN_OR_RETURN(
      const auto route_target_index,
      FindRouteTargetIndex(decision_explorations, preferred_lane_path, lc_num,
                           navi_dist, nearest_lane, is_open_gap),
      std::nullopt);

  const auto& target_decision_exploration =
      decision_explorations[route_target_index];
  const auto target_lane_path_ext = BackwardExtendLanePath(
      psmm,
      target_decision_exploration.plan_passage.extend_lane_path()
          .BeforeArclength(kLaneChangeCheckForwardLength),
      kLaneChangeCheckBackwardLength);
  auto target_frenet_frame_or = BuildKdTreeFrenetFrame(
      SampleLanePathPoints(psmm, target_lane_path_ext), true);
  if (!target_frenet_frame_or.ok()) return std::nullopt;

  ASSIGN_OR_RETURN(const auto ego_frenet_box,
                   target_frenet_frame_or->QueryFrenetBoxAt(ego_box),
                   std::nullopt);

  return RouteTargetInfo{
      .plan_id = route_target_index + 1,
      .frenet_frame = std::move(target_frenet_frame_or).value(),
      .ego_frenet_box = ego_frenet_box,
      .plan_passage = target_decision_exploration.plan_passage,
      .sl_boundary = target_decision_exploration.sl_boundary,
      .st_traj_mgr = st_traj_mgr_list[route_target_index]};
}

inline bool ShouldConsiderRouteTarget(
    int index, const DecisionExplorationOutput& decision_exploration,
    const std::optional<RouteTargetInfo>& route_target_info) {
  return route_target_info.has_value() &&
         ((index + 1 != route_target_info->plan_id &&
           decision_exploration.lane_change_state.stage() ==
               LaneChangeStage::LCS_NONE) ||
          decision_exploration.lane_change_state.stage() ==
              LaneChangeStage::LCS_PAUSE);
}

void AppendFallbackToResultList(
    const PlannerStatus& fallback_status, FallbackPlannerOutput fallback_result,
    StPlannerDebug fallback_debug,
    std::vector<SpacetimeTrajectoryManager>* st_traj_mgr_list,
    std::vector<PlannerStatus>* status_list,
    std::vector<StPlannerOutput>* results,
    std::vector<StPlannerDebug>* debug_list) {
  st_traj_mgr_list->push_back(std::move(fallback_result.filtered_traj_mgr));
  status_list->push_back(fallback_status);
  debug_list->push_back(std::move(fallback_debug));
  results->emplace_back(StPlannerOutput{
      .decision_exploration = std::move(fallback_result.decision_exploration),
      .path = std::move(fallback_result.path),
      .traj_points = std::move(fallback_result.trajectory_points),
      .st_path_points = std::move(fallback_result.st_path_points),
      .decision_state = std::move(fallback_result.decision_state),
      .considered_st_objects = std::move(fallback_result.considered_st_objects),
      .trajectory_end_info = std::move(fallback_result.trajectory_end_info),
      .tl_stop_interface = std::move(fallback_result.tl_stop_interface),
      .st_boundaries_with_decision =
          std::move(fallback_result.st_boundaries_with_decision),
      .speed_state = std::move(fallback_result.speed_state),
      .plan_id = fallback_result.plan_id});
}

void PreFilterByPreferred(const ALCState alc_state,
                          const mapping::LanePath& preferred_lane_path,
                          const std::vector<StPlannerOutput>& st_results,
                          const VehicleGeometryParamsProto& vehicle_geom,
                          PlcInternalResult* plc_result,
                          std::vector<PlannerStatus>* status_list,
                          LcFeasibility& lc_unable_reason_manual) {
  const auto& original_status_list = *status_list;
  plc_result->status = PlcInternalResult::kOk;
  absl::flat_hash_set<mapping::ElementId> preferred_lanes(
      preferred_lane_path.lane_ids().begin(),
      preferred_lane_path.lane_ids().end());
  int preferred_idx = -1;

  bool disable_all_others = false;
  const absl::Cleanup disable_other_status = [alc_state, &preferred_idx,
                                              &disable_all_others,
                                              &preferred_lanes, status_list]() {
    if (!disable_all_others &&
        !(alc_state == ALC_CROSSING_LANE || alc_state == ALC_RETURNING)) {
      return;
    }
    const std::string err_msg = absl::StrCat(
        "Planner branch ignored: teleop to lanes (",
        absl::StrJoin(preferred_lanes, ", "), ")",
        alc_state == ALC_CROSSING_LANE || alc_state == ALC_RETURNING
            ? absl::StrCat(" with state ", ALCState_Name(alc_state), ".")
            : ".");
    for (int i = 0; i < status_list->size(); ++i) {
      if (i == preferred_idx) continue;
      (*status_list)[i] =
          PlannerStatus(PlannerStatusProto::BRANCH_RESULT_IGNORED, err_msg);
    }
  };

  for (int i = 0; i < original_status_list.size(); ++i) {
    if (!preferred_lanes.contains(
            st_results[i]
                .decision_exploration.plan_passage.lane_path()
                .front()
                .lane_id())) {
      continue;
    }
    if (preferred_idx == -1 ||
        (original_status_list[i].ok() &&
         (!original_status_list[preferred_idx].ok() ||
          st_results[preferred_idx].decision_exploration.is_fallback))) {
      preferred_idx = i;
    }
  }

  if (preferred_idx == -1) {
    plc_result->status = PlcInternalResult::kBranchNotFound;

    lc_unable_reason_manual = LcFeasibility::FEASIBILITY_NO_LANE;
    return;
  }

  const auto& preferred_status = original_status_list[preferred_idx];
  const auto& preferred_result = st_results[preferred_idx];
  const auto& preferred_decision_exploration =
      preferred_result.decision_exploration;
  if (!preferred_status.ok()) {
    if (preferred_status.status_code() !=
        PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      plc_result->status = PlcInternalResult::kBranchFailedInternal;
    } else {
      plc_result->status = PlcInternalResult::kUnsafeObject;
      plc_result->unsafe_object_ids = {
          preferred_result.unsafe_object_ids.begin(),
          preferred_result.unsafe_object_ids.end()};

      plc_result->left_solid_boundary =
          preferred_decision_exploration.lane_change_state.lc_left();

      lc_unable_reason_manual = LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT;
    }
    return;
  }
  const auto preferred_crossed_or = HasTrajectoryCrossedSolidBoundary(
      preferred_decision_exploration.plan_passage,
      preferred_decision_exploration.sl_boundary, preferred_result.traj_points,
      vehicle_geom,
      preferred_decision_exploration.lane_change_state.stage() ==
          LaneChangeStage::LCS_PAUSE);
  if (preferred_crossed_or.ok() && *preferred_crossed_or) {
    bool all_crossed_solid = true;
    for (int i = 0; i < original_status_list.size(); ++i) {
      if (i == preferred_idx || !original_status_list[i].ok()) continue;

      const auto& branch_decision_exploration =
          st_results[i].decision_exploration;
      const auto branch_crossed_or = HasTrajectoryCrossedSolidBoundary(
          branch_decision_exploration.plan_passage,
          branch_decision_exploration.sl_boundary, st_results[i].traj_points,
          vehicle_geom,
          branch_decision_exploration.lane_change_state.stage() ==
              LaneChangeStage::LCS_PAUSE);
      if (branch_crossed_or.ok() && !(*branch_crossed_or)) {
        all_crossed_solid = false;
        break;
      }
    }
  }
  disable_all_others = true;
}

void PreFilterRedundant(const std::vector<StPlannerOutput>& st_results,
                        std::vector<PlannerStatus>* status_list) {
  const auto& original_status_list = *status_list;

  absl::flat_hash_map<mapping::ElementId, bool> id_stage_map;
  for (int i = 0; i < original_status_list.size(); ++i) {
    if (!original_status_list[i].ok() ||
        st_results[i].decision_exploration.is_fallback) {
      continue;
    }
    const auto start_lane_id =
        st_results[i]
            .decision_exploration.plan_passage.lane_path()
            .front()
            .lane_id();
    const auto [lc_executing, _] = id_stage_map.insert({start_lane_id, false});
    if (st_results[i].decision_exploration.lane_change_state.stage() ==
        LaneChangeStage::LCS_EXECUTING) {
      lc_executing->second = true;
    }
  }

  auto& considered_status_list = *status_list;
  for (int i = 0; i < considered_status_list.size(); ++i) {
    if (!considered_status_list[i].ok()) continue;

    const auto& decision_exploration = st_results[i].decision_exploration;
    const auto start_lane_id =
        decision_exploration.plan_passage.lane_path().front().lane_id();
    if (!id_stage_map.contains(start_lane_id)) continue;

    if (decision_exploration.is_fallback) {
      considered_status_list[i] = PlannerStatus(
          PlannerStatusProto::BRANCH_RESULT_IGNORED,
          "Fallback branch ignored: latest correspondence succeeded.");
    }
    if (decision_exploration.lane_change_state.stage() ==
            LaneChangeStage::LCS_PAUSE &&
        FindOrDie(id_stage_map, start_lane_id)) {
      considered_status_list[i] = PlannerStatus(
          PlannerStatusProto::BRANCH_RESULT_IGNORED,
          "LC pause branch ignored: lane change branch passed safety check.");
    }
  }
}

double UpdatePlanningHorizon(const ad_e2e::planning::MapPtr& map,
                             const double& raw_cruising_speed_limit,
                             const double& start_v) {
  std::optional<double> speed_limit = raw_cruising_speed_limit;
  double max_speed_limit = start_v;
  if (speed_limit.has_value()) {
    max_speed_limit = std::fmax(max_speed_limit, speed_limit.has_value());
  }
  if (map && map->route()) {
    const auto& route_sections = map->route()->GetRouteInfo().sections;
    for (const auto& section : route_sections) {
      for (const auto lane_id : section.lane_ids) {
        if (!map->GetLaneById(lane_id)) continue;
        if (lane_id.find("nextvr") != lane_id.npos) break;
        double lane_speed_limit = map->GetLaneById(lane_id)->speed_limit();
        max_speed_limit = std::fmax(max_speed_limit, lane_speed_limit);
      }
    }
  }
  return std::fmax(200.0, max_speed_limit * kPlanningTimeHorizon);
}

void E2EPlanner::init() {
  e2e_planner_context_ = std::make_shared<E2EPlannerContext>();
  ego_state_ = std::make_shared<EgoState>();
  e2e_planner_context_->set_ego_state(ego_state_);

  lane_change_command_update_ =
      std::make_shared<LaneChangeCommandUpdate>(e2e_planner_context_);
  e2e_planner_context_->set_lane_change_command_update(
      lane_change_command_update_);

  lane_manager_ = std::make_shared<LaneManager>(e2e_planner_context_);
  e2e_planner_context_->set_lane_manager(lane_manager_);
}

PlannerStatus E2EPlanner::run(const PlannerWorldInput& input,
                              const PlannerState& planner_state,
                              PlannerWorldOutput* output,
                              WorkerThreadManager* thread_pool) {
  SCOPED_TRACE(__FUNCTION__);

  ego_state_->update(input);
  const auto& plan_start_point = ego_state_->plan_start_point();
  const auto& vehicle_geometry = ego_state_->vehicle_geometry();

  const auto& psmm = *planner_state.planner_semantic_map_manager;
  const auto& smm = *psmm.map_ptr();

  std::optional<double> cruising_speed_limit = ComputeCruisingSpeedLimit(
      input.cruising_speed_limit, ego_state_->start_point_velocity());
  const double planning_horizon =
      UpdatePlanningHorizon(psmm.map_ptr(), *cruising_speed_limit,
                            ego_state_->start_point_velocity());

  const mapping::LanePoint destination;

  lane_change_command_update_->update(input, planner_state, output);

  lane_manager_->update(input, planner_state, output, thread_pool,
                        cruising_speed_limit);
  if (lane_manager_->planner_status().status_code() != PlannerStatusProto::OK) {
    return lane_manager_->planner_status();
  }

  const std::vector<LanePathInfo>& lp_infos = lane_manager_->target_lp_infos();
  const auto& function_id = lane_manager_->function_id();
  const bool is_navi = lane_manager_->is_navi();
  const bool is_lka = lane_manager_->is_lka();
  const std::vector<double>& last_angles = lane_manager_->last_angles();
  auto& prev_target_lane_path_from_start =
      lane_manager_->prev_target_lane_path_from_start();
  auto& prev_lane_path_before_lc_from_start =
      lane_manager_->prev_lane_path_before_lc_from_start();
  const auto& pre_lane_seq_before_lc = lane_manager_->pre_lane_seq_before_lc();
  const auto& scene_target_lane_seq = lane_manager_->scene_target_lane_seq();
  const auto& new_lane_change_reason = lane_manager_->new_lane_change_reason();
  const bool if_continuous_lc_secnario =
      lane_manager_->if_continuous_lc_secnario();
  const bool is_open_gap = lane_manager_->is_open_gap();
  const auto& target_lp_infos = lane_manager_->target_lp_infos();
  const bool if_miss_navi_secnario = lane_manager_->if_miss_navi_secnario();
  const int cur_navi_lc_num = lane_manager_->cur_navi_lc_num();
  const double left_navi_dist_v2 = lane_manager_->left_navi_dist_v2();
  const double left_navi_dist = lane_manager_->left_navi_dist();
  const double cur_dist_to_junction = lane_manager_->cur_dist_to_junction();
  const auto& cur_nearest_turn_type = lane_manager_->cur_nearest_turn_type();
  const auto& last_turn_type_v2 = lane_manager_->last_turn_type_v2();
  const auto& nearest_lane = lane_manager_->nearest_lane();
  const auto& pre_target_lane_seq = lane_manager_->pre_target_lane_seq();

  auto& updated_preferred_lane_path =
      lane_change_command_update_->preferred_lane_path();
  auto& new_alc_state = lane_change_command_update_->new_alc_state();
  auto& new_lc_cmd_state = lane_change_command_update_->new_lc_cmd_state();
  auto& lane_change_style = lane_change_command_update_->lane_change_style();
  const auto pnp_top1_reason = lane_change_command_update_->pnp_top1_reason();

  DecisionExplorationInput decision_exploration_input{
      .psmm = &psmm,
      .vehicle_geom = &vehicle_geometry,
      .st_traj_mgr = input.st_traj_mgr ? &input.st_traj_mgr.value() : nullptr,
      .obj_mgr = input.object_manager ? &input.object_manager.value() : nullptr,
      .obj_history_mgr = &planner_state.object_history_manager,
      .lane_path_infos = &lp_infos,
      .planning_horizon = planning_horizon,
      .destination = &destination,

      .prev_smooth_state = planner_state.prev_smooth_state,
      .plan_start_point = &plan_start_point,
      .station_anchor = &planner_state.station_anchor,
      .start_route_s = 0.0,
      .smooth_result_map = &planner_state.smooth_result_map,
      .prev_target_lane_path_from_start = &prev_target_lane_path_from_start,
      .prev_lane_path_before_lc_from_start =
          &prev_lane_path_before_lc_from_start,
      .prev_lc_state = &planner_state.lane_change_state,
      .cruising_speed_limit = cruising_speed_limit,

      .ego_pos = ego_state_->ego_pos(),
      .miss_navi_scenario = false,
      .continuous_lc_scenario = if_continuous_lc_secnario,

      .is_navi = is_navi,
      .lc_cmd_state = new_lc_cmd_state,
      .behavior = &input.behavior.value(),
      .pnp_top1_reason = pnp_top1_reason,
      .lc_push_dir = planner_state.lc_push_dir};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto de_multi_tasks,
      DecisionExplorationPlanTasks(decision_exploration_input, target_lp_infos,
                                   thread_pool),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  std::optional<absl::Time> last_st_path_plan_start_time;
  if (planner_state.selected_trajectory_optimizer_state_proto.has_value()) {
    last_st_path_plan_start_time = e2e_noa::FromProto(
        planner_state.selected_trajectory_optimizer_state_proto.value()
            .last_plan_start_time());
  }

  const auto path_start_point_info = GetStPathPlanStartPointInformation(
      input.min_path_look_ahead_duration, *input.start_point_info,
      planner_state.previous_trajectory,
      CHECK_NOTNULL(input.planner_params)
          ->trajectory_optimizer_params()
          .trajectory_time_step(),
      last_st_path_plan_start_time);

  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_list;

  st_traj_mgr_list.reserve(de_multi_tasks.size() + 2);
  st_traj_mgr_list.resize(de_multi_tasks.size());

  ParallelFor(
      0, de_multi_tasks.size(), thread_pool,
      [&de_multi_tasks, &input, &st_traj_mgr_list, &thread_pool](int i) {
        SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
            .passage = &de_multi_tasks[i].plan_passage,
            .sl_boundary = &de_multi_tasks[i].sl_boundary,
            .obj_mgr =
                input.object_manager ? &input.object_manager.value() : nullptr};
        st_traj_mgr_list[i] = BuildSpacetimeTrajectoryController(
            st_mgr_builder_input, thread_pool);
      });

  auto route_target_info = FindRouteTargetInfo(
      psmm, updated_preferred_lane_path, ego_state_->ego_box(), de_multi_tasks,
      st_traj_mgr_list, cur_navi_lc_num, left_navi_dist_v2, nearest_lane,
      is_open_gap);

  PushDirection push_dir = PushIntention(
      plan_start_point,
      input.object_manager ? &input.object_manager.value() : nullptr,
      de_multi_tasks, planner_state.lc_push_dir != PushDirection::Push_None,
      if_miss_navi_secnario);
  output->lc_push_dir = push_dir;

  if ((push_dir == PushDirection::Push_Congestion_Left ||
       push_dir == PushDirection::Push_Congestion_Right) &&
      ego_state_->start_point_velocity() < 13.8) {
    lane_change_style = LC_STYLE_RADICAL;
  }
  output->last_lc_style = lane_change_style;

  if (planner_state.lane_change_state.stage() != LCS_NONE) {
    lane_change_style = planner_state.last_lc_style;
  }

  std::vector<PlannerStatus> status_list(de_multi_tasks.size());
  std::vector<StPlannerOutput> results(de_multi_tasks.size());
  std::vector<StPlannerDebug> st_debugs(de_multi_tasks.size());
  std::vector<std::unordered_map<std::string,
                                 std::vector<ad_e2e::planning::AgentState>>>
      agent_status_historys(de_multi_tasks.size());
  std::vector<bool> route_result_list(de_multi_tasks.size());
  for (int i = 0; i < de_multi_tasks.size(); i++) {
    route_result_list[i] =
        ShouldConsiderRouteTarget(i, de_multi_tasks[i], route_target_info);
  }

  PlannerStatus fallback_status;
  FallbackPlannerOutput fallback_result;
  StPlannerDebug fallback_debug;

  DLOG(INFO)
      << "DecisionExplorationr output task size without fallback planner: "
      << de_multi_tasks.size();

  ParallelFor(0, de_multi_tasks.size() + 1, thread_pool, [&](int i) {
    if (i < de_multi_tasks.size()) {
      PushDirection task_push_dir =
          de_multi_tasks[i].lane_change_state.stage() ==
                  LaneChangeStage::LCS_NONE
              ? push_dir
              : PushDirection::Push_None;
      status_list[i] = StPlanner(
          StPlannerInput{
              .semantic_map_manager = &smm,
              .planner_semantic_map_manager = &psmm,
              .plan_id = i,
              .vehicle_params = input.vehicle_params,

              .decision_state = &planner_state.decision_state,
              .initialization_state = &planner_state.initialization_state,
              .trajectory_optimizer_state_proto =
                  planner_state.selected_trajectory_optimizer_state_proto
                          .has_value()
                      ? &planner_state.selected_trajectory_optimizer_state_proto
                             .value()
                      : nullptr,
              .st_planner_object_trajectories =
                  &planner_state.st_planner_object_trajectories,
              .obj_mgr = input.object_manager ? &input.object_manager.value()
                                              : nullptr,
              .start_point_info = input.start_point_info
                                      ? &input.start_point_info.value()
                                      : nullptr,
              .st_path_start_point_info = &path_start_point_info,

              .traffic_light_status_map =
                  input.traffic_light_status_map
                      ? &input.traffic_light_status_map.value()
                      : nullptr,
              .smooth_result_map = &planner_state.smooth_result_map,
              .stalled_objects = input.stalled_objects
                                     ? &input.stalled_objects.value()
                                     : nullptr,
              .scene_reasoning = input.scene_reasoning
                                     ? &input.scene_reasoning.value()
                                     : nullptr,
              .prev_target_lane_path_from_start =
                  &prev_target_lane_path_from_start,
              .time_aligned_prev_traj =
                  input.time_aligned_prev_traj
                      ? &input.time_aligned_prev_traj.value()
                      : nullptr,
              .pnp_infos = input.pnp_infos,
              .lane_change_style = lane_change_style,

              .enable_tl_ok_btn = input.enable_tl_ok_btn,
              .override_passable = input.override_passable,

              .route_target_info =
                  route_result_list[i] ? &route_target_info.value() : nullptr,
              .consider_lane_change_gap = input.consider_lane_change_gap,
              .st_traj_mgr = &st_traj_mgr_list[i],

              .decision_constraint_config =
                  &input.planner_params->decision_constraint_config(),
              .initialization_params =
                  &input.planner_params->initialization_params(),
              .trajectory_optimizer_params =
                  &input.planner_params->trajectory_optimizer_params(),
              .speed_planning_params =
                  &input.planner_params->speed_planning_params(),
              .spacetime_constraint_params =
                  &input.planner_params->spacetime_constraint_params(),
              .planner_functions_params =
                  &input.planner_params->planner_functions_params(),
              .vehicle_models_params =
                  &input.planner_params->vehicle_models_params(),
              .speed_planning_lc_radical_params =
                  &input.planner_params->speed_planning_lc_radical_params(),
              .speed_planning_lc_conservative_params =
                  &input.planner_params
                       ->speed_planning_lc_conservative_params(),
              .trajectory_optimizer_lc_radical_params =
                  &input.planner_params
                       ->trajectory_optimizer_lc_radical_params(),
              .trajectory_optimizer_lc_normal_params =
                  &input.planner_params
                       ->trajectory_optimizer_lc_normal_params(),
              .trajectory_optimizer_lc_conservative_params =
                  &input.planner_params
                       ->trajectory_optimizer_lc_conservative_params(),
              .spacetime_planner_object_trajectories_params =
                  &input.planner_params
                       ->spacetime_planner_object_trajectories_params(),
              .ilq_games_params = &input.planner_params->ilq_games_params(),

              .behavior = &(input.behavior.value()),
              .miss_navi_scenario = if_miss_navi_secnario,
              .obs_history = &(planner_state.object_history_manager),
              .speed_state = &(planner_state.speed_state),
              .cur_navi_lc_num = cur_navi_lc_num,
              .left_navi_dist = left_navi_dist,
              .left_navi_dist_v2 = left_navi_dist_v2,
              .last_turn_type_v2 = last_turn_type_v2,
              .prev_lane_change_stage = planner_state.lane_change_state.stage(),
              .lc_cmd_state = new_lc_cmd_state,
              .push_dir = task_push_dir,
              .nudge_object_info =
                  planner_state.nudge_object_info.has_value()
                      ? &planner_state.nudge_object_info.value()
                      : nullptr,
              .cur_dist_to_junction = cur_dist_to_junction,
              .lc_lead_obj_ids = planner_state.lc_lead_obj_ids,
              .is_open_gap = is_open_gap,
              .ego_history = &planner_state.ego_history,
              .seq_num = planner_state.seq_num,
              .agent_status_history = planner_state.agent_status_history

          },
          std::move(de_multi_tasks[i]), &results[i], &st_debugs[i],
          FLAGS_planner_allow_multi_threads_in_st ? thread_pool : nullptr,
          agent_status_historys[i]);
    } else {
      FallbackPlannerInput fallback_input{
          .psmm = &psmm,
          .start_point_info = input.start_point_info
                                  ? &input.start_point_info.value()
                                  : nullptr,
          .time_aligned_prev_trajectory =
              input.time_aligned_prev_traj
                  ? &input.time_aligned_prev_traj.value()
                  : nullptr,
          .prev_target_lane_path_from_start = &prev_target_lane_path_from_start,
          .prev_length_along_route = planner_state.prev_length_along_route,
          .prev_max_reach_length = planner_state.prev_max_reach_length,
          .station_anchor = &planner_state.station_anchor,
          .prev_smooth_state = planner_state.prev_smooth_state,
          .prev_lane_path_before_lc = &prev_lane_path_before_lc_from_start,
          .obj_mgr =
              input.object_manager ? &input.object_manager.value() : nullptr,
          .st_traj_mgr =
              input.st_traj_mgr ? &input.st_traj_mgr.value() : nullptr,
          .stalled_objects =
              input.stalled_objects ? &input.stalled_objects.value() : nullptr,
          .scene_reasoning =
              input.scene_reasoning ? &input.scene_reasoning.value() : nullptr,
          .prev_lc_state = &planner_state.lane_change_state,

          .pre_decision_state = &planner_state.decision_state,

          .traffic_light_status_map =
              input.traffic_light_status_map
                  ? &input.traffic_light_status_map.value()
                  : nullptr,
          .smooth_result_map = &planner_state.smooth_result_map,

          .cruising_speed_limit = cruising_speed_limit,
          .behavior = &(input.behavior.value()),
          .speed_state = &(planner_state.speed_state),
          .plan_id = i,
          .ego_history = &planner_state.ego_history};

      fallback_status = RunFallbackPlanner(
          fallback_input, *input.vehicle_params,
          input.planner_params->spacetime_constraint_params(),
          input.planner_params->decision_constraint_config(),
          input.planner_params->fallback_planner_params(), &fallback_result,
          &fallback_debug, nullptr);
      Log2FG::LogDataV1("task_info_fallback", "[FallbackPlannerStatus]: " +
                                                  fallback_status.ToString());
    }
  });

  for (int i = 0; i < results.size(); ++i) {
    DLOG(INFO) << "Task num: " << i << "; status: " << status_list[i].ok()
               << "; message: " << status_list[i].message() << "; cost: "
               << st_debugs[i].optimizer_debug_proto.ddp().final_costs().cost();
  }
  DLOG(INFO) << "Fallback planner status: " << fallback_status.ok()
             << "; message: " << fallback_status.message() << "; cost: "
             << fallback_debug.optimizer_debug_proto.ddp().final_costs().cost();

  std::set<std::string> speed_considered_object_ids;
  if (FLAGS_planner_export_all_prediction_to_speed_considered) {
    for (const auto& planner_object : input.object_manager->planner_objects()) {
      speed_considered_object_ids.insert(planner_object.id());
    }
  } else {
    for (int i = 0; i < results.size(); ++i) {
      for (const auto& part_st_traj : results[i].considered_st_objects) {
        speed_considered_object_ids.insert(
            std::string(part_st_traj.st_traj().object_id()));
      }
    }
    for (const auto& part_st_traj : fallback_result.considered_st_objects) {
      speed_considered_object_ids.insert(
          std::string(part_st_traj.st_traj().object_id()));
    }
  }
  output->agent_status_history.clear();
  for (int i = 0; i < agent_status_historys.size(); ++i) {
    for (const auto& [id, agent_status] : agent_status_historys[i]) {
      if (output->agent_status_history.find(id) ==
          output->agent_status_history.end()) {
        output->agent_status_history[id] = agent_status;
      }
    }
  }

  if (fallback_status.ok() && updated_preferred_lane_path.IsEmpty()) {
    const bool st_any_success =
        std::any_of(status_list.begin(), status_list.end(),
                    [](const PlannerStatus& status) { return status.ok(); });
    if (!st_any_success) {
      Log2FG::LogDataV1("task_info_fallback", "AppendFB: est all fail");
      AppendFallbackToResultList(fallback_status, std::move(fallback_result),
                                 std::move(fallback_debug), &st_traj_mgr_list,
                                 &status_list, &results, &st_debugs);
    } else {
      const auto fallback_start_id =
          fallback_result.decision_exploration.plan_passage.lane_path()
              .front()
              .lane_id();
      Log2FG::LogDataV1("task_info_fallback",
                        "AppendFB: fb start_lane:" + fallback_start_id);
      for (const auto& result : results) {
        if (!result.decision_exploration.plan_passage.empty() &&
            result.decision_exploration.plan_passage.lane_path()
                    .front()
                    .lane_id() == fallback_start_id) {
          AppendFallbackToResultList(
              fallback_status, std::move(fallback_result),
              std::move(fallback_debug), &st_traj_mgr_list, &status_list,
              &results, &st_debugs);
          Log2FG::LogDataV1("task_info_fallback", "AppendFB: est successok");
          break;
        }
      }
    }
  }

  std::vector<std::string> task_infos;
  task_infos.reserve(3);
  for (size_t i = 0; i < results.size(); i++) {
    const auto& output_lc = results[i].decision_exploration.lane_change_state;

    task_infos.emplace_back(absl::StrFormat(
        "%s:%s,is_fallback:%d,borrow:%d,lc_state:%d,push_state:%d,pre_push_"
        "state:%d,lc_left:%d,"
        "entered_target_lane:%d,force_merge:%d",
        "prefix", status_list[i].ToString(),
        results[i].decision_exploration.is_fallback,
        results[i].decision_exploration.borrow_lane, output_lc.stage(),
        output_lc.push_state(), output_lc.pre_push_state(), output_lc.lc_left(),
        output_lc.entered_target_lane(), output_lc.force_merge()));
  }
  Log2FG::LogDataV1("TaskInfos", std::move(task_infos));

  for (size_t i = 0; i < results.size(); i++) {
    results[i].decision_exploration.sl_boundary.DumpToDebugFrame(
        std::to_string(results[i].plan_id));
  }

  if (!planner_state.preferred_lane_path.IsEmpty() ||
      !updated_preferred_lane_path.IsEmpty()) {
    output->plc_result = PlcInternalResult();

    if (!updated_preferred_lane_path.IsEmpty()) {
      PreFilterByPreferred(new_alc_state, updated_preferred_lane_path, results,
                           vehicle_geometry, &output->plc_result.value(),
                           &status_list, output->lc_unable_reason);
    }
  }
  PreFilterRedundant(results, &status_list);

  SelectorParamsProto tuned_selector_params;
  if (FLAGS_planner_use_tuned_selector_params) {
    tuned_selector_params = LoadSelectorConfigurationFromFile(
        FLAGS_planner_selector_params_file_address);
  }

  const bool noa_need_lane_change_confirmation = false;

  SelectorFlags selector_flags{
      .planner_dumping_selector_features =
          FLAGS_planner_dumping_selector_features,
      .planner_begin_lane_change_frame = FLAGS_planner_begin_lane_change_frame,
      .planner_begin_signal_frame = FLAGS_planner_begin_signal_frame,
      .planner_enable_lane_change_in_intersection =
          FLAGS_planner_enable_lane_change_in_intersection,
      .planner_enable_cross_solid_boundary =
          FLAGS_planner_enable_cross_solid_boundary,
      .planner_lane_change_style = lane_change_style,
      .planner_need_to_lane_change_confirmation =
          noa_need_lane_change_confirmation,
      .planner_is_bus_model =
          input.planner_params->vehicle_models_params().is_vehicle_bus_model()};

  SelectorInput selector_input{
      .psmm = &psmm,
      .prev_lane_path_from_current = &prev_target_lane_path_from_start,
      .pre_lc_stage = planner_state.lane_change_state.stage(),
      .prev_traj = input.time_aligned_prev_traj
                       ? &input.time_aligned_prev_traj.value()
                       : nullptr,
      .spacetime_constraints =
          &input.planner_params->spacetime_constraint_params(),
      .vehicle_geom = &vehicle_geometry,
      .plan_start_point = &plan_start_point,
      .stalled_objects =
          input.stalled_objects ? &input.stalled_objects.value() : nullptr,

      .plan_time = input.start_point_info->plan_time,
      .alc_confirmation = std::nullopt,

      .selector_state = &planner_state.selector_state,
      .selector_flags = &selector_flags,
      .config = FLAGS_planner_use_tuned_selector_params
                    ? &tuned_selector_params
                    : &input.planner_params->selector_params()};
  DeviateNaviInput deviate_navi_input{
      .psmm = &psmm,
      .func_id = function_id,
      .pre_lc_stage = planner_state.lane_change_state.stage(),
      .vehicle_geom = &vehicle_geometry,
      .plan_start_point = &plan_start_point,
      .pre_target_laneseq = pre_target_lane_seq,
      .pre_target_laneseq_before_lc = pre_lane_seq_before_lc};

  for (int idx = 0; idx < results.size(); ++idx) {
    if (results[idx].decision_exploration.is_expert) continue;
    if (results[idx].decision_exploration.plan_passage.lane_seq_info() !=
        nullptr) {
      double dist_to_junction =
          results[idx]
              .decision_exploration.plan_passage.lane_seq_info()
              ->dist_to_junction;
      for (int idx = 0; idx < results.size(); ++idx) {
        Log2FG::LogDataV2("SelectorTraj", "[Idx]" + std::to_string(idx) +
                                              "\t[DistToJunction]" +
                                              std::to_string(dist_to_junction));
      }
    }
  }

  auto selected_idx_or = SelectTrajectory(
      selector_input, status_list, st_traj_mgr_list, results, input.pnp_infos,
      deviate_navi_input, &output->selector_debug, &output->selector_state);

  DLOG(INFO) << "Dump Selector Features (1: true; 0: false):"
             << selector_flags.planner_dumping_selector_features;
  DLOG(INFO) << "Selector Begin Lane Change Frame:"
             << selector_flags.planner_begin_lane_change_frame;
  DLOG(INFO) << "Selector Begin Signal Frame:"
             << selector_flags.planner_begin_signal_frame;
  DLOG(INFO) << "Selector Need LC Confirmation:"
             << selector_flags.planner_need_to_lane_change_confirmation;

  Log2FG::LogMissionData("Selected_ID",
                         (selected_idx_or.ok())
                             ? std::to_string(results[*selected_idx_or].plan_id)
                             : "-100");
  Log2FG::LogMissionData("seq_num", std::to_string(planner_state.seq_num));
  if (selected_idx_or.ok()) {
    std::swap(results[0], results[*selected_idx_or]);
    std::swap(status_list[0], status_list[*selected_idx_or]);
    std::swap(st_debugs[0], st_debugs[*selected_idx_or]);
    output->curr_selected_ego_frame = results[*selected_idx_or].curr_ego_frame;
    const auto& start_point_info = *input.start_point_info;
    const auto& decision_exploration = results[0].decision_exploration;

    if (output->plc_result.has_value()) {
      const auto plc_status = output->plc_result->status;
      RETURN_PLANNER_STATUS_OR_ASSIGN(
          auto process_status,
          ProcessPlcResultAndUpdateAlcState(
              plc_status, planner_state, input, decision_exploration,
              updated_preferred_lane_path, new_alc_state, new_lc_cmd_state,
              ego_state_),
          PlannerStatusProto::SCHEDULER_UNAVAILABLE);
    }

    UpdateOutputStates(updated_preferred_lane_path, new_alc_state,
                       new_lc_cmd_state, new_lane_change_reason, input,
                       scene_target_lane_seq, results, decision_exploration,
                       output);
  }

  if (!results.empty() &&
      results[0].decision_exploration.plan_passage.lane_seq_info()) {
    const auto& new_tgt_seq =
        results[0].decision_exploration.plan_passage.lane_seq_info()->lane_seq;

    PlannerStatus safety_status =
        CheckLateralSafety(last_angles, new_tgt_seq, ego_state_);
    if (!safety_status.ok()) {
      return safety_status;
    }
  }

  output->st_status_list = std::move(status_list);
  output->st_planner_output_list = std::move(results);
  output->st_planner_debug_list = std::move(st_debugs);
  UpdateLaneChangeNoticeAndJunctionStatus(left_navi_dist_v2,
                                          cur_dist_to_junction, cur_navi_lc_num,
                                          cur_nearest_turn_type, output);

  return selected_idx_or.ok()
             ? OkPlannerStatus()
             : PlannerStatus(PlannerStatusProto::SELECTOR_FAILED,
                             selected_idx_or.status().message());
}

void E2EPlanner::ParseDecisionExplorationOutputToPlannerState(
    const DecisionExplorationOutput& decision_exploration,
    PlannerState* planner_state, const ad_e2e::planning::MapPtr& map) {
  const auto& plan_passage = decision_exploration.plan_passage;
  planner_state->prev_target_lane_path =
      plan_passage.extend_lane_path()
          .BeforeLastOccurrenceOfLanePoint(plan_passage.lane_path().back())
          .AfterArclength(
              decision_exploration.av_frenet_box_on_plan_passage.s_min);

  if (planner_state->prev_target_lane_path.lane_seq() == nullptr) {
    std::vector<ad_e2e::planning::LaneConstPtr> lanes;
    for (const auto& id : planner_state->prev_target_lane_path.lane_ids()) {
      const auto& lane = map->GetLaneById(id);
      if (lane) {
        lanes.emplace_back(lane);
      }
    }
    ad_e2e::planning::LaneSequencePtr pre_lane_seq =
        std::make_shared<ad_e2e::planning::LaneSequence>(lanes);
    planner_state->prev_target_lane_path.set_lane_seq(std::move(pre_lane_seq));
  }

  planner_state->station_anchor =
      plan_passage.FindNearestStationAtS(0.0).GetLanePoint();
  planner_state->prev_length_along_route =
      decision_exploration.length_along_route;
  planner_state->prev_max_reach_length = decision_exploration.max_reach_length;
  planner_state->prev_smooth_state = decision_exploration.should_smooth;

  planner_state->lane_change_state = decision_exploration.lane_change_state;
  planner_state->prev_lane_path_before_lc =
      decision_exploration.lane_path_before_lc;
}

void E2EPlanner::ParseStPlannerOutputToPlannerState(
    const StPlannerOutput& st_planner_output, PlannerState* planner_state) {
  planner_state->decision_state = st_planner_output.decision_state;
  planner_state->initialization_state = st_planner_output.initialization_state;
  planner_state->st_planner_object_trajectories =
      st_planner_output.st_planner_object_trajectories;

  planner_state->selected_trajectory_optimizer_state_proto =
      st_planner_output.trajectory_optimizer_state_proto;
}

PushDirection E2EPlanner::PushIntention(
    const ApolloTrajectoryPointProto plan_start_point,
    const PlannerObjectController* obj_mgr,
    const std::vector<DecisionExplorationOutput>& tasks,
    const bool& last_lc_push_state, const bool& if_miss_navi_secnario) {
  if (last_lc_push_state && plan_start_point.v() > 18.1)
    return PushDirection::Push_None;
  if (!last_lc_push_state && plan_start_point.v() > 16.6)
    return PushDirection::Push_None;
  if (tasks.size() < 2) return PushDirection::Push_None;
  if (tasks[0].lane_change_state.stage() != LaneChangeStage::LCS_EXECUTING &&
      tasks[0].lane_change_state.stage() != LaneChangeStage::LCS_PAUSE) {
    return PushDirection::Push_None;
  }
  if (tasks[1].lane_change_state.stage() != LaneChangeStage::LCS_NONE ||
      tasks[1].borrow_lane) {
    return PushDirection::Push_None;
  }

  const bool lc_left = tasks[0].lane_change_state.lc_left();
  const auto lane_seq_info = tasks[0].plan_passage.lane_seq_info();

  const bool is_congestion_scenario = IsCongestionLane(
      plan_start_point, obj_mgr, lc_left, lane_seq_info, last_lc_push_state);
  Log2FG::LogDataV0("LaneChange", "[PushDir]: is congestion:" +
                                      std::to_string(is_congestion_scenario));

  if (lane_seq_info && !if_miss_navi_secnario) {
    double dist_to_lc_solidline = lc_left
                                      ? lane_seq_info->dist_to_right_solid_line
                                      : lane_seq_info->dist_to_left_solid_line;
    if (dist_to_lc_solidline < 15.0 + plan_start_point.v() * 2.0) {
      Log2FG::LogDataV0("LaneChange", "[PushDir]: none, solid line:" +
                                          std::to_string(dist_to_lc_solidline));
      return PushDirection::Push_None;
    }
  }

  if (lane_seq_info) {
    double dist_to_navi_end = lane_seq_info->dist_to_navi_end_v2;
    double dist_to_bus_lane = lane_seq_info->dist_to_bus_lane;
    if (dist_to_navi_end > 160.0 && lane_seq_info->lc_num <= 1 &&
        dist_to_bus_lane < dist_to_navi_end &&
        dist_to_bus_lane < std::max(5.0 * plan_start_point.v(), 50.0)) {
      Log2FG::LogDataV0("LaneChange", "[PushDir]: none, bus lane:" +
                                          std::to_string(dist_to_bus_lane));
      return PushDirection::Push_None;
    }
  }

  if (obj_mgr && lane_seq_info && !is_congestion_scenario) {
    const auto laneseq = lane_seq_info->lane_seq;
    const double ttc = last_lc_push_state ? 1.5 : 2.0;
    if (laneseq) {
      const Vec2d ego_pos = Extract2dVectorFromApolloProto(plan_start_point);
      double ego_s = DBL_MAX, ego_l = DBL_MAX;
      laneseq->GetProjectionDistance(ego_pos, &ego_s, &ego_l);
      for (const auto& obs : obj_mgr->planner_objects()) {
        if (obs.type() != OT_VEHICLE && obs.type() != OT_MOTORCYCLIST &&
            obs.type() != OT_TRICYCLIST && obs.type() != OT_LARGE_VEHICLE &&
            obs.type() != OT_CYCLIST && obs.type() != OT_BARRIER &&
            obs.type() != OT_CONE && obs.type() != OT_WARNING_TRIANGLE &&
            obs.type() != OT_UNKNOWN_STATIC) {
          continue;
        }
        double obs_s = DBL_MAX, obs_l = DBL_MAX;
        laneseq->GetProjectionDistance(obs.pose().pos(), &obs_s, &obs_l);
        double obs_tail_s = obs_s - 0.5 * obs.bounding_box().length();
        double obs_head_s = obs_s + 0.5 * obs.bounding_box().length();
        double ego_tail_s = ego_s - 2.0;
        double ego_head_s = ego_s + 5.0;
        double dv = obs.pose().v() - plan_start_point.v();
        if ((lc_left && ego_l > obs_l) || (!lc_left && obs_l > ego_l)) {
          continue;
        }
        if (obs.type() == OT_BARRIER || obs.type() == OT_CONE ||
            obs.type() == OT_WARNING_TRIANGLE ||
            obs.type() == OT_UNKNOWN_STATIC) {
          if (obs_tail_s > ego_head_s) {
            return PushDirection::Push_None;
          }
        }

        const bool is_valid_large_veh =
            obs.type() == OT_LARGE_VEHICLE && obs.bounding_box().length() > 6.0;
        const double update_ttc = is_valid_large_veh ? ttc * 1.5 : ttc;

        if ((obs_head_s < ego_tail_s) &&
            ((ego_tail_s - obs_head_s) > (update_ttc * dv))) {
          continue;
        }

        if ((obs_tail_s > ego_head_s) &&
            ((ego_head_s - obs_tail_s) < (update_ttc * dv))) {
          continue;
        }

        double dl =
            std::fabs(obs_l - ego_l) - 0.5 * obs.bounding_box().width() - 1.0;
        if (dl > 3.0) {
          continue;
        }

        return PushDirection::Push_None;
      }
    }
  }

  if (is_congestion_scenario) {
    return lc_left ? PushDirection::Push_Congestion_Left
                   : PushDirection::Push_Congestion_Right;
  }

  return lc_left ? PushDirection::Push_Normal_Left
                 : PushDirection::Push_Normal_Right;
}

}  // namespace e2e_noa::planning
