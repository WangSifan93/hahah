#include "context/lane_manager.h"

#include "behavior.pb.h"
#include "context/ego_state.h"
#include "context/lane_change_command_update.h"
#include "decision_exploration/candidate_lane_sequences.h"
#include "decision_exploration/lane_graph/lane_path_finder.h"
#include "decision_exploration/target_lane_path_filter.h"
#include "driving_style.pb.h"
#include "maps/map_or_die_macros.h"
#include "router/navi/route_navi_info_builder.h"

namespace e2e_noa::planning {
#define LANE_PATH_DEBUG (0)
#if LANE_PATH_DEBUG
static void DumpUpstreamPlannerInfo(const std::vector<LanePathInfo> lane_paths,
                                    const LaneGraph &lane_graph,
                                    const PlannerSemanticMapManager &psmm,
                                    const RouteSectionsInfo &sections_info,
                                    const RouteNaviInfo &aligned_rnavi_info,
                                    const Vec2d &ego_pos, const double &theta) {
  static int seq = 0;
  std::cout << "\nseq: " << seq << std::endl;
  std::cout << "ego_pos: " << ego_pos.x() << ", " << ego_pos.y() << ", "
            << theta << std::endl;
  seq++;

  const auto &map = psmm.map_ptr();
  auto &lanes = map->lanes();
  for (const auto &lane : lanes) {
    if (lane && lane->is_navigation()) {
      std::cout << "Lane ID: " << lane->id() << std::endl;
      const auto &points = lane->points();
      for (const auto &point : points) {
        std::cout << "Point: (" << point.x() << ", " << point.y() << ")"
                  << std::endl;
      }
    }
  }

  const auto &sections = sections_info.section_segments();
  if (map->route()) {
    for (const auto &section : map->route()->GetRouteInfo().sections) {
      std::cout << "navi_priority_lane_id: " << section.navi_priority_lane_id
                << std::endl;
    }
  }

#if 0
  
  const auto& sections = sections_info.section_segments();
  for (auto setction : sections) {
    for (const auto lane_id : setction.lane_ids) {
      std::cout << "lane id: " << lane_id << std::endl;

      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, lane_id, void());
      const std::vector<Vec2d>& lane_vertices = lane_info.points();
      for (auto pos : lane_vertices) {
        std::cout << "route start secs lane point: " << pos.x() << " , "
                  << pos.y() << std::endl;
      }

      for (auto tmp_pair : aligned_rnavi_info.route_lane_info_map) {
        if (tmp_pair.first == lane_id) {
          printf(
              "alined rout lane info: {max_driving_distance: %f, "
              "max_reach_length: %f, "
              "recommend_reach_length: %f, min_lc_num_to_target: %d, "
              "lc_num_within_driving_dist: %d, len_before_merge_lane: %f}\n",
              tmp_pair.second.max_driving_distance,
              tmp_pair.second.max_reach_length,
              tmp_pair.second.recommend_reach_length,
              tmp_pair.second.min_lc_num_to_target,
              tmp_pair.second.lc_num_within_driving_dist,
              tmp_pair.second.len_before_merge_lane);
        }
      }
    }
  }

  
  const auto& layers = lane_graph.layers;
  absl::flat_hash_map<LaneGraph::VertexId, Vec2d> vertex_pos;
  for (int i = 0; i < layers.size(); ++i) {
    std::cout << "Layer id: " << i << std::endl;
    const auto [sec_idx, frac] = layers[i];
    std::cout << "Layer sec_idx: " << sec_idx << ", frac: " << frac
              << std::endl;
    for (const auto lane_id : sections[sec_idx].lane_ids) {
      const Vec2d pos =
          ComputeLanePointPos(psmm, mapping::LanePoint(lane_id, frac));

      std::cout << "Layer point: " << pos.x() << " , " << pos.y() << " , "
                << lane_id << std::endl;
      vertex_pos[ToVertId(i, lane_id)] = pos;

      for (const auto& [to_id, edge_cost] :
           lane_graph.graph.edges_from(ToVertId(i, lane_id))) {
        std::cout << "to_id scond: " << to_id.second << " , " << edge_cost
                  << std::endl;
        std::cout << "to_id first: " << to_id.first << " , " << edge_cost
                  << std::endl;
      }
    }
    const auto rightmost_id = sections[sec_idx].lane_ids.back();
    const double theta =
        (-ComputeLanePointTangent(psmm, mapping::LanePoint(rightmost_id, frac))
              .Perp())
            .FastAngle();
  }

  std::cout << "fork_lk_edges size: " << lane_graph.fork_lk_edges.size()
            << std::endl;
  for (int i = 0; i < 4; ++i) {
    for (auto& tmp : lane_graph.fork_lk_edges) {
      std::cout << "fork_lk_edges: " << tmp.first.second << " , "
                << tmp.second.second << std::endl;
    }
  }

  
  const auto graph = lane_graph.graph;
  for (const auto& vertex : graph.vertices()) {
    if (!vertex_pos.contains(vertex)) continue;
    for (const auto& [to_id, cost] : graph.edges_from(vertex)) {
      if (!vertex_pos.contains(to_id)) continue;

      const auto &from_pos = vertex_pos[vertex], to_pos = vertex_pos[to_id];

      std::cout << "seg cost: " << cost << std::endl;
      std::cout << "Edge from_pos: " << from_pos.x() << " , " << from_pos.y()
                << std::endl;
      std::cout << "Edge to_pos: " << to_pos.x() << " , " << to_pos.y()
                << std::endl;
    }
  }
#endif

  int lane_path_idx = 0;
  for (const auto lane_path_info : lane_paths) {
    std::cout << "Lane path idx: " << lane_path_idx << std::endl;
    lane_path_idx++;
    if (lane_path_info.lane_path().lane_seq() == nullptr) {
      std::cout << "Lane path is null" << std::endl;
      continue;
    }
    for (const auto lane : lane_path_info.lane_path().lane_seq()->lanes()) {
      std::cout << "Lane id: " << lane->id() << std::endl;
      if (lane) {
        const auto &points = lane->center_line().points();
        for (const auto &point : points) {
          std::cout << "Lane path point: (" << point.x() << ", " << point.y()
                    << ")" << std::endl;
        }
      }
    }
  }
}
#endif
bool IsValidLane(const LaneConstPtr &lane) {
  if (!lane || lane->type() == LaneType::LANE_NON_MOTOR ||
      lane->type() == LaneType::LANE_EMERGENCY) {
    return false;
  }
  return true;
}
bool IfMissNaviScenarioForCur(
    const ad_e2e::planning::LaneSeqInfo *lane_seq_info, Vec2d ego_pos,
    e2e_noa::Behavior_FunctionId func_id) {
  if (!lane_seq_info) {
    return false;
  }

  double min_distance_to_end_or_junction = std::numeric_limits<double>::max();
  if (lane_seq_info->dist_to_junction > 10) {
    min_distance_to_end_or_junction = std::min(
        lane_seq_info->dist_to_navi_end_v2, lane_seq_info->dist_to_junction);
  } else {
    min_distance_to_end_or_junction = lane_seq_info->dist_to_navi_end_v2;
  }
  if (func_id == Behavior_FunctionId_CITY_NOA) {
    min_distance_to_end_or_junction = lane_seq_info->dist_to_navi_end;
  }
  if (lane_seq_info->dist_to_junction < 35.0) {
    return false;
  }
  if (lane_seq_info->lc_num == 0) {
    return false;
  }
  if (lane_seq_info->lc_num == 1 && lane_seq_info->dist_to_navi_end < 5.0) {
    return false;
  }
  if (lane_seq_info->lc_num == 2 && lane_seq_info->dist_to_navi_end < 10.0) {
    return false;
  }
  if (min_distance_to_end_or_junction >
      std::max(120.0, lane_seq_info->lc_num * 70.0)) {
    return false;
  }
  if (lane_seq_info->dist_to_navi_end >
          lane_seq_info->dist_to_junction + 20.0 &&
      func_id == Behavior_FunctionId_CITY_NOA) {
    return false;
  }
  auto tgt_lane_seq = lane_seq_info->lane_seq;
  if (!tgt_lane_seq) {
    return false;
  }
  auto nearest_lane = tgt_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
  if (!nearest_lane) {
    return false;
  }
  if (nearest_lane->type() != ad_e2e::planning::LANE_NORMAL &&
      nearest_lane->type() != ad_e2e::planning::LANE_BUS_NORMAL &&
      nearest_lane->type() != ad_e2e::planning::LANE_HOV_NORMAL) {
    return false;
  }

  return true;
}

void LaneManager::update(const PlannerWorldInput &input,
                         const PlannerState &planner_state,
                         PlannerWorldOutput *output,
                         WorkerThreadManager *thread_pool,
                         std::optional<double> &cruising_speed_limit) {
  function_id_ = input.behavior->function_id();
  is_navi_ = function_id_ != Behavior_FunctionId_LKA;
  is_lka_ = function_id_ == Behavior_FunctionId_LKA;

  auto e2e_planner_context = e2e_planner_context_.lock();
  if (!e2e_planner_context) {
    LOG(ERROR) << "E2EPlannerContext is no longer available in LaneManager.";
    return;
  }

  const auto ego_pos = e2e_planner_context->ego_state()->ego_pos();
  const auto ego_theta = e2e_planner_context->ego_state()->ego_theta();
  const auto start_point_velocity =
      e2e_planner_context->ego_state()->start_point_velocity();
  const auto &plan_start_point =
      e2e_planner_context->ego_state()->plan_start_point();

  const auto &psmm = *planner_state.planner_semantic_map_manager;
  const auto &map = psmm.map_ptr();
  // ad_e2e::planning::LaneSequencePtr pre_target_lane_seq = nullptr;//
  auto &lane_change_style =
      e2e_planner_context->lane_change_command_update()->lane_change_style();

  auto &updated_preferred_lane_path =
      e2e_planner_context->lane_change_command_update()->preferred_lane_path();
  const auto &new_lc_cmd_state =
      e2e_planner_context->lane_change_command_update()->new_lc_cmd_state();

  // std::vector<double> last_angles;//

  if (!planner_state.prev_target_lane_path.IsEmpty()) {
    const auto &last_tgt_seq = planner_state.prev_target_lane_path.lane_seq();

    GetInterestPointsOfLatSafetyCheck(ego_pos, start_point_velocity,
                                      last_tgt_seq, &last_angles_);

    pre_target_lane_seq_ =
        map->GetSameLaneSequence(last_tgt_seq, ego_pos.x(), ego_pos.y());
#if 0
    if (function_id_ == Behavior_FunctionId_MAPLESS_NOA ||
        function_id_ == Behavior_FunctionId_LKA) {
      std::vector<ad_e2e::planning::Point2d> debug_match_point;
      pre_target_lane_seq_ = map->GetSameLaneSequenceV2(
          last_tgt_seq, ego_pos.x(), ego_pos.y(), ego_theta,
          debug_pre_target_lane_seq, debug_match_point);
      Log2FG::LogPointsV2("debug_match_point_tgt", Log2FG::kGreen, {},
                           debug_match_point, 10.0);
    }
    Log2FG::LogDataV2("match_debug", debug_pre_target_lane_seq);
    Log2FG::LogDataV0("lc_pnp_prevent_jump",
                      "prev target obtained from prev frame.");
#endif

    if (pre_target_lane_seq_ && !pre_target_lane_seq_->lanes().empty()) {
      std::vector<LaneConstPtr> lanes = pre_target_lane_seq_->lanes();
      std::set<std::string> lane_set;
      for (const auto lane : lanes) {
        if (lane) lane_set.insert(lane->id());
      }
      while (lanes.back()) {
        LaneConstPtr next_lane = map->GetOptimalNextLane(lanes.back(), true);
        if (!next_lane || lane_set.find(next_lane->id()) != lane_set.end()) {
          break;
        }
        lanes.emplace_back(next_lane);
        lane_set.insert(next_lane->id());
      }
      pre_target_lane_seq_ =
          std::make_shared<ad_e2e::planning::LaneSequence>(lanes);
    }
  }
  if (!pre_target_lane_seq_) {
    auto nearest_lane =
        map->GetNearestLane(ego_pos, ego_theta, 3.2, is_navi_, false);
    if (!nearest_lane) {
      nearest_lane = map->GetNearestLane(ego_pos, ego_theta, 4.2, is_navi_,
                                         false, M_PI / 4.0);
      if (!nearest_lane) {
        planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                        "Cant get nearest lane 1!");
        return;
      }
    }
    pre_target_lane_seq_ = map->GetLaneSequence(nearest_lane, is_navi_);
  }

  cruising_speed_limit = UpdateCruisingSpeedLimitByLabel(
      map, start_point_velocity, *cruising_speed_limit, pre_target_lane_seq_);

  // mapping::LanePath prev_target_lane_path_from_start;//
  // mapping::LanePath prev_lane_path_before_lc_from_start;//
  if (pre_target_lane_seq_) {
    std::vector<std::string> lane_ids;
    const auto &nearest_lane = pre_target_lane_seq_->GetNearestLane(ego_pos);
    if (!nearest_lane) {
      planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                      "Cant get nearest lane !");
      return;
    }
    bool find_start = false;
    for (const auto &lane : pre_target_lane_seq_->lanes()) {
      if (lane && nearest_lane && nearest_lane->id() == lane->id()) {
        find_start = true;
      }
      if (!find_start) continue;
      if (!lane || lane->id().empty() || !lane->center_line().IsValid()) break;
      lane_ids.emplace_back(lane->id());
    }
    e2e_noa::mapping::LanePath lane_path(map, lane_ids, 0.0, 1.0);
    prev_target_lane_path_from_start_ = lane_path;
    prev_lane_path_before_lc_from_start_ = lane_path;
  } else {
    planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                    "Get pre_target_lane_seq fail!");
    return;
  }

  // ad_e2e::planning::LaneSequencePtr pre_lane_seq_before_lc = nullptr;//
  if (!planner_state.prev_lane_path_before_lc.IsEmpty()) {
    std::vector<ad_e2e::planning::LaneConstPtr> lanes;
    for (const auto &id : planner_state.prev_lane_path_before_lc.lane_ids()) {
      const auto &lane = map->GetLaneById(id);
      if (lane) {
        lanes.emplace_back(lane);
      }
    }
    ad_e2e::planning::LaneSequencePtr pre_lane_seq =
        planner_state.prev_lane_path_before_lc.lane_seq();

    pre_lane_seq_before_lc_ =
        map->GetSameLaneSequence(pre_lane_seq, ego_pos.x(), ego_pos.y());
#if 0
    if (function_id_ == Behavior_FunctionId_MAPLESS_NOA ||
        function_id_ == Behavior_FunctionId_LKA) {
      std::vector<ad_e2e::planning::Point2d> debug_match_point;
      pre_lane_seq_before_lc_ = map->GetSameLaneSequenceV2(
          pre_lane_seq, ego_pos.x(), ego_pos.y(), ego_theta,
          debug_pre_lane_seq_before_lc, debug_match_point);
      Log2FG::LogPointsV2("debug_match_point_before", Log2FG::kOrange, {},
                          debug_match_point, 10.0);
    }
    Log2FG::LogDataV2("match_debug", debug_pre_lane_seq_before_lc);
#endif

    if (pre_lane_seq_before_lc_ && !pre_lane_seq_before_lc_->lanes().empty()) {
      std::vector<LaneConstPtr> lanes = pre_lane_seq_before_lc_->lanes();
      std::set<std::string> lane_set;
      for (const auto lane : lanes) {
        if (lane) lane_set.insert(lane->id());
      }
      while (lanes.back()) {
        LaneConstPtr next_lane = map->GetOptimalNextLane(lanes.back(), true);
        if (!next_lane || lane_set.find(next_lane->id()) != lane_set.end()) {
          break;
        }
        lanes.emplace_back(next_lane);
        lane_set.insert(next_lane->id());
      }
      pre_lane_seq_before_lc_ =
          std::make_shared<ad_e2e::planning::LaneSequence>(lanes);
    }
    if (pre_lane_seq_before_lc_) {
      std::vector<std::string> lane_ids;
      const auto &nearest_lane =
          pre_lane_seq_before_lc_->GetNearestLane(ego_pos);
      if (!nearest_lane) {
        planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                        "Cant get nearest lane !");
        return;
      }
      bool find_start = false;
      for (const auto &lane : pre_lane_seq_before_lc_->lanes()) {
        if (lane && nearest_lane && nearest_lane->id() == lane->id()) {
          find_start = true;
        }
        if (!find_start) continue;
        if (!lane || lane->id().empty() || !lane->center_line().IsValid())
          break;
        lane_ids.emplace_back(lane->id());
      }
      mapping::LanePath lane_path(map, lane_ids, 0.0, 1.0);
      prev_lane_path_before_lc_from_start_ = lane_path;
    }
  }

  //== == == == == == == == == == == == == == == == == == == == == == == == ==
  //== == == == == == == == == == == == == == == == == == == == =

  std::set<std::string> navi_start_lanes;
  auto &navi_start = map->route()->navi_start();
  if (!navi_start.section_id.empty()) {
    const auto &navi_section = map->GetSectionById(navi_start.section_id);
    if (navi_section) {
      for (auto &id : navi_section->lanes()) {
        navi_start_lanes.insert(id);
      }
    }
  }

  auto nearest_lane_ =  //------------------------------------------
      map->GetNearestLane(ego_pos, ego_theta, 3.2, is_navi_, false);
  if (!nearest_lane_) {
    nearest_lane_ = map->GetNearestLane(ego_pos, ego_theta, 4.2, is_navi_,
                                        false, M_PI / 4.0);
    if (!nearest_lane_) {
      planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                      "Cant get nearest lane !");
      return;
    }
  }
  ad_e2e::planning::LaneConstPtr start_lane = nullptr;
  double min_dist = DBL_MAX;
  for (const auto &lane_id : navi_start_lanes) {
    const auto &lane = map->GetLaneById(lane_id);
    if (!lane) continue;
    double s, l;
    if (!lane->center_line().GetProjection(ego_pos, &s, &l)) continue;
    if (std::abs(l) < min_dist) {
      start_lane = lane;
      min_dist = std::abs(l);
    }
  }

  if (is_navi_ && start_lane == nullptr) {
    planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                    "Can't find nearest lane!");
    return;
  }
  if (!is_navi_) {
    start_lane = nearest_lane_;
  }
  if (map->route()) {
    map->route()->UpdateNaviPriorityLanes(nearest_lane_);
  }
  Log2FG::LogVehData("[LaneDecision] ego_lane",
                     "start_lane: " + start_lane->id() +
                         ", nearest_lane: " + nearest_lane_->id());

  // ad_e2e::planning::LaneSequencePtr scene_target_lane_seq = nullptr;//
  // LaneChangeReason new_lane_change_reason = LaneChangeReason::NO_CHANGE;//
  const std::optional<int> tunnel_status = input.behavior->tunnel();
  if (function_id_ == Behavior_FunctionId_LKA) {
    HandleSceneLcIntent(
        &plan_start_point, input.vehicle_params, input.cruising_speed_limit,
        function_id_, tunnel_status, planner_state.lane_change_command,
        &planner_state.lane_change_state, &psmm, ego_pos, nearest_lane_,
        *input.object_manager, pre_target_lane_seq_, input.pnp_infos, false,
        input.selector_state->lane_change_reason, new_lane_change_reason_,
        scene_target_lane_seq_);
  }

  if (!input.route_sections_from_start.has_value()) {
    planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                    "sections_info from start hasn't value!");
    return;
  }
  if (!input.route_sections_from_current.has_value()) {
    planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                    "sections_info from current hasn't value!");
    return;
  }

  const RouteSectionsInfo route_sections_info_from_start(
      psmm, &(*input.route_sections_from_start));
  if (!route_sections_info_from_start.IsValid()) {
    planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                    "Input RouteSectionsInfo is empty!");
    return;
  }
  // RETURN_PLANNER_STATUS_OR_ASSIGN(
  //     const auto route_section_in_horizon,
  //     ClampRouteSectionsBeforeArcLength(
  //         psmm, *input.route_sections_from_start,
  //         input.route_sections_from_start->planning_horizon(psmm)),
  //     PlannerStatusProto::BUILD_DRIVING_MAP_FAILED);

  const auto route_section_in_horizon = ClampRouteSectionsBeforeArcLength(
      psmm, *input.route_sections_from_start,
      input.route_sections_from_start->planning_horizon(psmm));
  if (!route_section_in_horizon.ok()) {
    planner_status_ =
        PlannerStatus(PlannerStatusProto::BUILD_DRIVING_MAP_FAILED,
                      route_section_in_horizon.status().message());
    return;
  }

  absl::StatusOr<RouteNaviInfo> route_navi_info = CalcNaviInfoByLaneGraph(
      *psmm.map_ptr(), route_sections_info_from_start, {}, 1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, route_sections_info_from_start, *input.object_manager,
      *input.stalled_objects, input.avoid_lanes);

  const auto aligned_route_navi_info = AlignRouteNaviInfoWithCurrentSections(
      psmm, *input.route_sections_from_current,
      route_section_in_horizon.value(), *route_navi_info);

  // RETURN_PLANNER_STATUS_OR_ASSIGN(
  //     auto lp_infos_cr,
  //     e2e_noa::planning::FindBestLanePathsFromStart(
  //         psmm, route_sections_info_from_start, aligned_route_navi_info,
  //         lane_graph, thread_pool),
  //     PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE);

  auto lp_infos_cr = e2e_noa::planning::FindBestLanePathsFromStart(
      psmm, route_sections_info_from_start, aligned_route_navi_info, lane_graph,
      thread_pool);
  if (!lp_infos_cr.ok()) {
    planner_status_ =
        PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
                      lp_infos_cr.status().message());
    return;
  }

  if (lp_infos_cr.value().empty()) {
    planner_status_ =
        PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
                      "No viable candidate route found to destination.");
    return;
  }

  const auto target_lp_infos_cr = FilterMultipleTargetLanePath(
      psmm, nearest_lane_, route_sections_info_from_start,
      aligned_route_navi_info, prev_target_lane_path_from_start_,
      plan_start_point, updated_preferred_lane_path, &lp_infos_cr.value());
#if LANE_PATH_DEBUG
  DumpUpstreamPlannerInfo(target_lp_infos_cr, lane_graph, psmm,
                          route_sections_info_from_start,
                          aligned_route_navi_info, ego_pos, ego_theta);
#endif
  if (target_lp_infos_cr.empty()) {
    planner_status_ =
        PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
                      "No valid target lane path from the current section.");
    return;
  }

  PNPInfos final_pnp_res;
  std::string lane_decison_str = "[PnPLanePath]:";
  for (const auto &target_lp_info : target_lp_infos_cr) {
    if (target_lp_info.lane_seq() == nullptr) continue;
    auto *pnp_info = final_pnp_res.add_infos();
    for (const auto &lane : target_lp_info.lane_seq()->lanes()) {
      if (lane) pnp_info->add_target_lane_sequence_ids(lane->id());
    }
    pnp_info->set_probability(1.0);
    pnp_info->set_lc_reason(target_lp_info.start_lane_id() == start_lane->id()
                                ? LcReason::LC_REASON_NONE
                                : LcReason::LC_REASON_FOR_NAVI);
    lane_decison_str += " " + target_lp_info.start_lane_id();
  }
  Log2FG::LogVehData("[LaneDecision] start_lane", lane_decison_str);
  if (final_pnp_res.infos().empty()) {
    planner_status_ =
        PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
                      "candidate lane seq is empty!.");
    return;
  }

  std::vector<std::vector<ad_e2e::planning::LaneConstPtr>> candidate_lane_seqs;
  std::vector<int> candidate_lane_seqs_probabilty;

  double ego_s_offset = 0.0;
  double ego_l_offset = 0.0;
  nearest_lane_->center_line().GetProjection(ego_pos, &ego_s_offset,
                                             &ego_l_offset);
  // bool if_continuous_lc_secnario = false;
  if (!ad_e2e::planning::DecisionExplorationCandidateLaneSequences(
          ego_pos, map, start_lane, nearest_lane_, final_pnp_res,
          new_lc_cmd_state, &updated_preferred_lane_path, &candidate_lane_seqs,
          &candidate_lane_seqs_probabilty,
          planner_state.lane_change_state.stage(), pre_target_lane_seq_,
          scene_target_lane_seq_, is_navi_, is_lka_, function_id_, ego_theta,
          ego_l_offset, if_continuous_lc_secnario_,
          planner_state.lane_change_state, planner_state.auto_drive_counter)) {
    planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                    "Can't Get Candidate laneseqs!");
    return;
  }

#if 0
  for (const auto& target_lp_info : target_lp_infos_cr) {
    if(target_lp_info.lane_seq()== nullptr) continue;
    candidate_lane_seqs.emplace_back(target_lp_info.lane_seq()->lanes());
    candidate_lane_seqs_probabilty.emplace_back(0.8);
  }
  if(candidate_lane_seqs.empty()){
    return PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
      "candidate lane seq is empty!.");
  }
#endif

  bool auto_model = input.auto_model;
  if (planner_state.lane_change_state.stage() != LCS_NONE) auto_model = true;
  Log2FG::LogDataV0("LancChange", "[Lc Mode]: " + std::to_string(auto_model));

  std::vector<std::vector<ad_e2e::planning::LaneConstPtr>>
      candidate_lane_seqs_select;
  if (auto_model || !updated_preferred_lane_path.IsEmpty()) {
    for (const auto &candidate_lane_seq : candidate_lane_seqs) {
      candidate_lane_seqs_select.emplace_back(candidate_lane_seq);
    }
  } else if (!candidate_lane_seqs.empty() && !auto_model &&
             updated_preferred_lane_path.IsEmpty()) {
    const auto &nearest_lane = pre_target_lane_seq_->GetNearestLane(ego_pos);
    Log2FG::LogDataV0("LaneChange",
                      "[Lc Mode][Cloest Lane]: " + nearest_lane->id());
    for (const auto &candidate_lane_seq : candidate_lane_seqs) {
      bool is_lc_seq = true;
      for (const auto &lane : candidate_lane_seq) {
        if (lane && lane->id() == nearest_lane->id()) {
          is_lc_seq = false;
          break;
        }
      }
      if (!is_lc_seq) {
        candidate_lane_seqs_select.emplace_back(candidate_lane_seq);
      }
    }
  } else {
    for (int i = 0; i < candidate_lane_seqs.size(); i++) {
      bool is_lc_seq = true;
      bool is_split = false;
      for (const auto &lane : candidate_lane_seqs[i]) {
        if (lane && lane->id() == nearest_lane_->id()) {
          is_lc_seq = false;
          break;
        }
      }
      for (const auto &lane : candidate_lane_seqs[i]) {
        if (lane && lane->is_split()) {
          is_split = true;
          break;
        }
      }
      if (!is_split || (is_split && is_lc_seq) || (is_split && !is_lc_seq)) {
        candidate_lane_seqs_select.emplace_back(candidate_lane_seqs[i]);
      }
    }
  }

  // bool is_open_gap = true;
  if (!candidate_lane_seqs_select.empty() &&
      !candidate_lane_seqs_select.front().empty()) {
    for (const auto &lane : candidate_lane_seqs_select.front()) {
      if (lane && lane->id() == nearest_lane_->id()) {
        is_open_gap_ = false;
        break;
      }
    }
  }

  if (planner_state.nudge_object_info.has_value() &&
      planner_state.nudge_object_info.value().nudge_state ==
          NudgeObjectInfo::NudgeState::BORROW) {
    is_open_gap_ = false;
  }
  Log2FG::LogVehData("LaneChange",
                     "[IsOpenGap]: " + std::to_string(is_open_gap_));
  lane_decison_str = "[CandidateLaneSeq]:";
  for (const auto &candidate_lane_seq : candidate_lane_seqs_select) {
    if (candidate_lane_seq.empty()) continue;
    const auto &lane = candidate_lane_seq.front();
    lane_decison_str += " " + (lane ? lane->id() : "null");
  }
  Log2FG::LogVehData("[LaneDecision] start_lane", lane_decison_str);

  //=========================================================================================================================
  std::vector<e2e_noa::mapping::LanePath> lane_paths;
  // std::vector<LanePathInfo> target_lp_infos;//
  std::vector<ad_e2e::planning::LaneSeqInfo> lane_seq_infos;

  // bool if_miss_navi_secnario = false;//
  output->if_continuous_lc = if_continuous_lc_secnario_;
  int cur_section_lane_num = 0;
  int cur_lane_position = -1;
  // int cur_navi_lc_num = 0;//
  // double left_navi_dist_v2 = 999.0;//
  // double left_navi_dist = 999.0;//
  // double cur_dist_to_junction = DBL_MAX;//
  // ad_e2e::planning::V2TurnInfo::V2DetailTurnType cur_nearest_turn_type =
  //     ad_e2e::planning::V2TurnInfo::V2DetailTurnType::NONE;//
  // ad_e2e::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2 =
  //     ad_e2e::planning::V2TurnInfo::V2DetailTurnType::NONE;//
  ad_e2e::planning::V2TurnInfo::V2TurnType turn_type_v2 =
      ad_e2e::planning::V2TurnInfo::V2TurnType::UNKNOWN;
  LaneSequencePtr top1_seq = nullptr;
  bool first_check_flag = true;

  for (const auto &candidate_lane_seq : candidate_lane_seqs_select) {
    std::vector<e2e_noa::mapping::ElementId> lane_id_infos;
    double accm_s = 0.0;
    const ad_e2e::planning::LaneConstPtr start_lane =
        *(candidate_lane_seq.begin());
    for (const auto &lane : candidate_lane_seq) {
      if (!lane || lane->id().empty() || !lane->center_line().IsValid() ||
          std::count(lane_id_infos.begin(), lane_id_infos.end(), lane->id())) {
        if (lane) {
          LOG(ERROR) << "lane id = " << lane->id()
                     << " valid = " << lane->center_line().IsValid();
        } else {
          LOG(ERROR) << "lane is null, fatal error";
        }
        break;
      }
      lane_id_infos.emplace_back(lane->id());
      accm_s += lane->curve_length();
    }

    if (lane_id_infos.empty()) {
      continue;
    }
    if (first_check_flag) {
      first_check_flag = false;
      top1_seq =
          std::make_shared<ad_e2e::planning::LaneSequence>(candidate_lane_seq);
    }

    double start_fraction = 0.0;
    const auto ff = BuildBruteForceFrenetFrame(start_lane->points(), false);
    if (ff.ok()) {
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha;
      ff.value().XYToSL(ego_pos, &sl, &normal, &index_pair, &alpha);
      sl.s = std::max(0.0, sl.s - 30.0);
      start_fraction = std::clamp(
          sl.s / std::fmax(start_lane->curve_length(), 1e-2), 0.0, 1.0 - 1e-2);
    }

    e2e_noa::mapping::LanePath lane_path(map, lane_id_infos, start_fraction,
                                         1.0);
    auto lane_path_info = LanePathInfo(lane_path, accm_s, 0.0, psmm);
    lane_path_info.set_lane_seq(
        std::make_shared<ad_e2e::planning::LaneSequence>(candidate_lane_seq));

    ad_e2e::planning::LaneSeqInfo temp_seq_info;
    bool result = GetLaneSeqInfo(
        nearest_lane_, ego_pos,
        std::make_shared<ad_e2e::planning::LaneSequence>(candidate_lane_seq),
        map, final_pnp_res, &temp_seq_info);
    const Behavior behavior = input.behavior.value();
    const auto map_func_id = behavior.function_id();
    if (result) {
      if (temp_seq_info.is_current &&
          IfMissNaviScenarioForCur(&temp_seq_info, ego_pos, map_func_id)) {
        if_miss_navi_secnario_ = true;
        if (start_point_velocity < 13.8) {
          lane_change_style = LC_STYLE_RADICAL;
        }
      }
      if (temp_seq_info.is_current) {
        cur_navi_lc_num_ = temp_seq_info.lc_num;
        left_navi_dist_ = temp_seq_info.dist_to_navi_end;
        left_navi_dist_v2_ = temp_seq_info.dist_to_navi_end_v2;
        cur_dist_to_junction_ = temp_seq_info.dist_to_junction;
        cur_nearest_turn_type_ = temp_seq_info.nearest_turn_type_v2;
        turn_type_v2 = temp_seq_info.turn_type_v2;
        last_turn_type_v2_ = temp_seq_info.last_turn_type_v2;
        cur_section_lane_num = temp_seq_info.cur_section_lane_num;
        cur_lane_position = temp_seq_info.cur_lane_position;
      }
      lane_path_info.set_lane_seq_info(temp_seq_info);
      lane_seq_infos.emplace_back(temp_seq_info);
    } else {
      temp_seq_info.lane_seq =
          std::make_shared<ad_e2e::planning::LaneSequence>(candidate_lane_seq);
      lane_path_info.set_lane_seq_info(temp_seq_info);
    }
    target_lp_infos_.emplace_back(lane_path_info);
  }
  TurnSignal turn_type = TURN_SIGNAL_NONE;
  if (turn_type_v2 == ad_e2e::planning::V2TurnInfo::V2TurnType::LEFT ||
      turn_type_v2 == ad_e2e::planning::V2TurnInfo::V2TurnType::RIGHT) {
    bool is_within_distance =
        (left_navi_dist_v2_ < 30 && left_navi_dist_v2_ > 0);
    if (is_within_distance) {
      switch (cur_nearest_turn_type_) {
        case ad_e2e::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT:
        case ad_e2e::planning::V2TurnInfo::V2DetailTurnType::SLIGHT_LEFT:
          if (cur_lane_position == 1) {
            turn_type = TURN_SIGNAL_LEFT;
          }
          break;
        case ad_e2e::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT:
        case ad_e2e::planning::V2TurnInfo::V2DetailTurnType::SLIGHT_RIGHT:
        case ad_e2e::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT_ONLY:
          if (cur_lane_position == cur_section_lane_num) {
            turn_type = TURN_SIGNAL_RIGHT;
          }
          break;
        default:
          break;
      }
    }
  }
  output->turn_type_signal = turn_type;
  const std::vector<LanePathInfo> &lp_infos = target_lp_infos_;
  if (target_lp_infos_.empty()) {
    planner_status_ =
        PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
                      "No valid target lane path from the current section.");
    return;
  } else {
    lane_decison_str = "[TargetLanePath]:";
    for (const auto &lane_path_info : target_lp_infos_) {
      if (lane_path_info.lane_seq() == nullptr) continue;
      lane_decison_str += " " + lane_path_info.start_lane_id();
    }
    Log2FG::LogVehData("[LaneDecision] start_lane", lane_decison_str);
  }
}

std::optional<double> LaneManager::UpdateCruisingSpeedLimitByLabel(
    const ad_e2e::planning::MapPtr &map, const double &start_v,
    const double &raw_cruising_speed_limit,
    const ad_e2e::planning::LaneSequencePtr &prev_target_lane_seq) {
  std::optional<double> speed_limit = raw_cruising_speed_limit;
  if (!map || !prev_target_lane_seq) return speed_limit;

  const auto &navi_start = map->route()->navi_start();
  const double frame_time = 0.1;
  double custom_limit = 0.0;
  double dis_to_custom = prev_target_lane_seq->GetDistanceToCustomSpeedLimit(
      navi_start, &custom_limit);

  if (dis_to_custom > 50.0 || custom_limit > raw_cruising_speed_limit) {
    return speed_limit;
  }

  double new_user_limit = raw_cruising_speed_limit;
  if (start_v - custom_limit < 5.0 * ad_e2e::planning::Constants::KPH2MPS) {
    new_user_limit = custom_limit;
  } else if (dis_to_custom < ad_e2e::planning::Constants::ZERO) {
    new_user_limit = start_v - 1.5 * frame_time;
  } else {
    double a =
        (custom_limit * custom_limit - start_v * start_v) / 2.0 / dis_to_custom;
    a = ad_e2e::planning::math::Clamp(a, -1.5, 1.5);
    new_user_limit = start_v + a * frame_time;
  }

  speed_limit = new_user_limit;
  return speed_limit;
}

void LaneManager::HandleSceneLcIntent(
    const ApolloTrajectoryPointProto *plan_start_point,
    const VehicleParamsProto *vehicle_param,
    const std::optional<double> lcc_cruising_speed_limit,
    const std::optional<Behavior_FunctionId> function_id,
    const std::optional<int> tunnel_status,
    const DriverAction::LaneChangeCommand &lane_change_command,
    const LaneChangeStateProto *lane_change_state,
    const PlannerSemanticMapManager *psmm, const Vec2d &ego_pos,
    const LaneConstPtr &nearest_lane, const PlannerObjectController &obj_mgr,
    const LaneSequencePtr &pre_target_laneseq,
    const std::optional<PNPInfos> pnp_infos, const bool lka_allow_navi_lc,
    const LaneChangeReason &lane_change_reason,
    LaneChangeReason &new_lane_change_reason,
    LaneSequencePtr &target_lane_seq) {
  target_lane_seq = nullptr;
  const auto map = psmm->map_ptr();
  if (!map || !nearest_lane) return;

  bool pnp_navi_lc_intent = false, is_left = true;
  if (lka_allow_navi_lc && pnp_infos.has_value() &&
      !pnp_infos.value().infos().empty() &&
      pnp_infos.value().infos()[0].lc_reason() ==
          LcReason::LC_REASON_FOR_NAVI) {
    std::set<std::string> pnp_top1_ids;
    for (const auto &lane_id :
         pnp_infos.value().infos()[0].target_lane_sequence_ids()) {
      pnp_top1_ids.insert(lane_id);
    }
    if (pnp_top1_ids.count(nearest_lane->left_lane_id()) != 0) {
      is_left = true;
      pnp_navi_lc_intent = true;
    } else if (pnp_top1_ids.count(nearest_lane->right_lane_id()) != 0) {
      is_left = false;
      pnp_navi_lc_intent = true;
    } else {
      pnp_navi_lc_intent = false;
    }
  }
  Log2FG::LogDataV2("navigation_lc_intent", pnp_navi_lc_intent);

  const bool navigation_bool =
      lane_change_state->stage() != LaneChangeStage::LCS_NONE &&
      lane_change_command == DriverAction::LC_CMD_NONE &&
      lane_change_reason == LCC_ROUTE_CHANGE;

  if (navigation_bool) {
    target_lane_seq = pre_target_laneseq;
    new_lane_change_reason = LCC_ROUTE_CHANGE;
    return;
  }

  ConstructionSceneIdentificationInput input{
      .plan_start_point = plan_start_point,
      .vehicle_param = vehicle_param,
      .psmm = psmm,

      .lcc_cruising_speed_limit = lcc_cruising_speed_limit,
      .function_id = function_id_,
      .tunnel_status = tunnel_status,
      .obj_mgr = obj_mgr,
      .target_lane_seq = pre_target_laneseq,
      .lane_change_state = lane_change_state,
  };
  static int debounce_construction_intent = 0;
  bool construction_intent = RunConstructionSceneIdentification(input);

  if (construction_intent && debounce_construction_intent < 2) {
    debounce_construction_intent++;
  } else {
    debounce_construction_intent = 0;
  }

  const bool construction_bool =
      lane_change_state->stage() != LaneChangeStage::LCS_NONE &&
      lane_change_command == DriverAction::LC_CMD_NONE &&
      lane_change_reason == AVOID_ROADWORK_CHANGE;

  if (debounce_construction_intent > 1 || construction_bool) {
    construction_intent = true;
  } else {
    construction_intent = false;
  }

  if (construction_bool) {
    target_lane_seq = pre_target_laneseq;
    new_lane_change_reason = AVOID_ROADWORK_CHANGE;
    return;
  }

  BlockerSceneIdentificationInput blockerinput{
      .plan_start_point = plan_start_point,
      .vehicle_param = vehicle_param,
      .psmm = psmm,

      .lcc_cruising_speed_limit = lcc_cruising_speed_limit,
      .function_id = function_id_,
      .tunnel_status = tunnel_status,
      .obj_mgr = obj_mgr,
      .target_lane_seq = pre_target_laneseq,
      .lane_change_state = lane_change_state,
      .target_lane_seq_blocker = pre_target_laneseq,
  };
  bool blocker_intent = false;
  if (0) {
    static int debounce_blocker_intent = 0;
    blocker_intent = RunBlockerSceneIdentification(blockerinput);

    if (blocker_intent && debounce_blocker_intent < 2) {
      debounce_blocker_intent++;
    } else {
      debounce_blocker_intent = 0;
    }

    const bool blocker_bool =
        lane_change_state->stage() != LaneChangeStage::LCS_NONE &&
        lane_change_command == DriverAction::LC_CMD_NONE &&
        lane_change_reason == AVOID_VEHICLE_CHANGE;

    blocker_intent = blocker_intent || blocker_bool;

    if (blocker_bool) {
      target_lane_seq = pre_target_laneseq;
      new_lane_change_reason = AVOID_VEHICLE_CHANGE;
      return;
    }
  }

  LaneConstPtr left_lane = map->GetLeftLane(nearest_lane);
  LaneSequencePtr left_laneseq = nullptr;
  if (IsValidLane(left_lane)) {
    left_laneseq = map->GetLaneSequence(left_lane, true);
  }
  LaneConstPtr right_lane = map->GetRightLane(nearest_lane);
  LaneSequencePtr right_laneseq = nullptr;
  if (IsValidLane(right_lane)) {
    right_laneseq = map->GetLaneSequence(right_lane, true);
  }
  if (pnp_navi_lc_intent &&
      ((is_left && left_laneseq) || (!is_left && right_laneseq))) {
    target_lane_seq = is_left ? left_laneseq : right_laneseq;
    new_lane_change_reason = LCC_ROUTE_CHANGE;
  } else if ((construction_intent || blocker_intent) &&
             (left_laneseq || right_laneseq)) {
    int left_navi_section_cnt = 0;
    double left_efficiency = 0.0;
    CountNaviAndEfficiencyInfo(ego_pos, left_laneseq, obj_mgr,
                               &left_navi_section_cnt, &left_efficiency);
    int right_navi_section_cnt = 0;
    double right_efficiency = 0.0;
    CountNaviAndEfficiencyInfo(ego_pos, right_laneseq, obj_mgr,
                               &right_navi_section_cnt, &right_efficiency);
    if (lka_allow_navi_lc && right_laneseq) {
      const int lc_num = map->route()->GetPriorityLaneRelation(nearest_lane);
      if (lc_num > 0) {
        target_lane_seq = left_laneseq;
      } else if (lc_num < 0) {
        target_lane_seq = right_laneseq;
      } else {
        target_lane_seq = nullptr;
      }
    } else if (left_navi_section_cnt != right_navi_section_cnt) {
      target_lane_seq = left_navi_section_cnt > right_navi_section_cnt
                            ? left_laneseq
                            : right_laneseq;
    } else {
      target_lane_seq =
          left_efficiency > right_efficiency ? left_laneseq : right_laneseq;
    }
    if (construction_intent) {
      new_lane_change_reason = AVOID_ROADWORK_CHANGE;
    } else if (blocker_intent) {
      new_lane_change_reason = AVOID_VEHICLE_CHANGE;
    }
  }
}

void LaneManager::CountNaviAndEfficiencyInfo(
    const Vec2d &ego_pos, const LaneSequencePtr &laneseq,
    const PlannerObjectController &obj_mgr, int *navi_section_cnt,
    double *average_speed) {
  if (!laneseq) return;

  int navi_cnt = 0;
  for (const auto &lane : laneseq->lanes()) {
    if (!lane->is_navigation()) break;
    navi_cnt++;
  }
  if (navi_section_cnt != nullptr) *navi_section_cnt = navi_cnt;

  double avg_speed = 0.0;
  int valid_obs_cnt = 0;
  double ego_s = 0.0;
  laneseq->GetProjectionDistance(ego_pos, &ego_s);
  for (const auto &obs : obj_mgr.planner_objects()) {
    const auto obs_pos = obs.pose().pos();
    double obs_s = 0.0;
    double obs_l = laneseq->GetProjectionDistance(obs_pos, &obs_s);
    if (obs_l > 2.0 || obs_s < ego_s) continue;
    avg_speed += obs.pose().v();
    valid_obs_cnt++;
  }
  if (valid_obs_cnt > 0 && average_speed != nullptr) {
    *average_speed = avg_speed / valid_obs_cnt;
  }
}

RouteNaviInfo LaneManager::AlignRouteNaviInfoWithCurrentSections(
    const PlannerSemanticMapManager &psmm, const RouteSections &route_sections,
    const RouteSections &current_sections,
    const RouteNaviInfo &route_navi_info) {
  const int back_sec_size = route_navi_info.back_extend_sections.size();
  const int sec_size = back_sec_size + route_sections.size();

  auto aligned_navi_info = route_navi_info;
  const auto cur_front_sec_id = current_sections.section_id(0);
  for (int i = 0; i < sec_size; ++i) {
    const auto route_sec_id =
        i < back_sec_size ? route_navi_info.back_extend_sections.section_id(i)
                          : route_sections.section_id(i - back_sec_size);

    if (route_sec_id == cur_front_sec_id) {
      SMM_ASSIGN_SECTION_OR_BREAK(sec_info, psmm, route_sec_id);
      const double end_frac = current_sections.start_fraction();
      const double curr_offset = sec_info.curve_length() * end_frac;

      auto &route_lane_infos = aligned_navi_info.route_lane_info_map;
      auto &navi_section_infos = aligned_navi_info.navi_section_info_map;
      navi_section_infos[sec_info.id()].length_before_intersection = std::max(
          0.0, navi_section_infos[sec_info.id()].length_before_intersection -
                   curr_offset);

      for (const auto &lane_id : sec_info.lanes()) {
        route_lane_infos[lane_id].max_driving_distance = std::max(
            0.0, route_lane_infos[lane_id].max_driving_distance - curr_offset);
        route_lane_infos[lane_id].max_reach_length = std::max(
            0.0, route_lane_infos[lane_id].max_reach_length - curr_offset);
        route_lane_infos[lane_id].recommend_reach_length =
            std::max(0.0, route_lane_infos[lane_id].recommend_reach_length -
                              curr_offset);
        route_lane_infos[lane_id].len_before_merge_lane = std::max(
            0.0, route_lane_infos[lane_id].len_before_merge_lane - curr_offset);
      }

      break;
    }
  }

  return aligned_navi_info;
}

}  // namespace e2e_noa::planning