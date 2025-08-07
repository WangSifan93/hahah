#include "router/navi/route_navi_info_builder.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <ostream>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/meta/type_traits.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "common/planning_macros.h"
#include "container/strong_int.h"
#include "maps/lane.h"
#include "maps/map_or_die_macros.h"
#include "route.pb.h"
#include "router/lane_graph/v2/dijkstra.h"
#include "router/lane_graph/v2/lane_graph.h"
#include "router/lane_graph/v2/lane_graph_util.h"
#include "router/lane_graph/v2/route_lane_graph_builder.h"
#include "router/navi/route_navi_util.h"
#include "util/map_util.h"

namespace e2e_noa::planning {

namespace {
constexpr double kOutNaviInfoDist = 1000.0;
constexpr double kLaneGraphInfoDistNormal = 50.0;
constexpr double kLaneGraphInfoDistHighWay = 100.0;
constexpr double kExtendDrivingDist = 200.0;
constexpr double kExtendDrivingDistHighWay = 500.0;

struct RecommendPathInfo {
  std::vector<mapping::ElementId> lane_ids;
  double start_fraction = 0.0;
  double end_fraction = 0.0;
};

struct LaneNaviInfo {
  double max_reach_length = 0.0;
  double recommend_reach_length = 0.0;
  double len_before_merge_lane = DBL_MAX;
  RecommendPathInfo recommend_path_info;
};

struct LaneGraphResultInfo {
  double recommend_reach_length = 0.0;
  double len_before_merge_lane = DBL_MAX;
  RecommendPathInfo path_info;
};

bool IsTwoLanesConnected(const ad_e2e::planning::Map& smm,
                         mapping::ElementId source_id,
                         mapping::ElementId target_id) {
  constexpr int kExpandLayers = 5;
  std::queue<std::pair<mapping::ElementId, int>> que;
  que.push({source_id, 0});

  while (!que.empty()) {
    const auto [cur_id, layer] = que.front();
    que.pop();

    if (layer > kExpandLayers) break;

    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, smm, cur_id);

    for (const auto next_lane_id : lane_info.next_lane_ids()) {
      if (next_lane_id == target_id) return true;
      que.push({next_lane_id, layer + 1});
    }
  }

  return false;
}

double CalcLengthBeforeMergeLane(
    const ad_e2e::planning::Map& smm,
    const std::vector<mapping::ElementId>& lane_ids, double start_frac,
    double end_frac) {
  if (lane_ids.empty()) return DBL_MAX;

  bool has_merge_lane = false;
  const auto get_section_len = [&](mapping::ElementId lane_id) {
    SMM_ASSIGN_LANE_OR_RETURN(lane_info, smm, lane_id, DBL_MAX);
    SMM_ASSIGN_SECTION_OR_RETURN(section_info, smm, lane_info.section_id(),
                                 DBL_MAX);

    has_merge_lane = lane_info.is_merge();
    const double start_fraction =
        lane_id == lane_ids.front() ? start_frac : 0.0;
    const double end_fraction = lane_id == lane_ids.back() ? end_frac : 1.0;
    return section_info.curve_length() * (end_fraction - start_fraction);
  };

  double accum_len = 0.0;
  for (const auto& lane_id : lane_ids) {
    double section_len = get_section_len(lane_id);
    accum_len += section_len;
    if (has_merge_lane) {
      return accum_len;
    }
  }
  return DBL_MAX;
}

RouteNaviInfo::NaviSectionInfo CalculateNaviSectionInfo(
    const ad_e2e::planning::Map& smm, const RouteSectionsInfo& sections_info,
    double preview_length, int start_section_idx) {
  std::optional<e2e_noa::NaviSectionInfoProto::Direction> direction =
      std::nullopt;
  RouteNaviInfo::NaviSectionInfo navi_section_info;
  for (int index = start_section_idx; index < sections_info.size(); ++index) {
    if (preview_length <= 0.0) break;
    const auto section_length =
        sections_info.section_segment(index).average_length;
    preview_length -= section_length;
    const auto& lane_ids = sections_info.section_segment(index).lane_ids;
    if (lane_ids.empty()) break;
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, smm, lane_ids.front());

    if (lane_info.interactions().size() != 0) {
      if (!direction.has_value()) {
        switch (lane_info.turn_type()) {
          case ad_e2e::planning::LEFT_TURN:
            direction = e2e_noa::NaviSectionInfoProto::LEFT_TURN;
            break;
          case ad_e2e::planning::RIGHT_TURN:
            direction = e2e_noa::NaviSectionInfoProto::RIGHT_TURN;
            break;
          case ad_e2e::planning::U_TURN:
            direction = e2e_noa::NaviSectionInfoProto::LEFT_TURN;
            break;
          case ad_e2e::planning::NO_TURN:
            direction = e2e_noa::NaviSectionInfoProto::STRAIGHT;
            break;
        }
        navi_section_info.intersection_direction = *direction;
      }

      for (const auto& intersection : lane_info.interactions()) {
        const auto intersection_ptr =
            smm.GetJunctionById(mapping::ElementId(intersection.other_lane_id));
        if (intersection_ptr != nullptr &&
            intersection_ptr->traffic_light_controlled()) {
          return navi_section_info;
        }
      }
    }
    navi_section_info.length_before_intersection += section_length;
  }
  return navi_section_info;
}

LaneGraphResultInfo RecoverLaneGraphResult(
    const ad_e2e::planning::Map& smm, const RouteSectionsInfo& sections_info,
    const LaneGraph& lane_graph, const TravelPath<LaneGraph::VertexId>& path,
    int layer) {
  const auto& layers = lane_graph.layers;
  CHECK(!layers.empty()) << "Input lane graph is empty.";
  const auto& vertices = path.vertices;

  std::vector<mapping::ElementId> lane_ids;
  const double start_frac = layers[layer].second;
  auto [cur_layer, cur_id] = FromVertId(vertices[0]);
  lane_ids.push_back(cur_id);

  int last_index = vertices.size();
  constexpr double kEpsilon = 0.1;
  for (int i = 0; i + 1 < vertices.size(); ++i) {
    if (lane_graph.graph.edge_cost(vertices[i], vertices[i + 1]) >
        LaneGraph::kDeadEndToTargetCost - kEpsilon) {
      last_index = i + 1;
      break;
    }
  }

  double end_frac = 0.0;
  for (int i = 1; i < last_index;) {
    auto [next_layer, next_id] = FromVertId(vertices[i]);
    while (i < last_index && next_id == cur_id) {
      std::tie(next_layer, next_id) = FromVertId(vertices[++i]);
    }
    const auto lanes_between = DirectlyConnectedLanesBetween(
        smm, sections_info, layers, cur_layer, next_layer, cur_id, next_id);

    if (lanes_between.empty()) {
      end_frac = layers[FromVertId(vertices[i - 1]).first].second;
      break;
    }
    lane_ids.insert(lane_ids.end(), lanes_between.begin(), lanes_between.end());
    cur_layer = next_layer;
    cur_id = next_id;
  }
  LaneGraphResultInfo lane_graph_result_info;
  lane_graph_result_info.recommend_reach_length =
      GetSectionsLength(smm, lane_ids, 0.0, end_frac);
  lane_graph_result_info.len_before_merge_lane =
      CalcLengthBeforeMergeLane(smm, lane_ids, 0.0, end_frac);
  lane_graph_result_info.path_info = {.lane_ids = std::move(lane_ids),
                                      .start_fraction = start_frac,
                                      .end_fraction = end_frac};

  return lane_graph_result_info;
}

LaneNaviInfo CalcLaneNaviInfo(const ad_e2e::planning::Map& smm,
                              const RouteSectionsInfo& sections_info,
                              const LaneGraph& lane_graph,
                              mapping::ElementId lane_id, int layer) {
  CHECK(!lane_graph.layers.empty()) << "Input lane graph is empty.";

  const auto path_or = Dijkstra(lane_graph.graph, ToVertId(layer, lane_id),
                                LaneGraph::kTargetVertex);
  LaneNaviInfo lane_navi_info;
  if (!path_or.ok()) {
    return lane_navi_info;
  }
  auto fork_graph_result_info =
      RecoverLaneGraphResult(smm, sections_info, lane_graph, *path_or, layer);
  lane_navi_info.recommend_reach_length =
      fork_graph_result_info.recommend_reach_length;
  lane_navi_info.max_reach_length =
      fork_graph_result_info.recommend_reach_length;
  lane_navi_info.len_before_merge_lane =
      fork_graph_result_info.len_before_merge_lane;
  lane_navi_info.recommend_path_info =
      std::move(fork_graph_result_info.path_info);

  if (!lane_graph.fork_lc_verts.empty()) {
    auto new_graph = lane_graph.graph;
    for (const auto& vert_id : lane_graph.fork_lc_verts) {
      new_graph.RemoveEdgesFrom(vert_id);
    }
    const auto new_path_or =
        Dijkstra(new_graph, ToVertId(layer, lane_id), LaneGraph::kTargetVertex);
    if (new_path_or.ok()) {
      const auto graph_result_info = RecoverLaneGraphResult(
          smm, sections_info, lane_graph, *new_path_or, layer);
      if (graph_result_info.recommend_reach_length >
          lane_navi_info.max_reach_length) {
        lane_navi_info.max_reach_length =
            graph_result_info.recommend_reach_length;
        lane_navi_info.len_before_merge_lane =
            graph_result_info.len_before_merge_lane;
      }
    }
  }

  return lane_navi_info;
}

void DeduceLaneNaviInfo(
    const ad_e2e::planning::Map& smm, const LaneNaviInfo& base_lane_info,
    absl::flat_hash_map<mapping::ElementId, RouteNaviInfo::RouteLaneInfo>*
        out) {
  double reduced_len = 0.0;
  const auto& path_info = base_lane_info.recommend_path_info;
  for (int i = 1; i < path_info.lane_ids.size(); ++i) {
    SMM_ASSIGN_LANE_OR_BREAK(prev_lane_info, smm, path_info.lane_ids[i - 1]);
    SMM_ASSIGN_SECTION_OR_BREAK(prev_sec_info, smm,
                                prev_lane_info.section_id());
    reduced_len += prev_sec_info.curve_length();

    auto [iter, insert_success] =
        out->insert({path_info.lane_ids[i], RouteNaviInfo::RouteLaneInfo()});
    if (insert_success) {
      iter->second.max_reach_length =
          std::max(0.0, base_lane_info.max_reach_length - reduced_len);
      iter->second.recommend_reach_length =
          std::max(0.0, base_lane_info.recommend_reach_length - reduced_len);
      const std::vector<mapping::ElementId> tmp_lane_ids(
          path_info.lane_ids.begin() + i, path_info.lane_ids.end());
      iter->second.len_before_merge_lane = CalcLengthBeforeMergeLane(
          smm, tmp_lane_ids, 0.0, path_info.end_fraction);
    }
  }
}
}

absl::StatusOr<RouteNaviInfo> CalcNaviInfoByLaneGraph(
    const ad_e2e::planning::Map& smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_dist) {
  if (sections_info.length() < 0.0) {
    return absl::InvalidArgumentError("Input sections are invalid.");
  }

  SMM_ASSIGN_SECTION_OR_RETURN(front_section_info, smm,
                               sections_info.front().id,
                               absl::NotFoundError(absl::StrFormat(
                                   "Can not find the first section info "
                                   "to calculate route navi info, id: %d",
                                   sections_info.front().id)));
  bool in_high_way = false;

  const auto road_class = front_section_info.road_class();
  in_high_way = road_class == ad_e2e::planning::Section::CITY_EXPRESS ||
                road_class == ad_e2e::planning::Section::HIGHWAY;

  const double lane_graph_info_dist =
      in_high_way ? kLaneGraphInfoDistHighWay : kLaneGraphInfoDistNormal;

  const auto lane_graph =
      BuildRouteLaneGraph(smm, sections_info, avoid_lanes, preview_dist);

  const auto driving_dist_map =
      smm2::CalculateMaxDrivingDistance(smm, sections_info, avoid_lanes, true);
  const auto min_lc_map = smm2::FindLcNumToTargets(
      smm, sections_info, avoid_lanes, preview_dist, 0);
  absl::flat_hash_map<mapping::ElementId, int> lc_within_driving_dist_map;

  RouteNaviInfo out_navi_info;
  auto& lane_navi_info_map = out_navi_info.route_lane_info_map;

  const auto fill_navi_info = [&out_navi_info, &driving_dist_map, &min_lc_map,
                               &lc_within_driving_dist_map](
                                  mapping::ElementId lane_id,
                                  const LaneNaviInfo& input_navi_info) {
    auto& lane_navi_info = out_navi_info.route_lane_info_map[lane_id];
    lane_navi_info.max_driving_distance =
        FindWithDefault(driving_dist_map, lane_id, 0.0);
    lane_navi_info.max_reach_length = input_navi_info.max_reach_length;
    lane_navi_info.recommend_reach_length =
        input_navi_info.recommend_reach_length;
    lane_navi_info.min_lc_num_to_target =
        FindWithDefault(min_lc_map, lane_id, std::numeric_limits<int>::max());
    lane_navi_info.lc_num_within_driving_dist = FindWithDefault(
        lc_within_driving_dist_map, lane_id, std::numeric_limits<int>::max());
    lane_navi_info.len_before_merge_lane =
        input_navi_info.len_before_merge_lane;
  };

  double accum_dist = 0.0;
  int layer_idx = 0;
  for (int i = 0; i < sections_info.size(); ++i) {
    LaneNaviInfo lane_navi_info;
    if (accum_dist < lane_graph_info_dist) {
      while (lane_graph.layers[layer_idx].first < i) {
        ++layer_idx;
      }

      for (const auto& lane_id : sections_info.section_segment(i).lane_ids) {
        const double max_driving_dist =
            FindWithDefault(driving_dist_map, lane_id, 0.0);
        const double extend_len =
            in_high_way ? kExtendDrivingDistHighWay : kExtendDrivingDist;
        const double preview_distance =
            std::min(max_driving_dist + extend_len, preview_dist - accum_dist);
        const auto lc_num_map = smm2::FindLcNumToTargets(
            smm, sections_info, avoid_lanes, preview_distance, i);
        lc_within_driving_dist_map[lane_id] = FindWithDefault(
            lc_num_map, lane_id, std::numeric_limits<int>::max());

        if (!lane_navi_info_map.contains(lane_id)) {
          if (lane_graph.layers[layer_idx].first != i && layer_idx > 0) {
            const auto& last_sec = sections_info.section_segment(
                lane_graph.layers[layer_idx - 1].first);
            for (const auto& last_lane_id : last_sec.lane_ids) {
              if (IsTwoLanesConnected(smm, last_lane_id, lane_id)) {
                lane_navi_info =
                    CalcLaneNaviInfo(smm, sections_info, lane_graph,
                                     last_lane_id, layer_idx - 1);
                const double length =
                    sections_info.length_between(
                        lane_graph.layers[layer_idx - 1].first, i) -
                    last_sec.length() * lane_graph.layers[layer_idx - 1].second;
                lane_navi_info.max_reach_length =
                    std::max(0.0, lane_navi_info.max_reach_length - length);
                lane_navi_info.recommend_reach_length = std::max(
                    0.0, lane_navi_info.recommend_reach_length - length);
                lane_navi_info.len_before_merge_lane = std::max(
                    0.0, lane_navi_info.len_before_merge_lane - length);

                break;
              }
            }
          } else {
            lane_navi_info = CalcLaneNaviInfo(smm, sections_info, lane_graph,
                                              lane_id, layer_idx);
          }
          fill_navi_info(lane_id, lane_navi_info);
          DeduceLaneNaviInfo(smm, lane_navi_info, &lane_navi_info_map);
        } else {
          lane_navi_info_map[lane_id].max_driving_distance =
              FindWithDefault(driving_dist_map, lane_id, 0.0);
          lane_navi_info_map[lane_id].min_lc_num_to_target = FindWithDefault(
              min_lc_map, lane_id, std::numeric_limits<int>::max());
          lane_navi_info_map[lane_id].lc_num_within_driving_dist =
              FindWithDefault(lc_within_driving_dist_map, lane_id,
                              std::numeric_limits<int>::max());
        }
      }
    } else if (lane_graph_info_dist <= accum_dist &&
               accum_dist < kOutNaviInfoDist) {
      for (const auto& lane_id : sections_info.section_segment(i).lane_ids) {
        if (lane_navi_info_map.contains(lane_id)) {
          lane_navi_info_map[lane_id].max_driving_distance =
              FindWithDefault(driving_dist_map, lane_id, 0.0);
          lane_navi_info_map[lane_id].min_lc_num_to_target = FindWithDefault(
              min_lc_map, lane_id, std::numeric_limits<int>::max());
          lane_navi_info_map[lane_id].lc_num_within_driving_dist =
              FindWithDefault(lc_within_driving_dist_map, lane_id,
                              std::numeric_limits<int>::max());
        } else {
          fill_navi_info(lane_id, lane_navi_info);
        }
      }
    } else if (accum_dist >= kOutNaviInfoDist) {
      break;
    }

    out_navi_info.navi_section_info_map[sections_info.section_segment(i).id] =
        CalculateNaviSectionInfo(smm, sections_info, preview_dist - accum_dist,
                                 i);

    accum_dist += sections_info.section_segment(i).length();
  }
  return out_navi_info;
}
}
