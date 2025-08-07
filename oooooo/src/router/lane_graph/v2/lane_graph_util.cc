#include "router/lane_graph/v2/lane_graph_util.h"

#include <algorithm>
#include <initializer_list>
#include <queue>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/types/span.h"
#include "container/strong_int.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_util.h"
#include "math/util.h"
#include "plan/planner_defs.h"
#include "router/route_util.h"
#include "util/map_util.h"

namespace e2e_noa::planning {
constexpr int kLayerNumPerLcLen = 2;
constexpr int kLayerNumPerSharpLcLen = kLayerNumPerLcLen / 2;
constexpr double kVertexSampleDist = kMinLcLaneLength / kLayerNumPerLcLen;

constexpr double kCrossSolidWhiteCost = 1000.0;
constexpr double kSharpLaneChangePunishment = 2.0 * kCrossSolidWhiteCost;
constexpr double kOrdinaryLaneChangeCost = 100.0;
constexpr double kLaneLengthCost = 1e-3;
constexpr double kAvoidLaneCostFactor =
    1.2 * kCrossSolidWhiteCost / (kLaneLengthCost * kVertexSampleDist);

constexpr double kLengthError = 1e-8;
constexpr double kMergeLaneFactor = 2.0;

using AvoidLanes = absl::flat_hash_set<mapping::ElementId>;

LaneGraph::LaneGraphLayers SplitSectionsAsVertices(
    const RouteSectionsInfo& sections_info, int last_sec_idx,
    double last_sec_frac) {
  const auto& sections = sections_info.section_segments();
  LaneGraph::LaneGraphLayers layers;
  layers.reserve(
      CeilToInteger(sections_info.length_between(0, last_sec_idx + 1) /
                    kVertexSampleDist) +
      1);
  double accum_len = sections[0].average_length * sections[0].start_fraction;
  for (int i = 0; i < last_sec_idx; ++i) {
    while (accum_len < sections[i].average_length + kLengthError) {
      const double fraction = accum_len / sections[i].average_length;
      layers.push_back({i, fraction});
      accum_len += kVertexSampleDist;
    }
    accum_len -= sections[i].average_length;
  }

  const auto& last_sec = sections[last_sec_idx];
  const double last_sec_len = last_sec.average_length * last_sec_frac;
  while (accum_len < last_sec_len) {
    if (last_sec_len - accum_len < 0.1 * kVertexSampleDist) break;

    const double fraction = accum_len / last_sec.average_length;
    layers.push_back({last_sec_idx, fraction});
    accum_len += kVertexSampleDist;
  }
  layers.push_back({last_sec_idx, last_sec_frac});

  return layers;
}

bool HasOutgoingLane(
    const ad_e2e::planning::Map& map, mapping::ElementId lane_id,
    const RouteSectionsInfo::RouteSectionSegmentInfo& next_sec) {
  SMM2_ASSIGN_LANE_OR_RETURN(lane_info, map, lane_id, false);
  for (const auto& out_lane_id : lane_info.next_lane_ids()) {
    if (next_sec.id_idx_map.contains(mapping::ElementId(out_lane_id))) {
      return true;
    }
  }
  return false;
}

double GetLaneKeepCost(const ad_e2e::planning::Map& smm,
                       const AvoidLanes& avoid_lanes,
                       mapping::ElementId lane_id, double from_frac,
                       double to_frac) {
  SMM2_ASSIGN_LANE_OR_RETURN(lane_info, smm, lane_id, 0.0);
  SMM2_ASSIGN_SECTION_OR_RETURN(
      sec_info, smm, mapping::SectionId(lane_info.section_id()), 0.0);
  double lane_cost =
      kLaneLengthCost * sec_info.curve_length() * (to_frac - from_frac);
  if (avoid_lanes.contains(lane_id) ||
      ad_e2e::planning::Lane::IsPassengerVehicleAvoidLaneType(
          lane_info.type())) {
    lane_cost *= kAvoidLaneCostFactor;
  }

  return lane_cost;
}

double GetLaneKeepTurnIndexCost(const ad_e2e::planning::Map& smm,
                                const AvoidLanes& avoid_lanes,
                                mapping::ElementId from_id, double from_frac,
                                mapping::ElementId to_id, double to_frac) {
  constexpr double kTurnIndexDiffWeight = 1.0;

  SMM2_ASSIGN_LANE_OR_RETURN(from_lane_info, smm, from_id, 0.0);
  SMM2_ASSIGN_SECTION_OR_RETURN(
      from_sec_info, smm, mapping::SectionId(from_lane_info.section_id()), 0.0);
  SMM2_ASSIGN_LANE_OR_RETURN(to_lane_info, smm, to_id, 0.0);
  SMM2_ASSIGN_SECTION_OR_RETURN(
      to_sec_info, smm, mapping::SectionId(to_lane_info.section_id()), 0.0);
  mapping::ElementId outgoing_id = mapping::kInvalidElementId;
  if (!to_lane_info.next_lane_ids().empty()) {
    outgoing_id = mapping::ElementId(to_lane_info.next_lane_ids()[0]);
  }
  SMM2_ASSIGN_LANE_OR_RETURN(out_lane_info, smm, outgoing_id, 0.0);
  SMM2_ASSIGN_SECTION_OR_RETURN(
      out_sec_info, smm, mapping::SectionId(out_lane_info.section_id()), 0.0);

  int from_left_index = 0, from_right_index = 0;
  int to_left_index = 0, to_right_index = 0;
  int out_left_index = 0, out_right_index = 0;

  bool find_target_lane = false;
  for (const auto& lane : from_sec_info.lanes()) {
    if (mapping::ElementId(lane) == from_id) {
      find_target_lane = true;
      continue;
    }
    if (!find_target_lane) from_left_index++;
    if (find_target_lane) from_right_index++;
  }
  find_target_lane = false;
  for (const auto& lane : to_sec_info.lanes()) {
    if (mapping::ElementId(lane) == to_id) {
      find_target_lane = true;
      continue;
    }
    if (!find_target_lane) to_left_index++;
    if (find_target_lane) to_right_index++;
  }
  find_target_lane = false;
  for (const auto& lane : out_sec_info.lanes()) {
    if (mapping::ElementId(lane) == outgoing_id) {
      find_target_lane = true;
      continue;
    }
    if (!find_target_lane) out_left_index++;
    if (find_target_lane) out_right_index++;
  }
#if 0
  from_left_index = from_lane_info.proto().lane_neighbors_on_left_size();
  from_right_index = from_lane_info.proto().lane_neighbors_on_right_size();
  to_left_index = to_lane_info.proto().lane_neighbors_on_left_size();
  to_right_index = to_lane_info.proto().lane_neighbors_on_right_size();
  out_left_index = out_lane_info.proto().lane_neighbors_on_left_size();
  out_right_index = out_lane_info.proto().lane_neighbors_on_right_size();
#endif

  if (to_lane_info.turn_type() == ad_e2e::planning::TurnType::LEFT_TURN) {
    if (outgoing_id != mapping::kInvalidElementId) {
      return std::abs(out_left_index - from_left_index) * kTurnIndexDiffWeight;
    } else {
      return std::abs(to_left_index - from_left_index) * kTurnIndexDiffWeight;
    }
  } else if (to_lane_info.turn_type() ==
             ad_e2e::planning::TurnType::RIGHT_TURN) {
    if (outgoing_id != mapping::kInvalidElementId) {
      return std::abs(out_right_index - from_right_index) *
             kTurnIndexDiffWeight;
    } else {
      return std::abs(to_right_index - from_right_index) * kTurnIndexDiffWeight;
    }
  }
  return 0.0;
}

double GetMixedLaneKeepCost(const ad_e2e::planning::Map& smm,
                            const AvoidLanes& avoid_lanes,
                            mapping::ElementId from_id, double from_frac,
                            mapping::ElementId to_id, double to_frac) {
  double turn_index_cost = GetLaneKeepTurnIndexCost(smm, avoid_lanes, from_id,
                                                    from_frac, to_id, to_frac);
  return GetLaneKeepCost(smm, avoid_lanes, from_id, from_frac, 1.0) +
         GetLaneKeepCost(smm, avoid_lanes, to_id, 0.0, to_frac) +
         turn_index_cost;
}
double GetLaneChangeCost(const ad_e2e::planning::Map& map, int layer_idx,
                         int layers_size, mapping::ElementId from_id,
                         bool lc_left, mapping::ElementId to_id,
                         bool is_sharp) {
  SMM2_ASSIGN_LANE_OR_RETURN(from_lane_info, map, from_id,
                             LaneGraph::kInfiniteCost);
  const auto from_neighbor =
      lc_left ? from_lane_info.left_lane_id() : from_lane_info.right_lane_id();
  if (from_neighbor == "") return LaneGraph::kInfiniteCost;

  SMM2_ASSIGN_LANE_OR_RETURN(to_lane_info, map, to_id,
                             LaneGraph::kInfiniteCost);

  const auto to_neighbors =
      lc_left ? to_lane_info.right_lane_id() : to_lane_info.left_lane_id();

  if (to_neighbors == "") return LaneGraph::kInfiniteCost;

#if 0 
  if (to_lane_info.type() == ad_e2e::planning::LaneType::BICYCLE_ONLY ||
      to_lane_info.type() == ad_e2e::planning::LaneType::REVERSIBLE ||
      to_lane_info.type() == ad_e2e::planning::LaneType::VARIABLE_TURN)
    return LaneGraph::kInfiniteCost;
#endif

  if (to_lane_info.type() == ad_e2e::planning::LaneType::LANE_BUS_NORMAL)
    return LaneGraph::kInfiniteCost;

  const double sharp_lc_cost = is_sharp ? kSharpLaneChangePunishment : 0.0;

  const double early_lc_cost_factor =
      0.5 * (1.0 - static_cast<double>(layer_idx + 1) / layers_size);
  auto neighbor_lane = map.GetLaneById(from_neighbor);
  const auto from_neighbor_boundary =
      neighbor_lane ? (lc_left ? neighbor_lane->left_boundary()
                               : neighbor_lane->right_boundary())
                    : nullptr;
  neighbor_lane = map.GetLaneById(to_neighbors);
  const auto to_neighbors_boundary =
      neighbor_lane ? (lc_left ? neighbor_lane->right_boundary()
                               : neighbor_lane->left_boundary())
                    : nullptr;
  if ((from_neighbor_boundary != nullptr &&
       CrossSolidBoundary(from_neighbor_boundary->boundary_types(), lc_left)) ||
      (to_neighbors_boundary != nullptr &&
       CrossSolidBoundary(to_neighbors_boundary->boundary_types(), lc_left))) {
    return sharp_lc_cost + kCrossSolidWhiteCost * (1.0 - early_lc_cost_factor);
  } else {
    return sharp_lc_cost +
           kOrdinaryLaneChangeCost * (1.0 + early_lc_cost_factor);
  }
}

std::vector<mapping::ElementId> FindDirectlyConnectedLanesInSection(
    const ad_e2e::planning::Map& map, const RouteSectionsInfo& sections_info,
    int sec_idx, mapping::ElementId from_id, int target_sec_idx) {
  const auto& sections = sections_info.section_segments();
  std::vector<mapping::ElementId> lane_ids;
  std::queue<std::pair<int, mapping::ElementId>> q;
  q.push({sec_idx, from_id});
  while (!q.empty()) {
    const auto [cur_sec_idx, cur_id] = q.front();
    q.pop();
    if (cur_sec_idx == target_sec_idx) {
      lane_ids.push_back(cur_id);
      continue;
    }
    SMM2_ASSIGN_LANE_OR_BREAK(lane_info, map, cur_id);
    const auto& next_sec = sections[cur_sec_idx + 1];
    for (const auto& out_lane_id : lane_info.next_lane_ids()) {
      if (next_sec.id_idx_map.contains(mapping::ElementId(out_lane_id))) {
        q.push({cur_sec_idx + 1, mapping::ElementId(out_lane_id)});
      }
    }
  }
  return lane_ids;
}

void ConnectEdgesWithinSections(
    const ad_e2e::planning::Map& map, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    const LaneGraph::LaneGraphLayers& layers,
    VertexGraph<LaneGraph::VertexId>* graph) {
  const auto& sections = sections_info.section_segments();
  const int n = layers.size();
  for (int i = 0; i + 1 < n; ++i) {
    if (layers[i].first != layers[i + 1].first) continue;

    const auto [sec_idx, from_frac] = layers[i];
    const auto& sec_lanes = sections[sec_idx].lane_ids;
    for (const auto lane_id : sec_lanes) {
      graph->AddEdge(ToVertId(i, lane_id), ToVertId(i + 1, lane_id),
                     GetLaneKeepCost(map, avoid_lanes, lane_id, from_frac,
                                     layers[i + 1].second));
    }

    for (int to_layer : {i + kLayerNumPerSharpLcLen, i + kLayerNumPerLcLen}) {
      if (to_layer >= n || layers[i].first != layers[to_layer].first) break;

      const bool is_sharp = (to_layer < i + kLayerNumPerLcLen);

      for (int lane_idx = 0; lane_idx < sec_lanes.size(); ++lane_idx) {
        const auto from_id = sec_lanes[lane_idx];
        if (lane_idx > 0) {
          const auto to_id = sec_lanes[lane_idx - 1];
          graph->AddEdge(
              ToVertId(i, from_id), ToVertId(to_layer, to_id),
              GetLaneChangeCost(map, i, n, from_id, true, to_id, is_sharp));
        }
        if (lane_idx + 1 < sec_lanes.size()) {
          const auto to_id = sec_lanes[lane_idx + 1];
          graph->AddEdge(
              ToVertId(i, from_id), ToVertId(to_layer, to_id),
              GetLaneChangeCost(map, i, n, from_id, false, to_id, is_sharp));
        }
      }
    }
  }
}

void ConnectEdgesAcrossSections(
    const ad_e2e::planning::Map& map, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    const LaneGraph::LaneGraphLayers& layers, int last_sec_idx,
    VertexGraph<LaneGraph::VertexId>* graph,
    absl::flat_hash_set<LaneGraph::VertexId>* fork_lc_verts,
    absl::flat_hash_set<LaneGraph::EdgeType>* fork_lk_edges) {
  const int n = layers.size();

  const auto add_lc_edges = [&](int sec_idx, mapping::ElementId cur_id,
                                mapping::ElementId neighbor_id, bool lc_left,
                                int start_layer, int end_layer) {
    for (int j = start_layer; j < end_layer; ++j) {
      for (int to_layer : {j + kLayerNumPerSharpLcLen, j + kLayerNumPerLcLen}) {
        auto target_lanes = FindDirectlyConnectedLanesInSection(
            map, sections_info, sec_idx, neighbor_id, layers[to_layer].first);
        const bool is_sharp = (to_layer < j + kLayerNumPerLcLen);
        for (const auto target_id : target_lanes) {
          graph->AddEdge(ToVertId(j, cur_id), ToVertId(to_layer, target_id),
                         GetLaneChangeCost(map, j, n, cur_id, lc_left,
                                           target_id, is_sharp));
        }
        if (target_lanes.size() > 1) {
          const auto& id_idx_map =
              sections_info.section_segment(layers[to_layer].first).id_idx_map;
          std::sort(target_lanes.begin(), target_lanes.end(),
                    [&id_idx_map](const auto& id1, const auto& id2) {
                      return FindOrDie(id_idx_map, id1) <
                             FindOrDie(id_idx_map, id2);
                    });

          const int start_idx = lc_left ? 0 : 1;
          const int end_idx =
              lc_left ? target_lanes.size() - 1 : target_lanes.size();
          for (int target_idx = start_idx; target_idx < end_idx; ++target_idx) {
            const auto target_id = target_lanes[target_idx];
            fork_lc_verts->insert(ToVertId(to_layer, target_id));
          }
        }
      }
    }
  };

  const auto& sections = sections_info.section_segments();
  int layer_idx = 0;
  absl::flat_hash_set<LaneGraph::VertexId> dead_end_vertices;
  for (int sec_idx = 0; sec_idx < last_sec_idx; ++sec_idx) {
    if (layers[layer_idx].first != sec_idx) {
      continue;
    }

    while (layer_idx + kLayerNumPerLcLen < n &&
           layers[layer_idx + kLayerNumPerLcLen].first == sec_idx) {
      layer_idx += 1;
    }

    int lk_layer_idx = layer_idx;
    while (layers[lk_layer_idx].first == sec_idx) ++lk_layer_idx;

    const int last_lc_layer = std::min(lk_layer_idx, n - kLayerNumPerLcLen);

    const auto& cur_lanes = sections[sec_idx].lane_ids;

    for (int lane_idx = 0; lane_idx < cur_lanes.size(); ++lane_idx) {
      const auto cur_id = cur_lanes[lane_idx];
      if (!HasOutgoingLane(map, cur_id, sections[sec_idx + 1])) {
        dead_end_vertices.insert(ToVertId(lk_layer_idx - 1, cur_id));
        continue;
      }

      const auto target_lanes = FindDirectlyConnectedLanesInSection(
          map, sections_info, sec_idx, cur_id, layers[lk_layer_idx].first);
      for (const auto target_id : target_lanes) {
        graph->AddEdge(
            ToVertId(lk_layer_idx - 1, cur_id),
            ToVertId(lk_layer_idx, target_id),
            GetMixedLaneKeepCost(map, avoid_lanes, cur_id,
                                 layers[lk_layer_idx - 1].second, target_id,
                                 layers[lk_layer_idx].second));
      }

      if (target_lanes.size() > 1) {
        for (const auto target_id : target_lanes) {
          fork_lk_edges->insert({ToVertId(lk_layer_idx - 1, cur_id),
                                 ToVertId(lk_layer_idx, target_id)});
        }
      }

      if (lane_idx > 0) {
        add_lc_edges(sec_idx, cur_id, cur_lanes[lane_idx - 1], true, layer_idx,
                     last_lc_layer);
      }
      if (lane_idx + 1 < cur_lanes.size()) {
        add_lc_edges(sec_idx, cur_id, cur_lanes[lane_idx + 1], false, layer_idx,
                     last_lc_layer);
      }
    }
    layer_idx = lk_layer_idx;
  }

  for (const auto& vert : dead_end_vertices) {
    graph->AddEdge(vert, LaneGraph::kTargetVertex,
                   LaneGraph::kDeadEndToTargetCost);
  }
}

std::vector<mapping::ElementId> DirectlyConnectedLanesBetween(
    const ad_e2e::planning::Map& map, const RouteSectionsInfo& sections_info,
    const LaneGraph::LaneGraphLayers& layers, int start_layer, int target_layer,
    mapping::ElementId start_id, mapping::ElementId target_id) {
  if (target_id == mapping::kInvalidElementId) return {};

  const auto& sections = sections_info.section_segments();
  const int start_sec_idx = layers[start_layer].first,
            target_sec_idx = layers[target_layer].first;
  if (start_sec_idx == target_sec_idx) return {};

  std::queue<std::pair<int, mapping::ElementId>> q;
  absl::flat_hash_map<mapping::ElementId, mapping::ElementId> prev_map;
  q.push({start_sec_idx, start_id});
  while (!q.empty()) {
    const auto [cur_sec_idx, cur_id] = q.front();
    q.pop();
    SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, map, cur_id);
    for (const auto next_id : lane_info.next_lane_ids()) {
      const mapping::ElementId out_lane_id = mapping::ElementId(next_id);
      if (out_lane_id == target_id) {
        std::vector<mapping::ElementId> lane_ids{target_id};
        auto rev_id = cur_id;
        while (rev_id != start_id) {
          lane_ids.push_back(rev_id);
          rev_id = FindOrDie(prev_map, rev_id);
        }
        std::reverse(lane_ids.begin(), lane_ids.end());
        return lane_ids;
      }

      if (cur_sec_idx + 1 < target_sec_idx &&
          cur_sec_idx + 1 < sections.size() &&
          sections[cur_sec_idx + 1].id_idx_map.contains(out_lane_id)) {
        q.push({cur_sec_idx + 1, out_lane_id});
        prev_map[out_lane_id] = cur_id;
      }
    }
  }
  return {};
}

double GetSectionsLength(const ad_e2e::planning::Map& map,
                         const std::vector<mapping::ElementId>& lane_ids,
                         double start_frac, double end_frac) {
  if (lane_ids.empty()) return 0.0;

  const auto get_section_len = [&map](mapping::ElementId lane_id) {
    SMM2_ASSIGN_LANE_OR_RETURN(lane_info, map, lane_id, 0.0);
    SMM2_ASSIGN_SECTION_OR_RETURN(
        section_info, map, mapping::SectionId(lane_info.section_id()), 0.0);
    return section_info.curve_length();
  };

  double accum_len = 0.0;
  for (const auto& lane_id : lane_ids) {
    accum_len += get_section_len(lane_id);
  }
  accum_len -= get_section_len(lane_ids.front()) * start_frac;
  accum_len -= get_section_len(lane_ids.back()) * (1.0 - end_frac);

  return std::max(0.0, accum_len);
}

}  // namespace e2e_noa::planning
