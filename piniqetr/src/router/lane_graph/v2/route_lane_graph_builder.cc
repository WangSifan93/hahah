#include "router/lane_graph/v2/route_lane_graph_builder.h"

#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/types/span.h"
#include "container/strong_int.h"
#include "maps/lane_point.h"
#include "plan/planner_defs.h"
#include "router/lane_graph/v2/lane_graph.h"
#include "router/lane_graph/v2/lane_graph_util.h"
#include "router/route_sections_util.h"
#include "util/lane_path_util.h"
#include "util/lane_point_util.h"
#include "util/planner_semantic_map_util.h"
#include "util/status_macros.h"

namespace e2e_noa::planning {

namespace {
constexpr double kExtendedLocalHorizon =
    kPlannerLaneGraphLength + 5.0 * kMinLcLaneLength;
constexpr int kLayerNumPerLcLen = 2;

using AvoidLanes = absl::flat_hash_set<mapping::ElementId>;

void ProjectStationaryObjects(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info, bool target_in_horizon,
    const PlannerObjectController& obj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const LaneGraph::LaneGraphLayers& layers,
    VertexGraph<LaneGraph::VertexId>* graph) {
  std::vector<Box2d> obj_boxes;
  obj_boxes.reserve(stalled_objects.size());
  for (const auto obj_ptr : obj_mgr.stationary_objects()) {
    if (!stalled_objects.contains(obj_ptr->id())) continue;

    constexpr double kObjectBoxBuffer = 0.8;
    auto obj_box = obj_ptr->contour().MinAreaBoundingBox();
    obj_box.LongitudinalExtend(kObjectBoxBuffer);
    obj_box.LateralExtend(kObjectBoxBuffer);
    ASSIGN_OR_CONTINUE(const auto obj_proj,
                       FindSmoothPointOnRouteSectionsByLateralOffset(
                           psmm, sections_info, obj_box.center()));
    ASSIGN_OR_CONTINUE(
        const auto tmp_lane_path,
        BuildLanePathFromData(
            mapping::LanePathData(0.0, 1.0, {obj_proj.lane_id}), psmm));
    if (!IsLanePathBlockedByBox2d(psmm, obj_box, tmp_lane_path, 0.0)) {
      continue;
    }

    std::cout << "obj_box: " << obj_ptr->id() << std::endl;
    obj_boxes.push_back(std::move(obj_box));
  }
  if (obj_boxes.empty()) return;

  const auto& sections = sections_info.section_segments();
  absl::flat_hash_map<LaneGraph::VertexId, Vec2d> vertex_pos;

  const int last_layer_idx =
      target_in_horizon ? layers.size() - kLayerNumPerLcLen : layers.size();
  for (int i = 0; i < last_layer_idx; ++i) {
    const auto [sec_idx, frac] = layers[i];
    for (const auto lane_id : sections[sec_idx].lane_ids) {
      vertex_pos[ToVertId(i, lane_id)] =
          ComputeLanePointPos(psmm, mapping::LanePoint(lane_id, frac));
    }
  }

  for (const auto& from_vert : graph->vertices()) {
    if (!vertex_pos.contains(from_vert)) continue;

    for (const auto& [to_vert, edge_cost] : graph->edges_from(from_vert)) {
      if (!vertex_pos.contains(to_vert)) continue;

      const Segment2d edge_seg(vertex_pos[from_vert], vertex_pos[to_vert]);
      for (const auto& box : obj_boxes) {
        if (box.HasOverlap(edge_seg)) {
          graph->ModifyEdge(from_vert, to_vert,
                            LaneGraph::kDeadEndToTargetCost);
          break;
        }
      }
    }
  }
}
}  

LaneGraph BuildLaneGraph(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const PlannerObjectController& obj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes) {
  const auto& sections = sections_info.section_segments();

  const auto destination_section_ptr =
      psmm.FindSectionByIdOrNull(sections.back().id);
  const bool target_in_horizon =
      (sections_info.length() <= kExtendedLocalHorizon) &&
      destination_section_ptr != nullptr;
  int last_sec_idx = 0;
  double last_sec_frac = 0.0;
  if (target_in_horizon) {
    last_sec_idx = sections_info.size() - 1;
    last_sec_frac = sections_info.end_fraction();
  } else {
    double accum_len = 0.0;
    bool not_loaded_yet = false;
    for (; last_sec_idx < sections_info.size(); ++last_sec_idx) {
      const auto& section = sections[last_sec_idx];
      if (section.lane_ids.empty()) {
        not_loaded_yet = true;
        break;
      }
      if (accum_len + section.length() >= kExtendedLocalHorizon) {
        last_sec_frac =
            section.start_fraction +
            (kExtendedLocalHorizon - accum_len) / section.average_length;
        break;
      }
      accum_len += section.length();
    }
    if (not_loaded_yet) {
      last_sec_frac = 1.0;
      --last_sec_idx;
    }
  }

  VertexGraph<LaneGraph::VertexId> graph;
  auto layers =
      SplitSectionsAsVertices(sections_info, last_sec_idx, last_sec_frac);

  for (int i = 0; i < layers.size(); ++i) {
    for (const auto lane_id : sections[layers[i].first].lane_ids) {
      graph.AddVertex(ToVertId(i, lane_id));
    }
  }
  graph.AddVertex(LaneGraph::kTargetVertex);

  ConnectEdgesWithinSections(*psmm.map_ptr(), sections_info, avoid_lanes,
                             layers, &graph);

  absl::flat_hash_set<LaneGraph::VertexId> fork_lc_verts;
  absl::flat_hash_set<LaneGraph::EdgeType> fork_lk_edges;
  ConnectEdgesAcrossSections(*psmm.map_ptr(), sections_info, avoid_lanes,
                             layers, last_sec_idx, &graph, &fork_lc_verts,
                             &fork_lk_edges);

  ProjectStationaryObjects(psmm, sections_info, target_in_horizon, obj_mgr,
                           stalled_objects, layers, &graph);

  const auto last_layer = layers.size() - 1;
  if (target_in_horizon) {
    for (const auto lane_id : sections.back().lane_ids) {
      const auto navi_priority_lane =
          psmm.map_ptr()->route()->GetNaviPriorityLane(sections.back().id);
      if (!navi_priority_lane) continue;
      const auto navi_priority_lane_id = navi_priority_lane->id();

      if (lane_id == navi_priority_lane_id) {
        graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                      0.0);
      } else {
        graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                      LaneGraph::kDeadEndToTargetCost);
      }
    }
  } else {
    const auto& last_sec = sections[last_sec_idx];
    for (const auto lane_id : last_sec.lane_ids) {
      graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                    0.0);
    }
  }

  return LaneGraph{.layers = std::move(layers),
                   .graph = std::move(graph),
                   .fork_lc_verts = std::move(fork_lc_verts),
                   .fork_lk_edges = std::move(fork_lk_edges)};
}

LaneGraph BuildRouteLaneGraph(const ad_e2e::planning::Map& smm,
                              const RouteSectionsInfo& sections_info,
                              const AvoidLanes& avoid_lanes,
                              double build_length) {
  const auto& sections = sections_info.section_segments();
  int last_sec_idx = 0;
  double last_sec_frac = 0.0;

  const bool target_in_horizon = sections_info.length() <= build_length;
  if (target_in_horizon) {
    last_sec_idx = sections_info.size() - 1;
    last_sec_frac = sections_info.end_fraction();
  } else {
    double accum_length = 0.0;
    for (; last_sec_idx < sections.size(); ++last_sec_idx) {
      const auto& section = sections[last_sec_idx];
      if (accum_length + section.length() >= build_length) {
        last_sec_frac = section.start_fraction +
                        (build_length - accum_length) / section.average_length;
        break;
      }
      accum_length += section.length();
    }
  }

  VertexGraph<LaneGraph::VertexId> graph;
  auto layers =
      SplitSectionsAsVertices(sections_info, last_sec_idx, last_sec_frac);

  for (int i = 0; i < layers.size(); ++i) {
    for (const auto lane_id : sections[layers[i].first].lane_ids) {
      graph.AddVertex(ToVertId(i, lane_id));
    }
  }
  graph.AddVertex(LaneGraph::kTargetVertex);

  ConnectEdgesWithinSections(smm, sections_info, avoid_lanes, layers, &graph);

  absl::flat_hash_set<LaneGraph::VertexId> fork_lc_verts;
  absl::flat_hash_set<LaneGraph::EdgeType> fork_lk_edges;
  ConnectEdgesAcrossSections(smm, sections_info, avoid_lanes, layers,
                             last_sec_idx, &graph, &fork_lc_verts,
                             &fork_lk_edges);

  const auto last_layer = layers.size() - 1;
  if (target_in_horizon) {
    for (const auto lane_id : sections.back().lane_ids) {
      const auto navi_priority_lane =
          smm.route()->GetNaviPriorityLane(sections.back().id);
      if (!navi_priority_lane) continue;
      const auto navi_priority_lane_id = navi_priority_lane->id();

      if (lane_id == navi_priority_lane_id) {
        graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                      0.0);
      } else {
        graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                      LaneGraph::kDeadEndToTargetCost);
      }
    }
  } else {
    const auto& last_sec = sections[last_sec_idx];
    for (const auto lane_id : last_sec.lane_ids) {
      graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                    0.0);
    }
  }

  return LaneGraph{.layers = std::move(layers),
                   .graph = std::move(graph),
                   .fork_lc_verts = std::move(fork_lc_verts),
                   .fork_lk_edges = std::move(fork_lk_edges)};
}
}  
