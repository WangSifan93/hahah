#pragma once

#include <vector>

#include "absl/container/flat_hash_set.h"
#include "maps/semantic_map_defs.h"
#include "router/lane_graph/v2/lane_graph.h"
#include "router/route_sections_info.h"
namespace e2e_noa::planning {

LaneGraph::LaneGraphLayers SplitSectionsAsVertices(
    const RouteSectionsInfo& sections_info, int last_sec_idx,
    double last_sec_frac);

void ConnectEdgesWithinSections(
    const ad_e2e::planning::Map& map, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    const LaneGraph::LaneGraphLayers& layers,
    VertexGraph<LaneGraph::VertexId>* graph);

void ConnectEdgesAcrossSections(
    const ad_e2e::planning::Map& map, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    const LaneGraph::LaneGraphLayers& layers, int last_sec_idx,
    VertexGraph<LaneGraph::VertexId>* graph,
    absl::flat_hash_set<LaneGraph::VertexId>* fork_lc_verts,
    absl::flat_hash_set<LaneGraph::EdgeType>* fork_lk_edges);

std::vector<mapping::ElementId> DirectlyConnectedLanesBetween(
    const ad_e2e::planning::Map& map, const RouteSectionsInfo& sections_info,
    const LaneGraph::LaneGraphLayers& layers, int start_layer, int target_layer,
    mapping::ElementId start_id, mapping::ElementId target_id);

double GetSectionsLength(const ad_e2e::planning::Map& map,
                         const std::vector<mapping::ElementId>& lane_ids,
                         double start_frac, double end_frac);

}  // namespace e2e_noa::planning
