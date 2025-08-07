#pragma once
#include "absl/container/flat_hash_set.h"
#include "maps/semantic_map_defs.h"
#include "object/planner_object_manager.h"
#include "router/lane_graph/v2/lane_graph.h"
#include "router/route_sections_info.h"
namespace e2e_noa::planning {

LaneGraph BuildLaneGraph(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const PlannerObjectController& obj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes);

LaneGraph BuildRouteLaneGraph(
    const ad_e2e::planning::Map& smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double build_length);

}  
