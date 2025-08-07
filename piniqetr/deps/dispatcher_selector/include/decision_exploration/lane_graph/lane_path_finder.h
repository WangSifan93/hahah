#ifndef PLANNER_SCHEDULER_LANE_GRAPH_LANE_PATH_FINDER_H_
#define PLANNER_SCHEDULER_LANE_GRAPH_LANE_PATH_FINDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "common/lane_path_info.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/lane_graph/v2/lane_graph.h"
#include "router/navi/route_navi_info.h"
#include "router/route_sections_info.h"

namespace e2e_noa::planning {

absl::StatusOr<std::vector<LanePathInfo>> FindBestLanePathsFromStart(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, const LaneGraph& lane_graph,
    WorkerThreadManager* thread_pool);

}

#endif
