#ifndef ONBOARD_PLANNER_SCHEDULER_LOCAL_MAP_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_LOCAL_MAP_BUILDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "maps/lane_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/navi/route_navi_info.h"
#include "router/route_sections.h"

namespace e2e_noa::planning {
absl::StatusOr<std::vector<mapping::LanePath>> BuildLocalMap(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const RouteNaviInfo& route_navi_info);
}

#endif
