#ifndef ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_BUILDER_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_BUILDER_H_

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "maps/semantic_map_defs.h"
#include "router/navi/route_navi_info.h"
#include "router/route_sections_info.h"
namespace e2e_noa::planning {

absl::StatusOr<RouteNaviInfo> CalcNaviInfoByLaneGraph(
    const ad_e2e::planning::Map& smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_dist);
}

#endif
