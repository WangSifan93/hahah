#ifndef ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_UTIL_H_

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "maps/semantic_map_defs.h"
#include "router/route_sections_info.h"
namespace e2e_noa {
namespace planning {
namespace smm2 {
absl::flat_hash_map<mapping::ElementId, double> CalculateMaxDrivingDistance(
    const ad_e2e::planning::Map& v2smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    bool from_lane_beginning);
absl::flat_hash_map<mapping::ElementId, int> FindLcNumToTargets(
    const ad_e2e::planning::Map& v2smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_length, int start_section_idx);
}  // namespace smm2
}  // namespace planning
}  // namespace e2e_noa

#endif
