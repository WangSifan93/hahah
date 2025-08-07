#include "maps/local_map_builder.h"

#include <algorithm>
#include <vector>

#include "absl/status/status.h"
#include "router/route_sections_info.h"
#include "router/route_util.h"

namespace e2e_noa::planning {

namespace {

constexpr double kDefaultLanePathLength = 150.0;

}

absl::StatusOr<std::vector<mapping::LanePath>> BuildLocalMap(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const RouteNaviInfo& route_navi_info) {
  const RouteSectionsInfo route_sections_info(psmm, &route_sections);
  if (route_sections_info.empty() ||
      route_sections_info.front().lane_ids.empty()) {
    return absl::NotFoundError("Route section is empty.");
  }

  std::vector<mapping::LanePath> lane_paths;
  const auto& current_section_info = route_sections_info.front();
  lane_paths.reserve(current_section_info.lane_ids.size());

  for (const auto lane_id : current_section_info.lane_ids) {
    auto lane_path = FindLanePathFromLaneAlongRouteSections(
        psmm, route_sections_info, route_navi_info, lane_id,
        route_sections_info.start_fraction(), kDefaultLanePathLength);
    if (!lane_path.IsEmpty()) {
      lane_paths.emplace_back(lane_path);
    };
  }
  return lane_paths;
}

}  
