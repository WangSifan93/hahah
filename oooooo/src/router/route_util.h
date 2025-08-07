#ifndef ST_PLANNING_ROUTER_ROUTE_UTIL
#define ST_PLANNING_ROUTER_ROUTE_UTIL

#include <algorithm>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "lane_path.pb.h"
#include "maps/lane_path.h"
#include "maps/lane_path_util.h"
#include "maps/lane_point.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_defs.h"
#include "math/vec.h"
#include "plan/composite_lane_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "positioning.pb.h"
#include "route.pb.h"
#include "router/navi/route_navi_info.h"
#include "router/route_manager_output.h"
#include "router/route_sections.h"
#include "router/route_sections_info.h"

namespace e2e_noa {
namespace planning {

mapping::LanePath FindLanePathFromLaneAlongRouteSections(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, mapping::ElementId start_lane_id,
    double start_frac, double extend_len);

bool IsTravelMatchedValid(const CompositeLanePath::CompositeIndex& first,
                          double first_fraction,
                          const CompositeLanePath::CompositeIndex& second,
                          double second_fraction, int64_t last_time_micros,
                          int64_t cur_time_micros, double distance,
                          double speed, double reroute_threshold_meters,
                          double estimate_speed);

bool CrossSolidBoundary(
    const std::vector<ad_e2e::planning::LaneBoundaryType>& types, bool lc_left);

Vec3d ComputePointOnLane(const std::vector<Vec2d>& geo_points, double fraction);

inline std::vector<mapping::ElementId> ConvertToElementIdFrom(
    const ::google::protobuf::RepeatedField<std::string>& lane_ids) {
  std::vector<mapping::ElementId> res;
  res.reserve(lane_ids.size());
  std::transform(
      lane_ids.begin(), lane_ids.end(), std::back_inserter(res),
      [](const std::string& lane_id) { return mapping::ElementId(lane_id); });
  return res;
}

}  // namespace planning
}  // namespace e2e_noa

#endif
