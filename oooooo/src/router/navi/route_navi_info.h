#ifndef ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_H_

#include <limits>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "maps/semantic_map_defs.h"
#include "route.pb.h"
#include "router/route_sections.h"

namespace e2e_noa {
namespace planning {
struct RouteNaviInfo {
  struct RouteLaneInfo {
    double max_driving_distance = 0.0;
    double max_reach_length = 0.0;
    double recommend_reach_length = 0.0;
    int min_lc_num_to_target = std::numeric_limits<int>::max();
    int lc_num_within_driving_dist = std::numeric_limits<int>::max();
    double len_before_merge_lane = DBL_MAX;
    std::string DebugString() const {
      return absl::StrFormat(
          "{max_driving_distance: %f, max_reach_length: %f, "
          "recommend_reach_length: %f, min_lc_num_to_target: %d, "
          "lc_num_within_driving_dist: %d, len_before_merge_lane: %f}",
          max_driving_distance, max_reach_length, recommend_reach_length,
          min_lc_num_to_target, lc_num_within_driving_dist,
          len_before_merge_lane);
    }
  };

  struct NaviSectionInfo {
    double length_before_intersection = 0.0;
    e2e_noa::NaviSectionInfoProto::Direction intersection_direction =
        e2e_noa::NaviSectionInfoProto::STRAIGHT;
  };

  absl::flat_hash_map<mapping::ElementId, RouteLaneInfo> route_lane_info_map;
  absl::flat_hash_map<mapping::SectionId, NaviSectionInfo>
      navi_section_info_map;
  RouteSections back_extend_sections;

  std::string DebugString() const {
    return absl::StrJoin(
        route_lane_info_map, "\n", [](std::string* out, auto i) {
          out->append(absl::StrCat(i.first, i.second.DebugString()));
        });
  }

  void FromProto(const e2e_noa::RouteNaviInfoProto& navi_info_proto);
  void ToProto(e2e_noa::RouteNaviInfoProto* navi_info_proto) const;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
