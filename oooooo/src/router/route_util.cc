#include "router/route_util.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <ostream>
#include <random>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "absl/types/span.h"
#include "affine_transformation.pb.h"
#include "boost/geometry/algorithms/simplify.hpp"
#include "boost/geometry/core/cs.hpp"
#include "boost/geometry/geometries/register/linestring.hpp"
#include "boost/geometry/geometries/register/point.hpp"
#include "container/strong_int.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "lane_path.pb.h"
#include "maps/map_def.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_util.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/segment2d.h"
#include "math/geometry/util.h"
#include "math/util.h"
#include "math/vec.h"
#include "util/file_util.h"
#include "util/map_util.h"
#include "util/proto_util.h"
#include "util/status_macros.h"
#include "util/time_util.h"

BOOST_GEOMETRY_REGISTER_POINT_2D(
    e2e_noa::Vec2d, double,
    ::boost::geometry::cs::cartesian, operator[](0), operator[](1));
BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<e2e_noa::Vec2d>);

namespace e2e_noa {
namespace planning {

namespace {

constexpr double kMaxRetrogradeMeters = -1.0;
constexpr double kMinForwardSpeed = 0.5;
constexpr double kLcOutermostDist = 800;
constexpr double kMinIgnoreRampDist = 350;

}  // namespace

namespace internal {}

mapping::LanePath FindLanePathFromLaneAlongRouteSections(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, mapping::ElementId start_lane_id,
    double start_frac, double extend_len) {
  if (extend_len <= 0.0) return {};

  const auto& sections = sections_info.section_segments();
  const auto& lane_navi_info_map = route_navi_info.route_lane_info_map;
  std::vector<mapping::ElementId> lane_ids;
  mapping::ElementId cur_lane_id = start_lane_id;
  double cur_frac = start_frac;
  if (cur_frac == 1.0) lane_ids.push_back(cur_lane_id);
  int sec_idx = 0;
  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, cur_lane_id);
    if (cur_frac == 1.0) {
      if (++sec_idx == sections.size()) break;
      const auto& next_sec = sections[sec_idx];
      mapping::ElementId outgoing_lane_id = mapping::kInvalidElementId;
      int min_lc_num = std::numeric_limits<int>::max();
      for (const auto next_lane_id : lane_info.next_lane_ids()) {
        if (next_sec.id_idx_map.contains(next_lane_id)) {
          const auto* lane_navi_info_ptr =
              FindOrNull(lane_navi_info_map, next_lane_id);
          const double lc_num = lane_navi_info_ptr == nullptr
                                    ? std::numeric_limits<int>::max()
                                    : lane_navi_info_ptr->min_lc_num_to_target;
          if (lc_num < min_lc_num) {
            outgoing_lane_id = next_lane_id;
            min_lc_num = lc_num;
          }
        }
      }
      if (outgoing_lane_id == mapping::kInvalidElementId) break;
      cur_lane_id = outgoing_lane_id;
      cur_frac = 0.0;
    } else {
      lane_ids.push_back(cur_lane_id);
      const double lane_len = lane_info.curve_length();
      if (sec_idx + 1 == sections.size()) {
        cur_frac = std::min(sections.back().end_fraction,
                            extend_len / lane_len + cur_frac);
        break;
      }
      const double len = lane_len * (sections[sec_idx].end_fraction - cur_frac);
      if (len >= extend_len) {
        cur_frac += extend_len / lane_len;
        break;
      }
      extend_len -= len;
      cur_frac = 1.0;
    }
  }
  return mapping::LanePath(psmm.map_ptr(), std::move(lane_ids), start_frac,
                           cur_frac);
}

bool IsTravelMatchedValid(const CompositeLanePath::CompositeIndex& first,
                          double first_fraction,
                          const CompositeLanePath::CompositeIndex& second,
                          double second_fraction, int64_t last_time_micros,
                          int64_t cur_time_micros, double distance,
                          double speed, double reroute_threshold_meters,
                          double estimate_speed) {
  VLOG(2) << "first" << first.DebugString() << ", second"
          << second.DebugString() << ", first_fraction:" << first_fraction
          << ", second_fraction:" << second_fraction
          << ", last_time_micros:" << last_time_micros
          << ", cur_time_micros:" << cur_time_micros
          << ", current speed:" << speed
          << ", estimate_speed:" << estimate_speed;
  if (last_time_micros == -1) {
    return true;
  }

  if (speed >= kMinForwardSpeed && distance <= kMaxRetrogradeMeters) {
    VLOG(2) << "Retrograde is not allowed.distance:" << distance
            << ", max:" << kMaxRetrogradeMeters << ", speed:" << speed;
    return false;
  }
  constexpr double kAllowedDriftThres = 35.0 * 0.5;
  const double reroute_threshold =
      std::min(reroute_threshold_meters,
               MicroSecondsToSeconds(cur_time_micros - last_time_micros) *
                   estimate_speed);
  VLOG(2) << "RerouteThresholdMeters:" << reroute_threshold_meters
          << ", reroute_threshold:" << reroute_threshold
          << ", drift_allowed: " << kAllowedDriftThres
          << ", distance:" << distance;
  return distance <= std::max(reroute_threshold, kAllowedDriftThres);
}

bool CrossSolidBoundary(
    const std::vector<ad_e2e::planning::LaneBoundaryType>& types,
    bool lc_left) {
  using namespace ad_e2e::planning;
  static const std::unordered_set<LineType> non_crossable_line_types = {
      SOLID, SOLID_SOLID, FISH_SOLID, CURB_LINE, SHADED_AREA};
  for (const auto& type : types) {
    return non_crossable_line_types.find(type.line_type) !=
           non_crossable_line_types.end();
  }

  return false;
}

Vec3d ComputePointOnLane(const std::vector<Vec2d>& geo_points,
                         double fraction) {
  const auto [pos, t] = mapping::ComputePosition(geo_points, fraction);
  const auto z = Lerp(geo_points[pos].x(), geo_points[pos + 1].y(), t);
  return Vec3d((pos >= geo_points.size() - 1)
                   ? geo_points[geo_points.size() - 1]
                   : Lerp(geo_points[pos], geo_points[pos + 1], t),
               z);
}

}  // namespace planning
}  // namespace e2e_noa
