#include "descriptor/speed_bump.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "maps/maps_helper.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_common.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/segment2d.h"
#include "math/util.h"
#include "math/vec.h"

namespace e2e_noa {
namespace planning {

namespace {
bool CalculateSpeedRegionRange(const PlanPassage& passage,
                               const Polygon2d& polygon,
                               std::pair<double, double>* min_max_s,
                               Vec2d* start_point, Vec2d* end_point) {
  min_max_s->first = std::numeric_limits<double>::infinity();
  min_max_s->second = -std::numeric_limits<double>::infinity();

  for (const auto index : passage.stations().index_from(1)) {
    const StationIndex prev_index(index.value() - 1);
    const Segment2d segment(passage.stations()[prev_index].xy(),
                            passage.stations()[index].xy());

    if (polygon.HasOverlap(segment)) {
      Vec2d first;
      Vec2d last;
      polygon.GetOverlap(segment, &first, &last);
      const auto sl_first = passage.QueryFrenetCoordinateAt(first);
      if (!sl_first.ok()) {
        return false;
      }
      if (sl_first.value().s < min_max_s->first) {
        min_max_s->first = sl_first.value().s;
        *start_point = first;
      }
      if (sl_first.value().s > min_max_s->second) {
        min_max_s->second = sl_first.value().s;
        *end_point = first;
      }

      const auto sl_last = passage.QueryFrenetCoordinateAt(last);
      if (!sl_last.ok()) {
        return false;
      }
      if (sl_last.value().s < min_max_s->first) {
        min_max_s->first = sl_last.value().s;
        *start_point = last;
      }
      if (sl_last.value().s > min_max_s->second) {
        min_max_s->second = sl_last.value().s;
        *end_point = last;
      }
    }
  }

  return !std::isinf(min_max_s->first) && !std::isinf(min_max_s->second);
}
}  // namespace

std::vector<ConstraintProto::SpeedRegionProto> BuildSpeedBumpConstraints(
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage) {
  struct SpeedBumpInfo {
    mapping::ElementId id;
    Polygon2d polygon;
    double speed_limit;
  };
  std::vector<SpeedBumpInfo> speed_bumps;
  speed_bumps.reserve(psmm.map_ptr()->speed_bump_map().size());
  for (const auto& speed_bump_pair : psmm.map_ptr()->speed_bump_map()) {
    if (!speed_bump_pair.second) continue;

    const double v = Mph2Mps(120.0);
    speed_bumps.push_back({.id = speed_bump_pair.first,
                           .polygon = speed_bump_pair.second->polygon(),
                           .speed_limit = v});
  }

  std::vector<ConstraintProto::SpeedRegionProto> speed_bump_constraints;
  speed_bump_constraints.reserve(speed_bumps.size());
  for (const auto& speed_bump : speed_bumps) {
    Vec2d start_point;
    Vec2d end_point;
    std::pair<double, double> min_max_s;
    const bool success = CalculateSpeedRegionRange(
        passage, speed_bump.polygon, &min_max_s, &start_point, &end_point);
    if (success) {
      ConstraintProto::SpeedRegionProto speed_bump_constraint;
      start_point.ToProto(speed_bump_constraint.mutable_start_point());
      end_point.ToProto(speed_bump_constraint.mutable_end_point());
      speed_bump_constraint.set_start_s(min_max_s.first);
      speed_bump_constraint.set_end_s(min_max_s.second);
      speed_bump_constraint.set_max_speed(speed_bump.speed_limit);
      speed_bump_constraint.set_min_speed(0.0);
      speed_bump_constraint.mutable_source()->mutable_speed_bump()->set_id(
          speed_bump.id);
      speed_bump_constraint.set_id(
          absl::StrFormat("speed_bump_%d", speed_bump.id));
      speed_bump_constraints.push_back(std::move(speed_bump_constraint));
    }
  }

  return speed_bump_constraints;
}

}  // namespace planning
}  // namespace e2e_noa
