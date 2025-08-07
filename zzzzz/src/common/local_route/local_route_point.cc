#include "apps/planning/src/common/local_route/local_route_point.h"

#include "point_factory.h"
namespace zark {
namespace planning {

void LocalRoutePoint::RemoveDuplicates(std::vector<LocalRoutePoint>* points) {
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedEpsilon * kDuplicatedEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

::common::PathPoint LocalRoutePoint::ToPathPoint(double s) const {
  return ::util::PointFactory::ToPathPoint(x(), y(), 0.0, s, heading(), kappa_,
                                           dkappa_);
}

}  // namespace planning
}  // namespace zark
