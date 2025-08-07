#ifndef MATH_GEOMETRY_POLYGON2D_UTIL
#define MATH_GEOMETRY_POLYGON2D_UTIL

#include <string>

#include "absl/strings/str_cat.h"
#include "math/geometry/box2d.h"
#include "math/geometry/halfplane.h"
#include "math/geometry/polygon2d.h"
#include "math/vec.h"

namespace e2e_noa::polygon2d {

Polygon2d CreateRegularPolygon(int num_points, const Vec2d& center,
                               double radius, double first_point_angle);

struct PolygonHalfPlaneOverlap {
  double dist;
  double in;
  double out;
  std::string DebugString() const {
    return absl::StrCat("dist: ", dist, ", in: ", in, ", out: ", out);
  }
};

PolygonHalfPlaneOverlap ComputePolygonHalfPlaneOverlap(
    const Polygon2d& polygon, const HalfPlane& halfplane);

struct PolygonBoxOverlap {
  double lat_dist;
  double in;
  double out;
  std::string DebugString() const {
    return absl::StrCat("lat_dist: ", lat_dist, ", in: ", in, ", out: ", out);
  }
};

PolygonBoxOverlap ComputePolygonBoxOverlap(const Polygon2d& polygon,
                                           const Box2d& box);

}  // namespace e2e_noa::polygon2d

#endif
