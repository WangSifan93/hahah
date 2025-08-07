#ifndef MATH_GEOMETRY_GRID_FRAME_UTIL
#define MATH_GEOMETRY_GRID_FRAME_UTIL

#include "absl/types/span.h"
#include "math/geometry/circle2d.h"
#include "math/geometry/grid_frame.h"
#include "math/geometry/polygon2d.h"
#include "math/vec.h"

namespace e2e_noa {

YMonotonicGridSet2di CalculateGridsTouchingConvexPolygon(
    const GridFrame2d& grid_frame, const Polygon2d& polygon);

YMonotonicGridSet2di ComputeGridsTouchingConvexPolygonPoints(
    const GridFrame2d& grid_frame, absl::Span<const Vec2d> points);

YMonotonicGridSet2di CalculateGridsTouchingCircle(const GridFrame2d& grid_frame,
                                                  const Circle2d& circle);

}  // namespace e2e_noa

#endif
