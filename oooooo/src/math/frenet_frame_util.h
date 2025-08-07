#ifndef ONBOARD_MATH_FRENET_FRAME_UTIL_H_
#define ONBOARD_MATH_FRENET_FRAME_UTIL_H_

#include "absl/types/span.h"
#include "math/frenet_common.h"
#include "math/vec.h"

namespace e2e_noa::frenet_frame_util {

Vec2d SLToXY(const FrenetCoordinate& sl, absl::Span<const Vec2d> anchor_points,
             absl::Span<const double> anchor_s);
FrenetCoordinate XYToSLBruteForce(const Vec2d& xy,
                                  absl::Span<const Vec2d> anchor_points,
                                  absl::Span<const double> anchor_s);

}  // namespace e2e_noa::frenet_frame_util

#endif
