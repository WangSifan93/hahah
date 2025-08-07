#include "util/qtfm_segment_matcher_util.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "glog/logging.h"

namespace e2e_noa::planning {

namespace qtfm_util {
bool SegmentRatioRangeInHalfPlane(const Vec2d& p0, const Vec2d& p1,
                                  const Segment2d& hp, bool reverse,
                                  double* start, double* end) {
  constexpr double kEpsilon = 1e-9;
  DCHECK(start != nullptr);
  DCHECK(end != nullptr);

  double r0 = hp.ProductOntoUnit(p0);
  double r1 = hp.ProductOntoUnit(p1);
  if (reverse) {
    r0 = -r0;
    r1 = -r1;
  }
  const double r01 = r1 - r0;
  if (std::abs(r01) < kEpsilon) {
    if (r0 >= 0.0) {
      *start = 0.0;
      *end = 1.0;
      return true;
    } else {
      return false;
    }
  }
  const double critical_ratio = -r0 / r01;
  if (r01 > 0) {
    if (r1 < 0.0) {
      return false;
    } else {
      *end = 1.0;
      *start = std::max(0., critical_ratio);
      return true;
    }
  } else {
    if (r0 < 0.0) {
      return false;
    } else {
      *start = 0.0;
      *end = std::min(1.0, critical_ratio);
      return true;
    }
  }
}

bool SegmentIntersectsFanRegion(const Vec2d& p0, const Vec2d& p1,
                                const Segment2d& right_bound,
                                const Segment2d& left_bound) {
  double rbl_start, rbl_end, lbr_start, lbr_end;
  if (!SegmentRatioRangeInHalfPlane(p0, p1, right_bound, false, &rbl_start,
                                    &rbl_end)) {
    return false;
  }
  if (!SegmentRatioRangeInHalfPlane(p0, p1, left_bound, true, &lbr_start,
                                    &lbr_end)) {
    return false;
  }

  return rbl_end >= lbr_start && lbr_end >= rbl_start;
}

bool NoReverseRegion(const AABox2d& box, const Segment2d& prev_div,
                     const Segment2d& cur_div) {
  const std::array<Vec2d, 4> points = box.GetAllCorners();
  for (int i = 0; i < 4; ++i) {
    const int next_i = (i + 1) % 4;
    if (SegmentIntersectsFanRegion(points[i], points[next_i], prev_div,
                                   cur_div)) {
      return false;
    }
  }
  return true;
}

}  // namespace qtfm_util

}  // namespace e2e_noa::planning
