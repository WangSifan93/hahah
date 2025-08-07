#ifndef ONBOARD_PLANNER_UTIL_QTFM_SEGMENT_MATCHER_UTIL_H_
#define ONBOARD_PLANNER_UTIL_QTFM_SEGMENT_MATCHER_UTIL_H_

#include "math/geometry/aabox2d.h"
#include "math/geometry/segment2d.h"
#include "math/vec.h"

namespace e2e_noa::planning {

namespace qtfm_util {
bool SegmentRatioRangeInHalfPlane(const Vec2d& p0, const Vec2d& p1,
                                  const Segment2d& hp, bool reverse,
                                  double* start, double* end);

bool SegmentIntersectsFanRegion(const Vec2d& p0, const Vec2d& p1,
                                const Segment2d& right_bound,
                                const Segment2d& left_bound);

bool NoReverseRegion(const AABox2d& box, const Segment2d& prev_div,
                     const Segment2d& cur_div);

}  // namespace qtfm_util

}  // namespace e2e_noa::planning

#endif
