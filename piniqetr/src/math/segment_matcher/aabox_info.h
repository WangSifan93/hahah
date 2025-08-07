#ifndef MATH_SEGMENT_MATCHER_AABOX_INFO
#define MATH_SEGMENT_MATCHER_AABOX_INFO

#include <string>
#include <utility>

#include "math/geometry/aabox2d.h"
#include "math/geometry/segment2d.h"

namespace e2e_noa {

class AABoxInfo {
 public:
  explicit AABoxInfo(const Segment2d& seg);
  AABoxInfo(const Segment2d& seg, int64_t index);

  AABox2d ComputeAABox() const;

  double DistanceSquareTo(const Vec2d& point) const;

  int64_t index() const;

  const Segment2d& segment() const;

 private:
  Segment2d segment_;

  int64_t index_ = -1;
};
static_assert(sizeof(AABoxInfo) == 80);

inline AABoxInfo::AABoxInfo(const Segment2d& seg) : segment_(seg) {
  DCHECK((segment_.start() != segment_.end()))
      << segment_.DebugString() << " id:  " << index_;
}

inline AABoxInfo::AABoxInfo(const Segment2d& seg, int64_t index)
    : segment_(seg), index_(index) {
  DCHECK((segment_.start() != segment_.end()))
      << segment_.DebugString() << " id:  " << index_;
}

inline AABox2d AABoxInfo::ComputeAABox() const {
  return AABox2d(segment_.start(), segment_.end());
}

inline double AABoxInfo::DistanceSquareTo(const Vec2d& point) const {
  return segment_.DistanceSquareTo(point);
}

inline int64_t AABoxInfo::index() const { return index_; }

inline const Segment2d& AABoxInfo::segment() const { return segment_; }

}  // namespace e2e_noa

#endif
