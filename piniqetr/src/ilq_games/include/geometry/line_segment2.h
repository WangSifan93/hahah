//////////////////////////////////////////////////////////////////////////////
//
// Line segment in 2D.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_GEOMETRY_LINE_SEGMENT2_H
#define ILQGAMES_GEOMETRY_LINE_SEGMENT2_H

#include <glog/logging.h>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class LineSegment2 {
 public:
  ~LineSegment2() {}
  LineSegment2(const Point2& point1, const Point2& point2)
      : p1_(point1),
        p2_(point2),
        length_((point1 - point2).norm()),
        unit_direction_((point2 - point1) / length_) {
    CHECK_GT(length_, constants::kSmallNumber);
  }

  // Accessors.
  float Length() const { return length_; }
  const Point2& FirstPoint() const { return p1_; }
  const Point2& SecondPoint() const { return p2_; }
  const Point2& UnitDirection() const { return unit_direction_; }
  float Heading() const {
    return std::atan2(UnitDirection().y(), UnitDirection().x());
  }

  // Compute which side of this line segment the query point is on.
  // Returns true for the "right" side and false for the "left.""
  bool Side(const Point2& query) const;

  // Find closest point on this line segment to a given point (and optionally
  // the signed squared distance, where right is positive, and whether or not
  // the closest point is an endpoint).
  Point2 ClosestPoint(const Point2& query, bool* is_endpoint = nullptr,
                      float* signed_squared_distance = nullptr) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Point2 p1_;
  Point2 p2_;
  float length_;
  Point2 unit_direction_;
};  // struct LineSegment2

}  // namespace e2e_noa::planning

#endif
