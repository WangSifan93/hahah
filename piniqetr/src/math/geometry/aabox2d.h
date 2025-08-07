#ifndef MATH_GEOMETRY_AABOX2D
#define MATH_GEOMETRY_AABOX2D

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "math/geometry/segment2d.h"
#include "math/util.h"
#include "math/vec.h"

namespace e2e_noa {

class AABox2d {
 public:
  AABox2d() = default;

  AABox2d(const Vec2d& center, const double length, const double width);

  AABox2d(double half_length, double half_width, const Vec2d& center);

  AABox2d(const Vec2d& one_corner, const Vec2d& opposite_corner);

  explicit AABox2d(absl::Span<const Vec2d> points);

  bool operator==(const AABox2d& o) const;

  const Vec2d& center() const { return center_; }

  double center_x() const { return center_.x(); }

  double center_y() const { return center_.y(); }

  double length() const { return half_length_ * 2.0; }

  double width() const { return half_width_ * 2.0; }

  double half_length() const { return half_length_; }

  double half_width() const { return half_width_; }

  double area() const { return half_length_ * half_width_ * 4.0; }

  double min_x() const { return center_.x() - half_length_; }

  double max_x() const { return center_.x() + half_length_; }

  double min_y() const { return center_.y() - half_width_; }

  double max_y() const { return center_.y() + half_width_; }

  Vec2d half_size() const { return {half_length_, half_width_}; }

  double half_diagonal() const { return half_size().norm(); }

  void GetAllCorners(std::vector<Vec2d>* const corners) const;

  std::array<Vec2d, 4> GetAllCorners() const;

  bool IsPointIn(const Vec2d& point) const;

  bool IsPointOnBoundary(const Vec2d& point) const;

  bool Contains(const AABox2d& other_aabox) const;

  double DistanceTo(const Vec2d& point) const;

  double SquaredDistanceTo(const Vec2d& point) const;

  double DistanceTo(const AABox2d& box) const;

  bool HasOverlap(const AABox2d& box) const {
    return std::abs(box.center_x() - center_.x()) <=
               box.half_length() + half_length_ &&
           std::abs(box.center_y() - center_.y()) <=
               box.half_width() + half_width_;
  }

  bool HasOverlap(const Segment2d& line_segmnet) const;

  double ComputeOverlapArea(const AABox2d& box) const;

  void Shift(const Vec2d& shift_vec);

  void MergeFrom(const AABox2d& other_box);

  void MergeFrom(const Vec2d& other_point);

  std::string DebugString() const;

 private:
  static constexpr double kEpsilon = 1e-10;

  Vec2d center_;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
};

inline AABox2d::AABox2d(const Vec2d& center, double length, double width)
    : center_(center), half_length_(length * 0.5), half_width_(width * 0.5) {
  DCHECK_GT(half_length_, -kEpsilon);
  DCHECK_GT(half_width_, -kEpsilon);
}

inline AABox2d::AABox2d(double half_length, double half_width,
                        const Vec2d& center)
    : center_(center), half_length_(half_length), half_width_(half_width) {
  DCHECK_GT(half_length_, -kEpsilon);
  DCHECK_GT(half_width_, -kEpsilon);
}

inline AABox2d::AABox2d(const Vec2d& one_corner, const Vec2d& opposite_corner)
    : AABox2d((one_corner + opposite_corner) * 0.5,
              std::abs(one_corner.x() - opposite_corner.x()),
              std::abs(one_corner.y() - opposite_corner.y())) {}

inline AABox2d::AABox2d(absl::Span<const Vec2d> points) {
  CHECK(!points.empty());
  double min_x = points[0].x();
  double max_x = points[0].x();
  double min_y = points[0].y();
  double max_y = points[0].y();
  for (const auto& point : points) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }
  center_ = {(min_x + max_x) * 0.5, (min_y + max_y) * 0.5};
  half_length_ = (max_x - min_x) * 0.5;
  half_width_ = (max_y - min_y) * 0.5;
}

inline void AABox2d::GetAllCorners(std::vector<Vec2d>* const corners) const {
  CHECK_NOTNULL(corners)->clear();
  corners->reserve(4);
  corners->emplace_back(center_.x() + half_length_, center_.y() - half_width_);
  corners->emplace_back(center_.x() + half_length_, center_.y() + half_width_);
  corners->emplace_back(center_.x() - half_length_, center_.y() + half_width_);
  corners->emplace_back(center_.x() - half_length_, center_.y() - half_width_);
}

inline std::array<Vec2d, 4> AABox2d::GetAllCorners() const {
  return {Vec2d{center_.x() + half_length_, center_.y() - half_width_},
          Vec2d{center_.x() + half_length_, center_.y() + half_width_},
          Vec2d{center_.x() - half_length_, center_.y() + half_width_},
          Vec2d{center_.x() - half_length_, center_.y() - half_width_}};
}

inline bool AABox2d::operator==(const AABox2d& o) const {
  return center_ == o.center() && half_length_ == o.half_length() &&
         half_width_ == o.half_width_;
}

inline bool AABox2d::IsPointIn(const Vec2d& point) const {
  return std::abs(point.x() - center_.x()) <= half_length_ + kEpsilon &&
         std::abs(point.y() - center_.y()) <= half_width_ + kEpsilon;
}

inline bool AABox2d::IsPointOnBoundary(const Vec2d& point) const {
  const double dx = std::abs(point.x() - center_.x());
  const double dy = std::abs(point.y() - center_.y());
  return (std::abs(dx - half_length_) <= kEpsilon &&
          dy <= half_width_ + kEpsilon) ||
         (std::abs(dy - half_width_) <= kEpsilon &&
          dx <= half_length_ + kEpsilon);
}

inline bool AABox2d::Contains(const AABox2d& other_aabox) const {
  return !(min_x() > other_aabox.min_x() || min_y() > other_aabox.min_y() ||
           max_x() < other_aabox.max_x() || max_y() < other_aabox.max_y());
}

inline double AABox2d::DistanceTo(const Vec2d& point) const {
  const double dx = std::abs(point.x() - center_.x()) - half_length_;
  const double dy = std::abs(point.y() - center_.y()) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

inline double AABox2d::SquaredDistanceTo(const Vec2d& point) const {
  const double dx = std::abs(point.x() - center_.x()) - half_length_;
  const double dy = std::abs(point.y() - center_.y()) - half_width_;
  return Sqr(std::max(0.0, dx)) + Sqr(std::max(0.0, dy));
}

inline double AABox2d::DistanceTo(const AABox2d& box) const {
  const double dx =
      std::abs(box.center_x() - center_.x()) - box.half_length() - half_length_;
  const double dy =
      std::abs(box.center_y() - center_.y()) - box.half_width() - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

inline double AABox2d::ComputeOverlapArea(const AABox2d& box) const {
  if (!HasOverlap(box)) return 0;
  double overlap_area =
      (std::min(max_x(), box.max_x()) - std::max(min_x(), box.min_x())) *
      (std::min(max_y(), box.max_y()) - std::max(min_y(), box.min_y()));
  return overlap_area;
}

inline void AABox2d::Shift(const Vec2d& shift_vec) { center_ += shift_vec; }

inline void AABox2d::MergeFrom(const AABox2d& other_box) {
  const double x1 = std::min(min_x(), other_box.min_x());
  const double x2 = std::max(max_x(), other_box.max_x());
  const double y1 = std::min(min_y(), other_box.min_y());
  const double y2 = std::max(max_y(), other_box.max_y());
  center_ = Vec2d((x1 + x2) * 0.5, (y1 + y2) * 0.5);
  half_length_ = x2 - center_.x();
  half_width_ = y2 - center_.y();
}

inline void AABox2d::MergeFrom(const Vec2d& other_point) {
  const double x1 = std::min(min_x(), other_point.x());
  const double x2 = std::max(max_x(), other_point.x());
  const double y1 = std::min(min_y(), other_point.y());
  const double y2 = std::max(max_y(), other_point.y());
  center_ = Vec2d((x1 + x2) * 0.5, (y1 + y2) * 0.5);
  half_length_ = x2 - center_.x();
  half_width_ = y2 - center_.y();
}

inline std::string AABox2d::DebugString() const {
  return absl::StrCat("aabox2d(center = ", center_.DebugString(),
                      "  length = ", length(), "  width = ", width(), ")");
}

}  // namespace e2e_noa

#endif
