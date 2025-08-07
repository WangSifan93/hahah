#ifndef MATH_GEOMETRY_SEGMENT2D_H_
#define MATH_GEOMETRY_SEGMENT2D_H_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "math/util.h"
#include "math/vec.h"

namespace e2e_noa {

class Segment2d {
 public:
  Segment2d()
      : unit_direction_(1, 0),
        heading_(std::numeric_limits<double>::quiet_NaN()) {}

  Segment2d(const Segment2d& s);

  Segment2d& operator=(const Segment2d& s);

  Segment2d(const Vec2d& start, const Vec2d& end);

  Segment2d(double length, const Vec2d& start, const Vec2d& tangent);

  const Vec2d& start() const { return start_; }

  const Vec2d& end() const { return end_; }

  const Vec2d& unit_direction() const { return unit_direction_; }
  Vec2d rotate_expand(const double angle, const double length) const;
  void Reset(const Vec2d& start, const Vec2d& end);

  Vec2d center() const { return (start_ + end_) * 0.5; }

  double heading() const {
    double heading = heading_.load(std::memory_order_acquire);
    if (std::isnan(heading)) {
      heading = unit_direction_.Angle();
      heading_.store(heading, std::memory_order_release);
    }
    return heading;
  }

  double cos_heading() const { return unit_direction_.x(); }

  double sin_heading() const { return unit_direction_.y(); }

  double length() const { return length_; }

  void set_length(double length);

  void Rotate(const Vec2d& center, const Vec2d& angle_unit) {
    start_ = center + (start_ - center).Rotate(angle_unit);
    end_ = center + (end_ - center).Rotate(angle_unit);
    unit_direction_ = unit_direction_.Rotate(angle_unit);
  }

  double length_sqr() const { return length_ * length_; }

  double min_x() const { return std::min(start_.x(), end_.x()); }

  double max_x() const { return std::max(start_.x(), end_.x()); }

  double min_y() const { return std::min(start_.y(), end_.y()); }

  double max_y() const { return std::max(start_.y(), end_.y()); }

  double SignedDistanceTo(const Vec2d& point) const;

  double DistanceTo(const Vec2d& point) const;

  double DistanceTo(const Vec2d& point, Vec2d* const nearest_pt) const;

  double DistanceTo(const Segment2d& segment) const;

  double DistanceTo(const Segment2d& segment, Vec2d* nearest_pt,
                    Vec2d* other_nearest_pt) const;

  double DistanceSquareTo(const Vec2d& point) const;

  double DistanceSquareTo(const Vec2d& point, Vec2d* const nearest_pt) const;

  bool IsPointIn(const Vec2d& point) const;

  bool HasIntersect(const Segment2d& other_segment) const;

  bool GetIntersect(const Segment2d& other_segment, Vec2d* const point) const;

  double ProjectOntoUnit(const Vec2d& point) const;

  double ProductOntoUnit(const Vec2d& point) const;

  double GetPerpendicularFoot(const Vec2d& point,
                              Vec2d* const foot_point) const;

  void ClampByYMax(double y_max);

  void ClampByYMin(double y_min);

  void Shift(const Vec2d& offset);

  void Scale(double gain);

  std::string DebugString() const;
  std::string DebugStringFullPrecision() const;

  void Reverse();

 private:
  static constexpr double kEpsilon = 1e-10;

  static double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                          const Vec2d& end_point_2) {
    return Vec2d(end_point_1 - start_point)
        .CrossProd(end_point_2 - start_point);
  }

  static bool IsWithin(double val, double bound1, double bound2) {
    if (bound1 > bound2) {
      std::swap(bound1, bound2);
    }
    return val >= bound1 - kEpsilon && val <= bound2 + kEpsilon;
  }

  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  double length_ = 0.0;
  mutable std::atomic<double> heading_;
};

inline Segment2d::Segment2d(const Vec2d& start, const Vec2d& end)
    : start_(start),
      end_(end),
      heading_(std::numeric_limits<double>::quiet_NaN()) {
  const Vec2d d = end_ - start_;
  length_ = d.Length();
  unit_direction_ = length_ <= kEpsilon ? Vec2d(0, 0) : d / length_;
}

inline Segment2d::Segment2d(double length, const Vec2d& start,
                            const Vec2d& tangent)
    : start_(start),
      end_(start + length * tangent),
      unit_direction_(tangent),
      length_(length),
      heading_(std::numeric_limits<double>::quiet_NaN()) {
  CHECK_GT(length, 0.0);
}

inline Segment2d::Segment2d(const Segment2d& s)
    : start_(s.start()),
      end_(s.end()),
      unit_direction_(s.unit_direction()),
      length_(s.length()),
      heading_(s.heading_.load()) {}

inline Segment2d& Segment2d::operator=(const Segment2d& s) {
  start_ = s.start();
  end_ = s.end();
  unit_direction_ = s.unit_direction();
  length_ = s.length();
  heading_.store(s.heading_.load());
  return *this;
}

inline void Segment2d::set_length(double length) {
  CHECK_GT(length, 0.0);
  length_ = length;
  end_ = start_ + unit_direction_ * length_;
}

inline double Segment2d::SignedDistanceTo(const Vec2d& point) const {
  if (length_ <= kEpsilon) return point.DistanceTo(start_);
  const Vec2d v = point - start_;
  const double signed_lateral = v.CrossProd(unit_direction_);
  const double proj = v.Dot(unit_direction_);
  if (proj <= 0.0) {
    return signed_lateral > 0.0 ? v.norm() : -v.norm();
  }
  if (proj >= length_) {
    return signed_lateral > 0.0 ? point.DistanceTo(end_)
                                : -point.DistanceTo(end_);
  }
  return signed_lateral;
}

inline double Segment2d::DistanceTo(const Vec2d& point) const {
  if (length_ <= kEpsilon) {
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return Hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

inline bool Segment2d::IsPointIn(const Vec2d& point) const {
  if (length_ <= kEpsilon) {
    return std::abs(point.x() - start_.x()) <= kEpsilon &&
           std::abs(point.y() - start_.y()) <= kEpsilon;
  }
  const double prod = CrossProd(point, start_, end_);
  if (std::abs(prod) > kEpsilon) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}

inline double Segment2d::DistanceSquareTo(const Vec2d& point) const {
  if (UNLIKELY(length_ <= kEpsilon)) {
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return Sqr(x0) + Sqr(y0);
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return Sqr(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

inline double Segment2d::DistanceSquareTo(const Vec2d& point,
                                          Vec2d* const nearest_pt) const {
  CHECK_NOTNULL(nearest_pt);
  if (UNLIKELY(length_ <= kEpsilon)) {
    *nearest_pt = start_;
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return Sqr(x0) + Sqr(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.DistanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return Sqr(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

inline double Segment2d::ProjectOntoUnit(const Vec2d& point) const {
  return unit_direction_.dot(point - start_);
}

inline double Segment2d::ProductOntoUnit(const Vec2d& point) const {
  return unit_direction_.CrossProd(point - start_);
}

}  // namespace e2e_noa

#endif
