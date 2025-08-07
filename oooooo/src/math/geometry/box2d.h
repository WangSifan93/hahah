#ifndef MATH_GEOMETRY_BOX2D
#define MATH_GEOMETRY_BOX2D

#include <float.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "box2d.pb.h"
#include "glog/logging.h"
#include "math/geometry/aabox2d.h"
#include "math/geometry/segment2d.h"
#include "math/util.h"
#include "math/vec.h"

namespace e2e_noa {

class Box2d {
 public:
  enum Corner {
    FRONT_LEFT = 0,
    REAR_LEFT = 1,
    REAR_RIGHT = 2,
    FRONT_RIGHT = 3,
  };
  Box2d() = default;

  Box2d(const Vec2d& center, double heading, double length, double width);

  Box2d(double half_length, double half_width, const Vec2d& center,
        double heading);

  Box2d(double half_length, double half_width, const Vec2d& center,
        double heading, const Vec2d& tangent);

  explicit Box2d(const Box2dProto& proto) { FromProto(proto); }

  Box2d(const Vec2d& center, const Vec2d& tangent, double length, double width);

  Box2d(double half_length, double half_width, const Vec2d& center,
        const Vec2d& tangent);

  Box2d(const Segment2d& axis, double width);

  explicit Box2d(const AABox2d& aabox);

  static Box2d CreateAABox(const Vec2d& one_corner,
                           const Vec2d& opposite_corner);

  const Vec2d& center() const { return center_; }

  double center_x() const { return center_.x(); }

  double center_y() const { return center_.y(); }

  double length() const { return half_length_ * 2.0; }

  double width() const { return half_width_ * 2.0; }

  double half_length() const { return half_length_; }

  double half_width() const { return half_width_; }

  double radius() const { return Hypot(half_length_, half_width_); }

  double heading() const { return heading_; }

  double cos_heading() const { return cos_heading_; }

  double sin_heading() const { return sin_heading_; }

  double area() const { return half_length_ * half_width_ * 4.0; }

  double diagonal() const { return Hypot(half_length_, half_width_) * 2.0; }

  Vec2d tangent() const { return Vec2d(cos_heading_, sin_heading_); }

  Vec2d GetCorner(Corner corner) const;

  Vec2d FrontCenterPoint() const;

  Vec2d RearCenterPoint() const;
  void InitCorners();

  std::vector<Vec2d> GetCornersWithBufferCounterClockwise(
      double lat_buffer, double lon_buffer) const;

  std::array<Segment2d, 4> GetEdgesWithBufferCounterClockwise(
      double lat_buffer, double lon_buffer) const;

  std::array<Segment2d, 4> GetEdgesCounterClockwise() const;

  std::vector<Vec2d> GetCornersCounterClockwise() const {
    return GetCornersWithBufferCounterClockwise(0.0, 0.0);
  }
  void GetAllCorners(std::vector<Vec2d>* const corners) const;

  const std::vector<Vec2d>& GetAllCorners() const;

  bool IsPointInWithBuffer(const Vec2d& point, double lat_buffer,
                           double lon_buffer) const;

  bool IsPointIn(const Vec2d& point) const {
    return IsPointInWithBuffer(point, 0.0, 0.0);
  }

  bool IsPointOnBoundary(const Vec2d& point) const;

  bool IsInAABox(const AABox2d& aabox) const;

  double DistanceTo(const Vec2d& point) const;

  double DistanceTo(const Segment2d& line_segment) const;

  double DistanceTo(const Box2d& box) const;

  bool HasOverlapWithBuffer(const Segment2d& line_segment, double lat_buffer,
                            double lon_buffer) const;

  bool HasOverlap(const Segment2d& line_segment) const {
    return HasOverlapWithBuffer(line_segment, 0.0, 0.0);
  }

  bool HasOverlapWithBuffer(const Box2d& box, double lat_buffer,
                            double lon_buffer) const;

  bool HasOverlap(const Box2d& box) const {
    return HasOverlapWithBuffer(box, 0.0, 0.0);
  }

  bool HasOverlap(const AABox2d& aabox) const;

  [[nodiscard]] AABox2d GetAABox() const;

  void RotateFromCenter(double rotate_angle);

  void Shift(const Vec2d& shift_vec);

  void MirrorByX();

  [[nodiscard]] Box2d Transform(const Vec2d& translation) const;

  [[nodiscard]] Box2d Transform(double rotation) const;

  [[nodiscard]] Box2d Transform(const Vec2d& translation,
                                double rotation) const;

  [[nodiscard]] Box2d AffineTransform(double rotation) const;

  [[nodiscard]] Box2d AffineTransform(const Vec2d& translation,
                                      double rotation) const;

  void LongitudinalExtend(double extension_length);
  [[nodiscard]] Box2d ExtendedAtFront(double extension_length) const;
  [[nodiscard]] Box2d ExtendedAtRear(double extension_length) const;

  void LateralExtend(double extension_length);

  void LongitudinalExtendByRatio(double extension_ratio);

  void LateralExtendByRatio(double extension_ratio);

  void FromProto(const Box2dProto& proto);

  void ToProto(Box2dProto* proto) const;

  std::string DebugString() const;
  std::string DebugStringFullPrecision() const;
  AABox2d aabox() const;
  double DistanceSquareTo(const Vec2d& point) const;

 private:
  static constexpr double kEpsilon = 1e-10;

  Vec2d center_;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;
  std::vector<Vec2d> corners_;

  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();
};

inline AABox2d Box2d::aabox() const { return GetAABox(); }

inline double Box2d::DistanceSquareTo(const Vec2d& point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy =
      std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return Sqr(std::max(0.0, dy));
  }
  if (dy <= 0.0) {
    return Sqr(dx);
  }
  return Sqr(dx) + Sqr(dy);
}

inline Box2d::Box2d(const Vec2d& center, double heading, double length,
                    double width)
    : Box2d(length * 0.5, width * 0.5, center, heading) {}

inline Box2d::Box2d(double half_length, double half_width, const Vec2d& center,
                    double heading)
    : center_(center),
      half_length_(half_length),
      half_width_(half_width),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) {
  CHECK_GT(half_length_, -kEpsilon);
  CHECK_GT(half_width_, -kEpsilon);
  InitCorners();
}

inline Box2d::Box2d(double half_length, double half_width, const Vec2d& center,
                    double heading, const Vec2d& tangent)
    : center_(center),
      half_length_(half_length),
      half_width_(half_width),
      heading_(heading),
      cos_heading_(tangent.x()),
      sin_heading_(tangent.y()) {
  CHECK_GT(half_length_, -kEpsilon);
  CHECK_GT(half_width_, -kEpsilon);
  InitCorners();
}

inline Box2d::Box2d(const Vec2d& center, const Vec2d& tangent, double length,
                    double width)
    : Box2d(length * 0.5, width * 0.5, center, tangent) {}

inline Box2d::Box2d(double half_length, double half_width, const Vec2d& center,
                    const Vec2d& tangent)
    : center_(center),
      half_length_(half_length),
      half_width_(half_width),
      heading_(tangent.Angle()),
      cos_heading_(tangent.x()),
      sin_heading_(tangent.y()) {
  CHECK_GT(half_length_, -kEpsilon);
  CHECK_GT(half_width_, -kEpsilon);

  DCHECK_LT(std::abs(tangent.Sqr() - 1.0), 1e-6);
  InitCorners();
}

inline Box2d::Box2d(const Segment2d& axis, double width)
    : center_(axis.center()),
      half_length_(axis.length() * 0.5),
      half_width_(width * 0.5),
      heading_(axis.heading()),
      cos_heading_(axis.cos_heading()),
      sin_heading_(axis.sin_heading()) {
  CHECK_GT(half_length_, -kEpsilon);
  CHECK_GT(half_width_, -kEpsilon);
  InitCorners();
}

inline Box2d::Box2d(const AABox2d& aabox)
    : center_(aabox.center()),
      half_length_(aabox.half_length()),
      half_width_(aabox.half_width()),
      heading_(0.0),
      cos_heading_(1.0),
      sin_heading_(0.0) {
  CHECK_GT(half_length_, -kEpsilon);
  CHECK_GT(half_width_, -kEpsilon);
  InitCorners();
}

inline std::vector<Vec2d> Box2d::GetCornersWithBufferCounterClockwise(
    double lat_buffer, double lon_buffer) const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d h = unit * (half_length_ + lon_buffer);
  const Vec2d w = unit.Perp() * (half_width_ + lat_buffer);
  return {
      center_ + h + w,
      center_ - h + w,
      center_ - h - w,
      center_ + h - w,
  };
}

inline std::array<Segment2d, 4> Box2d::GetEdgesWithBufferCounterClockwise(
    double lat_buffer, double lon_buffer) const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d unit_perp = unit.Perp();

  const double buffered_half_length = half_length_ + lon_buffer;
  const double buffered_half_width = half_width_ + lat_buffer;

  const double buffered_length = 2.0 * buffered_half_length;
  const double buffered_width = 2.0 * buffered_half_width;

  const Vec2d h = unit * buffered_half_length;
  const Vec2d w = unit_perp * buffered_half_width;

  return {Segment2d(buffered_length, center_ + h + w, -unit),
          Segment2d(buffered_width, center_ - h + w, -unit_perp),
          Segment2d(buffered_length, center_ - h - w, unit),
          Segment2d(buffered_width, center_ + h - w, unit_perp)};
}

inline std::array<Segment2d, 4> Box2d::GetEdgesCounterClockwise() const {
  return GetEdgesWithBufferCounterClockwise(0.0, 0.0);
}

inline Vec2d Box2d::GetCorner(Corner corner) const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d h = unit * half_length_;
  const Vec2d w = unit.Perp() * half_width_;
  switch (corner) {
    case FRONT_LEFT:
      return center_ + h + w;
    case REAR_LEFT:
      return center_ - h + w;
    case REAR_RIGHT:
      return center_ - h - w;
    case FRONT_RIGHT:
      return center_ + h - w;
    default:
      CHECK(false) << "unexpected enum";
  }
}

inline Vec2d Box2d::FrontCenterPoint() const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d h = unit * half_length_;
  return center_ + h;
}
inline Vec2d Box2d::RearCenterPoint() const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d h = unit * half_length_;
  return center_ - h;
}

inline Box2d Box2d::CreateAABox(const Vec2d& one_corner,
                                const Vec2d& opposite_corner) {
  const double x1 = std::min(one_corner.x(), opposite_corner.x());
  const double x2 = std::max(one_corner.x(), opposite_corner.x());
  const double y1 = std::min(one_corner.y(), opposite_corner.y());
  const double y2 = std::max(one_corner.y(), opposite_corner.y());
  return Box2d({(x1 + x2) * 0.5, (y1 + y2) * 0.5}, 0.0, x2 - x1, y2 - y1);
}

inline bool Box2d::IsPointInWithBuffer(const Vec2d& point, double lat_buffer,
                                       double lon_buffer) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
  const double box_x = half_length_ + lon_buffer;
  const double box_y = half_width_ + lat_buffer;
  return dx <= box_x + kEpsilon && dy <= box_y + kEpsilon;
}

inline bool Box2d::IsPointOnBoundary(const Vec2d& point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
  return (std::abs(dx - half_length_) <= kEpsilon &&
          dy <= half_width_ + kEpsilon) ||
         (std::abs(dy - half_width_) <= kEpsilon &&
          dx <= half_length_ + kEpsilon);
}

inline bool Box2d::IsInAABox(const AABox2d& aabox) const {
  return aabox.Contains(GetAABox());
}

inline double Box2d::DistanceTo(const Vec2d& point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy =
      std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return Hypot(dx, dy);
}

inline bool Box2d::HasOverlap(const AABox2d& aabox) const {
  const double shift_x = aabox.center_x() - center_.x();
  const double shift_y = aabox.center_y() - center_.y();

  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;

  const double dx3 = aabox.half_length();
  const double dy4 = -aabox.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_) + std::abs(dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_) + std::abs(-dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x) <= std::abs(dx1) + std::abs(dx2) + dx3 &&
         std::abs(shift_y) <= std::abs(dy1) + std::abs(dy2) - dy4;
}

inline double Box2d::DistanceTo(const Box2d& box) const {
  if (HasOverlap(box)) {
    return 0.0;
  }
  double distance_sqr = std::numeric_limits<double>::infinity();
  const auto self_corners = GetCornersCounterClockwise();
  for (const auto& corner : self_corners) {
    distance_sqr = std::min(distance_sqr, box.DistanceSquareTo(corner));
  }
  const auto other_corners = box.GetCornersCounterClockwise();
  for (const auto& corner : other_corners) {
    distance_sqr = std::min(distance_sqr, DistanceSquareTo(corner));
  }
  return std::sqrt(distance_sqr);
}

inline AABox2d Box2d::GetAABox() const {
  const double dx1 = std::abs(cos_heading_ * half_length_);
  const double dy1 = std::abs(sin_heading_ * half_length_);
  const double dx2 = std::abs(sin_heading_ * half_width_);
  const double dy2 = std::abs(cos_heading_ * half_width_);
  return AABox2d(center_, (dx1 + dx2) * 2.0, (dy1 + dy2) * 2.0);
}

inline void Box2d::RotateFromCenter(const double rotate_angle) {
  heading_ = NormalizeAngle(heading_ + rotate_angle);
  cos_heading_ = std::cos(heading_);
  sin_heading_ = std::sin(heading_);
}

inline void Box2d::Shift(const Vec2d& shift_vec) { center_ += shift_vec; }

inline void Box2d::MirrorByX() {
  center_.y() = -center_.y();
  heading_ = -heading_;
  sin_heading_ = -sin_heading_;
}

inline Box2d Box2d::Transform(const Vec2d& translation) const {
  Box2d box(*this);
  box.Shift(translation);
  return box;
}

inline Box2d Box2d::Transform(double rotation) const {
  const double heading = NormalizeAngle(heading_ + rotation);
  return Box2d(half_length_, half_width_, center_, heading);
}

inline Box2d Box2d::Transform(const Vec2d& translation, double rotation) const {
  const Vec2d center = center_ + translation;
  const double heading = NormalizeAngle(heading_ + rotation);
  return Box2d(half_length_, half_width_, center, heading);
}

inline Box2d Box2d::AffineTransform(double rotation) const {
  const Vec2d center = center_.Rotate(rotation);
  const double heading = NormalizeAngle(heading_ + rotation);
  return Box2d(half_length_, half_width_, center, heading);
}

inline Box2d Box2d::AffineTransform(const Vec2d& translation,
                                    double rotation) const {
  const Vec2d center = center_.Rotate(rotation) + translation;
  const double heading = NormalizeAngle(heading_ + rotation);
  return Box2d(half_length_, half_width_, center, heading);
}

inline void Box2d::LongitudinalExtend(const double extension_length) {
  half_length_ += extension_length * 0.5;
}
inline Box2d Box2d::ExtendedAtFront(double extension_length) const {
  Box2d new_box = *this;
  const double half_ext_len = 0.5 * extension_length;
  new_box.Shift(half_ext_len * Vec2d(cos_heading_, sin_heading_));
  new_box.LongitudinalExtend(half_ext_len);
  return new_box;
}
inline Box2d Box2d::ExtendedAtRear(double extension_length) const {
  Box2d new_box = *this;
  const double half_ext_len = 0.5 * extension_length;
  new_box.Shift(-half_ext_len * Vec2d(cos_heading_, sin_heading_));
  new_box.LongitudinalExtend(half_ext_len);
  return new_box;
}

inline void Box2d::LateralExtend(const double extension_length) {
  half_width_ += extension_length * 0.5;
}

inline void Box2d::LongitudinalExtendByRatio(double extension_ratio) {
  half_length_ *= extension_ratio;
}

inline void Box2d::LateralExtendByRatio(double extension_ratio) {
  half_width_ *= extension_ratio;
}

inline void Box2d::FromProto(const Box2dProto& proto) {
  *this = Box2d(Vec2d(proto.x(), proto.y()), proto.heading(), proto.length(),
                proto.width());
}

inline void Box2d::ToProto(Box2dProto* proto) const {
  proto->set_x(center().x());
  proto->set_y(center().y());
  proto->set_heading(heading());
  proto->set_length(length());
  proto->set_width(width());
}

inline std::string Box2d::DebugString() const {
  return absl::StrCat("box2d ( center = ", center_.DebugString(),
                      "  heading = ", heading_, "  length = ", length(),
                      "  width = ", width(), " )");
}

inline std::string Box2d::DebugStringFullPrecision() const {
  return absl::StrFormat(
      "Box2d(/*half_length=*/%.*e, /*half_width=*/%.*e, /*center=*/%s, "
      "/*heading=*/%.*e)",
      DECIMAL_DIG, half_length_, DECIMAL_DIG, half_width_,
      center_.DebugStringFullPrecision(), DECIMAL_DIG, heading_);
}

}  // namespace e2e_noa

#endif
