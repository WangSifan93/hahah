#ifndef MATH_GEOMETRY_HALFPLANE
#define MATH_GEOMETRY_HALFPLANE

#include <string>

#include "absl/strings/str_cat.h"
#include "affine_transformation.pb.h"
#include "halfplane.pb.h"
#include "math/geometry/util.h"
#include "math/vec.h"

namespace e2e_noa {

class HalfPlane {
 public:
  HalfPlane() = default;

  explicit HalfPlane(const HalfPlaneProto& proto) { FromProto(proto); }

  HalfPlane(const Vec2d& start, const Vec2d& end, const Vec2d& dir)
      : start_(start), end_(end), center_((start + end) * 0.5), tangent_(dir) {}

  HalfPlane(const Vec2d& start, const Vec2d& end)
      : start_(start), end_(end), center_((start + end) * 0.5) {
    const Vec2d d = end - start;
    tangent_ = d.Unit();
  }

  HalfPlane Inversed() const { return HalfPlane(end_, start_, -tangent_); }

  const Vec2d& start() const { return start_; }
  const Vec2d& end() const { return end_; }
  const Vec2d& center() const { return center_; }
  const Vec2d& tangent() const { return tangent_; }
  double length() const { return tangent_.Dot(end_ - start_); }

  double lon_proj(const Vec2d& p) const { return tangent_.Dot(p - start_); }

  double lat_proj(const Vec2d& p) const {
    return tangent_.CrossProd(p - start_);
  }

  Vec2d Transform(const Vec2d& p) const {
    const Vec2d diff = p - start_;
    return Vec2d(tangent_.Dot(diff), tangent_.CrossProd(diff));
  }

  Vec2d InvTransform(const Vec2d& p) const {
    const Vec2d neg_y_dir(tangent_.x(), -tangent_.y());
    return Vec2d(neg_y_dir.Dot(p), neg_y_dir.CrossProd(p)) + start_;
  }

  bool IsPointInside(const Vec2d& p) const {
    return tangent_.CrossProd(p - start_) > 0.0;
  }

  void FromProto(const HalfPlaneProto& proto) {
    *this =
        HalfPlane(Vec2dFromProto(proto.start()), Vec2dFromProto(proto.end()));
  }

  void ToProto(HalfPlaneProto* proto) const { ToProto(start_, end_, proto); }

  std::string DebugString() const {
    return absl::StrCat("HalfPlane({", start_.x(), ", ", start_.y(), "}", ",{",
                        end_.x(), ", ", end_.y(), "})");
  }

  static void ToProto(const Vec2d& start, const Vec2d& end,
                      HalfPlaneProto* proto) {
    CHECK_NOTNULL(proto)->mutable_start()->set_x(start.x());
    proto->mutable_start()->set_y(start.y());
    proto->mutable_end()->set_x(end.x());
    proto->mutable_end()->set_y(end.y());
  }

 private:
  Vec2d start_;
  Vec2d end_;
  Vec2d center_;

  Vec2d tangent_;
};

}  // namespace e2e_noa

#endif
