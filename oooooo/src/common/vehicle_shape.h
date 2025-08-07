#ifndef ONBOARD_PLANNER_COMMON_VEHICLE_SHAPE_H_
#define ONBOARD_PLANNER_COMMON_VEHICLE_SHAPE_H_

#include <memory>
#include <optional>
#include <vector>

#include "aabox3d.pb.h"
#include "math/geometry/box2d.h"
#include "math/geometry/circle2d.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/segment2d.h"
#include "math/util.h"
#include "math/vec.h"
#include "planner_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

class VehicleShapeBase {
 public:
  VehicleShapeBase(const VehicleGeometryParamsProto& vehicle_geo_params,
                   const Vec2d& rac, const Vec2d& tangent) {
    if (vehicle_geo_params.has_left_mirror() &&
        vehicle_geo_params.has_right_mirror()) {
      const Vec2d left_mirror_center =
          rac + vehicle_geo_params.left_mirror().x() * tangent +
          vehicle_geo_params.left_mirror().y() * tangent.Perp();
      const double left_mirror_radius =
          vehicle_geo_params.left_mirror().length() * 0.5;
      left_mirror_ = Circle2d(left_mirror_center, left_mirror_radius);

      const Vec2d right_mirror_center =
          rac + vehicle_geo_params.right_mirror().x() * tangent +
          vehicle_geo_params.right_mirror().y() * tangent.Perp();
      const double right_mirror_radius =
          vehicle_geo_params.right_mirror().length() * 0.5;
      right_mirror_ = Circle2d(right_mirror_center, right_mirror_radius);
    }
  }
  virtual ~VehicleShapeBase() = default;
  virtual const Vec2d& center() const = 0;
  virtual const double heading() const = 0;
  virtual std::vector<Vec2d> GetCornersWithBufferCounterClockwise(
      double lat_buffer, double lon_buffer) const = 0;
  virtual double MainBodyDistanceTo(const Segment2d& seg) const = 0;
  virtual double MainBodyDistanceTo(const Polygon2d& polygon) const = 0;
  virtual double MainBodyHasOverlapWithBuffer(const Segment2d& seg,
                                              double lat_buffer,
                                              double lon_buffer) const = 0;
  virtual double MainBodyHasOverlapWithBuffer(const Polygon2d& polygon,
                                              double lat_buffer,
                                              double lon_buffer) const = 0;
  const std::optional<Circle2d>& left_mirror() const { return left_mirror_; }
  const std::optional<Circle2d>& right_mirror() const { return right_mirror_; }

  template <typename T>
  std::optional<double> LeftMirrorDistanceTo(const T& target) const {
    if (!left_mirror_.has_value()) return std::nullopt;
    return target.DistanceTo(left_mirror_->center()) - left_mirror_->radius();
  }
  template <typename T>
  std::optional<double> RightMirrorDistanceTo(const T& target) const {
    if (!right_mirror_.has_value()) return std::nullopt;
    return target.DistanceTo(right_mirror_->center()) - right_mirror_->radius();
  }

  template <typename T>
  bool LeftMirrorHasOverlapWithBuffer(const T& target, double buffer) const {
    if (!left_mirror_.has_value()) return false;
    return target.DistanceSquareTo(left_mirror_->center()) <=
           Sqr(buffer + left_mirror_->radius());
  }
  template <typename T>
  bool RightMirrorHasOverlapWithBuffer(const T& target, double buffer) const {
    if (!right_mirror_.has_value()) return false;
    return target.DistanceSquareTo(right_mirror_->center()) <=
           Sqr(buffer + right_mirror_->radius());
  }

  template <typename T>
  bool HasOverlapWithBuffer(const T& target, double lat_buffer,
                            double lon_buffer, bool consider_mirrors) const {
    bool has_overlap =
        MainBodyHasOverlapWithBuffer(target, lat_buffer, lon_buffer);
    if (consider_mirrors) {
      has_overlap = has_overlap ||
                    LeftMirrorHasOverlapWithBuffer(target, lat_buffer) ||
                    RightMirrorHasOverlapWithBuffer(target, lat_buffer);
    }
    return has_overlap;
  }

 protected:
  std::optional<Circle2d> left_mirror_;
  std::optional<Circle2d> right_mirror_;
};
using VehicleShapeBasePtr = std::unique_ptr<VehicleShapeBase>;

class VehicleBoxShape : public VehicleShapeBase {
 public:
  VehicleBoxShape(const VehicleGeometryParamsProto& vehicle_geo_params,
                  const Vec2d& rac, const Vec2d& center, const Vec2d& tangent,
                  double theta, double half_length, double half_width)
      : VehicleShapeBase(vehicle_geo_params, rac, tangent),
        main_body_(Box2d(half_length, half_width, center, theta, tangent)) {}
  const Vec2d& center() const override { return main_body_.center(); }
  const double heading() const override { return main_body_.heading(); }
  std::vector<Vec2d> GetCornersWithBufferCounterClockwise(
      double lat_buffer, double lon_buffer) const override {
    return main_body_.GetCornersWithBufferCounterClockwise(lat_buffer,
                                                           lon_buffer);
  }
  double MainBodyDistanceTo(const Segment2d& seg) const override {
    return main_body_.DistanceTo(seg);
  }
  double MainBodyDistanceTo(const Polygon2d& polygon) const override {
    return polygon.DistanceTo(main_body_);
  }
  double MainBodyHasOverlapWithBuffer(const Segment2d& seg, double lat_buffer,
                                      double lon_buffer) const override {
    return main_body_.HasOverlapWithBuffer(seg, lat_buffer, lon_buffer);
  }
  double MainBodyHasOverlapWithBuffer(const Polygon2d& polygon,
                                      double lat_buffer,
                                      double lon_buffer) const override {
    return polygon.HasOverlapWithBuffer(main_body_, lat_buffer, lon_buffer);
  }

 private:
  Box2d main_body_;
};
}  // namespace e2e_noa::planning

#endif
