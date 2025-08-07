#include "math/circle.h"

#include "math/fast_math.h"
#include "math/util.h"

namespace e2e_noa {

Vec2d Circle::EvaluateXY(double theta) const {
  return center_ + radius_ * Vec2d::FastUnitFromAngle(theta);
}
double Circle::EvaluateTheta(const Vec2d& xy) const {
  return fast_math::Atan2(xy.y() - center_.y(), xy.x() - center_.x());
}
double Circle::EvaluateTangent(double theta) const {
  return NormalizeAngle(theta - M_PI_2);
}

double Circle::DistanceTo(const Vec2d& xy) const {
  return std::abs(xy.DistanceTo(center_) - radius_);
}

}  // namespace e2e_noa
