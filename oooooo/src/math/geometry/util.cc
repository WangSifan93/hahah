#include "math/geometry/util.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <vector>

#include "glog/logging.h"

namespace e2e_noa {

std::pair<double, double> ProjectBoxToRay(const Vec2d& ray_center,
                                          const Vec2d& ray_dir,
                                          const Box2d& box) {
  const double center = ray_dir.Dot(box.center() - ray_center);
  const Vec2d box_tangent = box.tangent();
  const double offset =
      0.5 * (std::abs(ray_dir.Dot(box_tangent * box.length())) +
             std::abs(ray_dir.Dot(box_tangent.Perp() * box.width())));
  return {center - offset, center + offset};
}

std::vector<Circle2d> GenerateCirclesInsideBoundingBox(
    const Box2d& box, double circle_center_dist) {
  if (std::abs(box.half_length() - box.half_width()) < 0.01) {
    return {
        Circle2d(box.center(), std::min(box.half_length(), box.half_width()))};
  }

  if (box.length() < box.width()) {
    return GenerateCirclesInsideBoundingBox(
        Box2d(box.half_width(), box.half_length(), box.center(),
              NormalizeAngle(box.heading() + M_PI_2), box.tangent().Perp()),
        circle_center_dist);
  }

  const double circle_radius = box.half_width();
  const int num_circles =
      CeilToInteger((box.length() - 2.0 * circle_radius) / circle_center_dist) +
      1;

  std::vector<Circle2d> circles;
  circles.reserve(num_circles);

  const Vec2d rear_center = box.center() - box.tangent() * box.half_length();
  const double inner_circle_step =
      (box.length() - 2.0 * circle_radius) / (num_circles - 1);
  for (int i = 0; i < num_circles; ++i) {
    circles.push_back(Circle2d(
        rear_center + box.tangent() * (circle_radius + i * inner_circle_step),
        circle_radius));
  }

  return circles;
}

Mat3d ComputeMeanValueForRotationMatrices(
    const std::vector<Mat3d>& rotation_matrices) {
  CHECK_GT(rotation_matrices.size(), 1);
  Mat3d R = rotation_matrices[0];
  const double epsilon = 1e-13;
  for (int loop = 0; loop < 20; ++loop) {
    Vec3d r = Vec3d::Zero();
    for (const auto& mat : rotation_matrices) {
      Eigen::AngleAxisd angle_axis(R.transpose() * mat);
      r += angle_axis.angle() * angle_axis.axis();
    }
    r /= rotation_matrices.size();

    if (r.norm() < epsilon) return R;

    Eigen::AngleAxisd so3_r(r.norm(), r.normalized());
    R = R * so3_r;
  }
  LOG(WARNING) << "Compute Mean Value For Roatation Matrices : bad converge!!!";
  return R;
}

}  // namespace e2e_noa
