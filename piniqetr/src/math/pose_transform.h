#ifndef AD_E2E_PLANNING_MATH_POSE_TRANSFROM_H_
#define AD_E2E_PLANNING_MATH_POSE_TRANSFROM_H_
#include "common/type_def.h"
#include "math/vec3d.h"

namespace ad_e2e {
namespace planning {
class PoseTransform {
 public:
  PoseTransform() = delete;
  explicit PoseTransform(const TransformInfo &info);
  explicit PoseTransform(const math::Vec2d &delta_pos, const double &delta_yaw);
  ~PoseTransform() = default;

  Point2d Trans(const Point2d &pt) const;
  std::vector<Point2d> Trans(const std::vector<Point2d> &pts) const;
  PathPoint Trans(const PathPoint &pt) const;
  std::vector<PathPoint> Trans(const std::vector<PathPoint> &pts) const;
  TrajectoryPoint Trans(const TrajectoryPoint &pt) const;

  Point2d TransInverse(const Point2d &pt) const;
  std::vector<Point2d> TransInverse(const std::vector<Point2d> &pts) const;
  PathPoint TransInverse(const PathPoint &pt) const;
  std::vector<PathPoint> TransInverse(const std::vector<PathPoint> &pts) const;
  TrajectoryPoint TransInverse(const TrajectoryPoint &pt) const;

  static math::Vector3d ToEulerAngles(const math::Quaterniond &q);

 private:
  math::Isometry3d tf_ = math::Isometry3d::Identity();
};
}  // namespace planning
}  // namespace ad_e2e

#endif
