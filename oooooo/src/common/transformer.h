#ifndef COMMON_TRANSFORMER
#define COMMON_TRANSFORMER
#include "common/type_def.h"
#include "math/vec3d.h"

namespace ad_e2e::planning {
class Transformer {
 public:
  Transformer() = delete;
  explicit Transformer(const TransformInfo &info);
  explicit Transformer(const math::Vec2d &delta_pos, const double &delta_yaw);
  ~Transformer() = default;

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

 private:
  math::Isometry3d tf_ = math::Isometry3d::Identity();
};
}  // namespace ad_e2e::planning

#endif
