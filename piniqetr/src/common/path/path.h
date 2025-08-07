#ifndef AD_E2E_PLANNING_COMMON_PATH_PATH_H
#define AD_E2E_PLANNING_COMMON_PATH_PATH_H

#include "common/log.h"
#include "common/type_def.h"
#include "math/box2d.h"
#include "math/cartesian_frenet_conversion.h"
#include "math/line_segment2d.h"
#include "math/linear_interpolation.h"

namespace ad_e2e {
namespace planning {

class Path {
 public:
  explicit Path(const std::vector<Point2d> &xy_points) {
    set_points(xy_points);
  }

  explicit Path(std::vector<PathPoint> &&path_points,
                bool compute_path_profile = true);

  void set_points(const std::vector<Point2d> &xy_points);
  void set_points(std::vector<PathPoint> &&path_points,
                  bool compute_path_profile = true);
  void Reset() { points_.clear(); }
  const std::vector<PathPoint> &points() const { return points_; }

  PathPoint GetPointAtS(const double &accum_s) const;

  int GetNearestIndexAtS(const double &accum_s) const;

  double GetDistance(const Point2d &point, PathPoint *nearest_point) const;

  bool SLToXY(const SLPoint &sl_point, Point2d *xy_point,
              PathPoint *ref_point) const;

  bool SLToXY(std::vector<PathPoint> &path_points) const;

  bool XYToSL(const Point2d &xy_point, SLPoint *sl_point,
              PathPoint *ref_point) const;

  bool XYToSL(const TrajectoryPoint &traj_point,
              FrenetPoint *frenet_point) const;

  bool XYToSL(std::vector<PathPoint> &path_points) const;

  bool XYToSL(const math::Box2d &box_2d, SLBoundary *sl_boundary) const;

  bool XYToSL(const std::vector<Point2d> &corners,
              SLBoundary *sl_boundary) const;

  double length() const {
    return points_.size() < 2 ? 0.0 : points_.back().accum_s;
  }

  bool IsValid() const {
    return points_.size() >= 2 && (!std::isnan(points_.front().x())) &&
           (!std::isnan(points_.front().y()));
  }

  bool SamplePoints(const double &start_accu_s, const double &length,
                    const double &interval,
                    std::vector<PathPoint> *const path_points) const;

  bool SamplePoints(const std::vector<double> &accum_sequence,
                    std::vector<PathPoint> *const path_points) const;

  bool HasIntersect(const math::LineSegment2d &input_segment,
                    math::Vec2d *const point) const;

  Path() = default;
  ~Path() = default;

 private:
  static bool ComputePathProfile(std::vector<PathPoint> &path_points);

  static bool ComputeAccumulatedS(std::vector<PathPoint> &path_points);

  static PathPoint MatchToPath(const std::vector<PathPoint> &reference_line,
                               const double &x, const double &y);

  inline size_t GetIndexOfLowerBound(const double &accum_s) const;

 private:
  std::vector<PathPoint> points_;
};
using PathPtr = std::shared_ptr<Path>;
}  // namespace planning
}  // namespace ad_e2e
#endif
