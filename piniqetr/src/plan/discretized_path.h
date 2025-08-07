#ifndef ONBOARD_PLANNER_DISCRETIZED_PATH_H_
#define ONBOARD_PLANNER_DISCRETIZED_PATH_H_

#include <utility>
#include <vector>

#include "glog/logging.h"
#include "math/frenet_common.h"
#include "math/geometry/polygon2d.h"
#include "math/util.h"
#include "math/vec.h"
#include "trajectory_point.pb.h"

namespace e2e_noa {
namespace planning {

class DiscretizedPath : public std::vector<PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<PathPoint> path_points);

  double length() const {
    if (empty()) {
      return 0.0;
    }
    return back().s();
  }

  FrenetCoordinate XYToSL(const Vec2d& pos) const;

  FrenetPolygon XYToSL(const Polygon2d& contour, const Vec2d& pos) const;

  PathPoint Evaluate(double path_s) const;

  PathPoint EvaluateReverse(double path_s) const;

  static DiscretizedPath CreateResampledPath(
      std::vector<PathPoint> raw_path_points, double interval) {
    CHECK_GE(raw_path_points.size(), 2);
    DiscretizedPath raw_path(std::move(raw_path_points));
    double s = 0.0;
    std::vector<PathPoint> path_points;
    path_points.reserve(CeilToInteger(raw_path.length() / interval));
    while (s < raw_path.length()) {
      path_points.push_back(raw_path.Evaluate(s));
      s += interval;
    }
    return DiscretizedPath(std::move(path_points));
  }

 protected:
  std::vector<PathPoint>::const_iterator QueryLowerBound(double path_s) const;
  std::vector<PathPoint>::const_iterator QueryUpperBound(double path_s) const;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
