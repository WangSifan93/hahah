#ifndef PLANNING_MAP_LANE_BOUNDARIES_H
#define PLANNING_MAP_LANE_BOUNDARIES_H

#include <memory>

#include "maps/lane_boundary.h"
#include "maps/map_def.h"
#include "math/line_curve2d.h"

namespace ad_e2e {
namespace planning {
class LaneBoundaries {
 public:
  explicit LaneBoundaries(const std::vector<LaneBoundaryConstPtr> &boundaries);
  ~LaneBoundaries() = default;

  const math::LineCurve2d &line_curve() const { return line_curve_; }

  const std::vector<LaneBoundaryType> &boundary_types() const {
    return boundary_types_;
  }

  const std::vector<LaneBoundaryConstPtr> &lane_boundaries() const {
    return boundaries_;
  };

  const int32_t GetBoundarySegmentIndex(const double &s) const;

 private:
  std::vector<LaneBoundaryConstPtr> boundaries_;
  std::vector<LaneBoundaryType> boundary_types_;
  math::LineCurve2d line_curve_;
};

using LaneBoundariesPtr = std::shared_ptr<LaneBoundaries>;
using LaneBoundariesConstPtr = std::shared_ptr<const LaneBoundaries>;

}  // namespace planning
}  // namespace ad_e2e

#endif
