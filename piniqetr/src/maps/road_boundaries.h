#ifndef MAP_ROAD_BOUNDARIES_H
#define MAP_ROAD_BOUNDARIES_H

#include <memory>

#include "maps/map_def.h"
#include "maps/road_boundary.h"
#include "math/line_curve2d.h"

namespace ad_e2e {
namespace planning {
class RoadBoundaries {
 public:
  explicit RoadBoundaries(const std::vector<RoadBoundaryConstPtr> &boundaries);
  ~RoadBoundaries() = default;

  const std::vector<RoadBoundaryConstPtr> &road_boundaries() const {
    return boundaries_;
  };

 private:
  std::vector<RoadBoundaryConstPtr> boundaries_;
};

using RoadBoundariesPtr = std::shared_ptr<RoadBoundaries>;
using RoadBoundariesConstPtr = std::shared_ptr<const RoadBoundaries>;

}  // namespace planning
}  // namespace ad_e2e

#endif
