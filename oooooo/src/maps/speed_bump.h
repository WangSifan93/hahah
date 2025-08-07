#ifndef MAP_SPEED_BUMP_H
#define MAP_SPEED_BUMP_H
#include "maps/map_def.h"
#include "math/polygon2d.h"

namespace ad_e2e {
namespace planning {
class SpeedBump {
 public:
  explicit SpeedBump(const SpeedBumpInfo &stop_line_info);

  const std::string &id() const { return id_; }
  const std::vector<Point2d> &points() const { return points_; }

  double DistanceTo(const Point2d &point) const;
  const math::Polygon2d &polygon() const { return polygon_; };

  void add_intersected_lanes(const std::string &lane_id) {
    intersected_lanes_.insert(lane_id);
  }

 private:
  std::string id_;
  std::vector<Point2d> points_;
  math::Polygon2d polygon_;
  std::set<std::string> intersected_lanes_;
};
using SpeedBumpPtr = std::shared_ptr<SpeedBump>;
using SpeedBumpConstPtr = std::shared_ptr<const SpeedBump>;

}  // namespace planning
}  // namespace ad_e2e
#endif
