#ifndef MAP_CLEAR_AERA_H
#define MAP_CLEAR_AERA_H
#include "map_def.h"
#include "math/polygon2d.h"

namespace ad_e2e {
namespace planning {
class ClearArea {
 public:
  explicit ClearArea(const ClearAreaInfo &clear_area_info) {
    clear_area_info_ = clear_area_info;
    points_ = clear_area_info.points;
    polygon_.SetPoints(clear_area_info.points);
  }

  const std::vector<math::Vec2d> &points() const { return points_; }
  const std::string &id() const { return clear_area_info_.id; }
  const ImpassableAeraType type() const { return clear_area_info_.type; }
  const math::Polygon2d &polygon() const { return polygon_; }

 private:
  math::Polygon2d polygon_;
  std::vector<math::Vec2d> points_;
  ClearAreaInfo clear_area_info_;
};
typedef std::shared_ptr<ClearArea> ClearAreaPtr;
typedef std::shared_ptr<const ClearArea> ClearAreaConstPtr;
}  // namespace planning
}  // namespace ad_e2e
#endif
