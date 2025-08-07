#ifndef MAP_ROAD_BOUNDARY_H
#define MAP_ROAD_BOUNDARY_H
#include <memory>

#include "maps/map_def.h"
#include "maps/road_boundary.h"
#include "math/line_curve2d.h"
namespace ad_e2e {
namespace planning {
class RoadBoundary {
 public:
  explicit RoadBoundary(const RoadBoundaryInfo& road_boundary_info);
  ~RoadBoundary() = default;

  const std::string& id() const { return id_; };
  const std::vector<Point2d>& points() const { return points_; }
  const std::vector<Point2d>& curve_points() const {
    return line_curve_.points();
  }
  const math::LineCurve2d& line_curve() const { return line_curve_; }
  const RoadBoundaryType& type() const { return type_; }
  const double& curve_length() const { return curve_length_; }
  const double length() const { return length_; }
  bool IsValid() const { return points_.size() > 1u; }

  void set_section_id(const std::string& section_id) {
    section_id_ = section_id;
  }
  const std::string& section_id() const { return section_id_; }

  static bool GetRoadBoundaryPoints(
      const std::vector<Point2d>& center_pts,
      const std::vector<math::LineSegment2d>& road_segs,
      const std::pair<double, double>& lat_dist_range,
      std::vector<Point2d>* road_pts);

  bool has_height() const { return false; }
  double height() const { return 0.0; }

 private:
  void InterpolatePoints();

 private:
  std::string id_;
  std::vector<Point2d> points_;
  math::LineCurve2d line_curve_;
  RoadBoundaryType type_;
  double length_ = 0.0;
  double curve_length_ = 0.0;

  std::string section_id_ = "";
};
using RoadBoundaryPtr = std::shared_ptr<RoadBoundary>;
using RoadBoundaryConstPtr = std::shared_ptr<const RoadBoundary>;
}  // namespace planning
}  // namespace ad_e2e
#endif
