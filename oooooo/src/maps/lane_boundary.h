#ifndef MAP_LANE_BOUNDARY_H
#define MAP_LANE_BOUNDARY_H

#include <memory>

#include "maps/lane_boundary.h"
#include "maps/map_def.h"
#include "math/line_curve2d.h"

namespace ad_e2e {
namespace planning {
class LaneBoundary {
 public:
  explicit LaneBoundary(const LaneBoundaryInfo& boundary_info);
  ~LaneBoundary() = default;

  void PushLeftLane(const std::string& lane_id) {
    left_lanes_.emplace_back(lane_id);
  }
  void PushRightLane(const std::string& lane_id) {
    right_lanes_.emplace_back(lane_id);
  }

  const std::string& id() const { return id_; };
  const std::vector<Point2d>& points() const { return points_; }
  const std::vector<Point2d>& curve_points() const {
    return line_curve_.points();
  }
  const math::LineCurve2d& line_curve() const { return line_curve_; }
  const LaneBoundaryType& type() const { return type_; }
  const double& curve_length() const { return curve_length_; }
  const std::vector<std::string>& left_lanes() const { return left_lanes_; };
  const std::vector<std::string>& right_lanes() const { return right_lanes_; };
  const double length() const { return length_; }
  bool IsValid() const {
    return points_.size() > 1u && line_curve_.points().size() > 1u;
  }

  void set_section_id(const std::string& section_id) {
    section_id_ = section_id;
  }
  const std::string& section_id() const { return section_id_; }

 private:
  void InterpolatePoints();

 private:
  std::string id_;
  std::vector<Point2d> points_;
  std::vector<std::string> left_lanes_;
  std::vector<std::string> right_lanes_;
  math::LineCurve2d line_curve_;
  LaneBoundaryType type_;
  double length_ = 0.0;
  double curve_length_ = 0.0;

  std::string section_id_ = "";
};

using LaneBoundaryPtr = std::shared_ptr<LaneBoundary>;
using LaneBoundaryConstPtr = std::shared_ptr<const LaneBoundary>;

}  // namespace planning
}  // namespace ad_e2e

#endif
