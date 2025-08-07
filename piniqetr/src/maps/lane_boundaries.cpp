#include "maps/lane_boundaries.h"

#include "common/gflags.h"
namespace ad_e2e {
namespace planning {
LaneBoundaries::LaneBoundaries(
    const std::vector<LaneBoundaryConstPtr>& boundaries) {
  double length = 0;
  std::vector<Point2d> points;
  for (auto boundary_ptr : boundaries) {
    if (!boundary_ptr->IsValid()) {
      continue;
    }

    if (!points.empty()) {
      double dist = points.back().DistanceTo(boundary_ptr->points().front());
      if (dist > FLAGS_ad_e2e_planning_map_boundary_gap_threshold) {
        LaneBoundaryType boundary_type = boundary_ptr->type();
        length += dist;
        boundary_type.s = length;
        boundary_types_.emplace_back(std::move(boundary_type));

        LaneBoundaryInfo boundary_info;
        boundary_info.points.emplace_back(points.back());
        boundary_info.points.emplace_back(boundary_ptr->points().front());
        boundaries_.emplace_back(std::make_shared<LaneBoundary>(boundary_info));
      }
    }

    for (const auto& pt : boundary_ptr->points()) {
      if (points.empty()) {
        points.emplace_back(pt);
        continue;
      }
      double dist = points.back().DistanceTo(pt);
      if (dist < FLAGS_ad_e2e_planning_map_point_distance_threshold) {
        continue;
      }
      length += dist;
      points.emplace_back(pt);
    }
    LaneBoundaryType boundary_type = boundary_ptr->type();
    boundary_type.s = length;
    boundary_types_.emplace_back(std::move(boundary_type));
    boundaries_.emplace_back(boundary_ptr);
  }
  line_curve_.InitializePoints(points);
}

const int32_t LaneBoundaries::GetBoundarySegmentIndex(const double& s) const {
  int32_t index = -1;
  for (int32_t i = 0; i < static_cast<int32_t>(boundary_types_.size()); ++i) {
    if (boundary_types_.at(i).s > s) {
      index = i;
      break;
    }
  }
  return index;
}
}  // namespace planning
}  // namespace ad_e2e
