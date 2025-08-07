#ifndef PLANNER_SPEED_ST_GRAPH_DEFS_H_
#define PLANNER_SPEED_ST_GRAPH_DEFS_H_
#include <string>
#include <vector>

#include "math/geometry/polygon2d.h"
#include "speed_planning.pb.h"

namespace e2e_noa::planning {
struct StDistancePoint {
  double t = 0.0;
  double path_s = 0.0;
  double distance = 0.0;
  double relative_v = 0.0;
};

struct CloseSpaceTimeObject {
  std::vector<StDistancePoint> st_distance_points;
  bool is_stationary = false;
  bool is_away_from_traj = false;
  StBoundaryProto::ObjectType object_type;
  std::string id;
  Polygon2d contour;
};

struct DistanceInfo {
  double s = 0.0;
  double dist = 0.0;
  std::string id;
  std::string info;

  std::pair<double, double> dists;
  ad_e2e::planning::RoadBoundaryType type;
};
}  // namespace e2e_noa::planning

#endif
